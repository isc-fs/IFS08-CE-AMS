// SPDX-License-Identifier: proprietary
//
// Accumulator-bus task (FDCAN1). Two responsibilities:
//
//  RX (always): drain acu_rx_queue -> VehicleService::update_from_frame,
//               with bootloader-trigger sniffing in front.
//
//  TX (periodic) -- the ECU-side telemetry matrix shipped in fix/53.
//  The ECU's FDCAN2 peripheral is wired to AMS FDCAN1, so these frames
//  feed real-time telemetry through the ECU. See docs/CAN_MAP.md and
//  ams_config.hpp for byte layouts.
//
//    50 ms: 0x135 currents (i16 deciamps; accu, dcdc)
//   100 ms: 0x020 ok_precharge
//           0x12C v_cell_min (pack-wide min cell mV)
//           0x131/0x132 vmin_module  (5 modules over two frames)
//           0x133/0x134 vmax_module
//   250 ms: 0x136/0x137 temp_max_module (+ temp_dcdc stub on 0x137)
//
// 0x130 (SOC) deferred until firmware has an estimator.
// 0x450 retired -- 0x135 is the successor.
//
// Cadence is driven by osMessageQueueGet with a computed timeout that
// expires at the nearest TX deadline, so RX latency stays low and TX
// jitter stays bounded.

#include "app/acu_can_task.h"

#include "acu_tx_encoders.hpp"
#include "ams_config.hpp"
#include "bms_service.hpp"
#include "bootloader.hpp"
#include "can_frame.hpp"
#include "current_service.hpp"
#include "pit_diag_emitter.hpp"
#include "state_machine.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
}

extern "C" osMessageQueueId_t acu_rx_queueHandle;

// FSM state mirror maintained by safety_task.cpp; used here for
// ok_precharge (= 1 iff state in {Run=3, Charge=4}).
extern "C" volatile std::uint8_t g_state_telemetry;

// Pit-diag (#247) needs read-only visibility into a handful of locals
// owned by other TUs. SafetyTask writes g_mode_locked_telemetry on the
// same cadence it writes g_state_telemetry. BmsPollTask writes
// g_bms_volt_poll_ms / _max, and run_temperature_poll exposes the
// last sweep failure mask. Per-LTC PEC counters live in bms_service.cpp.
extern "C" volatile std::uint8_t  g_mode_locked_telemetry;
extern "C" volatile std::uint32_t g_bms_volt_poll_ms;
extern "C" volatile std::uint32_t g_bms_volt_poll_max;
extern "C" volatile std::uint32_t g_temp_sweep_last_mask;
extern "C" volatile std::uint32_t g_ltc_pec_err_count[ams::config::LtcChainLength];

// Balance state mirrors written by maybe_run_balance_update().
extern "C" volatile std::uint32_t g_balance_dcc_bits[ams::config::BmsModuleCount];
extern "C" volatile std::uint32_t g_balance_cycles_total_pub;
extern "C" volatile std::uint32_t g_balance_cycles_active_pub;

// Boot diagnostics. Surfaced in flight
// pit-diag can surface them too.
extern "C" volatile std::uint8_t  g_app_init_progress;
extern "C" volatile std::uint32_t g_fdcan1_start_result;

// FreeRTOS stack-overflow / malloc-failed hooks (freertos.c).
extern "C" volatile std::uint32_t g_stack_overflow_task_addr;
extern "C" volatile std::uint32_t g_stack_overflow_watermark;
extern "C" volatile std::uint32_t g_malloc_failed_count;

// Firmware ID accessors (firmware_info.cpp).
extern "C" std::uint8_t        ams_fw_version_major(void);
extern "C" std::uint8_t        ams_fw_version_minor(void);
extern "C" std::uint8_t        ams_fw_version_patch(void);
extern "C" std::uint8_t        ams_bl_node_id(void);
extern "C" const std::uint8_t* ams_git_hash(void);

namespace {

// Telemetry counters; volatile so a remote-debug session can read.
volatile std::uint32_t g_acu_rx_dropped_unknown = 0;
volatile std::uint32_t g_acu_tx_fail            = 0;

// Pit-diag runtime flag (#247). Toggled by RX dispatch on the
// PitDiagCmdRxId frame; consumed by the TX scheduler below. Lives in
// .bss so power-up always starts with diag OFF -- the engineer must
// explicitly enable it via cansend after every reboot.
bool s_pit_diag_enabled = false;

bool send_acu(std::uint32_t id, std::uint8_t dlc,
              const std::uint8_t *data) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    // STM32H7 HAL takes DataLength as the raw byte count (FDCAN_DLC_BYTES_N
     // macros are unshifted; HAL applies the bit-16 register shift internally).
     // The pre-shift this site used to do produced DLC=0 on the wire for every
     // ECU TX matrix frame -- see #234. Note: STM32G4's HAL pre-shifts the
     // macros, which is why this regression was easy to port-import.
     tx.DataLength          = dlc;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx,
               const_cast<std::uint8_t *>(data)) == HAL_OK;
}

// Each tx_* wrapper builds the frame via the host-testable encoder in
// acu_tx_encoders.hpp, then ships it through send_acu. Keeping the
// encoders pure-logic in a header lets the unit-test build verify the
// byte layout without HAL.

template <std::size_t N>
inline void send_or_fail(std::uint32_t id,
                         const std::array<std::uint8_t, N>& payload) noexcept {
    if (!send_acu(id, static_cast<std::uint8_t>(N), payload.data())) {
        ++g_acu_tx_fail;
    }
}

// Diag-stream variant of send_or_fail (#257). The pit-diag burst pushes
// ~57 frames into a 16-deep TX FIFO inside a single task iteration. Without
// flow control, frames 17+ NACK silently and the engineer sees only the
// front 16 IDs on the wire. Yield-while-full keeps the burst end-to-end
// at the cost of ~6 ms of task time per scan (16 frames × ~110 us at
// 500 kbps + osDelay rounding). At 1 Hz scan cadence that's 0.6 % of the
// task budget -- the rest of the AcuCanTask loop still gets all its
// 50/100/250 ms deadlines.
//
// Only the pit-diag burst uses the blocking variant. The fast-path TX
// matrix (currents at 50 ms, ok_precharge / per-module v at 100 ms,
// temps at 250 ms) stays non-blocking -- a transient FIFO-full bump on
// the ECU TX matrix bumps g_acu_tx_fail rather than stalling the cadence.
template <std::size_t N>
inline void send_or_fail_blocking(std::uint32_t id,
                                  const std::array<std::uint8_t, N>& payload) noexcept {
    // Worst-case wait: 16 frames × ~110 us = ~1.8 ms. osDelay(1) gives
    // 1 ms granularity on the 1 kHz tick, which is the smallest yield
    // FreeRTOS offers without busy-spinning. Lower-priority tasks (e.g.
    // BmsPollTask at Normal) get to run during the wait.
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u) {
        osDelay(1);
    }
    if (!send_acu(id, static_cast<std::uint8_t>(N), payload.data())) {
        ++g_acu_tx_fail;
    }
}


void tx_ok_precharge() noexcept {
    send_or_fail(ams::config::AcuTxOkPrechargeId,
                 ams::acu_tx::encode_ok_precharge(g_state_telemetry));
}
void tx_min_voltage(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxMinVoltId,        ams::acu_tx::encode_min_voltage(b));
}
void tx_vmin_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxVminModuleAId,    ams::acu_tx::encode_vmin_module_a(b));
}
void tx_vmin_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxVminModuleBId,    ams::acu_tx::encode_vmin_module_b(b));
}
void tx_vmax_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxVmaxModuleAId,    ams::acu_tx::encode_vmax_module_a(b));
}
void tx_vmax_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxVmaxModuleBId,    ams::acu_tx::encode_vmax_module_b(b));
}
void tx_currents(const ams::CurrentState& c) noexcept {
    send_or_fail(ams::config::AcuTxCurrentsId,       ams::acu_tx::encode_currents(c));
}
void tx_temp_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxTempMaxModuleAId, ams::acu_tx::encode_tmax_module_a(b));
}
void tx_temp_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::AcuTxTempMaxModuleBId, ams::acu_tx::encode_tmax_module_b(b));
}

// ---- Pit-diag stream (#247) -----------------------------------------------

void tx_pit_diag_ack(bool enabled) noexcept {
    const std::uint8_t payload = enabled ? 0x01u : 0x00u;
    if (!send_acu(ams::config::PitDiagAckTxId, 1u, &payload)) {
        ++g_acu_tx_fail;
    }
}

std::uint32_t pec_err_sum() noexcept {
    std::uint32_t total = 0;
    for (std::uint8_t i = 0; i < ams::config::LtcChainLength; ++i) {
        total += g_ltc_pec_err_count[i];
    }
    return total;
}

void tx_pit_diag_scan(const ams::BmsState& bms) noexcept {
    // 24 cell + 25 temp + 7 status = 56 frames per scan.
    // FDCAN1 TX FIFO depth is 16 (main.c TxFifoQueueElmtsNbr). Without
    // flow control, frames 17+ get NACKed silently and only the front
    // of the burst reaches the wire (#257). Use the blocking variant
    // throughout so the entire scan lands; yield-while-full keeps
    // BmsPollTask + the 50/100/250 ms ECU TX matrix scheduled around
    // us. Worst-case scan duration: ~6 ms at 500 kbps -- still well
    // under the 1 s PitDiagScanPeriodMs.
    for (std::uint8_t i = 0; i < ams::config::PitDiagCellFrames; ++i) {
        send_or_fail_blocking(ams::config::PitDiagCellBaseId + i,
                              ams::pit_diag::encode_cell_frame(bms, i));
    }
    for (std::uint8_t i = 0; i < ams::config::PitDiagTempFrames; ++i) {
        send_or_fail_blocking(ams::config::PitDiagTempBaseId + i,
                              ams::pit_diag::encode_temp_frame(bms, i));
    }

    const bool tsms     = HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)       == GPIO_PIN_SET;
    const bool dash_chg = HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET;
    const std::uint8_t ams_ok =
        (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    send_or_fail_blocking(ams::config::PitDiagFsmStatusId,
                          ams::pit_diag::encode_fsm_status(
                              g_state_telemetry,
                              static_cast<ams::fsm::Mode>(g_mode_locked_telemetry),
                              tsms, dash_chg, ams_ok,
                              pec_err_sum()));
    send_or_fail_blocking(ams::config::PitDiagTimingId,
                          ams::pit_diag::encode_timing(
                              g_bms_volt_poll_ms,
                              g_bms_volt_poll_max,
                              g_temp_sweep_last_mask));

    // Encoders take the volatile array by reference so we read the
    // live values each scan -- no thread-locked snapshot needed.
    send_or_fail_blocking(ams::config::PitDiagBalanceMaskAId,
                          ams::pit_diag::encode_balance_mask_a(g_balance_dcc_bits));
    send_or_fail_blocking(ams::config::PitDiagBalanceMaskBId,
                          ams::pit_diag::encode_balance_mask_b(
                              g_balance_dcc_bits,
                              g_balance_cycles_total_pub,
                              g_balance_cycles_active_pub));

    // Read JumpReason once per scan -- it's stable across the boot.
    const std::uint32_t jump_reason =
        (&RTC->BKP0R)[ams::config::BkpJumpReasonReg];
    send_or_fail_blocking(ams::config::PitDiagBootDiagId,
                          ams::pit_diag::encode_boot_diag(
                              jump_reason, g_app_init_progress, g_fdcan1_start_result));

    send_or_fail_blocking(ams::config::PitDiagPostMortemId,
                          ams::pit_diag::encode_post_mortem(
                              g_stack_overflow_task_addr,
                              g_stack_overflow_watermark,
                              g_malloc_failed_count));

    send_or_fail_blocking(ams::config::PitDiagFwIdId,
                          ams::pit_diag::encode_fw_id(
                              ams_fw_version_major(),
                              ams_fw_version_minor(),
                              ams_fw_version_patch(),
                              ams_git_hash(),
                              ams_bl_node_id()));

    // Per-IC PEC counts (#258). Adds 2 frames to the scan, brings the
    // total to 58. Localises a chain failure to a specific IC --
    // 0x6C0[4..5] sum tells "chain unhealthy", these tell "which one".
    send_or_fail_blocking(ams::config::PitDiagPecPerIcAId,
                          ams::pit_diag::encode_pec_err_count_a(g_ltc_pec_err_count));
    send_or_fail_blocking(ams::config::PitDiagPecPerIcBId,
                          ams::pit_diag::encode_pec_err_count_b(g_ltc_pec_err_count));
}

}  // namespace

extern "C" void ams_acu_can_task_run(void *argument) {
    (void)argument;

    ams::CanFrame frame;
    const auto    now0          = osKernelGetTickCount();
    std::uint32_t last_fast_tx  = now0;
    std::uint32_t last_mid_tx   = now0;
    std::uint32_t last_slow_tx  = now0;
    std::uint32_t last_pit_scan = now0;

    for (;;) {
        const auto now           = osKernelGetTickCount();
        const auto next_fast     = last_fast_tx + ams::config::EcuFastTxMs;
        const auto next_mid      = last_mid_tx  + ams::config::EcuMidTxMs;
        const auto next_slow     = last_slow_tx + ams::config::EcuSlowTxMs;
        // The pit-diag deadline only counts when the stream is enabled.
        // Otherwise wedge it at "far future" so it doesn't shorten the
        // queue-get timeout in the off path.
        const auto next_pit      = s_pit_diag_enabled
                                       ? (last_pit_scan + ams::config::PitDiagScanPeriodMs)
                                       : (now + 0xFFFFu);
        const auto deadline      = std::min({ next_fast, next_mid, next_slow, next_pit });
        const auto timeout       = (deadline > now) ? deadline - now : 0u;

        if (osMessageQueueGet(acu_rx_queueHandle, &frame, nullptr, timeout) == osOK) {
            // Pit-diag command first -- it's a fast classify and shouldn't
            // be confused with a VCU frame on dispatch failure.
            const int diag_cmd = ams::pit_diag::classify_command(frame);
            if (diag_cmd != 0) {
                const bool desired = (diag_cmd > 0);
                if (s_pit_diag_enabled != desired) {
                    s_pit_diag_enabled = desired;
                    // Reset the cadence so the engineer sees a scan within
                    // PitDiagScanPeriodMs of the enable command, not whatever
                    // residual delay was on the clock.
                    last_pit_scan = osKernelGetTickCount() -
                                    ams::config::PitDiagScanPeriodMs;
                }
                tx_pit_diag_ack(s_pit_diag_enabled);
            } else {
                // Boot-trigger frame moved to FDCAN1 in v1.2.0 (#73). Has
                // to come BEFORE VehicleService so we still react to the
                // reboot request even if the trigger ID accidentally
                // collides with a future VCU frame. request_reboot() never
                // returns -- it opens all relays, writes
                // BL_BOOT_REQ_MAGIC into BKP0R, and resets.
                if (ams::Bootloader::matches_trigger(frame)) {
                    ams::Bootloader::request_reboot(
                        ams::config::JumpReason::CanTrigger);
                }
                if (!ams::VehicleService::instance().update_from_frame(frame)) {
                    ++g_acu_rx_dropped_unknown;
                }
            }
        }

        // ---- Snapshot service state once per loop iteration ----
        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto now2 = osKernelGetTickCount();

        // ---- TX scheduler ----
        if (now2 - last_fast_tx >= ams::config::EcuFastTxMs) {
            tx_currents(cur);
            last_fast_tx = now2;
        }
        if (now2 - last_mid_tx >= ams::config::EcuMidTxMs) {
            tx_ok_precharge();
            tx_min_voltage(bms);
            tx_vmin_module_a(bms);
            tx_vmin_module_b(bms);
            tx_vmax_module_a(bms);
            tx_vmax_module_b(bms);
            last_mid_tx = now2;
        }
        if (now2 - last_slow_tx >= ams::config::EcuSlowTxMs) {
            tx_temp_module_a(bms);
            tx_temp_module_b(bms);
            last_slow_tx = now2;
        }
        if (s_pit_diag_enabled &&
            (now2 - last_pit_scan >= ams::config::PitDiagScanPeriodMs)) {
            tx_pit_diag_scan(bms);
            last_pit_scan = now2;
        }
    }
}
