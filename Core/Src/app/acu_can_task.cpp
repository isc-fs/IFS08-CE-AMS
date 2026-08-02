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
#include "can_busoff_recovery.hpp"
#include "can_frame.hpp"
#include "current_service.hpp"
#include "diag_proto.hpp"
#include "isotp.hpp"
#include "app/sd_logger_task.h"
#include "pit_diag_emitter.hpp"
#include "fw_health.hpp"
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
// Latched-fault diagnostics from SafetyTask, surfaced on 0x6C0[6]/[7]
// (#276). reason matches ams::safety::FaultReason; detail is the
// per-branch detail byte (module index / mask).
extern "C" volatile std::uint8_t  g_fault_reason_telemetry;
// Pack SoC %, or ams::soc::Unknown (0xFF). Written by CurrentSensorTask.
extern "C" volatile std::uint8_t  g_soc_percent;
extern "C" volatile std::uint8_t  g_fault_detail_telemetry;
extern "C" volatile std::uint32_t g_bms_volt_poll_ms;
extern "C" volatile std::uint32_t g_bms_volt_poll_max;
extern "C" volatile std::uint32_t g_temp_sweep_last_mask;
extern "C" volatile std::uint32_t g_ltc_pec_err_count[ams::config::LtcChainLength];

// Balance state mirrors written by maybe_run_balance_update().
extern "C" volatile std::uint32_t g_balance_dcc_bits[ams::config::BmsModuleCount];
// Raw ADOW diagnostic grids (config::AdowRawDiag only), written by BmsPollTask.
extern "C" std::uint16_t g_adow_diag_pu[ams::config::BmsModuleCount *
                                        ams::config::CellsPerModule];
extern "C" std::uint16_t g_adow_diag_pd[ams::config::BmsModuleCount *
                                        ams::config::CellsPerModule];
extern "C" volatile std::uint32_t g_balance_cycles_total_pub;
extern "C" volatile std::uint32_t g_balance_cycles_active_pub;

// Boot diagnostics. Surfaced on the flight pit-diag boot-diag frame
// (0x6C4) so app-init progress + FDCAN1 start outcome are visible on
// can0 from any build.
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

// FDCAN1 Bus-Off recovery counter: incremented once per Stop/Start
// attempt issued by poll_fdcan1_busoff_recovery(). Surfaced on the
// pit-diag comms-health frame (0x6C9, bytes 0..3) so the CAN-only HIL
// bench can confirm a recovery fired without a debugger -- the AMS
// analogue of the bootloader's bl_health fdcan_recovery_count.
volatile std::uint32_t g_fdcan1_busoff_recovery_count = 0;

// ---- FDCAN1 Bus-Off poll + recovery -------------------------------------
//
// The STM32H7 M_CAN latches Bus_Off after sustained TX errors (the
// classic-CAN TEC > 255 path: transmitting into a bus with no node
// ACKing, or a transient short). Bus_Off sets CCCR.INIT, which halts
// BOTH TX and RX -- the AMS stops ACKing, goes silent, and does NOT
// self-clear; only a software Stop->Start re-arms the peripheral. Today
// the AMS survives on the bench/car only because the bus always has an
// ACKing node so it never sustains Bus_Off -- this is the latent
// robustness fix (mirrors IFS08-CE-ECU and the bootloader).
//
// Mirrors the bootloader's Bootloader_FdcanBusOffRecover
// (../stm32-can-bootloader, #125 C1 / #174 NG-9). GetProtocolStatus is a
// cheap PSR read, safe every loop pass; the Stop/Start only runs on an
// actual fault and is rate-limited by ams::can_recovery (see the header).
// Stop/Start touches neither the message-RAM RX filter nor the FDCAN_IE
// interrupt enables, so the boot-time ConfigGlobalFilter +
// ActivateNotification survive and interrupt-driven RX resumes
// automatically after a rejoin -- no reconfigure needed.
void poll_fdcan1_busoff_recovery(ams::can_recovery::BusOffState& st,
                                 std::uint32_t now) noexcept {
    FDCAN_ProtocolStatusTypeDef ps = {};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) != HAL_OK) {
        return;
    }
    if (!ams::can_recovery::should_attempt_recovery(
            st, ps.BusOff != 0u, now, ams::config::FdcanBusOffRetryMs)) {
        return;
    }

    // Stop -> Start clears CCCR.INIT and arms the M_CAN's automatic
    // recovery. Stop puts the peripheral back to READY (INIT stays set);
    // Start clears INIT and the node rejoins after 128*11 recessive bits.
    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        // CRUCIAL (#174 NG-9): a Stop/Start timeout latches
        // hfdcan1.State = HAL_FDCAN_STATE_ERROR, after which EVERY later
        // HAL_FDCAN_Stop/Start silently no-ops (they gate on State) --
        // the retry would spin forever and the bus would wedge
        // permanently deaf. Force the HAL back to READY (clearing the
        // latched error) so the next poll's Start genuinely re-attempts
        // the rejoin instead of no-opping.
        hfdcan1.State     = HAL_FDCAN_STATE_READY;
        hfdcan1.ErrorCode = HAL_FDCAN_ERROR_NONE;
    }

    ++g_fdcan1_busoff_recovery_count;
}

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
// 58 frames into a 16-deep TX FIFO inside a single task iteration. Without
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
// 0x130 SoC %, or 0xFF when there is no trustworthy estimate. Read straight
// from the byte CurrentSensorTask publishes -- TELEMETRY ONLY, nothing in the
// safety path is involved.
void tx_soc() noexcept {
    send_or_fail(ams::config::AcuTxSocId, ams::acu_tx::encode_soc(g_soc_percent));
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
    // 24 cell + 25 temp + 10 status = 59 frames per scan.
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

    const auto veh_snap = ams::VehicleService::instance().snapshot();
    // Pit-diag "balance_override" bit = balancing currently held OFF by the
    // operator master switch (Off, or the dead-man fallback). On / Auto -> 0.
    const bool balance_override =
        ams::VehicleService::effective_balance_cmd(
            osKernelGetTickCount(), veh_snap.last_balance_override_tick,
            veh_snap.balance_cmd) == ams::config::BalanceCmd::Off;
    send_or_fail_blocking(ams::config::PitDiagFsmStatusId,
                          ams::pit_diag::encode_fsm_status(
                              g_state_telemetry,
                              static_cast<ams::fsm::Mode>(g_mode_locked_telemetry),
                              tsms, dash_chg, ams_ok,
                              pec_err_sum(),
                              g_fault_reason_telemetry,
                              g_fault_detail_telemetry,
                              balance_override));
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

    // FDCAN1 comms health (#331). The 59th frame: Bus-Off recovery count +
    // ECU-TX enqueue failures. Lets the CAN-only HIL bench confirm a
    // Bus-Off recovery fired (count > 0 after an outage) without a debugger.
    send_or_fail_blocking(ams::config::PitDiagCommsHealthId,
                          ams::pit_diag::encode_comms_health(
                              g_fdcan1_busoff_recovery_count, g_acu_tx_fail));

    // BENCH DIAGNOSTIC (config::AdowRawDiag): raw ADOW PU + PD per-cell dump,
    // same 24-frame layout as the 0x680 cell grid, on AdowDiagPuBaseId /
    // AdowDiagPdBaseId. Dead-code-eliminated on flight builds (flag false).
    if (ams::config::AdowRawDiag) {
        for (std::uint8_t i = 0; i < ams::config::PitDiagCellFrames; ++i) {
            send_or_fail_blocking(ams::config::AdowDiagPuBaseId + i,
                                  ams::pit_diag::encode_adow_grid_frame(g_adow_diag_pu, i));
            send_or_fail_blocking(ams::config::AdowDiagPdBaseId + i,
                                  ams::pit_diag::encode_adow_grid_frame(g_adow_diag_pd, i));
        }
    }
}

// Ungated firmware-health frame (#411). 1 Hz, emitted REGARDLESS of the
// pit-diag arm, so a passive `pit-diag listen` sees AMS liveness (heap, task
// liveness, reset cause, uptime, last fault -- ECU 0x704 parity) with the
// stream off. Non-blocking send: a full TX FIFO just bumps g_acu_tx_fail
// (itself surfaced on 0x6C9) rather than stalling the task.
void tx_fw_health() noexcept {
    send_or_fail(ams::config::FwHealthId,
                 ams::pit_diag::encode_fw_health(
                     ams::fw_health::free_heap(),
                     ams::fw_health::min_free_heap(),
                     ams::fw_health::sample_liveness(),
                     ams::fw_health::reset_cause(),
                     ams::fw_health::uptime_s(),
                     ams::fw_health::last_fault()));
}

// ---------------------------------------------------------------------------
// LOGFS diag transport (#406 / #439).
//
// Bootloader addressing: host -> node on 0x000 + NodeID, node -> host on
// 0x010 + NodeID. Nothing needs a new hardware filter -- the global filter
// already accepts every standard frame into FIFO0.
//
// Note the request ID equals the boot-trigger ID (0x002) once the AMS moves to
// node 2 (#403). That is safe and stays safe: matches_trigger demands DLC 4
// AND the exact B0 07 AD 11 payload, while an ISO-TP frame is always DLC 8 --
// and 0xB0 could never be a valid ISO-TP PCI byte anyway (only 0x0-0x3 are).
// The trigger check runs first, so a LOGFS frame passes through it untouched.
// ---------------------------------------------------------------------------
constexpr std::uint32_t DiagRxId = 0x000u + ams::config::AmsNodeId;
constexpr std::uint32_t DiagTxId = 0x010u + ams::config::AmsNodeId;

ams::isotp::Reassembler s_diag_rx;
ams::isotp::Segmenter   s_diag_tx;
bool                    s_diag_tx_active = false;
std::uint8_t            s_diag_msg[ams::isotp::MaxMsg];

// Push as much of the outbound message as the TX FIFO will take right now.
//
// Deliberately NOT a blocking drain: a 512-byte reply is ~76 frames against a
// 3-deep FIFO, i.e. ~19 ms of bus time. Sitting on that would stall the 10 ms
// telemetry cadence this task also owns. Instead it dribbles out whatever fits
// each pass, and the loop below shortens its wait while a transfer is live.
void pump_diag_tx() noexcept {
    if (!s_diag_tx_active) return;
    std::uint8_t f[ams::isotp::FrameLen];
    // Leave DiagTxReservedSlots free so the flight telemetry matrix, which is
    // scheduled AFTER this in the same loop pass and ships non-blocking, always
    // finds room. Filling to zero silently blacked out telemetry for the whole
    // multi-minute pull (#449).
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >
           ams::config::DiagTxReservedSlots) {
        if (!s_diag_tx.next(f)) { s_diag_tx_active = false; return; }
        if (!send_acu(DiagTxId, ams::isotp::FrameLen, f)) {
            // Lost the race for the slot; the frame is already consumed from
            // the segmenter, so the message is broken -- abandon it and let
            // the host time out and retry rather than send a corrupt stream.
            ++g_acu_tx_fail;
            s_diag_tx_active = false;
            return;
        }
    }
}

// Feed one received diag frame into the reassembler; submit a completed
// message to SdLoggerTask, which owns the filesystem.
void handle_diag_frame(const ams::CanFrame& frame, std::uint32_t now) noexcept {
    const auto st = s_diag_rx.feed(frame.data, frame.dlc, 0u, now);

    if (s_diag_rx.fc_pending()) {
        std::uint8_t fc[ams::isotp::FrameLen];
        ams::isotp::build_fc_cts(fc);
        (void)send_acu(DiagTxId, ams::isotp::FrameLen, fc);
        s_diag_rx.clear_fc_pending();
    }
    if (st != ams::isotp::RxStatus::MsgComplete) return;

    if (!ams::sd_diag_submit(s_diag_rx.data(), s_diag_rx.size())) {
        // Already serving one, or the logger has not started. Answer BUSY so
        // the host retries instead of waiting out its timeout.
        ams::diag::Request req;
        std::uint8_t       nack[8];
        if (ams::diag::parse_request(s_diag_rx.data(), s_diag_rx.size(), req)) {
            const std::uint16_t n = ams::diag::build_nack(nack, sizeof nack, req.opcode,
                                                          ams::diag::NackBusy);
            if (n != 0 && s_diag_tx.begin(nack, n)) s_diag_tx_active = true;
        }
    }
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
    std::uint32_t last_fw_health_tx = now0;   // ungated 1 Hz health (#411)

    // FDCAN1 Bus-Off recovery latch (single bus, single owner: this task).
    ams::can_recovery::BusOffState busoff_state{};

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
        // While a diag reply is going out, come back every tick to top up the
        // TX FIFO. Otherwise a ~76-frame message would trickle at the slowest
        // telemetry cadence and take seconds instead of tens of milliseconds.
        const auto next_diag     = s_diag_tx_active ? (now + 1u) : (now + 0xFFFFu);
        const auto deadline      = std::min({ next_fast, next_mid, next_slow, next_pit,
                                              next_diag });
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
                // LOGFS (#406) rides the same ID the trigger uses once the AMS
                // is node 2, which is why this must come AFTER the check above.
                if (frame.id == DiagRxId &&
                    frame.bus == static_cast<std::uint8_t>(ams::CanBus::Acu)) {
                    handle_diag_frame(frame, now);
                } else if (!ams::VehicleService::instance().update_from_frame(frame)) {
                    ++g_acu_rx_dropped_unknown;
                }
            }
        }

        // Diag: collect a finished reply from SdLoggerTask and start shipping
        // it, then top up the TX FIFO. Both are non-blocking, so the card can
        // never stall this task.
        if (!s_diag_tx_active) {
            const std::uint16_t n = ams::sd_diag_collect(s_diag_msg, sizeof s_diag_msg);
            if (n != 0 && s_diag_tx.begin(s_diag_msg, n)) s_diag_tx_active = true;
        }
        pump_diag_tx();

        // AcuCanTask serviced its RX queue this pass -> CAN-RX liveness (#411).
        ams::fw_health::poke(ams::fw_health::CanRx);

        // ---- Snapshot service state once per loop iteration ----
        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto now2 = osKernelGetTickCount();

        // ---- FDCAN1 Bus-Off poll + recovery (every loop pass) ----
        // Runs unconditionally so it still fires while Bus_Off has the
        // node deaf (no RX frames arrive, so the queue-get above just
        // times out at the next TX deadline -- the loop still spins at
        // <= EcuFastTxMs). The recovery itself is rate-limited internally.
        poll_fdcan1_busoff_recovery(busoff_state, now2);

        // ---- TX scheduler ----
        if (now2 - last_fast_tx >= ams::config::EcuFastTxMs) {
            tx_currents(cur);
            ams::fw_health::poke(ams::fw_health::CanTx);   // TX path alive (#411)
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
            tx_soc();
            last_slow_tx = now2;
        }
        if (s_pit_diag_enabled &&
            (now2 - last_pit_scan >= ams::config::PitDiagScanPeriodMs)) {
            tx_pit_diag_scan(bms);
            last_pit_scan = now2;
        }

        // ---- Ungated firmware-health (#411): 1 Hz, no arm gate ----
        if (now2 - last_fw_health_tx >= ams::config::FwHealthPeriodMs) {
            tx_fw_health();
            last_fw_health_tx = now2;
        }
    }
}
