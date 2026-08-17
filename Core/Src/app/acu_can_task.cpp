// SPDX-License-Identifier: proprietary
//
// Accumulator-bus task (FDCAN1). Two jobs:
//
//  RX (always): drain acu_rx_queue -> VehicleService::update_from_frame,
//               sniffing for the bootloader trigger first.
//
//  TX (periodic): the ECU telemetry matrix. The ECU's FDCAN2 is wired to
//  AMS FDCAN1, so these frames reach real-time telemetry through the ECU.
//  Byte layouts: docs/CAN_MAP.md and ams_config.hpp.
//
//    50 ms: 0x135 currents (i16 deciamps; accu, dcdc)
//   100 ms: 0x020 ok_precharge
//           0x021 discharge_interlock (fsm_in_start + tsms, for the ECU)
//           0x12C v_cell_min (pack-wide min cell mV)
//           0x131/0x132 vmin_module  (5 modules over two frames)
//           0x133/0x134 vmax_module
//   250 ms: 0x136/0x137 temp_max_module (+ temp_dcdc stub on 0x137)
//           0x130 soc_percent (0xFF = no trustworthy estimate)
//
// 0x450 is retired; 0x135 is its successor.
//
// osMessageQueueGet waits with a timeout computed to expire at the nearest
// TX deadline, so RX latency stays low and TX jitter bounded.

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
// TSMS (PF9) level from SafetyTask: 1 = shutdown circuit complete. Published on
// 0x021 for the ECU's discharge decision -- see acu_discharge_interlock.def.
extern "C" volatile std::uint8_t g_tsms_telemetry;

// Read-only mirrors owned by other TUs, published here for pit-diag.
// SafetyTask writes g_mode_locked_telemetry on the same cadence as
// g_state_telemetry; BmsPollTask writes the poll timings and the last temp
// sweep failure mask; per-LTC PEC counters live in bms_service.cpp.
extern "C" volatile std::uint8_t  g_mode_locked_telemetry;
// Latched-fault diagnostics from SafetyTask, surfaced on 0x6C0[6]/[7].
// reason matches ams::safety::FaultReason; detail is the per-branch detail
// byte (module index / mask).
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
// Second-mode cell grid (config::AdcModeCrossCheck only), written by BmsPollTask.
extern "C" std::uint16_t g_adc_xcheck_mv[ams::config::BmsModuleCount *
                                         ams::config::CellsPerModule];
extern "C" volatile std::uint32_t g_balance_cycles_total_pub;
extern "C" volatile std::uint32_t g_balance_cycles_active_pub;
// Balance-quiesce health: DCC clears that succeeded before a measurement, vs
// polls that measured under bleed because both WRCFGA attempts failed. The
// RATIO is the diagnostic -- see pit_balance_health.def.
extern "C" volatile std::uint32_t g_balance_quiesce_count;
extern "C" volatile std::uint32_t g_balance_quiesce_fail_count;

// Boot diagnostics, surfaced on the boot-diag frame (0x6C4): app-init
// progress and FDCAN1 start outcome, visible on can0 from any build.
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
// Reboot triggers (0x002) refused because the FSM was in an energised state.
// Nonzero means someone tried to flash a live car. TELEMETRY ONLY -- no safety
// predicate reads it; the refusal itself is the safety action.
volatile std::uint32_t g_boot_trigger_refused    = 0;

// One count per Stop/Start attempt made by poll_fdcan1_busoff_recovery().
// Surfaced on the comms-health frame (0x6C9 bytes 0..3) so the CAN-only HIL
// bench can confirm a recovery fired without a debugger -- the AMS analogue of
// the bootloader's bl_health fdcan_recovery_count.
volatile std::uint32_t g_fdcan1_busoff_recovery_count = 0;

// ---- FDCAN1 Bus-Off poll + recovery -------------------------------------
//
// The STM32H7 M_CAN latches Bus_Off after sustained TX errors (classic-CAN
// TEC > 255: transmitting into a bus with no node ACKing, or a transient
// short). Bus_Off sets CCCR.INIT, halting BOTH TX and RX -- the AMS stops
// ACKing, goes silent, and does NOT self-clear. Only a software Stop->Start
// re-arms the peripheral.
//
// UNEXERCISED: on the bench and the car some other node always ACKs, so
// Bus_Off has never actually been sustained and this path has never run for
// real. Mirrors the bootloader's Bootloader_FdcanBusOffRecover
// (../stm32-can-bootloader) and IFS08-CE-ECU.
//
// GetProtocolStatus is a cheap PSR read, safe every loop pass; the Stop/Start
// runs only on an actual fault and is rate-limited by ams::can_recovery (see
// the header). Stop/Start touches neither the message-RAM RX filter nor the
// FDCAN_IE enables, so the boot-time ConfigGlobalFilter +
// ActivateNotification survive and interrupt-driven RX resumes on rejoin.
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

    // Stop puts the peripheral back to READY (INIT stays set); Start clears
    // INIT and the node rejoins after 128*11 recessive bits.
    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        // CRUCIAL: a Stop/Start timeout latches hfdcan1.State =
        // HAL_FDCAN_STATE_ERROR, and every later HAL_FDCAN_Stop/Start gates
        // on State and silently no-ops -- the retry would spin forever with
        // the node permanently deaf. Force the HAL back to READY so the next
        // poll's Start really re-attempts the rejoin.
        hfdcan1.State     = HAL_FDCAN_STATE_READY;
        hfdcan1.ErrorCode = HAL_FDCAN_ERROR_NONE;
    }

    ++g_fdcan1_busoff_recovery_count;
}

// Pit-diag runtime flag. Toggled by RX dispatch on the PitDiagCmdRxId frame,
// consumed by the TX scheduler below. In .bss, so every power-up starts with
// diag OFF and the engineer must re-enable it by cansend after each reboot.
bool s_pit_diag_enabled = false;

bool send_acu(std::uint32_t id, std::uint8_t dlc,
              const std::uint8_t *data) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    // DataLength is the raw byte count. The STM32H7 FDCAN_DLC_BYTES_N macros
    // are unshifted and the HAL applies the bit-16 register shift itself, so
    // pre-shifting here would put DLC=0 on the wire. STM32G4's HAL does
    // pre-shift -- code ported from it must have that shift stripped.
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

// Each tx_* wrapper builds its frame with the pure-logic encoder in
// acu_tx_encoders.hpp, then ships it through send_acu. The encoders live in
// a header so the host test build can check byte layouts without HAL.

template <std::size_t N>
inline void send_or_fail(std::uint32_t id,
                         const std::array<std::uint8_t, N>& payload) noexcept {
    if (!send_acu(id, static_cast<std::uint8_t>(N), payload.data())) {
        ++g_acu_tx_fail;
    }
}

// Blocking send, used by the pit-diag burst only: one scan pushes 60 frames
// into a 16-deep TX FIFO (main.c TxFifoQueueElmtsNbr) in a single task iteration, and without flow control
// frames 17+ NACK silently, so only the front of the burst reaches the wire.
// Yield-while-full lands the whole scan for ~6 ms of task time (60 frames x
// ~110 us at 500 kbps + osDelay rounding) -- 0.6 % of the budget at the 1 Hz
// scan cadence.
//
// The ECU TX matrix (50/100/250 ms) stays non-blocking: a transient FIFO-full
// there bumps g_acu_tx_fail rather than stalling the cadence.
template <std::size_t N>
inline void send_or_fail_blocking(std::uint32_t id,
                                  const std::array<std::uint8_t, N>& payload) noexcept {
    // Worst-case wait: 16 frames x ~110 us = ~1.8 ms. osDelay(1) is the
    // smallest yield FreeRTOS offers on the 1 kHz tick without busy-spinning;
    // lower-priority tasks (BmsPollTask at Normal) run during it.
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
void tx_discharge_interlock() noexcept {
    send_or_fail(ams::config::AcuTxDischargeInterlockId,
                 ams::acu_tx::encode_discharge_interlock(
                     g_state_telemetry, g_tsms_telemetry != 0u));
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
// from the byte CurrentSensorTask publishes. TELEMETRY ONLY -- no safety
// predicate reads SoC.
void tx_soc() noexcept {
    send_or_fail(ams::config::AcuTxSocId, ams::acu_tx::encode_soc(g_soc_percent));
}

// ---- Pit-diag stream -----------------------------------------------

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
    // 24 cell + 25 temp + 11 status = 60 frames, ~6 ms at 500 kbps -- well
    // under the 1 s PitDiagScanPeriodMs. Blocking sends throughout; see
    // send_or_fail_blocking for why.
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
    // The "balance_override" bit = balancing held OFF by the operator master
    // switch (Off, or the dead-man fallback). On / Auto -> 0.
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
                              balance_override,
                              g_boot_trigger_refused != 0u));
    send_or_fail_blocking(ams::config::PitDiagTimingId,
                          ams::pit_diag::encode_timing(
                              g_bms_volt_poll_ms,
                              g_bms_volt_poll_max,
                              g_temp_sweep_last_mask));

    // The encoders take the volatile array by reference, so each scan reads
    // live values -- no snapshot needed.
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

    // Per-IC PEC counts. 0x6C0[4..5] carries the sum ("chain unhealthy");
    // these two frames say which IC.
    send_or_fail_blocking(ams::config::PitDiagPecPerIcAId,
                          ams::pit_diag::encode_pec_err_count_a(g_ltc_pec_err_count));
    send_or_fail_blocking(ams::config::PitDiagPecPerIcBId,
                          ams::pit_diag::encode_pec_err_count_b(g_ltc_pec_err_count));

    // FDCAN1 comms health: Bus-Off recovery count + ECU-TX enqueue failures.
    // A count > 0 after an outage confirms a recovery fired, with no debugger.
    send_or_fail_blocking(ams::config::PitDiagCommsHealthId,
                          ams::pit_diag::encode_comms_health(
                              g_fdcan1_busoff_recovery_count, g_acu_tx_fail));

    // Balance-quiesce health (0x6CB): is the pre-measurement DCC clear landing?
    // A climbing fail count means cells are sampled while bleeding, which
    // corrupts the balance selector (ranks raw cell_mV) and the SoC filter
    // (corrects on min_cell_mV).
    send_or_fail_blocking(ams::config::PitDiagBalanceHealthId,
                          ams::pit_diag::encode_balance_health(
                              g_balance_quiesce_count, g_balance_quiesce_fail_count));

    // BENCH DIAGNOSTIC (config::AdowRawDiag): raw ADOW pull-up + pull-down
    // per-cell dump on AdowDiagPuBaseId / AdowDiagPdBaseId, in the same
    // 24-frame layout as the 0x680 cell grid. Flight builds have the flag
    // false, so this is dead-code-eliminated.
    if (ams::config::AdowRawDiag) {
        for (std::uint8_t i = 0; i < ams::config::PitDiagCellFrames; ++i) {
            send_or_fail_blocking(ams::config::AdowDiagPuBaseId + i,
                                  ams::pit_diag::encode_raw_grid_frame(g_adow_diag_pu, i));
            send_or_fail_blocking(ams::config::AdowDiagPdBaseId + i,
                                  ams::pit_diag::encode_raw_grid_frame(g_adow_diag_pd, i));
        }
    }

    // BENCH DIAGNOSTIC (config::AdcModeCrossCheck): the pack re-measured in a
    // second ADC mode, on AdcXCheckBaseId in the 0x680 grid layout. Diff
    // against 0x680 -- a cell that moves with conversion time has a
    // settling-limited tap; one that does not is genuinely at that voltage.
    if (ams::config::AdcModeCrossCheck) {
        for (std::uint8_t i = 0; i < ams::config::PitDiagCellFrames; ++i) {
            send_or_fail_blocking(ams::config::AdcXCheckBaseId + i,
                                  ams::pit_diag::encode_raw_grid_frame(g_adc_xcheck_mv, i));
        }
    }
}

// Firmware-health frame at 1 Hz, emitted REGARDLESS of the pit-diag arm, so
// a passive `pit-diag listen` sees AMS liveness (heap, task liveness, reset
// cause, uptime, last fault -- ECU 0x704 parity) with the stream off.
// Non-blocking: a full TX FIFO bumps g_acu_tx_fail (surfaced on 0x6C9)
// instead of stalling the task.
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
// LOGFS diag transport.
//
// Bootloader addressing: host -> node on 0x000 + NodeID, node -> host on
// 0x010 + NodeID. No new hardware filter needed -- the global filter already
// accepts every standard frame into FIFO0.
//
// At node 2 the request ID equals the boot-trigger ID (0x002). That is safe:
// matches_trigger demands DLC 4 AND the exact B0 07 AD 11 payload, an ISO-TP
// frame is always DLC 8, and 0xB0 is not a valid ISO-TP PCI byte (only
// 0x0-0x3 are). The trigger check runs first and a LOGFS frame passes
// through it untouched.
// ---------------------------------------------------------------------------
constexpr std::uint32_t DiagRxId = 0x000u + ams::config::AmsNodeId;
constexpr std::uint32_t DiagTxId = 0x010u + ams::config::AmsNodeId;

ams::isotp::Reassembler s_diag_rx;
ams::isotp::Segmenter   s_diag_tx;
bool                    s_diag_tx_active = false;
std::uint8_t            s_diag_msg[ams::isotp::MaxMsg];

// Push as much of the outbound message as the TX FIFO will take right now.
//
// Deliberately NOT a blocking drain: a 512-byte reply is ~76 frames, and
// sitting on that would stall the telemetry cadence this task also owns.
// It dribbles out whatever fits each pass, and the loop below shortens its
// wait while a transfer is live.
void pump_diag_tx() noexcept {
    if (!s_diag_tx_active) return;
    std::uint8_t f[ams::isotp::FrameLen];
    // Leave DiagTxReservedSlots free so the telemetry matrix -- scheduled
    // AFTER this in the same loop pass, and non-blocking -- always finds room.
    // Filling the FIFO to zero blacks telemetry out for the whole
    // multi-minute pull.
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >
           ams::config::DiagTxReservedSlots) {
        if (!s_diag_tx.next(f)) { s_diag_tx_active = false; return; }
        if (!send_acu(DiagTxId, ams::isotp::FrameLen, f)) {
            // Lost the race for the slot. The frame is already consumed from
            // the segmenter, so the message is broken -- abandon it and let
            // the host time out and retry rather than ship a corrupt stream.
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
    std::uint32_t last_fw_health_tx = now0;   // ungated 1 Hz health

    // FDCAN1 Bus-Off recovery latch (single bus, single owner: this task).
    ams::can_recovery::BusOffState busoff_state{};

    for (;;) {
        const auto now           = osKernelGetTickCount();
        const auto next_fast     = last_fast_tx + ams::config::EcuFastTxMs;
        const auto next_mid      = last_mid_tx  + ams::config::EcuMidTxMs;
        const auto next_slow     = last_slow_tx + ams::config::EcuSlowTxMs;
        // The pit-diag deadline counts only while the stream is enabled;
        // otherwise park it far in the future so it never shortens the
        // queue-get timeout.
        const auto next_pit      = s_pit_diag_enabled
                                       ? (last_pit_scan + ams::config::PitDiagScanPeriodMs)
                                       : (now + 0xFFFFu);
        // While a diag reply is going out, come back every tick to top up the
        // TX FIFO. At the slowest telemetry cadence a ~76-frame message would
        // take seconds instead of tens of milliseconds.
        const auto next_diag     = s_diag_tx_active ? (now + 1u) : (now + 0xFFFFu);
        const auto deadline      = std::min({ next_fast, next_mid, next_slow, next_pit,
                                              next_diag });
        const auto timeout       = (deadline > now) ? deadline - now : 0u;

        if (osMessageQueueGet(acu_rx_queueHandle, &frame, nullptr, timeout) == osOK) {
            // Pit-diag command first: a cheap classify, and it must not be
            // mistaken for a VCU frame if dispatch fails.
            const int diag_cmd = ams::pit_diag::classify_command(frame);
            if (diag_cmd != 0) {
                const bool desired = (diag_cmd > 0);
                if (s_pit_diag_enabled != desired) {
                    s_pit_diag_enabled = desired;
                    // Reset the cadence so the first scan follows the enable
                    // command immediately, not after whatever residual delay
                    // was left on the clock.
                    last_pit_scan = osKernelGetTickCount() -
                                    ams::config::PitDiagScanPeriodMs;
                }
                tx_pit_diag_ack(s_pit_diag_enabled);
            } else {
                // The trigger check comes BEFORE VehicleService so a reboot
                // request still lands if its ID ever collides with a future
                // VCU frame. request_reboot() never returns: it opens all
                // relays, writes BL_BOOT_REQ_MAGIC into BKP0R, and resets.
                if (ams::Bootloader::matches_trigger(frame)) {
                    // State-gated: in an energised state the reboot opens AIR+
                    // under inverter load -- on a 4-byte frame anyone on the
                    // accumulator bus can send. A refusal is counted, not
                    // answered: the trigger is a bare frame with no reply
                    // channel, so the count on the comms-health frame is how
                    // an operator learns the car was too live to flash.
                    if (ams::Bootloader::reboot_allowed_in(
                            static_cast<ams::fsm::State>(g_state_telemetry))) {
                        ams::Bootloader::request_reboot(
                            ams::config::JumpReason::CanTrigger);
                    } else {
                        ++g_boot_trigger_refused;
                    }
                }
                // At node 2 LOGFS rides the same ID as the trigger, so this
                // must stay AFTER the check above.
                if (frame.id == DiagRxId &&
                    frame.bus == static_cast<std::uint8_t>(ams::CanBus::Acu)) {
                    handle_diag_frame(frame, now);
                } else if (!ams::VehicleService::instance().update_from_frame(frame)) {
                    ++g_acu_rx_dropped_unknown;
                }
            }
        }

        // Collect a finished diag reply from SdLoggerTask, start shipping it,
        // then top up the TX FIFO. Both calls are non-blocking, so the SD card
        // can never stall this task.
        if (!s_diag_tx_active) {
            const std::uint16_t n = ams::sd_diag_collect(s_diag_msg, sizeof s_diag_msg);
            if (n != 0 && s_diag_tx.begin(s_diag_msg, n)) s_diag_tx_active = true;
        }
        pump_diag_tx();

        // AcuCanTask serviced its RX queue this pass -> CAN-RX liveness.
        ams::fw_health::poke(ams::fw_health::CanRx);

        // ---- Snapshot service state once per loop iteration ----
        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto now2 = osKernelGetTickCount();

        // ---- FDCAN1 Bus-Off poll + recovery (every loop pass) ----
        // Unconditional, so it still fires while Bus_Off has the node deaf:
        // no RX arrives, the queue-get above just times out at the next TX
        // deadline, and the loop keeps spinning at <= EcuFastTxMs. The
        // recovery is rate-limited internally.
        poll_fdcan1_busoff_recovery(busoff_state, now2);

        // ---- TX scheduler ----
        if (now2 - last_fast_tx >= ams::config::EcuFastTxMs) {
            tx_currents(cur);
            ams::fw_health::poke(ams::fw_health::CanTx);   // TX path alive
            last_fast_tx = now2;
        }
        if (now2 - last_mid_tx >= ams::config::EcuMidTxMs) {
            tx_ok_precharge();
            tx_discharge_interlock();
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

        // ---- Firmware health: 1 Hz, no pit-diag arm gate ----
        if (now2 - last_fw_health_tx >= ams::config::FwHealthPeriodMs) {
            tx_fw_health();
            last_fw_health_tx = now2;
        }
    }
}
