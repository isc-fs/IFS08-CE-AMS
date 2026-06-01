// SPDX-License-Identifier: proprietary
//
// MainTask — the collapsed SafetyTask + StateTask + TelemetryTask body
// from refactor/19 phase 3. Single 10 ms cadence loop that owns:
//
//   * Service snapshots (BMS / current / vehicle) once per iteration
//   * Safety predicate evaluation every 10 ms (SafetyPeriodMs)
//   * FSM step every 20 ms (StatePeriodMs)
//   * Telemetry emit every 500 ms (TelemetryPeriodMs)
//   * IWDG refresh on every iteration (gated by the fault path)
//   * AMS_OK / SDC-enable GPIO drive (PB4) every iteration: HIGH only
//     past boot grace with no ERROR latched, LOW otherwise (#299)
//
// The CubeMX-generated thread is still called "SafetyTask" in main.c +
// AMS.ioc. That cosmetic rename ships in a later phase to keep this
// commit free of CubeMX churn. The behavioural collapse is here.
//
// The relay-action ping-pong via osEventFlags between the old
// StateTask (producer) and SafetyTask (consumer) is gone: the FSM's
// output bitmask is consumed inline within this task.

#include "safety_task.hpp"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "relay_driver.hpp"
#include "safety_predicates.hpp"
#include "state_machine.hpp"
#include "telemetry_encoders.hpp"
#include "vehicle_service.hpp"
#include "watchdog.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;

// CubeMX-generated FreeRTOS task handle for BmsPollTask. NULL if
// osThreadNew silently failed at boot.
extern osThreadId_t BmsPollTaskHandle;

}

// FSM state mirror exposed for BmsPollTask / other read-only consumers.
// Updated on every transition.
extern "C" volatile std::uint8_t g_state_telemetry = 0;

// Mode locked at Start->Precharge, mirrored here so the pit-diag stream
// (#247) in AcuCanTask can surface it without taking a snapshot mutex.
// Same 8-bit volatile read pattern as g_state_telemetry. Values match
// the fsm::Mode enum (0=Undecided, 1=Car, 2=Charger).
extern "C" volatile std::uint8_t g_mode_locked_telemetry = 0;

// Telemetry TX failure counter, surfaced via a future diag frame.
extern "C" volatile std::uint32_t g_telemetry_tx_fail = 0;

// Predicate branch that latched ERROR, surfaced on pit-diag 0x6C0[6]
// (reason) and 0x6C0[7] (detail) for bench fault localisation (#276).
// Written once, on the first transition into the latched state;
// values match ams::safety::FaultReason (12 == FSM-driven Error path).
extern "C" volatile std::uint8_t g_fault_reason_telemetry = 0;
extern "C" volatile std::uint8_t g_fault_detail_telemetry = 0;

namespace ams {

SafetyTask& SafetyTask::instance() noexcept {
    static SafetyTask Instance;
    return Instance;
}

void SafetyTask::latch_error_() noexcept {
    Relays::open_all();
    Relays::set_ams_ok(false);   // drop the SDC enable with the AIRs (#299)
    ErrorLatch::set();
    error_latched_ = true;
}

namespace {

bool send_telem(std::uint32_t id, const telemetry::Frame& payload) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx,
               const_cast<std::uint8_t*>(payload.data())) == HAL_OK;
}

// Drive the relay actions encoded in the FSM's safety_flags bitmask.
// Inline replacement for the old osEventFlags(safety_eventsHandle, ...)
// ping-pong that used to hand the bits off to SafetyTask. Now both
// producer and consumer live in the same timeline so we just act.
void apply_relay_actions(std::uint32_t flags) noexcept {
    if (flags & events::safety::CloseAirN)      Relays::close_air_negative();
    if (flags & events::safety::CloseAirP)      Relays::close_air_positive();
    if (flags & events::safety::ClosePrecharge) Relays::close_precharge();
    if (flags & events::safety::OpenAirN)       Relays::open_air_negative();
    if (flags & events::safety::OpenAirP)       Relays::open_air_positive();
    if (flags & events::safety::OpenPrecharge)  Relays::open_precharge();
}

}  // namespace

void SafetyTask::run() noexcept {
    // ---------------- One-shot init ----------------
    // ErrorLatch bring-up. ErrorLatch::init is idempotent;
    // App_InitTask has almost certainly already called it. Fan PWM
    // retired in fix/48 (fan is wired permanently on; see main.c).
    ErrorLatch::init();

    // Boot in ERROR if the previous run latched it. App_InitTask
    // clears the latch under -DAMS_HIL_CLEAR_ERROR_LATCH, so on the
    // bench we come up clean unless the latch was set this session.
    const bool boot_in_error = ErrorLatch::is_set();
    if (boot_in_error) {
        Relays::open_all();
        Relays::set_ams_ok(false);   // keep the SDC open at boot (#299)
        error_latched_ = true;
    }

    fsm::State state          = boot_in_error ? fsm::State::Error : fsm::State::Start;
    fsm::Mode  mode_locked    = fsm::Mode::Undecided;
    g_state_telemetry         = static_cast<std::uint8_t>(state);

    std::uint32_t last_wake           = osKernelGetTickCount();
    std::uint32_t state_entry_tick    = last_wake;
    std::uint32_t last_state_tick     = last_wake;
    std::uint32_t last_telemetry_tick = last_wake;
    std::uint8_t  heartbeat           = 0;

    // DASH_CHG (PF10) is a MOMENTARY press button -- edge-detect it
    // (#305). Track the previous level at the 10 ms cadence and latch a
    // rising edge until the 20 ms FSM step consumes it, so a press that
    // lands between FSM steps is never lost. Seed prev from the live
    // level so a button held at boot doesn't fire a spurious edge.
    bool prev_dash_chg =
        (HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET);
    bool dash_chg_edge_pending = false;

    for (;;) {
        // ---------------- Wake at fixed 10 ms cadence ----------------
        last_wake += config::SafetyPeriodMs;
        osDelayUntil(last_wake);

        const std::uint32_t now = osKernelGetTickCount();

        // ---------------- Snapshot inputs once per iteration ----------------
        const auto bms_snap = BmsService::instance().snapshot();
        const auto cur_snap = CurrentService::instance().snapshot();
        const auto veh_snap = VehicleService::instance().snapshot();

        // Operator-facing GPIO inputs: TSMS (side-of-car external
        // switch, PF9) and DASH_CHG (cockpit dashboard / charger
        // button, PF10). Both active-high, external pull-down on the
        // carrier. Polled every 10 ms; the 20 ms FSM step consumes
        // the latest reading.
        const bool tsms    = HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)       == GPIO_PIN_SET;
        const bool dash_chg = HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET;

        // DASH_CHG rising-edge detect + latch (#305). dash_chg stays the
        // live level for telemetry (0x4A2[5] / 0x6C0[2]); the FSM consumes
        // the latched one-shot edge below and clears it after stepping.
        if (dash_chg && !prev_dash_chg) dash_chg_edge_pending = true;
        prev_dash_chg = dash_chg;

        // ---------------- Safety predicate (every 10 ms) ----------------
        constexpr bool force_error_set = false;  // no live setter
        // VCU staleness is only a fault once committed to Car mode; in
        // Charger mode (and pre-lock Undecided) the VCU is expected
        // absent, so its absence must not latch ERROR (#302). mode_locked
        // holds the previous tick's value -- the FSM lock below updates
        // it, so VcuStale arms one tick after the Car lock, which is
        // fine (the VCU is fresh at the moment of a Car lock anyway).
        const bool vcu_required = (mode_locked == fsm::Mode::Car);
        const safety::Inputs pred_in = {
            bms_snap, cur_snap, veh_snap,
            force_error_set,
            vcu_required,
            now,
        };
        const auto fault_res = safety::evaluate_fault_detail(pred_in);

        // Debounce the cell V/T range predicates (#279). A single
        // transient sub-threshold sample -- a torn read of the lock-free
        // BmsState snapshot, or an unsettled first poll / emulator-default
        // value at boot -- must not latch the sticky ERROR. The cell V/T
        // checks fault only after CellFaultConfirmTicks (~300 ms, > one
        // 250 ms voltage poll) consecutive evaluations report the SAME
        // reason; a transient that clears on the next poll never reaches
        // the count. Immediate-danger predicates (force-error, BMS
        // offline/stale, current-over-limit, VCU-stale) are NOT debounced
        // -- they latch on the first tick exactly as before.
        const bool cell_confirmed =
            cell_debounce_.update(fault_res.reason, config::CellFaultConfirmTicks);
        const bool predicate_fault =
            safety::is_cell_range_reason(fault_res.reason)
                ? cell_confirmed
                : fault_res.faulted();

        const bool fault = error_latched_ || predicate_fault;

        if (fault) {
            if (!error_latched_) {
                // Capture the branch that latched us before doing
                // anything else, so the bench can read it off pit-diag
                // 0x6C0[6]/[7] (#276).
                g_fault_reason_telemetry =
                    static_cast<std::uint8_t>(fault_res.reason);
                g_fault_detail_telemetry = fault_res.detail;

                latch_error_();
                state            = fsm::State::Error;
                state_entry_tick = now;
                g_state_telemetry = static_cast<std::uint8_t>(state);
            }
            // Stay alive in the latched state: relays already open,
            // ErrorLatch persists across reset, so refreshing the
            // watchdog is safe. Lets the operator read telemetry
            // and explicitly reset via the bootloader path. See PR
            // #107 for the loop-bug this avoids.
            Watchdog::refresh();
        } else {
            // ---------------- FSM step (every 20 ms) ----------------
            if (now - last_state_tick >= config::StatePeriodMs) {
                last_state_tick = now;

                // Lock the Car-vs-Charger mode at the EXACT iteration
                // that's about to take us out of Start. Captured here
                // (before fsm::step) so the FSM body sees a
                // self-consistent input snapshot. Read VCU 0x100
                // freshness via the snapshot already taken this tick.
                if (state == fsm::State::Start &&
                    mode_locked == fsm::Mode::Undecided &&
                    tsms && dash_chg_edge_pending) {
                    const bool vcu_fresh =
                        veh_snap.last_dc_bus_tick != 0u &&
                        (now - veh_snap.last_dc_bus_tick) <=
                            config::VcuFreshMs;
                    // Charger mode requires BOTH the operator's explicit
                    // charge-mode request AND VCU absence (#305). A car
                    // with a dead VCU does NOT send the request, so it
                    // locks Car and faults on VcuStale instead of
                    // silently charging; a stray charge request while the
                    // VCU is live cannot flip a running car into Charger.
                    const bool charge_req = VehicleService::charge_requested(
                        now, veh_snap.last_charge_req_tick);
                    mode_locked = (charge_req && !vcu_fresh)
                                      ? fsm::Mode::Charger
                                      : fsm::Mode::Car;
                    g_mode_locked_telemetry =
                        static_cast<std::uint8_t>(mode_locked);
                }

                const fsm::Inputs fsm_in = {
                    state, bms_snap, cur_snap, veh_snap,
                    tsms, dash_chg_edge_pending, mode_locked,
                    // Pass the already-debounced fault decision (#279);
                    // the FSM no longer re-evaluates the predicate. This
                    // is false here (step() only runs on a no-fault
                    // tick), but it keeps the FSM's Error backstop honest.
                    predicate_fault,
                    now, state_entry_tick,
                };
                const auto out = fsm::step(fsm_in);

                // The DASH_CHG edge is one-shot: consume it now that the
                // FSM (and the mode lock above) have seen it, so a single
                // press drives at most one transition (#305).
                dash_chg_edge_pending = false;

                apply_relay_actions(out.safety_flags);

                if (out.next != state) {
                    state             = out.next;
                    state_entry_tick  = now;
                    g_state_telemetry = static_cast<std::uint8_t>(state);

                    // Persist ERROR across resets even when the FSM
                    // got there without a predicate fault (e.g.
                    // precharge timeout, or DASH_CHG dropped mid-Run).
                    if (state == fsm::State::Error) {
                        ErrorLatch::set();
                        error_latched_ = true;
                        // Distinguish FSM-driven Error (precharge
                        // timeout / fault) from a predicate fault on
                        // pit-diag 0x6C0[6] (#276). 12 == FsmError
                        // (reserved past FaultReason).
                        if (g_fault_reason_telemetry == 0u) {
                            g_fault_reason_telemetry = 12u;
                        }
                    }

                    // Fell back to idle -- a TSMS drop de-energised us
                    // without latching (#327), or any other return to
                    // Start. Clear the mode lock so the re-arm re-locks
                    // Car/Charger and re-runs precharge from scratch.
                    if (state == fsm::State::Start) {
                        mode_locked = fsm::Mode::Undecided;
                        g_mode_locked_telemetry =
                            static_cast<std::uint8_t>(mode_locked);
                    }
                }
            }
            Watchdog::refresh();
        }

        // ---------------- AMS_OK / SDC enable (every 10 ms) (#299) ----------------
        // Drive PB4 to track the live safety state: HIGH only once the
        // boot grace has passed AND no ERROR is latched; LOW during grace
        // (predicates suppressed) and LOW the moment a fault latches. The
        // firmware previously never drove this pin, so the SDC enable was
        // never asserted in a healthy state and never deasserted on a
        // fault -- it just decayed from its boot-default level. error_latched_
        // already reflects this tick's fault/FSM decision above.
        Relays::set_ams_ok(safety::ams_ok_asserted(now, error_latched_));

        // ---------------- Telemetry (every 500 ms, regardless of state) ----------------
        if (now - last_telemetry_tick >= config::TelemetryPeriodMs) {
            last_telemetry_tick = now;

            const std::uint8_t ams_ok =
                (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET)
                    ? 1u : 0u;

            // Three telemetry frames. Diagnostic surfaces previously
            // gated behind a HIL build flag now live on the pit-diag
            // stream (0x6C0..0x6C8) which carries strictly more info.
            const std::uint8_t tx_fail_lo = static_cast<std::uint8_t>(
                g_telemetry_tx_fail & 0xFFu);

            // Cockpit byte at 0x4A2[5]: always-on cockpit-input snapshot.
            //   bit 7    1 (sentinel; distinguishes "live byte" from
            //              "byte got elided by an older firmware")
            //   bits 3:2 mode_locked (00=Undecided, 01=Car, 10=Charger)
            //   bit 1    TSMS readback (PF9)
            //   bit 0    DASH_CHG readback (PF10)
            const std::uint8_t tsms_dash_chg_byte = static_cast<std::uint8_t>(
                0x80u |
                (static_cast<std::uint8_t>(mode_locked) << 2) |
                (tsms    ? 0x02u : 0u) |
                (dash_chg ? 0x01u : 0u));

            const auto frame_status = telemetry::encode_status(
                g_state_telemetry, ams_ok, bms_snap);
            const auto frame_pack   = telemetry::encode_pack(bms_snap, cur_snap);
            const auto frame_temps  = telemetry::encode_temps(
                bms_snap, veh_snap, heartbeat, tx_fail_lo, tsms_dash_chg_byte);

            if (!send_telem(config::AmsTelemStatusId, frame_status)) ++g_telemetry_tx_fail;
            if (!send_telem(config::AmsTelemPackId,   frame_pack))   ++g_telemetry_tx_fail;
            if (!send_telem(config::AmsTelemTempsId,  frame_temps))  ++g_telemetry_tx_fail;

            ++heartbeat;  // 8-bit wraparound is intentional
        }
    }
}

}  // namespace ams

extern "C" void ams_safety_task_run(void *argument) {
    (void)argument;
    ams::SafetyTask::instance().run();
}
