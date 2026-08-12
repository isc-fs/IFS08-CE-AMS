// SPDX-License-Identifier: proprietary
//
// MainTask — one 10 ms loop owning safety, the FSM, the relays and
// telemetry. Every iteration it:
//
//   * snapshots the BMS / current / vehicle services once,
//   * evaluates the safety predicates (SafetyPeriodMs, 10 ms),
//   * steps the FSM every 20 ms (StatePeriodMs),
//   * emits telemetry every 500 ms (TelemetryPeriodMs),
//   * refreshes the IWDG, on the fault path as well as the clean one,
//   * drives AMS_OK / SDC-enable (PB4): HIGH only past boot grace with
//     no ERROR latched, LOW otherwise.
//
// The CubeMX thread is still named "SafetyTask" in main.c + AMS.ioc;
// this file is its body. The FSM's relay-action bitmask is consumed
// inline here, not handed off through osEventFlags.

#include "safety_task.hpp"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "fw_health.hpp"
#include "relay_driver.hpp"
#include "safety_predicates.hpp"
#include "state_machine.hpp"
#include "telemetry_encoders.hpp"
#include "vehicle_service.hpp"
#include "watchdog.hpp"

#include "app/sd_logger_task.h"

#include "cmsis_os2.h"
#include "main.h"

#include <cstring>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;

// CubeMX-generated FreeRTOS task handle for BmsPollTask. NULL if
// osThreadNew silently failed at boot.
extern osThreadId_t BmsPollTaskHandle;

}

// FSM state mirror exposed for BmsPollTask / other read-only consumers.
// Updated on every transition.
extern "C" volatile std::uint8_t g_state_telemetry = 0;

// Mode locked at Start->Precharge, mirrored here so the pit-diag stream in
// AcuCanTask can read it without taking a snapshot mutex. Same single-writer
// 8-bit contract as g_state_telemetry. Values match the fsm::Mode enum
// (0=Undecided, 1=Car, 2=Charger).
extern "C" volatile std::uint8_t g_mode_locked_telemetry = 0;

// TSMS (PF9) level mirror. 1 = the shutdown circuit is complete; any open
// shutdown element pulls it low. It is therefore also the AMS's view of whether
// the discharge relay is energised and the bleed disconnected. Published on
// 0x021 so the ECU, which cannot see the SDC, can decide whether to secure an
// interrupted discharge. Same single-writer 8-bit contract as g_state_telemetry.
extern "C" volatile std::uint8_t g_tsms_telemetry = 0;

// Telemetry TX failure counter, surfaced via a future diag frame.
extern "C" volatile std::uint32_t g_telemetry_tx_fail = 0;

// Which branch latched ERROR, on pit-diag 0x6C0[6] (reason) and 0x6C0[7]
// (detail) for bench fault localisation. Written once, on the first transition
// into the latched state. Values match ams::safety::FaultReason
// (12 == FSM-driven Error path).
extern "C" volatile std::uint8_t g_fault_reason_telemetry = 0;
extern "C" volatile std::uint8_t g_fault_detail_telemetry = 0;

namespace ams {

SafetyTask& SafetyTask::instance() noexcept {
    static SafetyTask Instance;
    return Instance;
}

void SafetyTask::latch_error_() noexcept {
    Relays::open_all();
    Relays::set_ams_ok(false);   // drop the SDC enable with the AIRs
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
    // ErrorLatch::init is idempotent, and App_InitTask has almost certainly
    // called it already. Nothing to start for the fan: it is wired
    // permanently on (main.c).
    ErrorLatch::init();

    // Boot in ERROR if a previous run latched it. The bench build
    // (-DAMS_HIL_CLEAR_ERROR_LATCH) has App_InitTask clear the latch, so
    // there we come up clean unless it was set this session.
    const bool boot_in_error = ErrorLatch::is_set();
    if (boot_in_error) {
        Relays::open_all();
        Relays::set_ams_ok(false);   // keep the SDC open at boot
        error_latched_ = true;
    }

    fsm::State state          = boot_in_error ? fsm::State::Error : fsm::State::Start;
    fsm::Mode  mode_locked    = fsm::Mode::Undecided;
    g_state_telemetry         = static_cast<std::uint8_t>(state);

    std::uint32_t last_wake           = osKernelGetTickCount();
    std::uint32_t state_entry_tick    = last_wake;
    std::uint32_t last_state_tick     = last_wake;
    std::uint32_t last_telemetry_tick = last_wake;
    std::uint32_t last_relay_tick     = last_wake;
    std::uint32_t last_log_tick       = last_wake;
    std::uint8_t  heartbeat           = 0;

    // DASH_CHG (PF10) is a MOMENTARY button, so edge-detect it: sample the
    // level every 10 ms and latch a rising edge until the 20 ms FSM step
    // consumes it, so a press landing between FSM steps is never lost. Seed
    // prev from the live level so a button held at boot fires no edge.
    bool prev_dash_chg =
        (HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET);
    bool dash_chg_edge_pending = false;

    // Consecutive 10 ms ticks of DC-bus collapse; see the debounce below.
    std::uint16_t bus_collapse_count = 0;

    for (;;) {
        // ---------------- Wake at fixed 10 ms cadence ----------------
        last_wake += config::SafetyPeriodMs;
        osDelayUntil(last_wake);

        const std::uint32_t now = osKernelGetTickCount();

        // MainTask stepped this 10 ms tick -> control-task liveness.
        ams::fw_health::poke(ams::fw_health::MainStepped);

        // ---------------- Snapshot inputs once per iteration ----------------
        const auto bms_snap = BmsService::instance().snapshot();
        const auto cur_snap = CurrentService::instance().snapshot();
        const auto veh_snap = VehicleService::instance().snapshot();

        // VCU 0x100 freshness for precharge_target_reached. Held to VcuStaleMs,
        // not the looser VcuFreshMs used for the mode lock: this criterion gates
        // closing AIR+, so the same staleness that raises VcuStale must also make
        // dc_bus_V unreadable. A never-seen VCU (tick 0) is not fresh.
        const bool dc_bus_fresh =
            veh_snap.last_dc_bus_tick != 0u &&
            (now - veh_snap.last_dc_bus_tick) <= config::VcuStaleMs;

        // Operator GPIO inputs, both active-high with an external pull-down
        // on the carrier: TSMS (side-of-car switch, PF9) and DASH_CHG
        // (cockpit / charger button, PF10). Polled every 10 ms; the 20 ms
        // FSM step consumes the latest reading.
        const bool tsms    = HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)       == GPIO_PIN_SET;
        g_tsms_telemetry = tsms ? 1u : 0u;
        const bool dash_chg = HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET;

        // DASH_CHG rising-edge detect + latch. dash_chg stays the
        // live level for telemetry (0x4A2[5] / 0x6C0[2]); the FSM consumes
        // the latched one-shot edge below and clears it after stepping.
        if (dash_chg && !prev_dash_chg) dash_chg_edge_pending = true;
        prev_dash_chg = dash_chg;

        // ---------------- Safety predicate (every 10 ms) ----------------
        constexpr bool force_error_set = false;  // no live setter
        // Each heartbeat is required only once the FSM has committed to the
        // matching mode: VCU 0x100 in Car, charger 0x101 in Charger. Before the lock (Undecided) both
        // are legitimately absent and must not latch ERROR. mode_locked still
        // holds the previous tick's value here -- the FSM lock below updates
        // it -- so each check arms one tick after its lock. That is fine: the
        // frame is fresh at the moment of the lock anyway.
        const bool vcu_required = (mode_locked == fsm::Mode::Car);
        const bool charger_required = (mode_locked == fsm::Mode::Charger);
        const safety::Inputs pred_in = {
            bms_snap, cur_snap, veh_snap,
            force_error_set,
            vcu_required,
            charger_required,
            now,
        };
        const auto fault_res = safety::evaluate_fault_detail(pred_in);

        // Debounce the slow-by-nature faults so one bad sample -- a torn read
        // of the lock-free BmsState snapshot, or an unsettled first poll at
        // boot -- cannot latch the sticky ERROR:
        //   cell V/T ranges -> cell_debounce_      / CellFaultConfirmTicks
        //   BmsStale        -> bms_stale_debounce_ / BmsStaleConfirmTicks
        // Both run every tick and self-gate on the reason: the count advances
        // only while consecutive evaluations report the SAME reason, so a
        // transient that clears on the next poll never reaches it.
        // CellFaultConfirmTicks = 25 x 10 ms = ~250 ms, spanning more than one
        // 200 ms BmsPollVoltMs cycle; with the poll that is ~460 ms worst case,
        // inside the < 500 ms fault-response budget. Debouncing BmsStale keeps
        // a far module that flickers just past the stale window on an EMI burst
        // from latching; a sustained loss still latches, ~250 ms later.
        // Every other predicate -- force-error, BMS module offline, current
        // over-limit/stale, VCU-stale -- latches on the FIRST tick.
        const bool cell_confirmed =
            cell_debounce_.update(fault_res.reason, config::CellFaultConfirmTicks);
        const bool bms_stale_confirmed =
            bms_stale_debounce_.update(fault_res.reason, config::BmsStaleConfirmTicks);
        const bool predicate_fault =
            safety::is_cell_range_reason(fault_res.reason) ? cell_confirmed
            : (fault_res.reason == safety::FaultReason::BmsStale) ? bms_stale_confirmed
            : fault_res.faulted();

        const bool fault = error_latched_ || predicate_fault;

        if (fault) {
            if (!error_latched_) {
                // Capture the branch that latched us before anything else,
                // so the bench can read it off pit-diag 0x6C0[6]/[7].
                g_fault_reason_telemetry =
                    static_cast<std::uint8_t>(fault_res.reason);
                g_fault_detail_telemetry = fault_res.detail;

                latch_error_();
                state            = fsm::State::Error;
                state_entry_tick = now;
                g_state_telemetry = static_cast<std::uint8_t>(state);
            }
            // Refresh the watchdog here too, deliberately. The relays are
            // already open and ErrorLatch survives a reset, so staying alive
            // is safe, and it lets the operator read telemetry and reset
            // explicitly via the bootloader path.
            Watchdog::refresh();
        } else {
            // ---------- DC-bus collapse debounce ----------
            // Only meaningful in Run (Car): the bus tracks the pack there, so
            // a sustained collapse means the AIRs were opened externally (a
            // cockpit SDC shutdown the AMS can't sense). Count consecutive
            // collapsed ticks; the FSM consumes the confirmed flag and falls
            // back to Start so a re-arm re-runs precharge. Any non-qualifying
            // tick resets the count.
            bool bus_collapsed = false;
            if (state == fsm::State::Run &&
                mode_locked == fsm::Mode::Car &&
                fsm::bus_below_collapse(bms_snap, veh_snap)) {
                if (bus_collapse_count < config::BusCollapseConfirmTicks) {
                    ++bus_collapse_count;
                }
                bus_collapsed =
                    (bus_collapse_count >= config::BusCollapseConfirmTicks);
            } else {
                bus_collapse_count = 0;
            }

            // ---------------- FSM step (every 20 ms) ----------------
            if (now - last_state_tick >= config::StatePeriodMs) {
                last_state_tick = now;

                // Lock Car-vs-Charger on the EXACT iteration about to leave
                // Start, before fsm::step, so the FSM body sees a
                // self-consistent input snapshot.
                if (state == fsm::State::Start &&
                    mode_locked == fsm::Mode::Undecided &&
                    tsms && dash_chg_edge_pending) {
                    const bool vcu_fresh =
                        veh_snap.last_dc_bus_tick != 0u &&
                        (now - veh_snap.last_dc_bus_tick) <=
                            config::VcuFreshMs;
                    // Charger mode needs BOTH an explicit charge request AND
                    // an absent VCU. A car with a dead VCU sends no request,
                    // so it locks Car and faults on VcuStale rather than
                    // silently charging; a stray request while the VCU is
                    // live cannot flip a running car into Charger.
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
                    // The already-debounced decision; the FSM does not
                    // re-evaluate. Always false here (step() runs only on a
                    // no-fault tick) but keeps the FSM's Error backstop honest.
                    predicate_fault,
                    bus_collapsed,   // debounced bus-collapse decision
                    dc_bus_fresh,
                    now, state_entry_tick,
                };
                const auto out = fsm::step(fsm_in);

                // One-shot: consume the edge now that the FSM and the mode
                // lock have seen it, so one press drives at most one
                // transition.
                dash_chg_edge_pending = false;

                apply_relay_actions(out.safety_flags);

                if (out.next != state) {
                    state             = out.next;
                    state_entry_tick  = now;
                    g_state_telemetry = static_cast<std::uint8_t>(state);

                    // Persist ERROR across resets even when the FSM got there
                    // without a predicate fault: precharge timeout, a failed
                    // contactor swap in Transition, or a TSMS drop in
                    // Charger mode.
                    if (state == fsm::State::Error) {
                        ErrorLatch::set();
                        error_latched_ = true;
                        // Distinguish FSM-driven Error from a predicate fault
                        // on pit-diag 0x6C0[6]: 12 == FsmError, or 15 ==
                        // ChargerTsmsOpen when a TSMS drop latched us in
                        // Charger mode (scrutineering: the charge output must
                        // not be re-activatable).
                        if (g_fault_reason_telemetry == 0u) {
                            g_fault_reason_telemetry = safety::fsm_error_reason(
                                mode_locked == fsm::Mode::Charger, tsms);
                        }
                    }

                    // Any return to Start -- a TSMS drop de-energising us
                    // without latching, or anything else -- clears the mode
                    // lock, so the re-arm re-locks Car/Charger and re-runs
                    // precharge from scratch.
                    if (state == fsm::State::Start) {
                        mode_locked = fsm::Mode::Undecided;
                        g_mode_locked_telemetry =
                            static_cast<std::uint8_t>(mode_locked);
                    }
                }
            }
            Watchdog::refresh();
        }

        // ---------------- AMS_OK / SDC enable (every 10 ms) ----------------
        // Drive PB4 to the live safety state: HIGH only once the boot grace
        // (SafetyBootGraceMs) has passed AND no ERROR is latched; LOW during
        // the grace, where predicates are suppressed, and LOW the moment a
        // fault latches. error_latched_ already reflects this tick's decision.
        Relays::set_ams_ok(safety::ams_ok_asserted(now, error_latched_));

        // ---------------- Telemetry (every 500 ms, regardless of state) ----------------
        if (now - last_telemetry_tick >= config::TelemetryPeriodMs) {
            last_telemetry_tick = now;

            const std::uint8_t ams_ok =
                (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET)
                    ? 1u : 0u;

            // Three telemetry frames; deeper diagnostics live on the
            // pit-diag stream (0x6C0..0x6C8).
            const std::uint8_t tx_fail_lo = static_cast<std::uint8_t>(
                g_telemetry_tx_fail & 0xFFu);

            // Cockpit-input byte at 0x4A2[5]:
            //   bit 7    1 (sentinel: a live byte, not an elided one)
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

        // 0x4A4 AMS_relay_status -- contactor + AMS_OK read-back on its own
        // cadence (RelayStatusPeriodMs), so an external logger can watch the
        // AIR / precharge sequence without arming pit-diag. These are ODR
        // read-backs: what we drive the coils to, NOT proof that a contactor
        // physically closed.
        if (now - last_relay_tick >= config::RelayStatusPeriodMs) {
            last_relay_tick = now;
            const bool ams_ok_pin =
                (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET);
            const auto frame_relay = telemetry::encode_relay_status(
                Relays::is_air_negative_closed(),
                Relays::is_air_positive_closed(),
                Relays::is_precharge_closed(),
                ams_ok_pin);
            if (!send_telem(config::AmsRelayStatusId, frame_relay))
                ++g_telemetry_tx_fail;
        }

        // ---------------- Datalogging sample (every LogSamplePeriodMs) ----------------
        // Best-effort hand-off to SdLoggerTask over the wait-free ring, built
        // from THIS tick's snapshots. sd_log_push() never blocks and never
        // faults -- a full ring (SD stall / log-pull) just drops the record.
        // Off the safety path; the only cost on the 10 ms loop is a bounded
        // ~590 B struct copy at 4 Hz.
        if (now - last_log_tick >= config::LogSamplePeriodMs) {
            last_log_tick = now;
            // Static scratch: keep the 632 B record off the SafetyTask stack
            // (it already holds a ~620 B bms_snap). Single-writer, fully
            // overwritten every pass before push -- no concurrency concern.
            static LogRecord rec{};
            rec.tick_ms             = now;
            rec.pack_mV             = bms_snap.pack_voltage_mV;
            rec.pack_current_raw_mA = cur_snap.raw_mA;
            rec.pack_current_mA     = cur_snap.filtered_mA;
            rec.dcdc_current_mA     = cur_snap.dcdc_filtered_mA;
            rec.min_cell_mV         = bms_snap.min_cell_mV;
            rec.max_cell_mV         = bms_snap.max_cell_mV;
            rec.dc_bus_V            = veh_snap.dc_bus_V;
            rec.min_tempC           = bms_snap.min_tempC;
            rec.max_tempC           = bms_snap.max_tempC;
            rec.avg_tempC           = bms_snap.avg_tempC;
            rec.fsm_state           = g_state_telemetry;
            rec.mode                = g_mode_locked_telemetry;
            rec.fault_reason        = g_fault_reason_telemetry;
            rec.fault_detail        = g_fault_detail_telemetry;
            rec.module_online_mask  = bms_snap.module_online_mask;
            rec.ams_ok              = (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin)
                                       == GPIO_PIN_SET) ? 1u : 0u;
            rec.tsms                = tsms ? 1u : 0u;
            rec.dash_chg            = dash_chg ? 1u : 0u;
            std::memcpy(rec.cell_mV,    bms_snap.cell_mV,    sizeof rec.cell_mV);
            std::memcpy(rec.cell_tempC, bms_snap.cell_tempC, sizeof rec.cell_tempC);
            sd_log_push(rec);
        }
    }
}

}  // namespace ams

extern "C" void ams_safety_task_run(void *argument) {
    (void)argument;
    ams::SafetyTask::instance().run();
}
