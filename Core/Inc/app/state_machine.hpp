// SPDX-License-Identifier: proprietary
//
// Pure-logic AMS finite state machine. Given the 4 input snapshots
// (BMS, current, vehicle, SDC) plus the current FSM state + tick,
// returns the next state and the set of safety_events relay-action
// bits to post.
//
// No FreeRTOS, no HAL -> fully unit-testable.

#pragma once

#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include <cstdint>

namespace ams::fsm {

enum class State : std::uint8_t {
    Start      = 0,
    Precharge  = 1,
    Transition = 2,
    Run        = 3,
    Charge     = 4,
    Error      = 5,
};

// Mode locked at the moment of Start->Precharge. Determines whether
// Transition exits into Run or Charge, and never re-evaluated for the
// rest of the boot cycle. Captured by SafetyTask, not by fsm::step --
// here only as an FSM input.
enum class Mode : std::uint8_t {
    Undecided = 0,    // before Start->Precharge has fired
    Car       = 1,    // VCU 0x100 heard within VcuFreshMs at the trigger
    Charger   = 2,    // VCU 0x100 silent at the trigger
};

struct Inputs {
    State               current;
    const BmsState&     bms;
    const CurrentState& current_sensor;
    const VehicleState& vehicle;
    bool                tsms;             // PF9 readback, LEVEL (held master switch)
    // PF10 is a MOMENTARY press button -- SafetyTask edge-detects it and
    // passes a one-shot RISING-EDGE flag here, latched until the FSM
    // consumes it. A single press drives Start->Precharge (with TSMS), in
    // BOTH car and charger. The charger's Precharge->Transition proceed is
    // gated on 0x101 freshness (the auto-emitted charge request), NOT a
    // second press. Run/Charge do NOT look at it -- they are sustained by
    // TSMS alone.
    bool                dash_chg_edge;
    Mode                mode_locked;      // set by SafetyTask at Start->Precharge
    // The safety supervisor's ALREADY-DEBOUNCED fault decision (#279).
    // SafetyTask is the single fault authority: it runs the predicate
    // set, debounces the cell V/T range checks, and passes the result
    // here. The FSM must NOT re-evaluate the predicate itself -- doing
    // so bypassed the debounce and let a transient cell < 2800 mV latch
    // FsmError at the boot-grace edge. SafetyTask only calls step() on a
    // no-fault tick, so this is false in normal operation; the
    // any-state-to-Error branch below is a kept backstop.
    bool                predicate_fault;
    // SafetyTask's DEBOUNCED "the DC bus collapsed while we think we're in
    // Run" decision (#330). The cockpit SDC shutdown opens the AIRs without
    // the AMS sensing it; the VCU still reports dc_bus_V, so a sustained
    // collapse means the AIRs opened externally. Run consumes this to fall
    // back to Start (non-latching) rather than reclosing AIR+ onto a
    // discharged DC-link. Car/Run only; false in every other state.
    bool                bus_collapsed;
    std::uint32_t       now_tick;
    std::uint32_t       state_entry_tick;
};

struct Output {
    State         next;
    std::uint32_t safety_flags;  // bitmask of events::safety::*
};

// Precharge target: DC bus must be at least 95% of the measured pack
// voltage (sum of cells). Compare entirely in mV so a sub-1V pack
// (truncates to 0 V if divided to volts) doesn't silently bypass
// the "no data yet" guard. Vehicle dc_bus_V is uint16 V, multiply
// by 1000 to land in mV; max possible value 65535 * 1000 = 6.5e7
// fits in uint32 with headroom.
[[nodiscard]] inline bool precharge_target_reached(const BmsState& bms,
                                                   const VehicleState& veh) noexcept {
    // "No data yet" guard: pack_voltage_mV is 0 until BmsPollTask
    // (or the HIL stub) has written at least one cycle. A real pack
    // can never reach 0 mV in-service, so 0 reliably means "no data".
    if (bms.pack_voltage_mV == 0u) return false;
    const std::uint64_t bus_mV  = static_cast<std::uint64_t>(veh.dc_bus_V) * 1000u;
    const std::uint64_t pack_mV = bms.pack_voltage_mV;
    return bus_mV * 100u >= pack_mV * 95u;
}

// DC-bus collapse detector (#330). True when the VCU-measured bus has
// fallen well below the pack voltage -- i.e. the AIRs opened externally
// (a cockpit SDC shutdown the AMS can't sense) while the FSM still thinks
// it's in Run. Same mV comparison as precharge_target_reached, against
// the looser BusCollapsePercent. Returns false with no pack data yet
// (pack_voltage_mV == 0) so it can't false-fire during bring-up.
// SafetyTask debounces this before handing the FSM `bus_collapsed`.
[[nodiscard]] inline bool bus_below_collapse(const BmsState& bms,
                                             const VehicleState& veh) noexcept {
    if (bms.pack_voltage_mV == 0u) return false;
    const std::uint64_t bus_mV  = static_cast<std::uint64_t>(veh.dc_bus_V) * 1000u;
    const std::uint64_t pack_mV = bms.pack_voltage_mV;
    return bus_mV * 100u < pack_mV * static_cast<std::uint64_t>(config::BusCollapsePercent);
}

[[nodiscard]] inline Output step(const Inputs& in) noexcept {
    // Sticky ERROR. Once latched, only a reset can clear (the safety
    // task writes the backup-register magic; App_InitTask reads it
    // and seeds State::Error at boot).
    if (in.current == State::Error) {
        return { State::Error, events::safety::ForceError };
    }

    // Any fault from the predicate set forces ERROR + opens AIRs.
    // Sourced from SafetyTask's already-debounced decision (#279) --
    // the FSM does not re-run the predicate (that bypassed the cell
    // V/T debounce). Backstop: in normal operation SafetyTask handles
    // the fault before ever calling step(), so this is false here.
    if (in.predicate_fault) {
        return { State::Error,
                 events::safety::ForceError |
                 events::safety::OpenAirN | events::safety::OpenAirP |
                 events::safety::OpenPrecharge };
    }

    // TSMS (PF9) is the held master enable for every energised state.
    // Its drop is a NORMAL operator de-energise, NOT a fault: open all
    // contactors and fall back to Start WITHOUT latching Error. This is
    // load-bearing for the FS "driver must be able to stop + restart the
    // tractive system from the cockpit, unaided" rule. AMS_OK is the
    // AMS's own SDC relay and sits UPSTREAM of TSMS in the loop; if a
    // TSMS drop latched Error it would drop AMS_OK, opening that upstream
    // relay, and reclosing TSMS could no longer restore the loop without
    // a reset/power-cycle. So AMS_OK stays health-only (driven by the
    // predicate set, never by TSMS), and a TSMS drop just returns us to
    // Start -- the driver re-arms with a DASH_CHG press, which re-runs
    // precharge. Genuine pack faults still latch via predicate_fault
    // above; this branch is reached only on a no-fault tick. (#327)
    if (in.current != State::Start && in.current != State::Error &&
        !in.tsms) {
        return { State::Start,
                 events::safety::OpenAirN | events::safety::OpenAirP |
                 events::safety::OpenPrecharge };
    }

    switch (in.current) {
    case State::Start: {
        // Leave Start on a DASH_CHG press (rising edge) while TSMS (the
        // held master switch) is on. SafetyTask has already locked the mode
        // for THIS iteration (safety_task.cpp), so in.mode_locked is Car or
        // Charger here -- never Undecided on a real transition. The press is
        // edge-detected so the operator must deliberately press -- not merely
        // hold a level -- to energise.
        //
        // Charger SKIPS the precharge resistor: the charger voltage-matches
        // its output to the pack BEFORE asserting the 0x101 request, so
        // closing AIR+ onto it has no inrush. The precharge contactor sits in
        // PARALLEL with AIR+, so closing it while the charger sources current
        // would route the full charge current through the transient-rated
        // resistor. Charger therefore closes only AIR- here and AIR+ on the
        // proceed; the resistor never enters the charge loop. Car keeps the
        // resistor precharge to soft-charge the inverter DC-link. Undecided
        // (shouldn't reach here) falls back to the conservative resistor path.
        if (in.tsms && in.dash_chg_edge) {
            const std::uint32_t connect =
                (in.mode_locked == Mode::Charger)
                    ? events::safety::CloseAirN
                    : (events::safety::CloseAirN |
                       events::safety::ClosePrecharge);
            return { State::Precharge, connect };
        }
        return { State::Start, 0u };
    }

    case State::Precharge: {
        // Bounded precharge (#302 follow-up). If the bus doesn't reach
        // the target within PrechargeMaxMs, latch Error and open every
        // contactor. This caps how long the precharge contactor +
        // resistor are held closed -- protecting the resistor (transient
        // duty only) for ANY stuck-precharge cause. The case that drove
        // this: a car with a dead VCU locks Charger mode (VCU-absence is
        // ambiguous) and, since dc_bus_V comes only from the VCU's 0x100,
        // precharge_target_reached can never become true, so it would
        // otherwise sit here forever. now_tick >= state_entry_tick always
        // (both owned by SafetyTask, which sets entry = now on the edge),
        // so the subtraction can't underflow.
        if (in.now_tick - in.state_entry_tick > config::PrechargeMaxMs) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        // Precharge-complete criterion is mode-specific (#305):
        //  - Car: confirm the inverter DC-link reached the target via
        //    dc_bus_V (VCU-measured) before closing AIR+.
        //  - Charger: the inverter isn't in the charge loop and dc_bus_V
        //    is VCU-only (absent during a charge), so there is nothing to
        //    voltage-gate on; the charger soft-starts its own output. The
        //    charger now auto-emits the 0x101 charge-mode request the
        //    moment it is connected (>=2 Hz), so a STILL-FRESH 0x101 is the
        //    "charger is connected and ready" proceed signal -- the single
        //    DASH_CHG press that entered Precharge is the human "go", and
        //    0x101 freshness authorises closing AIR+. If 0x101 goes stale
        //    before we proceed (charger unplugged / aborted), precharge
        //    holds and hits the PrechargeMaxMs timeout above -> Error,
        //    rather than closing AIR+ into a disconnected charger.
        const bool precharge_done =
            (in.mode_locked == Mode::Charger)
                ? VehicleService::charge_requested(in.now_tick,
                                                   in.vehicle.last_charge_req_tick)
                : precharge_target_reached(in.bms, in.vehicle);
        if (precharge_done) {
            return { State::Transition,
                     events::safety::CloseAirP |
                     events::safety::OpenPrecharge };
        }
        return { State::Precharge, 0u };
    }

    case State::Transition: {
        // No hold timer. Transition is a one-FSM-step passthrough:
        // we entered with the contactor swap (CloseAirP|OpenPrecharge)
        // already emitted on the Precharge->Transition edge; commit to
        // Run/Charge on this step. The bus-still-up guard remains so a
        // failed contactor swap (bus slumps the moment the precharge
        // contactor opens) lands in Error rather than energising the
        // tractive system on a degraded bus. Car-only: it relies on the
        // VCU-measured dc_bus_V, absent during a charge (#305), so
        // Charger commits to Charge directly.
        if (in.mode_locked == Mode::Car &&
            !precharge_target_reached(in.bms, in.vehicle)) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        // Branch on the mode SafetyTask captured at Start->Precharge.
        // Mode::Undecided here would be a programming error (Transition
        // reached without going through Start); treat as fault.
        if (in.mode_locked == Mode::Car) {
            return { State::Run, 0u };
        }
        if (in.mode_locked == Mode::Charger) {
            return { State::Charge, 0u };
        }
        return { State::Error,
                 events::safety::ForceError |
                 events::safety::OpenAirN | events::safety::OpenAirP |
                 events::safety::OpenPrecharge };
    }

    case State::Run: {
        // AIRs opened externally (#330). The cockpit SDC shutdown opens the
        // AIRs without the AMS sensing it; the VCU keeps reporting dc_bus_V,
        // so a sustained collapse (SafetyTask-debounced -> in.bus_collapsed)
        // means the contactors are physically open while we still think
        // we're in Run. De-energise to Start (non-latching, like a TSMS
        // drop) so a re-arm re-runs precharge rather than reclosing AIR+
        // onto a discharged DC-link when the shutdown is released.
        if (in.bus_collapsed) {
            return { State::Start,
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        // Otherwise sustained while TSMS (the held master switch) is on. A
        // TSMS drop is handled by the non-latching TSMS guard above (-> Start,
        // no Error, #327), so by the time we're here TSMS is held. DASH_CHG
        // is NOT checked here: it is a momentary press, low most of the time
        // -- checking its level would fault Run instantly (#305).
        return { State::Run, 0u };
    }

    case State::Charge: {
        // Same as Run -- TSMS-held is guaranteed by the guard above; a
        // TSMS drop de-energises to Start without latching (#327). DASH_CHG
        // is a momentary press, not checked here (#305).
        return { State::Charge, 0u };
    }

    case State::Error:
    default:
        // Already handled at function entry; defensive default.
        return { State::Error, events::safety::ForceError };
    }
}

}  // namespace ams::fsm
