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
    bool                tsms;             // PF9 readback (active-high)
    bool                dash_chg;          // PF10 readback (active-high)
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

    switch (in.current) {
    case State::Start: {
        // Wait for both inputs: TSMS (side-of-car external switch) = 1
        // AND DASH_CHG (cockpit dashboard / charger button) = 1,
        // level-polled at the 20 ms FSM cadence. Same gate for both
        // car and charger -- SafetyTask decides which one it is by
        // looking at the VCU 0x100 freshness at this exact moment and
        // captures the result into in.mode_locked, which we'll consume
        // on the way out of Transition.
        if (in.tsms && in.dash_chg) {
            return { State::Precharge,
                     events::safety::CloseAirN |
                     events::safety::ClosePrecharge };
        }
        return { State::Start, 0u };
    }

    case State::Precharge: {
        // No deadline -- sit in Precharge until V_dc_bus reaches
        // V_precharge_target. The supervisor's other failsafes still
        // apply: a stuck contactor or low pack-V eventually trips
        // through a freshness predicate (BmsStaleMs / VcuStaleMs /
        // IStaleMs) or a range predicate (cell U/V, T) and the FSM
        // hops to Error via the safety::evaluate_fault path above.
        if (precharge_target_reached(in.bms, in.vehicle)) {
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
        // tractive system on a degraded bus.
        if (!precharge_target_reached(in.bms, in.vehicle)) {
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
        // Any drop of TSMS or DASH_CHG while in Run latches Error and
        // opens all relays. Operator wanted conservative semantics:
        // every AIR-open event is a sticky fault requiring power
        // cycle (matches existing ErrorLatch backup-register design;
        // see error_latch.cpp).
        if (!in.tsms || !in.dash_chg) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        return { State::Run, 0u };
    }

    case State::Charge: {
        // Same exit semantics as Run -- TSMS/RST drop latches Error.
        if (!in.tsms || !in.dash_chg) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        return { State::Charge, 0u };
    }

    case State::Error:
    default:
        // Already handled at function entry; defensive default.
        return { State::Error, events::safety::ForceError };
    }
}

}  // namespace ams::fsm
