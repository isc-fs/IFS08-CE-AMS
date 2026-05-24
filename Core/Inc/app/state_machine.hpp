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
#include "safety_predicates.hpp"
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
    bool                force_error_set;
    std::uint32_t       now_tick;
    std::uint32_t       state_entry_tick;
};

struct Output {
    State         next;
    std::uint32_t safety_flags;  // bitmask of events::safety::*
};

// Precharge target: DC bus must be at least 95% of the measured pack
// voltage (sum of cells). Pack voltage in BmsState is mV, vehicle
// DC bus is V; convert and compare.
[[nodiscard]] inline bool precharge_target_reached(const BmsState& bms,
                                                   const VehicleState& veh) noexcept {
    const std::uint32_t pack_V = bms.pack_voltage_mV / 1000u;
    if (pack_V == 0) return false;  // no BMS data yet
    return static_cast<std::uint32_t>(veh.dc_bus_V) * 100u >=
           pack_V * 95u;
}

[[nodiscard]] inline Output step(const Inputs& in) noexcept {
    // Sticky ERROR. Once latched, only a reset can clear (the safety
    // task writes the backup-register magic; App_InitTask reads it
    // and seeds State::Error at boot).
    if (in.current == State::Error) {
        return { State::Error, events::safety::ForceError };
    }

    // Any fault from the predicate set forces ERROR + opens AIRs.
    const safety::Inputs pred = {
        in.bms, in.current_sensor, in.vehicle,
        in.force_error_set, in.now_tick,
    };
    if (safety::evaluate_fault(pred)) {
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
        // Hard deadline: if we don't hit precharge target within
        // PrechargeMaxMs, something is wrong (stuck contactor, low
        // pack V, mismeasured DC bus). Force ERROR.
        if (in.now_tick - in.state_entry_tick > config::PrechargeMaxMs) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        if (precharge_target_reached(in.bms, in.vehicle)) {
            return { State::Transition,
                     events::safety::CloseAirP |
                     events::safety::OpenPrecharge };
        }
        return { State::Precharge, 0u };
    }

    case State::Transition: {
        // Hold for TransitionHoldMs and require that the DC bus
        // STAYS at or above the precharge target the entire time --
        // if it slumps (a contactor opens, cap leaks) we go to ERROR.
        if (!precharge_target_reached(in.bms, in.vehicle)) {
            return { State::Error,
                     events::safety::ForceError |
                     events::safety::OpenAirN | events::safety::OpenAirP |
                     events::safety::OpenPrecharge };
        }
        if (in.now_tick - in.state_entry_tick >= config::TransitionHoldMs) {
            // Branch on the mode SafetyTask captured at Start->Precharge.
            // Mode::Undecided here would be a programming error
            // (Transition reached without going through Start); treat
            // as fault to be safe.
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
        return { State::Transition, 0u };
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
