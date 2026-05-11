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

struct Inputs {
    State               current;
    const BmsState&     bms;
    const CurrentState& current_sensor;
    const VehicleState& vehicle;
    bool                sdc_closed;
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
        return { State::Error, events::safety::kForceError };
    }

    // Any fault from the predicate set forces ERROR + opens AIRs.
    const safety::Inputs pred = {
        in.bms, in.current_sensor, in.vehicle,
        in.sdc_closed, in.force_error_set, in.now_tick,
    };
    if (safety::evaluate_fault(pred)) {
        return { State::Error,
                 events::safety::kForceError |
                 events::safety::kOpenAirN | events::safety::kOpenAirP |
                 events::safety::kOpenPrecharge };
    }

    switch (in.current) {
    case State::Start: {
        // Wait for charger detection, start button, OR a healthy
        // baseline. Charger is the cleanest trigger; absent that,
        // a start_button transitions us into Precharge.
        if (in.current_sensor.charger_detected) {
            return { State::Charge,
                     events::safety::kCloseAirN | events::safety::kCloseAirP };
        }
        if (in.vehicle.start_button != 0u) {
            return { State::Precharge,
                     events::safety::kCloseAirN |
                     events::safety::kClosePrecharge };
        }
        return { State::Start, 0u };
    }

    case State::Precharge: {
        // Hard deadline: if we don't hit precharge target within
        // kPrechargeMaxMs, something is wrong (stuck contactor, low
        // pack V, mismeasured DC bus). Force ERROR.
        if (in.now_tick - in.state_entry_tick > config::kPrechargeMaxMs) {
            return { State::Error,
                     events::safety::kForceError |
                     events::safety::kOpenAirN | events::safety::kOpenAirP |
                     events::safety::kOpenPrecharge };
        }
        if (precharge_target_reached(in.bms, in.vehicle)) {
            return { State::Transition,
                     events::safety::kCloseAirP |
                     events::safety::kOpenPrecharge };
        }
        return { State::Precharge, 0u };
    }

    case State::Transition: {
        // Hold for kTransitionHoldMs and require that the DC bus
        // STAYS at or above the precharge target the entire time --
        // if it slumps (a contactor opens, cap leaks) we go to ERROR.
        if (!precharge_target_reached(in.bms, in.vehicle)) {
            return { State::Error,
                     events::safety::kForceError |
                     events::safety::kOpenAirN | events::safety::kOpenAirP |
                     events::safety::kOpenPrecharge };
        }
        if (in.now_tick - in.state_entry_tick >= config::kTransitionHoldMs) {
            return { State::Run, 0u };
        }
        return { State::Transition, 0u };
    }

    case State::Run: {
        // Run = pack is in the car. Charger detection should be
        // physically impossible here; if it happens it's almost
        // certainly an electrical issue (noise on the charger CAN
        // id, miswired charger). Stay in Run -- the predicate set
        // will catch any genuine current anomaly via |I| > kImaxMa.
        return { State::Run, 0u };
    }

    case State::Charge: {
        // Charge = pack is at the charging station. Run and Charge
        // are mutually exclusive contexts entered ONCE per power
        // cycle from Start; we never transition between them at
        // runtime. Stay here until reset.
        return { State::Charge, 0u };
    }

    case State::Error:
    default:
        // Already handled at function entry; defensive default.
        return { State::Error, events::safety::kForceError };
    }
}

}  // namespace ams::fsm
