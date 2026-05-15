// SPDX-License-Identifier: proprietary

#include "app/state_task.h"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "fan.hpp"
#include "state_machine.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" osEventFlagsId_t safety_eventsHandle;

// Exported (C linkage) so TelemetryTask can read the FSM state without
// pulling state_task.cpp's internals in. Updated on every transition.
extern "C" volatile std::uint8_t g_state_telemetry = 0;

namespace {

// Per-state fan duty mirroring the legacy 40 % run / 75 % charge,
// off in transitional / error states.
constexpr std::uint8_t kFanDuty[6] = {
    /*Start*/      0,
    /*Precharge*/  0,
    /*Transition*/ 0,
    /*Run*/       40,
    /*Charge*/    75,
    /*Error*/      0,
};

}  // namespace

extern "C" void ams_state_task_run(void *argument) {
    (void)argument;

    // Bring up the fan PWM (idempotent; safe even if App_InitTask has
    // already done it).
    ams::Fan::init();

    // Boot in ERROR if the previous run latched it.
    ams::fsm::State state =
        ams::ErrorLatch::is_set() ? ams::fsm::State::Error : ams::fsm::State::Start;

    ams::Fan::set_duty_pct(kFanDuty[static_cast<std::uint8_t>(state)]);

    std::uint32_t last_wake        = osKernelGetTickCount();
    std::uint32_t state_entry_tick = last_wake;

    for (;;) {
        last_wake += ams::config::kStatePeriodMs;
        osDelayUntil(last_wake);

        const auto bms  = ams::BmsService::instance().snapshot();
        const auto cur  = ams::CurrentService::instance().snapshot();
        const auto veh  = ams::VehicleService::instance().snapshot();

        // SDC line not GPIO-sensed -- see comment in safety_task.cpp.
        const ams::fsm::Inputs in = {
            state, bms, cur, veh,
            /*force_error_set=*/false,
            osKernelGetTickCount(),
            state_entry_tick,
        };

        const auto out = ams::fsm::step(in);

        if (out.safety_flags != 0u) {
            osEventFlagsSet(safety_eventsHandle, out.safety_flags);
        }

        if (out.next != state) {
            state            = out.next;
            state_entry_tick = osKernelGetTickCount();
            g_state_telemetry = static_cast<std::uint8_t>(state);
            ams::Fan::set_duty_pct(
                kFanDuty[static_cast<std::uint8_t>(state)]);

            // Persist ERROR across resets. SafetyTask also writes the
            // latch on the fault path; we duplicate here so the latch
            // gets set even if FSM forces ERROR without a predicate
            // fault (e.g. precharge timeout).
            if (state == ams::fsm::State::Error) {
                ams::ErrorLatch::set();
            }
        }
    }
}
