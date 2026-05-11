// SPDX-License-Identifier: proprietary

#include "app/state_task.h"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "state_machine.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" osEventFlagsId_t safety_eventsHandle;

namespace {

volatile std::uint8_t g_state_telemetry = 0;  // ams::fsm::State as raw byte

}  // namespace

extern "C" void ams_state_task_run(void *argument) {
    (void)argument;

    // Boot in ERROR if the previous run latched it.
    ams::fsm::State state =
        ams::ErrorLatch::is_set() ? ams::fsm::State::Error : ams::fsm::State::Start;

    std::uint32_t last_wake        = osKernelGetTickCount();
    std::uint32_t state_entry_tick = last_wake;

    for (;;) {
        last_wake += ams::config::kStatePeriodMs;
        osDelayUntil(last_wake);

        const auto bms  = ams::BmsService::instance().snapshot();
        const auto cur  = ams::CurrentService::instance().snapshot();
        const auto veh  = ams::VehicleService::instance().snapshot();
        const bool sdc  =
            HAL_GPIO_ReadPin(DIGITAL1_GPIO_Port, DIGITAL1_Pin) == GPIO_PIN_SET;

        const ams::fsm::Inputs in = {
            state, bms, cur, veh, sdc,
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
        }
    }
}
