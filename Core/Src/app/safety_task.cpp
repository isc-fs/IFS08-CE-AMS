// SPDX-License-Identifier: proprietary

#include "safety_task.hpp"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "relay_driver.hpp"
#include "safety_predicates.hpp"
#include "vehicle_service.hpp"
#include "watchdog.hpp"

#include "cmsis_os2.h"
#include "main.h"

// FreeRTOS handle declared in CubeMX-generated main.c. We need a fresh
// extern here because main.c only exposes it via the .data section.
extern "C" osEventFlagsId_t safety_eventsHandle;

namespace ams {

// Forward declaration; definition at the bottom of the TU. Keeps the
// CMSIS-RTOS API out of safety_task.hpp.
static void osDelayUntilMs_(std::uint32_t *prev_tick, std::uint32_t period_ms) noexcept;

SafetyTask& SafetyTask::instance() noexcept {
    static SafetyTask kInstance;
    return kInstance;
}

void SafetyTask::latch_error_() noexcept {
    Relays::open_all();
    ErrorLatch::set();
    error_latched_ = true;
}

void SafetyTask::run() noexcept {
    // If the previous boot latched ERROR (via backup register), come up
    // already in the latched state. App_InitTask is responsible for
    // calling ErrorLatch::init() before this task gets the CPU; that
    // call is idempotent so calling it again here costs nothing and
    // hardens against a future reordering of init.
    ErrorLatch::init();
    if (ErrorLatch::is_set()) {
        latch_error_();
    }

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        // Periodic wake at 10 ms relative to the previous wake.
        // osDelayUntil takes a pointer so it can update last_wake to
        // the next deadline; no drift.
        osDelayUntilMs_(&last_wake, ams::config::kSafetyPeriodMs);

        // Drain pending requests / signals from the FSM. Non-blocking.
        const std::uint32_t evt = osEventFlagsWait(
            safety_eventsHandle,
            ams::events::safety::kAllRequest,
            osFlagsWaitAny | osFlagsNoClear,
            0);

        // osEventFlagsWait returns either the matching flag bits or a
        // negative osStatus_t cast to uint32_t (high bit set). The
        // osFlagsError mask (0x80000000) distinguishes the two.
        const bool flagged_error =
            ((evt & osFlagsError) == 0u) &&
            ((evt & ams::events::safety::kForceError) != 0u);

        // Snapshot every input domain at once; then evaluate.
        const auto bms_snap     = BmsService::instance().snapshot();
        const auto current_snap = CurrentService::instance().snapshot();
        const auto vehicle_snap = VehicleService::instance().snapshot();
        const bool sdc_closed   =
            HAL_GPIO_ReadPin(DIGITAL1_GPIO_Port, DIGITAL1_Pin) == GPIO_PIN_SET;

        const safety::Inputs ev = {
            bms_snap, current_snap, vehicle_snap,
            sdc_closed,
            flagged_error,
            osKernelGetTickCount(),
        };

        const bool fault = error_latched_ || safety::evaluate_fault(ev);

        if (fault) {
            latch_error_();
            // Deliberately NOT refreshing the watchdog -- HW reset
            // within one IWDG window (<= ~100 ms) brings us back up
            // with relays open per the MX_GPIO_Init contract.
            continue;
        }

        // Service FSM-driven relay actions on the clean path. We
        // consume the relay-action bits here (clearing them on read)
        // so each request gets actioned exactly once; the FSM is
        // free to post the same bit on the next 20 ms cycle and we
        // will pick it up again.
        const std::uint32_t actions = osEventFlagsWait(
            safety_eventsHandle,
            ams::events::safety::kAllRelayActions,
            osFlagsWaitAny,
            0);
        if ((actions & osFlagsError) == 0u) {
            if (actions & ams::events::safety::kCloseAirN)      Relays::close_air_negative();
            if (actions & ams::events::safety::kCloseAirP)      Relays::close_air_positive();
            if (actions & ams::events::safety::kClosePrecharge) Relays::close_precharge();
            if (actions & ams::events::safety::kOpenAirN)       Relays::open_air_negative();
            if (actions & ams::events::safety::kOpenAirP)       Relays::open_air_positive();
            if (actions & ams::events::safety::kOpenPrecharge)  Relays::open_precharge();
        }

        Watchdog::refresh();
    }
}

// Helper: wraps osDelayUntil, exposing it as a *_Ms_() name to make the
// units obvious at call sites and to absorb the (pointer-vs-value)
// awkwardness of the CMSIS-RTOS API. Body in the cpp to keep cmsis_os2
// out of the header.
static void osDelayUntilMs_(std::uint32_t *prev_tick, std::uint32_t period_ms) noexcept {
    *prev_tick += period_ms;  // ticks == ms at configTICK_RATE_HZ = 1000
    osDelayUntil(*prev_tick);
}

}  // namespace ams

extern "C" void ams_safety_task_run(void *argument) {
    (void)argument;
    ams::SafetyTask::instance().run();
}
