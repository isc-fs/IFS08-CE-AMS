// SPDX-License-Identifier: proprietary
//
// Accumulator-bus task (FDCAN1).
//
// Today (feat/12) the task only drains acu_rx_queue and hands each
// frame to VehicleService for parsing. The 100 ms TX heartbeat (pack
// status, current, min cell V, fault word) lands in feat/13.
//
// Producer of acu_rx_queue: HAL_FDCAN_RxFifo0Callback on hfdcan1
// (see can_isr.cpp).

#include "app/acu_can_task.h"

#include "can_frame.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"

extern "C" osMessageQueueId_t acu_rx_queueHandle;

namespace {

// Telemetry counter for frames on the accumulator bus that didn't
// match any known ID. Surfaced over UART once TelemetryTask is real.
volatile std::uint32_t g_acu_rx_dropped_unknown = 0;

}  // namespace

extern "C" void ams_acu_can_task_run(void *argument) {
    (void)argument;

    ams::CanFrame frame;

    for (;;) {
        if (osMessageQueueGet(acu_rx_queueHandle, &frame, nullptr,
                              osWaitForever) != osOK) {
            osDelay(10);  // queue deleted or spurious wake
            continue;
        }

        if (!ams::VehicleService::instance().update_from_frame(frame)) {
            ++g_acu_rx_dropped_unknown;
        }
    }
}
