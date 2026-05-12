// SPDX-License-Identifier: proprietary
//
// Drains the BMS RX queue and feeds the BmsService.
//
// Producer: HAL_FDCAN_RxFifo0Callback in can_isr.cpp (FDCAN2 branch).
// Consumer: this task. Single-writer to BmsService::update_from_frame,
// satisfying the ownership map in docs/ARCHITECTURE.md §3.
//
// Period: event-driven. We block on osMessageQueueGet with
// osWaitForever, so the task burns zero CPU until a frame lands.

#include "app/bms_rx_task.h"

#include "bms_service.hpp"
#include "bootloader.hpp"
#include "can_frame.hpp"

#include "cmsis_os2.h"

extern "C" osMessageQueueId_t bms_rx_queueHandle;

namespace {

// Telemeters per-frame parse failures and unknown IDs. Surfaced over
// UART once the telemetry task gets a real implementation in feat/16.
// File-scope so a remote-debug session can read them via the symbol.
volatile std::uint32_t g_bms_rx_dropped_unknown = 0;
volatile std::uint32_t g_bms_rx_dropped_parse   = 0;

}  // namespace

extern "C" void ams_bms_rx_task_run(void *argument) {
    (void)argument;

    ams::CanFrame frame;

    for (;;) {
        if (osMessageQueueGet(bms_rx_queueHandle, &frame, nullptr,
                              osWaitForever) != osOK) {
            // Spurious wake / queue deleted -- back off briefly and
            // try again rather than spinning.
            osDelay(10);
            continue;
        }

        // Boot-trigger check happens BEFORE BmsService parsing.
        // request_reboot() never returns -- it opens the relays,
        // writes BL_BOOT_REQ_MAGIC to BKP0R, and issues a HW reset.
        if (ams::Bootloader::matches_trigger(frame)) {
            ams::Bootloader::request_reboot();
        }

        if (!ams::BmsService::instance().update_from_frame(frame)) {
            // The frame didn't match a known voltage/temperature ID
            // for any of the 5 modules. Most likely a foreign frame
            // on the BMS bus or a corrupted ID; count it and move on.
            ++g_bms_rx_dropped_unknown;
        }
    }
}
