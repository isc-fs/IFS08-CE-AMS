// SPDX-License-Identifier: proprietary
//
// Accumulator-bus task (FDCAN1). Two responsibilities:
//
//  RX (always): drain acu_rx_queue -> VehicleService::update_from_frame.
//  TX (periodic):
//    - 0x12C extended, 500 ms, big-endian min cell mV
//      (suppressed when CurrentService says charger is attached --
//      mirrors the legacy "no balancing during charge" rule).
//    - 0x450 standard, 250 ms, byte 0 = 0x00, byte 1 = filtered current A
//      (low byte only, matches legacy 0x450 wire format -- the wider
//      16-bit signed mA frame is a future protocol change documented
//      in docs/CAN_MAP.md refactor checklist).
//
// Cadence is driven by osMessageQueueGet with a computed timeout that
// expires at the nearest TX deadline, so RX latency stays low and TX
// jitter stays bounded.

#include "app/acu_can_task.h"

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "bootloader.hpp"
#include "can_frame.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <algorithm>
#include <cstdlib>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
}

extern "C" osMessageQueueId_t acu_rx_queueHandle;

namespace {

// Telemetry counters; volatile so a remote-debug session can read.
volatile std::uint32_t g_acu_rx_dropped_unknown = 0;
volatile std::uint32_t g_acu_tx_fail            = 0;

constexpr std::uint32_t kMinVoltagePeriodMs = 500;
constexpr std::uint32_t kCurrentPeriodMs    = 250;

bool send_acu(std::uint32_t id, bool extended, std::uint8_t dlc,
              const std::uint8_t *data) {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = static_cast<std::uint32_t>(dlc) << 16;  // FDCAN DLC field
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx,
                                         const_cast<std::uint8_t *>(data)) == HAL_OK;
}

void tx_min_voltage(const ams::BmsState& bms) {
    const std::uint8_t data[2] = {
        static_cast<std::uint8_t>(bms.min_cell_mV >> 8),
        static_cast<std::uint8_t>(bms.min_cell_mV & 0xFF),
    };
    if (!send_acu(ams::config::kAcuTxMinVoltId, /*extended=*/true, 2, data)) {
        ++g_acu_tx_fail;
    }
}

void tx_current(const ams::CurrentState& cur) {
    // Legacy 0x450 wire format: byte 0 = 0x00, byte 1 = |A| low byte.
    // Sign is lost on the wire; the new signed 16-bit mA frame is a
    // future protocol change (CAN_MAP.md refactor checklist).
    const std::int32_t  amps    = cur.filtered_mA / 1000;
    const std::uint8_t  amps_lb = static_cast<std::uint8_t>(std::abs(amps) & 0xFF);
    const std::uint8_t  data[2] = { 0x00, amps_lb };
    if (!send_acu(ams::config::kAcuTxCurrentId, /*extended=*/false, 2, data)) {
        ++g_acu_tx_fail;
    }
}

}  // namespace

extern "C" void ams_acu_can_task_run(void *argument) {
    (void)argument;

    ams::CanFrame frame;
    std::uint32_t last_minv_tx = osKernelGetTickCount();
    std::uint32_t last_curr_tx = last_minv_tx;

    for (;;) {
        const std::uint32_t now = osKernelGetTickCount();
        const std::uint32_t next_minv = last_minv_tx + kMinVoltagePeriodMs;
        const std::uint32_t next_curr = last_curr_tx + kCurrentPeriodMs;
        const std::uint32_t deadline  = std::min(next_minv, next_curr);
        const std::uint32_t timeout   = (deadline > now) ? deadline - now : 0u;

        // Drain at most one RX frame per loop; the next iteration will
        // pick up the next one. Keeps TX cadence within ~1 RX-frame
        // worst-case jitter.
        if (osMessageQueueGet(acu_rx_queueHandle, &frame, nullptr, timeout) == osOK) {
            // Boot-trigger frame moved to FDCAN1 in v1.2.0 (#73). Has
            // to come BEFORE VehicleService so we still react to the
            // reboot request even if the trigger ID accidentally
            // collides with a future VCU frame.
            // request_reboot() never returns -- it opens all relays,
            // writes BL_BOOT_REQ_MAGIC into BKP0R, and resets.
            if (ams::Bootloader::matches_trigger(frame)) {
                ams::Bootloader::request_reboot(
                    ams::config::JumpReason::kCanTrigger);
            }
            if (!ams::VehicleService::instance().update_from_frame(frame)) {
                ++g_acu_rx_dropped_unknown;
            }
        }

        const std::uint32_t now2 = osKernelGetTickCount();
        const auto cur = ams::CurrentService::instance().snapshot();

        if (now2 - last_minv_tx >= kMinVoltagePeriodMs) {
            // Legacy: suppress min-V frame while charger is attached
            // (the VCU side doesn't expect / use it during charging).
            if (!cur.charger_detected) {
                const auto bms = ams::BmsService::instance().snapshot();
                tx_min_voltage(bms);
            }
            last_minv_tx = now2;
        }
        if (now2 - last_curr_tx >= kCurrentPeriodMs) {
            tx_current(cur);
            last_curr_tx = now2;
        }
    }
}
