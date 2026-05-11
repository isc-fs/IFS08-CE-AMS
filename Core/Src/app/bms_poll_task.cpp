// SPDX-License-Identifier: proprietary
//
// Periodically emits the BMS poll-request frames on FDCAN2:
//
//   every 250 ms (kBmsPollVoltMs) -> 5 voltage polls,   id = CANID(m)
//   every 500 ms (kBmsPollTempMs) -> 5 temperature polls, id = CANID(m) + 20
//
// Mechanism: two osTimers drive bms_events (kPollVDue, kPollTDue) bits;
// this task blocks on osEventFlagsWait and emits the polls when the
// corresponding bit fires. Single producer on FDCAN2 TX (BmsPollTask),
// so no TX queue or mutex around the HAL call.
//
// Per docs/ARCHITECTURE.md §2 task table.
// Wire protocol: docs/CAN_MAP.md §"TX -- AMS to BMS slaves".

#include "app/bms_poll_task.h"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "app/app_globals.h"
#include "current_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan2;
}

namespace {

osTimerId_t s_volt_timer = nullptr;
osTimerId_t s_temp_timer = nullptr;

// Telemetry: count TX failures so we surface FDCAN saturation in a
// future telemetry frame. Volatile so a remote-debug session can read.
volatile std::uint32_t g_bms_poll_tx_fail = 0;

void volt_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::kPollVDue);
}

void temp_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::kPollTDue);
}

// Encodes one frame and hands it to the FDCAN2 TX FIFO. The HAL queue
// holds 16 entries (per AMS.ioc); 5 polls per cycle keeps us well
// under depth.
void send_poll(std::uint32_t id,
               std::uint8_t  b0,
               std::uint8_t  b1) {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_2;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    std::uint8_t payload[2] = { b0, b1 };

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, payload) != HAL_OK) {
        ++g_bms_poll_tx_fail;
    }
}

void send_voltage_polls() {
    // Legacy parity: suppress balancing TX while a charger is on the
    // bus. The slaves interpret the 2-byte payload as a balancing
    // target -- we don't want passive balancing to fight the charger's
    // active balance algorithm.
    if (ams::CurrentService::instance().snapshot().charger_detected) {
        return;
    }

    for (std::uint8_t m = 0; m < ams::config::kBmsModuleCount; ++m) {
        const std::uint32_t canid =
            static_cast<std::uint32_t>(ams::config::kBmsVoltPollBase) +
            static_cast<std::uint32_t>(m) * ams::config::kBmsModuleStride;

        send_poll(canid,
                  static_cast<std::uint8_t>(ams::config::kBmsBalancingTargetmV >> 8),
                  static_cast<std::uint8_t>(ams::config::kBmsBalancingTargetmV & 0xFF));
    }
}

void send_temperature_polls() {
    for (std::uint8_t m = 0; m < ams::config::kBmsModuleCount; ++m) {
        const std::uint32_t canid =
            static_cast<std::uint32_t>(ams::config::kBmsVoltPollBase) +
            static_cast<std::uint32_t>(m) * ams::config::kBmsModuleStride +
            static_cast<std::uint32_t>(ams::config::kBmsTempPollOffset);

        send_poll(canid, 0x00, 0x00);
    }
}

}  // namespace

extern "C" void ams_bms_poll_task_run(void *argument) {
    (void)argument;

    // Create and start both periodic timers. Done inside the task so
    // creation runs after osKernelStart and the timer-service daemon
    // is alive.
    s_volt_timer = osTimerNew(&volt_timer_cb, osTimerPeriodic, nullptr, nullptr);
    s_temp_timer = osTimerNew(&temp_timer_cb, osTimerPeriodic, nullptr, nullptr);

    if (s_volt_timer != nullptr) {
        osTimerStart(s_volt_timer, ams::config::kBmsPollVoltMs);
    }
    if (s_temp_timer != nullptr) {
        osTimerStart(s_temp_timer, ams::config::kBmsPollTempMs);
    }

    constexpr std::uint32_t kAll =
        ams::events::bms::kPollVDue | ams::events::bms::kPollTDue;

    for (;;) {
        const std::uint32_t evt = osEventFlagsWait(
            bms_eventsHandle, kAll, osFlagsWaitAny, osWaitForever);

        if ((evt & osFlagsError) != 0u) {
            // Event group went away; back off and keep waiting. The
            // group is statically allocated for the lifetime of the
            // app, so this branch is defensive.
            osDelay(50);
            continue;
        }

        if (evt & ams::events::bms::kPollVDue) send_voltage_polls();
        if (evt & ams::events::bms::kPollTDue) send_temperature_polls();
    }
}
