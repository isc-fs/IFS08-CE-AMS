// SPDX-License-Identifier: proprietary
//
// FDCAN1 telemetry @ 500 ms. Three 8-byte frames per cycle:
//
//   0x4A0  AMS status  (FSM state, AMS_OK, module mask, min/max cell V)
//   0x4A1  AMS pack    (pack mV, filtered mA)
//   0x4A2  AMS temps   (min/max/avg tempC, dc_bus_V, heartbeat)
//
// Pure-logic encode lives in app/telemetry_encoders.hpp so unit tests
// exercise it on the host. This file is the thin task wrapper that
// pulls service snapshots, encodes, and pushes via HAL_FDCAN.

#include "app/telemetry_task.h"

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "telemetry_encoders.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;

// Exposed by state_task.cpp.
extern volatile std::uint8_t g_state_telemetry;
}

namespace {

// Telemetry counters, surfaced via a future diag frame.
volatile std::uint32_t g_telemetry_tx_fail = 0;

bool send_telem(std::uint32_t id, const ams::telemetry::Frame& payload) {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    // HAL_FDCAN_AddMessageToTxFifoQ takes a non-const data pointer
    // even though it doesn't mutate the bytes. const_cast is safe.
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx,
               const_cast<std::uint8_t*>(payload.data())) == HAL_OK;
}

}  // namespace

extern "C" void ams_telemetry_task_run(void *argument) {
    (void)argument;

    std::uint32_t last_wake = osKernelGetTickCount();
    std::uint8_t  heartbeat = 0;

    for (;;) {
        last_wake += ams::config::kTelemetryPeriodMs;
        osDelayUntil(last_wake);

        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto veh = ams::VehicleService::instance().snapshot();

        const std::uint8_t ams_ok =
            (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET)
                ? 1u : 0u;

        const auto frame_status = ams::telemetry::encode_status(
            g_state_telemetry, ams_ok, bms);
        const auto frame_pack   = ams::telemetry::encode_pack(bms, cur);
        const auto frame_temps  = ams::telemetry::encode_temps(
            bms, veh, heartbeat);

        if (!send_telem(ams::config::kAmsTelemStatusId, frame_status)) ++g_telemetry_tx_fail;
        if (!send_telem(ams::config::kAmsTelemPackId,   frame_pack))   ++g_telemetry_tx_fail;
        if (!send_telem(ams::config::kAmsTelemTempsId,  frame_temps))  ++g_telemetry_tx_fail;

        ++heartbeat;  // 8-bit wraparound is intentional
    }
}
