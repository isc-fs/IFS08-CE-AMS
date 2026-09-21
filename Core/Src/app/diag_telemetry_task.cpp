// SPDX-License-Identifier: proprietary
//
// diag_telemetry_task.cpp -- Dedicated RTOS task streaming full battery pack
// array diagnostics and cell matrices over FDCAN1.
//
// Streams:
//   0x4A3: AMS_diag_status  (100 ms, active fault latch registers)
//   0x4B0: AMS_diag_cell_v_mux (all 95 cell voltages, 5 mod x 7 chunks)
//   0x4B1: AMS_diag_cell_t_mux (all 190 NTC temperatures, 5 mod x 7 chunks, 0x80 pad)
//
// Priority: osPriorityBelowNormal (strictly lower than SafetyTask and BmsPollTask).
// Non-blocking: lock-free BmsService::snapshot() + non-blocking FDCAN1 TX.
// Paced: 1..2 ms inter-frame pacing to prevent TX FIFO/mailbox saturation.

#include "app/diag_telemetry_task.h"

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "error_latch.hpp"
#include "safety_predicates.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
extern volatile std::uint8_t g_fault_reason_telemetry;
extern volatile std::uint8_t g_fault_detail_telemetry;
}

namespace {

inline void put_le16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

// Non-blocking frame transmit: posts to FDCAN1 TX FIFO.
// If the hardware FIFO is full, returns false immediately without blocking.
bool send_diag_frame(std::uint32_t id, const std::uint8_t data[8]) noexcept {
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

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx,
               const_cast<std::uint8_t*>(data)) == HAL_OK;
}

osThreadId_t diagTelemetryTaskHandle = nullptr;
const osThreadAttr_t diagTelemetryTask_attributes = {
    .name = "DiagTelemTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = static_cast<osPriority_t>(osPriorityBelowNormal),
    .tz_module = 0,
    .reserved = 0
};

}  // namespace

extern "C" void DiagTelemetryTask_Init(void) {
    if (diagTelemetryTaskHandle == nullptr) {
        diagTelemetryTaskHandle = osThreadNew(StartDiagTelemetryTask, nullptr,
                                              &diagTelemetryTask_attributes);
    }
}

extern "C" void StartDiagTelemetryTask(void *argument) {
    (void)argument;
    std::uint8_t roll_counter = 0;

    for (;;) {
        const std::uint32_t loop_start_tick = osKernelGetTickCount();

        // 1. Lock-free snapshot of BMS state (zero mutex contention, zero priority inversion)
        const auto bms = ams::BmsService::instance().snapshot();

        // 2. Transmit 0x4A3 (AMS_diag_status, 8 bytes)
        {
            std::uint8_t d[8] = {};
            const auto reason = static_cast<ams::safety::FaultReason>(g_fault_reason_telemetry);
            d[0] = g_fault_reason_telemetry;

            std::uint8_t  fault_mod      = 0xFFu;
            std::uint8_t  fault_cell_ntc = 0xFFu;
            std::uint16_t tripped_val    = 0u;

            if (reason == ams::safety::FaultReason::CellUnderVoltage) {
                fault_mod = g_fault_detail_telemetry < ams::config::BmsModuleCount
                                ? g_fault_detail_telemetry
                                : 0u;
                std::uint16_t min_v = 0xFFFFu;
                for (std::uint8_t c = 0; c < ams::config::CellsPerModule; ++c) {
                    if (bms.cell_mV[fault_mod][c] < min_v) {
                        min_v = bms.cell_mV[fault_mod][c];
                        fault_cell_ntc = c;
                    }
                }
                tripped_val = min_v;
            } else if (reason == ams::safety::FaultReason::CellOverVoltage) {
                fault_mod = g_fault_detail_telemetry < ams::config::BmsModuleCount
                                ? g_fault_detail_telemetry
                                : 0u;
                std::uint16_t max_v = 0u;
                for (std::uint8_t c = 0; c < ams::config::CellsPerModule; ++c) {
                    if (bms.cell_mV[fault_mod][c] > max_v) {
                        max_v = bms.cell_mV[fault_mod][c];
                        fault_cell_ntc = c;
                    }
                }
                tripped_val = max_v;
            } else if (reason == ams::safety::FaultReason::CellOverTemp) {
                fault_mod = g_fault_detail_telemetry < ams::config::BmsModuleCount
                                ? g_fault_detail_telemetry
                                : 0u;
                std::int16_t max_t = -128;
                for (std::uint8_t n = 0; n < ams::config::TempsPerModule && n < 38; ++n) {
                    if (bms.cell_tempC[fault_mod][n] > max_t) {
                        max_t = bms.cell_tempC[fault_mod][n];
                        fault_cell_ntc = n;
                    }
                }
                tripped_val = static_cast<std::uint16_t>(max_t);
            } else if (reason == ams::safety::FaultReason::CellUnderTemp) {
                fault_mod = g_fault_detail_telemetry < ams::config::BmsModuleCount
                                ? g_fault_detail_telemetry
                                : 0u;
                std::int16_t min_t = 127;
                for (std::uint8_t n = 0; n < ams::config::TempsPerModule && n < 38; ++n) {
                    if (bms.cell_tempC[fault_mod][n] < min_t) {
                        min_t = bms.cell_tempC[fault_mod][n];
                        fault_cell_ntc = n;
                    }
                }
                tripped_val = static_cast<std::uint16_t>(min_t);
            } else {
                fault_mod      = g_fault_detail_telemetry;
                fault_cell_ntc = 0xFFu;
                tripped_val    = 0u;
            }

            d[1] = fault_mod;
            d[2] = fault_cell_ntc;

            // Error bitfield: bit 0 = latch set, bits 1..5 = temp_disconnect,
            // bits 6..10 = cell_open, bits 11..15 = tap_fault
            const std::uint16_t err_bits = static_cast<std::uint16_t>(
                (ams::ErrorLatch::is_set() ? 1u : 0u) |
                ((static_cast<std::uint16_t>(bms.temp_disconnect_mask) & 0x1Fu) << 1) |
                ((static_cast<std::uint16_t>(bms.cell_open_mask) & 0x1Fu) << 6) |
                ((static_cast<std::uint16_t>(bms.tap_fault_mask) & 0x1Fu) << 11));

            put_le16(&d[3], err_bits);
            put_le16(&d[5], tripped_val);
            d[7] = roll_counter++;

            (void)send_diag_frame(ams::config::AmsDiagStatusId, d);
            osDelay(ams::config::DiagTelemPaceMs);
        }

        // 3. Transmit 0x4B0 (AMS_diag_cell_v_mux): 5 modules x 7 chunks = 35 frames
        // Chunk carrying: 3 cells each (LE uint16_t), chunk 6 carries cell 18 (with padding 0xFFFF)
        for (std::uint8_t m = 0; m < ams::config::BmsModuleCount; ++m) {
            for (std::uint8_t chunk = 0; chunk < 7; ++chunk) {
                std::uint8_t d[8] = {};
                d[0] = m;
                d[1] = chunk;

                const std::uint8_t c0 = static_cast<std::uint8_t>(chunk * 3 + 0);
                const std::uint8_t c1 = static_cast<std::uint8_t>(chunk * 3 + 1);
                const std::uint8_t c2 = static_cast<std::uint8_t>(chunk * 3 + 2);

                const std::uint16_t v0 = (c0 < ams::config::CellsPerModule) ? bms.cell_mV[m][c0] : 0xFFFFu;
                const std::uint16_t v1 = (c1 < ams::config::CellsPerModule) ? bms.cell_mV[m][c1] : 0xFFFFu;
                const std::uint16_t v2 = (c2 < ams::config::CellsPerModule) ? bms.cell_mV[m][c2] : 0xFFFFu;

                put_le16(&d[2], v0);
                put_le16(&d[4], v1);
                put_le16(&d[6], v2);

                (void)send_diag_frame(ams::config::AmsDiagCellVMuxId, d);
                osDelay(ams::config::DiagTelemPaceMs);
            }
        }

        // 4. Transmit 0x4B1 (AMS_diag_cell_t_mux): 5 modules x 7 chunks = 35 frames
        // Chunk carrying: 6 NTC readings (int8_t °C), chunk 6 carries NTC 36..37 with pad 0x80
        for (std::uint8_t m = 0; m < ams::config::BmsModuleCount; ++m) {
            for (std::uint8_t chunk = 0; chunk < 7; ++chunk) {
                std::uint8_t d[8] = {};
                d[0] = m;
                d[1] = chunk;

                for (std::uint8_t i = 0; i < 6; ++i) {
                    const std::uint8_t ntc_idx = static_cast<std::uint8_t>(chunk * 6 + i);
                    if (ntc_idx < 38) {
                        d[2 + i] = static_cast<std::uint8_t>(bms.cell_tempC[m][ntc_idx]);
                    } else {
                        d[2 + i] = 0x80u;  // pad marker
                    }
                }

                (void)send_diag_frame(ams::config::AmsDiagCellTMuxId, d);
                osDelay(ams::config::DiagTelemPaceMs);
            }
        }

        // 5. Cadence pacing (100..200 ms total cycle)
        const std::uint32_t elapsed = osKernelGetTickCount() - loop_start_tick;
        if (elapsed < ams::config::DiagTelemCadenceMs) {
            osDelay(ams::config::DiagTelemCadenceMs - elapsed);
        } else {
            osDelay(1);
        }
    }
}
