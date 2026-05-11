// SPDX-License-Identifier: proprietary
//
// Plain-data CAN frame used across the FreeRTOS message queues
// (bms_rx_queue, acu_rx_queue, acu_tx_queue -- see docs/ARCHITECTURE.md §4).
//
// Kept deliberately POD so it copies cheaply and can sit in an
// osMessageQueue. Encode/decode of specific frame layouts lives in
// per-service code (bms_service.cpp, vehicle_service.cpp, etc.), not
// here -- this header is just the transport shape.

#pragma once

#include <array>
#include <cstdint>

namespace ams {

enum class CanBus : std::uint8_t {
    Bms = 0,  // FDCAN1 in the legacy code, BMS slaves
    Acu = 1,  // FDCAN2 in the legacy code, accumulator/vehicle bus
};

// Static across the project. Classic CAN payload is 8 bytes; the .ioc
// keeps FrameFormat = FDCAN_FRAME_FD_NO_BRS but we cap payloads at the
// classic size because every CAN partner today (BMS slaves, charger,
// VCU 0x100/0x600) speaks classic.
inline constexpr std::uint8_t kCanFrameMaxData = 8;

struct CanFrame {
    std::uint32_t id;                // 11- or 29-bit, masked at use site
    std::uint8_t  dlc;               // 0..8
    bool          extended;          // true => 29-bit id
    bool          fd;                // currently always false; reserved
    CanBus        bus;
    std::uint32_t timestamp_ms;      // osKernelGetTickCount() at RX
    std::array<std::uint8_t, kCanFrameMaxData> data;
};

static_assert(sizeof(CanFrame) <= 32,
              "CanFrame should fit in a 32-byte queue slot");

}  // namespace ams
