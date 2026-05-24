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

// The wire layout lives in the C header so main.c (CubeMX-generated, C)
// can do sizeof(CanFrame) when constructing the FreeRTOS message queues.
// We re-export the same struct into the ams:: namespace for the C++ side.
#include "app/can_frame.h"

#include <cstdint>

namespace ams {

// Per docs/CAN_MAP.md and the legacy AMS firmware: FDCAN1 is the
// accumulator/vehicle bus, FDCAN2 is the BMS slave bus. The numeric
// values match the wire transport byte in CanFrame::bus.
enum class CanBus : std::uint8_t {
    Acu = 0,  // FDCAN1 -- accumulator / vehicle / charger
    Bms = 1,  // FDCAN2 -- BMS slave modules
};

inline constexpr std::uint8_t CanFrameMaxData = AMS_CAN_FRAME_MAX_DATA;

// Same memory layout as the C struct -- this is just a namespaced alias.
using CanFrame = ::CanFrame;

}  // namespace ams
