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

enum class CanBus : std::uint8_t {
    Bms = 0,  // FDCAN1 in the legacy code, BMS slaves
    Acu = 1,  // FDCAN2 in the legacy code, accumulator/vehicle bus
};

inline constexpr std::uint8_t kCanFrameMaxData = AMS_CAN_FRAME_MAX_DATA;

// Same memory layout as the C struct -- this is just a namespaced alias.
using CanFrame = ::CanFrame;

}  // namespace ams
