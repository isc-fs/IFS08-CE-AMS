// SPDX-License-Identifier: proprietary
//
// Plain-data CAN frame used across the FreeRTOS message queues
// (acu_rx_queue, acu_tx_queue -- see docs/ARCHITECTURE.md §4). The BMS does
// NOT arrive over CAN: cell data comes in over LTC6811-1 isoSPI, so nothing
// here carries it.
//
// Kept deliberately POD so it copies cheaply and can sit in an
// osMessageQueue. Encode/decode of specific frame layouts lives in
// per-service code (vehicle_service.cpp, telemetry_encoders.hpp,
// acu_tx_encoders.hpp, etc.), not here -- this header is just the
// transport shape.

#pragma once

// The wire layout lives in the C header so main.c (CubeMX-generated, C)
// can do sizeof(CanFrame) when constructing the FreeRTOS message queues.
// We re-export the same struct into the ams:: namespace for the C++ side.
#include "app/can_frame.h"

#include <cstdint>

namespace ams {

// Two buses on the carrier. The app only reads and writes Acu (FDCAN1); the
// second value exists as a "wrong bus" sentinel for dispatch-rejection tests
// and any future second-bus consumer. The numeric values match the wire
// transport byte in CanFrame::bus.
//
// Do not read the second enumerator's name as a statement of ownership: the
// app never starts FDCAN2 nor enqueues
// frames with bus = CanBus::Bms in production code paths.
enum class CanBus : std::uint8_t {
    Acu = 0,  // FDCAN1 -- accumulator / vehicle / telemetry
    Bms = 1,  // FDCAN2 -- bootloader-only post-v1.2.0 (no app traffic)
};

inline constexpr std::uint8_t CanFrameMaxData = AMS_CAN_FRAME_MAX_DATA;

// Same memory layout as the C struct -- this is just a namespaced alias.
using CanFrame = ::CanFrame;

}  // namespace ams
