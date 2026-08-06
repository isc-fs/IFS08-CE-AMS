// SPDX-License-Identifier: proprietary
//
// Plain-data CAN frame used across the FreeRTOS message queues
// (acu_rx_queue, acu_tx_queue -- see docs/ARCHITECTURE.md §4).
// `bms_rx_queue` was retired in v1.2.0 when the BMS data path
// moved off FDCAN2 onto LTC6811-1 isoSPI.
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

// Two buses on the carrier. Post-v1.2.0 the app only reads
// and writes Acu (FDCAN1); the second value remains as a "wrong bus"
// sentinel for dispatch-rejection tests and any future second-bus
// consumer. The numeric values match the wire transport byte in
// CanFrame::bus. The enum-value name is kept for source compatibility
// with existing unit tests; semantically FDCAN2 is no longer the BMS
// bus -- the bootloader claims it after a magic-reset jump for the
// flash workflow, and the app never starts FDCAN2 nor enqueues
// frames with bus = CanBus::Bms in production code paths.
enum class CanBus : std::uint8_t {
    Acu = 0,  // FDCAN1 -- accumulator / vehicle / telemetry
    Bms = 1,  // FDCAN2 -- bootloader-only post-v1.2.0 (no app traffic)
};

inline constexpr std::uint8_t CanFrameMaxData = AMS_CAN_FRAME_MAX_DATA;

// Same memory layout as the C struct -- this is just a namespaced alias.
using CanFrame = ::CanFrame;

}  // namespace ams
