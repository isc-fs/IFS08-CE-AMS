// SPDX-License-Identifier: proprietary
//
// State the AMS receives from the wider vehicle on FDCAN1 (the
// "accumulator bus"). Today: DC bus voltage (0x100), start-button
// (0x600), and (indirectly via CurrentService) charger detection
// (0x18FF50E7).
//
// Single-writer: AcuCanTask. Many readers: SafetyTask, StateTask,
// TelemetryTask. Synchronisation: docs/ARCHITECTURE.md §3.

#pragma once

#include "can_frame.hpp"

#include <cstdint>

namespace ams {

struct VehicleState {
    std::uint16_t dc_bus_V;          // from 0x100, little-endian bytes 0..1
    std::uint8_t  start_button;      // from 0x600 byte 0 (0/1)
    std::uint32_t last_dc_bus_tick;
    std::uint32_t last_start_btn_tick;
};

class VehicleService {
public:
    static VehicleService& instance() noexcept;

    // Returns true if the frame matched a known accumulator-bus ID
    // and the state was updated (or, for charger detect, forwarded
    // to CurrentService). False -> caller may telemeter the drop.
    bool update_from_frame(const CanFrame& f) noexcept;

    [[nodiscard]] VehicleState snapshot() const noexcept;

    // Pure helpers, public for unit testing.
    static std::uint16_t decode_dc_bus_V(const std::uint8_t *data) noexcept;
    static std::uint8_t  decode_start_button(const std::uint8_t *data) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ams
