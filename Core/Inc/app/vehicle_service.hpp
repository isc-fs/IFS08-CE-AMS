// SPDX-License-Identifier: proprietary
//
// State the AMS receives from the wider vehicle on FDCAN1 (the
// "accumulator bus"). Currently a single signal: VCU DC-bus voltage
// heartbeat (0x100, ext). The 0x600 start-button and 0x18FF50E7
// charger-detect frames were retired in fix/48 -- their roles are now
// owned by the TSMS_Pin / DASH_CHG_Pin GPIOs read directly by
// SafetyTask. last_dc_bus_tick is load-bearing: SafetyTask uses its
// freshness against kVcuFreshMs to decide Car-vs-Charger mode at the
// moment of Start->Precharge.
//
// Single-writer: AcuCanTask. Many readers: MainTask.
// Synchronisation: docs/ARCHITECTURE.md §7.

#pragma once

#include "can_frame.hpp"

#include <cstdint>

namespace ams {

struct VehicleState {
    std::uint16_t dc_bus_V;          // from 0x100, little-endian bytes 0..1
    std::uint32_t last_dc_bus_tick;
};

class VehicleService {
public:
    static VehicleService& instance() noexcept;

    // Returns true if the frame matched a known accumulator-bus ID
    // and the state was updated. False -> caller may telemeter the drop.
    bool update_from_frame(const CanFrame& f) noexcept;

    [[nodiscard]] VehicleState snapshot() const noexcept;

    // Pure helper, public for unit testing.
    static std::uint16_t decode_dc_bus_V(const std::uint8_t *data) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ams
