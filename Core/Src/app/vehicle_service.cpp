// SPDX-License-Identifier: proprietary

#include "vehicle_service.hpp"

#include "ams_config.hpp"

#include <cstdint>

namespace ams {

VehicleService& VehicleService::instance() noexcept {
    static VehicleService Instance;
    return Instance;
}

std::uint16_t VehicleService::decode_dc_bus_V(const std::uint8_t *data) noexcept {
    // Legacy: DC_BUS = (buf[1] << 8) | buf[0]   (little-endian)
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[1]) << 8) |
         static_cast<std::uint16_t>(data[0]));
}

bool VehicleService::update_from_frame(const CanFrame& f) noexcept {
    if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;

    if (f.id == config::AcuRxDcBusId) {
        if (f.dlc < 2) return false;
        state_.dc_bus_V         = decode_dc_bus_V(f.data);
        state_.last_dc_bus_tick = f.timestamp_ms;
        return true;
    }
    return false;
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ams
