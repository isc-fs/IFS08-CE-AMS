// SPDX-License-Identifier: proprietary

#include "vehicle_service.hpp"

#include "ams_config.hpp"

#include <cstdint>

#if defined(AMS_BMS_HIL_STUB)
// #123 ACU RX dispatch probe. Surfaced in 0x4A2[4] via safety_task.
// Ticks on any matched ACU frame; useful as a "ACU bus is alive AND
// AcuCanTask is draining the queue AND VehicleService is dispatching"
// liveness counter. The start-button counter from fix/47 retired here
// along with the 0x600 path itself.
extern "C" volatile std::uint32_t g_acu_rx_total = 0;
#endif

namespace ams {

VehicleService& VehicleService::instance() noexcept {
    static VehicleService kInstance;
    return kInstance;
}

std::uint16_t VehicleService::decode_dc_bus_V(const std::uint8_t *data) noexcept {
    // Legacy: DC_BUS = (buf[1] << 8) | buf[0]   (little-endian)
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[1]) << 8) |
         static_cast<std::uint16_t>(data[0]));
}

bool VehicleService::update_from_frame(const CanFrame& f) noexcept {
    if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;

#if defined(AMS_BMS_HIL_STUB)
    ++g_acu_rx_total;
#endif

    if (f.id == config::kAcuRxDcBusId) {
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
