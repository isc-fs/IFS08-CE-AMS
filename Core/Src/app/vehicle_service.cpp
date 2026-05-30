// SPDX-License-Identifier: proprietary

#include "vehicle_service.hpp"

#include "ams_config.hpp"

#include <cstddef>
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

    // Operator charge-mode request (#305). Magic-gated so bus noise / a
    // stray frame can't flip the AMS into a HV charge mode. Only a frame
    // whose payload matches ChargeModeReqMagic advances the freshness
    // tick that SafetyTask reads at the Start->Precharge mode lock.
    if (f.id == config::ChargeModeReqId) {
        if (f.dlc < config::ChargeModeReqDlc) return false;
        for (std::size_t i = 0; i < config::ChargeModeReqDlc; ++i) {
            if (f.data[i] != config::ChargeModeReqMagic[i]) return false;
        }
        state_.last_charge_req_tick = f.timestamp_ms;
        return true;
    }
    return false;
}

bool VehicleService::charge_requested(std::uint32_t now_tick,
                                      std::uint32_t last_req_tick) noexcept {
    if (last_req_tick == 0u) return false;
    // Future-tick safe (the #276 lesson): a request stamped slightly
    // ahead of `now` counts as just-seen, never as ancient.
    const std::uint32_t age =
        (now_tick >= last_req_tick) ? (now_tick - last_req_tick) : 0u;
    return age <= config::ChargeReqFreshMs;
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ams
