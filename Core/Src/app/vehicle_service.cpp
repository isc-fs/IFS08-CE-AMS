// SPDX-License-Identifier: proprietary

#include "vehicle_service.hpp"

#include "ams_config.hpp"
#include "current_service.hpp"



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

std::uint8_t VehicleService::decode_start_button(const std::uint8_t *data) noexcept {
    return data[0] != 0u ? 1u : 0u;
}

bool VehicleService::update_from_frame(const CanFrame& f) noexcept {
    if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;

    switch (f.id) {
    case config::kAcuRxDcBusId: {
        if (f.dlc < 2) return false;
        state_.dc_bus_V         = decode_dc_bus_V(f.data);
        state_.last_dc_bus_tick = f.timestamp_ms;
        return true;
    }

    case config::kAcuRxStartBtnId: {
        if (f.dlc < 1) return false;
        state_.start_button       = decode_start_button(f.data);
        state_.last_start_btn_tick = f.timestamp_ms;
        return true;
    }

    case config::kAcuRxChargerId: {
        // Charger detection forwards to CurrentService (sticky: stays
        // set until either a reset or a future "current near zero for
        // 500 ms" rule, see feat/13+). We don't write VehicleState.
        CurrentService::instance().set_charger_detected(true);
        return true;
    }

    default:
        return false;
    }
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ams
