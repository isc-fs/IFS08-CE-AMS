// SPDX-License-Identifier: proprietary
//
// Pure-logic fault evaluator. Separated out from SafetyTask so it can
// be unit-tested on the host: given the three service snapshots, the
// SDC line state, and the current tick, return true iff the safety
// supervisor should latch ERROR.
//
// Lives in a header so unit tests can pull it without dragging
// FreeRTOS into the build.

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include <cstdlib>

namespace ams::safety {

struct Inputs {
    const BmsState&     bms;
    const CurrentState& current;
    const VehicleState& vehicle;
    bool                sdc_closed;      // PE9 / DIGITAL1 reads HIGH
    bool                force_error_set; // safety_events kForceError pending
    std::uint32_t       now_tick;
};

[[nodiscard]] inline bool evaluate_fault(const Inputs& in) noexcept {
    if (in.force_error_set) return true;
    if (!in.sdc_closed)     return true;

    // BMS module-online mask + freshness.
    if (in.bms.module_online_mask != config::kAllModulesMask) return true;
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        const std::uint32_t age = in.now_tick - in.bms.last_rx_tick[m];
        if (in.bms.last_rx_tick[m] != 0 && age > config::kBmsStaleMs) return true;
        if (in.bms.last_rx_tick[m] == 0)                              return true;
    }

    // Cell V / T ranges.
    if (in.bms.min_cell_mV < config::kCellUVmV)      return true;
    if (in.bms.max_cell_mV > config::kCellOVmV)      return true;
    if (in.bms.min_tempC   < config::kCellUTC)       return true;
    if (in.bms.max_tempC   > config::kCellOTC)       return true;

    // Current sensor: not faulted, fresh, within absolute limit.
    if (in.current.sensor_fault)                                              return true;
    if (in.current.last_update_tick == 0)                                     return true;
    if (in.now_tick - in.current.last_update_tick > config::kIStaleMs)        return true;
    if (std::abs(in.current.filtered_mA) > config::kImaxMa)                   return true;

    // VCU DC bus heartbeat.
    if (in.vehicle.last_dc_bus_tick == 0)                                     return true;
    if (in.now_tick - in.vehicle.last_dc_bus_tick > config::kVcuStaleMs)      return true;

    return false;
}

}  // namespace ams::safety
