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
    bool                force_error_set; // safety_events ForceError pending
    std::uint32_t       now_tick;
};

[[nodiscard]] inline bool evaluate_fault(const Inputs& in) noexcept {
    // The AMS does not sense the SDC line directly: it IS part of the
    // SDC via the AMS_OK output. There's no dedicated DIGITAL1 input on
    // the v1.2 daughterboard (PE9 was a holdover from the legacy bare-
    // metal port and the schematic doesn't route it). If we ever add an
    // SDC-feedback input back, it returns here.
    //
    // Immediate-safety predicates apply from t=0 -- a stuck task at
    // boot is never OK to tolerate.
    if (in.force_error_set) return true;

    // Boot grace: suppress the data-presence predicates while services
    // are still warming up. Every service initialises its `last_*_tick`
    // to 0; without this gate, the first SafetyTask iteration (~10 ms
    // after osKernelStart) trips on every freshness check, withholds
    // the watchdog refresh, and IWDG resets the chip in ~100 ms before
    // BmsPollTask (250 ms cadence) has fired even once.
    //
    // After the grace expires, the `== 0` short-circuits are no longer
    // needed: unsigned subtraction `now_tick - 0 = now_tick` will
    // naturally exceed the staleness window for any service that
    // hasn't yet reported -- equivalent fault, fewer special cases.
    if (in.now_tick < config::SafetyBootGraceMs) return false;

    // BMS module-online mask + freshness. Under -DAMS_BMS_HIL_STUB the
    // BmsPollTask seeds these fields with nominal-healthy values every
    // 250 ms (see bms_service.hpp::seed_for_hil_stub), so this block
    // is HIL-agnostic -- the predicate doesn't know or care whether
    // the data came from a real LTC chain or from the stub seeder.
    if (in.bms.module_online_mask != config::AllModulesMask) return true;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (in.now_tick - in.bms.last_rx_tick[m] > config::BmsStaleMs) return true;
    }

    // Cell V / T ranges.
    if (in.bms.min_cell_mV < config::CellUnderVoltageMv)      return true;
    if (in.bms.max_cell_mV > config::CellOverVoltageMv)      return true;
    if (in.bms.min_tempC   < config::CellUnderTempC)       return true;
    if (in.bms.max_tempC   > config::CellOverTempC)       return true;

    // Current sensor: not faulted, fresh, within absolute limit.
    if (in.current.sensor_fault)                                       return true;
#if !defined(AMS_BMS_HIL_STUB)
    // Bench has no real current sensor (#123 Block B/C sweep: chip
    // latched Error ~200 ms after boot_grace_ms because no fixture
    // feeds current-sensor frames). Skip the freshness check on the
    // stub build -- sensor_fault + Imax still apply, so a fixture
    // that DOES inject current frames will still see meaningful
    // safety behaviour.
    if (in.now_tick - in.current.last_update_tick > config::IStaleMs) return true;
#endif
    if (std::abs(in.current.filtered_mA) > config::CurrentMaxMa)            return true;

    // VCU DC bus heartbeat.
    if (in.now_tick - in.vehicle.last_dc_bus_tick > config::VcuStaleMs) return true;

    return false;
}

}  // namespace ams::safety
