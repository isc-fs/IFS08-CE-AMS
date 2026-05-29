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

// Which predicate branch latched ERROR. Surfaced on pit-diag 0x6C0[6]
// so the HIL bench can pinpoint the fault without a debugger (#276).
// Values are stable wire contract -- append only, never renumber.
enum class FaultReason : std::uint8_t {
    None               = 0,
    ForceError         = 1,
    BmsModuleOffline   = 2,
    BmsStale           = 3,
    CellUnderVoltage   = 4,
    CellOverVoltage    = 5,
    CellUnderTemp      = 6,
    CellOverTemp       = 7,
    CurrentSensorFault = 8,
    CurrentStale       = 9,
    CurrentOverLimit   = 10,
    VcuStale           = 11,
    // 12 (FsmError) is reserved for the SafetyTask FSM-driven Error
    // path (precharge timeout / input drop), set there, not here.
};

struct FaultResult {
    FaultReason  reason = FaultReason::None;
    // BmsStale: offending module index. BmsModuleOffline: the live
    // module_online_mask. Otherwise 0.
    std::uint8_t detail = 0;
    [[nodiscard]] bool faulted() const noexcept {
        return reason != FaultReason::None;
    }
};

// Unsigned tick age that is safe against a producer updating its
// `last_*_tick` between the SafetyTask `now` sample and the snapshot
// read. If `last` is *ahead* of `now` (it just reported), the naive
// `now - last` underflows to ~4e9 and spuriously trips staleness --
// this was the #276 boot-grace-boundary ERROR latch. Clamp to 0 in
// that case. A never-reported service still has `last == 0`, so
// `age == now` and the intended "no data after grace => stale"
// behaviour is preserved.
[[nodiscard]] inline std::uint32_t tick_age(std::uint32_t now,
                                            std::uint32_t last) noexcept {
    return (now >= last) ? (now - last) : 0u;
}

[[nodiscard]] inline FaultResult evaluate_fault_detail(const Inputs& in) noexcept {
    // The AMS does not sense the SDC line directly: it IS part of the
    // SDC via the AMS_OK output. There's no dedicated DIGITAL1 input on
    // the v1.2 daughterboard (PE9 was a holdover from the legacy bare-
    // metal port and the schematic doesn't route it). If we ever add an
    // SDC-feedback input back, it returns here.
    //
    // Immediate-safety predicates apply from t=0 -- a stuck task at
    // boot is never OK to tolerate.
    if (in.force_error_set) return { FaultReason::ForceError, 0 };

    // Boot grace: suppress the data-presence predicates while services
    // are still warming up. Every service initialises its `last_*_tick`
    // to 0; without this gate, the first SafetyTask iteration (~10 ms
    // after osKernelStart) trips on every freshness check, withholds
    // the watchdog refresh, and IWDG resets the chip in ~100 ms before
    // BmsPollTask (250 ms cadence) has fired even once.
    //
    // After the grace expires, the `== 0` short-circuits are no longer
    // needed: `tick_age(now, 0) = now` will naturally exceed the
    // staleness window for any service that hasn't yet reported.
    if (in.now_tick < config::SafetyBootGraceMs) return {};

    // BMS module-online mask + freshness. Real LTC chain (flight) or
    // the Pi Pico LTC6820+LTC6811 emulator (HIL bench) feeds these
    // fields on the same 250 ms cadence; either way the predicate
    // treats them identically.
    if (in.bms.module_online_mask != config::AllModulesMask) {
        return { FaultReason::BmsModuleOffline,
                 static_cast<std::uint8_t>(in.bms.module_online_mask) };
    }
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (tick_age(in.now_tick, in.bms.last_rx_tick[m]) > config::BmsStaleMs) {
            return { FaultReason::BmsStale, m };
        }
    }

    // Cell V / T ranges.
    if (in.bms.min_cell_mV < config::CellUnderVoltageMv) return { FaultReason::CellUnderVoltage, 0 };
    if (in.bms.max_cell_mV > config::CellOverVoltageMv)  return { FaultReason::CellOverVoltage,  0 };
    if (in.bms.min_tempC   < config::CellUnderTempC)     return { FaultReason::CellUnderTemp,    0 };
    if (in.bms.max_tempC   > config::CellOverTempC)      return { FaultReason::CellOverTemp,     0 };

    // Current sensor: not faulted, fresh, within absolute limit.
    if (in.current.sensor_fault) return { FaultReason::CurrentSensorFault, 0 };
    if (tick_age(in.now_tick, in.current.last_update_tick) > config::IStaleMs) {
        return { FaultReason::CurrentStale, 0 };
    }
    if (std::abs(in.current.filtered_mA) > config::CurrentMaxMa) return { FaultReason::CurrentOverLimit, 0 };

    // VCU DC bus heartbeat.
    if (tick_age(in.now_tick, in.vehicle.last_dc_bus_tick) > config::VcuStaleMs) {
        return { FaultReason::VcuStale, 0 };
    }

    return {};
}

// Boolean convenience wrapper. Behaviour is identical to the detailed
// form -- same branch order, same thresholds.
[[nodiscard]] inline bool evaluate_fault(const Inputs& in) noexcept {
    return evaluate_fault_detail(in).faulted();
}

}  // namespace ams::safety
