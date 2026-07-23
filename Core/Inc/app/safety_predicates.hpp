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
    // The VCU heartbeat is required only once committed to Car mode
    // (mode_locked == Car). In Charger mode -- and the pre-lock Start /
    // Undecided window -- the VCU is expected to be absent (the car
    // isn't running), so its staleness must NOT be a fault (#302).
    bool                vcu_required;
    // Mirror of vcu_required for the charge side: the charger heartbeat
    // (0x101) is required only once committed to Charger mode. In Car mode
    // and the pre-lock window the charger is absent by definition, so its
    // staleness must NOT fault.
    bool                charger_required;
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
    TempSensorDisconnected = 13,
    // Charger heartbeat (0x101) went stale while committed to Charger mode
    // (mirror of VcuStale for the charge side): the WarioCharger unplugged
    // mid-charge -> Error + open the AIRs. Only fires in Charger mode.
    ChargerStale       = 14,
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

// First module whose per-module aggregate is below / above a limit, for
// the 0x6C0[7] fault-detail byte (#279). Returns NoOffendingModule
// (0xFF) if NONE match -- which, when the summary min/max already
// crossed the threshold, means min_cell_mV / max_* disagrees with the
// per-module aggregates: the fingerprint of a torn lock-free snapshot
// read (the two were copied from different poll cycles). Distinct from a
// real module index 0..4 so the bench can tell "module N is genuinely
// low" from "inconsistent snapshot".
inline constexpr std::uint8_t NoOffendingModule = 0xFFu;

[[nodiscard]] inline std::uint8_t
module_below(const std::uint16_t (&per_module)[config::BmsModuleCount],
             std::uint16_t limit) noexcept {
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (per_module[m] < limit) return m;
    }
    return NoOffendingModule;
}
[[nodiscard]] inline std::uint8_t
module_above_u16(const std::uint16_t (&per_module)[config::BmsModuleCount],
                 std::uint16_t limit) noexcept {
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (per_module[m] > limit) return m;
    }
    return NoOffendingModule;
}
[[nodiscard]] inline std::uint8_t
module_above_i16(const std::int16_t (&per_module)[config::BmsModuleCount],
                 std::int16_t limit) noexcept {
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (per_module[m] > limit) return m;
    }
    return NoOffendingModule;
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

    // Temperature-sensor DISCONNECT (FS rule: a disconnected temp sensor opens
    // the SDC). temp_disconnect_mask holds, per online module, whether a channel
    // that had read valid is now open past the debounce (BmsService). This is a
    // PRESENCE check, not a range check -- an open NTC reads the rail regardless
    // of calibration -- so it is armed independently of TempFaultsTrusted, and
    // self-gated by the "seen valid once" latch so a boot-time or unpopulated
    // sentinel never trips it. Detail byte = the offending-module mask.
    if (config::TempSensorPresenceCheck && in.bms.temp_disconnect_mask != 0u) {
        return { FaultReason::TempSensorDisconnected, in.bms.temp_disconnect_mask };
    }

    // Cell V / T ranges. Gated on first_full_poll_done (#279): until
    // every module has reported PEC-clean at least once, cell_mV /
    // cell_tempC may hold boot sentinels or a partially-populated mix
    // that must not latch a false under-voltage at the grace edge. A
    // genuinely-absent module is still caught above by the freshness /
    // module_online_mask checks. The detail byte carries the offending
    // module index so the bench can localise the cell over CAN (0x6C0[7]).
    if (in.bms.first_full_poll_done) {
        if (in.bms.min_cell_mV < config::CellUnderVoltageMv) {
            return { FaultReason::CellUnderVoltage,
                     module_below(in.bms.vmin_module, config::CellUnderVoltageMv) };
        }
        if (in.bms.max_cell_mV > config::CellOverVoltageMv) {
            return { FaultReason::CellOverVoltage,
                     module_above_u16(in.bms.vmax_module, config::CellOverVoltageMv) };
        }
        // Cell TEMPERATURE faults are gated behind config::TempFaultsTrusted:
        // NTC temps come through the ADG731 mux, whose select word was wrong
        // and is not yet validated on flight, so we do NOT fault on them yet
        // (voltage protection above is unaffected). Flip the flag once the mux
        // fix ships to flight + temps are validated end-to-end.
        if (config::TempFaultsTrusted) {
            if (in.bms.min_tempC < config::CellUnderTempC) return { FaultReason::CellUnderTemp, 0 };
            if (in.bms.max_tempC > config::CellOverTempC) {
                return { FaultReason::CellOverTemp,
                         module_above_i16(in.bms.tmax_module, config::CellOverTempC) };
            }
        }
    }

    // Current sensor: not faulted, fresh, within absolute limit. The
    // freshness check is unconditional now that the HIL bench drives a
    // real current path (the Pico-emulated chain + bench current
    // injection), so flight and bench evaluate identically.
    if (in.current.sensor_fault) return { FaultReason::CurrentSensorFault, 0 };
    if (tick_age(in.now_tick, in.current.last_update_tick) > config::IStaleMs) {
        return { FaultReason::CurrentStale, 0 };
    }
    if (std::abs(in.current.filtered_mA) > config::CurrentMaxMa) return { FaultReason::CurrentOverLimit, 0 };

    // VCU DC bus heartbeat -- a fault ONLY once committed to Car mode.
    // In Charger mode the VCU is absent by definition (the car isn't
    // running), and in the pre-lock Undecided window the AIRs are still
    // open. Gating on vcu_required is what makes Charger mode reachable:
    // the lock needs the VCU stale > VcuFreshMs (1000 ms), but an
    // un-gated VcuStale (200 ms) would always latch ERROR first (#302).
    if (in.vcu_required &&
        tick_age(in.now_tick, in.vehicle.last_dc_bus_tick) > config::VcuStaleMs) {
        return { FaultReason::VcuStale, 0 };
    }

    // Charger heartbeat (0x101) stale while committed to Charger mode: the
    // WarioCharger was disconnected mid-charge. Mirror of VcuStale on the charge
    // side -- fault and open the AIRs so charging stops. Gated on
    // charger_required so it never fires in Car mode or the pre-lock window
    // (where the charger is legitimately absent).
    if (in.charger_required &&
        tick_age(in.now_tick, in.vehicle.last_charge_req_tick) > config::ChargerStaleMs) {
        return { FaultReason::ChargerStale, 0 };
    }

    return {};
}

// Boolean convenience wrapper. Behaviour is identical to the detailed
// form -- same branch order, same thresholds.
[[nodiscard]] inline bool evaluate_fault(const Inputs& in) noexcept {
    return evaluate_fault_detail(in).faulted();
}

// Whether the AMS should assert AMS_OK (PB4), its leg of the shutdown
// circuit (#299). Active-high: HIGH = "AMS healthy, not blocking the
// SDC". Asserted ONLY when (a) the boot grace has passed -- during grace
// the data-presence predicates are suppressed, so we must NOT enable the
// SDC against possibly-unverified inputs -- and (b) no ERROR is latched.
// SafetyTask calls this every 10 ms tick and drives the pin, so AMS_OK
// tracks the live latch state and deasserts the instant a fault latches.
[[nodiscard]] inline bool ams_ok_asserted(std::uint32_t now_tick,
                                          bool error_latched) noexcept {
    return (now_tick >= config::SafetyBootGraceMs) && !error_latched;
}

// The cell voltage / temperature RANGE reasons -- the slow-by-nature
// faults that get debounced (#279). A cell cannot leave its valid
// window for a single 10 ms tick and return, so a transient one is a
// glitch (torn snapshot read / unsettled poll), not a real condition.
[[nodiscard]] inline bool is_cell_range_reason(FaultReason r) noexcept {
    return r == FaultReason::CellUnderVoltage ||
           r == FaultReason::CellOverVoltage  ||
           r == FaultReason::CellUnderTemp    ||
           r == FaultReason::CellOverTemp;
}

// Debounce for the cell V/T range predicates. Pure + host-testable so
// the latch timing is unit-covered, not just live on the bench (#279).
// Call once per SafetyTask evaluation with the predicate's reason;
// returns true only once a cell-range reason has persisted for
// `confirm_ticks` consecutive calls. Any non-cell reason (including
// None) resets the streak -- immediate faults are handled by the caller
// and are never debounced.
struct CellFaultDebounce {
    std::uint16_t streak = 0;
    std::uint8_t  reason = 0;   // raw FaultReason of the current streak

    // Returns true iff a cell-range fault is now CONFIRMED (should
    // latch). For non-cell reasons it resets and returns false; the
    // caller applies its own immediate-fault decision in that case.
    [[nodiscard]] bool update(FaultReason r, std::uint16_t confirm_ticks) noexcept {
        if (!is_cell_range_reason(r)) {
            streak = 0;
            reason = 0;
            return false;
        }
        const auto ru = static_cast<std::uint8_t>(r);
        if (ru == reason) {
            if (streak < confirm_ticks) ++streak;
        } else {
            reason = ru;
            streak = 1;
        }
        return streak >= confirm_ticks;
    }
};

// BmsStale confirmation debounce. BmsStale is already a timeout (a BMS module
// silent past BmsStaleMs), but the safety loop latches it on the FIRST tick it
// crosses. This requires it to persist confirm_ticks consecutive evaluations
// first, so a far module that flickers just past the window under a brief EMI
// burst -- then reports on its next voltage poll (<= 250 ms later) -- does not
// spuriously open the contactors. Any reason other than BmsStale resets the
// streak (that fault takes its own immediate-latch path in the caller).
// See config::BmsStaleConfirmTicks for the SAFETY TRADEOFF (adds confirm time
// to the detection of a genuinely lost module). Mirrors CellFaultDebounce's
// self-gating pattern so the caller can drive it every tick.
struct BmsStaleDebounce {
    std::uint16_t streak = 0;

    [[nodiscard]] bool update(FaultReason r, std::uint16_t confirm_ticks) noexcept {
        if (r != FaultReason::BmsStale) {
            streak = 0;
            return false;
        }
        if (streak < confirm_ticks) ++streak;
        return streak >= confirm_ticks;
    }
};

}  // namespace ams::safety
