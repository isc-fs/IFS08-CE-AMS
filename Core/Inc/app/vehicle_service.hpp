// SPDX-License-Identifier: proprietary
//
// State the AMS receives from the wider vehicle on FDCAN1 (the "accumulator
// bus"): the VCU DC-bus heartbeat (0x100) plus the operator charge-mode
// (0x101) and balancing (0x103 / 0x104) requests. Cockpit inputs are NOT
// here -- TSMS_Pin / DASH_CHG_Pin are GPIOs read directly by SafetyTask.
// The retired 0x600 start-button and 0x18FF50E7 charger-detect frames are
// gone with them -- if you see either on the wire, nothing here consumes it.
// Standard frames only; the hardware filter drops extended IDs at the gate.
//
// last_dc_bus_tick is load-bearing: SafetyTask checks its freshness against
// VcuFreshMs to pick Car-vs-Charger mode at Start->Precharge.
//
// Single-writer: AcuCanTask. Many readers: MainTask.
// Synchronisation: docs/ARCHITECTURE.md §7.

#pragma once

#include "ams_config.hpp"
#include "can_frame.hpp"

#include <cstdint>

namespace ams {

struct VehicleState {
    std::uint16_t dc_bus_V;          // from 0x100, little-endian bytes 0..1
    std::uint32_t last_dc_bus_tick;
    // ECU report that the bleed resistor is CONNECTED across the DC link, from
    // 0x100 byte 2 bit 0. Set while the shutdown circuit is open, and while the
    // ECU is securing a discharge that the SDC closing again interrupted. The
    // AMS must close no contactor while it is set: with the SDC closed and the
    // bleed connected, closing an AIR puts pack current through a resistor
    // rated for transient duty only.
    bool          discharge_engaged;
    // The ECU's statement that dc_bus_V above is a PRESENT-TENSE measurement,
    // from 0x100 byte 2 bit 1. The ECU relays the inverter's reading and emits
    // every cycle regardless, substituting 0 V once the inverter has gone
    // quiet -- so an on-time frame says nothing about the age of the number
    // inside it. 0 V is safe for the 95 % precharge criterion (it fails it)
    // and WRONG for the re-arm gate, which reads a low voltage as a drained
    // link. Hence a separate bit instead of trusting the value.
    //
    // Defaults TRUE where discharge_engaged defaults false; both follow one
    // compatibility rule -- an ECU that cannot express the fact must not have
    // its silence change AMS behaviour. Absent byte 2, the bleed is assumed
    // disconnected and the voltage usable. Neither default is load-bearing on
    // a real bus: before the first 0x100, last_dc_bus_tick is 0 and every
    // consumer is already gated on freshness.
    bool          dc_bus_valid = true;
    // Sticky: an 0x100 with DLC >= 3 has been seen, so this ECU publishes the
    // discharge state and can drain a stranded link. Until then the AMS does
    // not enforce its own dc_bus re-arm block -- refusing to arm over a link
    // the other end cannot drain would brick the car against an older ECU.
    // Never cleared; a mid-session downgrade is not worth modelling.
    bool          ecu_discharge_capable;
    // Tick of the last valid (magic-matched) operator charge-mode request
    // (ChargeModeReqId). 0 = never seen. SafetyTask reads its freshness at the
    // Start->Precharge mode lock.
    std::uint32_t last_charge_req_tick;
    // Operator balance-control override (BalanceOverrideReqId 0x103). Tick
    // 0 = never seen; balance_cmd is the last command -- Off ("BALO"), On
    // ("BALN") or Auto ("BALX"). BmsPollTask resolves the pair through
    // effective_balance_cmd() (dead-man -> Off) each window.
    std::uint32_t     last_balance_override_tick;
    config::BalanceCmd balance_cmd;

    // Per-module balancing enable (0x104). Tick 0 = never seen;
    // balance_modules_mask is the last decoded 5-bit mask (bit m = module m
    // enabled). BmsPollTask resolves the pair through
    // effective_balance_modules_mask() (dead-man -> all enabled) each window.
    std::uint32_t     last_balance_modules_tick;
    std::uint8_t      balance_modules_mask;
};

class VehicleService {
public:
    static VehicleService& instance() noexcept;

    // True if the frame matched a known accumulator-bus ID and the state was
    // updated. False -> the caller may telemeter the drop.
    bool update_from_frame(const CanFrame& f) noexcept;

    [[nodiscard]] VehicleState snapshot() const noexcept;

    // Pure helper, public for unit testing.
    static std::uint16_t decode_dc_bus_V(const std::uint8_t *data) noexcept;

    // True iff an operator charge-mode request was seen within
    // ChargeReqFreshMs of now_tick. Pure; future-tick safe.
    [[nodiscard]] static bool charge_requested(std::uint32_t now_tick,
                                               std::uint32_t last_req_tick) noexcept;

    // Effective operator balancing command right now, with the dead-man: a
    // last 0x103 that is stale (older than BalanceOverrideFreshMs) or never
    // seen (last_override_tick == 0) gives BalanceCmd::Off, so a dead
    // WarioCharger link never leaves the pack bleeding. Otherwise the last
    // command as-is. Pure; future-tick safe.
    [[nodiscard]] static config::BalanceCmd effective_balance_cmd(
        std::uint32_t      now_tick,
        std::uint32_t      last_override_tick,
        config::BalanceCmd last_cmd) noexcept;

    // Effective per-module balancing enable mask right now, with the dead-man:
    // a last 0x104 that is stale (> BalanceModulesFreshMs) or never seen
    // (last_modules_tick == 0) gives BalanceModulesDefaultMask (all modules
    // enabled), so the pack behaves as global-only. Otherwise the last mask
    // (low 5 bits). Pure; future-tick safe.
    [[nodiscard]] static std::uint8_t effective_balance_modules_mask(
        std::uint32_t now_tick,
        std::uint32_t last_modules_tick,
        std::uint8_t  last_mask) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ams
