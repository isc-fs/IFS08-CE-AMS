// SPDX-License-Identifier: proprietary
//
// State the AMS receives from the wider vehicle on FDCAN1 (the
// "accumulator bus"). Currently a single signal: VCU DC-bus voltage
// heartbeat (0x100, standard). The 0x600 start-button and 0x18FF50E7
// charger-detect frames were retired in fix/48 -- their roles are now
// owned by the TSMS_Pin / DASH_CHG_Pin GPIOs read directly by
// SafetyTask. AMS firmware is standard-frame-only on FDCAN1 post-#231
// follow-up; the HW filter drops extended frames at the gate.
// last_dc_bus_tick is load-bearing: SafetyTask uses its freshness
// against VcuFreshMs to decide Car-vs-Charger mode at the moment of
// Start->Precharge.
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
    // Tick of the last valid (magic-matched) operator charge-mode request
    // frame (ChargeModeReqId). 0 = never seen. SafetyTask reads its
    // freshness at the Start->Precharge mode lock (#305).
    std::uint32_t last_charge_req_tick;
    // Operator balance-control override (#336, BalanceOverrideReqId 0x103).
    // last_balance_override_tick: 0 = never seen. balance_cmd: the last command
    // seen -- Off ("BALO"), On ("BALN"), or Auto ("BALX"). BmsPollTask resolves
    // the two through effective_balance_cmd() (dead-man -> Off) each window.
    std::uint32_t     last_balance_override_tick;
    config::BalanceCmd balance_cmd;

    // Per-module balancing enable (0x104). last_balance_modules_tick: 0 = never
    // seen. balance_modules_mask: last decoded 5-bit mask (bit m = module m
    // enabled). BmsPollTask resolves the two through effective_balance_modules_-
    // mask() (dead-man -> all enabled) each window.
    std::uint32_t     last_balance_modules_tick;
    std::uint8_t      balance_modules_mask;
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

    // True iff an operator charge-mode request was seen within
    // ChargeReqFreshMs of now_tick (#305). Pure; future-tick safe.
    [[nodiscard]] static bool charge_requested(std::uint32_t now_tick,
                                               std::uint32_t last_req_tick) noexcept;

    // Effective operator balancing command right now (#336). Applies the
    // dead-man: if the last 0x103 is stale (older than BalanceOverrideFreshMs)
    // or was never seen (last_override_tick == 0), returns BalanceCmd::Off so a
    // dead WarioCharger link never leaves the pack bleeding; otherwise returns
    // the last command as-is. Pure; future-tick safe.
    [[nodiscard]] static config::BalanceCmd effective_balance_cmd(
        std::uint32_t      now_tick,
        std::uint32_t      last_override_tick,
        config::BalanceCmd last_cmd) noexcept;

    // Effective per-module balancing enable mask right now. Dead-man: if the
    // last 0x104 is stale (> BalanceModulesFreshMs) or was never seen
    // (last_modules_tick == 0), returns BalanceModulesDefaultMask (all modules
    // enabled) so the pack behaves as global-only; otherwise returns the last
    // mask (low 5 bits). Pure; future-tick safe.
    [[nodiscard]] static std::uint8_t effective_balance_modules_mask(
        std::uint32_t now_tick,
        std::uint32_t last_modules_tick,
        std::uint8_t  last_mask) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ams
