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
        // Byte 2 is optional on the wire: a VCU that predates it sends DLC 2 and
        // the AMS behaves exactly as it did before rather than blocking the car.
        // The two absent-byte defaults point opposite ways on purpose -- bleed
        // disconnected, voltage usable -- because both mean "an ECU that cannot
        // state this must not have its silence change anything". Seeing the byte
        // once latches ecu_discharge_capable.
        if (f.dlc >= 3) {
            state_.ecu_discharge_capable = true;
            state_.discharge_engaged     = (f.data[2] & 0x01u) != 0u;
            state_.dc_bus_valid          = (f.data[2] & 0x02u) != 0u;
        } else {
            state_.discharge_engaged     = false;
            state_.dc_bus_valid          = true;
        }
        return true;
    }

    // Operator charge-mode request. Magic-gated so bus noise / a
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

    // Operator balance-control override: 3-state master switch on 0x103.
    // "BALO" -> Off, "BALN" -> On, "BALX" -> Auto. An unrecognised payload is
    // ignored (bus-noise safe) and leaves the prior command + tick in place.
    if (f.id == config::BalanceOverrideReqId) {
        if (f.dlc < config::BalanceOverrideReqDlc) return false;
        bool is_off = true, is_on = true, is_auto = true;
        for (std::size_t i = 0; i < config::BalanceOverrideReqDlc; ++i) {
            if (f.data[i] != config::BalanceCmdOffMagic[i])  is_off  = false;
            if (f.data[i] != config::BalanceCmdOnMagic[i])   is_on   = false;
            if (f.data[i] != config::BalanceCmdAutoMagic[i]) is_auto = false;
        }
        if      (is_off)  state_.balance_cmd = config::BalanceCmd::Off;
        else if (is_on)   state_.balance_cmd = config::BalanceCmd::On;
        else if (is_auto) state_.balance_cmd = config::BalanceCmd::Auto;
        else return false;                        // no magic matched -> ignore
        state_.last_balance_override_tick = f.timestamp_ms;
        return true;
    }

    // 0x104 -- per-module balancing enable. magic "BALM" (bytes 0..3) + byte 4 =
    // 5-bit enable mask. Unrecognised payload ignored (bus-noise safe), leaving
    // the prior mask + tick in place.
    if (f.id == config::BalanceModulesReqId) {
        if (f.dlc < config::BalanceModulesReqDlc) return false;
        for (std::size_t i = 0; i < 4u; ++i) {
            if (f.data[i] != config::BalanceModulesMagic[i]) return false;  // no magic -> ignore
        }
        state_.balance_modules_mask =
            static_cast<std::uint8_t>(f.data[4] & config::AllModulesMask);   // low 5 bits
        state_.last_balance_modules_tick = f.timestamp_ms;
        return true;
    }
    return false;
}

bool VehicleService::charge_requested(std::uint32_t now_tick,
                                      std::uint32_t last_req_tick) noexcept {
    if (last_req_tick == 0u) return false;
    // Future-tick safe: a request stamped slightly
    // ahead of `now` counts as just-seen, never as ancient.
    const std::uint32_t age =
        (now_tick >= last_req_tick) ? (now_tick - last_req_tick) : 0u;
    return age <= config::ChargeReqFreshMs;
}

config::BalanceCmd VehicleService::effective_balance_cmd(
    std::uint32_t      now_tick,
    std::uint32_t      last_override_tick,
    config::BalanceCmd last_cmd) noexcept {
    if (last_override_tick == 0u) return config::BalanceCmd::Off;  // never seen
    // Future-tick safe: a command stamped slightly ahead of
    // now counts as just-seen, never as ancient.
    const std::uint32_t age =
        (now_tick >= last_override_tick) ? (now_tick - last_override_tick) : 0u;
    if (age > config::BalanceOverrideFreshMs) return config::BalanceCmd::Off;  // dead-man
    return last_cmd;
}

std::uint8_t VehicleService::effective_balance_modules_mask(
    std::uint32_t now_tick,
    std::uint32_t last_modules_tick,
    std::uint8_t  last_mask) noexcept {
    // Never seen -> all modules enabled (behave as global-only).
    if (last_modules_tick == 0u) return config::BalanceModulesDefaultMask;
    // Future-tick safe.
    const std::uint32_t age =
        (now_tick >= last_modules_tick) ? (now_tick - last_modules_tick) : 0u;
    // Dead-man: a stale link re-enables every module -- the 0x103 dead-man is
    // what actually stops balancing on a dead WarioCharger, so a lost 0x104
    // must not silently freeze balancing off on some modules.
    if (age > config::BalanceModulesFreshMs) return config::BalanceModulesDefaultMask;
    return static_cast<std::uint8_t>(last_mask & config::AllModulesMask);
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ams
