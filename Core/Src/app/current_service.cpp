// SPDX-License-Identifier: proprietary

#include "current_service.hpp"



#include <cstdlib>


namespace ams {

CurrentService& CurrentService::instance() noexcept {
    static CurrentService kInstance;
    return kInstance;
}

std::int32_t CurrentService::adc_to_mA(std::uint16_t raw) noexcept {
    // Work in microvolts to keep integer-mV bias out of the rounding
    // around the zero point. Bipolar mapping after the carrier-board
    // diff amp (gain x4, R12 biased to Vref/2): S_CURRENT = 4 *
    // (OUTP - OUTN) + 1.65 V. Discharge -> positive S_CURRENT above
    // 1.65 V; charge -> negative S_CURRENT below 1.65 V.
    //
    //   v_uV     = raw * Vref_mV * 1000 / 4095
    //   delta_uV = v_uV - zero_mV * 1000        (+ above zero, - below)
    //   sensitivity = 20 mV/A * 10 = 200 (10*mV / A)
    //   mA       = delta_uV / sensitivity_uV_per_mA
    //            = delta_uV * 10 / kCurrentMvPerAmpe1
    //
    // The "+ = discharge, - = charge" convention is preserved: with
    // zero at 1.65 V and v_uV > zero on discharge, delta_uV is
    // positive and so is the returned mA value.
    //
    // raw * Vref_mV * 1000 can reach 4095 * 3300 * 1000 ≈ 1.35e10, so
    // the multiplication must be done in int64.
    const std::int64_t v_uV =
        (static_cast<std::int64_t>(raw) *
         static_cast<std::int64_t>(config::kAdcVrefMv) * 1000) /
        config::kAdcMaxCount;
    const std::int64_t delta_uV =
        v_uV - static_cast<std::int64_t>(config::kCurrentZeroMv) * 1000;
    return static_cast<std::int32_t>(
        (delta_uV * 10) / config::kCurrentMvPerAmpe1);
}

void CurrentService::update_from_adc(std::uint16_t raw, std::uint32_t now_tick) noexcept {
    const std::int32_t mA = adc_to_mA(raw);


    state_.raw_mA           = mA;
    state_.last_update_tick = now_tick;
    state_.sensor_fault     = false;  // we got a sample; trust it

    // IIR LPF: filtered <- filtered - (filtered >> N) + (raw >> N)
    // First sample seeds the filter so we don't ramp from zero.
    if (state_.filtered_mA == 0 && mA != 0) {
        state_.filtered_mA = mA;
    } else {
        state_.filtered_mA -= (state_.filtered_mA >> config::kCurrentFilterShift);
        state_.filtered_mA += (mA >> config::kCurrentFilterShift);
    }
}

CurrentState CurrentService::snapshot() const noexcept {
    return state_;
}

bool CurrentService::is_healthy(std::uint32_t now_tick) const noexcept {
    if (state_.sensor_fault)                                            return false;
    if (state_.last_update_tick == 0)                                   return false;
    if (now_tick - state_.last_update_tick > config::kIStaleMs)         return false;
    if (std::abs(state_.filtered_mA) > config::kImaxMa)                 return false;
    return true;
}

}  // namespace ams
