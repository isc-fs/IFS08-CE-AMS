// SPDX-License-Identifier: proprietary

#include "current_service.hpp"



#include <cstdlib>


namespace ams {

CurrentService& CurrentService::instance() noexcept {
    static CurrentService kInstance;
    return kInstance;
}

std::int32_t CurrentService::adc_to_mA(std::uint16_t raw) noexcept {
    // Work in microvolts to avoid the ~175 mA bias that integer-mV
    // arithmetic introduces around the 2.500 V zero point (the unit
    // test test_current_adc_symmetric_around_zero catches it).
    //
    //   v_uV    = raw * Vref_mV * 1000 / 4095
    //   delta_uV = zero_mV * 1000 - v_uV
    //   sensitivity = 5.7 mV/A = 5.7 uV/mA = kCurrentMvPerAmpe1 / 10 uV/mA
    //   mA       = delta_uV / sensitivity_uV_per_mA
    //            = delta_uV * 10 / kCurrentMvPerAmpe1
    //
    // raw * Vref_mV * 1000 can reach 4095 * 3300 * 1000 ≈ 1.35e10, so
    // the multiplication must be done in int64.
    const std::int64_t v_uV =
        (static_cast<std::int64_t>(raw) *
         static_cast<std::int64_t>(config::kAdcVrefMv) * 1000) /
        config::kAdcMaxCount;
    const std::int64_t delta_uV =
        static_cast<std::int64_t>(config::kCurrentZeroMv) * 1000 - v_uV;
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

void CurrentService::set_charger_detected(bool detected) noexcept {
    state_.charger_detected = detected;
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
