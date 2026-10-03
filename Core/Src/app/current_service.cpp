// SPDX-License-Identifier: proprietary

#include "current_service.hpp"

namespace ams {

CurrentService& CurrentService::instance() noexcept {
    static CurrentService Instance;
    return Instance;
}

std::int32_t CurrentService::adc_to_mA(std::uint16_t raw) noexcept {
    return adc_q4_to_mA(static_cast<std::uint32_t>(raw) << config::CurrentAdcFracBits);
}

std::int32_t CurrentService::adc_q4_to_mA(std::uint32_t q4) noexcept {
    // Pack current, read in ADC DIFFERENTIAL mode on PF7/PF8
    // (ADC3_INP3/INN3). The conversion encodes OUT_P - OUT_N over
    // -Vref..+Vref onto codes 0..4095, with the zero-difference point at
    // mid-scale (CurrentZeroCount ~= 2048). The sensor common-mode
    // (~1.44 V) cancels in the subtraction, leaving only the bipolar
    // +/- 5 mV/A differential. q4 is that code with F = CurrentAdcFracBits
    // fractional bits (the oversampler's output).
    //
    //   delta_q4    = q4 - CurrentZeroCount * 2^F   (+ discharge, - charge)
    //   diff_uV     = delta_q4 * 2 * Vref_mV * 1000 / (4095 * 2^F)  (2x SE LSB)
    //   sensitivity = 5 mV/A * 10 = 50 (10*mV / A)   [CurrentMvPerAmpe1]
    //   mA          = diff_uV * 10 / CurrentMvPerAmpe1
    //
    // For q4 = raw << F the 2^F cancels exactly, so a 12-bit code converts to
    // the same mA it always did. The "+ = discharge, - = charge" convention
    // is preserved: discharge drives OUT_P above OUT_N, so the code sits
    // above mid-scale and the returned mA is positive.
    //
    // delta_q4 * 2 * Vref_mV * 1000 reaches ~32768 * 2 * 3300 * 1000 ~= 2.2e11,
    // so the multiplication must be done in int64.
    constexpr std::int64_t One = std::int64_t{1} << config::CurrentAdcFracBits;
    const std::int64_t delta_q4 =
        static_cast<std::int64_t>(q4) - std::int64_t{config::CurrentZeroCount} * One;
    const std::int64_t diff_uV =
        (delta_q4 * 2 * static_cast<std::int64_t>(config::AdcVrefMv) * 1000) /
        (std::int64_t{config::AdcMaxCount} * One);
    return static_cast<std::int32_t>(
        (diff_uV * 10) / config::CurrentMvPerAmpe1);
}

bool CurrentService::leg_voltage_plausible(std::uint16_t raw) noexcept {
    // Single-ended OUT_P voltage in mV = raw * Vref / FS.
    const std::int32_t v_mV = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(raw) * config::AdcVrefMv) / config::AdcMaxCount);
    return v_mV >= config::CurrentLegPlausMinMv &&
           v_mV <= config::CurrentLegPlausMaxMv;
}

void CurrentService::update_from_adc(std::uint16_t raw, std::uint32_t now_tick,
                                     bool sensor_fault) noexcept {
    update_from_q4(static_cast<std::uint32_t>(raw) << config::CurrentAdcFracBits,
                   now_tick, sensor_fault);
}

void CurrentService::update_from_q4(std::uint32_t q4, std::uint32_t now_tick,
                                    bool sensor_fault) noexcept {
    const std::int32_t mA = adc_q4_to_mA(q4);

    state_.raw_mA           = mA;
    state_.last_update_tick = now_tick;
    state_.sensor_fault     = sensor_fault;  // debounced disconnect verdict

    // IIR LPF: filtered <- filtered - (filtered >> N) + (raw >> N)
    // First sample seeds the filter so we don't ramp from zero.
    if (state_.filtered_mA == 0 && mA != 0) {
        state_.filtered_mA = mA;
    } else {
        state_.filtered_mA -= (state_.filtered_mA >> config::CurrentFilterShift);
        state_.filtered_mA += (mA >> config::CurrentFilterShift);
    }
}

CurrentState CurrentService::snapshot() const noexcept {
    return state_;
}

}  // namespace ams
