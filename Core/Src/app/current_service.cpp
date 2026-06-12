// SPDX-License-Identifier: proprietary

#include "current_service.hpp"




namespace ams {

CurrentService& CurrentService::instance() noexcept {
    static CurrentService Instance;
    return Instance;
}

std::int32_t CurrentService::adc_to_mA(std::uint16_t raw) noexcept {
    // Pack current, read in ADC DIFFERENTIAL mode on PF7/PF8
    // (ADC3_INP3/INN3). The conversion encodes OUT_P - OUT_N over
    // -Vref..+Vref onto codes 0..4095, with the zero-difference point at
    // mid-scale (CurrentZeroCount ~= 2048). The sensor common-mode
    // (~1.44 V) cancels in the subtraction, leaving only the bipolar
    // +/- 5 mV/A differential.
    //
    //   delta_codes = raw - CurrentZeroCount        (+ discharge, - charge)
    //   diff_uV     = delta_codes * 2 * Vref_mV * 1000 / 4095  (2x SE LSB)
    //   sensitivity = 5 mV/A * 10 = 50 (10*mV / A)   [CurrentMvPerAmpe1]
    //   mA          = diff_uV * 10 / CurrentMvPerAmpe1
    //
    // The "+ = discharge, - = charge" convention is preserved: discharge
    // drives OUT_P above OUT_N, so raw sits above mid-scale and the
    // returned mA is positive.
    //
    // delta_codes * 2 * Vref_mV * 1000 can reach ~2048 * 2 * 3300 * 1000
    // ~= 1.35e10, so the multiplication must be done in int64.
    const std::int64_t delta_codes =
        static_cast<std::int64_t>(raw) - config::CurrentZeroCount;
    const std::int64_t diff_uV =
        (delta_codes * 2 * static_cast<std::int64_t>(config::AdcVrefMv) * 1000) /
        config::AdcMaxCount;
    return static_cast<std::int32_t>(
        (diff_uV * 10) / config::CurrentMvPerAmpe1);
}

std::int32_t CurrentService::adc_to_mA_dcdc(std::uint16_t raw) noexcept {
    // DCDC current, read SINGLE-ended on PC1 (ADC3_INP11): an Allegro
    // ACS758 Hall sensor through a unity buffer. Ratiometric @ 3.3 V ->
    // offset 1.65 V (Vcc/2), sensitivity 26.4 mV/A. Uses its own
    // zero/sens constants. Classic single-ended mapping (zero at the
    // offset voltage):
    //
    //   v_uV     = raw * Vref_mV * 1000 / 4095
    //   delta_uV = v_uV - DcdcCurrentZeroMv * 1000   (+ above zero)
    //   mA       = delta_uV * 10 / DcdcCurrentMvPerAmpe1
    //
    // raw * Vref_mV * 1000 can reach ~1.35e10 -> int64.
    const std::int64_t v_uV =
        (static_cast<std::int64_t>(raw) *
         static_cast<std::int64_t>(config::AdcVrefMv) * 1000) /
        config::AdcMaxCount;
    const std::int64_t delta_uV =
        v_uV - static_cast<std::int64_t>(config::DcdcCurrentZeroMv) * 1000;
    return static_cast<std::int32_t>(
        (delta_uV * 10) / config::DcdcCurrentMvPerAmpe1);
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
    const std::int32_t mA = adc_to_mA(raw);


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

void CurrentService::update_dcdc_from_adc(std::uint16_t raw,
                                          std::uint32_t now_tick) noexcept {
    // Separate single-ended sensor on PC1 (ADC3_INP11), distinct from
    // the differential pack channel -- use the DCDC-specific converter.
    const std::int32_t mA = adc_to_mA_dcdc(raw);

    state_.dcdc_raw_mA           = mA;
    state_.last_dcdc_update_tick = now_tick;
    state_.dcdc_sensor_fault     = false;

    if (state_.dcdc_filtered_mA == 0 && mA != 0) {
        state_.dcdc_filtered_mA = mA;
    } else {
        state_.dcdc_filtered_mA -= (state_.dcdc_filtered_mA >> config::CurrentFilterShift);
        state_.dcdc_filtered_mA += (mA >> config::CurrentFilterShift);
    }
}

CurrentState CurrentService::snapshot() const noexcept {
    return state_;
}

bool CurrentService::is_dcdc_fresh(std::uint32_t now_tick) const noexcept {
    if (state_.dcdc_sensor_fault)                                              return false;
    if (state_.last_dcdc_update_tick == 0u)                                    return false;
    if (now_tick - state_.last_dcdc_update_tick > config::DcdcIStaleMs)        return false;
    return true;
}

}  // namespace ams
