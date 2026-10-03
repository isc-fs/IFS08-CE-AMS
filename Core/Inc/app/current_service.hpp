// SPDX-License-Identifier: proprietary
//
// Pack current measurement service. Single-writer (CurrentSensorTask via
// update_from_adc); many readers (MainTask, AcuCanTask, BmsPollTask).
// Synchronisation: docs/ARCHITECTURE.md §7.

#pragma once

#include "ams_config.hpp"

#include <cstdint>

namespace ams {

struct CurrentState {
    // Pack current. Signed mA; convention is `+ = discharge, - = charge`.
    // Sourced from the differential pair PF7/PF8 (ADC3_INP3/INN3,
    // Bourns SSA-2-250A read in ADC differential mode; see adc_to_mA +
    // ams_config.hpp commentary).
    std::int32_t  raw_mA;        // mean over the last CurrentPeriodMs capture, no filter
    std::int32_t  filtered_mA;   // IIR low-pass, tau ~ 16 samples
    std::uint32_t last_update_tick;
    bool          sensor_fault;      // ADC failed to convert, or out of plausible range
};

class CurrentService {
public:
    static CurrentService& instance() noexcept;

    // Called by CurrentSensorTask only, once per CurrentPeriodMs, with the
    // mean of that cycle's capture as a Q4 code (16 x the 12-bit code).
    // Converts it to mA, updates the filter, refreshes the timestamp.
    // `sensor_fault` is the debounced disconnect verdict from the task (OUT_P
    // single-ended out of its plausible window); it sets the
    // CurrentState.sensor_fault flag the safety predicate reads.
    void update_from_q4(std::uint32_t q4, std::uint32_t now_tick,
                        bool sensor_fault = false) noexcept;

    // The same for a plain 12-bit code: update_from_q4(raw << 4).
    void update_from_adc(std::uint16_t raw, std::uint32_t now_tick,
                         bool sensor_fault = false) noexcept;

    // Atomic read of the full state.
    [[nodiscard]] CurrentState snapshot() const noexcept;

    // Pure helpers, exposed for unit testing. Static -> no mutex.
    //
    // adc_to_mA: maps a 12-bit DIFFERENTIAL ADC reading (0..4095, with
    // zero-current at CurrentZeroCount ~= 2048) to a signed pack current
    // in mA. The differential LSB is 2*Vref/4095 (twice the single-ended
    // LSB), and sensitivity is the bare-sensor CurrentMvPerAmpe1.
    static std::int32_t adc_to_mA(std::uint16_t raw) noexcept;

    // adc_q4_to_mA: the same mapping for an oversampled differential code
    // carrying CurrentAdcFracBits fractional bits (16 x the 12-bit code), as
    // ADC3 delivers it. adc_to_mA(raw) == adc_q4_to_mA(raw << 4) exactly; the
    // extra bits keep the resolution the oversampling bought (~22 mA instead
    // of ~350 mA per step) for the ELE log.
    static std::int32_t adc_q4_to_mA(std::uint32_t q4) noexcept;

    // Q4 code -> nearest 12-bit code, for the paths that take one.
    static constexpr std::uint16_t q4_to_raw(std::uint32_t q4) noexcept {
        return static_cast<std::uint16_t>((q4 + (1u << (config::CurrentAdcFracBits - 1u)))
                                          >> config::CurrentAdcFracBits);
    }

    // leg_voltage_plausible: true iff a SINGLE-ENDED reading of the
    // OUT_P leg (PF7 / ADC3_INP3) sits inside [CurrentLegPlausMinMv,
    // CurrentLegPlausMaxMv]. A connected SSA-2 holds OUT_P near the
    // common-mode +/- the half-swing; a disconnect (with the internal
    // pull-down) collapses it toward 0 V -> implausible -> the caller
    // debounces this into CurrentState.sensor_fault.
    static bool leg_voltage_plausible(std::uint16_t raw) noexcept;

private:
    CurrentService() = default;
    mutable CurrentState state_ = {};
};

}  // namespace ams
