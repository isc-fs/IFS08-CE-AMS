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
    std::int32_t  raw_mA;        // single-sample, no filter
    std::int32_t  filtered_mA;   // IIR low-pass, tau ~ 16 samples
    std::uint32_t last_update_tick;
    bool          sensor_fault;      // ADC failed to convert, or out of plausible range

    // DCDC current (fix/53). Sourced from PC1 / ADC3_INP11, a separate
    // single-ended sensor (own calibration, adc_to_mA_dcdc). The ECU
    // forwards both via 0x135. last_dcdc_update_tick is independent
    // from the pack tick so a DCDC ADC failure doesn't stale the
    // safety predicate that watches the pack channel.
    std::int32_t  dcdc_raw_mA;
    std::int32_t  dcdc_filtered_mA;
    std::uint32_t last_dcdc_update_tick;
    bool          dcdc_sensor_fault;
};

class CurrentService {
public:
    static CurrentService& instance() noexcept;

    // Called by CurrentSensorTask only. Converts raw ADC counts to mA,
    // updates the filter, refreshes the timestamp. Mirrors for the
    // DCDC channel are independent (separate filter state, separate
    // freshness tick).
    void update_from_adc(std::uint16_t raw, std::uint32_t now_tick) noexcept;
    void update_dcdc_from_adc(std::uint16_t raw, std::uint32_t now_tick) noexcept;

    // Atomic read of the full state.
    [[nodiscard]] CurrentState snapshot() const noexcept;

    // DCDC freshness probe. Informational only -- the DCDC channel is
    // not part of the safety predicate (DCDC failure is recoverable,
    // unlike pack-current sensor failure which has to latch Error).
    // Available for future telemetry surfaces / diagnostic frames.
    // True iff the last DCDC ADC update landed within DcdcIStaleMs
    // (500 ms) of now_tick.
    [[nodiscard]] bool is_dcdc_fresh(std::uint32_t now_tick) const noexcept;

    // Pure helpers, exposed for unit testing. Static -> no mutex.
    //
    // adc_to_mA: maps a 12-bit DIFFERENTIAL ADC reading (0..4095, with
    // zero-current at CurrentZeroCount ~= 2048) to a signed pack current
    // in mA. The differential LSB is 2*Vref/4095 (twice the single-ended
    // LSB), and sensitivity is the bare-sensor CurrentMvPerAmpe1.
    static std::int32_t adc_to_mA(std::uint16_t raw) noexcept;

    // adc_to_mA_dcdc: maps a 12-bit SINGLE-ENDED ADC reading (PC1 /
    // ADC3_INP11) to a signed DCDC current in mA, using the DcdcCurrent*
    // constants (zero at DcdcCurrentZeroMv, sensitivity
    // DcdcCurrentMvPerAmpe1). Informational only.
    static std::int32_t adc_to_mA_dcdc(std::uint16_t raw) noexcept;

private:
    CurrentService() = default;
    mutable CurrentState state_ = {};
};

}  // namespace ams
