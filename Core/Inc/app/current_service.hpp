// SPDX-License-Identifier: proprietary
//
// Pack current measurement service. Single-writer (CurrentSensorTask via
// update_from_adc); many readers (SafetyTask, StateTask, AcuCanTask,
// TelemetryTask). Synchronisation: docs/ARCHITECTURE.md §3.

#pragma once

#include "ams_config.hpp"

#include <cstdint>

namespace ams {

struct CurrentState {
    // Signed mA; convention is `+ = discharge, - = charge`.
    std::int32_t  raw_mA;        // single-sample, no filter
    std::int32_t  filtered_mA;   // IIR low-pass, tau ~ 16 samples
    std::uint32_t last_update_tick;
    bool          charger_detected;  // set by AcuCanTask (feat/12)
    bool          sensor_fault;      // ADC failed to convert, or out of plausible range
};

class CurrentService {
public:
    static CurrentService& instance() noexcept;

    // Called by CurrentSensorTask only. Converts raw ADC counts to mA,
    // updates the filter, refreshes the timestamp.
    void update_from_adc(std::uint16_t raw, std::uint32_t now_tick) noexcept;

    // Called by AcuCanTask on charger-detected CAN frame.
    void set_charger_detected(bool detected) noexcept;

    // Atomic read of the full state.
    [[nodiscard]] CurrentState snapshot() const noexcept;

    // True iff the last update was within kIStaleMs and no sensor
    // fault is latched. Consumed by SafetyTask.
    [[nodiscard]] bool is_healthy(std::uint32_t now_tick) const noexcept;

    // Pure helper, exposed for unit testing. Maps a 12-bit ADC reading
    // (0..4095 referenced to Vref) to a signed current in mA using
    // the constants in ams_config. Static -> no mutex.
    static std::int32_t adc_to_mA(std::uint16_t raw) noexcept;

private:
    CurrentService() = default;
    mutable CurrentState state_ = {};
};

}  // namespace ams
