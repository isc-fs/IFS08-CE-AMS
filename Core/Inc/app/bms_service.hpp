// SPDX-License-Identifier: proprietary
//
// Aggregates per-module cell voltages and temperatures from the 5 BMS
// slaves on FDCAN2.  Single-writer (BmsRxTask, feat/9); many readers
// (SafetyTask, StateTask, AcuCanTask, TelemetryTask).
//
// Wire protocol: docs/CAN_MAP.md §"RX -- BMS slaves to AMS".
// Synchronisation: docs/ARCHITECTURE.md §3 ownership map.

#pragma once

#include "ams_config.hpp"
#include "can_frame.hpp"

#include <cstdint>

namespace ams {

struct BmsState {
    std::uint16_t cell_mV     [config::kBmsModuleCount][config::kCellsPerModule];
    std::int16_t  cell_tempC  [config::kBmsModuleCount][config::kTempsPerModule];

    // Derived summaries (recomputed on each frame; cheap, < 200 cells).
    std::uint32_t pack_voltage_mV;
    std::uint16_t min_cell_mV;
    std::uint16_t max_cell_mV;
    std::int16_t  min_tempC;
    std::int16_t  max_tempC;
    std::int16_t  avg_tempC;

    // Per-module freshness for the SafetyTask staleness check.
    std::uint32_t last_rx_tick[config::kBmsModuleCount];

    // Bit N set <=> module N has reported at least once. Compared
    // against config::kAllModulesMask (0x1F) by SafetyTask.
    std::uint8_t  module_online_mask;
};

class BmsService {
public:
    static BmsService& instance() noexcept;

    // Called by BmsRxTask only. Parses the frame against the protocol
    // in docs/CAN_MAP.md and updates the relevant slice of state under
    // the mutex. Returns true if the frame matched a known voltage or
    // temperature ID; false if it was unrecognised (in which case
    // nothing was written).
    bool update_from_frame(const CanFrame& f) noexcept;

    // Atomic read of the full state. Caller gets its own copy; the
    // mutex is released before this returns.
    [[nodiscard]] BmsState snapshot() const noexcept;

    // True iff all 5 modules have reported within kBmsStaleMs and
    // module_online_mask covers them. Used by SafetyTask.
    [[nodiscard]] bool is_healthy(std::uint32_t now_tick) const noexcept;

private:
    BmsService();

    // Internal helpers; assume the mutex is already held.
    void parse_voltage_frame_(std::uint8_t module, std::uint8_t frame_idx,
                              const std::uint8_t *data) noexcept;
    void parse_temperature_frame_(std::uint8_t module, std::uint8_t frame_idx,
                                  const std::uint8_t *data) noexcept;
    void recompute_summaries_() noexcept;

    mutable BmsState state_ = {};
};

}  // namespace ams
