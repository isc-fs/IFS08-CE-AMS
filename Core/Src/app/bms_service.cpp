// SPDX-License-Identifier: proprietary

#include "bms_service.hpp"

#include "ams_config.hpp"
#include "scoped_mutex.hpp"

#include "cmsis_os2.h"

#include <algorithm>
#include <cstdint>
#include <limits>

extern "C" osMutexId_t bms_mutexHandle;  // created in main.c

namespace ams {

namespace {

// All BMS frame IDs sit between base and base + 5 * kBmsModuleStride.
// The two "panes" (voltage 0x12D..0x131 + per-module offset, and
// temperature 0x14D..0x151 + per-module offset) are 0x14D - 0x12C = 33
// = 0x21 apart from the voltage poll base. We work in offsets from
// kBmsVoltPollBase (0x12C) throughout, mapping to (module, kind, idx).
constexpr std::uint16_t kVoltRespBaseOffset = config::kBmsVoltRespBase - config::kBmsVoltPollBase;  // 0x01
constexpr std::uint16_t kTempRespBaseOffset = config::kBmsTempRespBase - config::kBmsVoltPollBase;  // 0x21
constexpr std::uint8_t  kFramesPerPane      = 5;

enum class FrameKind : std::uint8_t { Unknown, Voltage, Temperature };

struct Decoded {
    FrameKind   kind;
    std::uint8_t module;
    std::uint8_t frame_idx;
};

Decoded classify(std::uint32_t id) noexcept {
    if (id < config::kBmsVoltPollBase) return {FrameKind::Unknown, 0, 0};

    const std::uint16_t off = static_cast<std::uint16_t>(id - config::kBmsVoltPollBase);

    const std::uint8_t module = static_cast<std::uint8_t>(off / config::kBmsModuleStride);
    if (module >= config::kBmsModuleCount) return {FrameKind::Unknown, 0, 0};

    const std::uint16_t in_pane = off % config::kBmsModuleStride;

    if (in_pane >= kVoltRespBaseOffset &&
        in_pane <  kVoltRespBaseOffset + kFramesPerPane) {
        return {FrameKind::Voltage, module,
                static_cast<std::uint8_t>(in_pane - kVoltRespBaseOffset)};
    }
    if (in_pane >= kTempRespBaseOffset &&
        in_pane <  kTempRespBaseOffset + kFramesPerPane) {
        return {FrameKind::Temperature, module,
                static_cast<std::uint8_t>(in_pane - kTempRespBaseOffset)};
    }
    return {FrameKind::Unknown, 0, 0};
}

}  // namespace

BmsService& BmsService::instance() noexcept {
    static BmsService kInstance;
    return kInstance;
}

BmsService::BmsService() {
    // Initialise summaries to neutral values so SafetyTask reads
    // sensible numbers before the first frame lands.
    state_.min_cell_mV = std::numeric_limits<std::uint16_t>::max();
    state_.max_cell_mV = 0;
    state_.min_tempC   = std::numeric_limits<std::int16_t>::max();
    state_.max_tempC   = std::numeric_limits<std::int16_t>::min();
}

void BmsService::parse_voltage_frame_(std::uint8_t module, std::uint8_t frame_idx,
                                      const std::uint8_t *data) noexcept {
    // 4 cells per frame, big-endian uint16 mV. Last frame (idx 4)
    // covers 3 cells (0..18), bytes 6-7 unused.
    const std::uint8_t base_cell = static_cast<std::uint8_t>(frame_idx * 4);
    const std::uint8_t cells_this_frame =
        (base_cell + 4u > config::kCellsPerModule)
            ? static_cast<std::uint8_t>(config::kCellsPerModule - base_cell)
            : 4u;

    for (std::uint8_t k = 0; k < cells_this_frame; ++k) {
        const std::uint16_t mv = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(data[2u * k]) << 8) |
             static_cast<std::uint16_t>(data[2u * k + 1u]));
        state_.cell_mV[module][base_cell + k] = mv;
    }
}

void BmsService::parse_temperature_frame_(std::uint8_t module, std::uint8_t frame_idx,
                                          const std::uint8_t *data) noexcept {
    // 8 temps per frame, signed int8 °C. Last frame (idx 4) covers
    // 6 temps (32..37); bytes 6-7 unused.
    const std::uint8_t base_temp = static_cast<std::uint8_t>(frame_idx * 8);
    const std::uint8_t temps_this_frame =
        (base_temp + 8u > config::kTempsPerModule)
            ? static_cast<std::uint8_t>(config::kTempsPerModule - base_temp)
            : 8u;

    for (std::uint8_t k = 0; k < temps_this_frame; ++k) {
        state_.cell_tempC[module][base_temp + k] =
            static_cast<std::int16_t>(static_cast<std::int8_t>(data[k]));
    }
}

void BmsService::recompute_summaries_() noexcept {
    std::uint32_t sum_v_mV = 0;
    std::uint16_t min_mV   = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_mV   = 0;
    std::int16_t  min_t    = std::numeric_limits<std::int16_t>::max();
    std::int16_t  max_t    = std::numeric_limits<std::int16_t>::min();
    std::int32_t  sum_t    = 0;
    std::uint32_t n_t      = 0;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        if ((state_.module_online_mask & (1u << m)) == 0u) continue;

        for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
            const std::uint16_t v = state_.cell_mV[m][c];
            sum_v_mV += v;
            if (v < min_mV) min_mV = v;
            if (v > max_mV) max_mV = v;
        }
        for (std::uint8_t t = 0; t < config::kTempsPerModule; ++t) {
            const std::int16_t tc = state_.cell_tempC[m][t];
            if (tc < min_t) min_t = tc;
            if (tc > max_t) max_t = tc;
            sum_t += tc;
            ++n_t;
        }
    }

    state_.pack_voltage_mV = sum_v_mV;
    state_.min_cell_mV     = min_mV;
    state_.max_cell_mV     = max_mV;
    state_.min_tempC       = min_t;
    state_.max_tempC       = max_t;
    state_.avg_tempC       = (n_t > 0) ? static_cast<std::int16_t>(sum_t / static_cast<std::int32_t>(n_t)) : 0;
}

bool BmsService::update_from_frame(const CanFrame& f) noexcept {
    if (f.bus != static_cast<std::uint8_t>(CanBus::Bms)) return false;

    const Decoded d = classify(f.id);
    if (d.kind == FrameKind::Unknown) return false;
    if (f.dlc < 8) return false;  // every BMS data frame is 8 bytes

    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return false;

    if (d.kind == FrameKind::Voltage) {
        parse_voltage_frame_(d.module, d.frame_idx, f.data);
    } else {
        parse_temperature_frame_(d.module, d.frame_idx, f.data);
    }

    state_.last_rx_tick[d.module] = f.timestamp_ms;
    state_.module_online_mask     = static_cast<std::uint8_t>(
        state_.module_online_mask | (1u << d.module));

    recompute_summaries_();
    return true;
}

BmsState BmsService::snapshot() const noexcept {
    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return state_;  // best-effort; caller checks freshness
    return state_;
}

bool BmsService::is_healthy(std::uint32_t now_tick) const noexcept {
    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return false;

    if (state_.module_online_mask != config::kAllModulesMask) return false;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        if (now_tick - state_.last_rx_tick[m] > config::kBmsStaleMs) return false;
    }
    return true;
}

}  // namespace ams
