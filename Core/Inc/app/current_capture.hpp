// SPDX-License-Identifier: proprietary
//
// Reduction of one pack-current capture -- the oversampled ADC3 samples the
// DMA wrote during one CurrentPeriodMs cycle -- into the 10 ms windows of
// ELEnnnn.BIN, and the helpers CurrentSensorTask uses around it.
//
// Samples are Q4 differential codes (config::CurrentAdcFracBits): 16 x the
// 12-bit code, each the mean of 64 conversions. Nothing here assumes the
// sample rate: windows split the capture by sample count, and their end times
// are interpolated over the capture's measured start and stop ticks.
//
// TELEMETRY ONLY, except capture_mean_q4(), the one value per cycle that
// CurrentService (and through it the safety predicates) receives.
// Pure (HAL-free, RTOS-free) so the host tests cover it.

#pragma once

#include "ams_config.hpp"
#include "bin_log.hpp"
#include "current_service.hpp"

#include <cstddef>
#include <cstdint>

namespace ams::current_capture {

struct Window {
    std::uint16_t count = 0;
    std::uint32_t sum   = 0;        // of Q4 codes; 1024 x 65535 fits easily
    std::uint16_t min   = 0xFFFFu;
    std::uint16_t max   = 0;
};

// First sample of window i when n samples are split into k windows. Window i
// covers [begin(n, k, i), begin(n, k, i + 1)), so the windows tile the capture
// exactly and differ in length by at most one sample.
[[nodiscard]] inline constexpr std::uint16_t window_begin(std::uint16_t n, std::uint8_t k,
                                                          std::uint8_t i) noexcept {
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(n) * i) / k);
}

[[nodiscard]] inline Window reduce(const std::uint16_t* s, std::uint16_t begin,
                                   std::uint16_t end) noexcept {
    Window w;
    for (std::uint16_t i = begin; i < end; ++i) {
        const std::uint16_t v = s[i];
        w.sum += v;
        if (v < w.min) w.min = v;
        if (v > w.max) w.max = v;
    }
    w.count = static_cast<std::uint16_t>(end > begin ? end - begin : 0u);
    return w;
}

// Rounded mean of a window, still in Q4. 0 for an empty window.
[[nodiscard]] inline constexpr std::uint32_t mean_q4(const Window& w) noexcept {
    return w.count == 0u ? 0u : (w.sum + w.count / 2u) / w.count;
}

// Tick at sample index `idx` of a capture that ran from t_start to t_stop and
// holds n samples: linear interpolation, wrap-safe.
[[nodiscard]] inline constexpr std::uint32_t tick_at(std::uint32_t t_start, std::uint32_t t_stop,
                                                     std::uint16_t idx, std::uint16_t n) noexcept {
    return n == 0u ? t_stop
                   : t_start + static_cast<std::uint32_t>(
                                   (static_cast<std::uint64_t>(t_stop - t_start) * idx) / n);
}

// Mean of a whole capture, in Q4 -- the one value per cycle CurrentService
// receives. The ADC integrates continuously, so this is the average current
// over the cycle: every amp-second counts, where a single sample would catch
// or miss a pulse by chance. 0 for an empty capture (callers skip those).
[[nodiscard]] inline std::uint32_t capture_mean_q4(const std::uint16_t* s,
                                                   std::uint16_t n) noexcept {
    return mean_q4(reduce(s, 0, n));
}

[[nodiscard]] inline bin_log::EleRecord make_record(const Window& w, std::uint32_t tick_ms,
                                                    std::uint16_t seq, std::uint8_t flags,
                                                    std::uint16_t dcbus_V,
                                                    std::uint16_t dcbus_age_ms) noexcept {
    bin_log::EleRecord r{};
    r.tick_ms      = tick_ms;
    r.seq          = seq;
    r.n            = static_cast<std::uint8_t>(w.count > 255u ? 255u : w.count);
    r.flags        = flags;
    r.i_mean_mA    = CurrentService::adc_q4_to_mA(mean_q4(w));
    r.i_min_mA     = CurrentService::adc_q4_to_mA(w.min);
    r.i_max_mA     = CurrentService::adc_q4_to_mA(w.max);
    r.dcbus_V      = dcbus_V;
    r.dcbus_age_ms = dcbus_age_ms;
    return r;
}

}  // namespace ams::current_capture
