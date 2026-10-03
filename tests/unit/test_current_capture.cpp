// SPDX-License-Identifier: proprietary
//
// Tests for current_capture.hpp -- splitting one oversampled pack-current
// capture into ELE windows, the window statistics, timestamp interpolation and
// the per-cycle mean that reaches CurrentService.

#include "ams_config.hpp"
#include "bin_log.hpp"
#include "current_capture.hpp"
#include "current_service.hpp"

#include "unity.h"

#include <cstdint>
#include <initializer_list>

namespace {
using namespace ams;
namespace cc = ams::current_capture;
constexpr std::uint8_t K = config::EleWindowsPerCycle;
}  // namespace

// Windows tile the capture exactly, in order, and differ by at most one sample,
// whatever the sample count -- nominal, short (the disconnect-check pause) or
// a late, overrun capture.
extern "C" void test_capture_windows_tile_the_capture(void) {
    for (std::uint16_t n : {std::uint16_t{625}, std::uint16_t{623}, std::uint16_t{1},
                            std::uint16_t{4}, config::CurrentCaptureCapacity}) {
        TEST_ASSERT_EQUAL_UINT16(0u, cc::window_begin(n, K, 0));
        TEST_ASSERT_EQUAL_UINT16(n, cc::window_begin(n, K, K));
        std::uint16_t lo = 0xFFFFu, hi = 0;
        for (std::uint8_t i = 0; i < K; ++i) {
            const std::uint16_t len = static_cast<std::uint16_t>(
                cc::window_begin(n, K, static_cast<std::uint8_t>(i + 1u)) - cc::window_begin(n, K, i));
            if (len < lo) lo = len;
            if (len > hi) hi = len;
        }
        TEST_ASSERT_TRUE(hi - lo <= 1u);
    }
    TEST_ASSERT_EQUAL_UINT16(125u, cc::window_begin(625, K, 1));   // nominal 12.5 kHz: 10 ms = 125
}

extern "C" void test_capture_reduce_stats(void) {
    const std::uint16_t s[] = {100, 300, 200, 50, 400, 999};
    const cc::Window w = cc::reduce(s, 1, 5);   // 300, 200, 50, 400
    TEST_ASSERT_EQUAL_UINT16(4u, w.count);
    TEST_ASSERT_EQUAL_UINT32(950u, w.sum);
    TEST_ASSERT_EQUAL_UINT16(50u, w.min);
    TEST_ASSERT_EQUAL_UINT16(400u, w.max);
    TEST_ASSERT_EQUAL_UINT32(238u, cc::mean_q4(w));   // 237.5 rounds up

    const cc::Window empty = cc::reduce(s, 3, 3);
    TEST_ASSERT_EQUAL_UINT16(0u, empty.count);
    TEST_ASSERT_EQUAL_UINT32(0u, cc::mean_q4(empty));
}

// End-of-window ticks are spread over the capture's measured duration, and the
// subtraction is wrap-safe across the 32-bit tick rollover.
extern "C" void test_capture_tick_interpolation(void) {
    TEST_ASSERT_EQUAL_UINT32(1010u, cc::tick_at(1000, 1050, 125, 625));
    TEST_ASSERT_EQUAL_UINT32(1050u, cc::tick_at(1000, 1050, 625, 625));
    TEST_ASSERT_EQUAL_UINT32(1000u, cc::tick_at(1000, 1050, 0, 625));
    const std::uint32_t start = 0xFFFFFFF0u;           // 16 ms before the wrap
    const std::uint32_t stop  = start + 50u;           // 34 ms after it
    TEST_ASSERT_EQUAL_UINT32(stop, cc::tick_at(start, stop, 625, 625));
    TEST_ASSERT_EQUAL_UINT32(start + 10u, cc::tick_at(start, stop, 125, 625));
    TEST_ASSERT_EQUAL_UINT32(stop, cc::tick_at(start, stop, 0, 0));
}

// A steady current: the capture mean is that current, exactly.
extern "C" void test_capture_mean_steady(void) {
    std::uint16_t s[625];
    for (auto& v : s) v = static_cast<std::uint16_t>(16u * 2100u + 5u);
    TEST_ASSERT_EQUAL_UINT32(16u * 2100u + 5u, cc::capture_mean_q4(s, 625));
}

// A pulse shorter than the cycle enters the safety path as its charge spread
// over the cycle: ~600 A for 62 of 625 samples (~5 ms of 50 ms) is ~60 A, not
// 600 A or 0 A depending on where a single sample happened to land.
extern "C" void test_capture_mean_counts_a_short_pulse(void) {
    const std::uint32_t zero = std::uint32_t{config::CurrentZeroCount} << config::CurrentAdcFracBits;
    std::uint32_t pulse = zero;
    while (CurrentService::adc_q4_to_mA(pulse) < 600000) ++pulse;    // first Q4 code >= 600 A
    std::uint16_t s[625];
    for (std::uint16_t i = 0; i < 625; ++i) {
        s[i] = static_cast<std::uint16_t>((i >= 300 && i < 362) ? pulse : zero);
    }
    const std::int32_t mA = CurrentService::adc_q4_to_mA(cc::capture_mean_q4(s, 625));
    TEST_ASSERT_INT32_WITHIN(500, 600000 * 62 / 625, mA);
    TEST_ASSERT_EQUAL_UINT32(0u, cc::capture_mean_q4(s, 0));
}

extern "C" void test_capture_make_record(void) {
    const std::uint32_t zero = std::uint32_t{config::CurrentZeroCount} << config::CurrentAdcFracBits;
    cc::Window w;
    w.count = 300;   // above the u8 field: clamps
    w.sum   = 300u * (zero + 160u);
    w.min   = static_cast<std::uint16_t>(zero - 1600u);
    w.max   = static_cast<std::uint16_t>(zero + 16000u);
    const bin_log::EleRecord r = cc::make_record(w, 12345u, 7u, bin_log::ele_flag::Overrun, 351u, 12u);
    TEST_ASSERT_EQUAL_UINT32(12345u, r.tick_ms);
    TEST_ASSERT_EQUAL_UINT16(7u, r.seq);
    TEST_ASSERT_EQUAL_UINT8(255u, r.n);
    TEST_ASSERT_EQUAL_UINT8(bin_log::ele_flag::Overrun, r.flags);
    TEST_ASSERT_EQUAL_INT32(CurrentService::adc_q4_to_mA(zero + 160u), r.i_mean_mA);
    TEST_ASSERT_EQUAL_INT32(CurrentService::adc_to_mA(config::CurrentZeroCount - 100), r.i_min_mA);
    TEST_ASSERT_EQUAL_INT32(CurrentService::adc_to_mA(config::CurrentZeroCount + 1000), r.i_max_mA);
    TEST_ASSERT_TRUE(r.i_min_mA < 0 && r.i_mean_mA > 0 && r.i_max_mA > r.i_mean_mA);
    TEST_ASSERT_EQUAL_UINT16(351u, r.dcbus_V);
    TEST_ASSERT_EQUAL_UINT16(12u, r.dcbus_age_ms);
}
