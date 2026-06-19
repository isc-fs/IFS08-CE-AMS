// SPDX-License-Identifier: proprietary
//
// Tests for ams::can_recovery::should_attempt_recovery -- the pure
// rate-limiter half of the FDCAN1 Bus-Off recovery path. The HAL side
// (GetProtocolStatus / Stop / Start + the State/ErrorCode unwedge) resets
// the peripheral, so we test the decision logic only -- the same split as
// test_bootloader.cpp does for the jump path.
//
// The policy mirrors the bootloader's Bootloader_FdcanBusOffRecover:
// attempt on the transition INTO Bus_Off, then at most once per retry
// window while it persists, and immediately again after a healthy poll.

#include "can_busoff_recovery.hpp"

#include "unity.h"

#include <cstdint>

namespace {

constexpr std::uint32_t kRetryMs = 100;  // matches config::FdcanBusOffRetryMs

using ams::can_recovery::BusOffState;
using ams::can_recovery::should_attempt_recovery;

}  // namespace

// A healthy bus never triggers a recovery, at any tick.
extern "C" void test_busoff_healthy_bus_never_attempts(void) {
    BusOffState st{};
    TEST_ASSERT_FALSE(should_attempt_recovery(st, false, 0,    kRetryMs));
    TEST_ASSERT_FALSE(should_attempt_recovery(st, false, 1000, kRetryMs));
    TEST_ASSERT_FALSE(should_attempt_recovery(st, false, 5000, kRetryMs));
    TEST_ASSERT_FALSE(st.was_busoff);
}

// The very first poll that observes Bus_Off acts immediately (the
// healthy->off edge), even from a freshly value-initialised latch.
extern "C" void test_busoff_first_detection_attempts_immediately(void) {
    BusOffState st{};
    TEST_ASSERT_TRUE(should_attempt_recovery(st, true, 4242, kRetryMs));
    TEST_ASSERT_TRUE(st.was_busoff);
    TEST_ASSERT_EQUAL_UINT32(4242, st.last_attempt_ms);
}

// While Bus_Off persists, polls inside the retry window are suppressed so
// the M_CAN's automatic rejoin isn't restarted.
extern "C" void test_busoff_sustained_is_rate_limited(void) {
    BusOffState st{};
    TEST_ASSERT_TRUE (should_attempt_recovery(st, true, 1000, kRetryMs));  // edge
    TEST_ASSERT_FALSE(should_attempt_recovery(st, true, 1001, kRetryMs));
    TEST_ASSERT_FALSE(should_attempt_recovery(st, true, 1050, kRetryMs));
    TEST_ASSERT_FALSE(should_attempt_recovery(st, true, 1099, kRetryMs));
    // last_attempt stays pinned to the edge while suppressed.
    TEST_ASSERT_EQUAL_UINT32(1000, st.last_attempt_ms);
}

// Exactly retry_ms after the last attempt, the next poll re-attempts
// (the window is "< retry_ms", so the boundary tick acts).
extern "C" void test_busoff_boundary_exactly_retry_ms_attempts(void) {
    BusOffState st{};
    TEST_ASSERT_TRUE(should_attempt_recovery(st, true, 1000, kRetryMs));
    TEST_ASSERT_FALSE(should_attempt_recovery(st, true, 1099, kRetryMs));  // 99 < 100
    TEST_ASSERT_TRUE (should_attempt_recovery(st, true, 1100, kRetryMs));  // 100 !< 100
    TEST_ASSERT_EQUAL_UINT32(1100, st.last_attempt_ms);  // advanced
}

// Persistent Bus_Off yields a steady cadence: one attempt per window,
// each advancing the reference point.
extern "C" void test_busoff_steady_cadence_of_attempts(void) {
    BusOffState st{};
    std::uint32_t attempts = 0;
    for (std::uint32_t t = 0; t <= 1000; t += 10) {       // 10 ms poll, 1 s of Bus_Off
        if (should_attempt_recovery(st, true, t, kRetryMs)) ++attempts;
    }
    // t=0 (edge) + 100,200,...,1000 -> 1 + 10 = 11 attempts.
    TEST_ASSERT_EQUAL_UINT32(11, attempts);
}

// A healthy poll clears the latch, so a subsequent re-fault acts
// immediately again -- even if it lands inside what would have been the
// previous retry window.
extern "C" void test_busoff_recovers_then_refaults_attempts_immediately(void) {
    BusOffState st{};
    TEST_ASSERT_TRUE (should_attempt_recovery(st, true,  1000, kRetryMs));  // edge
    TEST_ASSERT_FALSE(should_attempt_recovery(st, false, 1010, kRetryMs));  // bus healed
    TEST_ASSERT_FALSE(st.was_busoff);
    // Re-fault only 20 ms after the first attempt: still immediate,
    // because the healthy poll reset the edge latch.
    TEST_ASSERT_TRUE (should_attempt_recovery(st, true,  1020, kRetryMs));
    TEST_ASSERT_EQUAL_UINT32(1020, st.last_attempt_ms);
}

// Modular tick subtraction keeps the spacing correct across the 32-bit
// rollover (49.7 days at the 1 kHz FreeRTOS tick).
extern "C" void test_busoff_tick_wrap_preserves_spacing(void) {
    BusOffState st{};
    const std::uint32_t near_max = 0xFFFFFFF0u;  // 16 ticks before wrap
    TEST_ASSERT_TRUE(should_attempt_recovery(st, true, near_max, kRetryMs));

    // 50 ms later, having wrapped past 0: (now - last) == 50 via modular
    // arithmetic -> still inside the window -> suppressed.
    const std::uint32_t after_50 = near_max + 50u;  // wraps to 0x00000022
    TEST_ASSERT_FALSE(should_attempt_recovery(st, true, after_50, kRetryMs));

    // 100 ms after the edge, also wrapped -> boundary reached -> attempt.
    const std::uint32_t after_100 = near_max + 100u;  // wraps to 0x00000054
    TEST_ASSERT_TRUE(should_attempt_recovery(st, true, after_100, kRetryMs));
    TEST_ASSERT_EQUAL_UINT32(after_100, st.last_attempt_ms);
}
