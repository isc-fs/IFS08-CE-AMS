// SPDX-License-Identifier: proprietary
//
// Host tests for the ungated firmware-health frame (0x6CA, #411): the
// reset-cause priority logic and the ECU-0x704 byte layout. Both are
// pure/header-only, so no HAL or FreeRTOS is linked here.

#include "unity.h"

#include "fw_health.hpp"
#include "pit_diag_emitter.hpp"

using ams::config::ResetCause;
namespace fwh = ams::fw_health;

static std::uint8_t rc(bool por, bool pin, bool sw, bool iwdg, bool wwdg, bool lp) {
    return static_cast<std::uint8_t>(fwh::map_reset_cause(por, pin, sw, iwdg, wwdg, lp));
}

// Priority: iwdg > wwdg > lowpower > software > por > pin; none -> Unknown.
extern "C" void test_fw_health_reset_cause_priority(void) {
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::PowerOn,  rc(true,  false, false, false, false, false));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Pin,      rc(false, true,  false, false, false, false));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Software, rc(false, false, true,  false, false, false));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Iwdg,     rc(false, false, false, true,  false, false));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Wwdg,     rc(false, false, false, false, true,  false));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::LowPower, rc(false, false, false, false, false, true));
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Unknown,  rc(false, false, false, false, false, false));

    // A cold boot asserts POR *and* PIN -> PowerOn must win.
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::PowerOn, rc(true, true, false, false, false, false));
    // An IWDG reset also asserts PIN -> IWDG must win (the specific cause).
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Iwdg,    rc(false, true, false, true, false, false));
    // Software reset also asserts PIN -> Software must win.
    TEST_ASSERT_EQUAL_UINT8((std::uint8_t)ResetCause::Software, rc(false, true, true, false, false, false));
}

// ECU-0x704 byte layout: heap fields big-endian, the rest single bytes.
extern "C" void test_fw_health_encode_layout(void) {
    const auto f = ams::pit_diag::encode_fw_health(
        /*free_heap*/ 0x1234u, /*min_free_heap*/ 0x0ABCu,
        /*task_liveness*/ 0x0Fu, /*reset_cause*/ 3u,
        /*uptime_s*/ 42u, /*last_fault*/ 1u);
    TEST_ASSERT_EQUAL_UINT8(0x12u, f[0]);  // free_heap BE hi
    TEST_ASSERT_EQUAL_UINT8(0x34u, f[1]);  // free_heap BE lo
    TEST_ASSERT_EQUAL_UINT8(0x0Au, f[2]);  // min_free_heap BE hi
    TEST_ASSERT_EQUAL_UINT8(0xBCu, f[3]);  // min_free_heap BE lo
    TEST_ASSERT_EQUAL_UINT8(0x0Fu, f[4]);  // task_liveness
    TEST_ASSERT_EQUAL_UINT8(3u,    f[5]);  // reset_cause
    TEST_ASSERT_EQUAL_UINT8(42u,   f[6]);  // uptime_s
    TEST_ASSERT_EQUAL_UINT8(1u,    f[7]);  // last_fault
}

// Heap values above u16 clamp to 0xFFFF rather than wrapping silently.
extern "C" void test_fw_health_encode_clamps_heap(void) {
    const auto f = ams::pit_diag::encode_fw_health(0x1FFFFu, 0x20000u, 0u, 0u, 0u, 0u);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[3]);
}
