// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for CurrentService::adc_to_mA(). The mutex-using
// public methods are exercised indirectly via the cmsis_os2 mock from
// tests/unit/mocks/.

#include "ams_config.hpp"
#include "current_service.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdlib>

// ---------------------------------------------------------------------------
// adc_to_mA: zero-current calibration point reads (almost) 0 mA.
// Sensor zero is 2.500 V = ADC count round((2500/3300)*4095) = 3102.
// Plug that back through the conversion and confirm |mA| is small.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_zero_point_reads_near_zero(void) {
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (ams::config::kCurrentZeroMv *
         static_cast<std::int32_t>(ams::config::kAdcMaxCount)) /
        ams::config::kAdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(200, std::abs(mA));  // within 200 mA of zero
}

// ---------------------------------------------------------------------------
// adc_to_mA: discharge (current OUT of pack) makes voltage drop below
// 2.5 V, which yields positive mA. Pick raw = 2.4 V -> +100 mV delta
// -> ~+17.5 A (100 mV / 5.7 mV/A).
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_discharge_positive(void) {
    const std::int32_t target_mv = ams::config::kCurrentZeroMv - 100;
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (target_mv * static_cast<std::int32_t>(ams::config::kAdcMaxCount)) /
        ams::config::kAdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_GREATER_THAN_INT32(15000, mA);
    TEST_ASSERT_LESS_THAN_INT32   (20000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: charging makes voltage rise above 2.5 V, yielding negative mA.
// Pick raw = 2.6 V -> -100 mV delta -> ~-17.5 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_charge_negative(void) {
    const std::int32_t target_mv = ams::config::kCurrentZeroMv + 100;
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (target_mv * static_cast<std::int32_t>(ams::config::kAdcMaxCount)) /
        ams::config::kAdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_LESS_THAN_INT32   (-15000, mA);
    TEST_ASSERT_GREATER_THAN_INT32(-20000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA is symmetric around the zero point.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_symmetric_around_zero(void) {
    const std::int32_t zero_count =
        (ams::config::kCurrentZeroMv *
         static_cast<std::int32_t>(ams::config::kAdcMaxCount)) /
        ams::config::kAdcVrefMv;

    const auto plus_50  = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero_count + 50));
    const auto minus_50 = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero_count - 50));

    // Symmetry: plus and minus should mirror within rounding. The
    // integer-uV math leaves a ~120 mA bias around the zero point
    // (the truncation in v_uV = raw * Vref / 4095 is asymmetric for
    // a non-power-of-two ADC max). Sensor noise floor is well above
    // that, so this tolerance is meaningful only as a "no gross
    // arithmetic error" check.
    TEST_ASSERT_INT32_WITHIN(200, -plus_50, minus_50);
}
