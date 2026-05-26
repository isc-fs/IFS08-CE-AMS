// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for CurrentService::adc_to_mA(). The mutex-using
// public methods are exercised indirectly via the cmsis_os2 mock from
// tests/unit/mocks/.
//
// Updated in fix/52 for the Bourns SSA-2-250A bipolar front-end:
// zero at Vref/2 (1.65 V), sensitivity 20 mV/A at S_CURRENT,
// discharge -> positive S_CURRENT (and positive mA).

#include "ams_config.hpp"
#include "current_service.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdlib>

// ---------------------------------------------------------------------------
// adc_to_mA: zero-current calibration point reads (almost) 0 mA.
// Sensor zero is now Vref/2 = 1.650 V (post-HW Vref/2 bias). ADC count =
// round((1650/3300)*4095) = 2047. Plug that back and confirm |mA| is small.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_zero_point_reads_near_zero(void) {
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (ams::config::CurrentZeroMv *
         static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(200, std::abs(mA));  // within 200 mA of zero
}

// ---------------------------------------------------------------------------
// adc_to_mA: discharge raises S_CURRENT above Vref/2, yielding positive mA.
// Pick raw = 1.75 V -> +100 mV delta. At 20 mV/A sensitivity -> +5 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_discharge_positive(void) {
    const std::int32_t target_mv = ams::config::CurrentZeroMv + 100;
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (target_mv * static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    // 100 mV / 20 mV/A = 5 A = 5000 mA. Allow +-200 mA rounding.
    TEST_ASSERT_INT32_WITHIN(200, 5000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: charging drops S_CURRENT below Vref/2, yielding negative mA.
// Pick raw = 1.55 V -> -100 mV delta -> -5 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_charge_negative(void) {
    const std::int32_t target_mv = ams::config::CurrentZeroMv - 100;
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (target_mv * static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv);

    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_INT32_WITHIN(200, -5000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA is symmetric around the zero point.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_symmetric_around_zero(void) {
    const std::int32_t zero_count =
        (ams::config::CurrentZeroMv *
         static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv;

    const auto plus_50  = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero_count + 50));
    const auto minus_50 = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero_count - 50));

    // Symmetry: |plus_50| ≈ |minus_50| within rounding (the integer-uV
    // math has < 50 mA bias around the zero point at 12-bit + 3.3 V
    // Vref). plus_50 is positive (discharge), minus_50 is negative.
    TEST_ASSERT_INT32_WITHIN(200, -minus_50, plus_50);
}

// ---------------------------------------------------------------------------
// adc_to_mA: full-scale discharge (S_CURRENT at the upper rail) maps
// near the HW clipping limit of +82.5 A (CurrentZeroMv + 1650 mV =
// Vref). Anything beyond clips at the rail and is not observable.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_upper_rail_near_82500_mA(void) {
    const std::uint16_t raw = ams::config::AdcMaxCount;  // 4095 -> Vref
    const std::int32_t mA   = ams::CurrentService::adc_to_mA(raw);
    // 1650 mV / 20 mV/A = 82500 mA. Allow ±500 mA for integer rounding.
    TEST_ASSERT_INT32_WITHIN(500, 82500, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: full-scale charge (S_CURRENT at 0 V rail) maps to -82.5 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_lower_rail_near_minus_82500_mA(void) {
    const std::int32_t mA = ams::CurrentService::adc_to_mA(0);
    TEST_ASSERT_INT32_WITHIN(500, -82500, mA);
}

// ---------------------------------------------------------------------------
// is_dcdc_fresh: false before any DCDC sample, true after a recent one,
// false again after DcdcIStaleMs elapses. (Pack channel staleness is
// covered indirectly via the safety-predicate tests; DCDC is purely
// informational so its own helper gets a focused test here.)
// ---------------------------------------------------------------------------
extern "C" void test_current_is_dcdc_fresh_lifecycle(void) {
    auto& cs = ams::CurrentService::instance();
    // Pristine state: never updated -> not fresh at any tick.
    cs.update_dcdc_from_adc(2047, 0);     // seeds last_dcdc_update_tick=0; still "no data"
    TEST_ASSERT_FALSE(cs.is_dcdc_fresh(0));

    // Real update at tick=1000.
    cs.update_dcdc_from_adc(2047, 1000);
    TEST_ASSERT_TRUE(cs.is_dcdc_fresh(1000));
    TEST_ASSERT_TRUE(cs.is_dcdc_fresh(1000 + ams::config::DcdcIStaleMs));
    TEST_ASSERT_FALSE(cs.is_dcdc_fresh(1000 + ams::config::DcdcIStaleMs + 1));
}
