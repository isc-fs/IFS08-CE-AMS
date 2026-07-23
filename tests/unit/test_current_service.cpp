// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for CurrentService::adc_to_mA() (pack, differential)
// and adc_to_mA_dcdc() (DCDC, single-ended). The mutex-using public
// methods are exercised indirectly via the cmsis_os2 mock from
// tests/unit/mocks/.
//
// Updated in feat/current-sensor-diff for the new PCB front-end: the
// external x4 carrier diff amp is removed. The Bourns SSA-2-250A
// OUT_P/OUT_N now feed the STM32 ADC in DIFFERENTIAL mode (PF7/PF8 =
// ADC3_INP3/INN3): zero current at mid-scale code (CurrentZeroCount),
// sensitivity 5 mV/A, differential LSB = 2*Vref/4095. DCDC current is a
// separate single-ended sensor on PC1 (ADC3_INP11).

#include "ams_config.hpp"
#include "current_service.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdlib>

namespace {

// Inverse of the differential pack transfer: amps(mA) -> raw ADC code.
//   diff_uV  = mA * CurrentMvPerAmpe1 / 10
//   counts   = diff_uV * AdcMaxCount / (2 * AdcVrefMv * 1000)
//   raw      = CurrentZeroCount + counts
std::uint16_t pack_raw_for_mA(std::int32_t mA) {
    const std::int64_t diff_uV =
        static_cast<std::int64_t>(mA) * ams::config::CurrentMvPerAmpe1 / 10;
    const std::int64_t counts =
        (diff_uV * ams::config::AdcMaxCount) /
        (2 * static_cast<std::int64_t>(ams::config::AdcVrefMv) * 1000);
    return static_cast<std::uint16_t>(ams::config::CurrentZeroCount + counts);
}

// One differential LSB in mA, the worst-case quantisation step. Used as
// the comparison tolerance so the round-trip tests are robust.
constexpr std::int32_t kPackLsbMa =
    (2 * ams::config::AdcVrefMv * 1000 / ams::config::AdcMaxCount) * 10 /
    ams::config::CurrentMvPerAmpe1;  // ~322 mA

}  // namespace

// ---------------------------------------------------------------------------
// adc_to_mA: zero-current is mid-scale (CurrentZeroCount) -> ~0 mA.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_zero_point_reads_near_zero(void) {
    const std::int32_t mA = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(ams::config::CurrentZeroCount));
    TEST_ASSERT_EQUAL_INT32(0, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: discharge drives OUT_P above OUT_N (raw above mid-scale) ->
// positive mA. +50 A here.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_discharge_positive(void) {
    const std::uint16_t raw = pack_raw_for_mA(50000);
    const std::int32_t mA   = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_INT32_WITHIN(kPackLsbMa, 50000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: charge drives OUT_P below OUT_N (raw below mid-scale) ->
// negative mA. -50 A here.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_charge_negative(void) {
    const std::uint16_t raw = pack_raw_for_mA(-50000);
    const std::int32_t mA   = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_INT32_WITHIN(kPackLsbMa, -50000, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA is symmetric around the mid-scale zero point.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_symmetric_around_zero(void) {
    const std::int32_t zero = ams::config::CurrentZeroCount;
    const auto plus_100  = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero + 100));
    const auto minus_100 = ams::CurrentService::adc_to_mA(
        static_cast<std::uint16_t>(zero - 100));
    TEST_ASSERT_INT32_WITHIN(kPackLsbMa, -minus_100, plus_100);
}

// ---------------------------------------------------------------------------
// adc_to_mA: the 200 A over-current threshold (CurrentMaxMa) is now
// genuinely OBSERVABLE -- it maps to a raw code well inside 0..4095,
// unlike the old x4 + 1.65 V front-end that clipped firmware-side at
// only +/- 82.5 A. Lock that in.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_over_limit_is_observable(void) {
    const std::uint16_t raw = pack_raw_for_mA(ams::config::CurrentMaxMa);
    TEST_ASSERT_LESS_THAN_UINT16(ams::config::AdcMaxCount, raw);
    const std::int32_t mA = ams::CurrentService::adc_to_mA(raw);
    TEST_ASSERT_INT32_WITHIN(kPackLsbMa, ams::config::CurrentMaxMa, mA);
}

// ---------------------------------------------------------------------------
// adc_to_mA: full-scale rails. raw = 4095 and raw = 0 are the extreme
// differential codes (~+/- Vref); confirm sign and rough magnitude with
// the HIL-commissioned constants (CurrentZeroCount 2050, sens 46):
// (4095 - 2050) * 2 * 3300 * 1000 / 4095 * 10 / 46 ~= +717 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_adc_full_scale_rails(void) {
    const std::int32_t hi = ams::CurrentService::adc_to_mA(ams::config::AdcMaxCount);
    const std::int32_t lo = ams::CurrentService::adc_to_mA(0);
    TEST_ASSERT_INT32_WITHIN(2000, 716674, hi);
    TEST_ASSERT_INT32_WITHIN(2000, -718426, lo);
}

// ---------------------------------------------------------------------------
// adc_to_mA_dcdc: single-ended Allegro ACS758 @ 3.3 V (ratiometric,
// 26.4 mV/A, offset 1.65 V = DcdcCurrentZeroMv). The zero-bias ADC code
// reads ~0 mA; a +264 mV step (26.4 mV/A x 10 A) reads +10 A.
// ---------------------------------------------------------------------------
extern "C" void test_current_dcdc_zero_and_discharge(void) {
    const std::uint16_t zero_raw = static_cast<std::uint16_t>(
        (ams::config::DcdcCurrentZeroMv *
         static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(
        200, std::abs(ams::CurrentService::adc_to_mA_dcdc(zero_raw)));

    // +10 A -> +264 mV above the 1.65 V offset (26.4 mV/A).
    const std::int32_t up_mv = ams::config::DcdcCurrentZeroMv + 264;
    const std::uint16_t up_raw = static_cast<std::uint16_t>(
        (up_mv * static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
        ams::config::AdcVrefMv);
    // 264 mV / 26.4 mV/A = 10 A = 10000 mA. Allow +-200 mA rounding.
    TEST_ASSERT_INT32_WITHIN(200, 10000,
                             ams::CurrentService::adc_to_mA_dcdc(up_raw));
}

// ---------------------------------------------------------------------------
// leg_voltage_plausible: a connected OUT_P leg sits near the 1.44 V
// common-mode (plus its half-swing); a disconnect (with the internal
// pull-down) reads ~0 V -> implausible. Check the window edges.
// ---------------------------------------------------------------------------
extern "C" void test_current_leg_voltage_window(void) {
    auto raw_for_mv = [](std::int32_t mv) {
        return static_cast<std::uint16_t>(
            (mv * static_cast<std::int32_t>(ams::config::AdcMaxCount)) /
            ams::config::AdcVrefMv);
    };
    // Connected: OUT_P at the common-mode (~1.44 V) and across the swing.
    TEST_ASSERT_TRUE(ams::CurrentService::leg_voltage_plausible(raw_for_mv(1440)));
    TEST_ASSERT_TRUE(ams::CurrentService::leg_voltage_plausible(raw_for_mv(940)));   // -200 A
    TEST_ASSERT_TRUE(ams::CurrentService::leg_voltage_plausible(raw_for_mv(1940)));  // +200 A
    // Disconnected: pulled to ~0 V -> below CurrentLegPlausMinMv.
    TEST_ASSERT_FALSE(ams::CurrentService::leg_voltage_plausible(raw_for_mv(0)));
    TEST_ASSERT_FALSE(ams::CurrentService::leg_voltage_plausible(
        raw_for_mv(ams::config::CurrentLegPlausMinMv - 50)));
    // Stuck at the upper rail -> above CurrentLegPlausMaxMv.
    TEST_ASSERT_FALSE(ams::CurrentService::leg_voltage_plausible(
        raw_for_mv(ams::config::CurrentLegPlausMaxMv + 50)));
}

// ---------------------------------------------------------------------------
// update_from_adc propagates the debounced sensor_fault verdict into the
// snapshot (the flag the CurrentSensorFault safety predicate reads).
// ---------------------------------------------------------------------------
extern "C" void test_current_update_sets_sensor_fault(void) {
    auto& cs = ams::CurrentService::instance();
    cs.update_from_adc(2050, 5000, /*sensor_fault=*/true);
    TEST_ASSERT_TRUE(cs.snapshot().sensor_fault);
    cs.update_from_adc(2050, 5050, /*sensor_fault=*/false);
    TEST_ASSERT_FALSE(cs.snapshot().sensor_fault);
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

// ---------------------------------------------------------------------------
// apply_report_deadband: |I| strictly BELOW CurrentReportDeadbandMa (0.5 A)
// reads as exactly 0; at/above the edge it passes through, for both signs.
// ---------------------------------------------------------------------------
extern "C" void test_current_report_deadband_pure(void) {
    using ams::CurrentService;
    const std::int32_t db = ams::config::CurrentReportDeadbandMa;
    TEST_ASSERT_EQUAL_INT32(0, CurrentService::apply_report_deadband(0));
    TEST_ASSERT_EQUAL_INT32(0, CurrentService::apply_report_deadband(db - 1));
    TEST_ASSERT_EQUAL_INT32(0, CurrentService::apply_report_deadband(-(db - 1)));
    // Exactly at the edge and beyond -> unchanged, both signs.
    TEST_ASSERT_EQUAL_INT32(db,     CurrentService::apply_report_deadband(db));
    TEST_ASSERT_EQUAL_INT32(-db,    CurrentService::apply_report_deadband(-db));
    TEST_ASSERT_EQUAL_INT32(50000,  CurrentService::apply_report_deadband(50000));
    TEST_ASSERT_EQUAL_INT32(-50000, CurrentService::apply_report_deadband(-50000));
}

// ---------------------------------------------------------------------------
// End-to-end: a sub-deadband current (~1 ADC LSB, ~322 mA) settles to a
// reported 0 A through update_from_adc, while a genuine current well above
// the band is preserved.
// ---------------------------------------------------------------------------
extern "C" void test_current_filtered_zeroed_below_deadband(void) {
    auto& cs = ams::CurrentService::instance();
    // ~1 LSB above the zero code -> ~322 mA, inside the 500 mA band.
    const std::uint16_t small_raw =
        static_cast<std::uint16_t>(ams::config::CurrentZeroCount + 1);
    for (int i = 0; i < 400; ++i) cs.update_from_adc(small_raw, 1000u + i);
    TEST_ASSERT_EQUAL_INT32(0, cs.snapshot().filtered_mA);

    // A genuine +5 A must survive the band untouched.
    const std::uint16_t big_raw = pack_raw_for_mA(5000);
    for (int i = 0; i < 400; ++i) cs.update_from_adc(big_raw, 2000u + i);
    TEST_ASSERT_INT32_WITHIN(kPackLsbMa, 5000, cs.snapshot().filtered_mA);
}
