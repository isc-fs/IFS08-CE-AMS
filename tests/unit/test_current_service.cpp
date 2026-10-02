// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for CurrentService::adc_to_mA() and adc_q4_to_mA() (pack,
// differential, 12-bit and oversampled). The public
// methods are exercised indirectly via the cmsis_os2 mock from
// tests/unit/mocks/.
//
// Updated in feat/current-sensor-diff for the new PCB front-end: the
// external x4 carrier diff amp is removed. The Bourns SSA-2-250A
// OUT_P/OUT_N now feed the STM32 ADC in DIFFERENTIAL mode (PF7/PF8 =
// ADC3_INP3/INN3): zero current at mid-scale code (CurrentZeroCount),
// sensitivity 5 mV/A, differential LSB = 2*Vref/4095.

#include "ams_config.hpp"
#include "current_service.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdlib>
#include <initializer_list>

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
// adc_q4_to_mA: the oversampled code (16 x the 12-bit code) must convert a
// whole-count value to exactly what adc_to_mA gives, so the safety path and
// the ELE log agree, and resolve steps 16x finer in between.
// ---------------------------------------------------------------------------
extern "C" void test_current_q4_matches_12bit(void) {
    for (std::uint16_t raw : {std::uint16_t{0}, std::uint16_t{1000}, std::uint16_t{2054},
                              std::uint16_t{2055}, std::uint16_t{3000}, std::uint16_t{4095}}) {
        TEST_ASSERT_EQUAL_INT32(ams::CurrentService::adc_to_mA(raw),
                                ams::CurrentService::adc_q4_to_mA(std::uint32_t{raw} << 4));
    }
}

// One Q4 step is 2 * 3300 mV / 4095 / 16 = 0.1 mV of differential, / 4.6 mV/A
// = ~22 mA (the 12-bit code alone steps ~350 mA).
extern "C" void test_current_q4_resolution(void) {
    const std::uint32_t zero = std::uint32_t{ams::config::CurrentZeroCount} << 4;
    TEST_ASSERT_EQUAL_INT32(0, ams::CurrentService::adc_q4_to_mA(zero));
    const std::int32_t step = ams::CurrentService::adc_q4_to_mA(zero + 1u);
    TEST_ASSERT_INT32_WITHIN(4, 22, step);
    TEST_ASSERT_INT32_WITHIN(4, -22, ams::CurrentService::adc_q4_to_mA(zero - 1u));
}

extern "C" void test_current_q4_to_raw_rounds(void) {
    TEST_ASSERT_EQUAL_UINT16(2054u, ams::CurrentService::q4_to_raw(2054u * 16u + 7u));
    TEST_ASSERT_EQUAL_UINT16(2055u, ams::CurrentService::q4_to_raw(2054u * 16u + 8u));
    TEST_ASSERT_EQUAL_UINT16(4095u, ams::CurrentService::q4_to_raw(64u * 4095u / 4u));   // ADC3 max
    TEST_ASSERT_EQUAL_UINT16(0u,    ams::CurrentService::q4_to_raw(0u));
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

