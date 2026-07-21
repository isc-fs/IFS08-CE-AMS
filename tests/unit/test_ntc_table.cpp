// SPDX-License-Identifier: proprietary
//
// Tests for ams::ntc -- the NTC R-T table that replaced the single-beta fit.
//
// Every expected value below comes from the MANUFACTURER's R-T appendix
// (docs/ntc_rt_table.csv, Fenghua CMFB103F3950FANT), not from this
// implementation. That matters: the bug this replaces was a self-consistent
// conversion that happened to describe a different thermistor, and no amount
// of asserting our decoder against our encoder would have caught it.

#include "ntc_table.hpp"

#include "ams_config.hpp"

#include "unity.h"

#include <cstdint>

namespace {

using namespace ams;

// The measurement path the firmware actually runs: a divider voltage in mV
// becomes a resistance, which becomes a temperature.
//   R_ntc = NtcPullupOhm * V / (VREF2 - V)
std::int32_t mv_to_decideg(std::uint16_t v_mV) {
    const std::uint32_t r =
        (static_cast<std::uint32_t>(config::NtcPullupOhm) * v_mV) /
        (static_cast<std::uint32_t>(config::NtcVrefMv) - v_mV);
    std::int32_t d = 0;
    return ntc::resistance_to_decideg(r, d) ? d : -32768;
}

}  // namespace

// --- table integrity --------------------------------------------------------

extern "C" void test_ntc_table_is_strictly_decreasing(void) {
    for (std::size_t i = 1; i < ntc::TableLen; ++i) {
        TEST_ASSERT_TRUE_MESSAGE(ntc::ResistanceOhm[i] < ntc::ResistanceOhm[i - 1],
                                 "NTC table must be strictly decreasing");
    }
}

extern "C" void test_ntc_table_covers_expected_range(void) {
    TEST_ASSERT_EQUAL_INT16(-55, ntc::TableMinC);
    TEST_ASSERT_EQUAL_INT16(125, ntc::TableMaxC);
    TEST_ASSERT_EQUAL_UINT32(181u, static_cast<std::uint32_t>(ntc::TableLen));
    // One entry per degree, endpoints inclusive.
    TEST_ASSERT_EQUAL_INT32(ntc::TableMaxC - ntc::TableMinC + 1,
                            static_cast<std::int32_t>(ntc::TableLen));
}

// --- datasheet anchor points ------------------------------------------------

// R25 = 10 kOhm is the part's defining spec. If this drifts, the table is not
// describing a CMFB103F3950.
extern "C" void test_ntc_r25_is_10k(void) {
    const std::size_t idx25 = static_cast<std::size_t>(25 - ntc::TableMinC);
    TEST_ASSERT_EQUAL_UINT32(10000u, ntc::ResistanceOhm[idx25]);
}

// Exact table resistances must round-trip to exactly their own temperature.
extern "C" void test_ntc_exact_table_points_round_trip(void) {
    const int temps[] = {-40, -10, 0, 25, 45, 50, 60, 80, 125};
    for (int t : temps) {
        const std::size_t idx = static_cast<std::size_t>(t - ntc::TableMinC);
        std::int32_t d = 0;
        TEST_ASSERT_TRUE(ntc::resistance_to_decideg(ntc::ResistanceOhm[idx], d));
        TEST_ASSERT_EQUAL_INT32(t * 10, d);
    }
}

// Datasheet resistances, independent of our indexing.
extern "C" void test_ntc_datasheet_resistances(void) {
    struct { int t; std::uint32_t r; } k[] = {
        {-40, 264279u}, {-10, 53198u}, {0, 32116u}, {25, 10000u},
        {45, 4345u},    {50, 3588u},   {60, 2457u}, {125, 326u},
    };
    for (auto& e : k) {
        const std::size_t idx = static_cast<std::size_t>(e.t - ntc::TableMinC);
        TEST_ASSERT_EQUAL_UINT32(e.r, ntc::ResistanceOhm[idx]);
    }
}

// --- the full measurement path ----------------------------------------------

// Divider voltages from the datasheet's own Vaux column (VREF2 = 3.0 V,
// pull-up = 6.8 k) must recover the temperature they were generated from.
// Tolerance is 1 degC: the mV column is rounded to whole millivolts before it
// reaches us, and the LTC quantises anyway.
extern "C" void test_ntc_divider_voltage_recovers_temperature(void) {
    struct { std::uint16_t mv; int t; } k[] = {
        {2925, -40}, {2660, -10}, {2476, 0}, {1786, 25},
        {1170, 45},  {1036, 50},  {796, 60}, {459, 80}, {137, 125},
    };
    for (auto& e : k) {
        const std::int32_t d = mv_to_decideg(e.mv);
        TEST_ASSERT_INT32_WITHIN_MESSAGE(10, e.t * 10, d,
                                         "divider voltage must recover its temperature");
    }
}

// The specific numbers that made the old fit dangerous. With beta=3380 and a
// 10k pull-up, a true 50 degC read 42.8 and a true 60 read 54.0 -- so
// BalanceTempMax=50 engaged at ~56 true and CellOverTempC=60 tripped at ~65.
// Both are now honest.
extern "C" void test_ntc_thresholds_are_now_true_degrees(void) {
    TEST_ASSERT_INT32_WITHIN(10, 500, mv_to_decideg(1036));   // true 50 C
    TEST_ASSERT_INT32_WITHIN(10, 600, mv_to_decideg(796));    // true 60 C
    TEST_ASSERT_INT32_WITHIN(10, 450, mv_to_decideg(1170));   // true 45 C (VTC6 charge limit)
}

// --- interpolation ----------------------------------------------------------

// A resistance between two table points lands between their temperatures, and
// monotonically so.
extern "C" void test_ntc_interpolates_between_points(void) {
    const std::size_t i25 = static_cast<std::size_t>(25 - ntc::TableMinC);
    const std::uint32_t r25 = ntc::ResistanceOhm[i25];
    const std::uint32_t r26 = ntc::ResistanceOhm[i25 + 1];
    const std::uint32_t mid = (r25 + r26) / 2u;

    std::int32_t d = 0;
    TEST_ASSERT_TRUE(ntc::resistance_to_decideg(mid, d));
    TEST_ASSERT_INT32_WITHIN(2, 255, d);   // ~25.5 C
    TEST_ASSERT_TRUE(d > 250 && d < 260);
}

extern "C" void test_ntc_is_monotonic_over_the_whole_range(void) {
    std::int32_t prev = -100000;
    for (std::uint32_t r = 400u; r < 200000u; r += 137u) {
        std::int32_t d = 0;
        if (!ntc::resistance_to_decideg(r, d)) continue;
        TEST_ASSERT_TRUE_MESSAGE(d >= prev, "temperature must fall as R rises");
        prev = d;
    }
}

// --- out of range -----------------------------------------------------------

// An open channel (no NTC fitted) sits near VREF2 -> enormous R; a short sits
// near 0 -> tiny R. Both must be refused rather than clamped, so the caller's
// "skip this slot" sentinel fires instead of a plausible-looking number.
extern "C" void test_ntc_rejects_out_of_range_resistance(void) {
    std::int32_t d = 0;
    TEST_ASSERT_FALSE(ntc::resistance_to_decideg(ntc::ResistanceOhm[0] + 1u, d));
    TEST_ASSERT_FALSE(ntc::resistance_to_decideg(ntc::ResistanceOhm[ntc::TableLen - 1] - 1u, d));
    TEST_ASSERT_FALSE(ntc::resistance_to_decideg(0u, d));
    TEST_ASSERT_FALSE(ntc::resistance_to_decideg(10000000u, d));
}

extern "C" void test_ntc_accepts_exact_endpoints(void) {
    std::int32_t d = 0;
    TEST_ASSERT_TRUE(ntc::resistance_to_decideg(ntc::ResistanceOhm[0], d));
    TEST_ASSERT_EQUAL_INT32(ntc::TableMinC * 10, d);
    TEST_ASSERT_TRUE(ntc::resistance_to_decideg(ntc::ResistanceOhm[ntc::TableLen - 1], d));
    TEST_ASSERT_EQUAL_INT32(ntc::TableMaxC * 10, d);
}

// The pull-up is the constant that was wrong, and its value changes the sign
// of the whole error. Pin it.
extern "C" void test_ntc_pullup_is_6k8(void) {
    TEST_ASSERT_EQUAL_UINT32(6800u, config::NtcPullupOhm);
    TEST_ASSERT_EQUAL_UINT16(3000u, config::NtcVrefMv);
}
