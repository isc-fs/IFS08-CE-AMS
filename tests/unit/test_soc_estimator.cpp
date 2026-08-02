// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::soc -- the OCV curve, the rest gate and the Coulomb
// counter. No HAL, no FreeRTOS, no services; state is built in place.

#include "ams_config.hpp"
#include "soc_estimator.hpp"

#include "unity.h"

#include <cstdint>
#include <initializer_list>

namespace {
using namespace ams;

// Charge in mA*s for a given SoC delta, mirroring the counter's own maths so a
// test failure points at the counter rather than at disagreeing arithmetic.
constexpr std::int64_t full_mAs() {
    return static_cast<std::int64_t>(config::PackCapacityMah) * 3600;
}
}  // namespace

// --- OCV curve --------------------------------------------------------------

extern "C" void test_soc_ocv_hits_breakpoints_exactly(void) {
    // Every fitted breakpoint must map to its own SoC -- if interpolation is
    // off by one index the whole curve shifts and nothing else here would catch
    // it, because interior points would still look monotonic and plausible.
    for (std::uint8_t i = 0; i < soc::OcvPoints; ++i) {
        TEST_ASSERT_EQUAL_UINT16(soc::OcvSocPermille[i],
                                 soc::ocv_to_soc_permille(soc::OcvCellMv[i]));
    }
}

extern "C" void test_soc_ocv_interpolates_midpoints(void) {
    // Halfway between two breakpoints -> halfway between their SoCs.
    // 3468 mV (300) .. 3655 mV (500): midpoint 3561 mV -> ~400 permille.
    const std::uint16_t mid = soc::ocv_to_soc_permille(3561);
    TEST_ASSERT_UINT16_WITHIN(6, 400, mid);

    // Near the top the curve is much steeper: 3994 (900) .. 4220 (1000).
    const std::uint16_t hi = soc::ocv_to_soc_permille(4107);
    TEST_ASSERT_UINT16_WITHIN(6, 950, hi);
}

extern "C" void test_soc_ocv_clamps_outside_the_curve(void) {
    // Below the protection cutoff and above the charge limit both rail rather
    // than extrapolating into nonsense.
    TEST_ASSERT_EQUAL_UINT16(0u,    soc::ocv_to_soc_permille(1000));
    TEST_ASSERT_EQUAL_UINT16(0u,    soc::ocv_to_soc_permille(2500));
    TEST_ASSERT_EQUAL_UINT16(1000u, soc::ocv_to_soc_permille(4220));
    TEST_ASSERT_EQUAL_UINT16(1000u, soc::ocv_to_soc_permille(5000));
}

extern "C" void test_soc_ocv_is_monotonic(void) {
    // A non-monotonic OCV curve would make SoC jump backwards as the pack
    // charges. Sweep the whole usable band and assert it never decreases.
    std::uint16_t prev = 0;
    for (std::uint16_t mv = 2500; mv <= 4220; mv += 5) {
        const std::uint16_t s = soc::ocv_to_soc_permille(mv);
        TEST_ASSERT_TRUE_MESSAGE(s >= prev, "OCV curve went backwards");
        prev = s;
    }
    TEST_ASSERT_EQUAL_UINT16(1000u, prev);
}

// --- rest gate --------------------------------------------------------------

extern "C" void test_soc_rest_gate_requires_low_current_and_settle(void) {
    const std::uint32_t settled = config::SocRestSettleMs;
    const std::int32_t  quiet   = static_cast<std::int32_t>(config::SocRestCurrentMa);

    TEST_ASSERT_TRUE (soc::ocv_anchor_valid(0,          settled));
    TEST_ASSERT_TRUE (soc::ocv_anchor_valid(quiet,      settled));
    TEST_ASSERT_TRUE (soc::ocv_anchor_valid(-quiet,     settled));   // charge side
    // Current too high -> terminal voltage carries I*R, anchor invalid.
    TEST_ASSERT_FALSE(soc::ocv_anchor_valid(quiet + 1,  settled));
    TEST_ASSERT_FALSE(soc::ocv_anchor_valid(-(quiet+1), settled));
    // Not rested long enough -> diffusion still relaxing, reads low.
    TEST_ASSERT_FALSE(soc::ocv_anchor_valid(0,          settled - 1u));
    TEST_ASSERT_FALSE(soc::ocv_anchor_valid(0,          0));
}

// --- Coulomb counter --------------------------------------------------------

extern "C" void test_soc_counter_starts_unknown(void) {
    // Never anchored -> Unknown, not 0 %. A consumer must be able to tell
    // "no estimate" from "empty pack".
    soc::CoulombCounter cc;
    TEST_ASSERT_FALSE(cc.anchored());
    TEST_ASSERT_EQUAL_UINT8(soc::Unknown, cc.soc_percent());
}

extern "C" void test_soc_counter_ignores_updates_before_anchor(void) {
    // Integrating without a reference would produce a number with no meaning.
    soc::CoulombCounter cc;
    for (int i = 0; i < 100; ++i) cc.update(10000, 50);
    TEST_ASSERT_EQUAL_UINT8(soc::Unknown, cc.soc_percent());
}

extern "C" void test_soc_counter_anchor_round_trips(void) {
    soc::CoulombCounter cc;
    for (std::uint16_t p : {0u, 250u, 500u, 750u, 1000u}) {
        cc.anchor(p);
        TEST_ASSERT_TRUE(cc.anchored());
        TEST_ASSERT_UINT16_WITHIN(1, p, cc.soc_permille());
    }
}

extern "C" void test_soc_counter_discharge_lowers_soc(void) {
    // Sign convention: POSITIVE current is discharge and must REMOVE charge.
    // Getting this backwards is the classic Coulomb-counting bug and would show
    // as SoC rising while the car drives.
    soc::CoulombCounter cc;
    cc.anchor(1000);

    // Remove exactly 10 % of 18 Ah = 1.8 Ah = 6480 A*s.
    // At 64.8 A for 100 s (2000 samples x 50 ms) -> 6480 A*s.
    for (int i = 0; i < 2000; ++i) cc.update(64800, 50);

    TEST_ASSERT_UINT16_WITHIN(5, 900, cc.soc_permille());
    TEST_ASSERT_EQUAL_UINT8(90, cc.soc_percent());
}

extern "C" void test_soc_counter_charge_raises_soc(void) {
    soc::CoulombCounter cc;
    cc.anchor(500);
    // Negative current = charge = adds charge back.
    for (int i = 0; i < 2000; ++i) cc.update(-64800, 50);
    TEST_ASSERT_UINT16_WITHIN(5, 600, cc.soc_permille());
}

extern "C" void test_soc_counter_small_currents_are_not_truncated(void) {
    // The reason charge is accumulated in mA*s rather than straight into
    // permille: one permille of 18 Ah is 64.8 A*s, so a 5 A sample over 50 ms
    // (0.25 A*s) is ~1/260 of a point. Integrating into permille would round
    // every one of those to zero and the counter would sit still through a real
    // discharge. Here 5 A for 3600 s must move SoC by a visible amount.
    soc::CoulombCounter cc;
    cc.anchor(1000);
    for (int i = 0; i < 72000; ++i) cc.update(5000, 50);   // 5 A, 1 hour
    // 5 A * 3600 s = 18000 A*s = 5 Ah of 18 Ah = 27.8 %
    TEST_ASSERT_UINT16_WITHIN(10, 722, cc.soc_permille());
}

extern "C" void test_soc_counter_clamps_at_rails(void) {
    // A drifted counter must not run past the physical range -- otherwise the
    // next anchor's correction looks like a fault instead of a re-sync.
    soc::CoulombCounter cc;
    cc.anchor(50);
    for (int i = 0; i < 20000; ++i) cc.update(64800, 50);   // way past empty
    TEST_ASSERT_EQUAL_UINT16(0u, cc.soc_permille());
    TEST_ASSERT_EQUAL_UINT8(0u, cc.soc_percent());

    cc.anchor(950);
    for (int i = 0; i < 20000; ++i) cc.update(-64800, 50);  // way past full
    TEST_ASSERT_EQUAL_UINT16(1000u, cc.soc_permille());
    TEST_ASSERT_EQUAL_UINT8(100u, cc.soc_percent());
}

extern "C" void test_soc_counter_rejects_implausible_gaps(void) {
    // A gap longer than SocMaxIntegrationGapMs means the task was starved.
    // Integrating across it would invent charge that may never have flowed.
    soc::CoulombCounter cc;
    cc.anchor(500);
    const std::uint16_t before = cc.soc_permille();

    cc.update(100000, config::SocMaxIntegrationGapMs + 1u);   // rejected
    TEST_ASSERT_EQUAL_UINT16(before, cc.soc_permille());
    cc.update(100000, 0);                                     // rejected
    TEST_ASSERT_EQUAL_UINT16(before, cc.soc_permille());

    // Exactly at the limit is still accepted.
    cc.update(100000, config::SocMaxIntegrationGapMs);
    TEST_ASSERT_TRUE(cc.soc_permille() < before);
}

extern "C" void test_soc_counter_invalidate_returns_to_unknown(void) {
    // Current sensor faulted -> the integral is no longer trustworthy.
    soc::CoulombCounter cc;
    cc.anchor(700);
    TEST_ASSERT_EQUAL_UINT8(70, cc.soc_percent());

    cc.invalidate();
    TEST_ASSERT_FALSE(cc.anchored());
    TEST_ASSERT_EQUAL_UINT8(soc::Unknown, cc.soc_percent());

    // And it stays put until re-anchored, rather than silently resuming.
    cc.update(64800, 50);
    TEST_ASSERT_EQUAL_UINT8(soc::Unknown, cc.soc_percent());
    cc.anchor(400);
    TEST_ASSERT_EQUAL_UINT8(40, cc.soc_percent());
}

extern "C" void test_soc_counter_net_zero_current_holds_soc(void) {
    // Symmetric charge/discharge must return to where it started -- guards
    // against an asymmetry in the integer rounding that would drift one way.
    soc::CoulombCounter cc;
    cc.anchor(500);
    for (int i = 0; i < 1000; ++i) { cc.update(30000, 50); cc.update(-30000, 50); }
    TEST_ASSERT_UINT16_WITHIN(2, 500, cc.soc_permille());
}

// A full discharge at the pack's own 1C rate should take one hour, which is the
// sanity check that PackCapacityMah is being applied at the right scale.
extern "C" void test_soc_counter_1C_discharge_empties_in_one_hour(void) {
    soc::CoulombCounter cc;
    cc.anchor(1000);
    const std::int32_t one_C_mA = static_cast<std::int32_t>(config::PackCapacityMah);  // 18 A
    for (int i = 0; i < 72000; ++i) cc.update(one_C_mA, 50);   // 3600 s
    TEST_ASSERT_UINT16_WITHIN(10, 0, cc.soc_permille());
    TEST_ASSERT_EQUAL_INT64(full_mAs(), static_cast<std::int64_t>(config::PackCapacityMah) * 3600);
}
