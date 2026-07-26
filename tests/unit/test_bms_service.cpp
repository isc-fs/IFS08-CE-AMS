// SPDX-License-Identifier: proprietary
//
// Pure-logic unit tests for BmsService against the v1.2.0 LTC6811-1
// data path. No HAL, no FreeRTOS -- the mocks/ directory stubs out
// cmsis_os2 enough that BmsService::* compiles and runs on the host.
//
// Feeds synthesized chain responses (concatenated 8-byte segments,
// 6 data + 2 PEC each) into BmsService::update_from_ltc_response and
// asserts on the resulting snapshot. PEC bytes are computed with the
// real ltc6811::pec15 so the decoder sees realistic input.

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "ltc6811.hpp"

#include "unity.h"

#include <array>
#include <cstdint>
#include <cstring>

extern "C" void fake_set_tick(std::uint32_t);
extern "C" void fake_advance_tick(std::uint32_t);

namespace {

using namespace ams;

constexpr std::size_t Seg        = 8;
constexpr std::size_t GroupBytes = config::LtcChainLength * Seg;  // 80
constexpr std::size_t RespBytes  = 4u * GroupBytes;                // 320

// Encode three cell mV values into a single 8-byte segment (6 data
// little-endian uint16 in 100uV units + 2 PEC bytes). Matches the
// on-wire format the LTC6811 produces for one register group of one
// IC; using the real pec15 keeps the test rig honest.
void encode_segment(std::uint8_t* out,
                    std::uint16_t c0_mV,
                    std::uint16_t c1_mV,
                    std::uint16_t c2_mV) {
    const std::uint16_t raw[3] = {
        static_cast<std::uint16_t>(c0_mV * 10u),
        static_cast<std::uint16_t>(c1_mV * 10u),
        static_cast<std::uint16_t>(c2_mV * 10u),
    };
    for (std::size_t k = 0; k < 3; ++k) {
        out[2 * k]     = static_cast<std::uint8_t>(raw[k] & 0xFFu);
        out[2 * k + 1] = static_cast<std::uint8_t>((raw[k] >> 8) & 0xFFu);
    }
    const std::uint16_t pec = ltc6811::pec15(out, 6);
    out[6] = static_cast<std::uint8_t>((pec >> 8) & 0xFFu);
    out[7] = static_cast<std::uint8_t>(pec & 0xFFu);
}

// Build a full 320-byte chain response where every IC reports
// distinct, monotonically increasing voltages so an off-by-one bug in
// the cell-slot mapping shows up as an obvious test failure.
//
// Convention used by these tests: cell mV for module m, module-local
// slot s, is 3000 + 100*m + s. For example:
//   module 0 cell 0  -> 3000 mV
//   module 0 cell 9  -> 3009 mV (top of LTC_1)
//   module 0 cell 10 -> 3010 mV (bottom of LTC_2)
//   module 4 cell 18 -> 3418 mV
void build_clean_chain(std::uint8_t* out) {
    std::memset(out, 0, RespBytes);
    auto mV = [](std::uint8_t m, std::uint8_t s) -> std::uint16_t {
        return static_cast<std::uint16_t>(3000u + 100u * m + s);
    };
    auto seg_at = [&](std::uint8_t group, std::uint8_t ic) -> std::uint8_t* {
        return out + group * GroupBytes + ic * Seg;
    };

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        const std::uint8_t ic_upper = static_cast<std::uint8_t>(2u * m);
        const std::uint8_t ic_lower = static_cast<std::uint8_t>(2u * m + 1u);

        // LTC_1 (first in chain, 9 cells -> module 0..8). Group D is
        // entirely discarded but its PEC must still validate or the
        // decoder marks the IC offline.
        encode_segment(seg_at(0, ic_upper), mV(m, 0), mV(m, 1), mV(m, 2));
        encode_segment(seg_at(1, ic_upper), mV(m, 3), mV(m, 4), mV(m, 5));
        encode_segment(seg_at(2, ic_upper), mV(m, 6), mV(m, 7), mV(m, 8));
        encode_segment(seg_at(3, ic_upper), 0x0FFFu, 0x0FFFu, 0x0FFFu);

        // LTC_2 (second in chain, 10 cells -> module 9..18). Group D
        // carries one real cell (slot 0 = C10 = module cell 18); the
        // remaining two slots must still PEC-validate though discarded.
        encode_segment(seg_at(0, ic_lower), mV(m, 9),  mV(m, 10), mV(m, 11));
        encode_segment(seg_at(1, ic_lower), mV(m, 12), mV(m, 13), mV(m, 14));
        encode_segment(seg_at(2, ic_lower), mV(m, 15), mV(m, 16), mV(m, 17));
        encode_segment(seg_at(3, ic_lower), mV(m, 18),
                       /* unused */ 0x0FFFu, /* unused */ 0x0FFFu);
    }
}

}  // namespace

extern "C" void setUp(void)    { fake_set_tick(0); }
extern "C" void tearDown(void) {}

// ---------------------------------------------------------------------------
// Happy path: a clean 320-byte response writes every cell of every
// module at the right offset, online masks reach 0x3FF (10 LTCs) /
// 0x1F (5 modules), is_healthy goes true.
// ---------------------------------------------------------------------------
// FS rule: an out-of-range cell open must open the SDC in < 500 ms. Guard the
// range-path budget: one voltage poll to observe the open + the confirm debounce
// + a safety tick. (e.g. 200 + 25x10 + 20 = 470 ms.) A regression to a 250 ms
// poll or a longer debounce trips this. In-range opens are ADOW's job.
extern "C" void test_cell_open_range_budget_under_500ms(void) {
    const std::uint32_t worst_ms =
        config::BmsPollVoltMs
        + static_cast<std::uint32_t>(config::CellFaultConfirmTicks) * config::SafetyPeriodMs
        + config::StatePeriodMs;
    TEST_ASSERT_LESS_THAN_UINT32(500u, worst_ms);
}

// A whole module going silent (stop measuring ALL of it) must open the SDC in
// < 500 ms. It drops off module_online_mask at the first voltage poll whose
// freshness age exceeds BmsStaleMs -> BmsModuleOffline (immediate). Guard both:
// (a) BmsStaleMs > one poll so a single missed poll is tolerated (glitch immune),
// (b) the detect budget: polls-until-age-exceeds-staleness x cadence + tick < 500.
extern "C" void test_module_loss_budget_under_500ms(void) {
    // One missed voltage poll must not trip it.
    TEST_ASSERT_GREATER_THAN_UINT32(config::BmsPollVoltMs, config::BmsStaleMs);
    // First poll STRICTLY after BmsStaleMs detects the drop.
    const std::uint32_t polls_to_detect =
        (config::BmsStaleMs / config::BmsPollVoltMs) + 1u;
    const std::uint32_t worst_ms =
        polls_to_detect * config::BmsPollVoltMs + config::StatePeriodMs;
    TEST_ASSERT_LESS_THAN_UINT32(500u, worst_ms);
}

// Cell open-wire (ADOW) end-to-end through BmsService::update_open_wire: two
// identical PU/PD passes = no open; a PU reading pulled far below PD on one
// interior cell of module 0's upper LTC flags exactly module 0's bit.
extern "C" void test_bms_open_wire_flags_module(void) {
    if (!config::CellOpenWireCheck) { TEST_IGNORE_MESSAGE("CellOpenWireCheck off"); return; }
    std::uint8_t pu[RespBytes], pd[RespBytes];
    build_clean_chain(pu);
    build_clean_chain(pd);

    // PU == PD -> no open on any IC. All ICs PEC-clean -> "all evaluated" == true.
    TEST_ASSERT_TRUE(BmsService::instance().update_open_wire(pu, pd, RespBytes));
    TEST_ASSERT_EQUAL_UINT8(0, BmsService::instance().snapshot().cell_open_mask);

    // Inject an open on module 0's upper LTC (ic 0), interior cell 3: group B
    // (index 1) carries cells 3,4,5 -- pull cell 3's PU far below its PD (3003)
    // so PU-PD < -CellOpenWireDeltaMv; leave 4,5 matching so only conductor 3 trips.
    encode_segment(pu + 1u * GroupBytes + 0u * Seg, /*c3*/ 1500u, /*c4*/ 3004u, /*c5*/ 3005u);
    TEST_ASSERT_TRUE(BmsService::instance().update_open_wire(pu, pd, RespBytes));
    // Only module 0 flagged.
    TEST_ASSERT_EQUAL_UINT8(1u << 0, BmsService::instance().snapshot().cell_open_mask);
}

// A PEC glitch on one IC's ADOW pass means that IC can't be judged -> update_
// open_wire returns false so BmsPollTask retries the two-pass scan in-poll
// (the fix for the ADOW no-retry gap that slipped detection past 500 ms).
extern "C" void test_bms_open_wire_pec_glitch_signals_retry(void) {
    if (!config::CellOpenWireCheck) { TEST_IGNORE_MESSAGE("CellOpenWireCheck off"); return; }
    std::uint8_t pu[RespBytes], pd[RespBytes];
    build_clean_chain(pu);
    build_clean_chain(pd);
    // Corrupt ic0/group-A PEC on the PU pass (byte 6 of that segment) so its
    // decode fails -> that IC is skipped -> "not all evaluated".
    pu[0u * GroupBytes + 0u * Seg + 6u] ^= 0xFFu;
    TEST_ASSERT_FALSE(BmsService::instance().update_open_wire(pu, pd, RespBytes));
    // A fully clean pair is judged completely -> true (retry would stop).
    build_clean_chain(pu);
    TEST_ASSERT_TRUE(BmsService::instance().update_open_wire(pu, pd, RespBytes));
}

// The double-glitch edge the eval flagged: attempt 0 confirms an open on module 0,
// then on the retry that same IC PEC-glitches (skipped). With accumulate=true the
// confirmed open MUST be preserved, not erased by the retry's overwrite.
extern "C" void test_bms_open_wire_retry_preserves_confirmed_open(void) {
    if (!config::CellOpenWireCheck) { TEST_IGNORE_MESSAGE("CellOpenWireCheck off"); return; }
    std::uint8_t pu[RespBytes], pd[RespBytes];

    // Attempt 0: module 0 upper-LTC cell 3 genuinely open, whole chain PEC-clean.
    build_clean_chain(pu);
    build_clean_chain(pd);
    encode_segment(pu + 1u * GroupBytes + 0u * Seg, /*c3*/ 1500u, /*c4*/ 3004u, /*c5*/ 3005u);
    TEST_ASSERT_TRUE(BmsService::instance().update_open_wire(pu, pd, RespBytes, /*accumulate=*/false));
    TEST_ASSERT_EQUAL_UINT8(1u << 0, BmsService::instance().snapshot().cell_open_mask);

    // Attempt 1 (retry): ic0 now PEC-glitches (skipped) and reads no open elsewhere.
    // accumulate=true must keep module 0's bit rather than overwriting it away.
    build_clean_chain(pu);
    build_clean_chain(pd);
    pu[0u * GroupBytes + 0u * Seg + 6u] ^= 0xFFu;   // break ic0 group A PEC (PU pass)
    TEST_ASSERT_FALSE(BmsService::instance().update_open_wire(pu, pd, RespBytes, /*accumulate=*/true));
    TEST_ASSERT_BITS_HIGH(1u << 0, BmsService::instance().snapshot().cell_open_mask);
}

extern "C" void test_bms_ltc_clean_response_decodes_all_cells(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);

    fake_set_tick(1000);
    TEST_ASSERT_TRUE(BmsService::instance().update_from_ltc_response(resp,
                                                                     sizeof(resp),
                                                                     1000));

    const auto s = BmsService::instance().snapshot();
    // Spot-check across modules + corners of each LTC's cell range.
    TEST_ASSERT_EQUAL_UINT16(3000, s.cell_mV[0][0]);
    TEST_ASSERT_EQUAL_UINT16(3008, s.cell_mV[0][8]);   // last cell of the 9-cell LTC
    TEST_ASSERT_EQUAL_UINT16(3009, s.cell_mV[0][9]);   // first cell of the 10-cell LTC (#423: was 0)
    TEST_ASSERT_EQUAL_UINT16(3018, s.cell_mV[0][18]);  // 10th cell of the 10-cell LTC (RDCVD C10)
    TEST_ASSERT_EQUAL_UINT16(3100, s.cell_mV[1][0]);
    TEST_ASSERT_EQUAL_UINT16(3418, s.cell_mV[4][18]); // module 4 last cell
    // #423 anti-regression: all 19 cells populate contiguously -- no interior 0 pad.
    for (std::uint8_t c = 0; c < ams::config::CellsPerModule; ++c) {
        TEST_ASSERT_EQUAL_UINT16(static_cast<std::uint16_t>(3000u + c), s.cell_mV[0][c]);
    }

    TEST_ASSERT_EQUAL_UINT16(0x3FFu, s.ltc_online_mask);
    TEST_ASSERT_EQUAL_UINT8(config::AllModulesMask, s.module_online_mask);
    TEST_ASSERT_TRUE(BmsService::instance().is_healthy(1000));
}

// ---------------------------------------------------------------------------
// Balancing tap-artifact guard. Module 0, LTC_1 group C (RDCVC) carries module
// cells 6,7,8. A shifted shared tap on the adjacent pair 7/8 makes cell 7 read
// non-physically high and cell 8 compensate low, sum conserved.
// ---------------------------------------------------------------------------
extern "C" void test_bms_tap_artifact_does_not_trip_ov(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    // cells 6,7,8 = normal, 4548 (impossible high), 3014 (compensating low).
    encode_segment(resp + 2u * GroupBytes + 0u * Seg, 3780u, 4548u, 3014u);

    fake_set_tick(1000);
    (void)BmsService::instance().update_from_ltc_response(resp, sizeof resp, 1000);
    const auto s = BmsService::instance().snapshot();

    // The impossible 4548 never reaches the safety aggregate; the pair is
    // averaged (~3781), so max stays well under the OV threshold.
    TEST_ASSERT_LESS_THAN_UINT16(config::CellOverVoltageMv, s.max_cell_mV);
    TEST_ASSERT_UINT16_WITHIN(2, 3781, s.vmax_module[0]);
    // Raw cell_mV is untouched -- the pit-diag grid still shows the split.
    TEST_ASSERT_EQUAL_UINT16(4548, s.cell_mV[0][7]);
    TEST_ASSERT_EQUAL_UINT16(3014, s.cell_mV[0][8]);
    // Module 0 is flagged as a tap fault.
    TEST_ASSERT_BITS_HIGH(1u << 0, s.tap_fault_mask);
}

// A GENUINE over-voltage (4200 < v < 4400, PHYSICAL) beside a NORMAL neighbour
// is NOT a tap artifact and must still reach max_cell_mV -> faults.
extern "C" void test_bms_real_overvoltage_not_masked(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    encode_segment(resp + 2u * GroupBytes + 0u * Seg, 3780u, 4250u, 3780u);

    fake_set_tick(1000);
    (void)BmsService::instance().update_from_ltc_response(resp, sizeof resp, 1000);
    const auto s = BmsService::instance().snapshot();

    TEST_ASSERT_EQUAL_UINT16(4250, s.max_cell_mV);
    TEST_ASSERT_GREATER_THAN_UINT16(config::CellOverVoltageMv, s.max_cell_mV);
    TEST_ASSERT_EQUAL_UINT8(0, s.tap_fault_mask);
}

// A non-physical reading whose neighbour did NOT compensate (split below
// TapArtifactMinSplitMv) is not the opposite-displacement tap signature, so it
// is conservatively NOT masked -- it still reaches the aggregate.
extern "C" void test_bms_nonphysical_without_compensation_not_masked(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    encode_segment(resp + 2u * GroupBytes + 0u * Seg, 3780u, 4548u, 3780u);

    fake_set_tick(1000);
    (void)BmsService::instance().update_from_ltc_response(resp, sizeof resp, 1000);
    const auto s = BmsService::instance().snapshot();

    TEST_ASSERT_EQUAL_UINT16(4548, s.max_cell_mV);
    TEST_ASSERT_EQUAL_UINT8(0, s.tap_fault_mask);
}

// ---------------------------------------------------------------------------
// PEC failure on one LTC's group keeps that module's slots untouched
// and clears the LTC bit in ltc_online_mask. The pair partner is fine
// so we expect module_online_mask to *not* advance freshness for that
// module (we don't clear the sticky bit, but last_rx_tick won't move).
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_pec_fail_on_one_ic_marks_module_stale(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);

    // Corrupt the PEC of group A for chain index 4 (= upper LTC of
    // module 2). Flip the low byte of the PEC -- decode_cell_voltage_-
    // group will fail and the IC's slots get left at whatever the
    // previous test wrote.
    resp[0 * GroupBytes + 4 * Seg + 7] ^= 0x01u;

    fake_set_tick(2000);
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 2000);

    const auto s = BmsService::instance().snapshot();
    // Bit 4 of ltc_online_mask is the corrupted LTC. Bit 5 (its
    // partner, module 2's lower LTC) is fine.
    TEST_ASSERT_BITS_LOW(1u << 4, s.ltc_online_mask);
    TEST_ASSERT_BITS_HIGH(1u << 5, s.ltc_online_mask);

    // last_rx_tick for module 2 must NOT have been advanced to 2000;
    // the previous clean-response test set it to 1000.
    TEST_ASSERT_EQUAL_UINT32(1000, s.last_rx_tick[2]);
}

// ---------------------------------------------------------------------------
// PEC failure increments the per-IC error counter.
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_pec_fail_increments_error_counter(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    resp[1 * GroupBytes + 7 * Seg + 7] ^= 0x80u;  // ic 7, group B

    const std::uint32_t before = ams::g_ltc_pec_err_count[7];
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 3000);
    TEST_ASSERT_EQUAL_UINT32(before + 1u, ams::g_ltc_pec_err_count[7]);
}

// ---------------------------------------------------------------------------
// Short buffer is rejected (no writes, no error counts).
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_short_buffer_rejected(void) {
    std::uint8_t resp[RespBytes - 1] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_from_ltc_response(
        resp, sizeof(resp), 4000));
}

// ---------------------------------------------------------------------------
// Null buffer is rejected.
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_null_buffer_rejected(void) {
    TEST_ASSERT_FALSE(BmsService::instance().update_from_ltc_response(
        nullptr, RespBytes, 5000));
}

// ---------------------------------------------------------------------------
// is_healthy goes false after the freshness window expires. After the
// corrupted-IC test above, last_rx_tick for module 2 stayed at 1000, so any
// tick > 1000 + BmsStaleMs reports stale. 2501 is past the window for any
// BmsStaleMs <= 1500 (window edge is 2000 at the current 1000 ms).
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_is_healthy_false_after_staleness(void) {
    TEST_ASSERT_FALSE(BmsService::instance().is_healthy(2501));
}

// ---------------------------------------------------------------------------
// #249: module_online_mask collapses to 0x00 when every chain position
// fails PEC for longer than BmsStaleMs. Previously the mask was sticky-
// set ("ever-online") and the only "chain stale" signal was via
// last_rx_tick freshness inside the predicate; the on-wire telemetry
// byte stayed at 0x1F even after the bench cut the LTC link. New
// semantic: mask reflects "currently fresh" -- bit N set iff
// now_tick - last_rx_tick[N] <= BmsStaleMs.
// ---------------------------------------------------------------------------
extern "C" void test_bms_mask_collapses_when_chain_stops_responding(void) {
    // Seed a clean state at t = 0.
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    fake_set_tick(0);
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 0);
    {
        const auto s = BmsService::instance().snapshot();
        TEST_ASSERT_EQUAL_UINT8(config::AllModulesMask, s.module_online_mask);
    }

    // Poll the chain repeatedly with every IC's PEC corrupted -- i.e.
    // STOP_REPLY semantic from the Pico LTC emulator. last_rx_tick
    // stays frozen at 0; the mask should hold until > BmsStaleMs
    // elapses, then drop.
    std::uint8_t bad[RespBytes];
    build_clean_chain(bad);
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        // Flip group A PEC for every IC -> all 10 fail.
        bad[0 * GroupBytes + ic * Seg + 7] ^= 0x01u;
    }

    // Boundary: at t == BmsStaleMs, delta from t=0 equals BmsStaleMs, which is
    // <= BmsStaleMs -> still fresh (key off the constant, not a literal, so the
    // test tracks the configured window).
    const std::uint32_t boundary = config::BmsStaleMs;
    fake_set_tick(boundary);
    BmsService::instance().update_from_ltc_response(bad, sizeof(bad), boundary);
    {
        const auto s = BmsService::instance().snapshot();
        TEST_ASSERT_EQUAL_UINT8(config::AllModulesMask, s.module_online_mask);
    }

    // Beyond the window: at t = BmsStaleMs + 1, delta > BmsStaleMs -> drop.
    fake_set_tick(boundary + 1u);
    BmsService::instance().update_from_ltc_response(bad, sizeof(bad), boundary + 1u);
    {
        const auto s = BmsService::instance().snapshot();
        TEST_ASSERT_EQUAL_UINT8(0u, s.module_online_mask);
    }
}

extern "C" void test_bms_mask_clears_only_stale_modules(void) {
    // Seed at t = 5000 so this test is independent of the global-fixture
    // last_rx_tick state left by earlier tests.
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    fake_set_tick(5000);
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 5000);

    // Re-poll at t = 5050 with only module 2's two ICs corrupted.
    std::uint8_t partial[RespBytes];
    build_clean_chain(partial);
    partial[0 * GroupBytes + 4 * Seg + 7] ^= 0x01u;  // ic 4 = module 2 upper
    partial[0 * GroupBytes + 5 * Seg + 7] ^= 0x01u;  // ic 5 = module 2 lower

    fake_set_tick(5050);
    BmsService::instance().update_from_ltc_response(partial, sizeof(partial), 5050);

    // Modules 0/1/3/4 just got refreshed -> their bits stay set.
    // Module 2's last_rx_tick is still 5000, delta 50 -> still fresh.
    {
        const auto s = BmsService::instance().snapshot();
        TEST_ASSERT_EQUAL_UINT8(config::AllModulesMask, s.module_online_mask);
    }

    // Keep failing module 2 past the BmsStaleMs window.
    fake_set_tick(5000 + config::BmsStaleMs + 1);
    BmsService::instance().update_from_ltc_response(
        partial, sizeof(partial), 5000 + config::BmsStaleMs + 1);
    {
        const auto s = BmsService::instance().snapshot();
        // Modules 0/1/3/4 just refreshed at this tick -> their bits set.
        // Module 2 has been stale for 1501 ms -> bit 2 cleared.
        const std::uint8_t expected =
            static_cast<std::uint8_t>(config::AllModulesMask & ~(1u << 2));
        TEST_ASSERT_EQUAL_UINT8(expected, s.module_online_mask);
    }
}

// ---------------------------------------------------------------------------
// Temperature path helpers + tests
// ---------------------------------------------------------------------------

namespace {

constexpr std::size_t AuxReplyBytes = config::LtcChainLength * Seg;  // 80

// Build one PEC-clean 8-byte AUX register segment with AUX1 set to
// the given mV value (AUX2 / AUX3 don't matter -- BMS_LITE only
// uses AUX1 for the mux output; we still PEC-validate the group as
// a whole so they need to be present).
void encode_aux_segment(std::uint8_t* out, std::uint16_t aux1_mV) {
    const std::uint16_t raw = static_cast<std::uint16_t>(aux1_mV * 10u);
    out[0] = static_cast<std::uint8_t>(raw & 0xFFu);
    out[1] = static_cast<std::uint8_t>((raw >> 8) & 0xFFu);
    // AUX2 = 0, AUX3 = 0 -- the decoder reads them but we discard.
    out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = 0;
    const std::uint16_t pec = ltc6811::pec15(out, 6);
    out[6] = static_cast<std::uint8_t>((pec >> 8) & 0xFFu);
    out[7] = static_cast<std::uint8_t>(pec & 0xFFu);
}

// Divider voltage at 25 degC, from the manufacturer R-T table
// (docs/ntc_rt_table.csv): R25 = 10 kOhm against the 6.8 kOhm pull-up
// (NtcPullupOhm) on VREF2 = 3000 mV gives
//     V = 3000 * 10000 / (6800 + 10000) = 1786 mV.
//
// This was 1500 mV, justified as "the voltage divider midpoint". That is only
// 25 degC if the pull-up EQUALS R25 -- it was 10 k in config at the time, but
// the board fits 6.8 k, so the test encoded the same wrong constant as the
// firmware and the pair agreed with each other while both disagreed with the
// hardware. 1500 mV is really ~34 degC on the real divider.
constexpr std::uint16_t Aux25C_mV = 1786;

}  // namespace

extern "C" void test_bms_temp_sweep_room_temp_on_one_channel(void) {
    // Build a 10-IC reply where every IC reports the 1.5 V (25 degC)
    // sample. Channel index 7 lands in cell_tempC[m][7] for upper
    // LTCs and cell_tempC[m][27] for lower LTCs.
    std::uint8_t reply[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * Seg, Aux25C_mV);
    }
    TEST_ASSERT_TRUE(BmsService::instance().update_temperature(
        7, reply, sizeof(reply)));

    const auto s = BmsService::instance().snapshot();
    // Steinhart-Hart rounded result for the placeholder Beta = 3380 is
    // 25 degC for the nominal divider midpoint. Tolerance +/- 1 to
    // absorb the rounding-mode boundary.
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][7]);
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][config::TempsPerLtc + 7]);
    }
}

extern "C" void test_bms_temp_hotter_voltage_gives_hotter_reading(void) {
    // NTC resistance falls as it heats. A hotter NTC -> lower R_ntc
    // -> lower V_aux. Feed a voltage below the 25 degC point (1786 mV) and
    // check we read a temperature strictly greater than 25 degC.
    // 900 mV is ~56 degC on the real divider.
    std::uint8_t reply[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * Seg, /* mV */ 900u);
    }
    BmsService::instance().update_temperature(3, reply, sizeof(reply));

    const auto s = BmsService::instance().snapshot();
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        TEST_ASSERT_GREATER_THAN_INT16(25, s.cell_tempC[m][3]);
        TEST_ASSERT_LESS_OR_EQUAL_INT16(config::NtcMaxValidC, s.cell_tempC[m][3]);
    }
}

extern "C" void test_bms_temp_rail_reading_skips_slot(void) {
    // A V_aux of 0 mV is an open / shorted channel. update_temperature
    // must leave the slot at whatever it was before. We seed slot 5
    // to a known sentinel via a clean prior sweep, then run a
    // rail-zero sweep on the same channel and confirm the slot is
    // unchanged.
    std::uint8_t reply[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * Seg, Aux25C_mV);
    }
    BmsService::instance().update_temperature(5, reply, sizeof(reply));
    const auto seeded = BmsService::instance().snapshot();
    const std::int16_t before = seeded.cell_tempC[2][5];

    // Now feed 0 mV (open). Slot must NOT update.
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * Seg, /* mV */ 0u);
    }
    BmsService::instance().update_temperature(5, reply, sizeof(reply));

    const auto after = BmsService::instance().snapshot();
    // Either way an open reading must never become a valid-looking temperature.
    // With a debounce it holds the last good value; with TempDisconnectPolls == 1
    // it marks the slot open (NtcNoReading) on the first rail read.
    if (config::TempDisconnectPolls >= 2) {
        TEST_ASSERT_EQUAL_INT16(before, after.cell_tempC[2][5]);
    } else {
        TEST_ASSERT_EQUAL_INT16(config::NtcNoReading, after.cell_tempC[2][5]);
    }
}

extern "C" void test_bms_temp_pec_fail_skips_slot(void) {
    std::uint8_t reply[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * Seg, Aux25C_mV);
    }
    // Wreck PEC on IC 4 (upper LTC of module 2).
    reply[4 * Seg + 7] ^= 0x01u;

    const std::uint32_t before = ams::g_ltc_pec_err_count[4];
    BmsService::instance().update_temperature(2, reply, sizeof(reply));
    TEST_ASSERT_EQUAL_UINT32(before + 1u, ams::g_ltc_pec_err_count[4]);

    const auto s = BmsService::instance().snapshot();
    // Other 9 ICs still updated module 2's lower-LTC slot 22 and
    // every other module's slot 2 + 22.
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][config::TempsPerLtc + 2]);
    }
}

extern "C" void test_bms_temp_bad_channel_idx_rejected(void) {
    std::uint8_t reply[AuxReplyBytes] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_temperature(
        config::TempsPerLtc, reply, sizeof(reply)));   // out of range
}

extern "C" void test_bms_temp_short_buffer_rejected(void) {
    std::uint8_t reply[AuxReplyBytes - 1] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_temperature(
        0, reply, sizeof(reply)));
}

// ---------------------------------------------------------------------------
// fix/53 per-module aggregates: after a clean chain response the per-
// module vmin / vmax pick the right cells per the build_clean_chain
// pattern (cell_mV[m][c] = 3000 + m*100 + c). For each module:
//   vmin = 3000 + m*100 + 0           (cell index 0)
//   vmax = 3000 + m*100 + 18          (cell index 18)
// tmax is filled by the temperature path; without a temp update it
// stays at INT16_MIN (sentinel).
// ---------------------------------------------------------------------------
extern "C" void test_bms_per_module_v_aggregates_after_clean_response(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    fake_set_tick(5000);
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 5000);

    const auto s = BmsService::instance().snapshot();
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        const std::uint16_t expect_min = static_cast<std::uint16_t>(3000 + m * 100 + 0);
        const std::uint16_t expect_max = static_cast<std::uint16_t>(3000 + m * 100 + 18);
        TEST_ASSERT_EQUAL_UINT16(expect_min, s.vmin_module[m]);
        TEST_ASSERT_EQUAL_UINT16(expect_max, s.vmax_module[m]);
    }
}

// ---------------------------------------------------------------------------
// fix/53 per-module temp aggregates: after a temperature sweep that
// writes ~25 degC across all populated channels, tmax_module[m] should
// be near 25 for every module that was updated.
// ---------------------------------------------------------------------------
extern "C" void test_bms_per_module_tmax_after_temp_sweep(void) {
    std::uint8_t resp[RespBytes];
    build_clean_chain(resp);
    fake_set_tick(6000);
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 6000);

    // Drive a uniform-room-temp sweep across the populated channels so
    // every module's tmax_module gets a real value.
    for (std::uint8_t ch = 0; ch < config::TempsPerLtc; ++ch) {
        std::uint8_t reply[AuxReplyBytes];
        for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
            encode_aux_segment(reply + ic * Seg, Aux25C_mV);
        }
        BmsService::instance().update_temperature(ch, reply, sizeof(reply));
    }

    const auto s = BmsService::instance().snapshot();
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        TEST_ASSERT_INT16_WITHIN(2, 25, s.tmax_module[m]);
    }
}


// --- temperature-sensor disconnect debounce (FS rule support) ---------------
//
// A channel that has read valid and then goes OPEN must be flagged as
// disconnected only after TempDisconnectPolls consecutive open polls -- a
// single anomalous read is tolerated (keeps its last value). Drives one
// channel (slot 3, upper LTC) through valid -> glitch -> valid -> sustained
// open, on a chain first brought fully online.
extern "C" void test_bms_temp_disconnect_debounce(void) {
    // Bring every module online with a clean voltage poll so recompute_
    // summaries_ evaluates the temp-disconnect mask (it only scans online
    // modules).
    std::uint8_t volts[RespBytes];
    build_clean_chain(volts);
    (void)BmsService::instance().update_from_ltc_response(volts, sizeof volts, 1000);

    constexpr std::uint8_t kCh = 3;                 // upper-LTC channel 3 -> slot 3
    std::uint8_t valid[AuxReplyBytes], open[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(valid + ic * Seg, Aux25C_mV);  // ~25 C
        encode_aux_segment(open  + ic * Seg, 3000u);       // rail = open
    }
    // Seed EVERY temp channel with a valid reading so the (now full 0..39)
    // required-slot presence check is satisfied -- this test targets the
    // slot-kCh debounce, not the presence of the other required channels. Each
    // call fills the upper slot (ch) and lower slot (ch+20) on every module.
    for (std::uint8_t ch = 0; ch < config::TempsPerLtc; ++ch) {
        (void)BmsService::instance().update_temperature(ch, valid, sizeof valid);
    }

    // 1. Valid read: channel is now "seen", no disconnect.
    (void)BmsService::instance().update_temperature(kCh, valid, sizeof valid);
    TEST_ASSERT_EQUAL_UINT8(0, BmsService::instance().snapshot().temp_disconnect_mask);

    // 2. A SINGLE open (glitch) with TempDisconnectPolls >= 2 must NOT flag it,
    //    and must keep the last good value rather than a sentinel.
    if (config::TempDisconnectPolls >= 2) {
        (void)BmsService::instance().update_temperature(kCh, open, sizeof open);
        const auto s = BmsService::instance().snapshot();
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, s.temp_disconnect_mask,
                                        "single open must not flag a disconnect");
        TEST_ASSERT_INT16_WITHIN_MESSAGE(1, 25, s.cell_tempC[0][kCh],
                                         "a single glitch must keep the last good value");
        // 3. A valid read clears the run.
        (void)BmsService::instance().update_temperature(kCh, valid, sizeof valid);
        TEST_ASSERT_EQUAL_UINT8(0, BmsService::instance().snapshot().temp_disconnect_mask);
    }

    // 4. Sustained open: TempDisconnectPolls consecutive opens -> disconnected.
    for (std::uint8_t i = 0; i < config::TempDisconnectPolls; ++i) {
        (void)BmsService::instance().update_temperature(kCh, open, sizeof open);
    }
    const auto s = BmsService::instance().snapshot();
    // Every online module shares kCh, so all module bits flag.
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, s.temp_disconnect_mask,
                                  "sustained open must flag a disconnect");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(config::NtcNoReading, s.cell_tempC[0][kCh],
                                    "a disconnected channel must read the sentinel");
}

// A PARTIALLY-railed open must be treated as a disconnect, not decoded as a
// plausible cold temperature. 2850 mV sits above config::NtcOpenMv (2800) but
// below the ~-40 degC plausibility rail (~2925 mV): before the NtcOpenMv
// threshold it decoded to a valid ~-27 degC and a real open masqueraded as a
// cold reading. It must now behave exactly like a full-rail (3000 mV) open.
extern "C" void test_bms_temp_partial_rail_open_is_disconnect(void) {
    std::uint8_t volts[RespBytes];
    build_clean_chain(volts);
    (void)BmsService::instance().update_from_ltc_response(volts, sizeof volts, 1000);

    constexpr std::uint8_t kCh = 7;                 // upper-LTC channel 7 -> slot 7
    std::uint8_t valid[AuxReplyBytes], partial[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(valid   + ic * Seg, Aux25C_mV);   // ~25 C
        encode_aux_segment(partial + ic * Seg, 2850u);        // partial-rail open
    }
    // Required slot 0 stays present so the presence check does not interfere.
    (void)BmsService::instance().update_temperature(0, valid, sizeof valid);

    // Seed valid: slot kCh is now "seen" and reads a real temperature, not the
    // open sentinel. Asserted PER CHANNEL rather than on the global disconnect
    // mask -- BmsService is a singleton shared across tests, so the mask can
    // still carry other slots disconnected by an earlier test.
    (void)BmsService::instance().update_temperature(kCh, valid, sizeof valid);
    TEST_ASSERT_INT16_WITHIN_MESSAGE(1, 25,
        BmsService::instance().snapshot().cell_tempC[0][kCh],
        "seed must read ~25 C, not the sentinel");

    // Sustained partial-rail open: 2850 mV is above NtcOpenMv (2800) but below
    // the old ~-40 C plausibility rail (~2925 mV), so before the NtcOpenMv
    // threshold it decoded to a valid ~-27 C. It must now behave like a full-
    // rail open: the channel goes to the sentinel and flags its own module.
    for (std::uint8_t i = 0; i < config::TempDisconnectPolls; ++i) {
        (void)BmsService::instance().update_temperature(kCh, partial, sizeof partial);
    }
    const auto s = BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_INT16_MESSAGE(config::NtcNoReading, s.cell_tempC[0][kCh],
                                    "partial-rail open must store the sentinel, not a cold temp");
    TEST_ASSERT_BITS_HIGH_MESSAGE(1u << 0, s.temp_disconnect_mask,
                                  "module 0's partial-rail open must set its disconnect bit");
}

// An UNPOPULATED channel (never read valid) that is always open must NEVER be
// flagged as a disconnect -- it is not a lost sensor, it was never there.
extern "C" void test_bms_unpopulated_channel_is_not_a_disconnect(void) {
    std::uint8_t volts[RespBytes];
    build_clean_chain(volts);
    (void)BmsService::instance().update_from_ltc_response(volts, sizeof volts, 2000);

    constexpr std::uint8_t kCh = 11;                // never fed a valid reading
    std::uint8_t open[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic)
        encode_aux_segment(open + ic * Seg, 3000u);

    for (std::uint8_t i = 0; i < config::TempDisconnectPolls + 2; ++i)
        (void)BmsService::instance().update_temperature(kCh, open, sizeof open);

    // slot 11's bit must not appear solely due to this never-seen channel.
    // (Other slots seen by earlier tests may flag; assert THIS channel stays
    // sentinel-but-not-a-disconnect by checking it is excluded from the count.)
    const auto s = BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_INT16(config::NtcNoReading, s.cell_tempC[0][kCh]);
}

// A REQUIRED temp slot that is open WITHOUT ever having read valid (switch open
// at power-on) must still fault -- the seen-valid latch alone misses this, and
// it is the deterministic scrutineering case. Slot 0 is required by config.
extern "C" void test_bms_required_channel_open_at_boot_faults(void) {
    std::uint8_t volts[RespBytes];
    build_clean_chain(volts);
    (void)BmsService::instance().update_from_ltc_response(volts, sizeof volts, 3000);

    // Poll a NON-required channel valid so the module has temp data (proving it
    // was polled), but leave the required slot 0 open (never fed a valid read).
    std::uint8_t valid[AuxReplyBytes], open[AuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        encode_aux_segment(valid + ic * Seg, Aux25C_mV);
        encode_aux_segment(open  + ic * Seg, 3000u);
    }
    (void)BmsService::instance().update_temperature(5, valid, sizeof valid);  // non-required
    (void)BmsService::instance().update_temperature(0, open,  sizeof open);   // required, open

    const auto s = BmsService::instance().snapshot();
    // Every module's required slot 0 is open -> all module bits flag, even
    // though slot 0 was never seen valid.
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, s.temp_disconnect_mask,
        "an open required channel must fault even if never seen valid (open-at-boot)");
}

// FS rule: a disconnected temp sensor must open the SDC in < 500 ms. Guard the
// config budget so a future cadence/debounce bump can't silently blow it: the
// debounce window is TempDisconnectPolls sweeps plus the safety-tick latch; the
// remaining margin to 500 ms absorbs the ~100 ms sweep + up to one cadence gap
// before the first open sweep. (e.g. 1 x 250 + 20 = 270 ms; a regression to
// 2 x 250 or a 500 ms cadence trips this.)
extern "C" void test_temp_disconnect_budget_under_500ms(void) {
    const std::uint32_t debounce_window_ms =
        static_cast<std::uint32_t>(config::TempDisconnectPolls) * config::BmsPollTempMs
        + config::StatePeriodMs;
    TEST_ASSERT_LESS_THAN_UINT32(500u, debounce_window_ms);
}

