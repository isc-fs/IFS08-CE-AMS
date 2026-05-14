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

constexpr std::size_t kSeg        = 8;
constexpr std::size_t kGroupBytes = config::kLtcChainLength * kSeg;  // 80
constexpr std::size_t kRespBytes  = 4u * kGroupBytes;                // 320

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
    std::memset(out, 0, kRespBytes);
    auto mV = [](std::uint8_t m, std::uint8_t s) -> std::uint16_t {
        return static_cast<std::uint16_t>(3000u + 100u * m + s);
    };
    auto seg_at = [&](std::uint8_t group, std::uint8_t ic) -> std::uint8_t* {
        return out + group * kGroupBytes + ic * kSeg;
    };

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        const std::uint8_t ic_upper = static_cast<std::uint8_t>(2u * m);
        const std::uint8_t ic_lower = static_cast<std::uint8_t>(2u * m + 1u);

        // LTC_1 (upper, 10 cells). Group D contains only one real
        // cell (slot 9); the remaining two slots in that segment
        // must still PEC-validate even though the values are
        // discarded by the decoder.
        encode_segment(seg_at(0, ic_upper), mV(m, 0), mV(m, 1), mV(m, 2));
        encode_segment(seg_at(1, ic_upper), mV(m, 3), mV(m, 4), mV(m, 5));
        encode_segment(seg_at(2, ic_upper), mV(m, 6), mV(m, 7), mV(m, 8));
        encode_segment(seg_at(3, ic_upper), mV(m, 9),
                       /* unused */ 0x0FFFu, /* unused */ 0x0FFFu);

        // LTC_2 (lower, 9 cells). Group D is entirely discarded but
        // its PEC must still validate or the decoder will mark the
        // IC offline.
        encode_segment(seg_at(0, ic_lower), mV(m, 10), mV(m, 11), mV(m, 12));
        encode_segment(seg_at(1, ic_lower), mV(m, 13), mV(m, 14), mV(m, 15));
        encode_segment(seg_at(2, ic_lower), mV(m, 16), mV(m, 17), mV(m, 18));
        encode_segment(seg_at(3, ic_lower), 0x0FFFu, 0x0FFFu, 0x0FFFu);
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
extern "C" void test_bms_ltc_clean_response_decodes_all_cells(void) {
    std::uint8_t resp[kRespBytes];
    build_clean_chain(resp);

    fake_set_tick(1000);
    TEST_ASSERT_TRUE(BmsService::instance().update_from_ltc_response(resp,
                                                                     sizeof(resp),
                                                                     1000));

    const auto s = BmsService::instance().snapshot();
    // Spot-check across modules + corners of each LTC's cell range.
    TEST_ASSERT_EQUAL_UINT16(3000, s.cell_mV[0][0]);
    TEST_ASSERT_EQUAL_UINT16(3009, s.cell_mV[0][9]);   // top of LTC_1 module 0
    TEST_ASSERT_EQUAL_UINT16(3010, s.cell_mV[0][10]);  // bottom of LTC_2 module 0
    TEST_ASSERT_EQUAL_UINT16(3018, s.cell_mV[0][18]);  // top of LTC_2 module 0
    TEST_ASSERT_EQUAL_UINT16(3100, s.cell_mV[1][0]);
    TEST_ASSERT_EQUAL_UINT16(3418, s.cell_mV[4][18]); // module 4 last cell

    TEST_ASSERT_EQUAL_UINT16(0x3FFu, s.ltc_online_mask);
    TEST_ASSERT_EQUAL_UINT8(config::kAllModulesMask, s.module_online_mask);
    TEST_ASSERT_TRUE(BmsService::instance().is_healthy(1000));
}

// ---------------------------------------------------------------------------
// PEC failure on one LTC's group keeps that module's slots untouched
// and clears the LTC bit in ltc_online_mask. The pair partner is fine
// so we expect module_online_mask to *not* advance freshness for that
// module (we don't clear the sticky bit, but last_rx_tick won't move).
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_pec_fail_on_one_ic_marks_module_stale(void) {
    std::uint8_t resp[kRespBytes];
    build_clean_chain(resp);

    // Corrupt the PEC of group A for chain index 4 (= upper LTC of
    // module 2). Flip the low byte of the PEC -- decode_cell_voltage_-
    // group will fail and the IC's slots get left at whatever the
    // previous test wrote.
    resp[0 * kGroupBytes + 4 * kSeg + 7] ^= 0x01u;

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
    std::uint8_t resp[kRespBytes];
    build_clean_chain(resp);
    resp[1 * kGroupBytes + 7 * kSeg + 7] ^= 0x80u;  // ic 7, group B

    const std::uint32_t before = ams::g_ltc_pec_err_count[7];
    BmsService::instance().update_from_ltc_response(resp, sizeof(resp), 3000);
    TEST_ASSERT_EQUAL_UINT32(before + 1u, ams::g_ltc_pec_err_count[7]);
}

// ---------------------------------------------------------------------------
// Short buffer is rejected (no writes, no error counts).
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_short_buffer_rejected(void) {
    std::uint8_t resp[kRespBytes - 1] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_from_ltc_response(
        resp, sizeof(resp), 4000));
}

// ---------------------------------------------------------------------------
// Null buffer is rejected.
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_null_buffer_rejected(void) {
    TEST_ASSERT_FALSE(BmsService::instance().update_from_ltc_response(
        nullptr, kRespBytes, 5000));
}

// ---------------------------------------------------------------------------
// is_healthy goes false after the freshness window expires.
// kBmsStaleMs is 1500; after the corrupted-IC test above last_rx_tick
// for module 2 stayed at 1000, so any tick > 2500 must report stale.
// ---------------------------------------------------------------------------
extern "C" void test_bms_ltc_is_healthy_false_after_staleness(void) {
    TEST_ASSERT_FALSE(BmsService::instance().is_healthy(2501));
}

// ---------------------------------------------------------------------------
// Legacy update_from_frame is a no-op (returns false). Sanity check
// to lock in that the old CAN data path is genuinely retired.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Temperature path helpers + tests
// ---------------------------------------------------------------------------

namespace {

constexpr std::size_t kAuxReplyBytes = config::kLtcChainLength * kSeg;  // 80

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

// Per the Beta model with R25 = 10 k, R_series = 10 k, V_ref = 3 V,
// B = 3380 K: room temperature (25 degC) gives R = 10 k, voltage
// divider midpoint -> V_aux = 1.5 V = 1500 mV. Use that as the
// reference test input where we want exactly 25 degC out.
constexpr std::uint16_t kAux25C_mV = 1500;

}  // namespace

extern "C" void test_bms_temp_sweep_room_temp_on_one_channel(void) {
    // Build a 10-IC reply where every IC reports the 1.5 V (25 degC)
    // sample. Channel index 7 lands in cell_tempC[m][7] for upper
    // LTCs and cell_tempC[m][27] for lower LTCs.
    std::uint8_t reply[kAuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * kSeg, kAux25C_mV);
    }
    TEST_ASSERT_TRUE(BmsService::instance().update_temperature(
        7, reply, sizeof(reply)));

    const auto s = BmsService::instance().snapshot();
    // Steinhart-Hart rounded result for the placeholder Beta = 3380 is
    // 25 degC for the nominal divider midpoint. Tolerance +/- 1 to
    // absorb the rounding-mode boundary.
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][7]);
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][config::kTempsPerLtc + 7]);
    }
}

extern "C" void test_bms_temp_hotter_voltage_gives_hotter_reading(void) {
    // NTC resistance falls as it heats. A hotter NTC -> lower R_ntc
    // -> lower V_aux. Feed a voltage below the 25 degC midpoint and
    // check we read a temperature strictly greater than 25 degC.
    std::uint8_t reply[kAuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * kSeg, /* mV */ 900u);
    }
    BmsService::instance().update_temperature(3, reply, sizeof(reply));

    const auto s = BmsService::instance().snapshot();
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        TEST_ASSERT_GREATER_THAN_INT16(25, s.cell_tempC[m][3]);
        TEST_ASSERT_LESS_OR_EQUAL_INT16(config::kNtcMaxValidC, s.cell_tempC[m][3]);
    }
}

extern "C" void test_bms_temp_rail_reading_skips_slot(void) {
    // A V_aux of 0 mV is an open / shorted channel. update_temperature
    // must leave the slot at whatever it was before. We seed slot 5
    // to a known sentinel via a clean prior sweep, then run a
    // rail-zero sweep on the same channel and confirm the slot is
    // unchanged.
    std::uint8_t reply[kAuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * kSeg, kAux25C_mV);
    }
    BmsService::instance().update_temperature(5, reply, sizeof(reply));
    const auto seeded = BmsService::instance().snapshot();
    const std::int16_t before = seeded.cell_tempC[2][5];

    // Now feed 0 mV (open). Slot must NOT update.
    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * kSeg, /* mV */ 0u);
    }
    BmsService::instance().update_temperature(5, reply, sizeof(reply));

    const auto after = BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_INT16(before, after.cell_tempC[2][5]);
}

extern "C" void test_bms_temp_pec_fail_skips_slot(void) {
    std::uint8_t reply[kAuxReplyBytes];
    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        encode_aux_segment(reply + ic * kSeg, kAux25C_mV);
    }
    // Wreck PEC on IC 4 (upper LTC of module 2).
    reply[4 * kSeg + 7] ^= 0x01u;

    const std::uint32_t before = ams::g_ltc_pec_err_count[4];
    BmsService::instance().update_temperature(2, reply, sizeof(reply));
    TEST_ASSERT_EQUAL_UINT32(before + 1u, ams::g_ltc_pec_err_count[4]);

    const auto s = BmsService::instance().snapshot();
    // Other 9 ICs still updated module 2's lower-LTC slot 22 and
    // every other module's slot 2 + 22.
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        TEST_ASSERT_INT16_WITHIN(1, 25, s.cell_tempC[m][config::kTempsPerLtc + 2]);
    }
}

extern "C" void test_bms_temp_bad_channel_idx_rejected(void) {
    std::uint8_t reply[kAuxReplyBytes] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_temperature(
        config::kTempsPerLtc, reply, sizeof(reply)));   // out of range
}

extern "C" void test_bms_temp_short_buffer_rejected(void) {
    std::uint8_t reply[kAuxReplyBytes - 1] = {};
    TEST_ASSERT_FALSE(BmsService::instance().update_temperature(
        0, reply, sizeof(reply)));
}

extern "C" void test_bms_legacy_can_path_is_inert(void) {
    CanFrame f = {};
    f.id  = 0x12D;
    f.dlc = 8;
    f.bus = static_cast<std::uint8_t>(CanBus::Bms);
    f.data[0] = 0x12; f.data[1] = 0x34;
    TEST_ASSERT_FALSE(BmsService::instance().update_from_frame(f));
}
