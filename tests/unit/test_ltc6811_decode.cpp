// SPDX-License-Identifier: proprietary
//
// Tests for the LTC6811-1 pure-logic protocol layer in
// Core/Src/app/ltc6811.cpp -- PEC15, command builders, register-group
// decoders, and the ADG731 mux address packing.

#include "ltc6811.hpp"

#include "unity.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace {

using namespace ams::ltc6811;

// Reverse of pack_command's PEC computation: pull the cmd bytes back
// out and check the PEC byte pair against an independent reference.
std::uint16_t compute_pec(std::initializer_list<std::uint8_t> bytes) {
    std::array<std::uint8_t, 16> buf{};
    std::size_t i = 0;
    for (auto b : bytes) buf[i++] = b;
    return pec15(buf.data(), bytes.size());
}

}  // namespace

// ---------------------------------------------------------------------------
// PEC15 sanity tests
// ---------------------------------------------------------------------------

// The LTC6811 datasheet section "Packet Error Code" gives the
// canonical worked example for the command bytes [0x00, 0x01]
// (WRCFGA). The expected PEC is 0x3D6E.
extern "C" void test_ltc6811_pec_datasheet_wrcfga(void) {
    const std::uint8_t cmd[2] = { 0x00, 0x01 };
    TEST_ASSERT_EQUAL_HEX16(0x3D6Eu, pec15(cmd, 2));
}

// Empty input must return the seed (shifted left by 1 = 0x0020).
extern "C" void test_ltc6811_pec_empty(void) {
    TEST_ASSERT_EQUAL_HEX16(0x0020u, pec15(nullptr, 0));
}

// PEC15 must be deterministic. Same input → same output.
extern "C" void test_ltc6811_pec_deterministic(void) {
    const std::uint8_t buf[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };
    const auto a = pec15(buf, 6);
    const auto b = pec15(buf, 6);
    TEST_ASSERT_EQUAL_HEX16(a, b);
}

// One-bit change in the payload must change the PEC (CRC sensitivity).
extern "C" void test_ltc6811_pec_sensitivity(void) {
    const std::uint8_t base[6]    = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const std::uint8_t flipped[6] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
    TEST_ASSERT_NOT_EQUAL(pec15(base, 6), pec15(flipped, 6));
}

// The on-wire PEC always has bit 0 = 0 (the algorithm shifts left by 1).
extern "C" void test_ltc6811_pec_lsb_is_zero(void) {
    const std::uint8_t buf[2] = { 0x00, 0x01 };
    TEST_ASSERT_EQUAL_HEX8(0, pec15(buf, 2) & 0x01u);
}

// ---------------------------------------------------------------------------
// pack_command
// ---------------------------------------------------------------------------

extern "C" void test_ltc6811_pack_command_wrcfga(void) {
    const auto frame = pack_command(CmdWRCFGA);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, frame[1]);
    TEST_ASSERT_EQUAL_HEX8(0x3D, frame[2]);   // 0x3D6E high byte
    TEST_ASSERT_EQUAL_HEX8(0x6E, frame[3]);   // 0x3D6E low byte
}

extern "C" void test_ltc6811_pack_command_rdcva(void) {
    const auto frame = pack_command(CmdRDCVA);  // 0x0004
    TEST_ASSERT_EQUAL_HEX8(0x00, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, frame[1]);
    // PEC of [0x00, 0x04] should match our table-driven implementation.
    const std::uint16_t crc = compute_pec({ 0x00, 0x04 });
    TEST_ASSERT_EQUAL_HEX8((crc >> 8) & 0xFFu, frame[2]);
    TEST_ASSERT_EQUAL_HEX8(crc & 0xFFu,        frame[3]);
}

// ---------------------------------------------------------------------------
// ADCV / ADAX command composition
// ---------------------------------------------------------------------------

// ADCV in 7kHz normal mode, with discharge permitted during the
// conversion (the operational default -- DCP=0 inhibits balancing
// FETs every poll, which throws away cycle time and hurts balance
// throughput). Per datasheet worked example this command bytes to
// 0x0370. DCP=0 would give 0x0360 (used only when an unbalanced
// reading is needed for diagnostics).
extern "C" void test_ltc6811_adcv_norm_dcp1_all_cells(void) {
    const std::uint16_t c = adcv_cmd(AdcMode::Norm7kHz, /*dcp=*/true, CellSel::All);
    TEST_ASSERT_EQUAL_HEX16(0x0370u, c);
}

extern "C" void test_ltc6811_adcv_norm_dcp0_all_cells(void) {
    const std::uint16_t c = adcv_cmd(AdcMode::Norm7kHz, /*dcp=*/false, CellSel::All);
    TEST_ASSERT_EQUAL_HEX16(0x0360u, c);
}

// ADAX in 7kHz mode, all AUX channels -- our standard "read NTC"
// command. Per datasheet table 40 this is 0x0560.
extern "C" void test_ltc6811_adax_norm_all_aux(void) {
    const std::uint16_t c = adax_cmd(AdcMode::Norm7kHz, AuxSel::All);
    TEST_ASSERT_EQUAL_HEX16(0x0560u, c);
}

// ---------------------------------------------------------------------------
// Register-group decoders
// ---------------------------------------------------------------------------

// Encode 3 known cell voltages, append PEC, decode -> values
// round-trip.
extern "C" void test_ltc6811_decode_cell_voltage_roundtrip(void) {
    // 3700 mV = 37000 in 100µV units = 0x9088
    // 4100 mV = 41000               = 0xA028
    // 3850 mV = 38500               = 0x9664
    std::uint8_t bytes[8] = {
        0x88, 0x90,   // little-endian 0x9088 -> cell0
        0x28, 0xA0,   // 0xA028 -> cell1
        0x64, 0x96,   // 0x9664 -> cell2
        0, 0          // PEC filled below
    };
    const std::uint16_t crc = pec15(bytes, 6);
    bytes[6] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    bytes[7] = static_cast<std::uint8_t>(crc & 0xFFu);

    std::array<std::uint16_t, 3> out{};
    TEST_ASSERT_TRUE(decode_cell_voltage_group(bytes, out));
    TEST_ASSERT_EQUAL_UINT16(3700, out[0]);
    TEST_ASSERT_EQUAL_UINT16(4100, out[1]);
    TEST_ASSERT_EQUAL_UINT16(3850, out[2]);
}

// PEC mismatch -- decoder must return false and not touch out.
extern "C" void test_ltc6811_decode_cell_voltage_bad_pec(void) {
    std::uint8_t bytes[8] = {
        0x88, 0x90, 0x28, 0xA0, 0x64, 0x96,
        0xDE, 0xAD,   // deliberately wrong PEC
    };
    std::array<std::uint16_t, 3> out = { 1, 2, 3 };
    TEST_ASSERT_FALSE(decode_cell_voltage_group(bytes, out));
    // Out untouched.
    TEST_ASSERT_EQUAL_UINT16(1, out[0]);
    TEST_ASSERT_EQUAL_UINT16(2, out[1]);
    TEST_ASSERT_EQUAL_UINT16(3, out[2]);
}

// Decoder for AUX groups uses the same wire layout (3 × 16-bit
// little-endian samples in 100 µV units, plus 2 PEC bytes).
// 3.000 V = 30000 -> 0x7530
// 2.000 V = 20000 -> 0x4E20
// 1.500 V = 15000 -> 0x3A98
extern "C" void test_ltc6811_decode_aux_voltage_roundtrip(void) {
    std::uint8_t bytes[8] = {
        0x30, 0x75,   // 0x7530 -> 3000 mV
        0x20, 0x4E,   // 0x4E20 -> 2000 mV
        0x98, 0x3A,   // 0x3A98 -> 1500 mV
        0, 0
    };
    const std::uint16_t crc = pec15(bytes, 6);
    bytes[6] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    bytes[7] = static_cast<std::uint8_t>(crc & 0xFFu);

    std::array<std::uint16_t, 3> out{};
    TEST_ASSERT_TRUE(decode_aux_voltage_group(bytes, out));
    TEST_ASSERT_EQUAL_UINT16(3000, out[0]);
    TEST_ASSERT_EQUAL_UINT16(2000, out[1]);
    TEST_ASSERT_EQUAL_UINT16(1500, out[2]);
}

// ---------------------------------------------------------------------------
// build_write_frame -- chain-of-10 layout
// ---------------------------------------------------------------------------

extern "C" void test_ltc6811_build_write_frame_chain10(void) {
    // 10 per-IC payloads, each filled with a distinct pattern so we
    // can prove the right bytes land in the right slot.
    std::uint8_t per_ic[10][6];
    for (std::uint8_t ic = 0; ic < 10; ++ic) {
        for (std::uint8_t k = 0; k < 6; ++k) {
            per_ic[ic][k] = static_cast<std::uint8_t>((ic + 1) * 0x10u + k);
        }
    }

    std::uint8_t out[4 + 8 * 10] = {};
    build_write_frame(CmdWRCFGA, per_ic, out, sizeof(out));

    // Command frame in the first 4 bytes.
    TEST_ASSERT_EQUAL_HEX8(0x00, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x3D, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x6E, out[3]);

    // Each per-IC segment: 6 data bytes match, 2 PEC bytes validate.
    for (std::uint8_t ic = 0; ic < 10; ++ic) {
        const std::uint8_t* seg = &out[4 + 8 * ic];
        for (std::uint8_t k = 0; k < 6; ++k) {
            TEST_ASSERT_EQUAL_HEX8(per_ic[ic][k], seg[k]);
        }
        const std::uint16_t expected_pec = pec15(seg, 6);
        const std::uint16_t actual_pec =
            static_cast<std::uint16_t>((seg[6] << 8) | seg[7]);
        TEST_ASSERT_EQUAL_HEX16(expected_pec, actual_pec);
    }
}

// Refuses to write past a short buffer.
extern "C" void test_ltc6811_build_write_frame_short_buffer_safe(void) {
    std::uint8_t per_ic[10][6] = {};
    std::uint8_t small[16];
    std::memset(small, 0xAA, sizeof(small));

    build_write_frame(CmdWRCFGA, per_ic, small, sizeof(small));

    // No bytes should have been touched -- still all 0xAA.
    for (std::size_t i = 0; i < sizeof(small); ++i) {
        TEST_ASSERT_EQUAL_HEX8(0xAA, small[i]);
    }
}

// ---------------------------------------------------------------------------
// pack_adg731_select
// ---------------------------------------------------------------------------

// Channel 5, enabled: data byte = 0x80 | (5 << 1) = 0x8A
// Slot 0 should drop CSBM LOW (ICOM=8) and release HIGH after (FCOM=9).
extern "C" void test_ltc6811_pack_adg731_channel_5(void) {
    const auto p = pack_adg731_select(5);

    // Slot 0 high byte: ICOM=8 | data_hi=0x8 -> 0x88
    TEST_ASSERT_EQUAL_HEX8(0x88, p[0]);
    // Slot 0 low byte:  data_lo=0xA | FCOM=9 -> 0xA9
    TEST_ASSERT_EQUAL_HEX8(0xA9, p[1]);
    // Slots 1, 2 marked "no transmission" (ICOM=0xF / FCOM=0xF)
    TEST_ASSERT_EQUAL_HEX8(0xF0, p[2]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, p[3]);
    TEST_ASSERT_EQUAL_HEX8(0xF0, p[4]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, p[5]);
}

// Channel 0, enabled: data byte = 0x80
extern "C" void test_ltc6811_pack_adg731_channel_0(void) {
    const auto p = pack_adg731_select(0);
    TEST_ASSERT_EQUAL_HEX8(0x88, p[0]);
    TEST_ASSERT_EQUAL_HEX8(0x09, p[1]);   // data_lo=0, FCOM=9
}

// Channel 31, enabled: data byte = 0x80 | (31 << 1) = 0xBE
extern "C" void test_ltc6811_pack_adg731_channel_31(void) {
    const auto p = pack_adg731_select(31);
    TEST_ASSERT_EQUAL_HEX8(0x8B, p[0]);   // ICOM=8, hi=0xB
    TEST_ASSERT_EQUAL_HEX8(0xE9, p[1]);   // lo=0xE, FCOM=9
}

// Channel field is only 5 bits wide. Anything above 31 must mask to
// the low 5 bits -- silently safe rather than encoding nonsense into
// the don't-care bit.
extern "C" void test_ltc6811_pack_adg731_channel_masked(void) {
    const auto p_low  = pack_adg731_select(0x05u);
    const auto p_high = pack_adg731_select(0xE5u);  // 0xE5 & 0x1F == 5
    TEST_ASSERT_EQUAL_HEX8(p_low[0], p_high[0]);
    TEST_ASSERT_EQUAL_HEX8(p_low[1], p_high[1]);
}

// ---------------------------------------------------------------------------
// count_pec_valid_segments -- chain length discovery walker
// ---------------------------------------------------------------------------

namespace {

// Build a synthetic chain reply: N segments of 6 data bytes + 2 PEC.
// Data bytes are arbitrary -- the walker only cares about PEC15
// matching, so we fill each segment with a unique pattern to make
// off-by-one bugs visible if a test ever asserts on the data side.
void make_valid_chain(std::uint8_t* out, std::size_t n_segments) {
    for (std::size_t i = 0; i < n_segments; ++i) {
        std::uint8_t* seg = out + 8 * i;
        for (std::size_t k = 0; k < 6; ++k) {
            seg[k] = static_cast<std::uint8_t>(0xA0u + (i * 7u) + k);
        }
        const std::uint16_t pec = pec15(seg, 6);
        seg[6] = static_cast<std::uint8_t>((pec >> 8) & 0xFFu);
        seg[7] = static_cast<std::uint8_t>(pec & 0xFFu);
    }
}

}  // namespace

// All 10 ICs answer with PEC-clean payloads -> count is 10.
extern "C" void test_ltc6811_chain_discovery_all_ten_valid(void) {
    std::uint8_t buf[8 * 10] = {};
    make_valid_chain(buf, 10);
    TEST_ASSERT_EQUAL_UINT8(10u, count_pec_valid_segments(buf, sizeof(buf), 10));
}

// One missing module: 9 valid segments then a PEC-bad segment in
// slot 10. Discovery stops at the first failure -> count is 9.
extern "C" void test_ltc6811_chain_discovery_nine_then_bad(void) {
    std::uint8_t buf[8 * 10] = {};
    make_valid_chain(buf, 10);
    // Corrupt the last segment's PEC.
    buf[8 * 9 + 7] ^= 0x01u;
    TEST_ASSERT_EQUAL_UINT8(9u, count_pec_valid_segments(buf, sizeof(buf), 10));
}

// 10 valid + trailing garbage past LtcChainLength. The walker caps
// at max_chain so the garbage is invisible -> count is 10.
extern "C" void test_ltc6811_chain_discovery_ten_plus_trailing_garbage(void) {
    std::uint8_t buf[8 * 12] = {};
    make_valid_chain(buf, 10);
    // Garbage in slots 10 and 11 -- random bytes, broken PECs.
    for (std::size_t k = 8 * 10; k < sizeof(buf); ++k) buf[k] = 0x5Au;
    TEST_ASSERT_EQUAL_UINT8(10u, count_pec_valid_segments(buf, sizeof(buf), 10));
}

// First segment already PEC-bad -> count is 0 (operator should see
// "ltc_chain=0 expected=10" and latch ERROR immediately).
extern "C" void test_ltc6811_chain_discovery_first_bad(void) {
    std::uint8_t buf[8 * 10] = {};
    make_valid_chain(buf, 10);
    buf[7] ^= 0xFFu;  // shred PEC of segment 0
    TEST_ASSERT_EQUAL_UINT8(0u, count_pec_valid_segments(buf, sizeof(buf), 10));
}

// Short buffer (less than max_chain * 8 bytes): walker stops at the
// available capacity. 7 full segments here -> at most 7 reported.
extern "C" void test_ltc6811_chain_discovery_short_buffer(void) {
    std::uint8_t buf[8 * 7] = {};
    make_valid_chain(buf, 7);
    TEST_ASSERT_EQUAL_UINT8(7u, count_pec_valid_segments(buf, sizeof(buf), 10));
}

// Null pointer guard -- shouldn't crash, just reports 0.
extern "C" void test_ltc6811_chain_discovery_null_safe(void) {
    TEST_ASSERT_EQUAL_UINT8(0u, count_pec_valid_segments(nullptr, 80, 10));
}
