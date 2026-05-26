// SPDX-License-Identifier: proprietary
//
// Tests for the pit-diag stream encoders (#247).
// Covers cell-frame / temp-frame layouts, FSM-status packing, timing
// frame clipping, and command parsing.

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "can_frame.hpp"
#include "pit_diag_emitter.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

ams::CanFrame make_pit_diag_cmd(const std::uint8_t magic[4]) {
    ams::CanFrame f = {};
    f.id  = ams::config::PitDiagCmdRxId;
    f.dlc = ams::config::PitDiagCmdDlc;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);
    std::memcpy(f.data, magic, 4);
    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Cell frame: row-major flat, 4 cells per frame, last frame sentinels.
// ---------------------------------------------------------------------------
extern "C" void test_pit_diag_cell_frame_zero_indexes_module0(void) {
    ams::BmsState bms = {};
    bms.cell_mV[0][0] = 0x1234;
    bms.cell_mV[0][1] = 0x5678;
    bms.cell_mV[0][2] = 0xABCD;
    bms.cell_mV[0][3] = 0x0001;

    const auto f = ams::pit_diag::encode_cell_frame(bms, 0);
    TEST_ASSERT_EQUAL_UINT8(0x12, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x56, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0x78, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0xAB, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xCD, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0x01, f[7]);
}

extern "C" void test_pit_diag_cell_frame_spans_module_boundary(void) {
    // Frame 4 (cells 16..19) spans cells 16,17,18 of module 0 + cell 0
    // of module 1 (row-major: 5*19=95 total, cells_per_module=19).
    ams::BmsState bms = {};
    bms.cell_mV[0][16] = 0xAA00;
    bms.cell_mV[0][17] = 0xBB11;
    bms.cell_mV[0][18] = 0xCC22;
    bms.cell_mV[1][0]  = 0xDD33;

    const auto f = ams::pit_diag::encode_cell_frame(bms, 4);
    TEST_ASSERT_EQUAL_UINT8(0xAA, f[0]);  // module 0, cell 16
    TEST_ASSERT_EQUAL_UINT8(0xBB, f[2]);  // module 0, cell 17
    TEST_ASSERT_EQUAL_UINT8(0xCC, f[4]);  // module 0, cell 18
    TEST_ASSERT_EQUAL_UINT8(0xDD, f[6]);  // module 1, cell 0
    TEST_ASSERT_EQUAL_UINT8(0x33, f[7]);
}

extern "C" void test_pit_diag_cell_frame_last_has_sentinels(void) {
    // 95 cells / 4 = 23.75 -> 24 frames. Frame 23 carries cells 92..94
    // (3 real cells) + one sentinel slot.
    ams::BmsState bms = {};
    // Cells 92..94 in row-major = module 4, cells 16..18.
    bms.cell_mV[4][16] = 0xAAAA;
    bms.cell_mV[4][17] = 0xBBBB;
    bms.cell_mV[4][18] = 0xCCCC;

    const auto f = ams::pit_diag::encode_cell_frame(bms, 23);
    TEST_ASSERT_EQUAL_UINT8(0xAA, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0xAA, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, f[5]);
    // Sentinel = 0xFFFF.
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[7]);
}

extern "C" void test_pit_diag_cell_frame_out_of_range_returns_zeros(void) {
    ams::BmsState bms = {};
    bms.cell_mV[0][0] = 0x1234;
    const auto f = ams::pit_diag::encode_cell_frame(bms, 99);
    for (auto b : f) TEST_ASSERT_EQUAL_UINT8(0u, b);
}

// ---------------------------------------------------------------------------
// Temp frame: 8 NTCs per frame, 25 frames covers 200 NTCs exactly.
// ---------------------------------------------------------------------------
extern "C" void test_pit_diag_temp_frame_first_byte_is_module0_temp0(void) {
    ams::BmsState bms = {};
    for (std::uint8_t i = 0; i < 8; ++i) {
        bms.cell_tempC[0][i] = static_cast<std::int16_t>(20 + i);
    }
    const auto f = ams::pit_diag::encode_temp_frame(bms, 0);
    for (std::uint8_t i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_INT8(20 + static_cast<int>(i),
                               static_cast<std::int8_t>(f[i]));
    }
}

extern "C" void test_pit_diag_temp_frame_clips_int8_range(void) {
    ams::BmsState bms = {};
    bms.cell_tempC[0][0] = -300;     // below int8 min
    bms.cell_tempC[0][1] =  300;     // above int8 max
    bms.cell_tempC[0][2] =   25;     // in-range
    const auto f = ams::pit_diag::encode_temp_frame(bms, 0);
    TEST_ASSERT_EQUAL_INT8(-128, static_cast<std::int8_t>(f[0]));
    TEST_ASSERT_EQUAL_INT8( 127, static_cast<std::int8_t>(f[1]));
    TEST_ASSERT_EQUAL_INT8(  25, static_cast<std::int8_t>(f[2]));
}

extern "C" void test_pit_diag_temp_frame_last_covers_module4(void) {
    // Frame 24 covers temp_indices 192..199 = module 4, temps 32..39
    // (TempsPerModule=40, last 8 of module 4).
    ams::BmsState bms = {};
    bms.cell_tempC[4][32] = 50;
    bms.cell_tempC[4][39] = -10;
    const auto f = ams::pit_diag::encode_temp_frame(bms, 24);
    TEST_ASSERT_EQUAL_INT8( 50, static_cast<std::int8_t>(f[0]));
    TEST_ASSERT_EQUAL_INT8(-10, static_cast<std::int8_t>(f[7]));
}

// ---------------------------------------------------------------------------
// FSM extended status frame.
// ---------------------------------------------------------------------------
extern "C" void test_pit_diag_fsm_status_layout(void) {
    const auto f = ams::pit_diag::encode_fsm_status(
        /*fsm_state=*/3u,                 // Run
        /*mode_locked=*/ams::fsm::Mode::Car,
        /*tsms=*/true,
        /*dash_chg=*/true,
        /*ams_ok_gpio=*/1u,
        /*pec_err_total=*/0x1234u);
    TEST_ASSERT_EQUAL_UINT8(3u, f[0]);
    TEST_ASSERT_EQUAL_UINT8(1u, f[1]);    // Mode::Car
    TEST_ASSERT_EQUAL_UINT8(0x03u, f[2]); // bit1=TSMS, bit0=DASH_CHG
    TEST_ASSERT_EQUAL_UINT8(1u, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0x12u, f[4]); // BE high byte
    TEST_ASSERT_EQUAL_UINT8(0x34u, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0u, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0u, f[7]);
}

extern "C" void test_pit_diag_fsm_status_pec_saturates(void) {
    const auto f = ams::pit_diag::encode_fsm_status(0u, ams::fsm::Mode::Undecided,
                                                    false, false, 0u,
                                                    0xFFFFFFFFu);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[5]);
}

// ---------------------------------------------------------------------------
// Timing frame.
// ---------------------------------------------------------------------------
extern "C" void test_pit_diag_timing_layout(void) {
    const auto f = ams::pit_diag::encode_timing(0x1234u, 0xABCDu, 0xDEADBEEFu);
    TEST_ASSERT_EQUAL_UINT8(0x12, f[0]);  // poll BE high
    TEST_ASSERT_EQUAL_UINT8(0x34, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0xAB, f[2]);  // max BE high
    TEST_ASSERT_EQUAL_UINT8(0xCD, f[3]);
    // sweep mask LE
    TEST_ASSERT_EQUAL_UINT8(0xEF, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xBE, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0xAD, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0xDE, f[7]);
}

extern "C" void test_pit_diag_timing_clips_poll_at_ffff(void) {
    const auto f = ams::pit_diag::encode_timing(0x10000u, 0xFFFFu, 0u);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f[3]);
}

// ---------------------------------------------------------------------------
// Command classifier.
// ---------------------------------------------------------------------------
extern "C" void test_pit_diag_classify_enable_returns_plus_one(void) {
    auto f = make_pit_diag_cmd(ams::config::PitDiagEnableMagic);
    TEST_ASSERT_EQUAL_INT(1, ams::pit_diag::classify_command(f));
}

extern "C" void test_pit_diag_classify_disable_returns_minus_one(void) {
    auto f = make_pit_diag_cmd(ams::config::PitDiagDisableMagic);
    TEST_ASSERT_EQUAL_INT(-1, ams::pit_diag::classify_command(f));
}

extern "C" void test_pit_diag_classify_random_payload_returns_zero(void) {
    const std::uint8_t junk[4] = { 0x12, 0x34, 0x56, 0x78 };
    auto f = make_pit_diag_cmd(junk);
    TEST_ASSERT_EQUAL_INT(0, ams::pit_diag::classify_command(f));
}

extern "C" void test_pit_diag_classify_wrong_id_returns_zero(void) {
    auto f = make_pit_diag_cmd(ams::config::PitDiagEnableMagic);
    f.id = 0x100;  // anything but PitDiagCmdRxId
    TEST_ASSERT_EQUAL_INT(0, ams::pit_diag::classify_command(f));
}

extern "C" void test_pit_diag_classify_wrong_dlc_returns_zero(void) {
    auto f = make_pit_diag_cmd(ams::config::PitDiagEnableMagic);
    f.dlc = 3;  // anything but PitDiagCmdDlc (=4)
    TEST_ASSERT_EQUAL_INT(0, ams::pit_diag::classify_command(f));
}
