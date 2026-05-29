// SPDX-License-Identifier: proprietary
//
// Phase-1 prototype parity test for the code-first CAN DSL.
//
// Asserts that for the 0x4A0 AMS_status frame, the new generated
// encoder/decoder in Core/Inc/can/can_codecs.hpp produce byte-for-byte
// identical output to the existing hand-rolled
// ams::telemetry::encode_status() in Core/Inc/app/telemetry_encoders.hpp.
//
// If this passes, the DSL faithfully reproduces the current wire
// behaviour and the same pattern can be ported to the remaining
// messages. If it ever fails, the .def diverged from the hand-rolled
// encoder and the wire-contract change needs an explicit review.

#include "bms_service.hpp"
#include "telemetry_encoders.hpp"
#include "can/can_codecs.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

void fill_sample(ams::BmsState& bms,
                 uint8_t& state, uint8_t& ams_ok) {
    std::memset(&bms, 0, sizeof(bms));
    state                   = 3;       // Run
    ams_ok                  = 1;
    bms.module_online_mask  = 0x1F;    // all 5 modules online
    bms.min_cell_mV         = 3412;
    bms.max_cell_mV         = 4127;
}

}  // namespace

extern "C" void test_dsl_ams_status_matches_handrolled_encoder() {
    ams::BmsState bms{};
    uint8_t state = 0, ams_ok = 0;
    fill_sample(bms, state, ams_ok);

    // Path A: existing hand-rolled encoder.
    const auto frame_legacy = ams::telemetry::encode_status(state, ams_ok, bms);

    // Path B: code-first generated encoder.
    ifs08::AMS_status_t in{};
    in.fsm_state          = state;
    in.ams_ok             = ams_ok;
    in.module_online_mask = bms.module_online_mask;
    in.reserved_b3        = 0;
    in.min_cell_mV        = bms.min_cell_mV;
    in.max_cell_mV        = bms.max_cell_mV;

    uint8_t frame_dsl[8] = {0};
    ifs08::encode_AMS_status(in, frame_dsl);

    for (unsigned i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_HEX8(frame_legacy[i], frame_dsl[i]);
    }
}

extern "C" void test_dsl_ams_status_roundtrip() {
    // Generated decoder must invert the generated encoder.
    ifs08::AMS_status_t a{};
    a.fsm_state          = 5;
    a.ams_ok             = 0;
    a.module_online_mask = 0x0A;
    a.reserved_b3        = 0;
    a.min_cell_mV        = 2800;
    a.max_cell_mV        = 4200;

    uint8_t buf[8] = {0};
    ifs08::encode_AMS_status(a, buf);

    ifs08::AMS_status_t b{};
    ifs08::decode_AMS_status(buf, b);

    TEST_ASSERT_EQUAL_UINT8 (a.fsm_state,          b.fsm_state);
    TEST_ASSERT_EQUAL_UINT8 (a.ams_ok,             b.ams_ok);
    TEST_ASSERT_EQUAL_UINT8 (a.module_online_mask, b.module_online_mask);
    TEST_ASSERT_EQUAL_UINT8 (a.reserved_b3,        b.reserved_b3);
    TEST_ASSERT_EQUAL_UINT16(a.min_cell_mV,        b.min_cell_mV);
    TEST_ASSERT_EQUAL_UINT16(a.max_cell_mV,        b.max_cell_mV);
}

extern "C" void test_dsl_ams_status_descriptor_table_well_formed() {
    // Sanity check that the generated FieldDesc[] / MsgDesc match the
    // .def declaration. Catches bumps to the macro expansions that
    // forget to update one pass.
    TEST_ASSERT_EQUAL_UINT(1u, ifs08::ALL_MSGS_COUNT);
    const auto& m = ifs08::ALL_MSGS[0];
    TEST_ASSERT_EQUAL_STRING("AMS_status", m.name);
    TEST_ASSERT_EQUAL_HEX32(0x4A0u, m.id);
    TEST_ASSERT_EQUAL_UINT(8u, m.dlc);
    TEST_ASSERT_EQUAL_STRING("AMS", m.sender);
    TEST_ASSERT_EQUAL_UINT(500u, m.period_ms);
    TEST_ASSERT_EQUAL_UINT(6u, m.n_fields);

    // Spot-check a BE field's start_bit follows DBC convention (8*byte+7).
    bool found_min = false;
    for (unsigned i = 0; i < m.n_fields; ++i) {
        if (std::strcmp(m.fields[i].name, "min_cell_mV") == 0) {
            TEST_ASSERT_TRUE (m.fields[i].big_endian);
            TEST_ASSERT_EQUAL_UINT(39u, m.fields[i].start_bit);  // 8*4 + 7
            TEST_ASSERT_EQUAL_UINT(16u, m.fields[i].len);
            found_min = true;
        }
    }
    TEST_ASSERT_TRUE(found_min);
}
