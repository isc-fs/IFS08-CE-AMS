// SPDX-License-Identifier: proprietary
//
// Phase-2a parity tests for the code-first CAN DSL.
//
// For each ported message, asserts the DSL-generated encoder produces
// byte-for-byte identical output to the existing hand-rolled encoder in
// Core/Inc/app/*_encoders.hpp, plus an encode/decode roundtrip and a
// descriptor-table sanity check. While both paths coexist these tests
// are the contract that lets us switch the firmware call sites over and
// retire the hand-rolled encoders without changing a single wire byte.

#include "bms_service.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"
#include "telemetry_encoders.hpp"
#include "can/can_codecs.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

// ===========================================================================
// 0x4A0  AMS_status
// ===========================================================================
extern "C" void test_dsl_status_matches_handrolled(void) {
    ams::BmsState bms{};
    std::memset(&bms, 0, sizeof(bms));
    const std::uint8_t state = 3, ams_ok = 1;
    bms.module_online_mask = 0x1F;
    bms.min_cell_mV = 3412;
    bms.max_cell_mV = 4127;

    const auto legacy = ams::telemetry::encode_status(state, ams_ok, bms);

    ifs08::AMS_status_t in{};
    in.fsm_state          = state;
    in.ams_ok             = ams_ok;
    in.module_online_mask = bms.module_online_mask;
    in.reserved_b3        = 0;
    in.min_cell_mV        = bms.min_cell_mV;
    in.max_cell_mV        = bms.max_cell_mV;

    std::uint8_t dsl[8] = {0};
    ifs08::encode_AMS_status(in, dsl);
    for (unsigned i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_HEX8(legacy[i], dsl[i]);
}

extern "C" void test_dsl_status_roundtrip(void) {
    ifs08::AMS_status_t a{};
    a.fsm_state = 5; a.ams_ok = 0; a.module_online_mask = 0x0A;
    a.reserved_b3 = 0; a.min_cell_mV = 2800; a.max_cell_mV = 4200;
    std::uint8_t buf[8] = {0};
    ifs08::encode_AMS_status(a, buf);
    ifs08::AMS_status_t b{};
    ifs08::decode_AMS_status(buf, b);
    TEST_ASSERT_EQUAL_UINT8 (a.fsm_state,          b.fsm_state);
    TEST_ASSERT_EQUAL_UINT8 (a.module_online_mask, b.module_online_mask);
    TEST_ASSERT_EQUAL_UINT16(a.min_cell_mV,        b.min_cell_mV);
    TEST_ASSERT_EQUAL_UINT16(a.max_cell_mV,        b.max_cell_mV);
}

// ===========================================================================
// 0x4A1  AMS_pack  (32-bit LE unsigned voltage + 32-bit LE SIGNED current)
// ===========================================================================
extern "C" void test_dsl_pack_matches_handrolled_discharge(void) {
    ams::BmsState bms{};      std::memset(&bms, 0, sizeof(bms));
    ams::CurrentState cur{};  std::memset(&cur, 0, sizeof(cur));
    bms.pack_voltage_mV = 356250;   // 5*19*3750
    cur.filtered_mA     = 123456;   // discharge (+)

    const auto legacy = ams::telemetry::encode_pack(bms, cur);

    ifs08::AMS_pack_t in{};
    in.pack_voltage_mV = bms.pack_voltage_mV;
    in.filtered_mA     = cur.filtered_mA;
    std::uint8_t dsl[8] = {0};
    ifs08::encode_AMS_pack(in, dsl);
    for (unsigned i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_HEX8(legacy[i], dsl[i]);
}

extern "C" void test_dsl_pack_matches_handrolled_charge(void) {
    // Negative current exercises the signed-LE path (two's complement).
    ams::BmsState bms{};      std::memset(&bms, 0, sizeof(bms));
    ams::CurrentState cur{};  std::memset(&cur, 0, sizeof(cur));
    bms.pack_voltage_mV = 300000;
    cur.filtered_mA     = -98765;   // charge (-)

    const auto legacy = ams::telemetry::encode_pack(bms, cur);

    ifs08::AMS_pack_t in{};
    in.pack_voltage_mV = bms.pack_voltage_mV;
    in.filtered_mA     = cur.filtered_mA;
    std::uint8_t dsl[8] = {0};
    ifs08::encode_AMS_pack(in, dsl);
    for (unsigned i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_HEX8(legacy[i], dsl[i]);
}

extern "C" void test_dsl_pack_roundtrip_signed(void) {
    ifs08::AMS_pack_t a{};
    a.pack_voltage_mV = 0xDEADBEEFu;
    a.filtered_mA     = -200000;
    std::uint8_t buf[8] = {0};
    ifs08::encode_AMS_pack(a, buf);
    ifs08::AMS_pack_t b{};
    ifs08::decode_AMS_pack(buf, b);
    TEST_ASSERT_EQUAL_UINT32(a.pack_voltage_mV, b.pack_voltage_mV);
    TEST_ASSERT_EQUAL_INT32 (a.filtered_mA,     b.filtered_mA);
}

// ===========================================================================
// 0x4A2  AMS_temps  (signed int8 temps + LE u16 + cockpit/diag bytes)
// ===========================================================================
extern "C" void test_dsl_temps_matches_handrolled(void) {
    ams::BmsState bms{};      std::memset(&bms, 0, sizeof(bms));
    ams::VehicleState veh{};  std::memset(&veh, 0, sizeof(veh));
    bms.min_tempC = -12; bms.max_tempC = 47; bms.avg_tempC = 25;  // in-range int8
    veh.dc_bus_V  = 412;
    const std::uint8_t hb = 0x5A, txf = 0x03, cockpit = 0x86;  // sentinel+Car+TSMS

    const auto legacy = ams::telemetry::encode_temps(bms, veh, hb, txf, cockpit);

    ifs08::AMS_temps_t in{};
    in.min_tempC          = ams::telemetry::clip_int8(bms.min_tempC);
    in.max_tempC          = ams::telemetry::clip_int8(bms.max_tempC);
    in.avg_tempC          = ams::telemetry::clip_int8(bms.avg_tempC);
    in.dc_bus_V           = veh.dc_bus_V;
    in.tsms_dash_chg_byte = cockpit;
    in.tx_fail_count_lo   = txf;
    in.heartbeat          = hb;
    std::uint8_t dsl[8] = {0};
    ifs08::encode_AMS_temps(in, dsl);
    for (unsigned i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_HEX8(legacy[i], dsl[i]);
}

extern "C" void test_dsl_temps_roundtrip_negative(void) {
    ifs08::AMS_temps_t a{};
    a.min_tempC = -40; a.max_tempC = -1; a.avg_tempC = -20;
    a.dc_bus_V = 350; a.tsms_dash_chg_byte = 0x80; a.tx_fail_count_lo = 0; a.heartbeat = 200;
    std::uint8_t buf[8] = {0};
    ifs08::encode_AMS_temps(a, buf);
    ifs08::AMS_temps_t b{};
    ifs08::decode_AMS_temps(buf, b);
    TEST_ASSERT_EQUAL_INT8  (a.min_tempC, b.min_tempC);
    TEST_ASSERT_EQUAL_INT8  (a.max_tempC, b.max_tempC);
    TEST_ASSERT_EQUAL_INT8  (a.avg_tempC, b.avg_tempC);
    TEST_ASSERT_EQUAL_UINT16(a.dc_bus_V,  b.dc_bus_V);
    TEST_ASSERT_EQUAL_UINT8 (a.heartbeat, b.heartbeat);
}

// ===========================================================================
// Registry well-formedness
// ===========================================================================
extern "C" void test_dsl_registry_well_formed(void) {
    // Phase 2a: 3 telemetry. Phase 2b: +9 ECU TX matrix. Phase 2c: +9 fixed pit-diag + ack + 5 RX = 27.
    TEST_ASSERT_EQUAL_UINT(27u, ifs08::ALL_MSGS_COUNT);
    // Spot-check the BE field's DBC start_bit convention (8*byte+7).
    bool checked = false;
    for (unsigned i = 0; i < ifs08::ALL_MSGS_COUNT; ++i) {
        const auto& m = ifs08::ALL_MSGS[i];
        if (m.id == 0x4A0u) {
            TEST_ASSERT_EQUAL_STRING("AMS_status", m.name);
            TEST_ASSERT_EQUAL_UINT(500u, m.period_ms);
            for (unsigned k = 0; k < m.n_fields; ++k) {
                if (std::strcmp(m.fields[k].name, "min_cell_mV") == 0) {
                    TEST_ASSERT_TRUE(m.fields[k].big_endian);
                    TEST_ASSERT_EQUAL_UINT(39u, m.fields[k].start_bit);  // 8*4+7
                    checked = true;
                }
            }
        }
    }
    TEST_ASSERT_TRUE(checked);
}
