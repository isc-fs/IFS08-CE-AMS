// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::acu_tx::* encoders. Byte-by-byte fixtures
// against the table in docs/CAN_MAP.md (ECU TX matrix, fix/53).

#include "acu_tx_encoders.hpp"
#include "ams_config.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

using namespace ams::acu_tx;

namespace {

ams::BmsState make_bms() {
    ams::BmsState b{};
    // Distinct per-module values so byte-layout regressions show up
    // immediately.
    b.min_cell_mV       = 3700;
    b.vmin_module[0]    = 3701;
    b.vmin_module[1]    = 3702;
    b.vmin_module[2]    = 3703;
    b.vmin_module[3]    = 3704;
    b.vmin_module[4]    = 3705;
    b.vmax_module[0]    = 3801;
    b.vmax_module[1]    = 3802;
    b.vmax_module[2]    = 3803;
    b.vmax_module[3]    = 3804;
    b.vmax_module[4]    = 3805;
    b.tmax_module[0]    = 30;
    b.tmax_module[1]    = 31;
    b.tmax_module[2]    = 32;
    b.tmax_module[3]    = 33;
    b.tmax_module[4]    = 34;
    return b;
}

ams::CurrentState make_cur(std::int32_t pack_mA, std::int32_t dcdc_mA) {
    ams::CurrentState c{};
    c.filtered_mA      = pack_mA;
    c.dcdc_filtered_mA = dcdc_mA;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// 0x020 ok_precharge
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_ok_precharge_high_in_run(void) {
    const auto f = encode_ok_precharge(/*Run=*/3);
    TEST_ASSERT_EQUAL_UINT8(1, f[0]);
}
extern "C" void test_acu_tx_ok_precharge_high_in_charge(void) {
    const auto f = encode_ok_precharge(/*Charge=*/4);
    TEST_ASSERT_EQUAL_UINT8(1, f[0]);
}
extern "C" void test_acu_tx_ok_precharge_low_in_start(void) {
    const auto f = encode_ok_precharge(/*Start=*/0);
    TEST_ASSERT_EQUAL_UINT8(0, f[0]);
}
extern "C" void test_acu_tx_ok_precharge_low_in_precharge_and_transition(void) {
    TEST_ASSERT_EQUAL_UINT8(0, encode_ok_precharge(/*Precharge=*/1)[0]);
    TEST_ASSERT_EQUAL_UINT8(0, encode_ok_precharge(/*Transition=*/2)[0]);
}
extern "C" void test_acu_tx_ok_precharge_low_in_error(void) {
    TEST_ASSERT_EQUAL_UINT8(0, encode_ok_precharge(/*Error=*/5)[0]);
}

// ---------------------------------------------------------------------------
// 0x12C v_cell_min — BE u16 of bms.min_cell_mV
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_min_voltage_big_endian(void) {
    auto b = make_bms();
    b.min_cell_mV = 0x1234;
    const auto f = encode_min_voltage(b);
    TEST_ASSERT_EQUAL_UINT8(0x12, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, f[1]);
}

// ---------------------------------------------------------------------------
// 0x131 vmin modules 0..2, 0x132 vmin modules 3..4
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_vmin_module_a_layout(void) {
    const auto b = make_bms();
    const auto f = encode_vmin_module_a(b);
    // 3701 -> 0x0E75, 3702 -> 0x0E76, 3703 -> 0x0E77, BE
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[0]); TEST_ASSERT_EQUAL_UINT8(0x75, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[2]); TEST_ASSERT_EQUAL_UINT8(0x76, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[4]); TEST_ASSERT_EQUAL_UINT8(0x77, f[5]);
}
extern "C" void test_acu_tx_vmin_module_b_layout(void) {
    const auto b = make_bms();
    const auto f = encode_vmin_module_b(b);
    // 3704 -> 0x0E78, 3705 -> 0x0E79
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[0]); TEST_ASSERT_EQUAL_UINT8(0x78, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[2]); TEST_ASSERT_EQUAL_UINT8(0x79, f[3]);
}

// ---------------------------------------------------------------------------
// 0x133 vmax modules 0..2, 0x134 vmax modules 3..4
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_vmax_module_a_layout(void) {
    const auto b = make_bms();
    const auto f = encode_vmax_module_a(b);
    // 3801 -> 0x0ED9, 3802 -> 0x0EDA, 3803 -> 0x0EDB
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[0]); TEST_ASSERT_EQUAL_UINT8(0xD9, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[2]); TEST_ASSERT_EQUAL_UINT8(0xDA, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[4]); TEST_ASSERT_EQUAL_UINT8(0xDB, f[5]);
}
extern "C" void test_acu_tx_vmax_module_b_layout(void) {
    const auto b = make_bms();
    const auto f = encode_vmax_module_b(b);
    // 3804 -> 0x0EDC, 3805 -> 0x0EDD
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[0]); TEST_ASSERT_EQUAL_UINT8(0xDC, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, f[2]); TEST_ASSERT_EQUAL_UINT8(0xDD, f[3]);
}

// ---------------------------------------------------------------------------
// 0x135 currents — BE i16 deciamps [accu | dcdc]
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_currents_zero(void) {
    const auto f = encode_currents(make_cur(0, 0));
    TEST_ASSERT_EACH_EQUAL_UINT8(0, f.data(), f.size());
}
extern "C" void test_acu_tx_currents_positive_pack(void) {
    // 5000 mA -> 50 dA -> 0x0032
    const auto f = encode_currents(make_cur(5000, 0));
    TEST_ASSERT_EQUAL_UINT8(0x00, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x32, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[3]);
}
extern "C" void test_acu_tx_currents_negative_pack_two_complement(void) {
    // -5000 mA -> -50 dA -> 0xFFCE
    const auto f = encode_currents(make_cur(-5000, 0));
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCE, f[1]);
}
extern "C" void test_acu_tx_currents_rounding_to_nearest_dA(void) {
    // 49 mA -> 0 dA (under half-dA boundary)
    TEST_ASSERT_EQUAL_INT16(0, mA_to_deciamps_i16(49));
    // 50 mA -> 1 dA (round up)
    TEST_ASSERT_EQUAL_INT16(1, mA_to_deciamps_i16(50));
    // -49 mA -> 0, -50 mA -> -1
    TEST_ASSERT_EQUAL_INT16( 0, mA_to_deciamps_i16(-49));
    TEST_ASSERT_EQUAL_INT16(-1, mA_to_deciamps_i16(-50));
}
extern "C" void test_acu_tx_currents_saturates_at_int16_extremes(void) {
    // 32_767 dA = 3_276_700 mA; anything beyond clips.
    TEST_ASSERT_EQUAL_INT16( 32767, mA_to_deciamps_i16( 5'000'000));
    TEST_ASSERT_EQUAL_INT16(-32768, mA_to_deciamps_i16(-5'000'000));
}
extern "C" void test_acu_tx_currents_dcdc_in_bytes_2_3(void) {
    // 12345 mA -> 123 dA -> 0x007B, in bytes 2..3
    const auto f = encode_currents(make_cur(0, 12345));
    TEST_ASSERT_EQUAL_UINT8(0x00, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0x7B, f[3]);
}

// ---------------------------------------------------------------------------
// 0x136 temp_max modules 0..2 + 0x137 temp_max modules 3..4 + temp_dcdc stub
// ---------------------------------------------------------------------------
extern "C" void test_acu_tx_tmax_module_a_layout(void) {
    const auto b = make_bms();
    const auto f = encode_tmax_module_a(b);
    // 30 -> 0x001E, 31 -> 0x001F, 32 -> 0x0020 (BE i16)
    TEST_ASSERT_EQUAL_UINT8(0x00, f[0]); TEST_ASSERT_EQUAL_UINT8(0x1E, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[2]); TEST_ASSERT_EQUAL_UINT8(0x1F, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[4]); TEST_ASSERT_EQUAL_UINT8(0x20, f[5]);
}
extern "C" void test_acu_tx_tmax_module_b_includes_dcdc_stub(void) {
    const auto b = make_bms();
    const auto f = encode_tmax_module_b(b);
    // 33 -> 0x0021, 34 -> 0x0022, then temp_dcdc stub = INT16_MIN = 0x8000
    TEST_ASSERT_EQUAL_UINT8(0x00, f[0]); TEST_ASSERT_EQUAL_UINT8(0x21, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[2]); TEST_ASSERT_EQUAL_UINT8(0x22, f[3]);
    TEST_ASSERT_EQUAL_UINT8(0x80, f[4]); TEST_ASSERT_EQUAL_UINT8(0x00, f[5]);
}
extern "C" void test_acu_tx_tmax_handles_negative_degC(void) {
    auto b = make_bms();
    b.tmax_module[0] = -10;   // 0xFFF6 BE
    const auto f = encode_tmax_module_a(b);
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0xF6, f[1]);
}
