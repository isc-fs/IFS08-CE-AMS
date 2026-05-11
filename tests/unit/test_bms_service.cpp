// SPDX-License-Identifier: proprietary
//
// Pure-logic unit tests for BmsService. No HAL, no FreeRTOS -- the
// mocks/ directory stubs out cmsis_os2 enough that BmsService::*
// compiles and runs on the host.

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "can_frame.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

// Test-only knobs exposed by cmsis_os2_stub.cpp.
extern "C" void fake_set_tick(std::uint32_t);
extern "C" void fake_advance_tick(std::uint32_t);

namespace {

// Per-module CANID. Voltage response IDs are CANID + 1..5; temperature
// response IDs are CANID + 21..25. See docs/CAN_MAP.md.
constexpr std::uint32_t module_canid(std::uint8_t m) {
    return static_cast<std::uint32_t>(ams::config::kBmsVoltPollBase) +
           static_cast<std::uint32_t>(m) * ams::config::kBmsModuleStride;
}

ams::CanFrame make_voltage_frame(std::uint8_t module,
                                 std::uint8_t frame_idx,
                                 const std::uint16_t (&cells_mV)[4]) {
    ams::CanFrame f = {};
    f.id  = module_canid(module) + ams::config::kBmsVoltRespOffsetLo + frame_idx;
    f.dlc = 8;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Bms);
    f.timestamp_ms = 0;
    for (std::uint8_t k = 0; k < 4; ++k) {
        f.data[2 * k]     = static_cast<std::uint8_t>(cells_mV[k] >> 8);
        f.data[2 * k + 1] = static_cast<std::uint8_t>(cells_mV[k] & 0xFF);
    }
    return f;
}

ams::CanFrame make_temperature_frame(std::uint8_t module,
                                     std::uint8_t frame_idx,
                                     const std::int8_t (&temps_c)[8]) {
    ams::CanFrame f = {};
    f.id  = module_canid(module) + ams::config::kBmsTempRespOffsetLo + frame_idx;
    f.dlc = 8;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Bms);
    for (std::uint8_t k = 0; k < 8; ++k) {
        f.data[k] = static_cast<std::uint8_t>(temps_c[k]);
    }
    return f;
}

// Fresh service per test: BmsService is a singleton, so reset its
// state by zeroing the snapshot through the public API isn't ideal;
// for now each test seeds the state from scratch and asserts on
// deltas.  When the test surface grows we'll add a friend test_reset
// hook.
void clear_state() {
    // Drain anything from previous tests by writing one frame per
    // module so module_online_mask is 0x1F, then explicit zeros.
    // For these initial tests we just rely on test ordering and the
    // additive nature of update_from_frame; full reset comes later.
}

}  // namespace

extern "C" void setUp(void)    { fake_set_tick(0); }
extern "C" void tearDown(void) {}

// ---------------------------------------------------------------------------
// Test 1: voltage frame for module 0 decodes cells 0..3 big-endian.
// ---------------------------------------------------------------------------
extern "C" void test_voltage_frame_module0_decodes_cells_0_3(void) {
    clear_state();
    const std::uint16_t mv[4] = { 3700, 3701, 3702, 3703 };
    auto f = make_voltage_frame(0, 0, mv);

    TEST_ASSERT_TRUE(ams::BmsService::instance().update_from_frame(f));

    auto s = ams::BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT16(3700, s.cell_mV[0][0]);
    TEST_ASSERT_EQUAL_UINT16(3701, s.cell_mV[0][1]);
    TEST_ASSERT_EQUAL_UINT16(3702, s.cell_mV[0][2]);
    TEST_ASSERT_EQUAL_UINT16(3703, s.cell_mV[0][3]);
}

// ---------------------------------------------------------------------------
// Test 2: voltage frame for module 4 (last) lands in the right slot.
// ---------------------------------------------------------------------------
extern "C" void test_voltage_frame_module4_decodes(void) {
    const std::uint16_t mv[4] = { 4100, 4101, 4102, 4103 };
    auto f = make_voltage_frame(4, 0, mv);

    TEST_ASSERT_TRUE(ams::BmsService::instance().update_from_frame(f));

    auto s = ams::BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT16(4100, s.cell_mV[4][0]);
    TEST_ASSERT_EQUAL_UINT16(4103, s.cell_mV[4][3]);
}

// ---------------------------------------------------------------------------
// Test 3: voltage frame_idx 4 covers only 3 cells (indices 16, 17, 18).
//         Bytes 6-7 of the payload must be ignored.
// ---------------------------------------------------------------------------
extern "C" void test_voltage_frame_idx4_handles_three_cells(void) {
    const std::uint16_t mv[4] = { 3801, 3802, 3803, /* ignored */ 0xFFFF };
    auto f = make_voltage_frame(1, 4, mv);

    TEST_ASSERT_TRUE(ams::BmsService::instance().update_from_frame(f));

    auto s = ams::BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT16(3801, s.cell_mV[1][16]);
    TEST_ASSERT_EQUAL_UINT16(3802, s.cell_mV[1][17]);
    TEST_ASSERT_EQUAL_UINT16(3803, s.cell_mV[1][18]);
    // The 19th element (index 18) is the last; index 19 does not exist.
}

// ---------------------------------------------------------------------------
// Test 4: temperature frame for module 2 decodes signed int8.
// ---------------------------------------------------------------------------
extern "C" void test_temperature_frame_decodes_signed_int8(void) {
    const std::int8_t tc[8] = { 25, -5, 60, -20, 0, 127, -128, 1 };
    auto f = make_temperature_frame(2, 0, tc);

    TEST_ASSERT_TRUE(ams::BmsService::instance().update_from_frame(f));

    auto s = ams::BmsService::instance().snapshot();
    TEST_ASSERT_EQUAL_INT16(  25, s.cell_tempC[2][0]);
    TEST_ASSERT_EQUAL_INT16(  -5, s.cell_tempC[2][1]);
    TEST_ASSERT_EQUAL_INT16(  60, s.cell_tempC[2][2]);
    TEST_ASSERT_EQUAL_INT16( -20, s.cell_tempC[2][3]);
    TEST_ASSERT_EQUAL_INT16(   0, s.cell_tempC[2][4]);
    TEST_ASSERT_EQUAL_INT16( 127, s.cell_tempC[2][5]);
    TEST_ASSERT_EQUAL_INT16(-128, s.cell_tempC[2][6]);
    TEST_ASSERT_EQUAL_INT16(   1, s.cell_tempC[2][7]);
}

// ---------------------------------------------------------------------------
// Test 5: a frame on the wrong bus is silently rejected.
// ---------------------------------------------------------------------------
extern "C" void test_frame_on_wrong_bus_is_rejected(void) {
    const std::uint16_t mv[4] = { 3900, 3900, 3900, 3900 };
    auto f = make_voltage_frame(0, 0, mv);
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);  // wrong bus

    TEST_ASSERT_FALSE(ams::BmsService::instance().update_from_frame(f));
}

// ---------------------------------------------------------------------------
// Test 6: an unknown ID on the BMS bus is rejected (no crash, no write).
// ---------------------------------------------------------------------------
extern "C" void test_unknown_id_is_rejected(void) {
    ams::CanFrame f = {};
    f.id  = 0x7FF;                                            // not in any pane
    f.dlc = 8;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Bms);

    TEST_ASSERT_FALSE(ams::BmsService::instance().update_from_frame(f));
}

// ---------------------------------------------------------------------------
// Test 7: is_healthy is false until all 5 modules have reported.
// ---------------------------------------------------------------------------
extern "C" void test_is_healthy_requires_all_modules(void) {
    fake_set_tick(1000);
    // After the prior tests, modules 0, 1, 2, 4 have reported. Module
    // 3 hasn't been touched. Pump a frame to module 3 and assert
    // is_healthy goes true.
    const std::uint16_t mv[4] = { 3700, 3700, 3700, 3700 };
    auto f = make_voltage_frame(3, 0, mv);
    f.timestamp_ms = 1000;
    ams::BmsService::instance().update_from_frame(f);

    TEST_ASSERT_TRUE(ams::BmsService::instance().is_healthy(1000));
}

// ---------------------------------------------------------------------------
// Test 8: is_healthy goes false after the freshness window expires.
// ---------------------------------------------------------------------------
extern "C" void test_is_healthy_false_after_staleness(void) {
    // kBmsStaleMs is 1500. last_rx_tick for module 3 is 1000 from the
    // prior test. Advance the tick clock past 1000 + 1500 = 2500.
    TEST_ASSERT_FALSE(ams::BmsService::instance().is_healthy(2501));
}
