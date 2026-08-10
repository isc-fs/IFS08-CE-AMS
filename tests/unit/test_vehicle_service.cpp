// SPDX-License-Identifier: proprietary
//
// Tests for VehicleService -- decode helpers plus update_from_frame
// dispatch on the accumulator-bus IDs.

#include "ams_config.hpp"
#include "can_frame.hpp"
#include "vehicle_service.hpp"

#include "unity.h"

#include <cstdint>

namespace {

ams::CanFrame make_acu_frame(std::uint32_t id, std::uint8_t dlc,
                             const std::uint8_t (&data)[8]) {
    ams::CanFrame f = {};
    f.id  = id;
    f.dlc = dlc;
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Acu);
    for (std::uint8_t k = 0; k < 8; ++k) f.data[k] = data[k];
    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// decode_dc_bus_V: legacy is little-endian, byte 1 is high byte.
// ---------------------------------------------------------------------------
extern "C" void test_decode_dc_bus_little_endian(void) {
    const std::uint8_t data[2] = { 0x2C, 0x01 };  // 0x012C = 300 V
    TEST_ASSERT_EQUAL_UINT16(300, ams::VehicleService::decode_dc_bus_V(data));
}

// ---------------------------------------------------------------------------
// update_from_frame: 0x100 frame updates dc_bus_V and timestamp.
// ---------------------------------------------------------------------------
extern "C" void test_update_dc_bus_frame(void) {
    const std::uint8_t data[8] = { 0xF4, 0x01, 0, 0, 0, 0, 0, 0 };  // 0x01F4 = 500 V
    auto f = make_acu_frame(ams::config::AcuRxDcBusId, 8, data);
    f.timestamp_ms = 1234;

    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));

    auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT16(500, s.dc_bus_V);
    TEST_ASSERT_EQUAL_UINT32(1234, s.last_dc_bus_tick);
}

// ---------------------------------------------------------------------------
// 0x100 byte 2: the two bits, and the opposite directions they default in.
// ---------------------------------------------------------------------------
extern "C" void test_dc_bus_byte2_bits_decode(void) {
    // bit 0 = discharge_engaged, bit 1 = dc_bus_valid.
    const std::uint8_t both[8] = { 0xF4, 0x01, 0x03, 0, 0, 0, 0, 0 };
    auto f = make_acu_frame(ams::config::AcuRxDcBusId, 3, both);
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_TRUE(s.discharge_engaged);
    TEST_ASSERT_TRUE(s.dc_bus_valid);
    TEST_ASSERT_TRUE(s.ecu_discharge_capable);

    // Valid measurement, bleed disconnected -- the normal running case.
    const std::uint8_t valid_only[8] = { 0xF4, 0x01, 0x02, 0, 0, 0, 0, 0 };
    f = make_acu_frame(ams::config::AcuRxDcBusId, 3, valid_only);
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_FALSE(s.discharge_engaged);
    TEST_ASSERT_TRUE(s.dc_bus_valid);

    // Byte 2 clear: the ECU is present and reporting that it has no measurement.
    // Distinct from the byte being absent, below.
    const std::uint8_t neither[8] = { 0x00, 0x00, 0x00, 0, 0, 0, 0, 0 };
    f = make_acu_frame(ams::config::AcuRxDcBusId, 3, neither);
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_FALSE(s.discharge_engaged);
    TEST_ASSERT_FALSE(s.dc_bus_valid);
}

// An ECU that predates byte 2 sends DLC 2. Both fields fall back to "this ECU's
// silence changes nothing" -- which is FALSE for the bleed and TRUE for the
// voltage, because each default has to be the one that does not alter behaviour.
extern "C" void test_dc_bus_dlc2_defaults_are_asymmetric(void) {
    // Land in the opposite state first, so the DLC-2 frame has to overwrite both.
    const std::uint8_t armed[8] = { 0xF4, 0x01, 0x01, 0, 0, 0, 0, 0 };
    auto f = make_acu_frame(ams::config::AcuRxDcBusId, 3, armed);
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_TRUE(s.discharge_engaged);
    TEST_ASSERT_FALSE(s.dc_bus_valid);

    const std::uint8_t legacy[8] = { 0xF4, 0x01, 0xFF, 0, 0, 0, 0, 0 };
    f = make_acu_frame(ams::config::AcuRxDcBusId, 2, legacy);
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_FALSE(s.discharge_engaged);   // no report -> assume disconnected
    TEST_ASSERT_TRUE(s.dc_bus_valid);         // no report -> assume usable
    // Byte 2 was present once, so the capability latch stays set: a mid-session
    // downgrade is not modelled.
    TEST_ASSERT_TRUE(s.ecu_discharge_capable);
}

// ---------------------------------------------------------------------------
// update_from_frame: wrong-bus frame is rejected.
// ---------------------------------------------------------------------------
extern "C" void test_acu_frame_on_wrong_bus_rejected(void) {
    const std::uint8_t data[8] = { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 };
    auto f = make_acu_frame(ams::config::AcuRxDcBusId, 8, data);
    f.bus = static_cast<std::uint8_t>(ams::CanBus::Bms);  // wrong bus

    TEST_ASSERT_FALSE(ams::VehicleService::instance().update_from_frame(f));
}

// ---------------------------------------------------------------------------
// update_from_frame: unknown id on the ACU bus is rejected.
// ---------------------------------------------------------------------------
extern "C" void test_acu_unknown_id_rejected(void) {
    const std::uint8_t data[8] = { 0 };
    auto f = make_acu_frame(0x7FE, 8, data);

    TEST_ASSERT_FALSE(ams::VehicleService::instance().update_from_frame(f));
}

// ---------------------------------------------------------------------------
// #305: operator charge-mode request. Magic-gated; advances the freshness
// tick. A wrong-magic / wrong-id frame must not.
// ---------------------------------------------------------------------------
extern "C" void test_charge_req_magic_frame_sets_tick(void) {
    const std::uint8_t data[8] = { 0x43, 0x48, 0x52, 0x47, 0, 0, 0, 0 };  // "CHRG"
    auto f = make_acu_frame(ams::config::ChargeModeReqId,
                            ams::config::ChargeModeReqDlc, data);
    f.timestamp_ms = 4242;
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    TEST_ASSERT_EQUAL_UINT32(
        4242, ams::VehicleService::instance().snapshot().last_charge_req_tick);
}

extern "C" void test_charge_req_wrong_magic_ignored(void) {
    // Seed a known-good tick, then a bad-magic frame must NOT advance it.
    const std::uint8_t good[8] = { 0x43, 0x48, 0x52, 0x47, 0, 0, 0, 0 };
    auto g = make_acu_frame(ams::config::ChargeModeReqId,
                            ams::config::ChargeModeReqDlc, good);
    g.timestamp_ms = 1000;
    (void)ams::VehicleService::instance().update_from_frame(g);

    const std::uint8_t bad[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0 };
    auto b = make_acu_frame(ams::config::ChargeModeReqId,
                            ams::config::ChargeModeReqDlc, bad);
    b.timestamp_ms = 2000;
    TEST_ASSERT_FALSE(ams::VehicleService::instance().update_from_frame(b));
    TEST_ASSERT_EQUAL_UINT32(
        1000, ams::VehicleService::instance().snapshot().last_charge_req_tick);
}

// charge_requested: fresh / stale / never / future-tick.
extern "C" void test_charge_requested_freshness(void) {
    TEST_ASSERT_FALSE(ams::VehicleService::charge_requested(10000, 0u));        // never
    TEST_ASSERT_TRUE (ams::VehicleService::charge_requested(
        10000, 10000 - ams::config::ChargeReqFreshMs));                         // exactly fresh
    TEST_ASSERT_FALSE(ams::VehicleService::charge_requested(
        10000, 10000 - ams::config::ChargeReqFreshMs - 1));                     // just stale
    TEST_ASSERT_TRUE (ams::VehicleService::charge_requested(10000, 10005));     // future -> fresh
}

// ---------------------------------------------------------------------------
// #336: operator balance master switch (0x103). "BALO"->Off, "BALN"->On,
// "BALX"->Auto, each stamps the tick; an unrecognised payload is ignored.
// ---------------------------------------------------------------------------
extern "C" void test_balance_override_balo_off(void) {
    const std::uint8_t balo[8] = { 0x42, 0x41, 0x4C, 0x4F, 0, 0, 0, 0 };  // "BALO"
    auto f = make_acu_frame(ams::config::BalanceOverrideReqId,
                            ams::config::BalanceOverrideReqDlc, balo);
    f.timestamp_ms = 7000;
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    const auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(ams::config::BalanceCmd::Off),
                            static_cast<std::uint8_t>(s.balance_cmd));
    TEST_ASSERT_EQUAL_UINT32(7000, s.last_balance_override_tick);
}

extern "C" void test_balance_override_baln_on(void) {
    const std::uint8_t baln[8] = { 0x42, 0x41, 0x4C, 0x4E, 0, 0, 0, 0 };  // "BALN"
    auto f = make_acu_frame(ams::config::BalanceOverrideReqId,
                            ams::config::BalanceOverrideReqDlc, baln);
    f.timestamp_ms = 7500;
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    const auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(ams::config::BalanceCmd::On),
                            static_cast<std::uint8_t>(s.balance_cmd));
    TEST_ASSERT_EQUAL_UINT32(7500, s.last_balance_override_tick);
}

extern "C" void test_balance_override_balx_auto(void) {
    const std::uint8_t balx[8] = { 0x42, 0x41, 0x4C, 0x58, 0, 0, 0, 0 };  // "BALX"
    auto f = make_acu_frame(ams::config::BalanceOverrideReqId,
                            ams::config::BalanceOverrideReqDlc, balx);
    f.timestamp_ms = 8000;
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    const auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(ams::config::BalanceCmd::Auto),
                            static_cast<std::uint8_t>(s.balance_cmd));
    TEST_ASSERT_EQUAL_UINT32(8000, s.last_balance_override_tick);
}

extern "C" void test_balance_override_wrong_magic_ignored(void) {
    const std::uint8_t bad[8] = { 0x42, 0x41, 0x4C, 0x5A, 0, 0, 0, 0 };  // "BALZ"
    auto f = make_acu_frame(ams::config::BalanceOverrideReqId,
                            ams::config::BalanceOverrideReqDlc, bad);
    TEST_ASSERT_FALSE(ams::VehicleService::instance().update_from_frame(f));
}

// effective_balance_cmd: fresh -> the command as-is; stale / never -> Off.
extern "C" void test_balance_effective_cmd_freshness(void) {
    using ams::config::BalanceCmd;
    const auto eff = ams::VehicleService::effective_balance_cmd;
    // Fresh On / Auto pass through.
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BalanceCmd::On),
        static_cast<std::uint8_t>(eff(10000, 10000 - ams::config::BalanceOverrideFreshMs,
                                      BalanceCmd::On)));                       // exactly fresh
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BalanceCmd::Auto),
        static_cast<std::uint8_t>(eff(10000, 10005, BalanceCmd::Auto)));      // future -> fresh
    // Dead-man: just-stale and never-seen both fall back to Off.
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BalanceCmd::Off),
        static_cast<std::uint8_t>(eff(10000, 10000 - ams::config::BalanceOverrideFreshMs - 1,
                                      BalanceCmd::On)));                       // just stale -> Off
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BalanceCmd::Off),
        static_cast<std::uint8_t>(eff(10000, 0u, BalanceCmd::On)));           // never -> Off
}

// ---------------------------------------------------------------------------
// Per-module balancing enable (0x104): magic "BALM" + byte 4 = 5-bit mask.
// ---------------------------------------------------------------------------
extern "C" void test_balance_modules_decode(void) {
    const std::uint8_t m[8] = { 0x42, 0x41, 0x4C, 0x4D, 0x15, 0, 0, 0 };  // "BALM" + 0b10101 (mods 0,2,4)
    auto f = make_acu_frame(ams::config::BalanceModulesReqId,
                            ams::config::BalanceModulesReqDlc, m);
    f.timestamp_ms = 9000;
    TEST_ASSERT_TRUE(ams::VehicleService::instance().update_from_frame(f));
    const auto s = ams::VehicleService::instance().snapshot();
    TEST_ASSERT_EQUAL_UINT8(0x15, s.balance_modules_mask);
    TEST_ASSERT_EQUAL_UINT32(9000, s.last_balance_modules_tick);
}

extern "C" void test_balance_modules_wrong_magic_ignored(void) {
    const std::uint8_t bad[8] = { 0x42, 0x41, 0x4C, 0x5A, 0x1F, 0, 0, 0 };  // "BALZ"
    auto f = make_acu_frame(ams::config::BalanceModulesReqId,
                            ams::config::BalanceModulesReqDlc, bad);
    TEST_ASSERT_FALSE(ams::VehicleService::instance().update_from_frame(f));
}

// effective_balance_modules_mask: fresh -> the mask; stale / never -> all enabled.
extern "C" void test_balance_modules_effective_freshness(void) {
    const auto eff = ams::VehicleService::effective_balance_modules_mask;
    TEST_ASSERT_EQUAL_UINT8(0x15,  // exactly fresh -> passes through
        eff(10000, 10000 - ams::config::BalanceModulesFreshMs, 0x15));
    TEST_ASSERT_EQUAL_UINT8(ams::config::BalanceModulesDefaultMask,  // just stale -> all enabled
        eff(10000, 10000 - ams::config::BalanceModulesFreshMs - 1, 0x15));
    TEST_ASSERT_EQUAL_UINT8(ams::config::BalanceModulesDefaultMask,  // never seen -> all enabled
        eff(10000, 0u, 0x00));
}
