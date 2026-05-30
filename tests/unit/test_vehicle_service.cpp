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
