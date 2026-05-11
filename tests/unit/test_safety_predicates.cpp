// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for safety::evaluate_fault. Constructs Inputs with
// known service-snapshot values and asserts the predicate boolean.

#include "ams_config.hpp"
#include "safety_predicates.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

// Build a fully-healthy input set keyed off a single now_tick.
ams::safety::Inputs make_nominal(ams::BmsState& bms,
                                 ams::CurrentState& cur,
                                 ams::VehicleState& veh,
                                 std::uint32_t now) {
    std::memset(&bms, 0, sizeof(bms));
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&veh, 0, sizeof(veh));

    bms.module_online_mask = ams::config::kAllModulesMask;
    for (std::uint8_t m = 0; m < ams::config::kBmsModuleCount; ++m) {
        bms.last_rx_tick[m] = now - 100;
    }
    bms.min_cell_mV = 3700;
    bms.max_cell_mV = 3800;
    bms.min_tempC   =  20;
    bms.max_tempC   =  35;

    cur.last_update_tick = now - 50;
    cur.filtered_mA      = 5000;       // 5 A discharge, well under limit
    cur.sensor_fault     = false;

    veh.last_dc_bus_tick = now - 50;

    return { bms, cur, veh,
             /*sdc_closed=*/true,
             /*force_error_set=*/false,
             now };
}

}  // namespace

extern "C" void test_predicates_nominal_no_fault(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);

    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_force_error(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    in.force_error_set = true;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_sdc_open(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    in.sdc_closed = false;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_undervoltage(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.min_cell_mV = ams::config::kCellUVmV - 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_overvoltage(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.max_cell_mV = ams::config::kCellOVmV + 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_overtemp(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.max_tempC = ams::config::kCellOTC + 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_bms_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.last_rx_tick[2] = 10000 - ams::config::kBmsStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_current_overlimit(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    cur.filtered_mA = ams::config::kImaxMa + 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_current_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    cur.last_update_tick = 10000 - ams::config::kIStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_vcu_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    veh.last_dc_bus_tick = 10000 - ams::config::kVcuStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}
