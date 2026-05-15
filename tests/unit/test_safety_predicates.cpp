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

// ---------------------------------------------------------------------------
// Boot grace (regression guard for the bench-discovered IWDG reset loop
// caused by all-zero last_*_ticks at first SafetyTask iteration).
// ---------------------------------------------------------------------------

// All ticks zero + within the grace window: every freshness check
// would otherwise fault. Boot grace must suppress them.
extern "C" void test_predicates_boot_grace_suppresses_data_predicates(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    // Build manually -- can't reuse make_nominal because it pre-sets
    // last_*_ticks, and the bug is exactly about ticks at zero.
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*sdc_closed=*/true,
        /*force_error_set=*/false,
        /*now_tick=*/500u,  // half a second in -- well inside grace
    };
    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

// Immediate-safety predicates must keep firing during the grace.
extern "C" void test_predicates_boot_grace_does_not_suppress_force_error(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*sdc_closed=*/true,
        /*force_error_set=*/true,
        /*now_tick=*/100u,
    };
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_boot_grace_does_not_suppress_sdc_open(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*sdc_closed=*/false,
        /*force_error_set=*/false,
        /*now_tick=*/100u,
    };
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

// One tick past the grace window with no service activity -> fault.
// The unsigned `now - 0 = now` age naturally exceeds every staleness
// window for now_tick > grace > max(kBmsStaleMs, kIStaleMs, kVcuStaleMs).
extern "C" void test_predicates_after_grace_zero_ticks_faults(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*sdc_closed=*/true,
        /*force_error_set=*/false,
        /*now_tick=*/ams::config::kSafetyBootGraceMs + 1u,
    };
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}
