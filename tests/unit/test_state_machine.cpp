// SPDX-License-Identifier: proprietary
//
// Tests for ams::fsm::step. Pure-logic transitions over the 6-state
// FSM. Inputs are constructed manually; no mutex, no HAL.

#include "ams_config.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

ams::fsm::Inputs make_inputs(ams::fsm::State current,
                             ams::BmsState& bms,
                             ams::CurrentState& cur,
                             ams::VehicleState& veh) {
    std::memset(&bms, 0, sizeof(bms));
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&veh, 0, sizeof(veh));
    bms.module_online_mask = ams::config::kAllModulesMask;
    for (auto& t : bms.last_rx_tick) t = 9900;
    bms.min_cell_mV     = 3700;
    bms.max_cell_mV     = 3800;
    bms.min_tempC       = 25;
    bms.max_tempC       = 30;
    bms.pack_voltage_mV = 5 * 19 * 3750;  // ~356 V
    cur.last_update_tick = 9950;
    cur.filtered_mA      = 1000;
    veh.last_dc_bus_tick = 9950;
    veh.dc_bus_V         = 350;
    return { current, bms, cur, veh,
             /*force*/false,
             /*now*/10000, /*entry*/9800 };
}

}  // namespace

extern "C" void test_fsm_start_waits_without_button(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

extern "C" void test_fsm_start_transitions_to_precharge_on_button(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    veh.start_button = 1;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kClosePrecharge);
}

extern "C" void test_fsm_start_transitions_to_charge_on_charger(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    cur.charger_detected = true;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, out.next);
}

extern "C" void test_fsm_precharge_reaches_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    veh.dc_bus_V = 340;  // > 0.95 * 356
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenPrecharge);
}

extern "C" void test_fsm_precharge_stays_below_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    veh.dc_bus_V = 100;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

extern "C" void test_fsm_transition_holds_then_runs(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.state_entry_tick = 9950;  // < 100ms ago
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, ams::fsm::step(in).next);
    in.state_entry_tick = 9800;  // >= 100ms ago
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_run_and_charge_are_terminal(void) {
    // Run and Charge are mutually exclusive contexts (pack-in-car vs
    // pack-in-charger) entered once per power cycle. The FSM must
    // NOT transition between them at runtime, even if charger_detected
    // wiggles (likely an electrical anomaly if it does).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;

    auto in_run = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    cur.charger_detected = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in_run).next);

    auto in_chg = make_inputs(ams::fsm::State::Charge, bms, cur, veh);
    cur.charger_detected = false;
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, ams::fsm::step(in_chg).next);
}

extern "C" void test_fsm_any_state_to_error_on_fault(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    // Trip via force_error_set since the SDC predicate was retired
    // in #18 (no GPIO-sensed SDC on the v1.2 daughterboard).
    in.force_error_set = true;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirP);
}

extern "C" void test_fsm_error_is_sticky(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Error, bms, cur, veh);
    // even with everything healthy
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, ams::fsm::step(in).next);
}

extern "C" void test_fsm_precharge_timeout_forces_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    veh.dc_bus_V = 50;  // never reaches target
    in.state_entry_tick = in.now_tick - (ams::config::kPrechargeMaxMs + 1);
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
}

extern "C" void test_fsm_transition_drops_voltage_to_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    // Started Transition; voltage now dropped well below target.
    veh.dc_bus_V = 100;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirP);
}
