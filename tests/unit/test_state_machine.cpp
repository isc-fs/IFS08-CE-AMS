// SPDX-License-Identifier: proprietary
//
// Tests for ams::fsm::step. Pure-logic transitions over the 6-state
// FSM. Inputs are constructed manually; no mutex, no HAL.
//
// Updated in fix/48 for the TSMS / DASH_CHG + mode_locked rewrite.
// The old 0x600 start_button / 0x18FF50E7 charger_detected triggers
// were retired; TSMS / DASH_CHG state lives directly in fsm::Inputs.

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
             /*tsms*/false, /*dash_chg*/false,
             /*mode_locked*/ams::fsm::Mode::Undecided,
             /*force*/false,
             /*now*/10000, /*entry*/9800 };
}

}  // namespace

// ---------------------------------------------------------------------------
// Start -> Precharge requires BOTH TSMS and DASH_CHG.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_start_waits_without_tsms_or_dash_chg(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    // tsms=false, dash_chg=false
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

extern "C" void test_fsm_start_waits_with_tsms_only(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

extern "C" void test_fsm_start_waits_with_dash_chg_only(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.dash_chg = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

extern "C" void test_fsm_start_to_precharge_on_both_inputs(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms    = true;
    in.dash_chg = true;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kClosePrecharge);
}

// ---------------------------------------------------------------------------
// Precharge: target reached / not reached / timeout.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_precharge_reaches_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    veh.dc_bus_V = 340;  // > 0.95 * 356
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenPrecharge);
}

extern "C" void test_fsm_precharge_stays_below_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    veh.dc_bus_V = 100;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

extern "C" void test_fsm_precharge_timeout_forces_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    veh.dc_bus_V = 50;  // never reaches target
    in.state_entry_tick = in.now_tick - (ams::config::kPrechargeMaxMs + 1);
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
}

// ---------------------------------------------------------------------------
// Transition: holds, branches on mode_locked, drops voltage to Error.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_transition_holds_then_runs_in_car_mode(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    in.mode_locked = ams::fsm::Mode::Car;

    in.state_entry_tick = 9950;  // < 100ms ago
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, ams::fsm::step(in).next);
    in.state_entry_tick = 9800;  // >= 100ms ago
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_transition_holds_then_charges_in_charger_mode(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    in.mode_locked = ams::fsm::Mode::Charger;

    in.state_entry_tick = 9800;
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, ams::fsm::step(in).next);
}

extern "C" void test_fsm_transition_undecided_mode_forces_error(void) {
    // Reaching Transition with mode_locked == Undecided would mean
    // SafetyTask failed to capture the mode at Start->Precharge.
    // The FSM treats this as a fault.
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    in.mode_locked = ams::fsm::Mode::Undecided;
    in.state_entry_tick = 9800;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
}

extern "C" void test_fsm_transition_drops_voltage_to_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    veh.dc_bus_V = 100;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirP);
}

// ---------------------------------------------------------------------------
// Run / Charge: TSMS or DASH_CHG drop latches Error; otherwise stays.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_run_stays_while_tsms_and_dash_chg_high(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
    in.mode_locked = ams::fsm::Mode::Car;
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_run_to_error_on_tsms_drop(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = false; in.dash_chg = true;
    in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirP);
}

extern "C" void test_fsm_run_to_error_on_dash_chg_drop(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg = false;
    in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
}

extern "C" void test_fsm_charge_to_error_on_tsms_or_dash_chg_drop(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Charge, bms, cur, veh);
    in.tsms = false; in.dash_chg = false;
    in.mode_locked = ams::fsm::Mode::Charger;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
}

// ---------------------------------------------------------------------------
// Error path: sticky; predicate fault routes to Error.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_any_state_to_error_on_fault(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg = true;
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
    in.tsms = true; in.dash_chg = true;  // even with both inputs high
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, ams::fsm::step(in).next);
}
