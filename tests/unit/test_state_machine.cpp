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
    bms.module_online_mask = ams::config::AllModulesMask;
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
             /*tsms*/false, /*dash_chg_edge*/false,
             /*mode_locked*/ams::fsm::Mode::Undecided,
             /*predicate_fault*/false,
             /*bus_collapsed*/false,
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
    in.dash_chg_edge = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

extern "C" void test_fsm_start_to_precharge_on_both_inputs(void) {
    // Car mode: classic resistor precharge (AIR- + precharge close).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms    = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ClosePrecharge);
}

// Charger mode SKIPS the precharge resistor: AIR- closes but the precharge
// contactor does NOT (the charger voltage-matches; AIR+ closes on the
// 0x101-fresh proceed). Keeps charge current out of the transient-rated
// resistor that sits in parallel with AIR+.
extern "C" void test_fsm_start_to_precharge_charger_skips_precharge(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms    = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Charger;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE (out.safety_flags & ams::events::safety::CloseAirN);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::ClosePrecharge);
}

// ---------------------------------------------------------------------------
// Precharge: target reached / not reached / timeout.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_precharge_reaches_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    veh.dc_bus_V = 340;  // > 0.95 * 356
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_precharge_stays_below_target(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    veh.dc_bus_V = 100;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// ---------------------------------------------------------------------------
// Transition: passthrough state -- commits to Run/Charge on first step
// based on mode_locked. Drops voltage to Error.
// (TransitionHoldMs stays removed -- Transition is one-step. The
// precharge deadline was re-added as PrechargeMaxMs, #302 follow-up.)
// ---------------------------------------------------------------------------
extern "C" void test_fsm_transition_commits_to_run_in_car_mode(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    // Bus still at target (precharge_target_reached==true with default
    // make_inputs values -- veh.dc_bus_V >= 95% of bms.pack_voltage_mV).
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_transition_commits_to_charge_in_charger_mode(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Charger;
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, ams::fsm::step(in).next);
}

extern "C" void test_fsm_transition_undecided_mode_forces_error(void) {
    // Reaching Transition with mode_locked == Undecided would mean
    // SafetyTask failed to capture the mode at Start->Precharge.
    // The FSM treats this as a fault.
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Undecided;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
}

extern "C" void test_fsm_transition_drops_voltage_to_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    veh.dc_bus_V = 100;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
}

// ---------------------------------------------------------------------------
// Run / Charge: a TSMS drop de-energises to Start WITHOUT latching (#327);
// DASH_CHG release is ignored; otherwise stays.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_run_stays_while_tsms_and_dash_chg_high(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_run_to_start_on_tsms_drop(void) {
    // A TSMS drop is a normal operator de-energise, NOT a fault: open all
    // contactors and fall back to Start, with NO ForceError / no latch, so
    // the TS can be re-armed from the cockpit without a reset (#327).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = false; in.dash_chg_edge = false;
    in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, out.next);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::ForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_run_stays_on_dash_chg_release(void) {
    // DASH_CHG is a momentary press -- low in Run is normal and must NOT
    // fault. Run is sustained by TSMS alone (#305).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = false;  // TSMS held, no press
    in.mode_locked = ams::fsm::Mode::Car;
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

extern "C" void test_fsm_charge_to_error_on_tsms_drop(void) {
    // Charger mode is the exception to the non-latching TSMS rule: the
    // scrutineering sheet forbids re-activating the charge output once the SDC
    // opens, so a TSMS drop in Charge LATCHES Error (+ ForceError, open all)
    // rather than falling back to Start. ChargerTsmsOpen.
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Charge, bms, cur, veh);
    in.tsms = false;
    in.mode_locked = ams::fsm::Mode::Charger;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_charger_precharge_to_error_on_tsms_drop(void) {
    // The charger latch covers ALL energised charger states: a TSMS drop
    // during charger Precharge latches Error too (contrast the Car-mode
    // Precharge case above, which is a non-latching return to Start).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = false;
    in.mode_locked = ams::fsm::Mode::Charger;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
}

extern "C" void test_fsm_precharge_to_start_on_tsms_drop(void) {
    // The non-latching TSMS guard covers every energised state, including
    // Precharge -- a TSMS drop mid-precharge opens up and returns to Start
    // rather than waiting for the PrechargeMaxMs timeout to latch (#327).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = false; in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, out.next);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::ForceError);
}

extern "C" void test_fsm_tsms_drop_still_yields_to_predicate_fault(void) {
    // A genuine pack fault takes priority over a TSMS drop: if both are
    // present, the predicate fault still latches Error (#327 guard sits
    // below the predicate-fault guard).
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = false; in.predicate_fault = true;
    in.mode_locked = ams::fsm::Mode::Car;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
}

extern "C" void test_fsm_charge_stays_on_dash_chg_release(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Charge, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = false;
    in.mode_locked = ams::fsm::Mode::Charger;
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, ams::fsm::step(in).next);
}

// ---------------------------------------------------------------------------
// #330: AIRs opened externally (cockpit SDC shutdown the AMS can't sense) ->
// Run de-energises to Start (non-latching) so a re-arm re-runs precharge.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_run_to_start_on_bus_collapse(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.mode_locked = ams::fsm::Mode::Car;
    in.bus_collapsed = true;   // SafetyTask-debounced collapse
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, out.next);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::ForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_run_stays_when_bus_healthy(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.mode_locked = ams::fsm::Mode::Car;
    in.bus_collapsed = false;
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, ams::fsm::step(in).next);
}

// bus_below_collapse threshold helper (BusCollapsePercent = 50).
extern "C" void test_bus_below_collapse_thresholds(void) {
    ams::BmsState bms{}; ams::VehicleState veh{};
    bms.pack_voltage_mV = 5u * 19u * 3750u;   // ~356 V cell-sum
    veh.dc_bus_V = 350;                        // ~full pack -> not collapsed
    TEST_ASSERT_FALSE(ams::fsm::bus_below_collapse(bms, veh));
    veh.dc_bus_V = 170;                        // < 50% of 356 -> collapsed
    TEST_ASSERT_TRUE(ams::fsm::bus_below_collapse(bms, veh));
    bms.pack_voltage_mV = 0; veh.dc_bus_V = 0;  // no data -> never collapsed
    TEST_ASSERT_FALSE(ams::fsm::bus_below_collapse(bms, veh));
}

// Charger precharge proceeds on a STILL-FRESH 0x101 charge request (#305) --
// the charger auto-emits it while connected -- not on dc_bus_V and not on a
// second DASH_CHG press. Without a fresh 0x101 it holds (-> PrechargeMaxMs).
extern "C" void test_fsm_precharge_charger_proceeds_on_fresh_request(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.mode_locked = ams::fsm::Mode::Charger;
    veh.dc_bus_V = 0;                        // no VCU during charge
    veh.last_charge_req_tick = in.now_tick;  // charger's auto 0x101, fresh
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_precharge_charger_holds_without_fresh_request(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.mode_locked = ams::fsm::Mode::Charger;
    veh.dc_bus_V = 0;
    // 0x101 stale (charger disconnected): older than ChargeReqFreshMs.
    veh.last_charge_req_tick =
        in.now_tick - ams::config::ChargeReqFreshMs - 1;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// ---------------------------------------------------------------------------
// Error path: sticky; predicate fault routes to Error.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_any_state_to_error_on_fault(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Run, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    in.predicate_fault = true;   // SafetyTask's debounced decision (#279)
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
}

extern "C" void test_fsm_error_is_sticky(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Error, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;  // even with both inputs high
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, ams::fsm::step(in).next);
}

// ---------------------------------------------------------------------------
// #302 follow-up: bounded precharge. A precharge that never reaches the
// target (e.g. dead VCU -> no dc_bus_V -> Charger-mode stuck) must latch
// Error at PrechargeMaxMs instead of holding the resistor closed forever.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_precharge_times_out_to_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    veh.dc_bus_V = 0;   // bus never confirmed (no VCU 0x100) -> never completes
    in.now_tick = in.state_entry_tick + ams::config::PrechargeMaxMs + 1;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenPrecharge);
}

extern "C" void test_fsm_precharge_holds_within_deadline(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true; in.dash_chg_edge = true;
    veh.dc_bus_V = 0;   // not reached yet, but still within the deadline
    in.now_tick = in.state_entry_tick + ams::config::PrechargeMaxMs - 1;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}
