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
    // The memset above wipes the struct's default, so restate it: a nominal
    // vehicle has an ECU reporting a real inverter measurement. Tests that care
    // clear it explicitly.
    veh.dc_bus_valid     = true;
    return { current, bms, cur, veh,
             /*tsms*/false, /*dash_chg_edge*/false,
             /*mode_locked*/ams::fsm::Mode::Undecided,
             /*predicate_fault*/false,
             /*bus_collapsed*/false,
             /*dc_bus_fresh*/true,
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

// A stale dc_bus_V frozen at pack voltage must NOT satisfy the precharge-complete
// criterion. VehicleState holds the last received value, so a VCU that stops
// publishing leaves the number sitting at whatever it last was -- and frozen high
// it would close AIR+ onto a link that has since bled to zero. VcuStale catches
// the dead VCU only after 200 ms and only once mode is locked to Car, while the
// FSM steps every 20 ms, so it loses that race by an order of magnitude.
extern "C" void test_fsm_precharge_rejects_stale_bus_reading(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.dc_bus_V = 350;              // clears the 95 % bar on its face

    // Fresh -> proceeds, closing AIR+.
    in.dc_bus_fresh = true;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirP);

    // Same reading, now stale -> holds in Precharge, AIR+ stays open. It will
    // time out at PrechargeMaxMs into Error, which is the correct outcome.
    in.dc_bus_fresh = false;
    out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::CloseAirP);
}

// Transition re-checks the same criterion before committing to Run, so a reading
// that goes stale between the two steps must not be trusted there either.
extern "C" void test_fsm_transition_rejects_stale_bus_reading(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Transition, bms, cur, veh);
    in.tsms = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.dc_bus_V = 350;
    in.dc_bus_fresh = false;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
}

// Charger does not consume dc_bus_V at all -- it proceeds on 0x101 freshness,
// because the VCU is absent by definition during a charge. Staleness here must
// not block it.
extern "C" void test_fsm_charger_precharge_unaffected_by_stale_bus(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true;
    in.mode_locked = ams::fsm::Mode::Charger;
    veh.last_charge_req_tick = 9950;   // fresh 0x101
    veh.dc_bus_V = 0;
    in.dc_bus_fresh = false;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirP);
}

// ---------------------------------------------------------------------------
// Re-arm gate. Opening the shutdown circuit de-energises the discharge relay
// (NC) so the bleed connects; closing it again re-energises the relay and the
// discharge STOPS part-way. What is left on the link is then not something the
// AMS can infer, so it waits for the ECU's report instead.
// ---------------------------------------------------------------------------
extern "C" void test_fsm_start_blocks_while_bleed_connected(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.ecu_discharge_capable = true;
    veh.dc_bus_V = 0;                      // link is down...
    veh.discharge_engaged = true;          // ...but the bleed is still connected

    // Hard interlock: closing a contactor now would put pack current through a
    // resistor rated for transient duty. Blocked regardless of the voltage.
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, out.next);
    TEST_ASSERT_EQUAL_UINT32(0u, out.safety_flags);

    veh.discharge_engaged = false;
    out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ClosePrecharge);
}

extern "C" void test_fsm_start_blocks_while_link_charged(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.ecu_discharge_capable = true;
    veh.discharge_engaged = false;

    // Above threshold -> a precharge here would be a no-op, since the 95 %
    // completion criterion is already satisfied on entry.
    veh.dc_bus_V = static_cast<std::uint16_t>(ams::config::DcBusDischargedV + 1u);
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);

    // At the threshold -> arms.
    veh.dc_bus_V = ams::config::DcBusDischargedV;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);

    // Stale 0x100 is not a discharged link.
    in.dc_bus_fresh = false;
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);
}

// The failure freshness alone cannot see. The ECU emits 0x100 every cycle
// whatever the inverter is doing, so the frame stays fresh; when it has no
// measurement it substitutes 0 V. Read literally that is "the link is drained"
// -- a permit to arm over a link that may be sitting at pack voltage. The bit is
// what stops the substituted value being mistaken for a reading.
extern "C" void test_fsm_start_blocks_on_unmeasured_link(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.ecu_discharge_capable = true;
    veh.discharge_engaged = false;
    veh.dc_bus_V = 0;                      // the ECU's substituted value...
    veh.dc_bus_valid = false;              // ...announced as not a measurement
    TEST_ASSERT_TRUE(in.dc_bus_fresh);     // and the frame itself is on time

    TEST_ASSERT_EQUAL(ams::fsm::State::Start, ams::fsm::step(in).next);

    // Same voltage, now backed by a real reading -> arms.
    veh.dc_bus_valid = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// An unmeasured link must not complete a precharge either. The ECU substitutes
// 0 V, which already fails the 95 % test, so this asserts the property rather
// than the arithmetic: were the substitution ever to change to a held value,
// the frozen-high case is exactly PRECHARGE-1 again.
extern "C" void test_fsm_precharge_holds_on_unmeasured_link(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Precharge, bms, cur, veh);
    in.tsms = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.dc_bus_V = 350;                    // would satisfy 95 % of a 356 V pack
    veh.dc_bus_valid = false;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);

    veh.dc_bus_valid = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, ams::fsm::step(in).next);
}

// Charger never sees this bit: dc_bus_V is ECU-only and absent during a charge,
// so gating on it would make Charger unarmable.
extern "C" void test_fsm_charger_arms_with_unmeasured_link(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Charger;
    veh.ecu_discharge_capable = true;
    veh.discharge_engaged = false;
    veh.dc_bus_valid = false;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// An ECU sending DLC 2 cannot express the bit at all. vehicle_service defaults
// it true for exactly that case, and the re-arm block is gated on
// ecu_discharge_capable besides -- so an older ECU still arms.
extern "C" void test_fsm_older_ecu_arms_with_default_validity(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.ecu_discharge_capable = false;     // never seen DLC >= 3
    veh.discharge_engaged = false;
    veh.dc_bus_valid = true;               // what update_from_frame leaves at DLC 2
    veh.dc_bus_V = 350;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// An ECU that predates the discharge protocol sends 0x100 with DLC 2, so it can
// neither report the bleed state nor drain a stranded link. Enforcing the
// voltage block against it would brick the car rather than protect it.
extern "C" void test_fsm_start_not_blocked_by_older_ecu(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Car;
    veh.ecu_discharge_capable = false;     // never seen DLC >= 3
    veh.discharge_engaged = false;
    veh.dc_bus_V = 350;                    // charged, but nothing can clear it
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, ams::fsm::step(in).next);
}

// Charger is exempt: the inverter is not in the charge loop and dc_bus_V is
// VCU-only, absent during a charge, so gating it would make Charger unarmable.
extern "C" void test_fsm_charger_arms_regardless_of_discharge_state(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_inputs(ams::fsm::State::Start, bms, cur, veh);
    in.tsms = true;
    in.dash_chg_edge = true;
    in.mode_locked = ams::fsm::Mode::Charger;
    veh.ecu_discharge_capable = true;
    veh.discharge_engaged = true;
    veh.dc_bus_V = 350;
    in.dc_bus_fresh = false;
    auto out = ams::fsm::step(in);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirN);
    TEST_ASSERT_FALSE(out.safety_flags & ams::events::safety::ClosePrecharge);
}
