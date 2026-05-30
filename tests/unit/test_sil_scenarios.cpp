// SPDX-License-Identifier: proprietary
//
// Multi-step scenario tests ("SIL"): drive the pure FSM through
// realistic input sequences and assert the resulting state trajectory.
// These complement the single-step unit tests by catching ordering /
// timing bugs that only show up across multiple step() calls.
//
// All inputs are constructed manually -- no FreeRTOS, no HAL. The
// helper Harness keeps the BMS / current / vehicle state, TSMS / DASH_CHG
// pin readback, locked-mode, and a monotonically advancing `now` tick.

#include "ams_config.hpp"
#include "safety_predicates.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

struct Harness {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    bool              tsms             = false;
    // DASH_CHG is a momentary press (#305): set dash_chg_edge=true for the
    // step where the operator presses; step() consumes it (one-shot).
    bool              dash_chg_edge    = false;
    ams::fsm::Mode    mode_locked      = ams::fsm::Mode::Undecided;
    // Start past SafetyBootGraceMs (2000) so the safety predicates
    // are active. Scenarios that need the grace itself live in
    // test_safety_predicates.cpp.
    std::uint32_t     now              = 3000;
    std::uint32_t     state_entry_tick = 3000;
    ams::fsm::State   state            = ams::fsm::State::Start;

    Harness() {
        bms.module_online_mask = ams::config::AllModulesMask;
        for (auto& t : bms.last_rx_tick) t = now;
        bms.min_cell_mV     = 3700;
        bms.max_cell_mV     = 3800;
        bms.min_tempC       = 25;
        bms.max_tempC       = 30;
        bms.pack_voltage_mV = 5u * 19u * 3750u;  // ~356 V
        cur.last_update_tick = now;
        cur.filtered_mA      = 0;
        veh.last_dc_bus_tick = now;
        veh.dc_bus_V         = 0;
    }

    // Advance the clock + reapply 'last_*_tick' so freshness checks
    // keep passing (in real firmware, services would write these
    // themselves on every fresh frame / ADC sample).
    void advance(std::uint32_t dt_ms) {
        now += dt_ms;
        for (auto& t : bms.last_rx_tick) t = now;
        cur.last_update_tick = now;
        veh.last_dc_bus_tick = now;
    }

    ams::fsm::Output step() {
        // Replicate SafetyTask: evaluate the predicate set, then feed
        // the decision into the FSM (which no longer self-evaluates,
        // #279). No debounce here -- SIL exercises FSM transition logic,
        // not the cell-fault latch timing (that's unit-tested directly
        // via CellFaultDebounce).
        const ams::safety::Inputs pred = {
            bms, cur, veh, /*force_error_set=*/false,
            /*vcu_required=*/(mode_locked == ams::fsm::Mode::Car), now,
        };
        const bool predicate_fault = ams::safety::evaluate_fault(pred);
        const ams::fsm::Inputs in = {
            state, bms, cur, veh,
            tsms, dash_chg_edge, mode_locked,
            predicate_fault,
            now, state_entry_tick,
        };
        const auto out = ams::fsm::step(in);
        dash_chg_edge = false;   // one-shot press, consumed by the step
        if (out.next != state) {
            state            = out.next;
            state_entry_tick = now;
        }
        return out;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Scenario 1: nominal startup in CAR mode.
// Start -> assert TSMS+DASH_CHG (+ lock Mode::Car) -> Precharge ->
// bus reaches target -> Transition -> hold elapses -> Run.
// ---------------------------------------------------------------------------
extern "C" void test_sil_nominal_startup_to_run(void) {
    Harness h;

    // Tick 0: idle in Start, neither TSMS nor DASH_CHG asserted.
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, h.step().next);

    // Assert TSMS + DASH_CHG and lock car mode (VCU heartbeat fresh per
    // Harness ctor).
    h.tsms        = true;
    h.dash_chg_edge = true;
    h.mode_locked = ams::fsm::Mode::Car;
    h.advance(20);
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, h.step().next);

    // DC bus ramps up over ~500 ms; meanwhile precharge holds.
    for (int i = 0; i < 24; ++i) {
        h.advance(20);
        h.veh.dc_bus_V = static_cast<std::uint16_t>((i + 1) * 15);
        const auto out = h.step();
        if (h.veh.dc_bus_V >= 340) {
            TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
            break;
        }
        TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    }
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, h.state);

    // Stay in Transition for the hold window. Voltage holds.
    h.veh.dc_bus_V = 350;
    for (int i = 0; i < 4; ++i) { h.advance(20); h.step(); }
    h.advance(40);  // total now > 100 ms in Transition
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, h.step().next);
}

// (Scenario 2 -- precharge_timeout_to_error -- removed; FSM no longer
//  has a Precharge deadline. Stuck contactors / low pack-V are caught
//  via the safety predicate freshness checks instead.)

// ---------------------------------------------------------------------------
// Scenario 3: BMS module drops out mid-Run -> Error.
// ---------------------------------------------------------------------------
extern "C" void test_sil_bms_dropout_in_run(void) {
    Harness h;
    h.state = ams::fsm::State::Run;
    h.tsms = true;
    h.mode_locked = ams::fsm::Mode::Car;
    h.advance(20);

    // Pretend module 2 stopped reporting > BmsStaleMs ago.
    h.bms.last_rx_tick[2] = h.now - ams::config::BmsStaleMs - 100;

    const auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
}

// ---------------------------------------------------------------------------
// Scenario 4: charger path -- ONE DASH_CHG press (#305). The charger auto-
// emits 0x101 the moment it is connected, so a still-fresh charge request is
// the "charger up, proceed" signal: the single press enters Precharge, then
// 0x101 freshness closes AIR+ -> Transition -> Charge. If 0x101 goes stale
// the precharge holds (then times out) instead of proceeding.
// ---------------------------------------------------------------------------
extern "C" void test_sil_charger_path(void) {
    Harness h;
    h.tsms = true;
    h.mode_locked = ams::fsm::Mode::Charger;   // SafetyTask would lock this
    h.veh.last_charge_req_tick = h.now;        // charger's auto 0x101, fresh

    // Single press: Start -> Precharge.
    h.dash_chg_edge = true;
    auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ClosePrecharge);

    // 0x101 stays fresh (charger still connected) -> proceed next step.
    h.advance(20);
    h.veh.last_charge_req_tick = h.now;
    out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Transition, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::CloseAirP);

    // Transition is a one-step passthrough -> Charge (not Run).
    h.advance(20);
    h.veh.last_charge_req_tick = h.now;
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, h.step().next);
}

// ---------------------------------------------------------------------------
// Scenario 4b: charger connected, single press enters Precharge, then 0x101
// goes STALE (charger unplugged) before the proceed -> precharge holds, then
// the PrechargeMaxMs timeout latches Error. Closing AIR+ into a disconnected
// charger is exactly what the freshness gate prevents (#305).
// ---------------------------------------------------------------------------
extern "C" void test_sil_charger_stale_request_times_out(void) {
    Harness h;
    h.tsms = true;
    h.mode_locked = ams::fsm::Mode::Charger;   // SafetyTask locked this earlier
    // 0x101 already stale at the press (charger disconnected right after the
    // mode lock). advance() never re-stamps last_charge_req_tick, so it stays
    // stale -> the charger proceed gate never fires.
    h.veh.last_charge_req_tick = h.now - ams::config::ChargeReqFreshMs - 1;

    h.dash_chg_edge = true;
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, h.step().next);

    // Hold (0x101 stale) until > PrechargeMaxMs -> Error.
    const std::uint32_t deadline = h.now + ams::config::PrechargeMaxMs;
    ams::fsm::State last = ams::fsm::State::Precharge;
    while (h.now <= deadline + 40) {
        h.advance(20);
        last = h.step().next;
        if (last == ams::fsm::State::Error) break;
    }
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, last);
}

// ---------------------------------------------------------------------------
// Scenario 5: TSMS drops mid-Run -> Error (sticky, every AIR-open latches).
// ---------------------------------------------------------------------------
extern "C" void test_sil_tsms_drop_in_run_latches_error(void) {
    Harness h;
    h.state = ams::fsm::State::Run;
    h.tsms = true;
    h.mode_locked = ams::fsm::Mode::Car;
    h.advance(20);
    TEST_ASSERT_EQUAL(ams::fsm::State::Run, h.step().next);

    // Driver turns the key off mid-run.
    h.tsms = false;
    const auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::ForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::OpenAirP);
}
