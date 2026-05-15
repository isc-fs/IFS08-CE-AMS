// SPDX-License-Identifier: proprietary
//
// Multi-step scenario tests ("SIL"): drive the pure FSM through
// realistic input sequences and assert the resulting state trajectory.
// These complement the single-step unit tests by catching ordering /
// timing bugs that only show up across multiple step() calls.
//
// All inputs are constructed manually -- no FreeRTOS, no HAL. The
// helper Harness keeps the BMS / current / vehicle state and a
// monotonically advancing `now` tick.

#include "ams_config.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

struct Harness {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    // Start past kSafetyBootGraceMs (2000) so the safety predicates
    // are active. Scenarios that need the grace itself live in
    // test_safety_predicates.cpp.
    std::uint32_t     now              = 3000;
    std::uint32_t     state_entry_tick = 3000;
    ams::fsm::State   state            = ams::fsm::State::Start;

    Harness() {
        bms.module_online_mask = ams::config::kAllModulesMask;
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
        const ams::fsm::Inputs in = {
            state, bms, cur, veh,
            /*force_error_set=*/false,
            now, state_entry_tick,
        };
        const auto out = ams::fsm::step(in);
        if (out.next != state) {
            state            = out.next;
            state_entry_tick = now;
        }
        return out;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Scenario 1: nominal startup.
// Start -> press button -> Precharge -> bus reaches target -> Transition
// -> hold elapses -> Run.
// ---------------------------------------------------------------------------
extern "C" void test_sil_nominal_startup_to_run(void) {
    Harness h;

    // Tick 0: idle in Start, no button.
    TEST_ASSERT_EQUAL(ams::fsm::State::Start, h.step().next);

    // Press start button.
    h.veh.start_button = 1;
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

// ---------------------------------------------------------------------------
// Scenario 2: precharge times out -> Error + open-all relays.
// ---------------------------------------------------------------------------
extern "C" void test_sil_precharge_timeout_to_error(void) {
    Harness h;
    h.veh.start_button = 1;
    h.step();                                 // -> Precharge
    TEST_ASSERT_EQUAL(ams::fsm::State::Precharge, h.state);

    // Sit in Precharge with no DC bus rise for > 1500 ms.
    h.veh.dc_bus_V = 20;
    h.advance(ams::config::kPrechargeMaxMs + 100);
    const auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kForceError);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenAirP);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kOpenPrecharge);
}

// ---------------------------------------------------------------------------
// Scenario 3: BMS module drops out mid-Run -> Error.
// ---------------------------------------------------------------------------
extern "C" void test_sil_bms_dropout_in_run(void) {
    Harness h;
    h.state = ams::fsm::State::Run;
    h.advance(20);

    // Pretend module 2 stopped reporting > kBmsStaleMs ago.
    h.bms.last_rx_tick[2] = h.now - ams::config::kBmsStaleMs - 100;

    const auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Error, out.next);
}

// ---------------------------------------------------------------------------
// Scenario 4: charger plugs in from Start -> straight to Charge.
// ---------------------------------------------------------------------------
extern "C" void test_sil_charger_path(void) {
    Harness h;
    h.cur.charger_detected = true;
    const auto out = h.step();
    TEST_ASSERT_EQUAL(ams::fsm::State::Charge, out.next);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirN);
    TEST_ASSERT_TRUE(out.safety_flags & ams::events::safety::kCloseAirP);
}

// ---------------------------------------------------------------------------
// (Former scenario 5 -- "SDC opens during Run" -- removed in #18 along
// with the sdc_closed predicate. The AMS no longer GPIO-senses the SDC
// line; equivalent "sticky Error" coverage lives in the kForceError +
// BMS-stale paths above.)
// ---------------------------------------------------------------------------
