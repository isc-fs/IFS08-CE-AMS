// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::balance::compute_mask -- the policy half
// of the passive balancing path (#74). No HAL, no FreeRTOS, no
// BmsService singleton; we build a BmsState in place and feed it in.

#include "ams_config.hpp"
#include "balance_controller.hpp"
#include "bms_service.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>
#include <limits>

namespace {

using namespace ams;

// Build a synthetic, fully-populated BmsState with every cell at
// base_mV and every NTC at base_C, plus the recomputed summaries
// (min_cell_mV, max_tempC) we rely on inside compute_mask.
BmsState make_uniform_state(std::uint16_t base_mV, std::int16_t base_C) {
    BmsState s{};
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            s.cell_mV[m][c] = base_mV;
        }
        for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
            s.cell_tempC[m][t] = base_C;
        }
    }
    s.min_cell_mV = base_mV;
    s.max_cell_mV = base_mV;
    s.min_tempC   = base_C;
    s.max_tempC   = base_C;
    // Every slot converted, so the thermal-data gate is satisfied.
    s.valid_temp_channels =
        static_cast<std::uint16_t>(config::BmsModuleCount * config::TempsPerModule);
    return s;
}

bool any_set(const balance::Mask& m) {
    for (std::uint8_t mm = 0; mm < config::BmsModuleCount; ++mm) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            if (m.cell[mm][c]) return true;
        }
    }
    return false;
}

std::uint8_t count_set_in_module(const balance::Mask& m, std::uint8_t module) {
    std::uint8_t n = 0;
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
        if (m.cell[module][c]) ++n;
    }
    return n;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Balanced pack in Charge: nothing to do, mask must be all zero.
// ---------------------------------------------------------------------------
extern "C" void test_balance_uniform_pack_no_discharge(void) {
    auto state = make_uniform_state(/* mV */ 4100, /* C */ 25);
    const auto mask = balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 2. One cell 80 mV above min in Charge -> that cell discharges.
//    Other cells stay quiet (delta below BalanceDeltaMv = 50 mV).
// ---------------------------------------------------------------------------
extern "C" void test_balance_single_hot_cell_in_charge(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[2][7] = static_cast<std::uint16_t>(4100 + 80);
    state.max_cell_mV  = state.cell_mV[2][7];

    const auto mask = balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto);
    TEST_ASSERT_TRUE(mask.cell[2][7]);
    // Only that one cell -- floor + 50 is the threshold, others sit
    // exactly at the floor so none should be selected.
    TEST_ASSERT_EQUAL_UINT8(1u, count_set_in_module(mask, 2));
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (m == 2) continue;
        TEST_ASSERT_EQUAL_UINT8(0u, count_set_in_module(mask, m));
    }
}

// ---------------------------------------------------------------------------
// 3. Six cells in one module above the threshold -- the controller
//    must cap to BalanceMaxActive (4) and pick the ones with the
//    largest excess.
// ---------------------------------------------------------------------------
extern "C" void test_balance_caps_at_max_active_per_module(void) {
    // Written relative to config::BalanceMaxActive so a change to the
    // dissipation budget does not silently invalidate the test. Put TWO more
    // cells over the threshold than the cap allows, with strictly increasing
    // excess, so the cap is genuinely exercised and the selection is ordered.
    constexpr std::uint8_t kCap  = config::BalanceMaxActive;
    constexpr std::uint8_t kOver = static_cast<std::uint8_t>(kCap + 2);
    static_assert(kOver <= config::CellsPerModule,
                  "test needs more cells per module than the cap + 2");

    auto state = make_uniform_state(4100, 25);
    for (std::uint8_t c = 0; c < kOver; ++c) {
        state.cell_mV[1][c] = static_cast<std::uint16_t>(4100 + 60 + 10 * c);
    }
    state.max_cell_mV = state.cell_mV[1][kOver - 1];

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true,
                                            config::BalanceCmd::Auto);

    // Exactly the cap discharges, never more -- this is the board dissipation
    // limit, so an off-by-one here is watts on a real board.
    TEST_ASSERT_EQUAL_UINT8(kCap, count_set_in_module(mask, 1));

    // ...and they are the HIGHEST cells: the top kCap by excess are set, the
    // two lowest over-threshold cells are not.
    for (std::uint8_t c = static_cast<std::uint8_t>(kOver - kCap); c < kOver; ++c) {
        TEST_ASSERT_TRUE_MESSAGE(mask.cell[1][c], "a top-excess cell was not selected");
    }
    TEST_ASSERT_FALSE_MESSAGE(mask.cell[1][0], "lowest over-threshold cell must not balance");
    TEST_ASSERT_FALSE_MESSAGE(mask.cell[1][1], "2nd-lowest over-threshold cell must not balance");
}

// ---------------------------------------------------------------------------
// 4. FSM in Run with an imbalanced pack -> mask is all zero. Passive
//    balancing only happens during Charge (issue acceptance bullet).
// ---------------------------------------------------------------------------
extern "C" void test_balance_disabled_outside_charge(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV = 4200;

    for (auto st : { fsm::State::Start, fsm::State::Precharge,
                     fsm::State::Transition, fsm::State::Run,
                     fsm::State::Error }) {
        const auto mask = balance::compute_mask(state, st, /*temps_trusted=*/true, config::BalanceCmd::Auto);
        TEST_ASSERT_FALSE_MESSAGE(any_set(mask),
            "mask must be empty outside Charge");
    }
}

// ---------------------------------------------------------------------------
// #336: operator OFF command forces all-zero even when the policy would
// discharge a hot cell in Charge. AUTO leaves autonomous behaviour unchanged.
// ---------------------------------------------------------------------------
extern "C" void test_balance_op_off_forces_no_discharge(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[2][7] = static_cast<std::uint16_t>(4100 + 80);  // would discharge
    state.max_cell_mV   = state.cell_mV[2][7];

    // Sanity: under AUTO that cell discharges in Charge.
    TEST_ASSERT_TRUE(
        balance::compute_mask(state, fsm::State::Charge,
                              /*temps_trusted=*/true, config::BalanceCmd::Auto).cell[2][7]);
    // Operator OFF: nothing discharges.
    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::Off);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// #336: operator ON forces balancing in ANY state (overrides the Charge-only
// default), while AUTO stays quiet outside Charge -- same imbalanced pack.
// ---------------------------------------------------------------------------
extern "C" void test_balance_op_on_runs_outside_charge(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;

    for (auto st : { fsm::State::Start, fsm::State::Precharge,
                     fsm::State::Transition, fsm::State::Run, fsm::State::Error }) {
        // AUTO: quiet outside Charge.
        TEST_ASSERT_FALSE_MESSAGE(
            any_set(balance::compute_mask(state, st, /*temps_trusted=*/true,
                                          config::BalanceCmd::Auto)),
            "AUTO must stay quiet outside Charge");
        // ON: the imbalanced cell discharges regardless of state.
        TEST_ASSERT_TRUE_MESSAGE(
            balance::compute_mask(state, st, /*temps_trusted=*/true,
                                  config::BalanceCmd::On).cell[0][0],
            "ON must balance in any state");
    }
}

// ON still honours the temp-trust guard: untrusted temps -> no discharge even
// with an explicit operator ON.
extern "C" void test_balance_op_on_respects_temp_trust(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;
    const auto mask = balance::compute_mask(state, fsm::State::Run,
                                            /*temps_trusted=*/false, config::BalanceCmd::On);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 5. Thermal lockout: max_tempC > BalanceTempMax forces all-zero
//    even if cells are imbalanced and FSM is in Charge.
// ---------------------------------------------------------------------------
extern "C" void test_balance_thermal_lockout(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[3][9] = 4200;
    state.max_cell_mV   = 4200;
    state.max_tempC     = static_cast<std::int16_t>(config::BalanceTempMax + 1);

    const auto mask = balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 5b. Temperature-trust gate. With untrusted temps, balancing is fully
//     disabled even for a clearly imbalanced pack in Charge -- the max_tempC
//     thermal lockout is meaningless on unvalidated data, so we must not
//     dump discharge heat into the pack. Mirrors config::TempFaultsTrusted.
// ---------------------------------------------------------------------------
extern "C" void test_balance_disabled_when_temps_untrusted(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[2][7] = static_cast<std::uint16_t>(4100 + 80);  // clearly imbalanced
    state.max_cell_mV   = state.cell_mV[2][7];

    // Sanity: with trusted temps this cell discharges in Charge.
    TEST_ASSERT_TRUE(
        balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto).cell[2][7]);
    // Untrusted temps -> nothing discharges, regardless of imbalance.
    const auto mask =
        balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/false, config::BalanceCmd::Auto);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 6. Excess exactly at the threshold (delta) -- strict-greater rule
//    means it stays untouched.
// ---------------------------------------------------------------------------
extern "C" void test_balance_threshold_strict_inequality(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[4][18] = static_cast<std::uint16_t>(4100 + config::BalanceDeltaMv);
    state.max_cell_mV    = state.cell_mV[4][18];

    const auto mask = balance::compute_mask(state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto);
    TEST_ASSERT_FALSE(mask.cell[4][18]);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// Config-invariant tripwires.
//
// These exist because of a silent failure that survived two releases: the
// balancing gate was wired to config::TempFaultsTrusted, which is false while
// the ADG731 mux path is unvalidated. The WarioCharger 0x103 toggle was
// received and accepted, VehicleService resolved it to On -- and compute_mask
// then returned an all-zero mask forever. Balancing could not run in ANY FSM
// state on ANY image, and nothing failed to say so.
// ---------------------------------------------------------------------------

// The two trust flags answer different questions and must stay independent
// knobs: "trust these temps enough to open the contactors" is not the same as
// "trust them enough to let balancing run". Re-coupling them silently kills the
// operator toggle again.
extern "C" void test_balance_temp_trust_is_decoupled_from_fault_trust(void) {
    // Balancing must be reachable while the cell-temp FAULTS stay disarmed --
    // that combination is the whole point of the split.
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;

    const auto mask = balance::compute_mask(
        state, fsm::State::Charge,
        /*temps_trusted=*/config::BalanceTempsTrusted, config::BalanceCmd::On);

    if (config::BalanceTempsTrusted) {
        TEST_ASSERT_TRUE_MESSAGE(
            any_set(mask),
            "BalanceTempsTrusted is true but balancing still produced no mask");
    } else {
        TEST_ASSERT_FALSE_MESSAGE(
            any_set(mask),
            "BalanceTempsTrusted is false so balancing must be fully inert");
    }
}

// Loud reminder rather than a silent dead toggle: if this build cannot balance,
// say so here instead of leaving an operator to discover it on a charger.
extern "C" void test_balance_operator_toggle_is_reachable_on_this_build(void) {
    TEST_ASSERT_TRUE_MESSAGE(
        config::BalanceTempsTrusted,
        "BalanceTempsTrusted is false -- the WarioCharger 0x103 toggle cannot "
        "discharge in ANY FSM state on this build. If that is intended, update "
        "this test and docs/CAN_MAP.md 0x103 together so it stays deliberate.");
}

// The operator switch must reach discharge in every FSM state on the build as
// configured -- not just with a hand-passed temps_trusted=true.
extern "C" void test_balance_on_discharges_in_all_states_as_configured(void) {
    if (!config::BalanceTempsTrusted) return;   // covered by the tripwire above
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;

    for (auto st : { fsm::State::Start, fsm::State::Precharge,
                     fsm::State::Transition, fsm::State::Run,
                     fsm::State::Charge, fsm::State::Error }) {
        TEST_ASSERT_TRUE_MESSAGE(
            balance::compute_mask(state, st,
                                  /*temps_trusted=*/config::BalanceTempsTrusted,
                                  config::BalanceCmd::On).cell[0][0],
            "operator ON must discharge in every FSM state");
    }
}

// The thermal lockout is balancing's only heat protection and must survive the
// trust split -- it applies to the operator override too.
extern "C" void test_balance_lockout_still_applies_with_trust_flag(void) {
    auto state = make_uniform_state(4100, config::BalanceTempMax + 1);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;
    state.max_tempC     = config::BalanceTempMax + 1;

    TEST_ASSERT_FALSE_MESSAGE(
        any_set(balance::compute_mask(state, fsm::State::Charge,
                                      /*temps_trusted=*/config::BalanceTempsTrusted,
                                      config::BalanceCmd::On)),
        "operator ON must NOT override the BalanceTempMax lockout");
}

// --- thermal-data gate ------------------------------------------------------
//
// These cover the hole that the 25 degC seed hid: balancing's only heat
// protection is the BalanceTempMax lockout on s.max_tempC, and with no
// converted channels that field is INT16_MIN -- which compares as extremely
// cool. Before the NtcNoReading sentinel, unconverted channels reported a
// plausible 25 degC instead, so this state was not even observable.

extern "C" void test_balance_refuses_with_no_thermal_data(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;
    // Nothing converted: sentinels, exactly as recompute_summaries_ leaves them.
    state.valid_temp_channels = 0;
    state.max_tempC = std::numeric_limits<std::int16_t>::min();
    state.min_tempC = std::numeric_limits<std::int16_t>::max();

    TEST_ASSERT_FALSE_MESSAGE(
        any_set(balance::compute_mask(state, fsm::State::Charge,
                                      /*temps_trusted=*/true, config::BalanceCmd::On)),
        "balancing must refuse when no temperature channel has converted");
}

// An operator ON must not override it either -- the operator overrides the
// ENABLE decision, never a safety guard.
extern "C" void test_balance_operator_on_cannot_override_thermal_data_gate(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;
    state.valid_temp_channels = 0;
    state.max_tempC = std::numeric_limits<std::int16_t>::min();

    for (auto st : { fsm::State::Start, fsm::State::Run, fsm::State::Charge }) {
        TEST_ASSERT_FALSE(any_set(balance::compute_mask(
            state, st, /*temps_trusted=*/true, config::BalanceCmd::On)));
    }
}

// Just below the threshold refuses; at it, balancing proceeds. Pins the
// boundary so raising the constant after a bench sweep is a deliberate act.
extern "C" void test_balance_thermal_data_gate_boundary(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;

    state.valid_temp_channels =
        static_cast<std::uint16_t>(config::BalanceMinValidTempCh - 1);
    TEST_ASSERT_FALSE(any_set(balance::compute_mask(
        state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::On)));

    state.valid_temp_channels = config::BalanceMinValidTempCh;
    TEST_ASSERT_TRUE(any_set(balance::compute_mask(
        state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::On)));
}

// A partially-populated pack still balances: the gate exists to catch a DEAD
// temperature path, not to demand full instrumentation.
extern "C" void test_balance_runs_with_partial_thermal_coverage(void) {
    auto state = make_uniform_state(4100, 30);
    state.cell_mV[0][0] = 4200;
    state.max_cell_mV   = 4200;
    state.valid_temp_channels =
        static_cast<std::uint16_t>(config::BalanceMinValidTempCh + 1);

    TEST_ASSERT_TRUE(any_set(balance::compute_mask(
        state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::On)));
}
