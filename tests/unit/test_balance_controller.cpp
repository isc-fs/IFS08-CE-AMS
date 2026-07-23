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
// Per-module enable (0x104): a disabled module never discharges, even with a
// cell that would otherwise balance; enabled modules are unaffected. The
// default arg (all enabled) preserves pre-0x104 behaviour.
// ---------------------------------------------------------------------------
extern "C" void test_balance_per_module_enable_gates_modules(void) {
    auto state = make_uniform_state(/* mV */ 3700, /* C */ 25);
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        state.cell_mV[m][0] = 3800;  // a hot cell in every module
    }
    state.max_cell_mV = 3800;        // min stays 3700 -> pack floor

    // Enable only modules 0, 2, 4 (mask 0b10101).
    const auto sel = balance::compute_mask(
        state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto,
        /*module_enable=*/0x15);
    TEST_ASSERT_GREATER_THAN_UINT8(0, count_set_in_module(sel, 0));
    TEST_ASSERT_EQUAL_UINT8(0, count_set_in_module(sel, 1));   // disabled -> empty
    TEST_ASSERT_GREATER_THAN_UINT8(0, count_set_in_module(sel, 2));
    TEST_ASSERT_EQUAL_UINT8(0, count_set_in_module(sel, 3));   // disabled -> empty
    TEST_ASSERT_GREATER_THAN_UINT8(0, count_set_in_module(sel, 4));

    // Default arg (all enabled) -> every module balances.
    const auto all = balance::compute_mask(
        state, fsm::State::Charge, /*temps_trusted=*/true, config::BalanceCmd::Auto);
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        TEST_ASSERT_GREATER_THAN_UINT8(0, count_set_in_module(all, m));
    }
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
    // The cap is a board-dissipation limit, so an off-by-one here is watts on a
    // real board. With spatial spreading on (BalanceSpreadNoAdjacent), the cap
    // is only reachable from NON-ADJACENT candidates -- so place kCap+1
    // over-threshold cells at alternating (even) indices, spanning both LTC
    // halves, with strictly increasing excess.
    constexpr std::uint8_t kCap = config::BalanceMaxActive;
    // Even indices 0,2,4,.. are pairwise non-adjacent and there are 10 of them
    // in 19 cells -- comfortably more than the cap.
    std::uint8_t placed[16]; std::uint8_t n_placed = 0;
    auto state = make_uniform_state(4100, 25);
    std::uint16_t ex = 60;
    for (std::uint8_t c = 0; c < config::CellsPerModule && n_placed < kCap + 1; c += 2) {
        state.cell_mV[1][c] = static_cast<std::uint16_t>(4100 + ex);
        placed[n_placed++] = c; ex += 10;
    }
    state.max_cell_mV = 4100 + ex;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true,
                                            config::BalanceCmd::Auto);

    // Exactly the cap discharges, never more.
    TEST_ASSERT_EQUAL_UINT8(kCap, count_set_in_module(mask, 1));

    // The single LOWEST over-threshold cell (placed[0], smallest excess) is the
    // one dropped; every higher one is set.
    TEST_ASSERT_FALSE_MESSAGE(mask.cell[1][placed[0]],
                              "lowest over-threshold cell must be the one dropped");
    for (std::uint8_t i = 1; i <= kCap; ++i) {
        TEST_ASSERT_TRUE_MESSAGE(mask.cell[1][placed[i]], "a top-excess cell was not selected");
    }
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

// --- spatial spread: never two physically-adjacent resistors on at once ------
//
// Adjacency is derived from the BMS_LITE PCB (balance::physically_adjacent):
// consecutive index within an LTC half, no adjacency across the 9/10 seam.

extern "C" void test_balance_adjacency_predicate(void) {
    // Same upper half, consecutive -> adjacent.
    TEST_ASSERT_TRUE (balance::physically_adjacent(0, 1));
    TEST_ASSERT_TRUE (balance::physically_adjacent(7, 8));
    // Same lower half, consecutive -> adjacent.
    TEST_ASSERT_TRUE (balance::physically_adjacent(9, 10));
    TEST_ASSERT_TRUE (balance::physically_adjacent(17, 18));
    // Non-consecutive -> not adjacent.
    TEST_ASSERT_FALSE(balance::physically_adjacent(0, 2));
    // ACROSS the LTC seam (8 upper, 9 lower) -> NOT adjacent, different rows.
    TEST_ASSERT_FALSE(balance::physically_adjacent(8, 9));
    // Symmetric.
    TEST_ASSERT_TRUE (balance::physically_adjacent(10, 9));
}

// With spreading on, no two selected cells in a module may be physically
// adjacent -- the whole point.
extern "C" void test_balance_no_two_adjacent_selected(void) {
    // Every cell wildly imbalanced so selection is forced to spread, not to
    // run out of candidates.
    auto state = make_uniform_state(3500, 25);
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m)
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
            state.cell_mV[m][c] = static_cast<std::uint16_t>(4000 + c);   // all >> delta
    state.min_cell_mV = 3500;   // floor well below, everything is a candidate
    state.max_cell_mV = 4100;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t a = 0; a < config::CellsPerModule; ++a) {
            if (!mask.cell[m][a]) continue;
            for (std::uint8_t b = 0; b < config::CellsPerModule; ++b) {
                if (a == b || !mask.cell[m][b]) continue;
                TEST_ASSERT_FALSE_MESSAGE(balance::physically_adjacent(a, b),
                                          "two physically-adjacent cells were selected");
            }
        }
    }
}

// Spreading must still reach a useful count on a fully-imbalanced pack: the
// halves give 5+5 non-adjacent slots, so the 8 cap is achievable.
extern "C" void test_balance_spread_still_reaches_cap(void) {
    auto state = make_uniform_state(3500, 25);
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
        state.cell_mV[1][c] = static_cast<std::uint16_t>(4000 + 2 * c);
    state.min_cell_mV = 3500;
    state.max_cell_mV = 4100;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    std::uint8_t n = 0;
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) n += mask.cell[1][c] ? 1 : 0;
    // 5 upper + 5 lower = 10 non-adjacent available, capped at 8.
    TEST_ASSERT_EQUAL_UINT8(config::BalanceMaxActive, n);
}

// It still picks the HIGHEST cells subject to the constraint: given a clear
// gradient, the top non-adjacent set wins.
extern "C" void test_balance_spread_prefers_higher_cells(void) {
    auto state = make_uniform_state(3700, 25);
    // Two clearly-highest cells, non-adjacent, must both be chosen.
    state.cell_mV[2][0]  = 4200;   // upper, very high
    state.cell_mV[2][12] = 4190;   // lower, very high, not adjacent to 0
    state.min_cell_mV = 3700;
    state.max_cell_mV = 4200;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    TEST_ASSERT_TRUE(mask.cell[2][0]);
    TEST_ASSERT_TRUE(mask.cell[2][12]);
}

// When only adjacent cells are imbalanced, spreading takes the alternating
// subset rather than the whole cluster -- fewer than the cap, by design.
extern "C" void test_balance_spread_thins_a_cluster(void) {
    auto state = make_uniform_state(3800, 25);
    // Cells 0..4 (upper, all consecutive) imbalanced, nothing else.
    for (std::uint8_t c = 0; c < 5; ++c) state.cell_mV[3][c] = static_cast<std::uint16_t>(3900 + c);
    state.min_cell_mV = 3800;
    state.max_cell_mV = 3904;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    std::uint8_t n = 0;
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) n += mask.cell[3][c] ? 1 : 0;
    // 5 consecutive cells -> at most 3 non-adjacent (0,2,4).
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(3, n);
    TEST_ASSERT_GREATER_THAN_UINT8(0, n);
    // and no two adjacent
    for (std::uint8_t a = 0; a < config::CellsPerModule; ++a)
        for (std::uint8_t b = a + 1; b < config::CellsPerModule; ++b)
            if (mask.cell[3][a] && mask.cell[3][b])
                TEST_ASSERT_FALSE(balance::physically_adjacent(a, b));
}

// --- floor robustness: one stuck-low cell must not balance the whole stack ---
//
// A disconnected cell tap reads spuriously low. If that collapsed the balancing
// floor, every real cell would look imbalanced and the whole pack would bleed
// off one bad reading. The 2nd-lowest floor ignores exactly one outlier.

extern "C" void test_balance_single_low_outlier_does_not_bleed_stack(void) {
    // A well-matched pack (everything ~3800) plus ONE spuriously-low cell.
    auto state = make_uniform_state(3800, 25);
    state.cell_mV[2][5] = 3000;          // disconnected-cell signature: reads low
    state.min_cell_mV   = 3000;          // true min (safety UV still sees this)
    state.max_cell_mV   = 3800;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    // The floor is the 2nd-lowest (3800), so nothing is >3800+50 -> NO cell
    // anywhere should be selected. Without the fix, every module would bleed.
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m)
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
            TEST_ASSERT_FALSE_MESSAGE(mask.cell[m][c],
                                      "a single low outlier must not trigger stack-wide balancing");
}

// The outlier itself (the lowest cell) is never selected -- you don't bleed the
// bottom of the pack.
extern "C" void test_balance_lowest_cell_not_selected(void) {
    auto state = make_uniform_state(3800, 25);
    state.cell_mV[0][0] = 3000;          // lowest
    state.cell_mV[1][1] = 3950;          // genuinely high, above 2nd-lowest+delta
    state.min_cell_mV   = 3000;
    state.max_cell_mV   = 3950;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    TEST_ASSERT_FALSE(mask.cell[0][0]);          // the low outlier is never bled
    TEST_ASSERT_TRUE (mask.cell[1][1]);          // the genuinely high cell still is
}

// A genuinely imbalanced pack still balances normally: two low cells set the
// floor near the bottom, the high cells are selected. Confirms the 2nd-lowest
// floor doesn't blunt real balancing.
extern "C" void test_balance_real_imbalance_still_works_with_robust_floor(void) {
    auto state = make_uniform_state(3900, 25);
    state.cell_mV[3][0] = 3700;          // two low cells -> floor ~3700
    state.cell_mV[3][2] = 3705;
    state.min_cell_mV   = 3700;
    state.max_cell_mV   = 3900;

    const auto mask = balance::compute_mask(state, fsm::State::Charge,
                                            /*temps_trusted=*/true, config::BalanceCmd::On);
    // High cells (3900 >> 3705+50) are selected across modules.
    std::uint8_t n = 0;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m)
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
            n += mask.cell[m][c] ? 1 : 0;
    TEST_ASSERT_GREATER_THAN_UINT8(0, n);
}
