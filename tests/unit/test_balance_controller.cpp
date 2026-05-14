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

namespace {

using namespace ams;

// Build a synthetic, fully-populated BmsState with every cell at
// base_mV and every NTC at base_C, plus the recomputed summaries
// (min_cell_mV, max_tempC) we rely on inside compute_mask.
BmsState make_uniform_state(std::uint16_t base_mV, std::int16_t base_C) {
    BmsState s{};
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
            s.cell_mV[m][c] = base_mV;
        }
        for (std::uint8_t t = 0; t < config::kTempsPerModule; ++t) {
            s.cell_tempC[m][t] = base_C;
        }
    }
    s.min_cell_mV = base_mV;
    s.max_cell_mV = base_mV;
    s.min_tempC   = base_C;
    s.max_tempC   = base_C;
    return s;
}

bool any_set(const balance::Mask& m) {
    for (std::uint8_t mm = 0; mm < config::kBmsModuleCount; ++mm) {
        for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
            if (m.cell[mm][c]) return true;
        }
    }
    return false;
}

std::uint8_t count_set_in_module(const balance::Mask& m, std::uint8_t module) {
    std::uint8_t n = 0;
    for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
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
    const auto mask = balance::compute_mask(state, fsm::State::Charge);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 2. One cell 80 mV above min in Charge -> that cell discharges.
//    Other cells stay quiet (delta below kBalanceDeltaMv = 50 mV).
// ---------------------------------------------------------------------------
extern "C" void test_balance_single_hot_cell_in_charge(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[2][7] = static_cast<std::uint16_t>(4100 + 80);
    state.max_cell_mV  = state.cell_mV[2][7];

    const auto mask = balance::compute_mask(state, fsm::State::Charge);
    TEST_ASSERT_TRUE(mask.cell[2][7]);
    // Only that one cell -- floor + 50 is the threshold, others sit
    // exactly at the floor so none should be selected.
    TEST_ASSERT_EQUAL_UINT8(1u, count_set_in_module(mask, 2));
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        if (m == 2) continue;
        TEST_ASSERT_EQUAL_UINT8(0u, count_set_in_module(mask, m));
    }
}

// ---------------------------------------------------------------------------
// 3. Six cells in one module above the threshold -- the controller
//    must cap to kBalanceMaxActive (4) and pick the ones with the
//    largest excess.
// ---------------------------------------------------------------------------
extern "C" void test_balance_caps_at_max_active_per_module(void) {
    auto state = make_uniform_state(4100, 25);
    // Cells 0..5 of module 1 sit at +60..+110 mV (all > delta = 50).
    // Top-4 by excess should be cells 5, 4, 3, 2 (excess 110, 100, 90, 80).
    for (std::uint8_t c = 0; c < 6; ++c) {
        state.cell_mV[1][c] = static_cast<std::uint16_t>(4100 + 60 + 10 * c);
    }
    state.max_cell_mV = state.cell_mV[1][5];

    const auto mask = balance::compute_mask(state, fsm::State::Charge);
    TEST_ASSERT_EQUAL_UINT8(config::kBalanceMaxActive,
                            count_set_in_module(mask, 1));
    TEST_ASSERT_TRUE(mask.cell[1][5]);
    TEST_ASSERT_TRUE(mask.cell[1][4]);
    TEST_ASSERT_TRUE(mask.cell[1][3]);
    TEST_ASSERT_TRUE(mask.cell[1][2]);
    TEST_ASSERT_FALSE(mask.cell[1][1]);
    TEST_ASSERT_FALSE(mask.cell[1][0]);
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
        const auto mask = balance::compute_mask(state, st);
        TEST_ASSERT_FALSE_MESSAGE(any_set(mask),
            "mask must be empty outside Charge");
    }
}

// ---------------------------------------------------------------------------
// 5. Thermal lockout: max_tempC > kBalanceTempMax forces all-zero
//    even if cells are imbalanced and FSM is in Charge.
// ---------------------------------------------------------------------------
extern "C" void test_balance_thermal_lockout(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[3][9] = 4200;
    state.max_cell_mV   = 4200;
    state.max_tempC     = static_cast<std::int16_t>(config::kBalanceTempMax + 1);

    const auto mask = balance::compute_mask(state, fsm::State::Charge);
    TEST_ASSERT_FALSE(any_set(mask));
}

// ---------------------------------------------------------------------------
// 6. Excess exactly at the threshold (delta) -- strict-greater rule
//    means it stays untouched.
// ---------------------------------------------------------------------------
extern "C" void test_balance_threshold_strict_inequality(void) {
    auto state = make_uniform_state(4100, 25);
    state.cell_mV[4][18] = static_cast<std::uint16_t>(4100 + config::kBalanceDeltaMv);
    state.max_cell_mV    = state.cell_mV[4][18];

    const auto mask = balance::compute_mask(state, fsm::State::Charge);
    TEST_ASSERT_FALSE(mask.cell[4][18]);
    TEST_ASSERT_FALSE(any_set(mask));
}
