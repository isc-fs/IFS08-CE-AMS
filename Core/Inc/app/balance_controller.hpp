// SPDX-License-Identifier: proprietary
//
// Passive cell-balancing controller. Pure logic; no HAL, no FreeRTOS,
// so the host unit-test build exercises the policy directly.
//
// Active only in fsm::State::Charge. Each call independently decides
// which cells should be discharged this balancing window based on a
// BmsState snapshot. The actual LTC6811 WRCFGA packing happens in
// ltc6811::pack_cfga_payload and the chain TX happens in
// BmsPollTask -- this header owns only the policy.
//
// Rules (config::Balance*):
//   0. temps not trusted, or operator override  -> mask all zero
//   1. fsm_state != Charge                       -> mask all zero
//   2. max_tempC > BalanceTempMax                -> mask all zero
//   3. cell voltage > min_cell_mV + delta        -> candidate
//   4. per module, keep at most BalanceMaxActive candidates with
//      the largest excess over the pack minimum (round-robin across
//      windows is overkill at 1 Hz cadence -- top-k is good enough)

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "state_machine.hpp"

#include <array>
#include <cstdint>

namespace ams::balance {

struct Mask {
    bool cell[config::BmsModuleCount][config::CellsPerModule];
};

// temps_trusted is REQUIRED (no default) so every call site must state the
// pack-temperature trust explicitly -- a new caller can't silently inherit a
// permissive default and balance on data the FSM won't even fault on. The
// firmware caller passes config::TempFaultsTrusted.
[[nodiscard]] inline Mask compute_mask(const BmsState&  s,
                                       fsm::State       fsm_state,
                                       bool             temps_trusted,
                                       bool             suppressed = false) noexcept {
    Mask out = {};

    // Operator override (#336): a fresh "BALO" on 0x103 pauses autonomous
    // balancing. Checked first so it can only ever PRODUCE an all-zero
    // mask -- never enable a discharge the policy below wouldn't -- and,
    // like the Charge gate, is moot outside Charge.
    if (suppressed)                            return out;
    if (fsm_state != fsm::State::Charge)       return out;

    // Temperature-trust gate. Passive balancing dumps heat into the cells and
    // the max_tempC lockout below is its ONLY thermal protection. When the
    // cell-temp path isn't trusted (temps_trusted == config::TempFaultsTrusted
    // == false -- the ADG731 mux path isn't validated on flight, and
    // unpopulated NTC slots default to a harmless-looking 25 C) that guard
    // reads meaningless data, so refuse to balance at all rather than heat the
    // pack on numbers we won't even let the FSM fault on. Mirrors the safety
    // predicates, which suppress the cell-temp FAULTS under the same flag.
    if (!temps_trusted)                        return out;

    // Bang-bang thermal lockout: no hysteresis because compute_mask
    // is stateless by design (pure function, unit-testable in
    // isolation). At the 1 Hz balance-update cadence and the slow
    // thermal time-constants of the pack, oscillation across the
    // threshold isn't a realistic failure mode. If a future
    // hysteresis becomes warranted, move the state into a small
    // BalanceController class that owns the latched lockout flag.
    if (s.max_tempC > config::BalanceTempMax) return out;

    // Bottom of the pack we're trying to match. Use the snapshot's
    // min_cell_mV; it's already the result of an iteration over the
    // full cell grid in BmsService::recompute_summaries_().
    const std::uint16_t floor_mV = s.min_cell_mV;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        // Walk the module's 19 cells, keep the top-K by excess over
        // the floor. K = BalanceMaxActive. Insertion sort into a
        // small array -- 19 cells * 4 slots = ~76 compares worst
        // case, well below noise at 1 Hz cadence.
        struct Cand {
            std::uint8_t  cell;
            std::uint16_t excess;
        };
        Cand top[config::BalanceMaxActive] = {};
        std::uint8_t n_top = 0;

        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v <= floor_mV + config::BalanceDeltaMv) continue;
            const std::uint16_t excess = static_cast<std::uint16_t>(v - floor_mV);

            if (n_top < config::BalanceMaxActive) {
                top[n_top++] = { c, excess };
            } else {
                // Replace the smallest entry if this one is bigger.
                std::uint8_t smallest = 0;
                for (std::uint8_t i = 1; i < n_top; ++i) {
                    if (top[i].excess < top[smallest].excess) smallest = i;
                }
                if (excess > top[smallest].excess) {
                    top[smallest] = { c, excess };
                }
            }
        }

        for (std::uint8_t i = 0; i < n_top; ++i) {
            out.cell[m][top[i].cell] = true;
        }
    }
    return out;
}

}  // namespace ams::balance
