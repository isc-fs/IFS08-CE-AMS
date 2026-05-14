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
// Rules (config::kBalance*):
//   1. fsm_state != Charge                  -> mask all zero
//   2. max_tempC > kBalanceTempMax          -> mask all zero
//   3. cell voltage > min_cell_mV + delta   -> candidate
//   4. per module, keep at most kBalanceMaxActive candidates with
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
    bool cell[config::kBmsModuleCount][config::kCellsPerModule];
};

[[nodiscard]] inline Mask compute_mask(const BmsState&  s,
                                       fsm::State       fsm_state) noexcept {
    Mask out = {};

    if (fsm_state != fsm::State::Charge)       return out;
    if (s.max_tempC > config::kBalanceTempMax) return out;

    // Bottom of the pack we're trying to match. Use the snapshot's
    // min_cell_mV; it's already the result of an iteration over the
    // full cell grid in BmsService::recompute_summaries_().
    const std::uint16_t floor_mV = s.min_cell_mV;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        // Walk the module's 19 cells, keep the top-K by excess over
        // the floor. K = kBalanceMaxActive. Insertion sort into a
        // small array -- 19 cells * 4 slots = ~76 compares worst
        // case, well below noise at 1 Hz cadence.
        struct Cand {
            std::uint8_t  cell;
            std::uint16_t excess;
        };
        Cand top[config::kBalanceMaxActive] = {};
        std::uint8_t n_top = 0;

        for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v <= floor_mV + config::kBalanceDeltaMv) continue;
            const std::uint16_t excess = static_cast<std::uint16_t>(v - floor_mV);

            if (n_top < config::kBalanceMaxActive) {
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
