// SPDX-License-Identifier: proprietary
//
// Passive cell-balancing controller. Pure logic; no HAL, no FreeRTOS,
// so the host unit-test build exercises the policy directly.
//
// Gated by the operator master switch (op_cmd, #336). Each call independently
// decides which cells should be discharged this balancing window based on a
// BmsState snapshot. The actual LTC6811 WRCFGA packing happens in
// ltc6811::pack_cfga_payload and the chain TX happens in BmsPollTask -- this
// header owns only the policy.
//
// Rules (config::Balance*):
//   0. op_cmd == Off (incl. the dead-man fallback)  -> mask all zero
//   1. op_cmd == Auto AND fsm_state != Charge        -> mask all zero
//      (op_cmd == On runs in ANY state -- operator override of Charge-only)
//   2. temps not trusted                             -> mask all zero
//   3. max_tempC > BalanceTempMax                    -> mask all zero
//   4. cell voltage > floor + delta (floor = 2nd-lowest cell)  -> candidate
//   5. per module, keep at most BalanceMaxActive candidates with
//      the largest excess over the pack minimum (round-robin across
//      windows is overkill at 1 Hz cadence -- top-k is good enough)

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "safety_predicates.hpp"   // safety::FaultReason (cell-data fault gate)
#include "state_machine.hpp"

#include <array>
#include <cstdint>

namespace ams::balance {

struct Mask {
    bool cell[config::BmsModuleCount][config::CellsPerModule];
};

// temps_trusted and op_cmd are BOTH required (no defaults) so every call site
// states the pack-temperature trust and the operator command explicitly -- a
// new caller can't silently inherit a permissive default and balance on data
// the FSM won't even fault on, or balance when the operator hasn't asked. The
// firmware caller passes config::TempFaultsTrusted and the freshness-resolved
// VehicleService::effective_balance_cmd.
// Are two module-local cell indices PHYSICALLY adjacent 2512 pairs?
//
// Verified from the BMS_LITE PCB placement (pcbs/BMS_LITE): each LTC drives one
// horizontal row of cell positions, and firmware cell index maps MONOTONICALLY
// onto that row -- LTC_1 carries module cells 0..(CellsPerLtcUpper-1) left to
// right, LTC_2 carries the rest. So two cells share a board edge iff they are
// consecutive indices in the SAME LTC half. The two halves sit in separate
// board regions (a wide X gap on the layout), so there is no cross-half
// adjacency -- index 8 and 9 are far apart, not neighbours.
//
// BENCH-VERIFIED 2026-07-22 on the real pack: forcing local indices 0..7 lit
// exactly 8 CONTIGUOUS 2512 pads on one LTC row with the other row cold (IR),
// confirming consecutive firmware index == physically consecutive resistor and
// that the two LTC halves are separate board rows. So the derivation below
// (schematic cell number == LTC channel, monotonic layout) holds on hardware.
[[nodiscard]] inline bool physically_adjacent(std::uint8_t a, std::uint8_t b) noexcept {
    const bool a_upper = a < config::CellsPerLtcUpper;
    const bool b_upper = b < config::CellsPerLtcUpper;
    if (a_upper != b_upper) return false;                 // different LTC rows
    const std::uint8_t d = (a > b) ? (a - b) : (b - a);
    return d == 1u;
}

// Does this latched fault mean the CELL VOLTAGES compute_mask ranks are
// untrustworthy? Only these -- a fault elsewhere in the system (current sensor,
// VCU/charger link, contactor path) leaves cell data intact and must stay
// operator-overridable so a pack can still be rebalanced in the pit.
//
// CellOpenWire: the tap is open, so BOTH cells sharing that node are wrong (one
//   rails high, one low, pair sum conserved) -- and the high one is exactly what
//   the greedy selects first.
// CellOverVoltage / CellUnderVoltage: either a real excursion (balancing must
//   not be part of the response) or the artifact that produced it, and we cannot
//   tell which from here.
[[nodiscard]] inline bool is_cell_data_fault(safety::FaultReason r) noexcept {
    return r == safety::FaultReason::CellOpenWire     ||
           r == safety::FaultReason::CellOverVoltage  ||
           r == safety::FaultReason::CellUnderVoltage;
}

[[nodiscard]] inline Mask compute_mask(const BmsState&    s,
                                       fsm::State         fsm_state,
                                       bool               temps_trusted,
                                       config::BalanceCmd op_cmd,
                                       safety::FaultReason fault_reason
                                           = safety::FaultReason::None,
                                       std::uint8_t       module_enable
                                           = config::AllModulesMask) noexcept {
    Mask out = {};

    // Operator master switch (#336). op_cmd is already freshness-resolved by
    // VehicleService::effective_balance_cmd (the dead-man is folded in, so a
    // stale / absent WarioCharger link arrives here as Off):
    //   Off  -> never balance.
    //   On   -> operator forces balancing in ANY state (skips the Charge gate).
    //   Auto -> autonomous policy, Charge-only.
    // The safety guards below (temp-trust, thermal lockout) apply to BOTH On
    // and Auto -- the operator overrides the ENABLE decision, never the guards.
    if (op_cmd == config::BalanceCmd::Off)     return out;
    if (op_cmd == config::BalanceCmd::Auto &&
        fsm_state != fsm::State::Charge)       return out;

    // CELL-DATA gate, and it binds On as well as Auto. The selector below reads
    // RAW s.cell_mV -- the tap-artifact guard's corrected pair average lives in a
    // LOCAL agg_v[] inside recompute_summaries_ and never reaches this function.
    // So a split tap (one half reading 4600 mV, the other 3000) is masked for the
    // OV predicate but still presents 4600 mV here, and the greedy picks it first
    // on every cycle, forever, while BalanceSpreadNoAdjacent locks out its
    // genuinely-imbalanced neighbour.
    //
    // ADOW (config::CellOpenWireCheck, live since v2.1.0) latches exactly this in
    // under 500 ms in ANY state -- but Auto only stops because the AIRs open and
    // we leave Charge, and On skips the state gate entirely. Refuse instead: a
    // latched cell-data fault means the voltages this function ranks are not
    // trustworthy, and heating the pack on numbers we have already faulted on is
    // never right. This finishes implementing the principle stated above (the
    // operator overrides the ENABLE decision, never the guards).
    //
    // Deliberately narrow: ONLY the reasons that say "the cell voltages are
    // wrong". A contactor/IMD/current fault leaves the cell data perfectly good,
    // so those stay overridable and the pit keeps its manual-rebalance path.
    if (is_cell_data_fault(fault_reason))      return out;

    // Temperature-trust gate. Passive balancing dumps heat into the cells and
    // the max_tempC lockout below is its ONLY thermal protection. When the
    // cell-temp path isn't trusted (temps_trusted == config::TempFaultsTrusted
    // == false -- the ADG731 mux path isn't validated on flight, and
    // unpopulated NTC slots default to a harmless-looking 25 C) that guard
    // reads meaningless data, so refuse to balance at all rather than heat the
    // pack on numbers we won't even let the FSM fault on. Mirrors the safety
    // predicates, which suppress the cell-temp FAULTS under the same flag.
    if (!temps_trusted)                        return out;

    // Thermal DATA gate, distinct from the trust gate above. The BalanceTempMax
    // lockout below is balancing's only heat protection, and it reads
    // s.max_tempC -- which is INT16_MIN when nothing has converted. INT16_MIN
    // compares as "wonderfully cool", so without this check a pack with a dead
    // temperature path balances with no thermal protection at all and no
    // symptom. Refuse instead.
    //
    // Before the NtcNoReading sentinel this could not even be detected:
    // unconverted channels were seeded to a plausible 25 degC.
    if (s.valid_temp_channels < config::BalanceMinValidTempCh) return out;

    // Bang-bang thermal lockout: no hysteresis because compute_mask
    // is stateless by design (pure function, unit-testable in
    // isolation). At the 1 Hz balance-update cadence and the slow
    // thermal time-constants of the pack, oscillation across the
    // threshold isn't a realistic failure mode. If a future
    // hysteresis becomes warranted, move the state into a small
    // BalanceController class that owns the latched lockout flag.
    if (s.max_tempC > config::BalanceTempMax) return out;

    // Bottom of the pack we're trying to match -- the SECOND-lowest cell, not
    // the absolute minimum.
    //
    // A disconnected cell tap reads spuriously low (the LTC measures each cell
    // across shared tap nodes, so an open node collapses one reading). If the
    // floor were the true minimum, that one bad cell would drop it far below the
    // pack, every real cell would then sit >BalanceDeltaMv above it, and the
    // WHOLE STACK would start balancing off a single faulty reading. Using the
    // 2nd-lowest ignores exactly one outlier, so a single stuck-low cell cannot
    // trigger pack-wide discharge.
    //
    // The true minimum still drives the safety UV predicate + telemetry
    // (s.min_cell_mV, untouched) -- a genuinely low cell still faults there.
    // Cost of 2nd-lowest: a single real weak cell is balanced toward the
    // next-lowest rather than itself, i.e. one deadband short -- negligible, and
    // the right trade against a false pack-wide bleed.
    std::uint16_t lo1 = 0xFFFFu;   // lowest
    std::uint16_t lo2 = 0xFFFFu;   // second-lowest
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v < lo1)      { lo2 = lo1; lo1 = v; }
            else if (v < lo2) { lo2 = v; }
        }
    }
    const std::uint16_t floor_mV = lo2;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        // Per-module operator enable (0x104), layered UNDER the global
        // OFF/ON/AUTO above: a disabled module never discharges. Already
        // freshness-resolved by VehicleService::effective_balance_modules_mask
        // (stale/absent -> all bits set), so the default all-enabled arg keeps
        // pre-0x104 behaviour. The pack-wide floor (lo2) still includes a
        // disabled module's cells -- they simply never get selected here.
        if ((module_enable & (1u << m)) == 0u) continue;
        // Walk the module's 19 cells, keep the top-K by excess over
        // the floor. K = BalanceMaxActive. Insertion sort into a
        // small array -- 19 cells * 4 slots = ~76 compares worst
        // case, well below noise at 1 Hz cadence.
        // Select up to BalanceMaxActive cells, highest excess first, with NO
        // two PHYSICALLY ADJACENT resistors on at once (see physically_adjacent
        // + BalanceSpreadNoAdjacent). Spreads the discharge heat across the
        // board instead of concentrating a hot cluster of 2512 pads -- measured
        // at ~71 C per pad at 8/module, so keeping neighbours cold bounds the
        // local hot-spot over a multi-hour C/101 balancing session.
        //
        // Greedy: repeatedly take the highest-excess cell that is over the delta
        // and not adjacent to one already chosen. It may select FEWER than the
        // cap when the imbalanced cells cluster -- which is correct: a smaller
        // set that never overlaps is exactly the goal, and the skipped cells are
        // bled on later cycles once their neighbours come down. 8*19 compares
        // per module, trivial at 1 Hz.
        std::uint8_t chosen[config::BalanceMaxActive] = {};
        std::uint8_t n_chosen = 0;

        for (std::uint8_t pick = 0; pick < config::BalanceMaxActive; ++pick) {
            std::uint16_t best_excess = 0;
            int           best_cell   = -1;
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
                const std::uint16_t v = s.cell_mV[m][c];
                if (v <= floor_mV + config::BalanceDeltaMv) continue;  // matched
                if (out.cell[m][c]) continue;                          // taken

                if (config::BalanceSpreadNoAdjacent) {
                    bool adj = false;
                    for (std::uint8_t i = 0; i < n_chosen; ++i) {
                        if (physically_adjacent(c, chosen[i])) { adj = true; break; }
                    }
                    if (adj) continue;
                }

                const std::uint16_t excess = static_cast<std::uint16_t>(v - floor_mV);
                if (excess > best_excess) { best_excess = excess; best_cell = c; }
            }
            if (best_cell < 0) break;                          // nothing eligible
            out.cell[m][static_cast<std::uint8_t>(best_cell)] = true;
            chosen[n_chosen++] = static_cast<std::uint8_t>(best_cell);
        }
    }
    return out;
}

}  // namespace ams::balance
