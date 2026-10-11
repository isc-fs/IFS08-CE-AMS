// SPDX-License-Identifier: proprietary
//
// Passive cell-balancing controller. Pure logic; no HAL, no FreeRTOS,
// so the host unit-test build exercises the policy directly.
//
// Gated by the operator master switch (op_cmd). Each call independently
// decides which cells should be discharged this balancing window based on a
// BmsState snapshot. The actual LTC6811 WRCFGA packing happens in
// ltc6811::pack_cfga_payload and the chain TX happens in BmsPollTask -- this
// header owns only the policy.
//
// Gates, in order. Any one of them yields an all-zero mask:
//   1. op_cmd == Off (including the dead-man fallback)
//   2. op_cmd == Auto and the FSM is not in Charge
//      (op_cmd == On runs in ANY state -- the operator's Charge-only override)
//   3. a latched CELL-DATA fault -- see is_cell_data_fault
//   4. cell temperatures not trusted, or too few valid channels to judge
//   5. max_tempC above BalanceTempMax
//
// Selection, per module, once the gates pass:
//   * floor = the SECOND-lowest cell in the pack, not the lowest
//   * a cell already discharging stays a candidate while it is more than
//     BalanceStopDeltaMv above the floor; one that is not must exceed the
//     wider BalanceDeltaMv to become one (hysteresis -- without it the
//     selection toggles, because this function is re-evaluated from scratch
//     every BalanceUpdatePolls)
//   * greedily take the highest excess, never two physically adjacent cells,
//     up to BalanceMaxActive

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "ltc6811.hpp"             // pec15 (maint::check_cfga_readback)
#include "maint_config.hpp"
#include "safety_predicates.hpp"   // safety::FaultReason (cell-data fault gate)
#include "state_machine.hpp"

#include <array>
#include <cstddef>
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
// Bench-verified on the real pack: forcing local indices 0..7 lit
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

// Bottom of the pack balancing matches cells down to: the SECOND-lowest cell
// across the modules in `modules`, not the absolute minimum.
//
// A disconnected cell tap reads spuriously low (the LTC measures each cell
// across shared tap nodes, so an open node collapses one reading). If the floor
// were the true minimum, that one bad cell would drop it far below the pack,
// every real cell would then sit >BalanceDeltaMv above it, and the WHOLE STACK
// would start balancing off a single faulty reading. Using the 2nd-lowest
// ignores exactly one outlier, so a single stuck-low cell cannot trigger
// pack-wide discharge.
//
// The true minimum still drives the safety UV predicate + telemetry
// (s.min_cell_mV, untouched) -- a genuinely low cell still faults there. Cost of
// 2nd-lowest: a single real weak cell is balanced toward the next-lowest rather
// than itself, i.e. one deadband short -- negligible, and the right trade
// against a false pack-wide bleed.
[[nodiscard]] inline std::uint16_t floor_over_modules(const BmsState& s,
                                                      std::uint8_t    modules) noexcept {
    std::uint16_t lo1 = 0xFFFFu;   // lowest
    std::uint16_t lo2 = 0xFFFFu;   // second-lowest
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((modules & (1u << m)) == 0u) continue;
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v < lo1)      { lo2 = lo1; lo1 = v; }
            else if (v < lo2) { lo2 = v; }
        }
    }
    return lo2;
}

// Pick the cells to discharge in each module of `modules`, measured against
// floor_mV. Every gate has already passed by the time this runs. max_active and
// spread_no_adjacent default to the flight limits (BalanceMaxActive,
// BalanceSpreadNoAdjacent); only the maintenance policy passes others.
[[nodiscard]] inline Mask select_cells(const BmsState& s,
                                       std::uint16_t   floor_mV,
                                       std::uint8_t    modules,
                                       const Mask*     previous,
                                       std::uint8_t    max_active = config::BalanceMaxActive,
                                       bool            spread_no_adjacent
                                           = config::BalanceSpreadNoAdjacent) noexcept {
    Mask out = {};
    if (max_active > config::CellsPerModule) max_active = config::CellsPerModule;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((modules & (1u << m)) == 0u) continue;
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
        std::uint8_t chosen[config::CellsPerModule] = {};
        std::uint8_t n_chosen = 0;

        for (std::uint8_t pick = 0; pick < max_active; ++pick) {
            std::uint32_t best_excess = 0;
            int           best_cell   = -1;
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
                const std::uint16_t v = s.cell_mV[m][c];
                // HYSTERESIS. A cell that was ALREADY discharging last cycle
                // holds its place until it comes within BalanceStopDeltaMv of
                // the floor; a cell that was not has to clear the wider
                // BalanceDeltaMv to earn a slot. Without this the function is
                // stateless and re-ranks from scratch every BalanceUpdatePolls,
                // so anything hovering near the single threshold toggles on and
                // off and never accumulates useful bleed time.
                //
                // `previous` is passed IN rather than cached in here on purpose:
                // compute_mask stays a pure function of its arguments, which is
                // what makes the whole policy host-testable without an RTOS.
                const bool was_on = previous != nullptr && previous->cell[m][c];
                const std::uint16_t threshold =
                    was_on ? config::BalanceStopDeltaMv : config::BalanceDeltaMv;
                if (v <= floor_mV + threshold) continue;               // matched
                if (out.cell[m][c]) continue;                          // taken

                if (spread_no_adjacent) {
                    bool adj = false;
                    for (std::uint8_t i = 0; i < n_chosen; ++i) {
                        if (physically_adjacent(c, chosen[i])) { adj = true; break; }
                    }
                    if (adj) continue;
                }

                // Rank by excess, but let an incumbent outrank a newcomer at
                // equal excess. Without this tie-break the cap could evict a
                // cell that is actively bleeding in favour of one a millivolt
                // higher, which is exactly the churn the hysteresis is meant to
                // stop -- the thresholds alone do not prevent it, because the
                // eviction happens through the BalanceMaxActive cap rather than
                // through the threshold test.
                const std::uint32_t excess =
                    static_cast<std::uint32_t>(v - floor_mV) * 2u + (was_on ? 1u : 0u);
                if (excess > best_excess) { best_excess = excess; best_cell = c; }
            }
            if (best_cell < 0) break;                          // nothing eligible
            out.cell[m][static_cast<std::uint8_t>(best_cell)] = true;
            chosen[n_chosen++] = static_cast<std::uint8_t>(best_cell);
        }
    }
    return out;
}

[[nodiscard]] inline Mask compute_mask(const BmsState&    s,
                                       fsm::State         fsm_state,
                                       bool               temps_trusted,
                                       config::BalanceCmd op_cmd,
                                       safety::FaultReason fault_reason
                                           = safety::FaultReason::None,
                                       std::uint8_t       module_enable
                                           = config::AllModulesMask,
                                       const Mask*        previous
                                           = nullptr) noexcept {
    Mask out = {};

    // Operator master switch. op_cmd is already freshness-resolved by
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
    // ADOW (config::CellOpenWireCheck) latches exactly this in
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

    // Per-module operator enable (0x104), layered UNDER the global OFF/ON/AUTO
    // above: a disabled module never discharges. Already freshness-resolved by
    // VehicleService::effective_balance_modules_mask (stale/absent -> all bits
    // set), so the default all-enabled arg keeps pre-0x104 behaviour. The
    // pack-wide floor still includes a disabled module's cells -- they simply
    // never get selected.
    return select_cells(s, floor_over_modules(s, config::AllModulesMask),
                        module_enable, previous);
}

// ---------------------------------------------------------------------------
// Maintenance-balancing policy. Called ONLY from the -DAMS_MAINT_BALANCE build
// (maint_config.hpp); the flight build never reaches it.
//
// The maintenance build keeps every safety predicate, so a missing module or an
// open sensor latches Error with the AIRs open, exactly as in flight. This policy
// lets the operator bleed cells in that state, and it replaces the flight gates
// that a latched Error would otherwise defeat:
//
//   * The latched fault reason is the FIRST fault only. Once BmsModuleOffline
//     has latched, a later open wire or over/under-voltage never shows up in
//     it, and with a module missing the FSM's own OV/UV checks never arm
//     (first_full_poll_done stays false). So the cell-data gate is re-derived
//     here, every window, from the live BmsState.
//   * A module that is not answering still holds old or boot-seed (3700 mV)
//     cell values. Only modules answering right now feed the floor or get
//     selected, so a missing module cannot drag the floor or be "balanced".
//   * The thermal gates read only modules whose sensors are trusted:
//     IgnoredTempModules is left out of the lockout, the data count and the
//     lost-sensor check.
//
// Gates, in order. Any one of them yields an all-zero mask:
//   1. op_cmd != On (Auto needs Charge, which this build never reaches; Off and
//      the dead-man stay Off)
//   2. FSM not in Start or Error -- the AIRs are open in both
//   3. no module answering
//   4. cell data untrustworthy on an answering module: open wire, displaced tap,
//      any cell outside CellUnderVoltageMv..CellOverVoltageMv, or a latched
//      cell-data fault
//   5. temperatures not trusted, too few valid channels on the trusted modules,
//      a trusted module's sensor lost, or the trusted max above BalanceTempMax
//
// Selection is the flight greedy with the maintenance limits
// (maint_config::MaxActivePerModule, SpreadNoAdjacent): uncapped, neighbours
// allowed.
// ---------------------------------------------------------------------------
namespace maint {

inline constexpr std::uint16_t InhibitNotOn           = 1u << 0;
inline constexpr std::uint16_t InhibitStateNotAllowed = 1u << 1;
inline constexpr std::uint16_t InhibitNoModules       = 1u << 2;
inline constexpr std::uint16_t InhibitCellData        = 1u << 3;
inline constexpr std::uint16_t InhibitTempsUntrusted  = 1u << 4;
inline constexpr std::uint16_t InhibitNoThermalData   = 1u << 5;
inline constexpr std::uint16_t InhibitPackHot         = 1u << 6;
inline constexpr std::uint16_t InhibitTempSensorLost  = 1u << 7;

// Modules answering right now AND holding real data: online (fresh within
// BmsStaleMs) and reported PEC-clean at least once. The second test excludes the
// first BmsStaleMs of uptime, when a module that has never answered still counts
// as online over its 3700 mV boot seed.
[[nodiscard]] inline std::uint8_t data_modules(const BmsState& s) noexcept {
    std::uint8_t mods = 0;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((s.module_online_mask & (1u << m)) != 0u && s.last_rx_tick[m] != 0u) {
            mods = static_cast<std::uint8_t>(mods | (1u << m));
        }
    }
    return mods;
}

[[nodiscard]] inline std::uint16_t gate_reasons(const BmsState&     s,
                                                fsm::State          fsm_state,
                                                bool                temps_trusted,
                                                config::BalanceCmd  op_cmd,
                                                safety::FaultReason fault_reason,
                                                std::uint8_t        ignored_temp_modules) noexcept {
    std::uint16_t r = 0;

    if (op_cmd != config::BalanceCmd::On) r |= InhibitNotOn;
    if (fsm_state != fsm::State::Start && fsm_state != fsm::State::Error) {
        r |= InhibitStateNotAllowed;
    }

    const std::uint8_t mods = data_modules(s);
    if (mods == 0u) {
        r |= InhibitNoModules;
        return r;   // nothing below has data to judge
    }

    // Cell data, judged on RAW cell_mV -- the values select_cells ranks.
    bool cell_bad = is_cell_data_fault(fault_reason) ||
                    (s.cell_open_mask & mods) != 0u ||
                    (s.tap_fault_mask & mods) != 0u;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((mods & (1u << m)) == 0u) continue;
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v < config::CellUnderVoltageMv || v > config::CellOverVoltageMv) cell_bad = true;
        }
    }
    if (cell_bad) r |= InhibitCellData;

    // Thermal, on the modules whose sensors are trusted.
    if (!temps_trusted) r |= InhibitTempsUntrusted;
    const std::uint8_t thermal = static_cast<std::uint8_t>(mods & ~ignored_temp_modules);
    std::uint16_t n_valid = 0;
    std::int16_t  t_max   = config::NtcNoReading;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((thermal & (1u << m)) == 0u) continue;
        for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
            const std::int16_t tc = s.cell_tempC[m][t];
            if (tc == config::NtcNoReading) continue;
            ++n_valid;
            if (tc > t_max) t_max = tc;
        }
    }
    if (n_valid < config::BalanceMinValidTempCh)        r |= InhibitNoThermalData;
    if ((s.temp_disconnect_mask & thermal) != 0u)       r |= InhibitTempSensorLost;
    if (t_max > config::BalanceTempMax)                 r |= InhibitPackHot;

    return r;
}

[[nodiscard]] inline Mask compute_mask(const BmsState&     s,
                                       fsm::State          fsm_state,
                                       bool                temps_trusted,
                                       config::BalanceCmd  op_cmd,
                                       safety::FaultReason fault_reason,
                                       std::uint8_t        module_enable,
                                       std::uint8_t        ignored_temp_modules,
                                       const Mask*         previous) noexcept {
    if (gate_reasons(s, fsm_state, temps_trusted, op_cmd, fault_reason,
                     ignored_temp_modules) != 0u) {
        return Mask{};
    }
    const std::uint8_t mods = data_modules(s);
    return select_cells(s, floor_over_modules(s, mods),
                        static_cast<std::uint8_t>(mods & module_enable), previous,
                        maint_config::MaxActivePerModule,
                        maint_config::SpreadNoAdjacent);
}

// Result of reading the configuration registers back after a WRCFGA.
enum class Readback : std::uint8_t {
    Ok,         // every IC with a discharge bit to set reads back exactly that
    Unproven,   // an IC that should be discharging did not answer PEC-clean
    Mismatch,   // an IC answered PEC-clean with DCC bits other than written
};

// Check an RDCFGA reply (LtcChainLength x [6 data + 2 PEC], chain order as the
// cell reads use it) against the per-IC payloads just written.
//
// Why this exists: with the far module unplugged the chain is shorter than the
// LtcChainLength payloads written, and which IC keeps which payload in a short
// daisy chain has not been verified on hardware. Reading back is the proof that
// each discharge bit landed on the IC it was meant for, independent of that.
//
// Only DCC1..DCC12 are compared (byte 4, low nibble of byte 5): byte 0 reads back
// GPIO pin levels, and the VUV/VOV bytes carry nothing balancing depends on. An
// IC that does not answer is fine when it was written no discharge -- it is the
// absent module -- and unproven otherwise.
[[nodiscard]] inline Readback check_cfga_readback(
        const std::uint8_t (&written)[config::LtcChainLength][6],
        const std::uint8_t* reply, std::size_t len) noexcept {
    if (reply == nullptr || len < 8u * config::LtcChainLength) return Readback::Unproven;
    Readback result = Readback::Ok;
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const std::uint8_t* seg = reply + 8u * ic;
        const std::uint16_t pec = static_cast<std::uint16_t>((seg[6] << 8) | seg[7]);
        const bool want_dcc = written[ic][4] != 0u || (written[ic][5] & 0x0Fu) != 0u;
        if (ltc6811::pec15(seg, 6) != pec) {
            if (want_dcc) result = Readback::Unproven;
            continue;
        }
        if (seg[4] != written[ic][4] ||
            (seg[5] & 0x0Fu) != (written[ic][5] & 0x0Fu)) {
            return Readback::Mismatch;
        }
    }
    return result;
}

}  // namespace maint

}  // namespace ams::balance
