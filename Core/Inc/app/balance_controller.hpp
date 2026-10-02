// SPDX-License-Identifier: proprietary
//
// Passive cell-balancing controller. Pure logic; no HAL, no FreeRTOS,
// so the host unit-test build exercises the policy directly.
//
// Two layers:
//   * compute_mask -- a PURE function of one BmsState snapshot: the gates, then
//     the cell selection. Every rule that decides WHICH cells bleed lives here.
//   * Controller   -- a small stateful wrapper (still HAL-free) that remembers
//     what compute_mask needs from the last window (the previous mask, for
//     hysteresis), adds hysteresis to the pack-temperature lockout, and reports
//     a state plus every reason balancing is blocked. BmsPollTask owns one.
// The LTC6811 WRCFGA packing happens in ltc6811::pack_cfga_payload and the
// chain TX in BmsPollTask -- this header owns only the policy.
//
// Gates (gate_reasons). Any one of them yields an all-zero mask:
//   1. op_cmd == Off (including the dead-man fallback)
//   2. the FSM state does not allow the command: Auto runs only in Charge,
//      On (operator force) only in Start or Charge. Never in Precharge,
//      Transition, Run or Error.
//   3. a latched CELL-DATA fault -- see is_cell_data_fault
//   4. cell temperatures not trusted, or too few valid channels to judge
//   5. max_tempC above BalanceTempMax
//   6. every module disabled by 0x104
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
#include "safety_predicates.hpp"   // safety::FaultReason (cell-data fault gate)
#include "state_machine.hpp"

#include <array>
#include <cstdint>

namespace ams::balance {

struct Mask {
    bool cell[config::BmsModuleCount][config::CellsPerModule];
};

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

// Why balancing is not running. A bitfield: every gate that blocks sets its
// bit, so the pit sees all of them at once, not just the first. Published on
// pit-diag 0x6CC and in the SD log; the bit positions are a wire contract.
namespace inhibit {
inline constexpr std::uint16_t OpOff           = 1u << 0;  // 0x103 Off, stale or never seen
inline constexpr std::uint16_t StateNotAllowed = 1u << 1;  // FSM state does not allow the command
inline constexpr std::uint16_t CellDataFault   = 1u << 2;  // open wire / OV / UV latched
inline constexpr std::uint16_t TempsUntrusted  = 1u << 3;  // BalanceTempsTrusted == false
inline constexpr std::uint16_t NoThermalData   = 1u << 4;  // < BalanceMinValidTempCh valid channels
inline constexpr std::uint16_t PackHot         = 1u << 5;  // max_tempC lockout (with hysteresis)
inline constexpr std::uint16_t ModulesDisabled = 1u << 6;  // every cell that needs bleeding is in a 0x104-disabled module
inline constexpr std::uint16_t QuiesceFailHold = 1u << 7;  // last quiesce unproven: mask held, not re-picked
}  // namespace inhibit

// Can the operator command run in this FSM state? Balancing heats the pack and
// ranks cells by voltage, so it belongs only where the pack is at rest or on
// the charger: Auto in Charge, On in Start or Charge. Precharge and Transition
// are seconds long and mid-sequence; Run carries load current (cells would be
// ranked by I*R, not state of charge); Error has already decided something is
// wrong.
[[nodiscard]] inline bool command_allowed_in(config::BalanceCmd op_cmd,
                                             fsm::State s) noexcept {
    switch (op_cmd) {
        case config::BalanceCmd::Auto: return s == fsm::State::Charge;
        case config::BalanceCmd::On:   return s == fsm::State::Start ||
                                              s == fsm::State::Charge;
        case config::BalanceCmd::Off:  return false;
    }
    return false;
}

// Every gate that stops balancing right now, as inhibit:: bits; 0 = balancing
// may run. Pure, so compute_mask and the Controller share one definition of
// "blocked" and the reported reasons can never drift from the behaviour.
[[nodiscard]] inline std::uint16_t gate_reasons(const BmsState&     s,
                                                fsm::State          fsm_state,
                                                bool                temps_trusted,
                                                config::BalanceCmd  op_cmd,
                                                safety::FaultReason fault_reason,
                                                std::uint8_t        module_enable) noexcept {
    std::uint16_t r = 0;

    // Operator master switch. op_cmd is already freshness-resolved by
    // VehicleService::effective_balance_cmd (the dead-man is folded in, so a
    // stale / absent WarioCharger link arrives here as Off). The safety guards
    // below (temp-trust, thermal lockout) apply to BOTH On and Auto -- the
    // operator overrides the ENABLE decision, never the guards.
    if (op_cmd == config::BalanceCmd::Off) {
        r |= inhibit::OpOff;
    } else if (!command_allowed_in(op_cmd, fsm_state)) {
        r |= inhibit::StateNotAllowed;
    }

    // CELL-DATA gate, and it binds On as well as Auto. The selector reads RAW
    // s.cell_mV -- the tap-artifact guard's corrected pair average lives in a
    // LOCAL agg_v[] inside recompute_summaries_ and never reaches it. So a split
    // tap (one half reading 4600 mV, the other 3000) is masked for the OV
    // predicate but still presents 4600 mV here, and the greedy would pick it
    // first on every cycle while BalanceSpreadNoAdjacent locks out its
    // genuinely-imbalanced neighbour. A latched cell-data fault means the
    // voltages ranked here are not trustworthy, and heating the pack on numbers
    // already faulted on is never right.
    //
    // Deliberately narrow: ONLY the reasons that say "the cell voltages are
    // wrong" (is_cell_data_fault).
    if (is_cell_data_fault(fault_reason)) r |= inhibit::CellDataFault;

    // Temperature-trust gate. Passive balancing dumps heat into the cells and
    // the max_tempC lockout is its only thermal protection. When the cell-temp
    // path isn't trusted that guard reads meaningless data, so refuse to balance
    // at all rather than heat the pack on numbers the FSM won't even fault on.
    if (!temps_trusted) r |= inhibit::TempsUntrusted;

    // Thermal DATA gate, distinct from the trust gate above. max_tempC is
    // INT16_MIN when nothing has converted, which compares as "wonderfully cool"
    // -- without this check a pack with a dead temperature path would balance
    // with no thermal protection at all and no symptom.
    if (s.valid_temp_channels < config::BalanceMinValidTempCh) r |= inhibit::NoThermalData;

    // Thermal lockout, stateless form: above BalanceTempMax. The Controller adds
    // the release hysteresis (BalanceTempHystC) on top.
    if (s.max_tempC > config::BalanceTempMax) r |= inhibit::PackHot;

    if ((module_enable & config::AllModulesMask) == 0u) r |= inhibit::ModulesDisabled;

    return r;
}

// The second-lowest cell in the pack: the floor balancing matches cells down
// to (why second-lowest: see compute_mask).
[[nodiscard]] inline std::uint16_t pack_floor_mV(const BmsState& s) noexcept {
    std::uint16_t lo1 = 0xFFFFu;   // lowest
    std::uint16_t lo2 = 0xFFFFu;   // second-lowest
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            const std::uint16_t v = s.cell_mV[m][c];
            if (v < lo1)      { lo2 = lo1; lo1 = v; }
            else if (v < lo2) { lo2 = v; }
        }
    }
    return lo2;
}

// The mask for one balancing window: empty if any gate blocks, otherwise the
// selection described at the top of this file.
//
// temps_trusted and op_cmd are BOTH required (no defaults) so every call site
// states the pack-temperature trust and the operator command explicitly -- a
// new caller can't silently inherit a permissive default and balance on data
// the FSM won't even fault on, or balance when the operator hasn't asked. The
// firmware path (Controller, from BmsPollTask) passes config::BalanceTempsTrusted
// and the freshness-resolved VehicleService::effective_balance_cmd.
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

    if (gate_reasons(s, fsm_state, temps_trusted, op_cmd, fault_reason,
                     module_enable) != 0u) {
        return out;
    }

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
    const std::uint16_t floor_mV = pack_floor_mV(s);

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

                if (config::BalanceSpreadNoAdjacent) {
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

// What the balancing controller is doing. Published on pit-diag 0x6CC and in
// the SD log; the values are a wire contract.
enum class State : std::uint8_t {
    Off     = 0,   // operator command Off (or stale): the only reason is OpOff
    Blocked = 1,   // at least one gate (other than Off alone) stops balancing
    Active  = 2,   // cells are being discharged
    Holding = 3,   // last quiesce unproven: previous mask kept, not re-picked
    Done    = 4,   // allowed to run, nothing to bleed: spread <= BalanceDeltaMv
};

struct Status {
    State         state       = State::Off;
    std::uint16_t inhibit     = inhibit::OpOff;  // inhibit:: bits
    std::uint8_t  active      = 0;               // cells discharging, whole pack
    std::uint16_t spread_mV   = 0;               // highest cell - pack_floor_mV
};

// One balancing window's inputs, as BmsPollTask resolves them.
struct Inputs {
    const BmsState&     bms;
    fsm::State          fsm_state;
    bool                temps_trusted;   // config::BalanceTempsTrusted
    config::BalanceCmd  op_cmd;          // VehicleService::effective_balance_cmd
    safety::FaultReason fault_reason;    // latched fault, for the cell-data gate
    std::uint8_t        module_enable;   // VehicleService::effective_balance_modules_mask
};

// Stateful wrapper around compute_mask. Owns the previous mask (hysteresis on
// the selection), the pack-temperature lockout latch (hysteresis on the thermal
// gate) and the reported Status. No RTOS, no HAL: host-tested like compute_mask.
class Controller {
public:
    // Run one balancing window and return the mask to write to the chain.
    const Mask& step(const Inputs& in) noexcept {
        std::uint16_t r = gate_reasons(in.bms, in.fsm_state, in.temps_trusted,
                                       in.op_cmd, in.fault_reason, in.module_enable);

        // Release hysteresis on the pack-temperature lockout. gate_reasons trips
        // it above BalanceTempMax; once tripped it holds until max_tempC is back
        // down to BalanceTempMax - BalanceTempHystC. Only judged on valid thermal
        // data -- without it NoThermalData blocks anyway, and the latch keeps
        // its last state rather than "cooling" on an INT16_MIN placeholder.
        if (in.bms.valid_temp_channels >= config::BalanceMinValidTempCh) {
            if (in.bms.max_tempC > config::BalanceTempMax) {
                pack_hot_ = true;
            } else if (in.bms.max_tempC <= config::BalanceTempMax - config::BalanceTempHystC) {
                pack_hot_ = false;
            }
        }
        if (pack_hot_) r |= inhibit::PackHot;

        status_.spread_mV = spread_mV(in.bms);

        if (r != 0u) {
            mask_           = Mask{};
            status_.state   = (r == inhibit::OpOff) ? State::Off : State::Blocked;
            status_.inhibit = r;
            status_.active  = 0;
            return mask_;
        }

        // compute_mask builds its result locally and reads `previous` only while
        // doing so, so passing our own mask as `previous` is safe.
        mask_ = compute_mask(in.bms, in.fsm_state, in.temps_trusted, in.op_cmd,
                             in.fault_reason, in.module_enable, &mask_);
        status_.active  = count(mask_);
        status_.inhibit = 0;

        if (status_.active > 0u) {
            status_.state = State::Active;
        } else if (status_.spread_mV <= config::BalanceDeltaMv) {
            status_.state = State::Done;
        } else {
            // Allowed to run and imbalanced, yet nothing selected: with every
            // module enabled the highest cell always qualifies, so the cells that
            // need bleeding must all sit in modules 0x104 has disabled.
            status_.state   = State::Blocked;
            status_.inhibit = inhibit::ModulesDisabled;
        }
        return mask_;
    }

    // The voltage poll could not prove the quiesce, so this window's snapshot
    // was measured under bleed and must not be ranked. The previous mask stays
    // on the chain untouched; only the reported state changes.
    void hold_for_quiesce_failure() noexcept {
        status_.state    = State::Holding;
        status_.inhibit |= inhibit::QuiesceFailHold;
    }

    [[nodiscard]] const Status& status() const noexcept { return status_; }
    [[nodiscard]] const Mask&   mask()   const noexcept { return mask_; }

    [[nodiscard]] static std::uint8_t count(const Mask& m) noexcept {
        std::uint8_t n = 0;
        for (std::uint8_t mo = 0; mo < config::BmsModuleCount; ++mo)
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
                if (m.cell[mo][c]) ++n;
        return n;
    }

    // Highest cell minus the floor balancing works toward (pack_floor_mV), so a
    // single stuck-low tap does not inflate the reported spread either.
    [[nodiscard]] static std::uint16_t spread_mV(const BmsState& s) noexcept {
        std::uint16_t hi = 0;
        for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m)
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c)
                if (s.cell_mV[m][c] > hi) hi = s.cell_mV[m][c];
        const std::uint16_t lo = pack_floor_mV(s);
        return (hi > lo) ? static_cast<std::uint16_t>(hi - lo) : 0u;
    }

private:
    Mask   mask_     = {};
    Status status_   = {};
    bool   pack_hot_ = false;
};

}  // namespace ams::balance
