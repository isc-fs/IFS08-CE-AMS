// SPDX-License-Identifier: proprietary
//
// LTC6811 open-wire (ADOW) detection -- pure logic, HAL-free so it is
// host-tested without a chain. Implements the datasheet "Open Wire Check
// (ADOW Command)" algorithm on the two conversion passes.
//
// Method (per LTC6811-1 datasheet):
//   1. Run ADOW with PUP=1 (pull-UP current) a couple of times, read all cell
//      voltages -> CELL_PU(n).
//   2. Run ADOW with PUP=0 (pull-DOWN current), read all cell voltages ->
//      CELL_PD(n).
//   3. An open on the conductor between two cells collapses the affected
//      reading differently under pull-up vs pull-down:
//        - for interior conductor n (1..N-1):  CELL_PU(n+1) - CELL_PD(n+1) < -threshold
//        - conductor 0 (bottom):               CELL_PU(1)  == 0
//        - conductor N (top):                  CELL_PD(N)  == 0
//
// `pu` / `pd` are the per-cell readings in mV, index 0 == CELL(1). `n_cells` is
// the number of cells on THIS IC (9 or 10 here, not the datasheet's 12). The
// returned mask has bit k set when conductor C(k) (k in 0..n_cells) is open --
// any non-zero value means this IC has at least one broken cell-sense wire.
//
// The measurement/encoding side (adow_cmd, the two RDCV passes, settling) is
// HAL and lives in bms_poll_task; this file owns only the decision, which is
// the part that must be provably correct. Gated at the call site behind
// config::CellOpenWireCheck until validated on the HIL chain.

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// POLL-INTEGRATION CONTRACT (for the HIL follow-up that wires ADOW live).
// Keep in lockstep with bms_service.cpp's RDCV decode + the BMS_LITE schematic;
// an off-by-one here silently mis-detects open wires.
//
// CELL COUNT IS PER-LTC, NOT UNIFORM (#423):
//   * Upper IC (chain index EVEN, LTC_1): 9 cells  -> module cells 0..8
//       RDCVA->0,1,2   RDCVB->3,4,5   RDCVC->6,7,8    (RDCVD UNUSED -- ignore)
//   * Lower IC (chain index ODD,  LTC_2): 10 cells -> module cells 9..18
//       RDCVA->9,10,11 RDCVB->12,13,14 RDCVC->15,16,17 RDCVD[0]->18
//   Call detect_open_conductors(pu, pd, n_cells, ...) with n_cells = 9 for
//   upper ICs and 10 for lower ICs. Decode ONLY those valid cells -- feeding
//   the upper IC's unused RDCVD registers would false-flag conductors C(9..12).
//   OR an upper-IC open into module cells 0..8, a lower-IC open into 9..18.
//
// TEMP-side note (same board): the ADG731 mux carries NTC_1..20 (LTC_1) /
// NTC_21..40 (LTC_2) on S1..S10 + S17..S26 -> Adg731ChannelMap {0..9,16..25};
// firmware slots 0..19 (upper) / 20..39 (lower). All 40 are populated and
// required (config::RequiredTempSlots), so slot 0 MUST NOT false-open -- which
// is exactly what the #482 mux warm-up guarantees (see WARM-UP below).
//
// WARM-UP (mirror the existing LTC settling precedents -- do not skip):
//   * ADOW: run the conversion TWICE per PUP setting before RDCV so the pull-
//     up/down current settles (datasheet "Open Wire Check"). This is the cell-
//     domain twin of the ADG731 first-select warm-up (#482 -- a throwaway
//     select to UNPOPULATED S32 absorbs the mux first-select drop so temp slot 0
//     latches instead of reading open).
//   * RDCV warm-up (#214): issue a no-op RDCFGA before the first RDCV after the
//     ADOW+settle idle, as attempt_voltage_poll() does, or stale MOSI fails PEC
//     on RDCVA for every IC.
//   * Quiesce balancing (DCP=0 + BalanceQuiesceMs) before ADOW, like the voltage
//     poll -- bleed current corrupts the open-wire delta.
// ---------------------------------------------------------------------------

namespace ams::open_wire {

// Max cells per LTC6811 IC (datasheet). Sizes the returned conductor mask
// (0..12 conductors -> fits a uint16_t).
inline constexpr std::uint8_t MaxCellsPerIc = 12u;

// Returns a bitmask of OPEN conductors: bit k => conductor C(k) is open,
// for k in 0..n_cells. 0 => no open wire detected on this IC.
//
//   pu, pd     : per-cell readings in mV from the PUP=1 / PUP=0 ADOW passes;
//                pu[0] == CELL_PU(1), pd[0] == CELL_PD(1), ...
//   n_cells    : cells on this IC (<= MaxCellsPerIc)
//   delta_mv   : open threshold (datasheet uses 400 mV) -- config::CellOpenWireDeltaMv
[[nodiscard]] inline std::uint16_t
detect_open_conductors(const std::uint16_t* pu, const std::uint16_t* pd,
                       std::uint8_t n_cells, std::uint16_t delta_mv) noexcept {
    std::uint16_t mask = 0u;
    if (pu == nullptr || pd == nullptr || n_cells == 0u ||
        n_cells > MaxCellsPerIc) {
        return 0u;
    }

    // Interior conductors 1..N-1: a large NEGATIVE pull-up-minus-pull-down
    // delta on cell (i+1)'s reading flags conductor i open. pu[i]/pd[i] are
    // CELL_*(i+1) in datasheet 1-indexing.
    for (std::uint8_t i = 1u; i < n_cells; ++i) {
        const std::int32_t diff =
            static_cast<std::int32_t>(pu[i]) - static_cast<std::int32_t>(pd[i]);
        if (diff < -static_cast<std::int32_t>(delta_mv)) {
            mask = static_cast<std::uint16_t>(mask | (1u << i));
        }
    }
    // Bottom conductor C(0): CELL_PU(1) reads 0.
    if (pu[0] == 0u) mask = static_cast<std::uint16_t>(mask | 1u);
    // Top conductor C(N): CELL_PD(N) reads 0.
    if (pd[n_cells - 1u] == 0u) {
        mask = static_cast<std::uint16_t>(mask | (1u << n_cells));
    }
    return mask;
}

// Convenience: any open wire on this IC?
[[nodiscard]] inline bool has_open(const std::uint16_t* pu, const std::uint16_t* pd,
                                   std::uint8_t n_cells, std::uint16_t delta_mv) noexcept {
    return detect_open_conductors(pu, pd, n_cells, delta_mv) != 0u;
}

}  // namespace ams::open_wire
