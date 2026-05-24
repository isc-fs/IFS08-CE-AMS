// SPDX-License-Identifier: proprietary
//
// Aggregates per-module cell voltages and (in #71) temperatures from
// the LTC6811-1 daisy-chain (LtcChainLength = 10 ICs feeding 5 BMS
// modules at 2 ICs each). Single-writer (BmsPollTask, #72); many
// readers (MainTask, AcuCanTask, BalanceController).
//
// Wire protocol: docs/BMS_LTC6811.md (#75) -- replaces the legacy
// FDCAN2 BMS section now marked DEPRECATED in docs/CAN_MAP.md.
// Synchronisation: docs/ARCHITECTURE.md §7 ownership map.

#pragma once

#include "ams_config.hpp"
#include "can_frame.hpp"

#include <cstddef>
#include <cstdint>

namespace ams {

struct BmsState {
    std::uint16_t cell_mV     [config::BmsModuleCount][config::CellsPerModule];
    std::int16_t  cell_tempC  [config::BmsModuleCount][config::TempsPerModule];

    // Derived summaries (recomputed on each frame; cheap, < 200 cells).
    std::uint32_t pack_voltage_mV;
    std::uint16_t min_cell_mV;
    std::uint16_t max_cell_mV;
    std::int16_t  min_tempC;
    std::int16_t  max_tempC;
    std::int16_t  avg_tempC;

    // Per-module aggregates feeding the 0x131..0x134 + 0x136..0x137
    // ECU TX matrix (fix/53). Recomputed in recompute_summaries_()
    // from cell_mV / cell_tempC; no extra cost beyond a single pass
    // per cycle. vmin/vmax in mV, tmax in degC (signed int16, clipped
    // from the per-cell int16 range).
    std::uint16_t vmin_module[config::BmsModuleCount];
    std::uint16_t vmax_module[config::BmsModuleCount];
    std::int16_t  tmax_module[config::BmsModuleCount];

    // Per-module freshness for the SafetyTask staleness check. Updated
    // on a successful poll where BOTH LTCs of the module reported
    // PEC-clean.
    std::uint32_t last_rx_tick[config::BmsModuleCount];

    // Bit N set <=> module N has reported PEC-clean at least once.
    // Compared against config::AllModulesMask (0x1F) by SafetyTask.
    // Sticky: dynamic disappearance is detected via the staleness
    // window, not by clearing this mask.
    std::uint8_t  module_online_mask;

    // 10-bit "this poll's per-IC PEC-OK" mask (LSB = chain index 0).
    // Source of truth from which module_online_mask is derived. Not
    // sticky -- reflects the most recent update_from_ltc_response.
    std::uint16_t ltc_online_mask;
};

class BmsService {
public:
    static BmsService& instance() noexcept;

    // ------------------------------------------------------------------
    // Active data path (v1.2.0+): one call per polling cycle of
    // BmsPollTask. Walks 4 register groups (RDCVA + RDCVB + RDCVC +
    // RDCVD), LtcChainLength ICs each, 8 bytes per IC (6 data + 2
    // PEC). Expected buffer layout, all 4 groups concatenated in
    // RDCV_A,B,C,D order:
    //
    //   [group_A[ic0]..[ic9]] [group_B[ic0]..[ic9]]
    //   [group_C[ic0]..[ic9]] [group_D[ic0]..[ic9]]
    //
    //   total len = 4 * LtcChainLength * 8 = 320 bytes
    //
    // Per-IC cell-slot mapping inside the module's 19-cell window:
    //
    //   LTC_1 (chain index 2N, "upper", 10 cells)
    //     RDCVA -> module cells 0,1,2
    //     RDCVB -> module cells 3,4,5
    //     RDCVC -> module cells 6,7,8
    //     RDCVD -> module cell 9 (slots 2,3 of group D unused)
    //
    //   LTC_2 (chain index 2N+1, "lower", 9 cells)
    //     RDCVA -> module cells 10,11,12
    //     RDCVB -> module cells 13,14,15
    //     RDCVC -> module cells 16,17,18
    //     RDCVD -> discarded
    //
    // Per-IC PEC handling: if ANY of the 4 register groups for an IC
    // fails PEC, that IC is marked offline this cycle and its cell
    // slots are left untouched (last known voltages remain in
    // cell_mV). The corresponding entry in g_ltc_pec_err_count
    // increments so telemetry can surface bus noise without the
    // safety supervisor latching ERROR on a single PEC blip. ERROR
    // only fires when the module's freshness window expires, exactly
    // as before.
    //
    // Returns true if the call wrote at least one module's slice;
    // false on buffer size mismatch, null pointer, or mutex timeout.
    bool update_from_ltc_response(const std::uint8_t* chain_response,
                                  std::size_t         len,
                                  std::uint32_t       now_tick_ms) noexcept;

    // ------------------------------------------------------------------
    // Temperature path. Called once per mux step in the 20-channel
    // sweep, with the RDAUXA reply for the chain
    // (LtcChainLength * 8 bytes; 6 data + 2 PEC per IC). AUX1 of
    // each IC carries the buffered ADG731 output for the channel
    // currently selected, so one call writes one column of
    // cell_tempC -- specifically:
    //
    //   slot 0..19  on cell_tempC[m]  <- LTC_1 of module m (chain_idx 2m)
    //   slot 20..39 on cell_tempC[m]  <- LTC_2 of module m (chain_idx 2m+1)
    //
    // channel_idx is the 0..19 temperature-table index (NOT the raw
    // ADG731 channel; that's hidden behind config::Adg731ChannelMap
    // in BmsPollTask). Out-of-range or PEC-failed readings keep the
    // previous value (so unpopulated NTCs don't disturb min/max/avg).
    // Returns true on PEC-clean decode of at least one IC.
    bool update_temperature(std::uint8_t        channel_idx,
                            const std::uint8_t* chain_response,
                            std::size_t         len) noexcept;

    // Atomic read of the full state. Caller gets its own copy; the
    // mutex is released before this returns.
    [[nodiscard]] BmsState snapshot() const noexcept;

    // True iff all 5 modules have reported within BmsStaleMs and
    // module_online_mask covers them. Used by SafetyTask.
    [[nodiscard]] bool is_healthy(std::uint32_t now_tick) const noexcept;

#if defined(AMS_BMS_HIL_STUB)
    // HIL-only: stamp a nominal-healthy snapshot into the state so the
    // safety predicate sees fresh, in-range BMS data without an actual
    // LTC chain on the bench. Refreshes last_rx_tick for every module
    // to now_tick and forces module_online_mask = AllModulesMask.
    // Cell V/T arrays are filled with plausible nominal values. Called
    // by BmsPollTask under the same flag on its 250 ms cadence.
    //
    // NEVER compiled into flight HW (the function literally doesn't
    // exist in the binary when the flag isn't defined). The whole
    // point is that no flight path can accidentally fake-heal a real
    // BMS fault.
    void seed_for_hil_stub(std::uint32_t now_tick) noexcept;
#endif

private:
    BmsService();

    // Internal helper; assumes the mutex is already held. Recomputes
    // pack_voltage_mV / min/max cell / min/max/avg temp from the cell
    // and temperature matrices.
    void recompute_summaries_() noexcept;

    mutable BmsState state_ = {};
};

// Per-IC PEC error counter, exported for telemetry diagnostics
// (#72 / #75). 10 ICs == LtcChainLength.
extern "C" volatile std::uint32_t g_ltc_pec_err_count[config::LtcChainLength];

}  // namespace ams
