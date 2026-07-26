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
    // Cell-temp channels that produced a VALID conversion this poll, across
    // online modules. Unpopulated / open / shorted / PEC-failed channels do
    // NOT count. Consumers that gate on temperature must check this: with a
    // count of 0, min/max_tempC are sentinels and mean "no thermal data", not
    // "cold". See NtcNoReading.
    std::uint16_t valid_temp_channels;
    std::int16_t  avg_tempC;

    // Bit m set <=> online module m has >=1 cell-temp channel that read VALID
    // at least once and is now OPEN (disconnected) past the debounce. Drives
    // the TempSensorDisconnected safety fault -- a disconnected sensor opens the
    // SDC (FS rule). Independent of temperature accuracy: an open NTC reads the
    // rail regardless of calibration. See config::TempSensorPresenceCheck.
    std::uint8_t  temp_disconnect_mask;

    // Bit m set <=> online module m has a physically-adjacent cell pair whose
    // shared tap node is displaced (one cell reads non-physical, the neighbour
    // compensates, pair-sum conserved) -- a high-resistance/open cell tap
    // exposed by balancing current. For those pairs recompute_summaries_ feeds
    // the tap-immune pair AVERAGE to min/max_cell_mV + vmin/vmax_module so the
    // artifact cannot false-trip Cell Over/UnderVoltage and open the SDC. The
    // raw per-cell cell_mV is left untouched (pit-diag grid still shows the
    // split for diagnosis). See config::CellImplausibleMaxMv / MinMv.
    std::uint8_t  tap_fault_mask;

    // Bit m set <=> online module m has at least one OPEN cell-sense wire, as
    // found by the LTC6811 ADOW two-pass open-wire check (open_wire.hpp) in
    // BmsPollTask. Distinct from tap_fault_mask (which MASKS a high-R tap so it
    // cannot false-trip OV/UV): an ADOW-confirmed open is a real fault and
    // latches FaultReason::CellOpenWire in any state. Only written when
    // config::CellOpenWireCheck is enabled; 0 otherwise.
    std::uint8_t  cell_open_mask;

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

    // Bit N set <=> module N's last PEC-clean response is within
    // BmsStaleMs of `now_tick`. Re-derived on every
    // update_from_ltc_response from last_rx_tick freshness (see #249).
    // Compared against config::AllModulesMask (0x1F) by SafetyTask;
    // also surfaced directly in 0x4A0[2] telemetry. A module that has
    // never reported (last_rx_tick == 0) stays masked off forever.
    std::uint8_t  module_online_mask;

    // 10-bit "this poll's per-IC PEC-OK" mask (LSB = chain index 0).
    // Source of truth from which module_online_mask is derived. Not
    // sticky -- reflects the most recent update_from_ltc_response.
    std::uint16_t ltc_online_mask;

    // Sticky: set true once EVERY module has reported PEC-clean (both
    // its LTCs) at least once, i.e. all last_rx_tick[m] != 0. Gates the
    // cell V/T range predicates so a partially-populated BmsState at the
    // boot-grace edge can't trip CellUnderVoltage on cells that haven't
    // been read yet (#279). Never cleared -- once the whole pack has
    // been seen, the range checks stay armed. A module that goes silent
    // afterwards is caught by the freshness / module_online_mask path,
    // not this flag.
    bool          first_full_poll_done;
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
    //   LTC_1 (chain index 2N, "upper", FIRST -- 9 cells)  [#423]
    //     RDCVA -> module cells 0,1,2
    //     RDCVB -> module cells 3,4,5
    //     RDCVC -> module cells 6,7,8
    //     RDCVD -> discarded
    //
    //   LTC_2 (chain index 2N+1, "lower", SECOND -- 10 cells)  [#423]
    //     RDCVA -> module cells 9,10,11
    //     RDCVB -> module cells 12,13,14
    //     RDCVC -> module cells 15,16,17
    //     RDCVD -> module cell 18 (slots 1,2 of group D unused)
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

    // ------------------------------------------------------------------
    // Cell open-wire (LTC6811 ADOW). BmsPollTask runs the two conversion
    // passes and hands the two RDCV chain responses here: `pu_reply` from the
    // PUP=1 pass, `pd_reply` from PUP=0 (each 4 groups x LtcChainLength x 8 B).
    // Per IC, decodes the valid cells (9 upper / 10 lower), runs
    // open_wire::detect_open_conductors, and sets state_.cell_open_mask bit m
    // if module m's upper OR lower LTC shows an open. Only ICs PEC-clean on
    // BOTH passes are evaluated (a PEC-fail is the offline/stale path's job).
    // Recomputed from scratch each call. Single-writer (BmsPollTask). No-op
    // (clears the mask) when config::CellOpenWireCheck is off. Pure decode +
    // open_wire.hpp logic -> host-testable without a chain.
    // Returns true iff EVERY IC was PEC-clean on both passes (i.e. the whole
    // chain's open-wire state was judged this call). False means at least one IC
    // was skipped (PEC glitch on a pass) and its bit in cell_open_mask may be a
    // stale/absent verdict -- the caller (BmsPollTask) retries the two-pass scan
    // up to config::OpenWireRetries times so a transient glitch doesn't slip the
    // fault to the next poll.
    // `accumulate`: false (first attempt) overwrites cell_open_mask -- clearing
    // any stale bit from the previous poll; true (a retry) ORs the new result in
    // so a later attempt whose glitch SKIPS an IC can never erase an open a prior
    // attempt already confirmed this poll. OR is fail-safe (an open bit only ever
    // persists one extra poll, re-judged fresh next cycle).
    [[nodiscard]] bool update_open_wire(const std::uint8_t* pu_reply,
                                        const std::uint8_t* pd_reply,
                                        std::size_t         len,
                                        bool                accumulate = false) noexcept;

    // BENCH DIAGNOSTIC (config::AdowRawDiag): decode both ADOW passes into flat
    // 95-cell mV grids (module m, cell c -> index m*CellsPerModule + c; upper LTC
    // -> cells 0..8, lower -> 9..18) for a raw pit-diag dump so the ADOW encoding
    // / timing can be debugged on a real chain. PEC-skipped cells stay 0xFFFF.
    // Static + does NOT touch cell_open_mask -- pure raw dump, independent of the
    // live detector; usable while CellOpenWireCheck is off.
    static void capture_adow_raw(const std::uint8_t* pu_reply,
                                 const std::uint8_t* pd_reply,
                                 std::size_t         len,
                                 std::uint16_t*      pu_out,
                                 std::uint16_t*      pd_out) noexcept;

    // Atomic read of the full state. Caller gets its own copy; the
    // mutex is released before this returns.
    [[nodiscard]] BmsState snapshot() const noexcept;

    // True iff all 5 modules have reported within BmsStaleMs and
    // module_online_mask covers them. Used by SafetyTask.
    [[nodiscard]] bool is_healthy(std::uint32_t now_tick) const noexcept;

    // Bitmask of chain-index ICs that were PEC-clean on the most recent
    // update_from_ltc_response (bit i = IC i clean). Single-writer
    // (BmsPollTask); read from that same task right after the update, so the
    // voltage-poll retry can tell a fully-clean read (all LtcChainLength bits
    // set) from a partial one without copying the whole snapshot.
    [[nodiscard]] std::uint16_t ltc_online_mask() const noexcept {
        return state_.ltc_online_mask;
    }

private:
    BmsService();

    // Internal helper; assumes the mutex is already held. Recomputes
    // pack_voltage_mV / min/max cell / min/max/avg temp from the cell
    // and temperature matrices.
    void recompute_summaries_() noexcept;

    mutable BmsState state_ = {};

    // Per cell-temp channel disconnect tracking (not part of the snapshot).
    // seen_valid_ latches once a channel has produced a real reading, so an
    // unpopulated channel (never valid) is never mistaken for a disconnect.
    // open_run_ counts consecutive OPEN polls on a seen channel; at
    // TempDisconnectPolls the channel is marked NtcNoReading and its module bit
    // set in state_.temp_disconnect_mask. Single-writer (BmsPollTask) like the
    // rest of the service.
    bool          seen_valid_[config::BmsModuleCount][config::TempsPerModule] = {};
    std::uint8_t  open_run_ [config::BmsModuleCount][config::TempsPerModule] = {};
};

// Per-IC PEC error counter, exported for telemetry diagnostics
// (#72 / #75). 10 ICs == LtcChainLength.
extern "C" volatile std::uint32_t g_ltc_pec_err_count[config::LtcChainLength];

}  // namespace ams
