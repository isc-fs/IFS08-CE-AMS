// SPDX-License-Identifier: proprietary

#include "bms_service.hpp"

#include "ntc_table.hpp"

#include "ams_config.hpp"
#include "ltc6811.hpp"
#include "open_wire.hpp"

#include <array>
#include <cstdint>
#include <limits>

// Lock-free single-writer / multi-reader contract. BmsPollTask is the only
// writer; MainTask, AcuCanTask and BalanceController are readers. Cortex-M7
// 32-bit aligned loads/stores are atomic, so a multi-field read can briefly
// observe a mid-update snapshot -- the predicates and telemetry tolerate one
// cycle of staleness. CubeMX still declares bms_mutexHandle in main.c, but no
// app code takes it.

namespace ams {

extern "C" volatile std::uint32_t g_ltc_pec_err_count[config::LtcChainLength] = {};

BmsService& BmsService::instance() noexcept {
    static BmsService Instance;
    return Instance;
}

BmsService::BmsService() {
    // Initialise summaries to neutral values so SafetyTask reads
    // sensible numbers before the first frame lands.
    state_.min_cell_mV = std::numeric_limits<std::uint16_t>::max();
    state_.max_cell_mV = 0;
    state_.min_tempC   = std::numeric_limits<std::int16_t>::max();
    state_.max_tempC   = std::numeric_limits<std::int16_t>::min();

    // Seed every cell-temp slot to the NO-READING sentinel, never to a plausible
    // temperature. A plausible seed would make an unpopulated or mis-muxed
    // channel report room temperature forever: max_tempC would look healthy
    // whatever the pack did, defeating every threshold built on it.
    // recompute_summaries_ skips sentinels and counts what remains in
    // valid_temp_channels, so "no data" stays visible.
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
            state_.cell_tempC[m][t] = config::NtcNoReading;
        }
    }
    state_.valid_temp_channels = 0;

    // Seed cell_mV to a nominal-healthy value for the same reason: before the
    // first poll -- or for a module the boot free-pass window briefly marks
    // "online" before it has reported -- zero-initialised cells would read 0 mV
    // and trip CellUnderVoltage. 3700 mV sits well inside
    // [CellUnderVoltageMv, CellOverVoltageMv], and a PEC-clean RDCV group
    // overwrites each slot. Hygiene only; first_full_poll_done below is the
    // authoritative guard.
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            state_.cell_mV[m][c] = 3700;
        }
    }

    state_.first_full_poll_done = false;
}

namespace {

// Convert one AUX1 mV reading (ltc6811::decode_aux_voltage_group has already
// de-scaled the LTC6811 100-uV units to mV) into a temperature. Returns
// INT16_MIN -- the caller's "skip this slot" sentinel -- on an open circuit, a
// short, or a result outside NtcMinValidC..NtcMaxValidC.
std::int16_t ntc_mV_to_tempC(std::uint16_t v_aux_mV) noexcept {
    using namespace ams::config;

    // Open or shorted -> drop. 0 mV is a short; NtcOpenMv (< NtcVrefMv) catches
    // a partially-railed open that would otherwise decode to a plausible but
    // bogus cold temperature. Both return the sentinel, so the caller's
    // disconnect path fires instead of a spuriously cold reading.
    if (v_aux_mV == 0u || v_aux_mV >= NtcOpenMv) {
        return std::numeric_limits<std::int16_t>::min();
    }

    // R_ntc = R_pullup * V_aux / (V_ref - V_aux). Integer maths: the divider is
    // exact and the table lookup does the rest, so no float is needed.
    const std::uint32_t v_aux = v_aux_mV;
    const std::uint32_t v_ref = NtcVrefMv;
    const std::uint32_t r_ntc =
        (static_cast<std::uint32_t>(NtcPullupOhm) * v_aux) / (v_ref - v_aux);

    // Table lookup (ntc_table.hpp) rather than a single-beta Steinhart fit:
    // beta is only accurate near its fitting interval, and this part's own
    // datasheet quotes two different betas (3950 at 25/50, 4021 at 25/85).
    std::int32_t decideg = 0;
    if (!ntc::resistance_to_decideg(r_ntc, decideg)) {
        return std::numeric_limits<std::int16_t>::min();
    }
    // Plausibility gate in tenths, so an unpopulated channel that decodes to a
    // physically impossible temperature is dropped rather than reported.
    if (decideg < static_cast<std::int32_t>(NtcMinValidC) * 10 ||
        decideg > static_cast<std::int32_t>(NtcMaxValidC) * 10) {
        return std::numeric_limits<std::int16_t>::min();
    }

    // Round to nearest degree, away from zero.
    const std::int32_t rounded = (decideg + (decideg >= 0 ? 5 : -5)) / 10;
    return static_cast<std::int16_t>(rounded);
}

}  // namespace

void BmsService::recompute_summaries_() noexcept {
    state_.temp_disconnect_mask = 0;
    state_.tap_fault_mask       = 0;
    std::uint32_t sum_v_mV = 0;
    std::uint16_t min_mV   = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_mV   = 0;
    std::int16_t  min_t    = std::numeric_limits<std::int16_t>::max();
    std::int16_t  max_t    = std::numeric_limits<std::int16_t>::min();
    std::int32_t  sum_t    = 0;
    std::uint32_t n_t      = 0;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        // Default the per-module aggregates to safe sentinels even when the
        // module has not reported: the 0x131..0x134 / 0x136..0x137 TX in
        // acu_can_task.cpp reads them unconditionally every cycle.
        std::uint16_t mod_min_v = std::numeric_limits<std::uint16_t>::max();
        std::uint16_t mod_max_v = 0;
        std::int16_t  mod_max_t = std::numeric_limits<std::int16_t>::min();

        if ((state_.module_online_mask & (1u << m)) != 0u) {
            // Tap-artifact guard. Balancing current through a high-resistance or
            // open cell tap shifts the node SHARED by two physically-adjacent
            // cells: one reads non-physical (> CellImplausibleMaxMv or <
            // CellImplausibleMinMv), the neighbour compensates the opposite way,
            // and their PAIR SUM stays in the normal window because it is immune
            // to a shared-tap error. Aggregate such a pair on that tap-immune
            // AVERAGE instead of the impossible individual reading, so a bad tap
            // cannot false-trip Cell Over/UnderVoltage and open the SDC. cell_mV
            // itself is untouched -- the pit-diag grid still shows the raw split.
            // A GENUINE over/under-voltage has a NORMAL neighbour, so the sum
            // leaves the window, is NOT masked, and still faults.
            std::uint16_t agg_v[config::CellsPerModule];
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
                agg_v[c] = state_.cell_mV[m][c];
            }
            for (std::uint8_t c = 0; c + 1u < config::CellsPerModule; ++c) {
                // Physically adjacent == consecutive indices in the SAME LTC half
                // (balance::physically_adjacent). The one in-range boundary that
                // is NOT adjacent is the LTC split at CellsPerLtcUpper.
                if (c + 1u == config::CellsPerLtcUpper) continue;
                const std::uint16_t va = state_.cell_mV[m][c];
                const std::uint16_t vb = state_.cell_mV[m][c + 1u];
                const std::uint16_t hi = va > vb ? va : vb;
                const std::uint16_t lo = va < vb ? va : vb;
                const std::uint32_t sum =
                    static_cast<std::uint32_t>(va) + static_cast<std::uint32_t>(vb);
                const bool nonphysical = hi > config::CellImplausibleMaxMv ||
                                         lo < config::CellImplausibleMinMv;
                const bool wide_split =
                    static_cast<std::uint16_t>(hi - lo) > config::TapArtifactMinSplitMv;
                const bool sum_in_window =
                    sum >= 2u * static_cast<std::uint32_t>(config::CellUnderVoltageMv) &&
                    sum <= 2u * static_cast<std::uint32_t>(config::CellOverVoltageMv);
                if (nonphysical && wide_split && sum_in_window) {
                    const std::uint16_t avg = static_cast<std::uint16_t>(sum / 2u);
                    agg_v[c]      = avg;
                    agg_v[c + 1u] = avg;
                    state_.tap_fault_mask =
                        static_cast<std::uint8_t>(state_.tap_fault_mask | (1u << m));
                }
            }
            for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
                const std::uint16_t v = agg_v[c];
                sum_v_mV += v;
                if (v < min_mV)    min_mV    = v;
                if (v > max_mV)    max_mV    = v;
                if (v < mod_min_v) mod_min_v = v;
                if (v > mod_max_v) mod_max_v = v;
            }
            std::uint8_t mod_valid_temps = 0;
            for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
                const std::int16_t tc = state_.cell_tempC[m][t];
                if (tc == config::NtcNoReading) {
                    // Sentinel on a channel that HAS read valid = disconnected
                    // (open past the debounce). An unpopulated channel also sits
                    // at the sentinel but was never seen, so it is no disconnect.
                    if (seen_valid_[m][t]) {
                        state_.temp_disconnect_mask =
                            static_cast<std::uint8_t>(state_.temp_disconnect_mask | (1u << m));
                    }
                    continue;   // NOT data -- excluded from min/max/avg
                }
                if (tc < min_t)     min_t     = tc;
                if (tc > max_t)     max_t     = tc;
                if (tc > mod_max_t) mod_max_t = tc;
                sum_t += tc;
                ++n_t;
                ++mod_valid_temps;
            }

            // REQUIRED-channel presence (config::RequiredTempSlots): a declared
            // slot reading open faults whatever its seen-valid history, so an
            // open-at-boot channel is caught (scrutineering determinism). Gated
            // on the module having produced >= 1 valid temp this poll, so the
            // boot window -- every slot still at the seed sentinel, none polled
            // yet -- cannot false-fault a channel before it has been read.
            if (mod_valid_temps > 0) for (std::uint8_t rs : config::RequiredTempSlots) {
                if (rs < config::TempsPerModule &&
                    state_.cell_tempC[m][rs] == static_cast<std::int16_t>(config::NtcNoReading)) {
                    state_.temp_disconnect_mask =
                        static_cast<std::uint8_t>(state_.temp_disconnect_mask | (1u << m));
                }
            }
        }

        // An offline module keeps the sentinels: vmin = 0xFFFF, vmax = 0,
        // tmax = INT16_MIN. The ECU flags those as "no data" on its side.
        state_.vmin_module[m] = (mod_min_v == std::numeric_limits<std::uint16_t>::max())
                                ? std::uint16_t{0xFFFFu} : mod_min_v;
        state_.vmax_module[m] = mod_max_v;  // 0 if offline
        state_.tmax_module[m] = mod_max_t;  // INT16_MIN if offline
    }

    state_.pack_voltage_mV = sum_v_mV;
    state_.min_cell_mV     = min_mV;
    state_.max_cell_mV     = max_mV;
    state_.min_tempC       = min_t;
    state_.max_tempC       = max_t;
    state_.avg_tempC       = (n_t > 0) ? static_cast<std::int16_t>(sum_t / static_cast<std::int32_t>(n_t)) : 0;
    // Count of channels that actually produced data. At 0, min/max_tempC are the
    // untouched sentinels (INT16_MAX / INT16_MIN) and callers must read them as
    // "unknown" -- notably balancing, whose only thermal guard is max_tempC and
    // which would otherwise take INT16_MIN for "nice and cool".
    state_.valid_temp_channels = static_cast<std::uint16_t>(n_t);
}

bool BmsService::update_from_ltc_response(const std::uint8_t* chain_response,
                                          std::size_t         len,
                                          std::uint32_t       now_tick_ms) noexcept {
    constexpr std::size_t Seg        = 8;                              // 6 data + 2 PEC
    constexpr std::size_t GroupBytes = config::LtcChainLength * Seg; // 10 * 8 = 80
    constexpr std::size_t Expected   = 4u * GroupBytes;               // 320

    if (chain_response == nullptr || len < Expected) return false;


    // PEC-check an IC's four groups before committing any of its cells. That
    // keeps the commit atomic per IC, so a half-updated IC is never observable
    // through snapshot().
    std::uint16_t new_ltc_online = 0u;
    std::array<std::array<std::uint16_t, 3>, 4> groups{};  // [group_idx][slot]

    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        bool ic_ok = true;
        for (std::uint8_t g = 0; g < 4; ++g) {
            const std::uint8_t* seg =
                chain_response + g * GroupBytes + ic * Seg;
            if (!ltc6811::decode_cell_voltage_group(seg, groups[g])) {
                ic_ok = false;
                break;
            }
        }
        if (!ic_ok) {
            g_ltc_pec_err_count[ic]++;
            continue;
        }

        // Commit. Module index and "upper vs lower" derive from the chain index:
        // even slots are LTC_1 (FIRST in chain, 9 cells -> module cells 0..8),
        // odd slots are LTC_2 (SECOND, 10 cells -> module cells 9..18). The
        // 10-cell LTC's tenth cell is RDCVD slot 0 (its C10); the 9-cell LTC's
        // RDCVD group is discarded. Swapping the two zeroes module cell 9 and
        // drops cell 18.
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::LtcsPerModule);
        const bool         is_upper = (ic % config::LtcsPerModule) == 0u;

        if (is_upper) {          // first LTC: 9 cells -> module 0..8
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][0 + k] = groups[0][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][3 + k] = groups[1][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][6 + k] = groups[2][k];
            // RDCVD (groups[3]) discarded -- the first LTC owns only 9 cells.
        } else {                 // second LTC: 10 cells -> module 9..18
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][9 + k]  = groups[0][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][12 + k] = groups[1][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][15 + k] = groups[2][k];
            state_.cell_mV[module][18] = groups[3][0];   // LTC_2's cell 10 (RDCVD C10)
        }

        new_ltc_online = static_cast<std::uint16_t>(new_ltc_online | (1u << ic));
    }

    state_.ltc_online_mask = new_ltc_online;

    // Advance last_rx_tick for every module whose BOTH LTCs passed PEC this
    // cycle. module_online_mask is then re-derived from that freshness (below),
    // so it means "currently responding", not "ever responded": a bit drops on
    // its own once now_tick_ms - last_rx_tick[m] > BmsStaleMs.
    bool any_module_fresh = false;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        const std::uint16_t pair =
            static_cast<std::uint16_t>((1u << (2u * m)) | (1u << (2u * m + 1u)));
        if ((new_ltc_online & pair) == pair) {
            state_.last_rx_tick[m] = now_tick_ms;
            any_module_fresh = true;
        }
    }

    // Derive the mask from last_rx_tick freshness. A module that has never
    // reported (last_rx_tick == 0) looks fresh while now_tick_ms <= BmsStaleMs,
    // then drops out. That boot transient is harmless: SafetyTask suppresses
    // data-presence predicates for SafetyBootGraceMs (2000 ms), which covers the
    // whole window (BmsStaleMs = 350 ms).
    std::uint8_t fresh_mask = 0;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((now_tick_ms - state_.last_rx_tick[m]) <= config::BmsStaleMs) {
            fresh_mask = static_cast<std::uint8_t>(fresh_mask | (1u << m));
        }
    }
    state_.module_online_mask = fresh_mask;

    // Arm the cell V/T range predicates only once EVERY module has reported at
    // least once. last_rx_tick[m] is set above solely on a both-LTCs-PEC-clean
    // poll, so a non-zero tick for every module means every cell holds real
    // data. Until then cells may still hold the 3700 mV boot seed or a
    // partially-populated mix, which must not trip CellUnderVoltage.
    if (!state_.first_full_poll_done) {
        bool all_reported = true;
        for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
            if (state_.last_rx_tick[m] == 0u) { all_reported = false; break; }
        }
        if (all_reported) state_.first_full_poll_done = true;
    }

    recompute_summaries_();
    return any_module_fresh;
}

bool BmsService::update_temperature(std::uint8_t        channel_idx,
                                    const std::uint8_t* chain_response,
                                    std::size_t         len) noexcept {
    constexpr std::size_t Seg      = 8;
    constexpr std::size_t Expected = config::LtcChainLength * Seg;

    if (chain_response == nullptr || len < Expected) return false;
    if (channel_idx >= config::TempsPerLtc)          return false;


    bool any_ok = false;
    std::array<std::uint16_t, 3> aux{};

    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const std::uint8_t* seg = chain_response + ic * Seg;
        if (!ltc6811::decode_aux_voltage_group(seg, aux)) {
            g_ltc_pec_err_count[ic]++;
            continue;
        }
        // AUX1 (GPIO1) carries the buffered ADG731 output. AUX2/AUX3 are unused
        // on BMS_LITE; they arrive in the same group and are discarded.
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::LtcsPerModule);
        const bool         is_upper = (ic % config::LtcsPerModule) == 0u;
        const std::uint8_t slot     = is_upper
            ? channel_idx
            : static_cast<std::uint8_t>(config::TempsPerLtc + channel_idx);

        const std::int16_t t = ntc_mV_to_tempC(aux[0]);
        if (t == std::numeric_limits<std::int16_t>::min()) {
            // OPEN reading (rail voltage). A channel that was never valid is
            // simply unpopulated: leave it at the seed sentinel, no fault. A
            // channel that HAS read valid and now reads open is a candidate
            // DISCONNECT -- keep its last good value while the open run is short
            // (one anomalous mux read is tolerated), and mark it NtcNoReading,
            // which makes the summary flag the module, only once the run reaches
            // TempDisconnectPolls. See config::TempSensorPresenceCheck.
            if (seen_valid_[module][slot]) {
                if (open_run_[module][slot] < 0xFFu) ++open_run_[module][slot];
                if (open_run_[module][slot] >= config::TempDisconnectPolls) {
                    state_.cell_tempC[module][slot] =
                        static_cast<std::int16_t>(config::NtcNoReading);
                }
            }
            continue;  // do not overwrite with a temperature
        }

        // Valid reading: clear the open run, latch that this channel exists.
        open_run_[module][slot]   = 0;
        seen_valid_[module][slot] = true;
        state_.cell_tempC[module][slot] = t;
        any_ok = true;
    }

    recompute_summaries_();
    return any_ok;
}

bool BmsService::update_open_wire(const std::uint8_t* pu_reply,
                                 const std::uint8_t* pd_reply,
                                 std::size_t         len, bool accumulate) noexcept {
    // Gated off -> report "no open" so a stale mask cannot linger, and report
    // fully evaluated so the caller does not retry.
    if (!config::CellOpenWireCheck) {
        state_.cell_open_mask = 0u;
        for (auto& v : state_.cell_open_cells) v = 0u;
        return true;
    }

    constexpr std::size_t Seg        = 8;                                // 6 data + 2 PEC
    constexpr std::size_t GroupBytes = config::LtcChainLength * Seg;     // 80
    constexpr std::size_t Expected   = 4u * GroupBytes;                  // 320
    if (pu_reply == nullptr || pd_reply == nullptr || len < Expected) return true;

    std::uint8_t  new_mask = 0u;
    std::uint32_t new_cells[config::BmsModuleCount] = {};
    bool          all_ok   = true;   // false if any IC was PEC-skipped this call
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::LtcsPerModule);
        const bool         is_upper = (ic % config::LtcsPerModule) == 0u;
        // Per-LTC cell count: 9 on the upper (RDCVD discarded), 10 on the lower
        // (RDCVD C10 is the 10th cell). NEVER a uniform count.
        const std::uint8_t n_cells  =
            is_upper ? config::CellsPerLtcUpper : config::CellsPerLtcLower;

        std::uint16_t pu_cells[config::CellsPerLtcLower] = {};   // max 10
        std::uint16_t pd_cells[config::CellsPerLtcLower] = {};
        bool ic_ok = true;
        for (std::uint8_t g = 0; g < 4 && ic_ok; ++g) {
            std::array<std::uint16_t, 3> gpu{};
            std::array<std::uint16_t, 3> gpd{};
            const std::uint8_t* spu = pu_reply + g * GroupBytes + ic * Seg;
            const std::uint8_t* spd = pd_reply + g * GroupBytes + ic * Seg;
            // The delta is only trustworthy if both passes PEC-clean, so skip the
            // IC otherwise; its loss is caught by the module-online/stale path.
            if (!ltc6811::decode_cell_voltage_group(spu, gpu) ||
                !ltc6811::decode_cell_voltage_group(spd, gpd)) {
                ic_ok = false;
                break;
            }
            for (std::uint8_t k = 0; k < 3; ++k) {
                const std::uint8_t idx = static_cast<std::uint8_t>(g * 3 + k);
                if (idx < n_cells) {   // upper: keep A/B/C (0..8); lower: +RDCVD[0]=idx 9
                    pu_cells[idx] = gpu[k];
                    pd_cells[idx] = gpd[k];
                }
            }
        }
        if (!ic_ok) { all_ok = false; continue; }   // PEC glitch -> caller retries
        const std::uint16_t conductors = open_wire::detect_open_conductors(
            pu_cells, pd_cells, n_cells, config::CellOpenWireDeltaMv);
        if (conductors != 0u) {
            new_mask = static_cast<std::uint8_t>(new_mask | (1u << module));
            // Conductor -> cell mapping. Conductor k is the NODE between cells
            // k-1 and k (IC-local, 0-indexed), so an open there corrupts both;
            // the endpoints C(0) and C(N) border one cell each. Offset by the
            // IC's base so upper-LTC cells land on module cells 0..8 and
            // lower-LTC cells on 9..18.
            const std::uint8_t base = is_upper ? 0u : config::CellsPerLtcUpper;
            for (std::uint8_t k = 0; k <= n_cells; ++k) {
                if ((conductors & (1u << k)) == 0u) continue;
                if (k > 0u) {
                    new_cells[module] |= (1u << (base + k - 1u));
                }
                if (k < n_cells) {
                    new_cells[module] |= (1u << (base + k));
                }
            }
        }
    }
    // First attempt overwrites (clears last poll's stale bits); a retry ORs so it
    // can't erase an open a prior attempt of THIS poll already confirmed.
    state_.cell_open_mask = accumulate
        ? static_cast<std::uint8_t>(state_.cell_open_mask | new_mask)
        : new_mask;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        state_.cell_open_cells[m] = accumulate
            ? (state_.cell_open_cells[m] | new_cells[m])
            : new_cells[m];
    }
    return all_ok;
}

void BmsService::capture_cell_raw(const std::uint8_t* reply,
                                  std::size_t         len,
                                  std::uint16_t*      out) noexcept {
    constexpr std::size_t Seg        = 8;
    constexpr std::size_t GroupBytes = config::LtcChainLength * Seg;
    constexpr std::size_t Expected   = 4u * GroupBytes;
    constexpr std::size_t N = static_cast<std::size_t>(config::BmsModuleCount) *
                              config::CellsPerModule;   // 95
    if (out == nullptr) return;
    for (std::size_t i = 0; i < N; ++i) out[i] = 0xFFFFu;
    if (reply == nullptr || len < Expected) return;

    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::LtcsPerModule);
        const bool         is_upper = (ic % config::LtcsPerModule) == 0u;
        // Module-cell offset + count: upper LTC = cells 0..8 (9), lower = 9..18 (10).
        const std::uint8_t base    = is_upper ? 0u : config::CellsPerLtcUpper;
        const std::uint8_t n_cells = is_upper ? config::CellsPerLtcUpper
                                              : config::CellsPerLtcLower;
        for (std::uint8_t g = 0; g < 4; ++g) {
            std::array<std::uint16_t, 3> grp{};
            const std::uint8_t* seg = reply + g * GroupBytes + ic * Seg;
            if (!ltc6811::decode_cell_voltage_group(seg, grp)) continue;  // leave sentinel
            for (std::uint8_t k = 0; k < 3; ++k) {
                const std::uint8_t idx = static_cast<std::uint8_t>(g * 3 + k);  // LTC-local
                if (idx < n_cells) {
                    const std::size_t flat =
                        static_cast<std::size_t>(module) * config::CellsPerModule + base + idx;
                    out[flat] = grp[k];
                }
            }
        }
    }
}

void BmsService::capture_adow_raw(const std::uint8_t* pu_reply,
                                  const std::uint8_t* pd_reply,
                                  std::size_t         len,
                                  std::uint16_t*      pu_out,
                                  std::uint16_t*      pd_out) noexcept {
    if (pu_out == nullptr || pd_out == nullptr) return;
    capture_cell_raw(pu_reply, len, pu_out);
    capture_cell_raw(pd_reply, len, pd_out);

    // Report a cell only if BOTH passes were PEC-clean. The diagnostic IS the
    // PU-vs-PD delta, so a cell that survived one pass has no delta to read, and
    // publishing the surviving half would invite a comparison against the 0xFFFF
    // sentinel on the other side.
    constexpr std::size_t N = static_cast<std::size_t>(config::BmsModuleCount) *
                              config::CellsPerModule;
    for (std::size_t i = 0; i < N; ++i) {
        if (pu_out[i] == 0xFFFFu || pd_out[i] == 0xFFFFu) {
            pu_out[i] = 0xFFFFu;
            pd_out[i] = 0xFFFFu;
        }
    }
}

BmsState BmsService::snapshot() const noexcept {
    return state_;
}

bool BmsService::is_healthy(std::uint32_t now_tick) const noexcept {

    if (state_.module_online_mask != config::AllModulesMask) return false;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if (now_tick - state_.last_rx_tick[m] > config::BmsStaleMs) return false;
    }
    return true;
}

}  // namespace ams
