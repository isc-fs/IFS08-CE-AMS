// SPDX-License-Identifier: proprietary

#include "bms_service.hpp"

#include "ams_config.hpp"
#include "ltc6811.hpp"
#include "scoped_mutex.hpp"

#include "cmsis_os2.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" osMutexId_t bms_mutexHandle;  // created in main.c

namespace ams {

extern "C" volatile std::uint32_t g_ltc_pec_err_count[config::kLtcChainLength] = {};

BmsService& BmsService::instance() noexcept {
    static BmsService kInstance;
    return kInstance;
}

BmsService::BmsService() {
    // Initialise summaries to neutral values so SafetyTask reads
    // sensible numbers before the first frame lands.
    state_.min_cell_mV = std::numeric_limits<std::uint16_t>::max();
    state_.max_cell_mV = 0;
    state_.min_tempC   = std::numeric_limits<std::int16_t>::max();
    state_.max_tempC   = std::numeric_limits<std::int16_t>::min();

    // Default cell_tempC to 25 degC so unpopulated NTC slots can't
    // dominate the max/min on the first temp poll. Real readings
    // overwrite each slot once update_temperature commits a valid
    // Steinhart/Beta conversion.
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        for (std::uint8_t t = 0; t < config::kTempsPerModule; ++t) {
            state_.cell_tempC[m][t] = 25;
        }
    }
}

namespace {

// Convert one AUX1 mV reading (LTC6811 100-uV units already de-scaled
// to mV by ltc6811::decode_aux_voltage_group) into a temperature.
// Returns INT16_MIN on out-of-plausibility (open circuit, shorted,
// or computed °C outside kNtcMinValidC..kNtcMaxValidC), which the
// caller uses as a "skip this slot" sentinel.
std::int16_t ntc_mV_to_tempC(std::uint16_t v_aux_mV) noexcept {
    using namespace ams::config;

    // Rail readings -> open or shorted. Drop.
    if (v_aux_mV == 0u || v_aux_mV >= kNtcVrefMv) {
        return std::numeric_limits<std::int16_t>::min();
    }

    // R_ntc = R_series * V_aux / (V_ref - V_aux)
    const float v_aux_f = static_cast<float>(v_aux_mV);
    const float v_ref_f = static_cast<float>(kNtcVrefMv);
    const float r_ntc   = static_cast<float>(kNtcSeriesR) * v_aux_f
                          / (v_ref_f - v_aux_f);

    // Beta model: 1/T = 1/T0 + (1/B) * ln(R/R25)
    const float ratio = r_ntc / static_cast<float>(kNtcR25);
    if (!(ratio > 0.0f)) {
        return std::numeric_limits<std::int16_t>::min();
    }
    const float inv_T = (1.0f / kNtcT0Kelvin)
                       + (std::log(ratio) / static_cast<float>(kNtcBeta));
    if (!(inv_T > 0.0f)) {
        return std::numeric_limits<std::int16_t>::min();
    }
    const float t_K = 1.0f / inv_T;
    const float t_C = t_K - 273.15f;

    if (t_C < static_cast<float>(kNtcMinValidC) ||
        t_C > static_cast<float>(kNtcMaxValidC)) {
        return std::numeric_limits<std::int16_t>::min();
    }

    // Round to nearest degree.
    return static_cast<std::int16_t>(t_C + (t_C >= 0.0f ? 0.5f : -0.5f));
}

}  // namespace

void BmsService::recompute_summaries_() noexcept {
    std::uint32_t sum_v_mV = 0;
    std::uint16_t min_mV   = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_mV   = 0;
    std::int16_t  min_t    = std::numeric_limits<std::int16_t>::max();
    std::int16_t  max_t    = std::numeric_limits<std::int16_t>::min();
    std::int32_t  sum_t    = 0;
    std::uint32_t n_t      = 0;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        if ((state_.module_online_mask & (1u << m)) == 0u) continue;

        for (std::uint8_t c = 0; c < config::kCellsPerModule; ++c) {
            const std::uint16_t v = state_.cell_mV[m][c];
            sum_v_mV += v;
            if (v < min_mV) min_mV = v;
            if (v > max_mV) max_mV = v;
        }
        for (std::uint8_t t = 0; t < config::kTempsPerModule; ++t) {
            const std::int16_t tc = state_.cell_tempC[m][t];
            if (tc < min_t) min_t = tc;
            if (tc > max_t) max_t = tc;
            sum_t += tc;
            ++n_t;
        }
    }

    state_.pack_voltage_mV = sum_v_mV;
    state_.min_cell_mV     = min_mV;
    state_.max_cell_mV     = max_mV;
    state_.min_tempC       = min_t;
    state_.max_tempC       = max_t;
    state_.avg_tempC       = (n_t > 0) ? static_cast<std::int16_t>(sum_t / static_cast<std::int32_t>(n_t)) : 0;
}

// ---------------------------------------------------------------------------
// Legacy CAN entry point. The BMS bus moved to LTC6811-1 / isoSPI in
// v1.2.0 (#67-#74) so this path no longer ingests live data; it is
// retained only so BmsRxTask still compiles until #73 retires it.
// ---------------------------------------------------------------------------
bool BmsService::update_from_frame(const CanFrame& f) noexcept {
    (void)f;
    return false;
}

bool BmsService::update_from_ltc_response(const std::uint8_t* chain_response,
                                          std::size_t         len,
                                          std::uint32_t       now_tick_ms) noexcept {
    constexpr std::size_t kSeg        = 8;                              // 6 data + 2 PEC
    constexpr std::size_t kGroupBytes = config::kLtcChainLength * kSeg; // 10 * 8 = 80
    constexpr std::size_t kExpected   = 4u * kGroupBytes;               // 320

    if (chain_response == nullptr || len < kExpected) return false;

    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return false;

    // Per-IC PEC sweep first, then commit cell values for ICs that
    // passed all four groups. Doing it in two passes keeps the state
    // strictly atomic per IC -- a half-updated module is never
    // observable through snapshot().
    std::uint16_t new_ltc_online = 0u;
    std::array<std::array<std::uint16_t, 3>, 4> groups{};  // [group_idx][slot]

    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        bool ic_ok = true;
        for (std::uint8_t g = 0; g < 4; ++g) {
            const std::uint8_t* seg =
                chain_response + g * kGroupBytes + ic * kSeg;
            if (!ltc6811::decode_cell_voltage_group(seg, groups[g])) {
                ic_ok = false;
                break;
            }
        }
        if (!ic_ok) {
            g_ltc_pec_err_count[ic]++;
            continue;
        }

        // Commit. Module index and "upper vs lower" derive from the
        // chain index: even slots are LTC_1 (top of the module, 10
        // cells), odd slots are LTC_2 (bottom, 9 cells).
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::kLtcsPerModule);
        const bool         is_upper = (ic % config::kLtcsPerModule) == 0u;

        if (is_upper) {
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][0 + k] = groups[0][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][3 + k] = groups[1][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][6 + k] = groups[2][k];
            state_.cell_mV[module][9] = groups[3][0];   // LTC_1's cell 10
        } else {
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][10 + k] = groups[0][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][13 + k] = groups[1][k];
            for (std::uint8_t k = 0; k < 3; ++k) state_.cell_mV[module][16 + k] = groups[2][k];
            // RDCVD entirely discarded -- LTC_2 owns only 9 cells.
        }

        new_ltc_online = static_cast<std::uint16_t>(new_ltc_online | (1u << ic));
    }

    state_.ltc_online_mask = new_ltc_online;

    // Module is healthy this cycle iff BOTH its LTCs passed PEC.
    // module_online_mask stays sticky (once-online); freshness is the
    // dynamic gate via last_rx_tick + is_healthy.
    bool any_module_fresh = false;
    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        const std::uint16_t pair =
            static_cast<std::uint16_t>((1u << (2u * m)) | (1u << (2u * m + 1u)));
        if ((new_ltc_online & pair) == pair) {
            state_.module_online_mask = static_cast<std::uint8_t>(
                state_.module_online_mask | (1u << m));
            state_.last_rx_tick[m] = now_tick_ms;
            any_module_fresh = true;
        }
    }

    recompute_summaries_();
    return any_module_fresh;
}

bool BmsService::update_temperature(std::uint8_t        channel_idx,
                                    const std::uint8_t* chain_response,
                                    std::size_t         len) noexcept {
    constexpr std::size_t kSeg      = 8;
    constexpr std::size_t kExpected = config::kLtcChainLength * kSeg;

    if (chain_response == nullptr || len < kExpected) return false;
    if (channel_idx >= config::kTempsPerLtc)          return false;

    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return false;

    bool any_ok = false;
    std::array<std::uint16_t, 3> aux{};

    for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
        const std::uint8_t* seg = chain_response + ic * kSeg;
        if (!ltc6811::decode_aux_voltage_group(seg, aux)) {
            g_ltc_pec_err_count[ic]++;
            continue;
        }
        // AUX1 (GPIO1) carries the buffered ADG731 output. AUX2/AUX3
        // are unused on BMS_LITE -- we read them anyway because they
        // come in the same group, but discard.
        const std::int16_t t = ntc_mV_to_tempC(aux[0]);
        if (t == std::numeric_limits<std::int16_t>::min()) {
            continue;  // out of range -> keep last good value
        }
        const std::uint8_t module   = static_cast<std::uint8_t>(ic / config::kLtcsPerModule);
        const bool         is_upper = (ic % config::kLtcsPerModule) == 0u;
        const std::uint8_t slot     = is_upper
            ? channel_idx
            : static_cast<std::uint8_t>(config::kTempsPerLtc + channel_idx);
        state_.cell_tempC[module][slot] = t;
        any_ok = true;
    }

    recompute_summaries_();
    return any_ok;
}

BmsState BmsService::snapshot() const noexcept {
    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return state_;  // best-effort; caller checks freshness
    return state_;
}

bool BmsService::is_healthy(std::uint32_t now_tick) const noexcept {
    ScopedMutex lock(bms_mutexHandle);
    if (!lock.acquired()) return false;

    if (state_.module_online_mask != config::kAllModulesMask) return false;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        if (now_tick - state_.last_rx_tick[m] > config::kBmsStaleMs) return false;
    }
    return true;
}

}  // namespace ams
