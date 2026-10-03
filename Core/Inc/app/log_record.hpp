// SPDX-License-Identifier: proprietary
//
// LogRecord -- one datalogging sample: a flat snapshot of AMS state that
// SafetyTask captures every LogSamplePeriodMs and hands to SdLoggerTask via
// the lock-free LogRing. Carries the FULL per-cell voltage (cell_mV[5][19],
// 95 cells) and per-thermistor temperature (cell_tempC[5][40], 200 temps)
// matrices -- not just the pack summary -- so the on-card CSV is a complete
// per-cell record for SoC modelling and post-analysis. Pure data + pure CSV
// formatting (HAL-free, RTOS-free) -> lives in the host-testable core.
//
// On-card format is CSV (build_header / format_row). The MingoCAN log-pull
// service treats files as opaque bytes + filename + CRC, so this
// column layout is the AMS team's to evolve without touching the protocol.
//
// Column layout, in order:
//   1. the HEAD scalar columns (AMS_LOG_HEAD_COLUMNS)
//   2. c<m>_<n>: the 95 cell voltages, mV
//   3. t<m>_<n>: the 200 thermistor temperatures, whole degC
//   4. the TAIL scalar columns (AMS_LOG_TAIL_COLUMNS)
// New columns are only ever appended to the TAIL, so every existing column
// keeps its position and name for readers that index either way.
//
// An EMPTY field means "no valid value": BMS-derived fields before the first
// full BMS poll (bms_valid = 0), SoC fields while the estimator has none.
// 0 is never used for that, because 0 is a real reading.

#pragma once

#include "ams_config.hpp"
#include "soc_estimator.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace ams {

struct LogRecord {
    std::uint32_t tick_ms;             // osKernelGetTickCount() at capture
    std::uint32_t pack_mV;             // BmsState.pack_voltage_mV
    std::int32_t  pack_current_raw_mA; // CurrentState.raw_mA: mean of the last 50 ms (+ = discharge)
    std::int32_t  pack_current_mA;     // CurrentState.filtered_mA
    std::uint16_t min_cell_mV;         // BmsState.min_cell_mV (tap-compensated summary)
    std::uint16_t max_cell_mV;         // BmsState.max_cell_mV
    std::uint16_t dc_bus_V;            // VehicleState.dc_bus_V: LAST received 0x100 (see dcbus_age_ms)
    std::int16_t  min_tempC;           // BmsState.min_tempC
    std::int16_t  max_tempC;           // BmsState.max_tempC
    std::int16_t  avg_tempC;           // BmsState.avg_tempC
    std::uint8_t  fsm_state;           // g_state_telemetry
    std::uint8_t  mode;                // g_mode_locked_telemetry (0=Undec,1=Car,2=Chg)
    std::uint8_t  fault_reason;        // g_fault_reason_telemetry
    std::uint8_t  fault_detail;        // g_fault_detail_telemetry
    std::uint8_t  module_online_mask;  // BmsState.module_online_mask
    std::uint8_t  ams_ok;              // AMS_OK pin (PB4) readback
    std::uint8_t  tsms;                // TSMS pin (PF9) readback
    std::uint8_t  dash_chg;            // DASH_CHG pin (PF10) readback

    // Full matrices, copied straight from BmsState. [module][index].
    std::uint16_t cell_mV   [config::BmsModuleCount][config::CellsPerModule]; // 5 x 19 = 95
    std::int16_t  cell_tempC[config::BmsModuleCount][config::TempsPerModule]; // 5 x 40 = 200

    // --- TAIL columns. All captured by SafetyTask in the same tick as the rest
    // of the row, so every value in a row describes one instant. ---

    // Balancing (balance::Controller status), from one atomic 32-bit word so the
    // three always come from the same balance window.
    std::uint8_t  bal_state;           // balance::State
    std::uint16_t bal_inhibit;         // balance::inhibit bits
    std::uint8_t  bal_active;          // cells discharging, whole pack

    // 1 once every BMS module has reported (BmsState::first_full_poll_done).
    // Until then the cells hold the boot seed, so the cell, temperature and BMS
    // summary columns are written EMPTY and the row still records state,
    // faults and current.
    std::uint8_t  bms_valid;
    // How old this row's cell voltages are: now minus the newest module poll,
    // saturating at 65535 (also 65535 before any module has reported). Rows
    // (LogSamplePeriodMs) and voltage polls (BmsPollVoltMs) do not line up.
    std::uint16_t bms_age_ms;

    // State of charge at full resolution (KalmanSoc), TELEMETRY ONLY.
    std::uint32_t soc_ppm;             // 0..1000000; empty while !Valid
    std::uint32_t soc_sig_ppm;         // 1-sigma of the estimate, ppm; empty while !Valid
    std::uint8_t  soc_flags;           // soc::flags bits
    std::uint8_t  soc_seeds;           // wrapping count of estimator seeds

    // Monotonic charge totals (soc::ChargeTally), mA*s, wrapping at 2^32.
    std::uint32_t q_dis_mAs;
    std::uint32_t q_chg_mAs;
    std::uint16_t q_gaps;              // wrapping count of intervals not integrated

    // VCU / charger link state behind dc_bus_V and the re-arm decisions.
    std::uint16_t dcbus_age_ms;        // since the last 0x100, saturating; 65535 = never
    std::uint8_t  veh_flags;           // b0 dc_bus_valid, b1 discharge_engaged, b2 ecu_discharge_capable
    std::uint16_t chg_age_ms;          // since the last 0x101, saturating; 65535 = never

    // isoSPI health counters (running totals since boot).
    std::uint32_t pec_err;             // sum of per-IC PEC errors
    std::uint32_t spi_err;             // bus-level SPI failures
    std::uint32_t chain_rec;           // chain re-wake/recovery count
};

namespace log_csv {

[[nodiscard]] inline bool soc_valid(const LogRecord& r) noexcept {
    return (r.soc_flags & soc::flags::Valid) != 0u;
}

// ---------------------------------------------------------------------------
// The column tables. Each column is declared ONCE, as
//     X(header_name, U|S, value_expression, present_expression)
// and both build_header() and format_row() expand the same table, so the
// header and the rows cannot drift apart. U formats unsigned (%lu), S signed
// (%ld); present_expression false writes an empty field. `r` is the LogRecord.
// ---------------------------------------------------------------------------
#define AMS_LOG_HEAD_COLUMNS(X)                                   \
    X(tick_ms,   U, r.tick_ms,             true)                  \
    X(fsm,       U, r.fsm_state,           true)                  \
    X(mode,      U, r.mode,                true)                  \
    X(ams_ok,    U, r.ams_ok,              true)                  \
    X(fault,     U, r.fault_reason,        true)                  \
    X(detail,    U, r.fault_detail,        true)                  \
    X(tsms,      U, r.tsms,                true)                  \
    X(dash_chg,  U, r.dash_chg,            true)                  \
    X(mod_mask,  U, r.module_online_mask,  true)                  \
    X(pack_mV,   U, r.pack_mV,             r.bms_valid != 0u)     \
    X(I_raw_mA,  S, r.pack_current_raw_mA, true)                  \
    X(I_filt_mA, S, r.pack_current_mA,     true)                  \
    X(dcbus_V,   U, r.dc_bus_V,            true)                  \
    X(vmin_mV,   U, r.min_cell_mV,         r.bms_valid != 0u)     \
    X(vmax_mV,   U, r.max_cell_mV,         r.bms_valid != 0u)     \
    X(tmin_C,    S, r.min_tempC,           r.bms_valid != 0u)     \
    X(tmax_C,    S, r.max_tempC,           r.bms_valid != 0u)     \
    X(tavg_C,    S, r.avg_tempC,           r.bms_valid != 0u)

#define AMS_LOG_TAIL_COLUMNS(X)                                   \
    X(bal_state,    U, r.bal_state,    true)                      \
    X(bal_inhibit,  U, r.bal_inhibit,  true)                      \
    X(bal_active,   U, r.bal_active,   true)                      \
    X(bms_valid,    U, r.bms_valid,    true)                      \
    X(bms_age_ms,   U, r.bms_age_ms,   true)                      \
    X(soc_ppm,      U, r.soc_ppm,      soc_valid(r))              \
    X(soc_sig_ppm,  U, r.soc_sig_ppm,  soc_valid(r))              \
    X(soc_flags,    U, r.soc_flags,    true)                      \
    X(soc_seeds,    U, r.soc_seeds,    true)                      \
    X(q_dis_mAs,    U, r.q_dis_mAs,    true)                      \
    X(q_chg_mAs,    U, r.q_chg_mAs,    true)                      \
    X(q_gaps,       U, r.q_gaps,       true)                      \
    X(dcbus_age_ms, U, r.dcbus_age_ms, true)                      \
    X(veh_flags,    U, r.veh_flags,    true)                      \
    X(chg_age_ms,   U, r.chg_age_ms,   true)                      \
    X(pec_err,      U, r.pec_err,      true)                      \
    X(spi_err,      U, r.spi_err,      true)                      \
    X(chain_rec,    U, r.chain_rec,    true)

#define AMS_LOG_COUNT_COLUMN(name, kind, value, present) +1u
inline constexpr std::size_t HeadColumns = 0u AMS_LOG_HEAD_COLUMNS(AMS_LOG_COUNT_COLUMN);
inline constexpr std::size_t TailColumns = 0u AMS_LOG_TAIL_COLUMNS(AMS_LOG_COUNT_COLUMN);
#undef AMS_LOG_COUNT_COLUMN
inline constexpr std::size_t CellColumns =
    static_cast<std::size_t>(config::BmsModuleCount) * config::CellsPerModule;
inline constexpr std::size_t TempColumns =
    static_cast<std::size_t>(config::BmsModuleCount) * config::TempsPerModule;
inline constexpr std::size_t TotalColumns = HeadColumns + CellColumns + TempColumns + TailColumns;

// Upper bound on one formatted CSV row (and the header), incl. newline + NUL.
// Every cell/temp field and name is <= 8 chars ("-32768,", "c4_18,"). Each
// scalar field is <= 12 chars ("4294967295,") and each scalar name <= 13, so 16
// per scalar column bounds both the header and the rows.
inline constexpr std::size_t MaxRowBytes =
    (HeadColumns + TailColumns) * 16u + (CellColumns + TempColumns) * 8u + 2u;

namespace detail {

// Each helper appends one field plus its trailing comma at `off`, and returns
// false on truncation. The caller replaces the final comma with '\n'.
inline bool put_text(char* buf, std::size_t cap, int& off, const char* s) noexcept {
    const std::size_t rem = cap - static_cast<std::size_t>(off);
    const int k = std::snprintf(buf + off, rem, "%s,", s);
    if (k < 0 || static_cast<std::size_t>(k) >= rem) return false;
    off += k;
    return true;
}

inline bool put_U(char* buf, std::size_t cap, int& off, bool present,
                  unsigned long v) noexcept {
    const std::size_t rem = cap - static_cast<std::size_t>(off);
    const int k = present ? std::snprintf(buf + off, rem, "%lu,", v)
                          : std::snprintf(buf + off, rem, ",");
    if (k < 0 || static_cast<std::size_t>(k) >= rem) return false;
    off += k;
    return true;
}

inline bool put_S(char* buf, std::size_t cap, int& off, bool present,
                  long v) noexcept {
    const std::size_t rem = cap - static_cast<std::size_t>(off);
    const int k = present ? std::snprintf(buf + off, rem, "%ld,", v)
                          : std::snprintf(buf + off, rem, ",");
    if (k < 0 || static_cast<std::size_t>(k) >= rem) return false;
    off += k;
    return true;
}

}  // namespace detail

#define AMS_LOG_PUT_NAME(name, kind, value, present) \
    if (!detail::put_text(buf, cap, off, #name)) return 0;
#define AMS_LOG_PUT_VALUE(name, kind, value, present) \
    if (!detail::put_##kind(buf, cap, off, (present), (value))) return 0;

// Build the CSV header into `buf` (newline-terminated). Returns bytes written
// (excl. NUL), or 0 on truncation.
inline std::size_t build_header(char* buf, std::size_t cap) noexcept {
    if (cap == 0u) return 0;
    int off = 0;
    AMS_LOG_HEAD_COLUMNS(AMS_LOG_PUT_NAME)
    char name[12];
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::CellsPerModule; ++n) {
            std::snprintf(name, sizeof name, "c%u_%u", m, n);
            if (!detail::put_text(buf, cap, off, name)) return 0;
        }
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::TempsPerModule; ++n) {
            std::snprintf(name, sizeof name, "t%u_%u", m, n);
            if (!detail::put_text(buf, cap, off, name)) return 0;
        }
    AMS_LOG_TAIL_COLUMNS(AMS_LOG_PUT_NAME)
    buf[off - 1] = '\n';  // overwrite the trailing comma
    return static_cast<std::size_t>(off);
}

// Format one record as a CSV line into `buf` (newline-terminated). Returns
// bytes written (excl. NUL), or 0 on truncation/error. Integer-only ->
// deterministic, no FPU, no locale.
inline std::size_t format_row(const LogRecord& r, char* buf, std::size_t cap) noexcept {
    if (cap == 0u) return 0;
    int off = 0;
    AMS_LOG_HEAD_COLUMNS(AMS_LOG_PUT_VALUE)
    const bool bms = r.bms_valid != 0u;
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::CellsPerModule; ++n)
            if (!detail::put_U(buf, cap, off, bms, r.cell_mV[m][n])) return 0;
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::TempsPerModule; ++n)
            if (!detail::put_S(buf, cap, off, bms, r.cell_tempC[m][n])) return 0;
    AMS_LOG_TAIL_COLUMNS(AMS_LOG_PUT_VALUE)
    buf[off - 1] = '\n';  // overwrite the trailing comma
    return static_cast<std::size_t>(off);
}

#undef AMS_LOG_PUT_NAME
#undef AMS_LOG_PUT_VALUE

// Age of something last updated at `tick`, saturating at 65535 ms. A tick of 0
// means "never" (the services' convention) and also reads 65535.
[[nodiscard]] inline constexpr std::uint16_t age_ms(std::uint32_t now_ms,
                                                    std::uint32_t tick) noexcept {
    if (tick == 0u) return 0xFFFFu;
    const std::uint32_t d = now_ms - tick;   // unsigned: wrap-safe
    return d >= 0xFFFFu ? static_cast<std::uint16_t>(0xFFFFu) : static_cast<std::uint16_t>(d);
}

// Age of the NEWEST of the per-module poll ticks (BmsState::last_rx_tick):
// how old the freshest cell data in the snapshot is. 65535 if none reported.
[[nodiscard]] inline std::uint16_t newest_age_ms(std::uint32_t now_ms,
                                                 const std::uint32_t* ticks,
                                                 std::size_t n) noexcept {
    std::uint16_t best = 0xFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint16_t a = age_ms(now_ms, ticks[i]);
        if (a < best) best = a;
    }
    return best;
}

// Is a sample due this tick? Rows are written from boot, including before the
// first full BMS poll: the BMS-derived fields are simply empty then (see
// LogRecord::bms_valid), so a boot whose chain never comes up still records
// state, faults and current instead of leaving a header-only file.
[[nodiscard]] inline constexpr bool sample_due(std::uint32_t now_ms,
                                               std::uint32_t last_sample_ms) noexcept {
    return (now_ms - last_sample_ms) >= config::LogSamplePeriodMs;
}

}  // namespace log_csv
}  // namespace ams
