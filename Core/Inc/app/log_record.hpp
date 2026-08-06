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

#pragma once

#include "ams_config.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace ams {

struct LogRecord {
    std::uint32_t tick_ms;             // osKernelGetTickCount() at capture
    std::uint32_t pack_mV;             // BmsState.pack_voltage_mV
    std::int32_t  pack_current_raw_mA; // CurrentState.raw_mA       (+ = discharge)
    std::int32_t  pack_current_mA;     // CurrentState.filtered_mA
    std::int32_t  dcdc_current_mA;     // CurrentState.dcdc_filtered_mA
    std::uint16_t min_cell_mV;         // BmsState.min_cell_mV  (summary, also in cell_mV)
    std::uint16_t max_cell_mV;         // BmsState.max_cell_mV
    std::uint16_t dc_bus_V;            // VehicleState.dc_bus_V (VCU 0x100)
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
};

namespace log_csv {

// Upper bound on one formatted CSV row (and the header), incl. newline + NUL.
// "65535," / "-32768," <= 8 chars each; 256 covers the scalar block.
inline constexpr std::size_t MaxRowBytes =
    256u
    + static_cast<std::size_t>(config::BmsModuleCount) * config::CellsPerModule * 8u
    + static_cast<std::size_t>(config::BmsModuleCount) * config::TempsPerModule * 8u
    + 2u;

// Build the CSV header into `buf` (newline-terminated). Column order MUST
// match format_row(). Returns bytes written (excl. NUL), or 0 on truncation.
inline std::size_t build_header(char* buf, std::size_t cap) noexcept {
    int off = std::snprintf(
        buf, cap,
        "tick_ms,fsm,mode,ams_ok,fault,detail,tsms,dash_chg,mod_mask,"
        "pack_mV,I_raw_mA,I_filt_mA,Idcdc_mA,dcbus_V,"
        "vmin_mV,vmax_mV,tmin_C,tmax_C,tavg_C,");
    if (off < 0 || static_cast<std::size_t>(off) >= cap) return 0;
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::CellsPerModule; ++n) {
            const std::size_t rem = cap - static_cast<std::size_t>(off);
            const int k = std::snprintf(buf + off, rem, "c%u_%u,", m, n);
            if (k < 0 || static_cast<std::size_t>(k) >= rem) return 0;
            off += k;
        }
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::TempsPerModule; ++n) {
            const std::size_t rem = cap - static_cast<std::size_t>(off);
            const int k = std::snprintf(buf + off, rem, "t%u_%u,", m, n);
            if (k < 0 || static_cast<std::size_t>(k) >= rem) return 0;
            off += k;
        }
    buf[off - 1] = '\n';  // overwrite the trailing comma
    return static_cast<std::size_t>(off);
}

// Format one record as a CSV line into `buf` (newline-terminated). Returns
// bytes written (excl. NUL), or 0 on truncation/error. Integer-only ->
// deterministic, no FPU, no locale. Column order MUST match build_header().
inline std::size_t format_row(const LogRecord& r, char* buf, std::size_t cap) noexcept {
    int off = std::snprintf(
        buf, cap,
        "%lu,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%lu,%ld,%ld,%ld,%u,"
        "%u,%u,%d,%d,%d,",
        static_cast<unsigned long>(r.tick_ms),
        static_cast<unsigned>(r.fsm_state),
        static_cast<unsigned>(r.mode),
        static_cast<unsigned>(r.ams_ok),
        static_cast<unsigned>(r.fault_reason),
        static_cast<unsigned>(r.fault_detail),
        static_cast<unsigned>(r.tsms),
        static_cast<unsigned>(r.dash_chg),
        static_cast<unsigned>(r.module_online_mask),
        static_cast<unsigned long>(r.pack_mV),
        static_cast<long>(r.pack_current_raw_mA),
        static_cast<long>(r.pack_current_mA),
        static_cast<long>(r.dcdc_current_mA),
        static_cast<unsigned>(r.dc_bus_V),
        static_cast<unsigned>(r.min_cell_mV),
        static_cast<unsigned>(r.max_cell_mV),
        static_cast<int>(r.min_tempC),
        static_cast<int>(r.max_tempC),
        static_cast<int>(r.avg_tempC));
    if (off < 0 || static_cast<std::size_t>(off) >= cap) return 0;
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::CellsPerModule; ++n) {
            const std::size_t rem = cap - static_cast<std::size_t>(off);
            const int k = std::snprintf(buf + off, rem, "%u,",
                                        static_cast<unsigned>(r.cell_mV[m][n]));
            if (k < 0 || static_cast<std::size_t>(k) >= rem) return 0;
            off += k;
        }
    for (unsigned m = 0; m < config::BmsModuleCount; ++m)
        for (unsigned n = 0; n < config::TempsPerModule; ++n) {
            const std::size_t rem = cap - static_cast<std::size_t>(off);
            const int k = std::snprintf(buf + off, rem, "%d,",
                                        static_cast<int>(r.cell_tempC[m][n]));
            if (k < 0 || static_cast<std::size_t>(k) >= rem) return 0;
            off += k;
        }
    buf[off - 1] = '\n';  // overwrite the trailing comma
    return static_cast<std::size_t>(off);
}

}  // namespace log_csv
}  // namespace ams
