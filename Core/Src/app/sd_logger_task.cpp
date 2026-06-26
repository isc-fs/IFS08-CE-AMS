// SPDX-License-Identifier: proprietary
//
// SdLoggerTask body -- see app/sd_logger_task.h for the contract.
//
// Consumer loop (LogDrainPeriodMs cadence):
//   1. ensure mounted   -- non-fatal f_mount; no card -> retry next tick
//   2. ensure file open -- LOGnnnn.TMP + CSV header
//   3. drain the ring   -- format each LogRecord -> f_write; rotate at LogFileMaxBytes
//   4. periodic f_sync  -- bound power-cut loss
//
// Any I/O error (card pulled mid-write, etc.) tears down to the unmounted
// state and re-mounts on the next tick. Nothing here can block or fault the
// safety loop -- the only coupling is the wait-free ring the producer fills.

#include "app/sd_logger_task.h"

#include "ams_config.hpp"
#include "log_record.hpp"
#include "log_ring.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "fatfs.h"     // FATFS, FIL, FRESULT, FILINFO, f_*, UINT

#include <cstdio>

extern "C" {
// hsd1 is OWNED here. With MX_SDMMC1_SD_Init decoupled in CubeMX (#407) the
// handle is no longer defined in main.c, so the logger -- which now owns SD
// bring-up -- defines it; bsp_driver_sd.c (the FatFs BSP) externs and drives
// the same handle. If CubeMX's SDMMC1 init call is ever re-enabled, drop this
// definition to avoid a duplicate symbol.
SD_HandleTypeDef hsd1;
extern char SDPath[4];          // FatFs logical drive, set by MX_FATFS_Init
}

namespace {

// ---- producer <-> consumer ring + health counters (file-local) ----
ams::SpscRing<ams::LogRecord, ams::config::LogRingCapacity> g_ring;
volatile std::uint32_t g_log_rows    = 0;   // CSV rows written
volatile std::uint32_t g_log_dropped = 0;   // producer drops (ring full)
volatile std::uint32_t g_log_files   = 0;   // files sealed
volatile std::uint8_t  g_log_state   = 0;   // 0=boot 1=no_card 2=logging 3=io_error

// ---- consumer-side file state ----
FATFS         g_fs;
FIL           g_fil;
bool          g_mounted    = false;
bool          g_file_open  = false;
std::uint32_t g_file_idx   = 0;
std::uint32_t g_file_bytes = 0;
char          g_rowbuf[ams::log_csv::MaxRowBytes];
char          g_name[16];

// Mirror the AMS.ioc SDMMC1 config onto hsd1. The boot-path MX_SDMMC1_SD_Init()
// is intentionally NOT auto-called (CubeMX Advanced Settings) so an absent
// card can't brick the node (#407); we set the handle here and let f_mount run
// the non-fatal BSP_SD_Init lazily. KEEP IN SYNC with AMS.ioc SDMMC1.
void configure_hsd1() noexcept {
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 3;   // 132 MHz / (2*3) = 22 MHz
}

// Lowest LOGnnnn.CSV index not present on the card -> the next free file.
// Probed once per mount; O(existing logs), negligible at the rotation rate.
std::uint32_t next_free_index() noexcept {
    FILINFO fno;
    for (std::uint32_t i = 0; i < 10000u; ++i) {
        std::snprintf(g_name, sizeof g_name, ams::config::LogSealedNameFmt,
                      static_cast<unsigned long>(i));
        if (f_stat(g_name, &fno) != FR_OK) return i;   // not present -> free
    }
    return 0;   // card already full of logs -> wrap (best-effort)
}

// Open LOGnnnn.TMP for the current index and write the CSV header.
bool open_new_file() noexcept {
    std::snprintf(g_name, sizeof g_name, ams::config::LogActiveNameFmt,
                  static_cast<unsigned long>(g_file_idx));
    if (f_open(&g_fil, g_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
    const std::size_t hn = ams::log_csv::build_header(g_rowbuf, sizeof g_rowbuf);
    UINT bw = 0;
    if (hn == 0 || f_write(&g_fil, g_rowbuf, hn, &bw) != FR_OK || bw != hn) {
        f_close(&g_fil);
        return false;
    }
    g_file_bytes = static_cast<std::uint32_t>(hn);
    g_file_open  = true;
    return true;
}

// Close LOGnnnn.TMP and rename to LOGnnnn.CSV so the #406 extractor only ever
// sees finished (sealed) files. Advance to the next index.
void seal_file() noexcept {
    f_close(&g_fil);
    char sealed[16];
    std::snprintf(g_name,  sizeof g_name,  ams::config::LogActiveNameFmt,
                  static_cast<unsigned long>(g_file_idx));
    std::snprintf(sealed,  sizeof sealed,  ams::config::LogSealedNameFmt,
                  static_cast<unsigned long>(g_file_idx));
    (void)f_rename(g_name, sealed);
    g_file_open = false;
    ++g_file_idx;
    ++g_log_files;
}

// Drop to the unmounted state on an I/O error / card pull so the next tick
// re-mounts cleanly. Best-effort; ignores secondary errors.
void teardown(std::uint8_t new_state) noexcept {
    if (g_file_open) { (void)f_close(&g_fil); g_file_open = false; }
    if (g_mounted)   { (void)f_mount(nullptr, SDPath, 0); g_mounted = false; }
    g_log_state = new_state;
}

}  // namespace

namespace ams {

bool sd_log_push(const LogRecord& rec) noexcept {
    if (!g_ring.push(rec)) { ++g_log_dropped; return false; }
    return true;
}

SdLogStats sd_log_stats() noexcept {
    return SdLogStats{ g_log_rows, g_log_dropped, g_log_files, g_log_state };
}

}  // namespace ams

extern "C" void ams_sd_logger_task_run(void *argument) {
    (void)argument;

    configure_hsd1();

    std::uint32_t last_wake = osKernelGetTickCount();
    std::uint32_t last_sync = last_wake;

    for (;;) {
        last_wake += ams::config::LogDrainPeriodMs;
        osDelayUntil(last_wake);
        const std::uint32_t now = osKernelGetTickCount();

        // (1) Mount (non-fatal). f_mount(opt=1) runs BSP_SD_Init, which checks
        // the PE3 detect pin first and only returns codes -- it never bricks.
        if (!g_mounted) {
            if (f_mount(&g_fs, SDPath, 1) == FR_OK) {
                g_mounted   = true;
                g_file_idx  = next_free_index();
                g_log_state = 2;
            } else {
                g_log_state = 1;   // no card / not ready -> retry next tick
                ams::LogRecord scratch;        // keep the ring from wedging
                while (g_ring.pop(scratch)) { /* discard while cardless */ }
                continue;
            }
        }

        // (2) Ensure an active file is open.
        if (!g_file_open && !open_new_file()) { teardown(3); continue; }

        // (3) Drain the ring -> CSV rows.
        ams::LogRecord r;
        while (g_ring.pop(r)) {
            const std::size_t n = ams::log_csv::format_row(r, g_rowbuf, sizeof g_rowbuf);
            if (n == 0) continue;              // skip a malformed row, keep going
            UINT bw = 0;
            if (f_write(&g_fil, g_rowbuf, n, &bw) != FR_OK || bw != n) {
                teardown(3);                   // card pulled / write error
                break;
            }
            g_file_bytes += static_cast<std::uint32_t>(n);
            ++g_log_rows;
            if (g_file_bytes >= ams::config::LogFileMaxBytes) {
                seal_file();                   // rotate; next file opens next tick
                break;
            }
        }

        // (4) Periodic flush -- bounds data lost on a power-cut to one window.
        if (g_file_open && (now - last_sync) >= ams::config::LogSyncPeriodMs) {
            last_sync = now;
            if (f_sync(&g_fil) != FR_OK) teardown(3);
        }
    }
}
