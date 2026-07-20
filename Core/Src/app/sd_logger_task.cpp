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
#include "crc32.hpp"
#include "diag_dispatch.hpp"
#include "diag_proto.hpp"
#include "log_record.hpp"
#include "log_ring.hpp"
#include "logfs_server.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "fatfs.h"     // FATFS, FIL, FRESULT, FILINFO, f_*, UINT

#include <cstdio>
#include <cstring>

extern "C" {
// hsd1 is OWNED here. With MX_SDMMC1_SD_Init decoupled in CubeMX (#407) the
// handle is no longer defined in main.c, so the logger -- which now owns SD
// bring-up -- defines it; bsp_driver_sd.c (the FatFs BSP) externs and drives
// the same handle. If CubeMX's SDMMC1 init call is ever re-enabled, drop this
// definition to avoid a duplicate symbol.
SD_HandleTypeDef hsd1;
extern char SDPath[4];          // FatFs logical drive, set by MX_FATFS_Init

// SDMMC1 low-level bring-up. HAL_SD_Init() calls this from f_mount (via the
// FatFs BSP); with MX_SDMMC1_SD_Init decoupled (#407) nothing else configures
// the peripheral, so the HAL's __weak default would run instead -- and it is
// empty. That left RCC_AHB3ENR.SDMMC1EN clear and PC8-12/PD2 unconfigured, so
// the peripheral never shifted CMD0 out (no CMDSENT) and every mount timed out
// in SDMMC_GetCmdError regardless of the card (#408). Enabling the bus clock +
// pins here is the missing piece; the kernel clock source (SDMMCSEL=PLL2R) is
// already set by PeriphCommonClock_Config at boot. Called lazily at mount, so
// an absent card still cannot stall boot (#407-safe). KEEP the pin map in sync
// with AMS.ioc SDMMC1 (PC8=D0 PC9=D1 PC10=D2 PC11=D3 PC12=CK, PD2=CMD, AF12).
void HAL_SD_MspInit(SD_HandleTypeDef *hsd) {
    if (hsd->Instance != SDMMC1) return;

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;              // external 47k pull-ups on MAIN_LITE
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_SDMMC1;

    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &g);

    // #408: the FatFs diskio reads via HAL_SD_ReadBlocks_DMA and blocks on a
    // queue posted from the Rx-complete callback, which runs in the SDMMC1 ISR.
    // Enable that IRQ or the DMA transfer never completes -> f_mount returns
    // FR_DISK_ERR with hsd1.ErrorCode=0 (it simply never finishes). Priority 5 =
    // FreeRTOS-syscall-safe (== configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
    // matches FDCAN1_IT0); the callback does an ISR-safe osMessageQueuePut.
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd) {
    if (hsd->Instance != SDMMC1) return;
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                           GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}

// SDMMC1 completion ISR -- routes to the HAL, which fires the FatFs BSP Rx/Tx-
// complete callbacks that post READ_CPLT_MSG/WRITE_CPLT_MSG to the SD_read /
// SD_write message queue. Not emitted by CubeMX (SDMMC1 init is decoupled, #407),
// so DMA transfers would otherwise never complete (#408).
void SDMMC1_IRQHandler(void) { HAL_SD_IRQHandler(&hsd1); }
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
// Running CRC-32 of the active file, folded in as rows are written. Kept so
// the #406 CRC opcode can answer without re-reading a 4 MiB file off the card
// (which would stall this task, and with it the drain, for seconds).
std::uint32_t g_file_crc   = ams::crc::Crc32Init;
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
    g_file_crc   = ams::crc::update(ams::crc::Crc32Init, g_rowbuf, hn);
    g_file_open  = true;
    return true;
}

// Write LOGnnnn.CRC next to the sealed CSV: 8 ASCII hex digits + newline.
//
// A sidecar rather than a trailer inside the CSV, because the file must stay
// directly openable in a spreadsheet -- that is the whole reason it is CSV.
// Best-effort: if this fails the CRC opcode falls back to reading the file,
// so a missing sidecar costs time, not correctness. Files written before this
// existed simply have no sidecar and take the slow path.
void write_crc_sidecar(std::uint32_t idx, std::uint32_t crc) noexcept {
    char  path[16];
    char  text[16];
    FIL   f;
    std::snprintf(path, sizeof path, ams::config::LogCrcNameFmt,
                  static_cast<unsigned long>(idx));
    const int tn = std::snprintf(text, sizeof text, "%08lX\n",
                                 static_cast<unsigned long>(crc));
    if (tn <= 0) return;
    if (f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return;
    UINT bw = 0;
    (void)f_write(&f, text, static_cast<UINT>(tn), &bw);
    (void)f_close(&f);
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
    // Sidecar AFTER the rename, so a .CRC only ever exists next to a sealed
    // .CSV -- never next to a .TMP that is still growing.
    write_crc_sidecar(g_file_idx, ams::crc::finalize(g_file_crc));
    g_file_open = false;
    g_file_crc  = ams::crc::Crc32Init;
    ++g_file_idx;
    ++g_log_files;
}

// Drop to the unmounted state on an I/O error / card pull so the next tick
// re-mounts cleanly. Best-effort; ignores secondary errors.
void teardown(std::uint8_t new_state) noexcept {
    if (g_file_open) { (void)f_close(&g_fil); g_file_open = false; }
    if (g_mounted)   { (void)f_mount(nullptr, SDPath, 0); g_mounted = false; }
    // The abandoned file stays a .TMP and never gets a sidecar, so the partial
    // CRC must not carry into the next file.
    g_file_crc  = ams::crc::Crc32Init;
    g_log_state = new_state;
}

// ---------------------------------------------------------------------------
// LOGFS backend (#406) -- the FatFs half of ams::logfs::Server.
//
// Every method here runs on THIS thread; see logfs_server.hpp for why the
// server is not given its own task. Only sealed LOGnnnn.CSV files are visible:
// the active .TMP is still growing (its length would be a lie by the time the
// host finished reading it) and .CRC sidecars are an implementation detail.
// ---------------------------------------------------------------------------
class FatFsLogBackend {
public:
    [[nodiscard]] bool card_present() const noexcept { return g_mounted; }

    bool list_begin(std::uint16_t cursor) noexcept {
        close_dir();
        if (f_opendir(&dir_, "/") != FR_OK) return false;
        dir_open_ = true;
        skip_     = cursor;
        return true;
    }

    // _USE_FIND is 0 in ffconf.h, so there is no f_findfirst -- enumerate and
    // filter by hand. _USE_LFN is 0 too, so fname is a 13-byte 8.3 name.
    bool list_next(ams::logfs::Entry& out) noexcept {
        if (!dir_open_) return false;
        FILINFO fno;
        for (;;) {
            if (f_readdir(&dir_, &fno) != FR_OK || fno.fname[0] == '\0') {
                close_dir();
                return false;
            }
            if ((fno.fattrib & AM_DIR) != 0) continue;
            std::uint16_t idx = 0;
            if (!parse_sealed_name(fno.fname, idx)) continue;   // .TMP/.CRC/other
            if (skip_ > 0) { --skip_; continue; }

            out.index = idx;
            out.size  = static_cast<std::uint32_t>(fno.fsize);
            // Packed FAT timestamp: date in the high half, time in the low.
            out.mtime = (static_cast<std::uint32_t>(fno.fdate) << 16) |
                         static_cast<std::uint32_t>(fno.ftime);
            std::snprintf(out.name, sizeof out.name, "%s", fno.fname);
            return true;
        }
    }

    bool open(std::uint16_t index, std::uint32_t& size_out) noexcept {
        close_file();
        char path[16];
        std::snprintf(path, sizeof path, ams::config::LogSealedNameFmt,
                      static_cast<unsigned long>(index));
        if (f_open(&rd_, path, FA_READ) != FR_OK) return false;
        rd_open_ = true;
        size_out = static_cast<std::uint32_t>(f_size(&rd_));
        return true;
    }

    int read(std::uint16_t, std::uint32_t off, std::uint8_t* out,
             std::uint16_t len) noexcept {
        if (!rd_open_) return -1;
        if (f_lseek(&rd_, static_cast<FSIZE_t>(off)) != FR_OK) return -1;
        UINT br = 0;
        if (f_read(&rd_, out, static_cast<UINT>(len), &br) != FR_OK) return -1;
        return static_cast<int>(br);   // short read == EOF, not an error
    }

    // Prefer the sidecar written at seal; fall back to streaming the file.
    // The fallback exists for cards written before sidecars, and costs seconds
    // on a 4 MiB file -- which is why the sidecar exists.
    bool crc32(std::uint16_t index, std::uint32_t& crc_out) noexcept {
        if (read_sidecar(index, crc_out)) return true;
        if (!rd_open_) return false;
        if (f_lseek(&rd_, 0) != FR_OK) return false;

        std::uint32_t running = ams::crc::Crc32Init;
        for (;;) {
            UINT br = 0;
            if (f_read(&rd_, g_rowbuf, sizeof g_rowbuf, &br) != FR_OK) return false;
            if (br == 0) break;
            running = ams::crc::update(running, g_rowbuf, br);
        }
        crc_out = ams::crc::finalize(running);
        return true;
    }

    void close(std::uint16_t) noexcept { close_file(); }

private:
    // "LOGnnnn.CSV" -> nnnn. Rejects .TMP (still growing) and .CRC (sidecar).
    static bool parse_sealed_name(const char* n, std::uint16_t& idx) noexcept {
        if (std::strlen(n) != 11u) return false;
        if (std::strncmp(n, "LOG", 3) != 0) return false;
        if (std::strcmp(n + 7, ".CSV") != 0) return false;
        std::uint32_t v = 0;
        for (int i = 3; i < 7; ++i) {
            if (n[i] < '0' || n[i] > '9') return false;
            v = v * 10u + static_cast<std::uint32_t>(n[i] - '0');
        }
        idx = static_cast<std::uint16_t>(v);
        return true;
    }

    bool read_sidecar(std::uint16_t index, std::uint32_t& crc_out) noexcept {
        char path[16];
        char text[16] = {};
        std::snprintf(path, sizeof path, ams::config::LogCrcNameFmt,
                      static_cast<unsigned long>(index));
        FIL f;
        if (f_open(&f, path, FA_READ) != FR_OK) return false;
        UINT br = 0;
        const FRESULT fr = f_read(&f, text, sizeof text - 1, &br);
        (void)f_close(&f);
        if (fr != FR_OK || br < 8u) return false;

        std::uint32_t v = 0;
        for (UINT i = 0; i < 8u; ++i) {
            const char c = text[i];
            std::uint32_t d;
            if      (c >= '0' && c <= '9') d = static_cast<std::uint32_t>(c - '0');
            else if (c >= 'A' && c <= 'F') d = static_cast<std::uint32_t>(c - 'A' + 10);
            else if (c >= 'a' && c <= 'f') d = static_cast<std::uint32_t>(c - 'a' + 10);
            else return false;
            v = (v << 4) | d;
        }
        crc_out = v;
        return true;
    }

    void close_dir() noexcept {
        if (dir_open_) { (void)f_closedir(&dir_); dir_open_ = false; }
    }
    void close_file() noexcept {
        if (rd_open_) { (void)f_close(&rd_); rd_open_ = false; }
    }

    DIR           dir_{};
    FIL           rd_{};
    bool          dir_open_ = false;
    bool          rd_open_  = false;
    std::uint16_t skip_     = 0;
};

using LogfsSrv = ams::logfs::Server<FatFsLogBackend>;

FatFsLogBackend                 g_logfs_be;
LogfsSrv                        g_logfs_srv(g_logfs_be);
ams::diag::Session              g_diag_session;
ams::diag::Dispatcher<LogfsSrv> g_diag_disp(g_diag_session, g_logfs_srv);

// Single in-flight transaction: the session admits one host at a time, so a
// second concurrent request is a protocol error, not a case to buffer for.
std::uint8_t  g_diag_req[ams::isotp::MaxMsg];
std::uint8_t  g_diag_rsp[ams::isotp::MaxMsg];
volatile std::uint16_t g_diag_req_len = 0;
volatile std::uint16_t g_diag_rsp_len = 0;
volatile bool          g_diag_busy    = false;   // request posted, reply pending

// Wakes the logger thread the moment a request is posted, instead of letting
// it sit out the rest of the drain period. Binary: at most one transaction is
// ever in flight.
osSemaphoreId_t g_diag_sem = nullptr;

// Serve whatever is sitting in g_diag_req. Runs on the logger thread.
//
// Routing/session policy lives in ams::diag::Dispatcher (host-tested); this
// function is only the thread handoff around it. A returned length of 0 means
// "say nothing" -- e.g. a CMD-typed frame, which belongs to the bootloader's
// namespace and which the application must not answer even to refuse.
void serve_diag_request() noexcept {
    const std::uint16_t n = g_diag_disp.handle(g_diag_req, g_diag_req_len,
                                               ams::config::AmsNodeId,
                                               osKernelGetTickCount(),
                                               g_diag_rsp, sizeof g_diag_rsp);

    g_diag_rsp_len = n;
    // Publish the length BEFORE clearing the busy flag the CAN side polls.
    // volatile orders the compiler but not the Cortex-M7 store buffer, and
    // unlike the submit path there is no semaphore here to supply the barrier.
    __DMB();
    g_diag_busy    = false;
}

}  // namespace

namespace ams {

bool sd_diag_submit(const std::uint8_t* msg, std::uint16_t len) noexcept {
    if (msg == nullptr || len == 0 || len > sizeof g_diag_req) return false;
    if (g_diag_sem == nullptr) return false;                // logger not up yet
    if (g_diag_busy || g_diag_rsp_len != 0) return false;   // caller -> NACK BUSY
    std::memcpy(g_diag_req, msg, len);
    g_diag_req_len = len;
    g_diag_busy    = true;
    (void)osSemaphoreRelease(g_diag_sem);
    return true;
}

std::uint16_t sd_diag_collect(std::uint8_t* out, std::uint16_t cap) noexcept {
    if (g_diag_busy || g_diag_rsp_len == 0) return 0;
    const std::uint16_t n = g_diag_rsp_len;
    if (out == nullptr || cap < n) { g_diag_rsp_len = 0; return 0; }
    std::memcpy(out, g_diag_rsp, n);
    g_diag_rsp_len = 0;
    return n;
}

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

    // Created here, before the first sd_diag_submit can succeed -- submit()
    // refuses while this is null, so a diag frame arriving during boot is
    // rejected rather than lost into an uninitialised semaphore.
    g_diag_sem = osSemaphoreNew(1, 0, nullptr);

    std::uint32_t next_drain = osKernelGetTickCount() + ams::config::LogDrainPeriodMs;
    std::uint32_t last_sync  = osKernelGetTickCount();

    for (;;) {
        // Block until the next drain is due, or until a diag request arrives --
        // same deadline-bounded wait AcuCanTask uses. A LOGFS pull is a
        // request/response walk of thousands of round trips, so making each one
        // wait out a full 50 ms drain period would roughly triple an already
        // multi-minute transfer. Waiting on the semaphore (rather than polling)
        // keeps the idle cost at one wakeup per drain.
        {
            const std::int32_t left =
                static_cast<std::int32_t>(next_drain - osKernelGetTickCount());
            (void)osSemaphoreAcquire(g_diag_sem, (left > 0) ? static_cast<std::uint32_t>(left) : 0u);
        }

        // Serve diag first: bounded work (one 512-byte read at most) and the
        // host is blocked on the answer.
        if (g_diag_busy) serve_diag_request();

        // Session idle-timeout: releases any file handle left open by a host
        // that walked away mid-pull.
        (void)g_diag_disp.tick(osKernelGetTickCount());

        if (static_cast<std::int32_t>(next_drain - osKernelGetTickCount()) > 0) continue;
        next_drain += ams::config::LogDrainPeriodMs;
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
            g_file_crc    = ams::crc::update(g_file_crc, g_rowbuf, n);
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
