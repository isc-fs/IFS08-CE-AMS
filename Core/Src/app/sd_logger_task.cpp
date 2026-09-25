// SPDX-License-Identifier: proprietary
//
// SdLoggerTask body -- see app/sd_logger_task.h for the contract.
//
// Consumer loop (LogDrainPeriodMs cadence):
//   1. ensure mounted   -- non-fatal f_mount; no card -> retry next tick
//   2. ensure file open -- LOGnnnn.TMP + CSV header
//   3. drain the rings  -- LogRecords -> LOGnnnn.TMP, ImuSamples -> IMUnnnn.TMP
//                          (opened on the first IMU sample, so a board with no
//                          IMU writes no IMU files)
//   4. rotate the PAIR  -- on LogFileMaxBytes OR LogFileMaxMs of either file,
//                          sealing both .TMP -> .CSV and moving to the next index
//   5. periodic f_sync  -- bound power-cut loss
//
// LOGnnnn and IMUnnnn always share an index and a time window: they are opened
// against the same index and sealed together (log_names.hpp).
//
// Any I/O error (card pulled mid-write, etc.) tears down to the unmounted
// state and re-mounts on the next tick. Nothing here can block or fault the
// safety loop -- the only coupling is the wait-free rings the producers fill.

#include "app/sd_logger_task.h"

#include "ams_config.hpp"
#include "crc32.hpp"
#include "diag_dispatch.hpp"
#include "diag_proto.hpp"
#include "imu_record.hpp"
#include "log_names.hpp"
#include "log_record.hpp"
#include "log_rotation.hpp"
#include "log_ring.hpp"
#include "logfs_server.hpp"
#include "state_machine.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "fatfs.h"     // FATFS, FIL, FRESULT, FILINFO, f_*, UINT

#include <cstdio>
#include <cstring>

extern "C" {
// FSM state mirror written by MainTask. Read-only here, and only to refuse
// log extraction while the tractive system is live.
extern volatile std::uint8_t g_state_telemetry;

// hsd1 is OWNED here. With MX_SDMMC1_SD_Init decoupled in CubeMX the
// handle is no longer defined in main.c, so the logger -- which now owns SD
// bring-up -- defines it; bsp_driver_sd.c (the FatFs BSP) externs and drives
// the same handle. If CubeMX's SDMMC1 init call is ever re-enabled, drop this
// definition to avoid a duplicate symbol.
SD_HandleTypeDef hsd1;
extern char SDPath[4];          // FatFs logical drive, set by MX_FATFS_Init

// SDMMC1 low-level bring-up. HAL_SD_Init() calls this from f_mount (via the
// FatFs BSP); with MX_SDMMC1_SD_Init decoupled nothing else configures
// the peripheral, so the HAL's __weak default would run instead -- and it is
// empty. That left RCC_AHB3ENR.SDMMC1EN clear and PC8-12/PD2 unconfigured, so
// the peripheral never shifted CMD0 out (no CMDSENT) and every mount timed out
// in SDMMC_GetCmdError regardless of the card. Enabling the bus clock +
// pins here is the missing piece; the kernel clock source (SDMMCSEL=PLL2R) is
// already set by PeriphCommonClock_Config at boot. Called lazily at mount, so
// an absent card still cannot stall boot (safe against a missing card). KEEP the pin map in sync
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

    // The FatFs diskio reads via HAL_SD_ReadBlocks_DMA and blocks on a
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
// SD_write message queue. Not emitted by CubeMX (SDMMC1 init is decoupled),
// so DMA transfers would otherwise never complete.
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
// the LOGFS CRC opcode can answer without re-reading a 4 MiB file off the card
// (which would stall this task, and with it the drain, for seconds).
std::uint32_t g_file_crc   = ams::crc::Crc32Init;
std::uint32_t g_file_open_ms = 0;   // tick at which the active file was opened
// Rows in the ACTIVE file. Gates time-based rotation so a stalled producer
// cannot litter the card with header-only files.
std::uint32_t g_rows_this_file = 0;
char          g_rowbuf[ams::log_csv::MaxRowBytes];
char          g_name[16];

// ---- IMU ring + the IMUnnnn file paired with the active LOG file ----
ams::SpscRing<ams::ImuSample, ams::config::ImuRingCapacity> g_imu_ring;
volatile std::uint32_t g_imu_rows    = 0;
volatile std::uint32_t g_imu_dropped = 0;
FIL           g_imu_fil;
bool          g_imu_open           = false;
std::uint32_t g_imu_bytes          = 0;
std::uint32_t g_imu_crc            = ams::crc::Crc32Init;
std::uint32_t g_imu_rows_this_file = 0;
char          g_imu_rowbuf[ams::imu_csv::MaxRowBytes];

using ams::log_names::Kind;
using ams::log_names::Stage;

// Mirror the AMS.ioc SDMMC1 config onto hsd1. The boot-path MX_SDMMC1_SD_Init()
// is intentionally NOT auto-called (CubeMX Advanced Settings) so an absent
// card can't brick the node; we set the handle here and let f_mount run
// the non-fatal BSP_SD_Init lazily. KEEP IN SYNC with AMS.ioc SDMMC1.
void configure_hsd1() noexcept {
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 3;   // 132 MHz / (2*3) = 22 MHz
}

// Stream a whole file and return its CRC-32. Used for files whose running CRC
// was lost (an orphan.TMP from a run that ended with the power), and as the
// CRC opcode's fallback for cards written before sidecars existed.
//
// Reuses g_rowbuf as the read buffer -- it is only live inside the drain loop,
// and both callers run outside it (mount, and a diag request serviced between
// drains). Costs one full read of the file: seconds on 4 MiB, which is exactly
// why the sidecar exists.
bool compute_file_crc(const char* path, std::uint32_t& crc_out) noexcept {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;

    std::uint32_t running = ams::crc::Crc32Init;
    for (;;) {
        UINT br = 0;
        if (f_read(&f, g_rowbuf, sizeof g_rowbuf, &br) != FR_OK) {
            (void)f_close(&f);
            return false;
        }
        if (br == 0) break;
        running = ams::crc::update(running, g_rowbuf, br);
    }
    (void)f_close(&f);
    crc_out = ams::crc::finalize(running);
    return true;
}

// Write LOGnnnn.CRC next to the sealed CSV: 8 ASCII hex digits + newline.
//
// A sidecar rather than a trailer inside the CSV, because the file must stay
// directly openable in a spreadsheet -- that is the whole reason it is CSV.
// Best-effort: if this fails the CRC opcode falls back to reading the file,
// so a missing sidecar costs time, not correctness. Files written before this
// existed simply have no sidecar and take the slow path.
void write_crc_sidecar(Kind kind, std::uint32_t idx, std::uint32_t crc) noexcept {
    char  path[16];
    char  text[16];
    FIL   f;
    if (!ams::log_names::format(path, sizeof path, kind, Stage::Crc, idx)) return;
    const int tn = std::snprintf(text, sizeof text, "%08lX\n",
                                 static_cast<unsigned long>(crc));
    if (tn <= 0) return;
    if (f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return;
    UINT bw = 0;
    (void)f_write(&f, text, static_cast<UINT>(tn), &bw);
    (void)f_close(&f);
}

// Seal an orphaned LOGnnnn.TMP left behind by a previous run.
//
// A.TMP is only renamed to .CSV at rotation, and power simply vanishes at
// shutdown -- so every run that ends before LogFileMaxBytes leaves one behind.
// An orphan is by definition from a run that is over and never coming back, so
// promoting it to.CSV is always correct.
//
// This is what makes short runs produce a readable log at all, and it is why
// the index scan below must consider.TMP as well: it previously looked only
// at.CSV, so the next boot picked the same index and reopened the orphan with
// FA_CREATE_ALWAYS -- truncating the whole previous run.
bool seal_orphan_file(Kind kind, std::uint32_t idx) noexcept {
    char active[16];
    char sealed[16];
    if (!ams::log_names::format(active, sizeof active, kind, Stage::Active, idx) ||
        !ams::log_names::format(sealed, sizeof sealed, kind, Stage::Sealed, idx)) {
        return false;
    }
    if (f_rename(active, sealed) != FR_OK) return false;

    // The running CRC died with the power that ended that run, so it has to be
    // recomputed by streaming the file. Done ONCE, at mount, for a file that is
    // now sealed and immutable -- unlike the CRC opcode's fallback, which would
    // otherwise re-stream the same 4 MiB on every host request. Best-effort: a
    // missing sidecar costs the host time, never correctness.
    std::uint32_t crc = 0;
    if (compute_file_crc(sealed, crc)) write_crc_sidecar(kind, idx, crc);
    return true;
}

// Seal both halves of an orphaned pair. IMU first, LOG last, matching
// seal_file(): the LOG .TMP is what marks the index as orphaned, so if power
// dies between the two renames the next mount still finds it and finishes the
// job. The IMU half may not exist (no IMU fitted, or no sample arrived before
// power was lost); that is not a failure.
bool seal_orphan(std::uint32_t idx) noexcept {
    (void)seal_orphan_file(Kind::Imu, idx);
    if (!seal_orphan_file(Kind::Log, idx)) return false;
    ++g_log_files;
    return true;
}

// Lowest index with neither a sealed.CSV nor an active .TMP. Orphans found on
// the way are sealed. Policy lives in log_rotation.hpp (host-tested); the
// callbacks here are the only part that touches FatFs.
//
// Probed once per mount; O(existing logs), negligible at the rotation rate.
std::uint32_t next_free_index() noexcept {
    return ams::log_rotation::next_index(
        [](std::uint32_t i, bool sealed) noexcept {
            FILINFO fno;
            if (!ams::log_names::format(g_name, sizeof g_name, Kind::Log,
                                        sealed ? Stage::Sealed : Stage::Active, i)) {
                return false;
            }
            return f_stat(g_name, &fno) == FR_OK;
        },
        [](std::uint32_t i) noexcept { return seal_orphan(i); });
}

// Open LOGnnnn.TMP for the current index and write the CSV header.
// `now` is the loop's tick, sampled BEFORE this call. The file-open timestamp
// MUST use it, not a fresh osKernelGetTickCount() taken here: f_open + the
// header f_write are hundreds of ms of SD latency, so a tick sampled after them
// lands ahead of the loop's `now`, and `now - g_file_open_ms` then underflows
// and seals the file on its first rows every iteration.
bool open_new_file(std::uint32_t now) noexcept {
    if (!ams::log_names::format(g_name, sizeof g_name, Kind::Log, Stage::Active,
                                g_file_idx)) {
        return false;
    }
    if (f_open(&g_fil, g_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
    const std::size_t hn = ams::log_csv::build_header(g_rowbuf, sizeof g_rowbuf);
    UINT bw = 0;
    if (hn == 0 || f_write(&g_fil, g_rowbuf, hn, &bw) != FR_OK || bw != hn) {
        f_close(&g_fil);
        return false;
    }
    g_file_bytes     = static_cast<std::uint32_t>(hn);
    g_file_crc       = ams::crc::update(ams::crc::Crc32Init, g_rowbuf, hn);
    g_file_open_ms   = now;
    g_rows_this_file = 0;
    g_file_open      = true;
    return true;
}

// Open IMUnnnn.TMP against the ACTIVE LOG file's index and write its header.
// Called on the first IMU sample of a file window, never on its own, so the
// pair always shares an index. Its window starts at g_file_open_ms like the
// LOG file's; rotation ages both from that one timestamp.
bool open_imu_file() noexcept {
    if (!g_file_open) return false;
    char name[16];
    if (!ams::log_names::format(name, sizeof name, Kind::Imu, Stage::Active,
                                g_file_idx)) {
        return false;
    }
    if (f_open(&g_imu_fil, name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
    const std::size_t hn = sizeof ams::imu_csv::Header - 1u;
    UINT bw = 0;
    if (f_write(&g_imu_fil, ams::imu_csv::Header, hn, &bw) != FR_OK || bw != hn) {
        f_close(&g_imu_fil);
        return false;
    }
    g_imu_bytes          = static_cast<std::uint32_t>(hn);
    g_imu_crc            = ams::crc::update(ams::crc::Crc32Init, ams::imu_csv::Header, hn);
    g_imu_rows_this_file = 0;
    g_imu_open           = true;
    return true;
}

// Close one .TMP, rename it to .CSV and write its sidecar. The sidecar comes
// AFTER the rename, so a .CRC only ever exists next to a sealed .CSV -- never
// next to a .TMP that is still growing.
void seal_one(FIL& fil, Kind kind, std::uint32_t idx, std::uint32_t running_crc) noexcept {
    f_close(&fil);
    char active[16];
    char sealed[16];
    if (!ams::log_names::format(active, sizeof active, kind, Stage::Active, idx) ||
        !ams::log_names::format(sealed, sizeof sealed, kind, Stage::Sealed, idx)) {
        return;
    }
    (void)f_rename(active, sealed);
    write_crc_sidecar(kind, idx, ams::crc::finalize(running_crc));
}

// Seal the active PAIR so the LOGFS extractor only ever sees finished files,
// then advance to the next index. IMU first, LOG last: the LOG .TMP is the
// orphan marker next_free_index() looks for, so it must be the last to go.
void seal_file() noexcept {
    if (g_imu_open) {
        seal_one(g_imu_fil, Kind::Imu, g_file_idx, g_imu_crc);
        g_imu_open = false;
        g_imu_crc  = ams::crc::Crc32Init;
    }
    seal_one(g_fil, Kind::Log, g_file_idx, g_file_crc);
    g_file_open = false;
    g_file_crc  = ams::crc::Crc32Init;
    ++g_file_idx;
    ++g_log_files;
}

// Drop to the unmounted state on an I/O error / card pull so the next tick
// re-mounts cleanly. Best-effort; ignores secondary errors.
void teardown(std::uint8_t new_state) noexcept {
    if (g_imu_open)  { (void)f_close(&g_imu_fil); g_imu_open = false; }
    if (g_file_open) { (void)f_close(&g_fil); g_file_open = false; }
    if (g_mounted)   { (void)f_mount(nullptr, SDPath, 0); g_mounted = false; }
    // The abandoned files stay .TMP and never get sidecars, so the partial
    // CRCs must not carry into the next pair.
    g_file_crc  = ams::crc::Crc32Init;
    g_imu_crc   = ams::crc::Crc32Init;
    g_log_state = new_state;
}

// ---------------------------------------------------------------------------
// LOGFS backend -- the FatFs half of ams::logfs::Server.
//
// Every method here runs on THIS thread; see logfs_server.hpp for why the
// server is not given its own task. Only sealed LOGnnnn.CSV and IMUnnnn.CSV
// files are visible (IMU at index 0x8000|nnnn, see log_names.hpp): an active
// .TMP is still growing (its length would be a lie by the time the host
// finished reading it) and .CRC sidecars are an implementation detail.
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
            if (!ams::log_names::parse_sealed(fno.fname, idx)) continue;   // .TMP/.CRC/other
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

    // crc_out is the SEALED CRC from the.CRC sidecar, or 0 meaning "not
    // available" (a log written before sidecars existed). It is NEVER computed
    // by streaming here: OPEN must stay O(1), or a 4 MiB file would blow both
    // the host timeout and BL_ISOTP_TIMEOUT_MS. A host wanting the CRC of a
    // sidecar-less file asks for it explicitly with LOGFS_CRC.
    //
    // The sidecar is read BEFORE rd_ is opened, deliberately. _FS_LOCK counts
    // files AND directories, and SdLoggerTask permanently holds up to two
    // slots with the active LOG and IMU .TMPs. Opening the sidecar afterwards
    // would stack a third and fourth slot (rd_ + sidecar, plus the directory
    // if a LIST is still open) -- the FR_TOO_MANY_OPEN_FILES that once made
    // the CRC opcode fall back to streaming every time.
    bool open(std::uint16_t index, std::uint32_t& size_out,
              std::uint32_t& crc_out) noexcept {
        close_file();
        crc_out = 0;
        if (!read_sidecar(index, crc_out)) crc_out = 0;

        char path[16];
        if (!sealed_path(index, path, sizeof path)) return false;
        if (f_open(&rd_, path, FA_READ) != FR_OK) return false;
        rd_open_    = true;
        open_crc_   = crc_out;
        size_out    = static_cast<std::uint32_t>(f_size(&rd_));
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
        // Cached at OPEN, so the common path costs nothing and needs no slot.
        if (rd_open_ && open_crc_ != 0u) { crc_out = open_crc_; return true; }
        if (read_sidecar(index, crc_out)) return true;
        // No sidecar: a log written before sidecars existed, or one whose
        // sidecar write failed. Stream it. Seconds on a 4 MiB file, which is
        // the whole reason seal writes a sidecar in the first place.
        char path[16];
        if (!sealed_path(index, path, sizeof path)) return false;
        return compute_file_crc(path, crc_out);
    }

    void close(std::uint16_t) noexcept { close_file(); }

    // Seal the ACTIVE pair so the run that just happened becomes listable
    // without waiting for rotation. Safe to call seal_file() directly: the
    // LOGFS server is serviced ON this thread, between drains, so nothing else
    // is touching the open files.
    //
    // Refused when neither file has rows yet -- sealing header-only files would
    // hand the operator an empty log and burn an index. Reports the LOG file's
    // index; the IMU half, if any, is at the same index with bit 15 set.
    bool finalize(std::uint16_t& sealed_index_out) noexcept {
        if (!g_mounted || !g_file_open) return false;
        const bool imu_rows = g_imu_open && g_imu_rows_this_file > 0;
        if (g_rows_this_file == 0 && !imu_rows) return false;
        sealed_index_out = ams::log_names::logfs_index(Kind::Log, g_file_idx);
        seal_file();   // both halves: f_close -> rename .TMP->.CSV -> sidecar -> ++idx
        return true;
    }

private:
    static bool sealed_path(std::uint16_t logfs_idx, char* out, std::size_t cap) noexcept {
        Kind kind;
        std::uint32_t idx = 0;
        return ams::log_names::from_logfs_index(logfs_idx, kind, idx) &&
               ams::log_names::format(out, cap, kind, Stage::Sealed, idx);
    }

    bool read_sidecar(std::uint16_t logfs_idx, std::uint32_t& crc_out) noexcept {
        char path[16];
        char text[16] = {};
        Kind kind;
        std::uint32_t idx = 0;
        if (!ams::log_names::from_logfs_index(logfs_idx, kind, idx) ||
            !ams::log_names::format(path, sizeof path, kind, Stage::Crc, idx)) {
            return false;
        }
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
    std::uint32_t open_crc_ = 0;   // sealed CRC captured at open(), 0 = unknown
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
    // Vehicle-state gate: extraction is permitted only with the car
    // stopped and the TS off. g_state_telemetry is MainTask's FSM mirror.
    const auto vehicle_state = static_cast<ams::fsm::State>(g_state_telemetry);

    const std::uint16_t n = g_diag_disp.handle(g_diag_req, g_diag_req_len,
                                               ams::config::AmsNodeId,
                                               osKernelGetTickCount(),
                                               vehicle_state,
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

bool sd_imu_push(const ImuSample& s) noexcept {
    if (!g_imu_ring.push(s)) { ++g_imu_dropped; return false; }
    return true;
}

SdLogStats sd_log_stats() noexcept {
    return SdLogStats{ g_log_rows, g_log_dropped, g_log_files,
                       g_imu_rows, g_imu_dropped, g_log_state };
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
                ams::LogRecord scratch;        // keep the rings from wedging
                while (g_ring.pop(scratch)) { /* discard while cardless */ }
                ams::ImuSample imu_scratch;
                while (g_imu_ring.pop(imu_scratch)) { /* discard while cardless */ }
                continue;
            }
        }

        // (2) Ensure an active file is open.
        if (!g_file_open && !open_new_file(now)) { teardown(3); continue; }

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
            ++g_rows_this_file;
            ++g_log_rows;
            if (g_file_bytes >= ams::config::LogFileMaxBytes) {
                seal_file();                   // rotate; next file opens next tick
                break;
            }
        }

        // (3a) Drain the IMU ring -> IMUnnnn rows. Only while a LOG file is
        // open: the IMU file must share its index, so after a rotation the IMU
        // samples wait in the ring (2.5 s deep) until the next pair opens on
        // the following tick.
        if (g_mounted && g_file_open) {
            ams::ImuSample s;
            while (g_imu_ring.pop(s)) {
                if (!g_imu_open && !open_imu_file()) { teardown(3); break; }
                const std::size_t n =
                    ams::imu_csv::format_row(s, g_imu_rowbuf, sizeof g_imu_rowbuf);
                if (n == 0) continue;
                UINT bw = 0;
                if (f_write(&g_imu_fil, g_imu_rowbuf, n, &bw) != FR_OK || bw != n) {
                    teardown(3);
                    break;
                }
                g_imu_bytes += static_cast<std::uint32_t>(n);
                g_imu_crc    = ams::crc::update(g_imu_crc, g_imu_rowbuf, n);
                ++g_imu_rows_this_file;
                ++g_imu_rows;
                if (g_imu_bytes >= ams::config::LogFileMaxBytes) {
                    seal_file();               // rotate the pair
                    break;
                }
            }
        }

        // (3b) Time-based rotation. Without this a file is only sealed on the
        // size cap (~13 min of rows), so an ordinary bench session ends leaving
        // a.TMP that no tool treats as a finished log. Checked outside the
        // drain loop so it still fires during a lull in the ring.
        //
        // Either half can trigger it, and it always seals both. With no BMS
        // the LOG file gets no rows (log_csv::sample_due), so the IMU rows are
        // what rotate the pair, and each window leaves a header-only LOG file
        // beside its IMU file. That keeps the one-index-one-window pairing.
        if (g_file_open) {
            const std::uint32_t age = ams::log_rotation::file_age_ms(now, g_file_open_ms);
            if (ams::log_rotation::should_rotate(g_file_bytes, g_rows_this_file, age) ||
                (g_imu_open &&
                 ams::log_rotation::should_rotate(g_imu_bytes, g_imu_rows_this_file, age))) {
                seal_file();                   // next pair opens next tick
            }
        }

        // (4) Periodic flush -- bounds data lost on a power-cut to one window.
        if (g_file_open && (now - last_sync) >= ams::config::LogSyncPeriodMs) {
            last_sync = now;
            if (f_sync(&g_fil) != FR_OK ||
                (g_imu_open && f_sync(&g_imu_fil) != FR_OK)) {
                teardown(3);
            }
        }
    }
}
