// SPDX-License-Identifier: proprietary
//
// SdLoggerTask -- datalogging consumer. Low-priority FreeRTOS task that owns
// the microSD: non-fatal lazy mount + retry, drains the lock-free LogRing,
// and streams CSV to rotated/sealed files. Strictly off the safety path --
// it never blocks or faults the AMS; an absent/failed card just stops logging.
//
// Producer side (SafetyTask) only ever calls sd_log_push(). Wiring: the
// boot-path MX_SDMMC1_SD_Init() is decoupled in CubeMX so an absent card
// cannot brick the node (#407); this task configures hsd1 and mounts lazily.
//
// Dual-use header: the C++ producer/consumer API is gated behind __cplusplus
// so the C-compiled CubeMX main.c can include it just for the trampoline
// declaration (same role as app/safety_task.h etc.).

#pragma once

#ifdef __cplusplus

#include "log_record.hpp"

namespace ams {

// Producer API. SafetyTask pushes one record per LogSamplePeriodMs. Wait-free
// and best-effort: returns false (and counts a drop) if the ring is full.
// NEVER blocks -- safe to call from the realtime safety loop.
bool sd_log_push(const LogRecord& rec) noexcept;

// Lightweight logger health, for a future diag/health frame (e.g. #406 host
// status). Plain snapshot of the file-local counters.
struct SdLogStats {
    std::uint32_t rows;     // CSV rows written to the card
    std::uint32_t dropped;  // records dropped (ring full -- SD stall/pull)
    std::uint32_t files;    // files sealed (rotated to .CSV)
    std::uint8_t  state;    // 0=boot 1=no_card/not_ready 2=logging 3=io_error
};
SdLogStats sd_log_stats() noexcept;

// ---------------------------------------------------------------------------
// LOGFS diag transport hooks (#406 / #439).
//
// The CAN side reassembles an ISO-TP message and hands it here; this task
// executes it against FatFs and leaves the reply for collection. All
// filesystem work therefore happens on ONE thread -- see logfs_server.hpp for
// why that is preferred even though FatFs is reentrant in this build.
//
// Both calls are non-blocking, so the CAN task never waits on the SD card.
// ---------------------------------------------------------------------------

// Post a reassembled request. False if one is already in flight (the caller
// should answer NACK BUSY, 0x08) or the logger has not started yet.
bool sd_diag_submit(const std::uint8_t* msg, std::uint16_t len) noexcept;

// Collect the reply, or 0 if it is not ready yet. Clears the slot.
std::uint16_t sd_diag_collect(std::uint8_t* out, std::uint16_t cap) noexcept;

}  // namespace ams

extern "C" {
#endif  // __cplusplus

// FreeRTOS trampoline -- the CubeMX SdLoggerTask thread entry calls this.
void ams_sd_logger_task_run(void *argument);

#ifdef __cplusplus
}
#endif
