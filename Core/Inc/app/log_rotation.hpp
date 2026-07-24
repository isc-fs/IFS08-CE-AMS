// SPDX-License-Identifier: proprietary
//
// Log file rotation + index-selection policy.
//
// Split out of sd_logger_task.cpp so the two rules that actually lost data can
// be tested without a card:
//
//  1. WHICH INDEX a fresh run takes. This used to consider only LOGnnnn.CSV,
//     so a run that ended before the size cap left LOGnnnn.TMP behind, the next
//     boot picked the SAME index, and open_new_file's FA_CREATE_ALWAYS
//     truncated the whole previous run. An orphan .TMP must make an index
//     taken, and should be sealed rather than skipped -- it is from a run that
//     is over and never coming back, so promoting it to .CSV is always right.
//
//  2. WHEN to rotate. Sealing only on the size cap means ~13 min of rows before
//     a file is ever finished, so ordinary sessions ended with nothing but a
//     .TMP -- which no tool (including the #406 extractor) treats as a log.
//
// Pure logic: the filesystem is reached only through caller-supplied callbacks.

#pragma once

#include "ams_config.hpp"

#include <cstdint>

namespace ams::log_rotation {

// Upper bound on the index scan. FAT 8.3 + "LOG%04lu" gives 0000-9999.
inline constexpr std::uint32_t MaxIndex = 10000u;

// True if the active file should be sealed now.
//
// `rows` gates the time rule so a stalled producer cannot litter the card with
// header-only files; the size rule needs no such guard because reaching it
// means rows were written.
[[nodiscard]] inline bool should_rotate(std::uint32_t bytes,
                                        std::uint32_t rows,
                                        std::uint32_t age_ms) noexcept {
    if (bytes >= config::LogFileMaxBytes) return true;
    return rows > 0u && age_ms >= config::LogFileMaxMs;
}

// Underflow-safe age of the active file. `open` is the tick the file was
// opened, `now` the current tick. If `open` is briefly AHEAD of `now` -- the
// #495 bug, where `open` was sampled AFTER the hundreds-of-ms SD open while
// `now` was sampled at the top of the loop -- the naive `now - open` unsigned
// subtraction wraps to ~4.29e9 ms, sails past LogFileMaxMs, and seals the file
// on its first rows every iteration (hundreds of 1-2 row files per run). Clamp
// that inverted pair to 0 so a skewed sample can never force an early rotation.
// A never-opened file has open == 0, so age == now behaves as intended.
[[nodiscard]] inline std::uint32_t file_age_ms(std::uint32_t now,
                                               std::uint32_t open) noexcept {
    return (now >= open) ? (now - open) : 0u;
}

// Pick the index for a new file.
//
//   exists(idx, sealed) -> true if LOGidx.CSV (sealed=true) / .TMP (false) is
//                          present on the card
//   seal(idx)           -> rename LOGidx.TMP to LOGidx.CSV; result advisory
//
// Returns the lowest index with neither file present. A failed seal still
// consumes the index: handing it back is precisely the truncation bug.
template <class Exists, class Seal>
[[nodiscard]] std::uint32_t next_index(Exists exists, Seal seal) noexcept {
    for (std::uint32_t i = 0; i < MaxIndex; ++i) {
        if (exists(i, true))  continue;          // sealed .CSV -> taken
        if (exists(i, false)) {                  // orphan .TMP -> seal, taken
            (void)seal(i);
            continue;
        }
        return i;
    }
    return 0;   // card full of logs -> wrap (best-effort)
}

}  // namespace ams::log_rotation
