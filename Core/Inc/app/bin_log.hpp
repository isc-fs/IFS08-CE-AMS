// SPDX-License-Identifier: proprietary
//
// Binary companion log files -- IMUnnnn.BIN and CELnnnn.BIN -- written next to
// LOGnnnn.CSV for the streams that are too fast or too wide for CSV.
//
// A file is a 512-byte self-describing header followed by fixed-size,
// little-endian records with no separators. The header carries a plain-text
// schema, so tools/log_decode.py turns any of these files into CSV without
// knowing the stream in advance.
//
// Header (all offsets fixed, little-endian):
//    0  char[8]  magic "AMSBIN1\0"
//    8  u16      header format version (FormatVersion)
//   10  u16      record size in bytes
//   12  u32      rotation index, shared with LOGnnnn.CSV
//   16  u32      tick_ms when the file was opened (same clock as tick_ms in LOG)
//   20  char[8]  stream name, NUL-padded ("IMU", "CEL")
//   28  u8[3]    firmware version major, minor, patch
//   31  u8       reserved, 0
//   32  u8[4]    firmware git hash, first 4 bytes
//   36  u8[28]   reserved, 0
//   64  char[448] schema, NUL-terminated
//
// Schema: one line per field, in record order, no padding between fields:
//
//   <name> <type> <count> <scale> <unit> [<flags>]
//
//   type   u8 i8 u16 i16 u32 i32
//   count  1, N (columns name0..nameN-1) or AxB (columns nameA_B, row-major)
//   scale  multiplier to the unit: a decimal or a ratio "a/b"
//   unit   free text, "-" if none
//   flags  "z" = a raw 0 means "not measured" and decodes to an empty field
//
// The record structs below are laid out with no implicit padding and the
// static_asserts pin every offset, so the schema text and the struct cannot
// drift apart without breaking the build.
//
// TELEMETRY ONLY. Nothing on the safety path reads these records.
// Pure (HAL-free, RTOS-free) so the host tests cover it.

#pragma once

#include "ams_config.hpp"
#include "imu_record.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ams::bin_log {

inline constexpr std::size_t   HeaderBytes   = 512;
inline constexpr char          Magic[8]      = "AMSBIN1";
inline constexpr std::uint16_t FormatVersion = 1;

inline constexpr std::size_t   SchemaOffset  = 64;
inline constexpr std::size_t   SchemaMax     = HeaderBytes - SchemaOffset;   // incl. NUL

struct HeaderInfo {
    const char*   stream;        // <= 7 chars
    const char*   schema;
    std::uint16_t record_size;
    std::uint32_t file_index;
    std::uint32_t open_tick_ms;
    std::uint8_t  fw_version[3];
    std::uint8_t  git_hash[4];
};

namespace detail {
inline void put_u16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}
inline void put_u32(std::uint8_t* p, std::uint32_t v) noexcept {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}
}  // namespace detail

// Fill `out` (exactly HeaderBytes) with the header. False if the stream name
// or the schema does not fit, in which case `out` is left zeroed.
inline bool build_header(std::uint8_t* out, std::size_t cap, const HeaderInfo& h) noexcept {
    if (out == nullptr || cap < HeaderBytes || h.stream == nullptr || h.schema == nullptr) {
        return false;
    }
    std::memset(out, 0, HeaderBytes);
    const std::size_t sn = std::strlen(h.stream);
    const std::size_t cn = std::strlen(h.schema);
    if (sn >= 8u || cn >= SchemaMax) return false;

    std::memcpy(out, Magic, sizeof Magic);
    detail::put_u16(out + 8,  FormatVersion);
    detail::put_u16(out + 10, h.record_size);
    detail::put_u32(out + 12, h.file_index);
    detail::put_u32(out + 16, h.open_tick_ms);
    std::memcpy(out + 20, h.stream, sn);
    std::memcpy(out + 28, h.fw_version, 3);
    std::memcpy(out + 32, h.git_hash, 4);
    std::memcpy(out + SchemaOffset, h.schema, cn);
    return true;
}

// ---------------------------------------------------------------------------
// IMUnnnn.BIN -- one ImuSample (imu_record.hpp) per record, raw counts.
// Scales match bmi088::acc_g_e4 / gyr_rad_s_e4: +/-6 g and +/-500 dps over
// +/-32768 counts.
// ---------------------------------------------------------------------------
inline constexpr char ImuStream[] = "IMU";
inline constexpr char ImuSchema[] =
    "tick_ms u32 1 1 ms\n"
    "a i16 3 6/32768 g\n"
    "g i16 3 8.726646259971648/32768 rad/s\n";

static_assert(offsetof(ImuSample, tick_ms) == 0,  "IMU schema: tick_ms");
static_assert(offsetof(ImuSample, acc)     == 4,  "IMU schema: a");
static_assert(offsetof(ImuSample, gyr)     == 10, "IMU schema: g");
static_assert(sizeof(ImuSample)            == 16, "IMU schema: record size");

// ---------------------------------------------------------------------------
// CELnnnn.BIN -- one record per cell-voltage read (every ADCV + RDCVA..D that
// completed on the bus), including retries within a poll.
//
// Cells of an IC that failed PEC on this read are 0, not the previous value,
// so every non-empty cell in a record was converted at t_adcv_ms. The LTC6811
// converts all cells of the chain within ~1 ms of the ADCV broadcast.
//
// i_mA is the latest pack-current sample when the read completed (~7 ms after
// ADCV) and i_tick_ms when it was taken; t_adcv_ms - i_tick_ms says how far
// apart the two are. Current is sampled every CurrentPeriodMs (50 ms), so they
// can be up to ~50 ms apart.
// ---------------------------------------------------------------------------
struct CelFrame {
    std::uint32_t t_adcv_ms;   // tick when ADCV was issued
    std::uint32_t i_tick_ms;   // tick of the current sample in i_mA
    std::int32_t  i_mA;        // raw pack current, + = discharge
    std::uint16_t seq;         // +1 per frame; a gap means frames were dropped
    std::uint16_t ltc_ok;      // bit k = chain IC k PEC-clean on this read
    std::uint8_t  attempt;     // 0 = first read of a poll, >0 = retry
    std::uint8_t  flags;       // CelFlag bits
    std::uint16_t cell_mV[config::BmsModuleCount * config::CellsPerModule];
};

namespace cel_flag {
inline constexpr std::uint8_t BalanceQuiesced = 1u << 0;   // bleed FETs were off for this read
inline constexpr std::uint8_t CurrentFault    = 1u << 1;   // i_mA came from a faulted sensor
}  // namespace cel_flag

inline constexpr char CelStream[] = "CEL";
inline constexpr char CelSchema[] =
    "t_adcv_ms u32 1 1 ms\n"
    "i_tick_ms u32 1 1 ms\n"
    "i i32 1 0.001 A\n"
    "seq u16 1 1 -\n"
    "ltc_ok u16 1 1 -\n"
    "attempt u8 1 1 -\n"
    "flags u8 1 1 -\n"
    "c u16 5x19 1 mV z\n";

static_assert(config::BmsModuleCount == 5 && config::CellsPerModule == 19,
              "CEL schema hard-codes the 5x19 cell matrix");
static_assert(offsetof(CelFrame, t_adcv_ms) == 0,  "CEL schema: t_adcv_ms");
static_assert(offsetof(CelFrame, i_tick_ms) == 4,  "CEL schema: i_tick_ms");
static_assert(offsetof(CelFrame, i_mA)      == 8,  "CEL schema: i");
static_assert(offsetof(CelFrame, seq)       == 12, "CEL schema: seq");
static_assert(offsetof(CelFrame, ltc_ok)    == 14, "CEL schema: ltc_ok");
static_assert(offsetof(CelFrame, attempt)   == 16, "CEL schema: attempt");
static_assert(offsetof(CelFrame, flags)     == 17, "CEL schema: flags");
static_assert(offsetof(CelFrame, cell_mV)   == 18, "CEL schema: c");
static_assert(sizeof(CelFrame)              == 208, "CEL schema: record size");

static_assert(sizeof ImuSchema <= SchemaMax && sizeof CelSchema <= SchemaMax,
              "schema does not fit the header");

}  // namespace ams::bin_log
