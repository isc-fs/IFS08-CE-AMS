// SPDX-License-Identifier: proprietary
//
// Binary companion log files -- IMUnnnn.BIN, CELnnnn.BIN and ELEnnnn.BIN --
// written next to LOGnnnn.CSV for the streams that are too fast or too wide
// for CSV.
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
//   20  char[8]  stream name, NUL-padded ("IMU", "CEL", "ELE")
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
// i_mA is the mean pack current over the conversion itself: the oversampled
// ADC samples from the moment ADCV was issued through config::CelCurrentSyncUs
// (~2.3 ms), so voltage and current describe the same instant and
// dV/dI between frames gives each cell's resistance. i_n is how many samples
// that mean covers and i_span_us the time they span; a short window means the
// capture ended inside the conversion. i_n = 0 means no sync was available
// (no capture running, or the samples already reused) and i_mA is then
// CurrentService's latest 50 ms mean instead.
// ---------------------------------------------------------------------------
struct CelFrame {
    std::uint32_t t_adcv_ms;   // tick when ADCV was issued
    std::uint16_t i_n;         // ADC samples in i_mA; 0 = no sync (50 ms mean instead)
    std::uint16_t i_span_us;   // time those samples span
    std::int32_t  i_mA;        // pack current during the conversion, + = discharge
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
    "i_n u16 1 1 -\n"
    "i_span_us u16 1 1 us\n"
    "i i32 1 0.001 A\n"
    "seq u16 1 1 -\n"
    "ltc_ok u16 1 1 -\n"
    "attempt u8 1 1 -\n"
    "flags u8 1 1 -\n"
    "c u16 5x19 1 mV z\n";

static_assert(config::BmsModuleCount == 5 && config::CellsPerModule == 19,
              "CEL schema hard-codes the 5x19 cell matrix");
static_assert(offsetof(CelFrame, t_adcv_ms) == 0,  "CEL schema: t_adcv_ms");
static_assert(offsetof(CelFrame, i_n)       == 4,  "CEL schema: i_n");
static_assert(offsetof(CelFrame, i_span_us) == 6,  "CEL schema: i_span_us");
static_assert(offsetof(CelFrame, i_mA)      == 8,  "CEL schema: i");
static_assert(offsetof(CelFrame, seq)       == 12, "CEL schema: seq");
static_assert(offsetof(CelFrame, ltc_ok)    == 14, "CEL schema: ltc_ok");
static_assert(offsetof(CelFrame, attempt)   == 16, "CEL schema: attempt");
static_assert(offsetof(CelFrame, flags)     == 17, "CEL schema: flags");
static_assert(offsetof(CelFrame, cell_mV)   == 18, "CEL schema: c");
static_assert(sizeof(CelFrame)              == 208, "CEL schema: record size");

// ---------------------------------------------------------------------------
// ELEnnnn.BIN -- one record per 10 ms window of pack current (100 Hz).
//
// Each window reduces the oversampled ADC samples that fell inside it (each
// sample already an 80 us integration, see config::CurrentAdcFracBits): the
// mean, and the lowest and highest single sample, so a sub-millisecond peak
// survives even though the mean smooths it. n is the number of samples (125
// nominal at 12.5 kHz); a short window is the one that contained the
// disconnect check's ~0.1 ms pause, or the edge of a late capture.
//
// tick_ms is the end of the window, interpolated over the capture's measured
// duration (1 ms tick resolution). dcbus_V is the ECU's last 0x100 value and
// dcbus_age_ms how old it was -- the AMS cannot measure the link itself.
// ---------------------------------------------------------------------------
struct EleRecord {
    std::uint32_t tick_ms;       // end of the window
    std::uint16_t seq;           // +1 per record; a gap means records were dropped
    std::uint8_t  n;             // ADC samples in the window
    std::uint8_t  flags;         // ele_flag bits
    std::int32_t  i_mean_mA;     // + = discharge
    std::int32_t  i_min_mA;
    std::int32_t  i_max_mA;
    std::uint16_t dcbus_V;
    std::uint16_t dcbus_age_ms;  // saturating; 65535 = none received or older
};

namespace ele_flag {
inline constexpr std::uint8_t SensorFault = 1u << 0;   // debounced disconnect verdict was set
inline constexpr std::uint8_t Overrun     = 1u << 1;   // capture buffer filled: samples after it lost
}  // namespace ele_flag

inline constexpr char EleStream[] = "ELE";
inline constexpr char EleSchema[] =
    "tick_ms u32 1 1 ms\n"
    "seq u16 1 1 -\n"
    "n u8 1 1 -\n"
    "flags u8 1 1 -\n"
    "i_mean i32 1 0.001 A\n"
    "i_min i32 1 0.001 A\n"
    "i_max i32 1 0.001 A\n"
    "dcbus_V u16 1 1 V\n"
    "dcbus_age_ms u16 1 1 ms\n";

static_assert(offsetof(EleRecord, tick_ms)      == 0,  "ELE schema: tick_ms");
static_assert(offsetof(EleRecord, seq)          == 4,  "ELE schema: seq");
static_assert(offsetof(EleRecord, n)            == 6,  "ELE schema: n");
static_assert(offsetof(EleRecord, flags)        == 7,  "ELE schema: flags");
static_assert(offsetof(EleRecord, i_mean_mA)    == 8,  "ELE schema: i_mean");
static_assert(offsetof(EleRecord, i_min_mA)     == 12, "ELE schema: i_min");
static_assert(offsetof(EleRecord, i_max_mA)     == 16, "ELE schema: i_max");
static_assert(offsetof(EleRecord, dcbus_V)      == 20, "ELE schema: dcbus_V");
static_assert(offsetof(EleRecord, dcbus_age_ms) == 22, "ELE schema: dcbus_age_ms");
static_assert(sizeof(EleRecord)                 == 24, "ELE schema: record size");

static_assert(sizeof ImuSchema <= SchemaMax && sizeof CelSchema <= SchemaMax &&
              sizeof EleSchema <= SchemaMax,
              "schema does not fit the header");

}  // namespace ams::bin_log
