// SPDX-License-Identifier: proprietary
//
// On-card file names and their LOGFS indices.
//
// Each rotation index nnnn owns a PAIR of files covering the same time window:
//
//   LOGnnnn.TMP / .CSV / .CRC   AMS state rows (log_record.hpp)
//   IMUnnnn.TMP / .CSV / .CRC   IMU rows       (imu_record.hpp)
//
// LOGFS addresses a sealed file by a u16 index. LOG files keep the plain
// rotation index (0..9999); IMU files use the same number with bit 15 set, so
// IMU0003.CSV is LOGFS index 0x8003. The host treats the index as opaque and
// names the pulled file from the LIST entry, so it needs no change to fetch
// IMU files. 9999 < 0x8000, so the two ranges cannot collide.
//
// Pure: no FatFs, host-testable.

#pragma once

#include "ams_config.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ams::log_names {

enum class Kind : std::uint8_t { Log, Imu };
enum class Stage : std::uint8_t { Active, Sealed, Crc };

inline constexpr std::uint16_t ImuIndexFlag = 0x8000u;
inline constexpr std::uint32_t MaxIndex     = 10000u;   // 4 decimal digits

// Write the 8.3 name for (kind, stage, idx) into buf. Returns false if it did
// not fit or idx is out of range.
inline bool format(char* buf, std::size_t cap, Kind kind, Stage stage,
                   std::uint32_t idx) noexcept {
    if (idx >= MaxIndex) return false;
    const char* fmt = nullptr;
    if (kind == Kind::Log) {
        fmt = (stage == Stage::Active) ? config::LogActiveNameFmt
            : (stage == Stage::Sealed) ? config::LogSealedNameFmt
                                       : config::LogCrcNameFmt;
    } else {
        fmt = (stage == Stage::Active) ? config::ImuActiveNameFmt
            : (stage == Stage::Sealed) ? config::ImuSealedNameFmt
                                       : config::ImuCrcNameFmt;
    }
    const int n = std::snprintf(buf, cap, fmt, static_cast<unsigned long>(idx));
    return n > 0 && static_cast<std::size_t>(n) < cap;
}

// LOGFS index for a sealed file of `kind` at rotation index `idx`.
[[nodiscard]] inline constexpr std::uint16_t logfs_index(Kind kind,
                                                         std::uint32_t idx) noexcept {
    return static_cast<std::uint16_t>(
        (kind == Kind::Imu ? ImuIndexFlag : 0u) | (idx & 0x7FFFu));
}

// Inverse of logfs_index. False for an index no file could have.
inline bool from_logfs_index(std::uint16_t logfs_idx, Kind& kind,
                             std::uint32_t& idx) noexcept {
    kind = (logfs_idx & ImuIndexFlag) ? Kind::Imu : Kind::Log;
    idx  = logfs_idx & 0x7FFFu;
    return idx < MaxIndex;
}

// "LOGnnnn.CSV" -> nnnn, "IMUnnnn.CSV" -> 0x8000 | nnnn. Rejects the growing
// .TMP, the .CRC sidecar and anything else on the card.
inline bool parse_sealed(const char* name, std::uint16_t& logfs_idx) noexcept {
    if (std::strlen(name) != 11u) return false;
    Kind kind;
    if      (std::strncmp(name, "LOG", 3) == 0) kind = Kind::Log;
    else if (std::strncmp(name, "IMU", 3) == 0) kind = Kind::Imu;
    else return false;
    if (std::strcmp(name + 7, ".CSV") != 0) return false;
    std::uint32_t v = 0;
    for (int i = 3; i < 7; ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
        v = v * 10u + static_cast<std::uint32_t>(name[i] - '0');
    }
    logfs_idx = logfs_index(kind, v);
    return true;
}

}  // namespace ams::log_names
