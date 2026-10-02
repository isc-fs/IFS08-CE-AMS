// SPDX-License-Identifier: proprietary
//
// On-card file names and their LOGFS indices.
//
// Each rotation index nnnn owns a SET of files covering the same time window:
//
//   LOGnnnn.TMP / .CSV / .CRC   AMS state rows (log_record.hpp)
//   IMUnnnn.TMP / .BIN / .CRC   IMU samples    (bin_log.hpp)
//   CELnnnn.TMP / .BIN / .CRC   cell frames    (bin_log.hpp)
//
// LOGFS addresses a sealed file by a u16 index: the rotation index (0..9999)
// in the low 14 bits and the kind in the top two -- 00 LOG, 10 IMU, 01 CEL --
// so IMU0003.BIN is LOGFS index 0x8003 and CEL0003.BIN is 0x4003. 9999 <
// 0x4000, so the ranges cannot collide; 11 is reserved for the next stream.
// The host treats the index as opaque and names the pulled file from the LIST
// entry, so it needs no change to fetch any of them.
//
// Pure: no FatFs, host-testable.

#pragma once

#include "ams_config.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ams::log_names {

enum class Kind : std::uint8_t { Log, Imu, Cel };
enum class Stage : std::uint8_t { Active, Sealed, Crc };

inline constexpr std::uint16_t ImuIndexFlag = 0x8000u;
inline constexpr std::uint16_t CelIndexFlag = 0x4000u;
inline constexpr std::uint16_t KindMask     = 0xC000u;
inline constexpr std::uint16_t IndexMask    = 0x3FFFu;
inline constexpr std::uint32_t MaxIndex     = 10000u;   // 4 decimal digits

// Write the 8.3 name for (kind, stage, idx) into buf. Returns false if it did
// not fit or idx is out of range.
inline bool format(char* buf, std::size_t cap, Kind kind, Stage stage,
                   std::uint32_t idx) noexcept {
    if (idx >= MaxIndex) return false;
    const char* fmt = nullptr;
    switch (kind) {
    case Kind::Log:
        fmt = (stage == Stage::Active) ? config::LogActiveNameFmt
            : (stage == Stage::Sealed) ? config::LogSealedNameFmt
                                       : config::LogCrcNameFmt;
        break;
    case Kind::Imu:
        fmt = (stage == Stage::Active) ? config::ImuActiveNameFmt
            : (stage == Stage::Sealed) ? config::ImuSealedNameFmt
                                       : config::ImuCrcNameFmt;
        break;
    case Kind::Cel:
        fmt = (stage == Stage::Active) ? config::CelActiveNameFmt
            : (stage == Stage::Sealed) ? config::CelSealedNameFmt
                                       : config::CelCrcNameFmt;
        break;
    }
    if (fmt == nullptr) return false;
    const int n = std::snprintf(buf, cap, fmt, static_cast<unsigned long>(idx));
    return n > 0 && static_cast<std::size_t>(n) < cap;
}

// LOGFS index for a sealed file of `kind` at rotation index `idx`.
[[nodiscard]] inline constexpr std::uint16_t logfs_index(Kind kind,
                                                         std::uint32_t idx) noexcept {
    const std::uint16_t flag = (kind == Kind::Imu) ? ImuIndexFlag
                             : (kind == Kind::Cel) ? CelIndexFlag
                                                   : 0u;
    return static_cast<std::uint16_t>(flag | (idx & IndexMask));
}

// Inverse of logfs_index. False for an index no file could have.
inline bool from_logfs_index(std::uint16_t logfs_idx, Kind& kind,
                             std::uint32_t& idx) noexcept {
    switch (logfs_idx & KindMask) {
    case 0u:           kind = Kind::Log; break;
    case ImuIndexFlag: kind = Kind::Imu; break;
    case CelIndexFlag: kind = Kind::Cel; break;
    default:           return false;          // reserved
    }
    idx = logfs_idx & IndexMask;
    return idx < MaxIndex;
}

// "LOGnnnn.CSV" -> nnnn, "IMUnnnn.BIN" -> 0x8000 | nnnn, "CELnnnn.BIN" ->
// 0x4000 | nnnn. Rejects the growing .TMP, the .CRC sidecar, a kind with the
// wrong extension and anything else on the card.
inline bool parse_sealed(const char* name, std::uint16_t& logfs_idx) noexcept {
    if (std::strlen(name) != 11u) return false;
    Kind        kind;
    const char* ext;
    if      (std::strncmp(name, "LOG", 3) == 0) { kind = Kind::Log; ext = ".CSV"; }
    else if (std::strncmp(name, "IMU", 3) == 0) { kind = Kind::Imu; ext = ".BIN"; }
    else if (std::strncmp(name, "CEL", 3) == 0) { kind = Kind::Cel; ext = ".BIN"; }
    else return false;
    if (std::strcmp(name + 7, ext) != 0) return false;
    std::uint32_t v = 0;
    for (int i = 3; i < 7; ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
        v = v * 10u + static_cast<std::uint32_t>(name[i] - '0');
    }
    logfs_idx = logfs_index(kind, v);
    return true;
}

}  // namespace ams::log_names
