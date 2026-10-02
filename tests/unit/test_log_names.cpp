// SPDX-License-Identifier: proprietary
//
// Tests for log_names.hpp -- the on-card names of a LOG/IMU/CEL/ELE file set and
// the LOGFS indices the extractor sees them under.

#include "log_names.hpp"

#include "unity.h"

#include <cstdint>

using ams::log_names::Kind;
using ams::log_names::Stage;

extern "C" void test_lognames_format_set(void) {
    char b[16];
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Active, 3));
    TEST_ASSERT_EQUAL_STRING("LOG0003.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Sealed, 3));
    TEST_ASSERT_EQUAL_STRING("LOG0003.CSV", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Active, 3));
    TEST_ASSERT_EQUAL_STRING("IMU0003.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Sealed, 9999));
    TEST_ASSERT_EQUAL_STRING("IMU9999.BIN", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Crc, 42));
    TEST_ASSERT_EQUAL_STRING("IMU0042.CRC", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Cel, Stage::Active, 7));
    TEST_ASSERT_EQUAL_STRING("CEL0007.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Cel, Stage::Sealed, 7));
    TEST_ASSERT_EQUAL_STRING("CEL0007.BIN", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Cel, Stage::Crc, 7));
    TEST_ASSERT_EQUAL_STRING("CEL0007.CRC", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Ele, Stage::Active, 8));
    TEST_ASSERT_EQUAL_STRING("ELE0008.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Ele, Stage::Sealed, 8));
    TEST_ASSERT_EQUAL_STRING("ELE0008.BIN", b);
}

extern "C" void test_lognames_format_rejects_out_of_range(void) {
    char b[16];
    TEST_ASSERT_FALSE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Sealed, 10000));
    char tiny[4];
    TEST_ASSERT_FALSE(ams::log_names::format(tiny, sizeof tiny, Kind::Log, Stage::Sealed, 1));
}

extern "C" void test_lognames_parse_sealed_all_kinds(void) {
    std::uint16_t idx = 0;
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("LOG0003.CSV", idx));
    TEST_ASSERT_EQUAL_HEX16(0x0003, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("IMU0003.BIN", idx));
    TEST_ASSERT_EQUAL_HEX16(0x8003, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("IMU9999.BIN", idx));
    TEST_ASSERT_EQUAL_HEX16(0x8000 | 9999, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("CEL0003.BIN", idx));
    TEST_ASSERT_EQUAL_HEX16(0x4003, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("ELE0003.BIN", idx));
    TEST_ASSERT_EQUAL_HEX16(0xC003, idx);
}

// Growing files, sidecars and strangers must never be listed.
extern "C" void test_lognames_parse_rejects_non_logs(void) {
    std::uint16_t idx = 0;
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("LOG0003.TMP", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU0003.TMP", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU0003.CRC", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("CEL0003.TMP", idx));
    // Each kind has exactly one sealed extension.
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU0003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("CEL0003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("ELE0003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("LOG0003.BIN", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("LOGX003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("GPS0003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU003.CSV", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("", idx));
}

extern "C" void test_lognames_logfs_index_round_trip(void) {
    Kind kind;
    std::uint32_t idx = 0;
    TEST_ASSERT_TRUE(ams::log_names::from_logfs_index(
        ams::log_names::logfs_index(Kind::Imu, 17), kind, idx));
    TEST_ASSERT_TRUE(kind == Kind::Imu);
    TEST_ASSERT_EQUAL_UINT32(17u, idx);

    TEST_ASSERT_TRUE(ams::log_names::from_logfs_index(
        ams::log_names::logfs_index(Kind::Log, 9999), kind, idx));
    TEST_ASSERT_TRUE(kind == Kind::Log);
    TEST_ASSERT_EQUAL_UINT32(9999u, idx);

    TEST_ASSERT_TRUE(ams::log_names::from_logfs_index(
        ams::log_names::logfs_index(Kind::Cel, 9999), kind, idx));
    TEST_ASSERT_TRUE(kind == Kind::Cel);
    TEST_ASSERT_EQUAL_UINT32(9999u, idx);

    TEST_ASSERT_TRUE(ams::log_names::from_logfs_index(0xC000 | 3, kind, idx));
    TEST_ASSERT_TRUE(kind == Kind::Ele);
    TEST_ASSERT_EQUAL_UINT32(3u, idx);

    // Indices no file can have: past 9999.
    TEST_ASSERT_FALSE(ams::log_names::from_logfs_index(10000, kind, idx));
    TEST_ASSERT_FALSE(ams::log_names::from_logfs_index(0x8000 | 10000, kind, idx));
    TEST_ASSERT_FALSE(ams::log_names::from_logfs_index(0xC000 | 10000, kind, idx));
}

// The ranges cannot overlap: the largest rotation index fits under the kind bits.
extern "C" void test_lognames_ranges_disjoint(void) {
    TEST_ASSERT_TRUE(ams::log_names::MaxIndex - 1u <= ams::log_names::IndexMask);
    TEST_ASSERT_EQUAL_HEX16(0x4000 | 9999,
        ams::log_names::logfs_index(Kind::Cel, ams::log_names::MaxIndex - 1));
    TEST_ASSERT_EQUAL_HEX16(0x8000 | 9999,
        ams::log_names::logfs_index(Kind::Imu, ams::log_names::MaxIndex - 1));
    TEST_ASSERT_EQUAL_HEX16(0xC000 | 9999,
        ams::log_names::logfs_index(Kind::Ele, ams::log_names::MaxIndex - 1));
}
