// SPDX-License-Identifier: proprietary
//
// Tests for log_names.hpp -- the on-card names of a LOG/IMU file pair and the
// LOGFS indices the extractor sees them under.

#include "log_names.hpp"

#include "unity.h"

#include <cstdint>

using ams::log_names::Kind;
using ams::log_names::Stage;

extern "C" void test_lognames_format_pair(void) {
    char b[16];
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Active, 3));
    TEST_ASSERT_EQUAL_STRING("LOG0003.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Sealed, 3));
    TEST_ASSERT_EQUAL_STRING("LOG0003.CSV", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Active, 3));
    TEST_ASSERT_EQUAL_STRING("IMU0003.TMP", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Sealed, 9999));
    TEST_ASSERT_EQUAL_STRING("IMU9999.CSV", b);
    TEST_ASSERT_TRUE(ams::log_names::format(b, sizeof b, Kind::Imu, Stage::Crc, 42));
    TEST_ASSERT_EQUAL_STRING("IMU0042.CRC", b);
}

extern "C" void test_lognames_format_rejects_out_of_range(void) {
    char b[16];
    TEST_ASSERT_FALSE(ams::log_names::format(b, sizeof b, Kind::Log, Stage::Sealed, 10000));
    char tiny[4];
    TEST_ASSERT_FALSE(ams::log_names::format(tiny, sizeof tiny, Kind::Log, Stage::Sealed, 1));
}

extern "C" void test_lognames_parse_sealed_both_kinds(void) {
    std::uint16_t idx = 0;
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("LOG0003.CSV", idx));
    TEST_ASSERT_EQUAL_HEX16(0x0003, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("IMU0003.CSV", idx));
    TEST_ASSERT_EQUAL_HEX16(0x8003, idx);
    TEST_ASSERT_TRUE(ams::log_names::parse_sealed("IMU9999.CSV", idx));
    TEST_ASSERT_EQUAL_HEX16(0x8000 | 9999, idx);
}

// Growing files, sidecars and strangers must never be listed.
extern "C" void test_lognames_parse_rejects_non_logs(void) {
    std::uint16_t idx = 0;
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("LOG0003.TMP", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU0003.TMP", idx));
    TEST_ASSERT_FALSE(ams::log_names::parse_sealed("IMU0003.CRC", idx));
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

    // Indices no file can have.
    TEST_ASSERT_FALSE(ams::log_names::from_logfs_index(10000, kind, idx));
    TEST_ASSERT_FALSE(ams::log_names::from_logfs_index(0x8000 | 10000, kind, idx));
}

// The two ranges cannot overlap: the largest LOG index is below the IMU flag.
extern "C" void test_lognames_ranges_disjoint(void) {
    TEST_ASSERT_TRUE(ams::log_names::logfs_index(Kind::Log, ams::log_names::MaxIndex - 1) <
                     ams::log_names::ImuIndexFlag);
}
