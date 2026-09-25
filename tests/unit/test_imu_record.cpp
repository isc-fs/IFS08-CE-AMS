// SPDX-License-Identifier: proprietary
//
// Tests for imu_record.hpp -- BMI088 raw decoding, unit scaling and the
// IMUnnnn.CSV row format. The bus side (ImuTask) needs hardware; everything
// that turns bytes into logged numbers is checked here.

#include "imu_record.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

extern "C" void test_imu_decode_axes_little_endian_signed(void) {
    const std::uint8_t raw[6] = {0x34, 0x12, 0xFF, 0xFF, 0x00, 0x80};
    std::int16_t out[3] = {};
    ams::bmi088::decode_axes(raw, out);
    TEST_ASSERT_EQUAL_INT(0x1234, out[0]);
    TEST_ASSERT_EQUAL_INT(-1, out[1]);
    TEST_ASSERT_EQUAL_INT(-32768, out[2]);
}

// +/-6 g full scale: 32768 counts = 6000 mg.
extern "C" void test_imu_acc_mg_scaling(void) {
    TEST_ASSERT_EQUAL_INT(0, ams::bmi088::acc_mg(0));
    TEST_ASSERT_EQUAL_INT(6000, ams::bmi088::acc_mg(32767));    // 5999.8 rounds up
    TEST_ASSERT_EQUAL_INT(-6000, ams::bmi088::acc_mg(-32768));
    TEST_ASSERT_EQUAL_INT(1000, ams::bmi088::acc_mg(5461));     // ~1 g
    TEST_ASSERT_EQUAL_INT(-1000, ams::bmi088::acc_mg(-5461));   // symmetric rounding
}

// +/-500 dps full scale: 32768 counts = 500000 mdps. Needs 64-bit products.
extern "C" void test_imu_gyr_mdps_scaling(void) {
    TEST_ASSERT_EQUAL_INT(0, ams::bmi088::gyr_mdps(0));
    TEST_ASSERT_EQUAL_INT(15, ams::bmi088::gyr_mdps(1));        // 15.26 mdps/count
    TEST_ASSERT_EQUAL_INT(-15, ams::bmi088::gyr_mdps(-1));
    TEST_ASSERT_EQUAL_INT(499985, ams::bmi088::gyr_mdps(32767));
    TEST_ASSERT_EQUAL_INT(-500000, ams::bmi088::gyr_mdps(-32768));
}

extern "C" void test_imu_csv_row_values_and_columns(void) {
    ams::ImuSample s{};
    s.tick_ms = 123;
    s.acc[0] = 5461; s.acc[1] = 0; s.acc[2] = -32768;
    s.gyr[0] = 1;    s.gyr[1] = -1; s.gyr[2] = -32768;
    char buf[ams::imu_csv::MaxRowBytes];
    const std::size_t n = ams::imu_csv::format_row(s, buf, sizeof buf);
    TEST_ASSERT_EQUAL_STRING("123,1000,0,-6000,15,-15,-500000\n", buf);
    TEST_ASSERT_EQUAL_UINT(std::strlen(buf), n);

    // Same number of fields as the header.
    int row_commas = 0, hdr_commas = 0;
    for (const char* p = buf; *p; ++p) row_commas += (*p == ',');
    for (const char* p = ams::imu_csv::Header; *p; ++p) hdr_commas += (*p == ',');
    TEST_ASSERT_EQUAL_INT(hdr_commas, row_commas);
}

// The widest possible row must fit the buffer the logger allocates.
extern "C" void test_imu_csv_widest_row_fits(void) {
    ams::ImuSample s{};
    s.tick_ms = 0xFFFFFFFFu;
    for (int i = 0; i < 3; ++i) { s.acc[i] = -32768; s.gyr[i] = -32768; }
    char buf[ams::imu_csv::MaxRowBytes];
    TEST_ASSERT_GREATER_THAN(0u, ams::imu_csv::format_row(s, buf, sizeof buf));
}

extern "C" void test_imu_csv_truncation_returns_zero(void) {
    ams::ImuSample s{};
    char tiny[8];
    TEST_ASSERT_EQUAL_INT(0, (int)ams::imu_csv::format_row(s, tiny, sizeof tiny));
}

// The low-pass corners must sit below the 50 Hz Nyquist of the log rate, or
// taking the newest sample every period aliases. Pinned so a config edit that
// breaks the reasoning in imu_record.hpp fails here.
extern "C" void test_imu_sensor_config_matches_log_rate(void) {
    TEST_ASSERT_EQUAL_UINT(10u, ams::config::ImuSamplePeriodMs);    // 100 Hz log
    TEST_ASSERT_EQUAL_HEX8(0x8A, ams::bmi088::AccConf);             // OSR4 @ 400 Hz -> 37 Hz
    TEST_ASSERT_EQUAL_HEX8(0x03, ams::bmi088::GyrBw400Hz47Hz);      // 400 Hz ODR, 47 Hz
}
