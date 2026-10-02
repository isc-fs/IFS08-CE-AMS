// SPDX-License-Identifier: proprietary
//
// Tests for imu_record.hpp -- BMI088 raw decoding and unit scaling. The bus
// side (ImuTask) needs hardware; everything that turns bytes into numbers is
// checked here. The IMUnnnn.BIN record layout is covered by test_bin_log.cpp.

#include "imu_record.hpp"

#include "unity.h"

#include <cstdint>

extern "C" void test_imu_decode_axes_little_endian_signed(void) {
    const std::uint8_t raw[6] = {0x34, 0x12, 0xFF, 0xFF, 0x00, 0x80};
    std::int16_t out[3] = {};
    ams::bmi088::decode_axes(raw, out);
    TEST_ASSERT_EQUAL_INT(0x1234, out[0]);
    TEST_ASSERT_EQUAL_INT(-1, out[1]);
    TEST_ASSERT_EQUAL_INT(-32768, out[2]);
}

// +/-6 g full scale, 1e-4 g units: 32768 counts = 60000 (6.0000 g).
extern "C" void test_imu_acc_g_scaling(void) {
    TEST_ASSERT_EQUAL_INT(0, ams::bmi088::acc_g_e4(0));
    TEST_ASSERT_EQUAL_INT(30000, ams::bmi088::acc_g_e4(16384));    // exactly 3 g
    TEST_ASSERT_EQUAL_INT(59998, ams::bmi088::acc_g_e4(32767));
    TEST_ASSERT_EQUAL_INT(-60000, ams::bmi088::acc_g_e4(-32768));
    TEST_ASSERT_EQUAL_INT(2, ams::bmi088::acc_g_e4(1));            // 1.83e-4 g/count
    TEST_ASSERT_EQUAL_INT(-2, ams::bmi088::acc_g_e4(-1));          // symmetric rounding
}

// +/-500 dps full scale in 1e-4 rad/s: 32768 counts = 8.7266 rad/s.
extern "C" void test_imu_gyr_rad_s_scaling(void) {
    TEST_ASSERT_EQUAL_INT(0, ams::bmi088::gyr_rad_s_e4(0));
    TEST_ASSERT_EQUAL_INT(3, ams::bmi088::gyr_rad_s_e4(1));         // 2.66e-4 rad/s/count
    TEST_ASSERT_EQUAL_INT(-3, ams::bmi088::gyr_rad_s_e4(-1));
    TEST_ASSERT_EQUAL_INT(43633, ams::bmi088::gyr_rad_s_e4(16384)); // 250 dps
    TEST_ASSERT_EQUAL_INT(87264, ams::bmi088::gyr_rad_s_e4(32767));
    TEST_ASSERT_EQUAL_INT(-87266, ams::bmi088::gyr_rad_s_e4(-32768));
}

// The low-pass corners must sit below the 50 Hz Nyquist of the log rate, or
// taking the newest sample every period aliases. Pinned so a config edit that
// breaks the reasoning in imu_record.hpp fails here.
extern "C" void test_imu_sensor_config_matches_log_rate(void) {
    TEST_ASSERT_EQUAL_UINT(10u, ams::config::ImuSamplePeriodMs);    // 100 Hz log
    TEST_ASSERT_EQUAL_HEX8(0x8A, ams::bmi088::AccConf);             // OSR4 @ 400 Hz -> 37 Hz
    TEST_ASSERT_EQUAL_HEX8(0x03, ams::bmi088::GyrBw400Hz47Hz);      // 400 Hz ODR, 47 Hz
}
