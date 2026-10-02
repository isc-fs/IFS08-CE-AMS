// SPDX-License-Identifier: proprietary
//
// ImuSample -- one BMI088 reading as it travels from ImuTask to the SD card,
// plus everything about the sensor that can be expressed without a bus: the
// register map, the configuration ImuTask writes, raw-byte decoding and unit
// scaling. The sample is written to IMUnnnn.BIN as-is (bin_log.hpp), and the
// scales here are the ones its header schema declares. Pure (HAL-free,
// RTOS-free) so the host tests cover it.
//
// TELEMETRY ONLY. Nothing on the safety path reads an ImuSample.
//
// Axes are the SENSOR frame printed on the BMI088 package, not the car frame.
// Mapping them onto the car needs the MLC's mounting orientation in the AMS,
// which is a post-processing step, not something the firmware assumes.

#pragma once

#include "ams_config.hpp"

#include <cstddef>
#include <cstdint>

namespace ams {

// Raw counts, kept raw across the ring and on the card so the 100 Hz producer
// does no arithmetic; tools/log_decode.py scales them. 16 B, power-of-two
// friendly.
struct ImuSample {
    std::uint32_t tick_ms;   // osKernelGetTickCount() when the read started
    std::int16_t  acc[3];    // x, y, z accelerometer counts
    std::int16_t  gyr[3];    // x, y, z gyroscope counts
};
static_assert(sizeof(ImuSample) == 16, "ImuSample layout drifted");

namespace bmi088 {

// Register map -- BMI088 datasheet, accelerometer and gyroscope register maps.
// The two dies are separate I2C targets (config::ImuAccAddr7b / ImuGyrAddr7b).
inline constexpr std::uint8_t AccChipIdReg   = 0x00;
inline constexpr std::uint8_t AccDataReg     = 0x12;  // X_LSB .. Z_MSB, 6 bytes
inline constexpr std::uint8_t AccConfReg     = 0x40;
inline constexpr std::uint8_t AccRangeReg    = 0x41;
inline constexpr std::uint8_t AccPwrConfReg  = 0x7C;
inline constexpr std::uint8_t AccPwrCtrlReg  = 0x7D;

inline constexpr std::uint8_t GyrChipIdReg   = 0x00;
inline constexpr std::uint8_t GyrDataReg     = 0x02;  // X_LSB .. Z_MSB, 6 bytes
inline constexpr std::uint8_t GyrRangeReg    = 0x0F;
inline constexpr std::uint8_t GyrBandwidthReg = 0x10;
inline constexpr std::uint8_t GyrLpm1Reg     = 0x11;

inline constexpr std::uint8_t AccChipId      = 0x1E;
inline constexpr std::uint8_t GyrChipId      = 0x0F;

// Configuration written at init.
//
// Accelerometer: ACC_CONF = acc_bwp OSR4 (0x8 in the high nibble) | ODR 400 Hz
// (0xA). OSR4 at 400 Hz gives a 3 dB corner of ~37 Hz, below the 50 Hz Nyquist
// of the 100 Hz log, so taking the newest sample every 10 ms does not alias.
inline constexpr std::uint8_t AccConf        = 0x8A;
inline constexpr std::uint8_t AccRange6g     = 0x01;  // +/-6 g
inline constexpr std::uint8_t AccPwrActive   = 0x00;  // leave suspend
inline constexpr std::uint8_t AccPwrOn       = 0x04;  // accelerometer on

// Gyroscope: ODR 400 Hz with a 47 Hz filter (bandwidth code 0x03), same
// reasoning as the accelerometer. +/-500 dps: a car's yaw rate stays well
// inside it, and it keeps 4x the resolution of the +/-2000 dps default.
inline constexpr std::uint8_t GyrRange500dps = 0x02;
inline constexpr std::uint8_t GyrBw400Hz47Hz = 0x03;
inline constexpr std::uint8_t GyrLpm1Normal  = 0x00;
// Bit 7 of GYRO_BANDWIDTH always reads back as 1; compare with it masked.
inline constexpr std::uint8_t GyrBandwidthReadMask = 0x7F;

// Settling times from the datasheet's start-up sequence, rounded up.
inline constexpr std::uint32_t AccPwrConfMs   = 5;    // spec 450 us
inline constexpr std::uint32_t AccPwrOnMs     = 50;   // first valid sample

// Little-endian 16-bit axis triple, as both dies emit it.
inline void decode_axes(const std::uint8_t* b6, std::int16_t out[3]) noexcept {
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<std::int16_t>(
            static_cast<std::uint16_t>(b6[2 * i]) |
            (static_cast<std::uint16_t>(b6[2 * i + 1]) << 8));
    }
}

// Round-half-away-from-zero of v * num / den. 64-bit because a count times
// the gyro numerator below is ~2.9e13.
inline std::int32_t scale_counts(std::int16_t v, std::int64_t num, std::int64_t den) noexcept {
    const std::int64_t p = static_cast<std::int64_t>(v) * num;
    const std::int64_t half = (p >= 0) ? den / 2 : -(den / 2);
    return static_cast<std::int32_t>((p + half) / den);
}

// Reference conversions, fixed-point with 4 decimals (units of 1e-4), which
// is finer than one sensor count on either die. The scales the IMUnnnn.BIN
// schema declares must agree with these; test_bin_log.cpp checks that.

// Acceleration in 1e-4 g. Datasheet: accel_g = counts / 32768 * 2^(range+1) * 1.5;
// range 0x01 -> 6 g full scale -> counts * 60000 / 32768. One count = 1.83e-4 g.
inline std::int32_t acc_g_e4(std::int16_t counts) noexcept {
    return scale_counts(counts, 60000, 32768);
}

// Angular rate in 1e-4 rad/s. +/-500 dps full scale = 500 * pi / 180 =
// 8.726646 rad/s, so counts * 87266.4626 / 32768; kept exact to 1e-9 as
// 872664626 / 327680000. One count = 2.66e-4 rad/s (0.0153 dps).
inline std::int32_t gyr_rad_s_e4(std::int16_t counts) noexcept {
    return scale_counts(counts, 872664626LL, 327680000LL);
}

}  // namespace bmi088

}  // namespace ams
