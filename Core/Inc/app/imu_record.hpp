// SPDX-License-Identifier: proprietary
//
// ImuSample -- one BMI088 reading as it travels from ImuTask to the SD card,
// plus everything about the sensor that can be expressed without a bus: the
// register map, the configuration ImuTask writes, raw-byte decoding, unit
// scaling and the IMUnnnn.CSV row format. Pure (HAL-free, RTOS-free) so the
// host tests cover it.
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
#include <cstdio>

namespace ams {

// Raw counts, kept raw across the ring so the 100 Hz producer does no
// arithmetic; the consumer scales at format time. 16 B, power-of-two friendly.
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

// Round-half-away-from-zero of v * num / 32768. 64-bit because a gyro count
// times 500000 overflows 32 bits.
inline std::int32_t scale_counts(std::int16_t v, std::int32_t num) noexcept {
    const std::int64_t p = static_cast<std::int64_t>(v) * num;
    const std::int64_t half = (p >= 0) ? 16384 : -16384;
    return static_cast<std::int32_t>((p + half) / 32768);
}

// Datasheet: accel_mg = counts / 32768 * 1000 * 2^(range + 1) * 1.5.
// Range 0x01 -> 2^2 * 1.5 = 6 g full scale -> counts * 6000 / 32768.
inline std::int32_t acc_mg(std::int16_t counts) noexcept {
    return scale_counts(counts, 6000);
}

// +/-500 dps full scale: counts * 500 / 32768 dps = counts * 500000 / 32768 mdps.
inline std::int32_t gyr_mdps(std::int16_t counts) noexcept {
    return scale_counts(counts, 500000);
}

}  // namespace bmi088

namespace imu_csv {

inline constexpr char Header[] =
    "tick_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps\n";

// Widest row: 10-digit tick, three "-6000", three "-500000", 6 commas, '\n'.
inline constexpr std::size_t MaxRowBytes = 64;

// Returns bytes written (excl. NUL), or 0 on truncation. tick_ms is the same
// clock as LOGnnnn.CSV's tick_ms, so the two files of one index line up.
inline std::size_t format_row(const ImuSample& s, char* buf, std::size_t cap) noexcept {
    const int n = std::snprintf(
        buf, cap, "%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
        static_cast<unsigned long>(s.tick_ms),
        static_cast<long>(bmi088::acc_mg(s.acc[0])),
        static_cast<long>(bmi088::acc_mg(s.acc[1])),
        static_cast<long>(bmi088::acc_mg(s.acc[2])),
        static_cast<long>(bmi088::gyr_mdps(s.gyr[0])),
        static_cast<long>(bmi088::gyr_mdps(s.gyr[1])),
        static_cast<long>(bmi088::gyr_mdps(s.gyr[2])));
    if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;
    return static_cast<std::size_t>(n);
}

}  // namespace imu_csv
}  // namespace ams
