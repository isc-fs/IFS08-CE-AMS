// SPDX-License-Identifier: proprietary
//
// ImuTask -- samples the MLC's BMI088 (I2C2, DMA) every ImuSamplePeriodMs and
// hands each reading to SdLoggerTask for IMUnnnn.CSV.
//
// TELEMETRY ONLY. It runs at osPriorityLow1, below every task on the safety
// path, is not part of fw_health or the watchdog, and a missing or failing IMU
// never faults the AMS: the task retries once per ImuRetryPeriodMs and the
// card simply gets no IMU rows meanwhile.
//
// Dual-use header: the C++ API is gated behind __cplusplus so the C-compiled
// CubeMX main.c can include it just for the trampoline declaration.

#pragma once

#ifdef __cplusplus

#include <cstdint>

namespace ams {

enum class ImuState : std::uint8_t {
    Init      = 0,   // not yet brought up
    Running   = 1,   // sampling
    NotFound  = 2,   // chip-ID read failed or returned the wrong ID
    BusError  = 3,   // a transfer failed or timed out; re-initialising
};

struct ImuStats {
    std::uint32_t samples;       // readings pushed toward the logger
    std::uint32_t read_errors;   // failed or timed-out DMA reads
    std::uint32_t inits;         // successful sensor initialisations
    std::uint32_t i2c_error;     // HAL ErrorCode when the last init failed
    std::uint8_t  fail_step;     // init step that last failed, 0 = last init OK
    ImuState      state;
};
ImuStats imu_stats() noexcept;

}  // namespace ams

extern "C" {
#endif  // __cplusplus

// FreeRTOS trampoline -- the CubeMX ImuTask thread entry calls this.
void ams_imu_task_run(void *argument);

#ifdef __cplusplus
}
#endif
