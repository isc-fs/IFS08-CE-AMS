// SPDX-License-Identifier: proprietary
//
// Periodic ADC poll of the pack current sensor (PF7 -> ADC3 channel 3).
//
// 50 ms period. Each cycle:
//   - HAL_ADC_Start, poll for conversion (5 ms timeout), get value
//   - feed it into CurrentService::update_from_adc
//   - on HAL error: do NOT call update_from_adc -> last_update_tick
//     does not advance -> SafetyTask trips on staleness within
//     kIStaleMs (200 ms) and forces ERROR
//
// First-time calibration via HAL_ADCEx_Calibration_Start runs once at
// task entry. CubeMX configures ADC3 for single-channel regular
// conversion, 12-bit resolution. ADC3's oversampling is left at the
// CubeMX default (2x, no right-shift) -- the Hall sensor's intrinsic
// noise floor is well above the LSB at 12-bit, so additional
// oversampling brings diminishing returns. Bump in the .ioc if a
// future calibration shows otherwise.
//
// Moved from ADC1 (PF11) -> ADC3 (PF7) in chore/49 (carrier-board
// redesign relocated the Hall sensor input). PF8 stays configured
// as an analog input on ADC3_INP7 for a future DCDC current
// measurement -- the regular conversion only reads channel 3 today.

#include "app/current_task.h"

#include "ams_config.hpp"
#include "current_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern ADC_HandleTypeDef hadc3;
}

namespace {

// Failed-conversion counter for telemetry.
volatile std::uint32_t g_current_adc_fail = 0;

}  // namespace

extern "C" void ams_current_sensor_task_run(void *argument) {
    (void)argument;

    // Calibrate before first use. Single-ended, linearity calibration
    // for STM32H7. Result is applied internally; nothing to consume.
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_SINGLE_ENDED);

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        last_wake += ams::config::kCurrentPeriodMs;
        osDelayUntil(last_wake);

        if (HAL_ADC_Start(&hadc3) != HAL_OK) {
            ++g_current_adc_fail;
            continue;
        }
        if (HAL_ADC_PollForConversion(&hadc3, 5) != HAL_OK) {
            ++g_current_adc_fail;
            (void)HAL_ADC_Stop(&hadc3);
            continue;
        }

        const std::uint32_t raw = HAL_ADC_GetValue(&hadc3);
        (void)HAL_ADC_Stop(&hadc3);

        ams::CurrentService::instance().update_from_adc(
            static_cast<std::uint16_t>(raw),
            osKernelGetTickCount());
    }
}
