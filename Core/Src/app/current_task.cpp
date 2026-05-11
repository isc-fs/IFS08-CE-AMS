// SPDX-License-Identifier: proprietary
//
// Periodic ADC poll of the pack current sensor (PF11 -> ADC1 channel 2).
//
// 50 ms period. Each cycle:
//   - HAL_ADC_Start, poll for conversion (5 ms timeout), get value
//   - feed it into CurrentService::update_from_adc
//   - on HAL error: do NOT call update_from_adc -> last_update_tick
//     does not advance -> SafetyTask trips on staleness within
//     kIStaleMs (200 ms) and forces ERROR
//
// First-time calibration via HAL_ADCEx_Calibration_Start runs once at
// task entry. CubeMX configures ADC1 for single-channel regular
// conversion, oversampling 64 with right-shift 6, 12-bit resolution.

#include "app/current_task.h"

#include "ams_config.hpp"
#include "current_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern ADC_HandleTypeDef hadc1;
}

namespace {

// Failed-conversion counter for telemetry.
volatile std::uint32_t g_current_adc_fail = 0;

}  // namespace

extern "C" void ams_current_task_run(void *argument) {
    (void)argument;

    // Calibrate before first use. Single-ended, linearity calibration
    // for STM32H7. Result is applied internally; nothing to consume.
    HAL_ADCEx_Calibration_Start(&hadc1,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_SINGLE_ENDED);

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        last_wake += ams::config::kCurrentPeriodMs;
        osDelayUntil(last_wake);

        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
            ++g_current_adc_fail;
            continue;
        }
        if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
            ++g_current_adc_fail;
            (void)HAL_ADC_Stop(&hadc1);
            continue;
        }

        const std::uint32_t raw = HAL_ADC_GetValue(&hadc1);
        (void)HAL_ADC_Stop(&hadc1);

        ams::CurrentService::instance().update_from_adc(
            static_cast<std::uint16_t>(raw),
            osKernelGetTickCount());
    }
}
