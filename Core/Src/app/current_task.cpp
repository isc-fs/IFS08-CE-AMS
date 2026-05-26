// SPDX-License-Identifier: proprietary
//
// Periodic ADC poll of TWO current sensors on ADC3:
//   - PF7 / ADC3_INP3 -> pack current (Bourns SSA-2-250A behind diff amp)
//   - PF8 / ADC3_INP7 -> DCDC current (same sensor topology, fix/53)
//
// 50 ms period. Each cycle:
//   1. Reconfigure ADC3 regular channel for INP3 (pack), start, poll, get
//   2. Feed into CurrentService::update_from_adc
//   3. Reconfigure ADC3 regular channel for INP7 (DCDC), start, poll, get
//   4. Feed into CurrentService::update_dcdc_from_adc
//   5. On HAL error at any step: skip that channel's update so the
//      corresponding last_*_update_tick does not advance -> SafetyTask
//      trips on staleness for the pack channel (IStaleMs = 200 ms) and
//      forces ERROR. DCDC staleness is informational only (no FSM impact).
//
// The channel-swap costs ~10 us of HAL overhead per swap; total cycle
// budget at 50 ms is comfortable. If timing ever gets tight we can
// move to ADC3 scan mode (two ranks via CubeMX) and drop the runtime
// reconfigure.
//
// First-time calibration via HAL_ADCEx_Calibration_Start runs once at
// task entry. CubeMX configures ADC3 for 12-bit single-channel regular
// conversion with rank 1 = Channel 3 (PF7); the rank-1 channel is
// re-set on every iteration so the order of reads is deterministic.

#include "app/current_task.h"

#include "ams_config.hpp"
#include "current_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern ADC_HandleTypeDef hadc3;
}

namespace {

// Failed-conversion counters for telemetry. Separate for pack vs DCDC
// so the bench can localise which channel is the problem.
volatile std::uint32_t g_current_adc_fail      = 0;
volatile std::uint32_t g_current_adc_dcdc_fail = 0;

// One-shot single-channel read on ADC3. Reconfigures rank 1 to the
// requested channel, starts, polls, gets the value. Returns false on
// any HAL failure -- caller bumps its own fail counter and skips the
// CurrentService update so freshness doesn't advance.
bool read_adc3_channel(std::uint32_t channel, std::uint16_t& out_raw) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    cfg.OffsetSign   = ADC3_OFFSET_SIGN_NEGATIVE;
    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK)    return false;
    if (HAL_ADC_Start(&hadc3)              != HAL_OK)    return false;
    if (HAL_ADC_PollForConversion(&hadc3, 5) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc3);
        return false;
    }
    out_raw = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc3));
    (void)HAL_ADC_Stop(&hadc3);
    return true;
}

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
        last_wake += ams::config::CurrentPeriodMs;
        osDelayUntil(last_wake);

        // --- Pack current (PF7 / ADC3_INP3) ---
        std::uint16_t raw_pack = 0;
        if (read_adc3_channel(ADC_CHANNEL_3, raw_pack)) {
            ams::CurrentService::instance().update_from_adc(
                raw_pack, osKernelGetTickCount());
        } else {
            ++g_current_adc_fail;
        }

        // --- DCDC current (PF8 / ADC3_INP7) ---
        std::uint16_t raw_dcdc = 0;
        if (read_adc3_channel(ADC_CHANNEL_7, raw_dcdc)) {
            ams::CurrentService::instance().update_dcdc_from_adc(
                raw_dcdc, osKernelGetTickCount());
        } else {
            ++g_current_adc_dcdc_fail;
        }
    }
}
