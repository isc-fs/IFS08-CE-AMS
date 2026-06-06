// SPDX-License-Identifier: proprietary
//
// Periodic ADC poll of TWO current sensors on ADC3:
//   - PF7/PF8 / ADC3_INP3+INN3 -> pack current (Bourns SSA-2-250A read
//     in ADC DIFFERENTIAL mode; OUT_P=PF7, OUT_N=PF8)
//   - PC1 / ADC3_INP11 -> DCDC current (separate single-ended sensor)
//
// 50 ms period. Each cycle:
//   1. Reconfigure ADC3 regular channel for INP3 differential (pack),
//      start, poll, get
//   2. Feed into CurrentService::update_from_adc
//   3. Reconfigure ADC3 regular channel for INP11 single-ended (DCDC),
//      start, poll, get
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
// task entry -- BOTH single-ended and differential offset/linearity,
// since this revision mixes a differential channel (pack) and a
// single-ended one (DCDC). CubeMX configures ADC3 rank 1 = Channel 3
// differential (PF7/PF8); the rank-1 channel + single/diff selection is
// re-set on every read so the order and mode are deterministic.

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
// requested channel and single/differential mode, starts, polls, gets
// the value. Returns false on any HAL failure -- caller bumps its own
// fail counter and skips the CurrentService update so freshness doesn't
// advance.
bool read_adc3_channel(std::uint32_t channel, std::uint32_t single_diff,
                       std::uint16_t& out_raw) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    cfg.SingleDiff   = single_diff;
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

    // Calibrate before first use. Offset + linearity for BOTH the
    // single-ended (DCDC / INP11) and differential (pack / INP3+INN3)
    // signal paths -- on STM32H7 the two have independent calibration
    // factors. Results are applied internally; nothing to consume.
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_DIFFERENTIAL_ENDED);

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        last_wake += ams::config::CurrentPeriodMs;
        osDelayUntil(last_wake);

        // --- Pack current (PF7/PF8 / ADC3_INP3+INN3, differential) ---
        std::uint16_t raw_pack = 0;
        if (read_adc3_channel(ADC_CHANNEL_3, ADC_DIFFERENTIAL_ENDED, raw_pack)) {
            ams::CurrentService::instance().update_from_adc(
                raw_pack, osKernelGetTickCount());
        } else {
            ++g_current_adc_fail;
        }

        // --- DCDC current (PC1 / ADC3_INP11, single-ended) ---
        std::uint16_t raw_dcdc = 0;
        if (read_adc3_channel(ADC_CHANNEL_11, ADC_SINGLE_ENDED, raw_dcdc)) {
            ams::CurrentService::instance().update_dcdc_from_adc(
                raw_dcdc, osKernelGetTickCount());
        } else {
            ++g_current_adc_dcdc_fail;
        }
    }
}
