// SPDX-License-Identifier: proprietary

#include "watchdog.hpp"

#include "main.h"  // pulls stm32h7xx_hal_iwdg.h via stm32h7xx_hal.h

namespace {

// File-scope HAL handle. Not exported beyond this translation unit --
// callers go through ams_watchdog_refresh() so there is exactly one
// place that owns the handle.
IWDG_HandleTypeDef hiwdg = {};

}  // namespace

extern "C" void ams_watchdog_init(void) {
    hiwdg.Instance       = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload    = 100u;   // ~100 ms at nominal LSI 32 kHz

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        // IWDG_Init failure is unrecoverable at this stage -- we can't
        // bring the safety supervisor up without a watchdog backing it.
        // Hard reset and try again on the next boot.
        NVIC_SystemReset();
    }
}

extern "C" void ams_watchdog_refresh(void) {
    (void)HAL_IWDG_Refresh(&hiwdg);
}
