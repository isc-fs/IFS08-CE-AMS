// SPDX-License-Identifier: proprietary

#include "watchdog.hpp"

#include "main.h"  // pulls stm32h7xx_hal_iwdg.h via stm32h7xx_hal.h

// CubeMX owns the IWDG handle now: MX_IWDG1_Init() runs in main.c
// before USER CODE BEGIN 2 (i.e. before osKernelStart), satisfying
// the "watchdog alive in the pre-scheduler window" invariant we used
// to enforce here ourselves. We just borrow the generated handle so
// there is exactly one place that talks to the peripheral.
extern "C" IWDG_HandleTypeDef hiwdg1;

extern "C" void ams_watchdog_refresh(void) {
    (void)HAL_IWDG_Refresh(&hiwdg1);
}
