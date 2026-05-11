// SPDX-License-Identifier: proprietary
//
// App_InitTask body. Runs once, at osPriorityHigh, after the scheduler
// starts.  Brings up application-side peripheral state that has to
// happen post-scheduler (FDCAN start, ISR notification activation,
// service singletons), then self-deletes so its TCB and stack go back
// to the heap.
//
// Pre-scheduler init (IWDG, GPIO defaults) lives in main.c's USER CODE
// BEGIN 2 block.

#include "app/app_init_task.h"

#include "error_latch.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void ams_app_init_task_run(void *argument)
{
    (void)argument;

    // Backup-domain write access -- safe to call from anywhere but
    // landing it here ensures it runs BEFORE SafetyTask first looks
    // at the latch (SafetyTask::run() also calls it; idempotent).
    ams::ErrorLatch::init();

    // FDCAN bring-up: ISR notifications first, then Start. Either
    // order works but doing notifications first guarantees we don't
    // lose the first frame that arrives between Start and Notify.
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

    // Task is single-shot. CMSIS-RTOS v2: terminate self.
    osThreadExit();
}

}  // extern "C"
