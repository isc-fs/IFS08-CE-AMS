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

    // FDCAN filter setup. CubeMX configures 0 standard / 5 extended
    // filter slots; with no software-defined filters the default for
    // unmatched frames depends on GlobalFilterConfig. Be explicit:
    // accept every unmatched standard and extended frame into FIFO0
    // on both buses. This covers:
    //   FDCAN1 (ACU bus): VCU 0x100 (ext), 0x600 (std), charger
    //     0x18FF50E7 (ext), and any future VCU frames.
    //   FDCAN2 (BMS bus): slave responses 0x12D+ (std) plus the
    //     boot-trigger frame at 0x002 (std).
    // Remote frames are rejected on both buses.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

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
