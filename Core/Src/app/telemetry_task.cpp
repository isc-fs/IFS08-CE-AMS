// SPDX-License-Identifier: proprietary
//
// Telemetry task retired in refactor/19 phase 3 — the 500 ms CAN
// frame emit now runs inside the consolidated MainTask body (see
// safety_task.cpp). The CubeMX-generated thread is still created by
// MX_FREERTOS_Init in main.c (cosmetic cleanup deferred to a follow-
// up). We self-exit immediately so the TCB + stack are returned to
// the heap and no CPU is wasted on a phantom task.

#include "app/telemetry_task.h"

#include "cmsis_os2.h"

extern "C" void ams_telemetry_task_run(void *argument) {
    (void)argument;
    osThreadExit();  // single-shot; TCB freed, stack returned to heap
}
