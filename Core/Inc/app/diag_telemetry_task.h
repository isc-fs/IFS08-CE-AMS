// SPDX-License-Identifier: proprietary
//
// diag_telemetry_task.h -- Dedicated RTOS task for multiplexed 95-cell voltage,
// 190-NTC thermal, and diagnostic status CAN streaming (0x4A3, 0x4B0, 0x4B1).
// Priority: osPriorityBelowNormal. Non-blocking lock-free operation.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void DiagTelemetryTask_Init(void);
void StartDiagTelemetryTask(void *argument);

#ifdef __cplusplus
}
#endif
