/* SPDX-License-Identifier: proprietary
 *
 * Stub cmsis_os2.h for the host-side unit-test build. Provides just
 * enough surface for ScopedMutex and BmsService to compile and run on
 * a developer's laptop without bringing in FreeRTOS.
 *
 * The mock mutex always succeeds (the test driver is single-threaded
 * so this is correct, just degenerate). Threading-related symbols are
 * stubbed too in case future tests touch tasks / queues / event flags.
 */

#ifndef AMS_MOCK_CMSIS_OS2_H
#define AMS_MOCK_CMSIS_OS2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *osMutexId_t;
typedef void *osMessageQueueId_t;
typedef void *osEventFlagsId_t;
typedef void *osThreadId_t;
typedef void *osTimerId_t;
typedef int   osStatus_t;

#define osOK                   ((osStatus_t)0)
#define osError                ((osStatus_t)-1)
#define osErrorTimeout         ((osStatus_t)-2)
#define osErrorResource        ((osStatus_t)-3)
#define osWaitForever          ((uint32_t)0xFFFFFFFFu)

#define osFlagsError           ((uint32_t)0x80000000u)
#define osFlagsErrorTimeout    ((uint32_t)0xFFFFFFFEu)
#define osFlagsWaitAny         ((uint32_t)0x00000000u)
#define osFlagsNoClear         ((uint32_t)0x00000002u)

osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex);
uint32_t   osKernelGetTickCount(void);

/* For BmsService / CurrentService: extern mutex handles. Non-null
 * stubs provided by cmsis_os2_stub.cpp. */
extern osMutexId_t bms_mutexHandle;
extern osMutexId_t current_mutexHandle;
extern osMutexId_t vehicle_mutexHandle;

#ifdef __cplusplus
}
#endif

#endif /* AMS_MOCK_CMSIS_OS2_H */
