/* SPDX-License-Identifier: proprietary
 *
 * CubeMX 6.16 / CMake target silently drops the FreeRTOS event group
 * definitions when generating main.c (mutexes and queues come through
 * fine; event groups do not). Until that's fixed upstream, we create
 * the two event groups declared in AMS.ioc ourselves.
 *
 * ams_app_globals_init() is called from main.c's USER CODE BEGIN 2
 * block, AFTER osKernelInitialize() and BEFORE osKernelStart(). The
 * handles are declared here as C-linkage extern so both the
 * CubeMX-generated C code and the application C++ code can resolve
 * them.
 */

#ifndef AMS_APP_GLOBALS_H
#define AMS_APP_GLOBALS_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osEventFlagsId_t safety_eventsHandle;
extern osEventFlagsId_t bms_eventsHandle;

void ams_app_globals_init(void);

#ifdef __cplusplus
}
#endif

#endif /* AMS_APP_GLOBALS_H */
