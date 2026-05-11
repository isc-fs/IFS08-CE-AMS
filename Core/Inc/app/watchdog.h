/* SPDX-License-Identifier: proprietary
 *
 * C-callable surface for the IWDG wrapper. Lets us call ams_watchdog_init
 * from main.c's USER CODE BEGIN 2 block (which CubeMX preserves across
 * regenerations) before osKernelStart, so the watchdog is alive even
 * before App_InitTask runs.
 *
 * The C++ side (watchdog.hpp) is just a thin namespace wrapper over the
 * same handle.
 */

#ifndef AMS_WATCHDOG_H
#define AMS_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_watchdog_init(void);
void ams_watchdog_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* AMS_WATCHDOG_H */
