/* SPDX-License-Identifier: proprietary
 *
 * C-callable surface for the IWDG refresh helper. The HAL handle and
 * the one-shot init now live in CubeMX-generated code (MX_IWDG1_Init
 * in main.c, ahead of osKernelStart). All application paths route
 * refreshes through ams_watchdog_refresh() so there is a single owner.
 *
 * The C++ side (watchdog.hpp) is just a thin namespace wrapper over the
 * same handle.
 */

#ifndef AMS_WATCHDOG_H
#define AMS_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_watchdog_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* AMS_WATCHDOG_H */
