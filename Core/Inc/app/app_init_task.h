/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for App_InitTask. main.c's
 *   USER CODE BEGIN StartAppInitTask
 * block calls this; the implementation lives in app_init_task.cpp.
 * Function self-terminates the calling task once init completes.
 */

#ifndef AMS_APP_INIT_TASK_H
#define AMS_APP_INIT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_app_init_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_APP_INIT_TASK_H */
