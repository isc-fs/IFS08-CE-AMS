/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for CurrentTask. main.c's
 *   USER CODE BEGIN StartCurrentTask
 * block calls this.
 */

#ifndef AMS_CURRENT_TASK_H
#define AMS_CURRENT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_current_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_CURRENT_TASK_H */
