/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for SafetyTask. main.c's
 *   USER CODE BEGIN StartSafetyTask
 * block calls this; the implementation lives in safety_task.cpp.
 */

#ifndef AMS_SAFETY_TASK_H
#define AMS_SAFETY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_safety_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_SAFETY_TASK_H */
