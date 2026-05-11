/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for BmsPollTask. main.c's
 *   USER CODE BEGIN StartBmsPollTask
 * block calls this; implementation lives in bms_poll_task.cpp.
 */

#ifndef AMS_BMS_POLL_TASK_H
#define AMS_BMS_POLL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_bms_poll_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_BMS_POLL_TASK_H */
