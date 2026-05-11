/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for AcuCanTask. main.c's
 *   USER CODE BEGIN StartAcuCanTask
 * block calls this; implementation lives in acu_can_task.cpp.
 */

#ifndef AMS_ACU_CAN_TASK_H
#define AMS_ACU_CAN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_acu_can_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_ACU_CAN_TASK_H */
