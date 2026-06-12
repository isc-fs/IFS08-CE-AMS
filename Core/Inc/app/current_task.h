/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for CurrentSensorTask. main.c's
 *   USER CODE BEGIN StartCurrentSensorTask
 * block calls this. (File still called current_task.h to keep this
 * commit free of file renames; renamed in spirit only.)
 */

#ifndef AMS_CURRENT_TASK_H
#define AMS_CURRENT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_current_sensor_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_CURRENT_TASK_H */
