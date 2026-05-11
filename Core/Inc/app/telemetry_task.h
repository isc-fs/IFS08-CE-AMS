/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for TelemetryTask.
 */

#ifndef AMS_TELEMETRY_TASK_H
#define AMS_TELEMETRY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_telemetry_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_TELEMETRY_TASK_H */
