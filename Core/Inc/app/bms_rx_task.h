/* SPDX-License-Identifier: proprietary
 *
 * C-callable entry point for BmsRxTask. main.c's
 *   USER CODE BEGIN StartBmsRxTask
 * block calls this; implementation lives in bms_rx_task.cpp.
 */

#ifndef AMS_BMS_RX_TASK_H
#define AMS_BMS_RX_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ams_bms_rx_task_run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* AMS_BMS_RX_TASK_H */
