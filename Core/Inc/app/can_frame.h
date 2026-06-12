/* SPDX-License-Identifier: proprietary
 *
 * C-visible POD shape for the CAN frame carried through the FreeRTOS
 * message queues. main.c (CubeMX-generated, C) needs to see sizeof(CanFrame)
 * when calling osMessageQueueNew(); the C++ side (can_frame.hpp) wraps
 * this same layout in the ams:: namespace.
 *
 * Keep this in sync with Core/Inc/app/can_frame.hpp.
 */

#ifndef AMS_CAN_FRAME_H
#define AMS_CAN_FRAME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_CAN_FRAME_MAX_DATA 8u

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  extended;     /* bool: 1 if 29-bit id */
    uint8_t  fd;           /* reserved, always 0 today */
    uint8_t  bus;          /* 0 = BMS, 1 = ACU */
    uint32_t timestamp_ms;
    uint8_t  data[AMS_CAN_FRAME_MAX_DATA];
} CanFrame;

#ifndef __cplusplus
_Static_assert(sizeof(CanFrame) <= 32, "CanFrame must fit a 32-byte queue slot");
#else
static_assert(sizeof(CanFrame) <= 32, "CanFrame must fit a 32-byte queue slot");
#endif

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* AMS_CAN_FRAME_H */
