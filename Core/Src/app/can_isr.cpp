// SPDX-License-Identifier: proprietary
//
// Single point of HAL FDCAN RX-FIFO0 dispatch.
//
// HAL_FDCAN_RxFifo0Callback is weak in the HAL; our strong override
// below is the one the IRQ handler reaches. We branch on the
// FDCAN_HandleTypeDef* and enqueue the frame on the right queue.
//
// Per docs/CAN_MAP.md: FDCAN1 = accumulator/vehicle bus, FDCAN2 = BMS.
// Today (feat/8) only FDCAN2 has a consumer; the FDCAN1 branch posts
// to acu_rx_queue which will be drained by AcuCanTask in feat/11.

#include "app/app_globals.h"
#include "app/can_frame.h"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {

// HAL handles created by CubeMX in main.c.
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

// FreeRTOS queue handles created by CubeMX in main.c.
extern osMessageQueueId_t bms_rx_queueHandle;
extern osMessageQueueId_t acu_rx_queueHandle;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxbuf[8] = {0};

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxbuf) != HAL_OK) {
            break;
        }

        CanFrame frame = {0};
        frame.id           = rxh.Identifier;
        frame.dlc          = (uint8_t)((rxh.DataLength >> 16) & 0x0F);  // FDCAN DLC field
        frame.extended     = (rxh.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
        frame.fd           = (rxh.FDFormat == FDCAN_FD_CAN)   ? 1u : 0u;
        frame.timestamp_ms = osKernelGetTickCount();

        osMessageQueueId_t target;
        if (hfdcan == &hfdcan2) {
            frame.bus = 1u;  // CanBus::Bms
            target    = bms_rx_queueHandle;
        } else {
            frame.bus = 0u;  // CanBus::Acu
            target    = acu_rx_queueHandle;
        }

        for (uint8_t i = 0; i < 8u && i < frame.dlc; ++i) {
            frame.data[i] = rxbuf[i];
        }

        if (target != nullptr) {
            // Non-blocking from ISR context: drop on full queue. A
            // future feat/7 telemetry counter can surface the drop.
            (void)osMessageQueuePut(target, &frame, 0u, 0u);
        }
    }
}

}  // extern "C"
