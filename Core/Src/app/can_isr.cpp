// SPDX-License-Identifier: proprietary
//
// Single point of HAL FDCAN RX-FIFO0 dispatch.
//
// HAL_FDCAN_RxFifo0Callback is weak in the HAL; our strong override
// below is the one the IRQ handler reaches. Only FDCAN1 (the
// accumulator/vehicle bus) feeds an application queue post-v1.2.0
// (#73). FDCAN2 stays initialised by CubeMX so the bootloader can
// take it over after reset, but the app never starts FDCAN2 nor
// activates its RX-FIFO0 notification, so this callback only fires
// for &hfdcan1.

#include "app/app_globals.h"
#include "app/can_frame.h"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {

extern osMessageQueueId_t acu_rx_queueHandle;

// ISR-side drop counter: incremented when osMessageQueuePut returns
// non-osOK in the callback below (typically the queue is full because
// AcuCanTask hasn't drained fast enough, or pre-scheduler bursts).
// Distinct from g_acu_rx_dropped_unknown (acu_can_task.cpp), which
// counts frames that DID land in the queue but whose ID didn't match
// any known ACU dispatch case. Read via GDB / SWD / a future diag
// telemetry frame.
volatile uint32_t g_acu_rx_isr_drop = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }
    if (acu_rx_queueHandle == NULL) {
        return;  // pre-scheduler / shutdown -- nowhere to land the frame.
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
        frame.bus          = 0u;  // CanBus::Acu

        for (uint8_t i = 0; i < 8u && i < frame.dlc; ++i) {
            frame.data[i] = rxbuf[i];
        }

        // Non-blocking from ISR context: drop on full queue and
        // bump the ISR-drop counter. The unknown-ID drop counter
        // (g_acu_rx_dropped_unknown) lives task-side and counts a
        // different failure mode.
        if (osMessageQueuePut(acu_rx_queueHandle, &frame, 0u, 0u) != osOK) {
            ++g_acu_rx_isr_drop;
        }
    }
}

}  // extern "C"
