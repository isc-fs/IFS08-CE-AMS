// SPDX-License-Identifier: proprietary
//
// Accumulator-bus task (FDCAN1). Two responsibilities:
//
//  RX (always): drain acu_rx_queue -> VehicleService::update_from_frame,
//               with bootloader-trigger sniffing in front.
//
//  TX (periodic) -- the ECU-side telemetry matrix shipped in fix/53.
//  The ECU's FDCAN2 peripheral is wired to AMS FDCAN1, so these frames
//  feed real-time telemetry through the ECU. See docs/CAN_MAP.md and
//  ams_config.hpp for byte layouts.
//
//    50 ms: 0x135 currents (i16 deciamps; accu, dcdc)
//   100 ms: 0x020 ok_precarga
//           0x12C v_celda_min (pack-wide min cell mV)
//           0x131/0x132 vmin_modulo  (5 modules over two frames)
//           0x133/0x134 vmax_modulo
//   250 ms: 0x136/0x137 temp_max_modulo (+ temp_dcdc stub on 0x137)
//
// 0x130 (SOC) deferred until firmware has an estimator.
// 0x450 retired -- 0x135 is the successor.
//
// Cadence is driven by osMessageQueueGet with a computed timeout that
// expires at the nearest TX deadline, so RX latency stays low and TX
// jitter stays bounded.

#include "app/acu_can_task.h"

#include "acu_tx_encoders.hpp"
#include "ams_config.hpp"
#include "bms_service.hpp"
#include "bootloader.hpp"
#include "can_frame.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;
}

extern "C" osMessageQueueId_t acu_rx_queueHandle;

// FSM state mirror maintained by safety_task.cpp; used here for
// ok_precarga (= 1 iff state in {Run=3, Charge=4}).
extern "C" volatile std::uint8_t g_state_telemetry;

namespace {

// Telemetry counters; volatile so a remote-debug session can read.
volatile std::uint32_t g_acu_rx_dropped_unknown = 0;
volatile std::uint32_t g_acu_tx_fail            = 0;

bool send_acu(std::uint32_t id, std::uint8_t dlc,
              const std::uint8_t *data) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = static_cast<std::uint32_t>(dlc) << 16;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx,
               const_cast<std::uint8_t *>(data)) == HAL_OK;
}

// Each tx_* wrapper builds the frame via the host-testable encoder in
// acu_tx_encoders.hpp, then ships it through send_acu. Keeping the
// encoders pure-logic in a header lets the unit-test build verify the
// byte layout without HAL.

template <std::size_t N>
inline void send_or_fail(std::uint32_t id,
                         const std::array<std::uint8_t, N>& payload) noexcept {
    if (!send_acu(id, static_cast<std::uint8_t>(N), payload.data())) {
        ++g_acu_tx_fail;
    }
}

void tx_ok_precarga() noexcept {
    send_or_fail(ams::config::kAcuTxOkPrechargeId,
                 ams::acu_tx::encode_ok_precarga(g_state_telemetry));
}
void tx_min_voltage(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxMinVoltId,        ams::acu_tx::encode_min_voltage(b));
}
void tx_vmin_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxVminModuleAId,    ams::acu_tx::encode_vmin_module_a(b));
}
void tx_vmin_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxVminModuleBId,    ams::acu_tx::encode_vmin_module_b(b));
}
void tx_vmax_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxVmaxModuleAId,    ams::acu_tx::encode_vmax_module_a(b));
}
void tx_vmax_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxVmaxModuleBId,    ams::acu_tx::encode_vmax_module_b(b));
}
void tx_currents(const ams::CurrentState& c) noexcept {
    send_or_fail(ams::config::kAcuTxCurrentsId,       ams::acu_tx::encode_currents(c));
}
void tx_temp_module_a(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxTempMaxModuleAId, ams::acu_tx::encode_tmax_module_a(b));
}
void tx_temp_module_b(const ams::BmsState& b) noexcept {
    send_or_fail(ams::config::kAcuTxTempMaxModuleBId, ams::acu_tx::encode_tmax_module_b(b));
}

}  // namespace

extern "C" void ams_acu_can_task_run(void *argument) {
    (void)argument;

    ams::CanFrame frame;
    const auto    now0          = osKernelGetTickCount();
    std::uint32_t last_fast_tx  = now0;
    std::uint32_t last_mid_tx   = now0;
    std::uint32_t last_slow_tx  = now0;

    for (;;) {
        const auto now           = osKernelGetTickCount();
        const auto next_fast     = last_fast_tx + ams::config::kEcuFastTxMs;
        const auto next_mid      = last_mid_tx  + ams::config::kEcuMidTxMs;
        const auto next_slow     = last_slow_tx + ams::config::kEcuSlowTxMs;
        const auto deadline      = std::min({ next_fast, next_mid, next_slow });
        const auto timeout       = (deadline > now) ? deadline - now : 0u;

        if (osMessageQueueGet(acu_rx_queueHandle, &frame, nullptr, timeout) == osOK) {
            // Boot-trigger frame moved to FDCAN1 in v1.2.0 (#73). Has
            // to come BEFORE VehicleService so we still react to the
            // reboot request even if the trigger ID accidentally
            // collides with a future VCU frame. request_reboot() never
            // returns -- it opens all relays, writes
            // BL_BOOT_REQ_MAGIC into BKP0R, and resets.
            if (ams::Bootloader::matches_trigger(frame)) {
                ams::Bootloader::request_reboot(
                    ams::config::JumpReason::kCanTrigger);
            }
            if (!ams::VehicleService::instance().update_from_frame(frame)) {
                ++g_acu_rx_dropped_unknown;
            }
        }

        // ---- Snapshot service state once per loop iteration ----
        const auto bms = ams::BmsService::instance().snapshot();
        const auto cur = ams::CurrentService::instance().snapshot();
        const auto now2 = osKernelGetTickCount();

        // ---- TX scheduler ----
        if (now2 - last_fast_tx >= ams::config::kEcuFastTxMs) {
            tx_currents(cur);
            last_fast_tx = now2;
        }
        if (now2 - last_mid_tx >= ams::config::kEcuMidTxMs) {
            tx_ok_precarga();
            tx_min_voltage(bms);
            tx_vmin_module_a(bms);
            tx_vmin_module_b(bms);
            tx_vmax_module_a(bms);
            tx_vmax_module_b(bms);
            last_mid_tx = now2;
        }
        if (now2 - last_slow_tx >= ams::config::kEcuSlowTxMs) {
            tx_temp_module_a(bms);
            tx_temp_module_b(bms);
            last_slow_tx = now2;
        }
    }
}
