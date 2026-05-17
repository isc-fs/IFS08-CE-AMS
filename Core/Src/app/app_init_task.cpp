// SPDX-License-Identifier: proprietary
//
// App_InitTask body. Runs once, at osPriorityHigh, after the scheduler
// starts.  Brings up application-side peripheral state that has to
// happen post-scheduler (FDCAN start, ISR notification activation,
// service singletons), then self-deletes so its TCB and stack go back
// to the heap.
//
// Pre-scheduler init (IWDG, GPIO defaults) lives in main.c's USER CODE
// BEGIN 2 block.

#include "app/app_init_task.h"

#include "ams_config.hpp"
#include "app_globals.h"
#include "error_latch.hpp"
#include "ltc6811.hpp"
#include "ltc6820.hpp"
#include "relay_driver.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <cstddef>
#include <cstdint>

extern "C" {

extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef   hspi1;

#if defined(AMS_BMS_HIL_STUB)
// #123 iter 13: boot-trace frame on CAN ID 0x7FF, payload[0] = milestone
// marker. Emitted inline at each App_InitTask milestone -- bypasses
// MainTask entirely so the operator can observe init progress on the
// bus even if MainTask never gets created or runs (operator's H4).
// Pre-Start markers will silently fail HAL queuing (FDCAN not in
// BUSY state yet); post-Start markers actually transmit. So:
//   0 x 0x7FF seen   -> HAL_FDCAN_Start failed (H2) or PHY hardware (H3)
//   >=1 x 0x7FF seen -> Start succeeded; payload[0] is the last milestone
//                       that reached the call.
static void send_boot_trace(std::uint8_t marker, std::uint32_t start_rc) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = 0x7FFu;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    std::uint8_t data[8] = {
        marker,
        static_cast<std::uint8_t>(start_rc & 0xFFu),         // HAL return low byte
        static_cast<std::uint8_t>((start_rc >> 8) & 0xFFu),
        static_cast<std::uint8_t>((start_rc >> 16) & 0xFFu),
        static_cast<std::uint8_t>((start_rc >> 24) & 0xFFu),
        0, 0, 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
}
#endif

// Diagnostic for #123 (HIL_STUB only): monotonic milestone counter
// incremented at each App_InitTask init step. Surfaced in 0x4A0[3]
// via encode_status so the bench can see how far this task got
// without SWD, even if FDCAN1 never starts and other telemetry
// silences. If 0x4A0 transmits at all, byte 3 == this counter at
// the moment MainTask snapshotted it.
//   0 -> App_InitTask never started OR hung before any milestone
//   1 -> post-ErrorLatch::init
//   2 -> post-ErrorLatch::clear         (HIL_STUB only)
//   3 -> post-HAL_FDCAN_ConfigGlobalFilter
//   4 -> post-HAL_FDCAN_ActivateNotification
//   5 -> post-HAL_FDCAN_Start (regardless of return code)
//   6 -> Start returned HAL_OK
//   7 -> task self-exit reached (App_InitTask completed cleanly)
//
// HAL_FDCAN_Start's return code is also captured into a sibling
// global so the operator can read it later via SWD or a follow-up
// probe if needed. The byte itself only surfaces the milestone.
#if defined(AMS_BMS_HIL_STUB)
extern "C" volatile std::uint8_t  g_app_init_progress   = 0;
extern "C" volatile std::uint32_t g_fdcan1_start_result = 0xFFFFFFFFu;
#endif

void ams_app_init_task_run(void *argument)
{
    (void)argument;

    // Backup-domain write access -- safe to call from anywhere but
    // landing it here ensures it runs BEFORE SafetyTask first looks
    // at the latch (SafetyTask::run() also calls it; idempotent).
    ams::ErrorLatch::init();
#if defined(AMS_BMS_HIL_STUB)
    g_app_init_progress = 1u;   // post-ErrorLatch::init
    send_boot_trace(0xB1u, g_fdcan1_start_result);  // will silently fail (FDCAN not started)
#endif

#if defined(AMS_BMS_HIL_STUB)
    // HIL-only: VBAT-backed RTC_BKP_DR1 outlives long power-offs on
    // the bench (carrier has a coin cell + bulk caps), and the bench
    // has no SWD to clear it externally. The whole point of the stub
    // build is "real BMS safety doesn't apply here" -- a stale latch
    // from a previous session has the same semantic, so wipe it on
    // every boot. NEVER compiled into flight HW: this defeats the
    // sticky-error contract that protects against intermittent
    // pre-charge / SDC events surviving a brown-out.
    ams::ErrorLatch::clear();
    g_app_init_progress = 2u;   // post-ErrorLatch::clear
    send_boot_trace(0xB2u, g_fdcan1_start_result);  // also silent (pre-Start)
#endif

    // FDCAN1 (accumulator + boot-trigger bus). FDCAN2 is left
    // initialised but unstarted -- the bootloader claims it after the
    // BL_BOOT_REQ_MAGIC reset, and the app no longer listens on it
    // (BmsRxTask was retired in #73; the boot-trigger frame moved to
    // FDCAN1, handled inside AcuCanTask).
    //
    // GlobalFilter: accept every unmatched standard / extended frame
    // into FIFO0 (covers VCU 0x100 ext, 0x600 std, charger 0x18FF50E7
    // ext, boot-trigger 0x002 std). Remote frames rejected on both.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
#if defined(AMS_BMS_HIL_STUB)
    g_app_init_progress = 3u;   // post-ConfigGlobalFilter
    send_boot_trace(0xB3u, g_fdcan1_start_result);  // still pre-Start, silent
#endif

    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#if defined(AMS_BMS_HIL_STUB)
    g_app_init_progress = 4u;   // post-ActivateNotification
    send_boot_trace(0xB4u, g_fdcan1_start_result);  // still pre-Start, silent
#endif

#if defined(AMS_BMS_HIL_STUB)
    g_fdcan1_start_result = static_cast<std::uint32_t>(HAL_FDCAN_Start(&hfdcan1));
    g_app_init_progress   = (g_fdcan1_start_result == HAL_OK) ? 6u : 5u;
    // First call that can ACTUALLY transmit, assuming Start succeeded.
    // If Start returned HAL_OK, this frame should appear on the bus
    // with payload[0] = 0xB6 and payload[1..4] = 0 (HAL_OK = 0).
    // If Start failed, this queues but never transmits.
    send_boot_trace((g_fdcan1_start_result == HAL_OK) ? 0xB6u : 0xB5u,
                    g_fdcan1_start_result);
#else
    (void)HAL_FDCAN_Start(&hfdcan1);
#endif

    // ----------------------------------------------------------------
    // LTC6811-1 chain bring-up + length discovery (#68 + #69).
    //
    // 1. Wake every IC in the daisy-chain with a CS pulse train.
    // 2. Issue a low-impact read (RDCFGA -- never triggers an ADC,
    //    just shifts back the current configuration register of every
    //    IC plus its PEC). If kLtcChainLength ICs answer with PEC-
    //    clean payloads, the pack wiring is sound.
    // 3. Mismatch -> latch ERROR before any relay close becomes
    //    possible. A missing or PEC-noisy module makes the pack
    //    unsafe to drive (we can't reason about cell voltages we
    //    can't observe), so the safety supervisor must boot into
    //    ERROR and refuse to leave it until the operator power-
    //    cycles with the chain healthy.
    // ----------------------------------------------------------------
#if !defined(AMS_BMS_HIL_STUB)
    auto& ltc_bus = ams::ltc6820::Bus::default_instance();
    ltc_bus.configure(&hspi1,
                      ams::ltc6820::CsPin{ LTC6820_CS_GPIO_Port, LTC6820_CS_Pin });
    ltc_bus.wakeup();

    const auto cmd_rdcfga = ams::ltc6811::pack_command(ams::ltc6811::kCmdRDCFGA);
    std::uint8_t reply[8 * ams::config::kLtcChainLength] = {};
    const bool   read_ok  = ltc_bus.read_register_group(cmd_rdcfga.data(),
                                                       reply, sizeof(reply));

    const std::uint8_t discovered = read_ok
        ? ams::ltc6811::count_pec_valid_segments(reply, sizeof(reply),
                                                 ams::config::kLtcChainLength)
        : 0u;

    if (discovered != ams::config::kLtcChainLength) {
        // Sticky-latch the fault in backup RAM (survives the watchdog
        // reset that may follow) and open all relays. MainTask will
        // see ErrorLatch::is_set()==true on its first iteration and
        // come up already latched. Refactor/19 phase 3 retired the
        // safety_eventsHandle event-flag set that used to live here;
        // there's no longer a separate consumer that would react to
        // it independently of the latch.
        ams::ErrorLatch::set();
        ams::Relays::open_all();
    }
#else
    // HIL stub: no LTC chain on the bench. Skipping discovery avoids
    // the guaranteed-fail path that would re-latch + post kForceError
    // right after we just cleared the latch above.
    (void)hspi1;
#endif

#if defined(AMS_BMS_HIL_STUB)
    g_app_init_progress = 7u;   // reached self-exit
    // Final trace before App_InitTask self-deletes. If both 0xB6 AND
    // 0xB7 appear on the wire, App_InitTask ran end-to-end and the
    // problem with MainTask telemetry is downstream.
    send_boot_trace(0xB7u, g_fdcan1_start_result);
#endif

    // Task is single-shot. CMSIS-RTOS v2: terminate self.
    osThreadExit();
}

}  // extern "C"
