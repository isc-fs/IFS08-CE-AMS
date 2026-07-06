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
#include "fw_health.hpp"
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

// Monotonic milestone counter incremented at each App_InitTask init
// step. Surfaced on the pit-diag boot-diag frame (0x6C4[4]) so an
// engineer plugged into can0 can see how far this task got even if
// FDCAN1 never starts cleanly and other telemetry silences.
//   0 -> App_InitTask never started OR hung before any milestone
//   1 -> post-ErrorLatch::init
//   2 -> post-ErrorLatch::clear         (HIL clear-latch build only)
//   3 -> post-HAL_FDCAN_ConfigGlobalFilter
//   4 -> post-HAL_FDCAN_ActivateNotification
//   5 -> post-HAL_FDCAN_Start (regardless of return code)
//   6 -> Start returned HAL_OK
//   7 -> task self-exit reached (App_InitTask completed cleanly)
//
// HAL_FDCAN_Start's return code is captured into a sibling global,
// surfaced on 0x6C4[5..7]. The milestone byte only surfaces progress.
extern "C" volatile std::uint8_t  g_app_init_progress   = 0;
extern "C" volatile std::uint32_t g_fdcan1_start_result = 0xFFFFFFFFu;

void ams_app_init_task_run(void *argument)
{
    (void)argument;

    // Backup-domain write access -- safe to call from anywhere but
    // landing it here ensures it runs BEFORE SafetyTask first looks
    // at the latch (SafetyTask::run() also calls it; idempotent).
    ams::ErrorLatch::init();
    g_app_init_progress = 1u;   // post-ErrorLatch::init

    // Firmware-health boot capture (#411): latch the RCC reset cause and the
    // sticky last-fault sentinel (BKP3) into RAM for the 0x6CA health frame,
    // then clear the RCC flags + BKP3 so a clean next boot reports PowerOn /
    // None. Runs after ErrorLatch::init -- backup-domain access is enabled.
    ams::fw_health::capture_reset_cause();
    ams::fw_health::latch_boot_fault();

#if defined(AMS_HIL_CLEAR_ERROR_LATCH)
    // HIL-only: wipe any stale ErrorLatch on boot so a fault latched in
    // a previous bench iteration doesn't trap the chip in Error for the
    // whole run (the bench has no external clear path). RTC_BKP_DR1
    // survives a WARM reset -- the 0x002 trigger -> NVIC_SystemReset, an
    // IWDG timeout -- so a warm-reset-preserved stale latch is what this
    // clears; it also lets the bench recover from a transient LTC
    // chain-discovery glitch (Pi Pico emulator, #224) without a reset.
    //
    // NOTE: the backup domain only persists across a POWER-OFF / brown-
    // out when the carrier has a VBAT source (coin cell / supercap). The
    // MLC bench carrier has none, so a power-cycle wipes BKP_DR1 there
    // regardless of this flag; flight-HW VBAT is unconfirmed -- see #324
    // for the sticky-error-across-power-cycle gap this exposed.
    //
    // NEVER compiled into flight HW: clearing the latch on boot defeats
    // the sticky-error contract -- a latched pre-charge / SDC / cell
    // fault must NOT be wiped by a reboot.
    ams::ErrorLatch::clear();
    g_app_init_progress = 2u;   // post-ErrorLatch::clear
#endif

    // FDCAN1 (accumulator + boot-trigger bus). FDCAN2 is left
    // initialised but unstarted -- the bootloader claims it after the
    // BL_BOOT_REQ_MAGIC reset, and the app no longer listens on it
    // (BmsRxTask was retired in #73; the boot-trigger frame moved to
    // FDCAN1, handled inside AcuCanTask).
    //
    // GlobalFilter: accept every unmatched standard frame into FIFO0;
    // REJECT extended frames at the hardware filter. The AMS is
    // standard-only on FDCAN1 -- nothing the firmware listens for is
    // 29-bit (VCU 0x100, boot-trigger 0x002 both fit in 11 bits; the
    // legacy 0x18FF50E7 charger-detect was retired in fix/48). Rejecting
    // extended at the HW gate keeps the RX ISR off junk frames and
    // makes the contract crisp. Remote frames rejected on both kinds.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
    g_app_init_progress = 3u;   // post-ConfigGlobalFilter

    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    g_app_init_progress = 4u;   // post-ActivateNotification

    g_fdcan1_start_result = static_cast<std::uint32_t>(HAL_FDCAN_Start(&hfdcan1));
    g_app_init_progress   = (g_fdcan1_start_result == HAL_OK) ? 6u : 5u;

    // ----------------------------------------------------------------
    // LTC6811-1 chain bring-up + length discovery (#68 + #69).
    //
    // 1. Wake every IC in the daisy-chain with a CS pulse train.
    // 2. Issue a low-impact read (RDCFGA -- never triggers an ADC,
    //    just shifts back the current configuration register of every
    //    IC plus its PEC). If LtcChainLength ICs answer with PEC-
    //    clean payloads, the pack wiring is sound.
    // 3. Mismatch -> latch ERROR before any relay close becomes
    //    possible. A missing or PEC-noisy module makes the pack
    //    unsafe to drive (we can't reason about cell voltages we
    //    can't observe), so the safety supervisor must boot into
    //    ERROR and refuse to leave it until the operator power-
    //    cycles with the chain healthy.
    // ----------------------------------------------------------------
    // LTC chain discovery runs on every boot regardless of build flavour.
    // On the HIL bench this requires the Pi Pico LTC6820+LTC6811 emulator
    // (IFS08_HIL feat/pico-ltc-emulator) to be wired and running on
    // MLC2 J8; without it discovery fails, the latch sticks, and the
    // bench comes up in Error -- which is the correct behaviour now that
    // the BMS-data stub is gone (see #205).
    auto& ltc_bus = ams::ltc6820::Bus::default_instance();
    ltc_bus.configure(&hspi1,
                      ams::ltc6820::CsPin{ LTC6820_CS_GPIO_Port, LTC6820_CS_Pin });
    ltc_bus.wakeup();

    const auto cmd_rdcfga = ams::ltc6811::pack_command(ams::ltc6811::CmdRDCFGA);
    std::uint8_t reply[8 * ams::config::LtcChainLength] = {};
    const bool   read_ok  = ltc_bus.read_register_group(cmd_rdcfga.data(),
                                                       reply, sizeof(reply));

    const std::uint8_t discovered = read_ok
        ? ams::ltc6811::count_pec_valid_segments(reply, sizeof(reply),
                                                 ams::config::LtcChainLength)
        : 0u;

    if (discovered != ams::config::LtcChainLength) {
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

    g_app_init_progress = 7u;   // reached self-exit

    // Task is single-shot. CMSIS-RTOS v2: terminate self.
    osThreadExit();
}

}  // extern "C"
