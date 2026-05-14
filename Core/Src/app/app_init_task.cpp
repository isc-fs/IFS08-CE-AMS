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
#include "ams_events.hpp"
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
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef   hspi1;

void ams_app_init_task_run(void *argument)
{
    (void)argument;

    // Backup-domain write access -- safe to call from anywhere but
    // landing it here ensures it runs BEFORE SafetyTask first looks
    // at the latch (SafetyTask::run() also calls it; idempotent).
    ams::ErrorLatch::init();

    // FDCAN filter setup. CubeMX configures 0 standard / 5 extended
    // filter slots; with no software-defined filters the default for
    // unmatched frames depends on GlobalFilterConfig. Be explicit:
    // accept every unmatched standard and extended frame into FIFO0
    // on both buses. This covers:
    //   FDCAN1 (ACU bus): VCU 0x100 (ext), 0x600 (std), charger
    //     0x18FF50E7 (ext), and any future VCU frames.
    //   FDCAN2 (BMS bus): slave responses 0x12D+ (std) plus the
    //     boot-trigger frame at 0x002 (std).
    // Remote frames are rejected on both buses.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    // FDCAN bring-up: ISR notifications first, then Start. Either
    // order works but doing notifications first guarantees we don't
    // lose the first frame that arrives between Start and Notify.
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

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
    static ams::ltc6820::Bus ltc_bus{
        &hspi1,
        ams::ltc6820::CsPin{ LTC6820_CS_GPIO_Port, LTC6820_CS_Pin }
    };
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
        // reset that may follow), open all relays, and post the
        // event so SafetyTask + StateTask see ERROR on their first
        // tick. Equivalent to the BMS-stale / SDC-open path -- once
        // here, only a clean power-cycle clears it.
        ams::ErrorLatch::set();
        ams::Relays::open_all();
        if (safety_eventsHandle != nullptr) {
            osEventFlagsSet(safety_eventsHandle,
                            ams::events::safety::kForceError);
        }
    }

    // Task is single-shot. CMSIS-RTOS v2: terminate self.
    osThreadExit();
}

}  // extern "C"
