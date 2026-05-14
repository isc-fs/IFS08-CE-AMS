// SPDX-License-Identifier: proprietary
//
// Drives the LTC6811-1 daisy-chain to acquire cell voltages and, in a
// follow-up branch (#71), cell temperatures via the per-LTC ADG731
// 32:1 mux. Replaces the legacy FDCAN2 poll emitter; the wire-format
// layer lives in ltc6811.hpp / ltc6820.hpp.
//
// Cadence (unchanged from the legacy task):
//   * kPollVDue every kBmsPollVoltMs (250 ms) -> ADCV + RDCVA/B/C/D
//   * kPollTDue every kBmsPollTempMs (500 ms) -> 20-channel mux sweep
//                                                (stub here, lands in #71)
//
// Mechanism: two osTimers raise event-flag bits, this task wakes on
// osEventFlagsWait. Single producer over the SPI bus, so no mutex
// around the HAL_SPI calls.
//
// Per docs/ARCHITECTURE.md §2 task table.

#include "app/bms_poll_task.h"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "app/app_globals.h"
#include "bms_service.hpp"
#include "ltc6811.hpp"
#include "ltc6820.hpp"

#include "cmsis_os2.h"

#include <cstddef>
#include <cstdint>

namespace {

osTimerId_t s_volt_timer = nullptr;
osTimerId_t s_temp_timer = nullptr;

// Telemetry counters. Volatile so a remote-debug session can read
// them via the symbol; not part of any task's hot path budget. The
// PEC-error counter already lives in bms_service.cpp (per-IC, 10
// entries); these are bus-level failures (HAL_OK != 0).
volatile std::uint32_t g_ltc_spi_err_count = 0;

// Round-trip timing for the voltage poll, both last-cycle and worst-
// case-since-boot. Lets the HIL operator verify the issue's "complete
// within 50 ms" acceptance criterion without a scope.
volatile std::uint32_t g_bms_volt_poll_ms  = 0;
volatile std::uint32_t g_bms_volt_poll_max = 0;

void volt_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::kPollVDue);
}

void temp_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::kPollTDue);
}

// ---------------------------------------------------------------------------
// Voltage poll: ADCV broadcast (kicks all 10 LTCs into a cell-V
// conversion) followed by 4 register-group reads (RDCVA, RDCVB, RDCVC,
// RDCVD) concatenated into a 320-byte buffer that BmsService digests
// in one mutex-acquire.
// ---------------------------------------------------------------------------
void run_voltage_poll() {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();

    // 1. ADCV broadcast. Discharge-permit = false during normal data
    //    acquisition (#74 will flip it for balancing windows). All
    //    cells channel-select = CellSel::All.
    const auto adcv = ltc6811::pack_command(
        ltc6811::adcv_cmd(static_cast<ltc6811::AdcMode>(config::kAdcMode),
                          /*discharge_permit=*/false,
                          ltc6811::CellSel::All));
    if (!bus.send_command(adcv.data())) {
        ++g_ltc_spi_err_count;
        return;
    }

    // 2. ADC settling. Norm-7kHz mode converts all 12 channels in
    //    ~2.3 ms; we round to 3 ms (config::kAdcvSettleMs). osDelay
    //    rounds up to the next FreeRTOS tick (1 kHz) so worst-case
    //    wait is 3-4 ms -- well below the 50 ms budget.
    osDelay(config::kAdcvSettleMs);

    // 3. Read the four cell-voltage register groups into one
    //    contiguous buffer that BmsService::update_from_ltc_response
    //    can walk. Group layout:
    //      [A: 10 segments][B: 10 segments][C: 10 segments][D: 10 segments]
    constexpr std::size_t kSegBytes   = 8;
    constexpr std::size_t kGroupBytes = config::kLtcChainLength * kSegBytes;
    std::uint8_t          reply[4 * kGroupBytes] = {};

    static constexpr std::uint16_t kRdcvCmds[4] = {
        ltc6811::kCmdRDCVA, ltc6811::kCmdRDCVB,
        ltc6811::kCmdRDCVC, ltc6811::kCmdRDCVD,
    };
    for (std::uint8_t g = 0; g < 4; ++g) {
        const auto cmd = ltc6811::pack_command(kRdcvCmds[g]);
        if (!bus.read_register_group(cmd.data(),
                                     reply + g * kGroupBytes,
                                     kGroupBytes)) {
            ++g_ltc_spi_err_count;
            return;  // partial reply -> don't poison BmsService state
        }
    }

    // 4. Hand the assembled chain response to BmsService. Per-IC PEC
    //    is checked inside; bus-level failures already returned above.
    const std::uint32_t now_ms = osKernelGetTickCount();
    (void)BmsService::instance().update_from_ltc_response(
        reply, sizeof(reply), now_ms);
}

// ---------------------------------------------------------------------------
// Temperature poll. Lands fully in #71 (ADG731 mux sweep + ADAX +
// RDAUXA per channel). Stubbed here so the periodic timer fires and
// the rest of the task structure is in place; right now it just
// pumps the chain (idle wakeup) so the LTCs don't drop into T_SLEEP
// (~2 s) when the operator has paused the voltage loop for debug.
// ---------------------------------------------------------------------------
void run_temperature_poll() {
    ams::ltc6820::Bus::default_instance().wakeup();
    // TODO(#71): ADG731 mux sweep + ADAX + RDAUXA, then
    //            BmsService::update_temps_from_ltc_response(...).
}

}  // namespace

extern "C" void ams_bms_poll_task_run(void *argument) {
    (void)argument;

    s_volt_timer = osTimerNew(&volt_timer_cb, osTimerPeriodic, nullptr, nullptr);
    s_temp_timer = osTimerNew(&temp_timer_cb, osTimerPeriodic, nullptr, nullptr);

    if (s_volt_timer != nullptr) {
        osTimerStart(s_volt_timer, ams::config::kBmsPollVoltMs);
    }
    if (s_temp_timer != nullptr) {
        osTimerStart(s_temp_timer, ams::config::kBmsPollTempMs);
    }

    constexpr std::uint32_t kAll =
        ams::events::bms::kPollVDue | ams::events::bms::kPollTDue;

    for (;;) {
        const std::uint32_t evt = osEventFlagsWait(
            bms_eventsHandle, kAll, osFlagsWaitAny, osWaitForever);

        if ((evt & osFlagsError) != 0u) {
            // Event group went away; back off and keep waiting. The
            // group is statically allocated for the lifetime of the
            // app, so this branch is defensive.
            osDelay(50);
            continue;
        }

        if (evt & ams::events::bms::kPollVDue) {
            const std::uint32_t t0 = osKernelGetTickCount();
            run_voltage_poll();
            const std::uint32_t dt = osKernelGetTickCount() - t0;
            g_bms_volt_poll_ms = dt;
            if (dt > g_bms_volt_poll_max) g_bms_volt_poll_max = dt;
        }
        if (evt & ams::events::bms::kPollTDue) {
            run_temperature_poll();
        }
    }
}
