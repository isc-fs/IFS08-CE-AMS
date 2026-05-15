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
#include "balance_controller.hpp"
#include "bms_service.hpp"
#include "ltc6811.hpp"
#include "ltc6820.hpp"
#include "state_machine.hpp"

#include "cmsis_os2.h"

#include <cstddef>
#include <cstdint>

// FSM state mirror, owned by StateTask. Reading a single byte from
// another task is safe without a lock; we only need a coherent
// snapshot at the call point, which volatile + 8-bit read guarantees
// on Cortex-M7.
extern "C" volatile std::uint8_t g_state_telemetry;

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

// Balancing-update counters: cycles since last WRCFGA + total
// cycles where at least one DCC bit was set. Surfaced for HIL.
volatile std::uint32_t g_balance_cycles_total  = 0;
volatile std::uint32_t g_balance_cycles_active = 0;
std::uint32_t          s_volt_poll_count       = 0;

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
// Cell balancing (#74). Once every kBalanceUpdatePolls voltage cycles
// we snapshot BmsState, ask BalanceController what to discharge, pack
// DCC bits into 10 WRCFGA payloads, and broadcast them. Outside of
// fsm::State::Charge the controller returns an all-zero mask so the
// next WRCFGA clears whatever was set previously.
// ---------------------------------------------------------------------------
void maybe_run_balance_update() {
    using namespace ams;

    if (++s_volt_poll_count < config::kBalanceUpdatePolls) return;
    s_volt_poll_count = 0;

    const auto       state    = BmsService::instance().snapshot();
    const fsm::State fsm_curr =
        static_cast<fsm::State>(g_state_telemetry);
    const auto       mask     = balance::compute_mask(state, fsm_curr);

    std::uint8_t per_ic[config::kLtcChainLength][6];
    bool         any_dcc = false;

    for (std::uint8_t m = 0; m < config::kBmsModuleCount; ++m) {
        // LTC_1 (upper, chain index 2m) owns module cells 0..9.
        std::uint16_t dcc_upper = 0;
        for (std::uint8_t c = 0; c < config::kCellsPerLtcUpper; ++c) {
            if (mask.cell[m][c]) {
                dcc_upper = static_cast<std::uint16_t>(dcc_upper | (1u << c));
            }
        }
        // LTC_2 (lower, chain index 2m+1) owns module cells 10..18.
        std::uint16_t dcc_lower = 0;
        for (std::uint8_t c = 0; c < config::kCellsPerLtcLower; ++c) {
            if (mask.cell[m][config::kCellsPerLtcUpper + c]) {
                dcc_lower = static_cast<std::uint16_t>(dcc_lower | (1u << c));
            }
        }

        const auto upper = ltc6811::pack_cfga_payload(dcc_upper);
        const auto lower = ltc6811::pack_cfga_payload(dcc_lower);
        for (std::size_t k = 0; k < 6; ++k) {
            per_ic[2u * m + 0u][k] = upper[k];
            per_ic[2u * m + 1u][k] = lower[k];
        }

        if (dcc_upper != 0u || dcc_lower != 0u) any_dcc = true;
    }

    if (!ltc6820::Bus::default_instance().write_chain_command(
            ltc6811::kCmdWRCFGA, per_ic)) {
        ++g_ltc_spi_err_count;
        return;
    }

    ++g_balance_cycles_total;
    if (any_dcc) ++g_balance_cycles_active;
}

// ---------------------------------------------------------------------------
// Temperature poll. Lands fully in #71 (ADG731 mux sweep + ADAX +
// RDAUXA per channel). Stubbed here so the periodic timer fires and
// the rest of the task structure is in place; right now it just
// pumps the chain (idle wakeup) so the LTCs don't drop into T_SLEEP
// (~2 s) when the operator has paused the voltage loop for debug.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Temperature poll: 20-step ADG731 mux sweep.
//
//   for ch_idx in 0..19:
//     WRCOMM  load(pack_adg731_select(map[ch_idx])) for every IC
//     STCOMM  shift the COMM register out -> mux selects channel
//     1 ms    mux + NTC divider settle
//     ADAX    AUX-ADC conversion (AUX1 only -> Gpio1)
//     1 ms    ADC settle
//     RDAUXA  chain reply -> BmsService::update_temperature
//
// 20 channels x ~3 ms ~= 60 ms total -- well below the 500 ms poll
// budget and the 100 ms acceptance criterion stays achievable with
// HAL/jitter overhead.
// ---------------------------------------------------------------------------
void run_temperature_poll() {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();

    // Same mux-select payload broadcast to every LTC each step. The
    // ADG731 ignores the bits it can't address (only ch < 32 used).
    std::uint8_t per_ic_payload[config::kLtcChainLength][6];

    for (std::uint8_t ch_idx = 0; ch_idx < config::kTempsPerLtc; ++ch_idx) {
        const std::uint8_t mux_ch = config::kAdg731ChannelMap[ch_idx];
        const auto sel = ltc6811::pack_adg731_select(mux_ch);

        for (std::uint8_t ic = 0; ic < config::kLtcChainLength; ++ic) {
            for (std::size_t k = 0; k < 6; ++k) {
                per_ic_payload[ic][k] = sel[k];
            }
        }

        // 1. WRCOMM: load the select word into every IC's COMM reg.
        if (!bus.write_chain_command(ltc6811::kCmdWRCOMM, per_ic_payload)) {
            ++g_ltc_spi_err_count;
            continue;
        }
        // 2. STCOMM: shift COMM register out -> mux receives.
        if (!bus.stcomm()) {
            ++g_ltc_spi_err_count;
            continue;
        }
        // 3. Settling for the mux + NTC voltage-divider. The
        //    osDelay tick (1 kHz) is plenty; ADG731 t_TRANSITION is
        //    ~80 ns and the 10 k / 10 k divider settles in << 1 ms.
        osDelay(1);

        // 4. ADAX(Gpio1) broadcast -> AUX-ADC conversion on every IC.
        const auto adax_cmd = ltc6811::pack_command(
            ltc6811::adax_cmd(static_cast<ltc6811::AdcMode>(config::kAdcMode),
                              ltc6811::AuxSel::Gpio1));
        if (!bus.send_command(adax_cmd.data())) {
            ++g_ltc_spi_err_count;
            continue;
        }
        osDelay(config::kAdaxSettleMs);

        // 5. RDAUXA: read AUX1..AUX3 + PEC per IC.
        constexpr std::size_t kReply = config::kLtcChainLength * 8u;
        std::uint8_t reply[kReply] = {};
        const auto rdauxa = ltc6811::pack_command(ltc6811::kCmdRDAUXA);
        if (!bus.read_register_group(rdauxa.data(), reply, sizeof(reply))) {
            ++g_ltc_spi_err_count;
            continue;
        }

        (void)BmsService::instance().update_temperature(ch_idx, reply, sizeof(reply));
    }
}

}  // namespace

extern "C" void ams_bms_poll_task_run(void *argument) {
    (void)argument;

#if defined(AMS_BMS_HIL_STUB)
    // HIL stub: no LTC chain on the bench, so don't touch SPI at all.
    // Seed a nominal-healthy snapshot every kBmsPollVoltMs (250 ms),
    // which keeps last_rx_tick fresh and module_online_mask = 0x1F so
    // the (unmodified) predicate sees healthy BMS data. The whole
    // chain-discovery / WRCFGA / ADCV / RDCV* path is compiled out
    // of this branch -- not "guarded at runtime", literally not in
    // the binary. Predicate file stays HIL-agnostic.
    for (;;) {
        ams::BmsService::instance().seed_for_hil_stub(osKernelGetTickCount());
        osDelay(ams::config::kBmsPollVoltMs);
    }
#else
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
            // Balancing piggybacks on the V-poll cadence; the
            // controller is gated by FSM state internally so it's a
            // no-op outside Charge.
            maybe_run_balance_update();
            const std::uint32_t dt = osKernelGetTickCount() - t0;
            g_bms_volt_poll_ms = dt;
            if (dt > g_bms_volt_poll_max) g_bms_volt_poll_max = dt;
        }
        if (evt & ams::events::bms::kPollTDue) {
            run_temperature_poll();
        }
    }
#endif  // AMS_BMS_HIL_STUB
}
