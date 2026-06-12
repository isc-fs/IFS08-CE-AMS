// SPDX-License-Identifier: proprietary
//
// Drives the LTC6811-1 daisy-chain to acquire cell voltages and, in a
// follow-up branch (#71), cell temperatures via the per-LTC ADG731
// 32:1 mux. Replaces the legacy FDCAN2 poll emitter; the wire-format
// layer lives in ltc6811.hpp / ltc6820.hpp.
//
// Cadence (unchanged from the legacy task):
//   * PollVDue every BmsPollVoltMs (250 ms) -> ADCV + RDCVA/B/C/D
//   * PollTDue every BmsPollTempMs (500 ms) -> 20-channel mux sweep
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
#include "vehicle_service.hpp"

#include "cmsis_os2.h"

#include <cstddef>
#include <cstdint>

// FSM state mirror, written by MainTask on every transition (the FSM
// step body lives inside MainTask since refactor/19 phase 3). Reading
// a single byte from another task is safe without a lock; we only
// need a coherent snapshot at the call point, which volatile + 8-bit
// read guarantees on Cortex-M7.
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
// within 50 ms" acceptance criterion without a scope. Promoted to
// extern "C" external linkage so the pit-diag stream (#247) can
// surface them via AcuCanTask.
}  // close anonymous namespace temporarily for the extern "C" decls

extern "C" volatile std::uint32_t g_bms_volt_poll_ms  = 0;
extern "C" volatile std::uint32_t g_bms_volt_poll_max = 0;

// DCC mask snapshot from the last balance cycle, exposed for pit-diag
// (#247). bit c of g_balance_dcc_bits[m] == 1 iff cell c of module m
// was selected for discharge this cycle. 19 cells per module fit in
// the low 19 bits of each uint32; bits 19..31 are always 0.
extern "C" volatile std::uint32_t g_balance_dcc_bits[5] = {0, 0, 0, 0, 0};
extern "C" volatile std::uint32_t g_balance_cycles_total_pub  = 0;
extern "C" volatile std::uint32_t g_balance_cycles_active_pub = 0;

namespace {

// Balancing-update counters: cycles since last WRCFGA + total
// cycles where at least one DCC bit was set. Surfaced for HIL.
volatile std::uint32_t g_balance_cycles_total  = 0;
volatile std::uint32_t g_balance_cycles_active = 0;
std::uint32_t          s_volt_poll_count       = 0;

void volt_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::PollVDue);
}

void temp_timer_cb(void * /*arg*/) {
    osEventFlagsSet(bms_eventsHandle, ams::events::bms::PollTDue);
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
        ltc6811::adcv_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                          /*discharge_permit=*/false,
                          ltc6811::CellSel::All));
    if (!bus.send_command(adcv.data())) {
        ++g_ltc_spi_err_count;
        return;
    }

    // 2. ADC settling. Norm-7kHz mode converts all 12 channels in
    //    ~2.3 ms; we round to 3 ms (config::AdcvSettleMs). osDelay
    //    rounds up to the next FreeRTOS tick (1 Hz) so worst-case
    //    wait is 3-4 ms -- well below the 50 ms budget.
    osDelay(config::AdcvSettleMs);

    // 3. Warm-up cmd before RDCVA (#214). After the multi-ms idle
    //    between ADCV+settle and the first RDCV, MOSI drifts toward
    //    its idle-high level long enough that slaves which re-sync
    //    on CS edges (e.g. the Pi Pico LTC6820 emulator on the HIL
    //    bench) sample a stray HIGH as bit 7 of byte 0 of RDCVA --
    //    PEC then mismatches for every IC in the chain. Issuing a
    //    no-op RDCFGA first burns the stale-MOSI sample into a cmd
    //    whose reply we discard; the subsequent RDCV* commands come
    //    back-to-back with MOSI continuously driven, so bit-sync
    //    holds. ~700 us at 1 MHz SCK, well under the 50 ms budget.
    //
    //    TODO: replace settle delay + warm-up with PLADC polling
    //    (LTC6811 cmd 0x0714) so we hold CS low across the
    //    conversion and there's no idle gap to bridge.
    const auto rdcfga = ltc6811::pack_command(ltc6811::CmdRDCFGA);
    std::uint8_t warmup_reply[8 * config::LtcChainLength];
    if (!bus.read_register_group(rdcfga.data(),
                                 warmup_reply, sizeof(warmup_reply))) {
        ++g_ltc_spi_err_count;
        return;
    }

    // 4. Read the four cell-voltage register groups into one
    //    contiguous buffer that BmsService::update_from_ltc_response
    //    can walk. Group layout:
    //      [A: 10 segments][B: 10 segments][C: 10 segments][D: 10 segments]
    constexpr std::size_t SegBytes   = 8;
    constexpr std::size_t GroupBytes = config::LtcChainLength * SegBytes;
    std::uint8_t          reply[4 * GroupBytes] = {};

    static constexpr std::uint16_t RdcvCmds[4] = {
        ltc6811::CmdRDCVA, ltc6811::CmdRDCVB,
        ltc6811::CmdRDCVC, ltc6811::CmdRDCVD,
    };
    for (std::uint8_t g = 0; g < 4; ++g) {
        const auto cmd = ltc6811::pack_command(RdcvCmds[g]);
        if (!bus.read_register_group(cmd.data(),
                                     reply + g * GroupBytes,
                                     GroupBytes)) {
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
// Cell balancing (#74). Once every BalanceUpdatePolls voltage cycles
// we snapshot BmsState, ask BalanceController what to discharge, pack
// DCC bits into 10 WRCFGA payloads, and broadcast them. Outside of
// fsm::State::Charge the controller returns an all-zero mask so the
// next WRCFGA clears whatever was set previously.
// ---------------------------------------------------------------------------
void maybe_run_balance_update() {
    using namespace ams;

    if (++s_volt_poll_count < config::BalanceUpdatePolls) return;
    s_volt_poll_count = 0;

    const auto       state    = BmsService::instance().snapshot();
    const fsm::State fsm_curr =
        static_cast<fsm::State>(g_state_telemetry);
    // Operator balance override (#336): pause autonomous balancing while a
    // fresh "BALO" (0x103) is in effect. Reverts to auto on "BALX" or when
    // the override goes stale.
    const auto       veh      = VehicleService::instance().snapshot();
    const bool       suppress = VehicleService::balance_suppressed(
        osKernelGetTickCount(), veh.last_balance_override_tick,
        veh.balance_override_suppress);
    const auto       mask     = balance::compute_mask(state, fsm_curr, suppress);

    std::uint8_t per_ic[config::LtcChainLength][6];
    bool         any_dcc = false;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        // LTC_1 (upper, chain index 2m) owns module cells 0..9.
        std::uint16_t dcc_upper = 0;
        for (std::uint8_t c = 0; c < config::CellsPerLtcUpper; ++c) {
            if (mask.cell[m][c]) {
                dcc_upper = static_cast<std::uint16_t>(dcc_upper | (1u << c));
            }
        }
        // LTC_2 (lower, chain index 2m+1) owns module cells 10..18.
        std::uint16_t dcc_lower = 0;
        for (std::uint8_t c = 0; c < config::CellsPerLtcLower; ++c) {
            if (mask.cell[m][config::CellsPerLtcUpper + c]) {
                dcc_lower = static_cast<std::uint16_t>(dcc_lower | (1u << c));
            }
        }

        const auto upper = ltc6811::pack_cfga_payload(dcc_upper);
        const auto lower = ltc6811::pack_cfga_payload(dcc_lower);
        for (std::size_t k = 0; k < 6; ++k) {
            per_ic[2u * m + 0u][k] = upper[k];
            per_ic[2u * m + 1u][k] = lower[k];
        }

        // Publish the per-module DCC selection so pit-diag (#247) can
        // surface "which cell is being balanced right now" without
        // a debugger probe. dcc_upper covers cells 0..9, dcc_lower covers 10..18 of
        // the module -- shift the lower half by CellsPerLtcUpper so a
        // single uint32 mirrors the cell index layout.
        g_balance_dcc_bits[m] =
            static_cast<std::uint32_t>(dcc_upper) |
            (static_cast<std::uint32_t>(dcc_lower) << config::CellsPerLtcUpper);

        if (dcc_upper != 0u || dcc_lower != 0u) any_dcc = true;
    }

    if (!ltc6820::Bus::default_instance().write_chain_command(
            ltc6811::CmdWRCFGA, per_ic)) {
        ++g_ltc_spi_err_count;
        return;
    }

    ++g_balance_cycles_total;
    if (any_dcc) ++g_balance_cycles_active;
    // Mirror to extern-linkage copies for pit-diag.
    g_balance_cycles_total_pub  = g_balance_cycles_total;
    g_balance_cycles_active_pub = g_balance_cycles_active;
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
// Per-channel sweep-failure tracking. `last_mask` reflects ONLY the most
// recent sweep (cleared at the start). `sticky_mask` is OR-accumulated
// across all sweeps since boot; useful for catching intermittent NTC /
// mux failures that don't show up every cycle. Reset on next boot by
// writing 0 to sticky_mask.
extern "C" volatile std::uint32_t g_temp_sweep_last_mask   = 0;
extern "C" volatile std::uint32_t g_temp_sweep_sticky_mask = 0;

void run_temperature_poll() {
    using namespace ams;

#if defined(AMS_TEMP_STUB) && AMS_TEMP_STUB
    // Bench bring-up: no NTC sensor PCB. Skip the ADG731/ADAX mux sweep
    // entirely (there's nothing to talk to) and pin every temperature
    // slot to a nominal in-range value so the over/under-temp predicate
    // passes -- BMS-voltage + FSM integration can be exercised without
    // the temp hardware. Voltage polls still arm first_full_poll_done,
    // so cell-voltage faults are detected normally. NEVER in a flight
    // build (CMake AMS_TEMP_STUB defaults OFF).
    BmsService::instance().set_all_temperatures(config::TempStubValueC);
    g_temp_sweep_last_mask   = 0;
    g_temp_sweep_sticky_mask = 0;
    return;
#else
    auto& bus = ltc6820::Bus::default_instance();

    // Same mux-select payload broadcast to every LTC each step. The
    // ADG731 ignores the bits it can't address (only ch < 32 used).
    std::uint8_t per_ic_payload[config::LtcChainLength][6];

    // Per-sweep failure tracking. Each bit corresponds to one
    // temperature-table channel (ch_idx 0..19) that failed at any of
    // the WRCOMM / STCOMM / ADAX / RDAUXA steps. Captured at end of
    // sweep into the globals so the bench can localise which NTC /
    // mux is misbehaving.
    std::uint32_t this_sweep_fail = 0;

    for (std::uint8_t ch_idx = 0; ch_idx < config::TempsPerLtc; ++ch_idx) {
        const std::uint8_t mux_ch = config::Adg731ChannelMap[ch_idx];
        const auto sel = ltc6811::pack_adg731_select(mux_ch);

        for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
            for (std::size_t k = 0; k < 6; ++k) {
                per_ic_payload[ic][k] = sel[k];
            }
        }

        // 1. WRCOMM: load the select word into every IC's COMM reg.
        if (!bus.write_chain_command(ltc6811::CmdWRCOMM, per_ic_payload)) {
            ++g_ltc_spi_err_count;
            this_sweep_fail |= (1u << ch_idx);
            continue;
        }
        // 2. STCOMM: shift COMM register out -> mux receives.
        if (!bus.stcomm()) {
            ++g_ltc_spi_err_count;
            this_sweep_fail |= (1u << ch_idx);
            continue;
        }
        // 3. Settling for the mux + NTC voltage-divider. The
        //    osDelay tick (1 Hz) is plenty; ADG731 t_TRANSITION is
        //    ~80 ns and the 10 k / 10 k divider settles in << 1 ms.
        osDelay(1);

        // 4. ADAX(Gpio1) broadcast -> AUX-ADC conversion on every IC.
        const auto adax_cmd = ltc6811::pack_command(
            ltc6811::adax_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                              ltc6811::AuxSel::Gpio1));
        if (!bus.send_command(adax_cmd.data())) {
            ++g_ltc_spi_err_count;
            this_sweep_fail |= (1u << ch_idx);
            continue;
        }
        osDelay(config::AdaxSettleMs);

        // 5. RDAUXA: read AUX1..AUX3 + PEC per IC.
        constexpr std::size_t Reply = config::LtcChainLength * 8u;
        std::uint8_t reply[Reply] = {};
        const auto rdauxa = ltc6811::pack_command(ltc6811::CmdRDAUXA);
        if (!bus.read_register_group(rdauxa.data(), reply, sizeof(reply))) {
            ++g_ltc_spi_err_count;
            this_sweep_fail |= (1u << ch_idx);
            continue;
        }

        (void)BmsService::instance().update_temperature(ch_idx, reply, sizeof(reply));
    }

    g_temp_sweep_last_mask    = this_sweep_fail;
    g_temp_sweep_sticky_mask |= this_sweep_fail;
#endif  // AMS_TEMP_STUB
}

}  // namespace

extern "C" void ams_bms_poll_task_run(void *argument) {
    (void)argument;

    s_volt_timer = osTimerNew(&volt_timer_cb, osTimerPeriodic, nullptr, nullptr);
    s_temp_timer = osTimerNew(&temp_timer_cb, osTimerPeriodic, nullptr, nullptr);

    if (s_volt_timer != nullptr) {
        osTimerStart(s_volt_timer, ams::config::BmsPollVoltMs);
    }
    if (s_temp_timer != nullptr) {
        osTimerStart(s_temp_timer, ams::config::BmsPollTempMs);
    }

    constexpr std::uint32_t All =
        ams::events::bms::PollVDue | ams::events::bms::PollTDue;

    for (;;) {
        const std::uint32_t evt = osEventFlagsWait(
            bms_eventsHandle, All, osFlagsWaitAny, osWaitForever);

        if ((evt & osFlagsError) != 0u) {
            // Event group went away; back off and keep waiting. The
            // group is statically allocated for the lifetime of the
            // app, so this branch is defensive.
            osDelay(50);
            continue;
        }

        if (evt & ams::events::bms::PollVDue) {
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
        if (evt & ams::events::bms::PollTDue) {
            run_temperature_poll();
        }
    }
}
