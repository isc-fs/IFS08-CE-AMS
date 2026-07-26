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
#include "fw_health.hpp"
#include "ltc6811.hpp"
#include "ltc6820.hpp"
#include "state_machine.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

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

// Chain-recovery counter: incremented every time run_voltage_poll re-wakes
// and reconfigures the chain after consecutive failed polls. Zero on a
// healthy bus; climbing means the chain is repeatedly dropping out (the
// inverter-EMI T_SLEEP case this recovery exists for). extern "C" so the
// pit-diag stream can surface it.
extern "C" volatile std::uint32_t g_ltc_chain_recover_count = 0;

// Times the voltage poll cleared the DCC bits before measuring (#balance
// measurement integrity). Climbs at the poll rate whenever balancing is
// actually discharging, and stays flat when it is not -- so the bench can
// confirm from CAN alone that the quiesce is running.
extern "C" volatile std::uint32_t g_balance_quiesce_count = 0;

namespace {

// Balancing-update counters: cycles since last WRCFGA + total
// cycles where at least one DCC bit was set. Surfaced for HIL.
volatile std::uint32_t g_balance_cycles_total  = 0;
volatile std::uint32_t g_balance_cycles_active = 0;
std::uint32_t          s_volt_poll_count       = 0;

// Last CFGA payload actually written to the chain, and whether it asserted any
// DCC bit. The mask is only recomputed every BalanceUpdatePolls (1 Hz) but
// PERSISTS on the chain between writes, so quiescing for a measurement has to
// put it back afterwards -- otherwise clearing it every 250 ms poll would
// destroy three quarters of the balancing duty.
std::uint8_t s_last_cfga[ams::config::LtcChainLength][6] = {};
bool         s_balance_active = false;

// ---------------------------------------------------------------------------
// Chain-sleep recovery.
//
// The LTC6811 drops into T_SLEEP (~2 s) after its last VALID command, and a
// sleeping IC ignores every normal command -- only the CS-low wake pulse
// train (Bus::wakeup) brings it back. Until this landed, wakeup() was issued
// exactly once at boot (app_init_task) and never again, so the chain survived
// only on uninterrupted poll traffic: any interruption longer than T_SLEEP
// (e.g. inverter switching noise corrupting commands through a torque event)
// put the chain to sleep permanently, with no code path able to recover it.
//
// So: count consecutive failed polls, and once the chain looks gone, re-wake
// + reconfigure before each subsequent attempt. Retrying every poll (rather
// than once) is deliberate -- while the disturbance lasts the wake won't take,
// and we want the chain back on the first poll after it clears.
//
// Waking an already-awake chain is harmless (CS pulses carry no command), so
// a false positive costs ~500 us, not correctness.
// ---------------------------------------------------------------------------

// Consecutive failed voltage polls before recovery kicks in. 2 polls =
// 500 ms at BmsPollVoltMs (250 ms), comfortably inside BmsStaleMs (1500 ms)
// so a recovered chain never reaches the stale predicate. Not 1: an isolated
// PEC glitch shouldn't clear DCC bits mid-balance for no reason.
constexpr std::uint32_t RecoverAfterFailedPolls = 2;

std::uint32_t s_consecutive_poll_failures = 0;

// Re-establish the chain after a suspected T_SLEEP: wake it, then restore
// CFGR. The reconfigure is NOT optional -- sleeping resets CFGR to defaults,
// which re-enables the GPIO pull-downs and would short the ADG731 mux / NTC
// divider that the ADAX temperature path reads, and drops REFON. DCC is
// written all-zero (discharge off); maybe_run_balance_update restores the
// real mask on its next cycle.
void recover_chain() noexcept {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();
    bus.wakeup();

    // The chain slept, so CFGR is back at defaults and nothing is discharging.
    // Drop the balance cache: restoring a pre-sleep mask would re-assert bits
    // the controller has not re-derived since. The next 1 Hz update recomputes.
    s_balance_active = false;

    const auto   cfg = ltc6811::pack_cfga_payload(/*dcc_bits=*/0u);
    std::uint8_t per_ic[config::LtcChainLength][6];
    for (std::size_t i = 0; i < config::LtcChainLength; ++i) {
        std::memcpy(per_ic[i], cfg.data(), 6);
    }
    (void)bus.write_chain_command(ltc6811::CmdWRCFGA, per_ic);

    ++g_ltc_chain_recover_count;
}

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
// Result of one voltage-poll attempt.
struct VoltAttempt {
    bool         any_module_fresh;  // >= 1 module had BOTH its LTCs PEC-clean
    std::uint8_t clean_ltcs;        // count of PEC-clean ICs this attempt
};

// One voltage-poll attempt: ADCV -> settle -> warm-up -> RDCVA/B/C/D -> digest.
// update_from_ltc_response refreshes last_rx_tick for whichever modules came
// back PEC-clean, so calling this repeatedly (the retry loop below) gives each
// module more chances to report. Returns {false, 0} on any bus-level failure.
VoltAttempt attempt_voltage_poll() {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();

    // 1. ADCV broadcast. Discharge-permit = false during normal data
    //    acquisition (#74 flips it for balancing windows). Cells = All.
    const auto adcv = ltc6811::pack_command(
        ltc6811::adcv_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                          /*discharge_permit=*/false,
                          ltc6811::CellSel::All));
    if (!bus.send_command(adcv.data())) {
        ++g_ltc_spi_err_count;
        return { false, 0 };
    }

    // 2. ADC settling. Norm-7kHz mode converts all 12 channels in
    //    ~2.3 ms; we round to 3 ms (config::AdcvSettleMs).
    osDelay(config::AdcvSettleMs);

    // 3. Warm-up cmd before RDCVA (#214). After the multi-ms idle between
    //    ADCV+settle and the first RDCV, MOSI drifts toward its idle-high
    //    level long enough that slaves which re-sync on CS edges (e.g. the Pi
    //    Pico LTC6820 emulator on the HIL bench) sample a stray HIGH as bit 7
    //    of byte 0 of RDCVA -- PEC then mismatches for every IC. A no-op RDCFGA
    //    first burns the stale-MOSI sample into a cmd whose reply we discard;
    //    the RDCV* commands then come back-to-back with MOSI continuously
    //    driven, so bit-sync holds. ~700 us at 1 MHz SCK.
    const auto rdcfga = ltc6811::pack_command(ltc6811::CmdRDCFGA);
    std::uint8_t warmup_reply[8 * config::LtcChainLength];
    if (!bus.read_register_group(rdcfga.data(),
                                 warmup_reply, sizeof(warmup_reply))) {
        ++g_ltc_spi_err_count;
        return { false, 0 };
    }

    // 4. Read the four cell-voltage register groups into one contiguous buffer
    //    that BmsService::update_from_ltc_response can walk. Group layout:
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
            return { false, 0 };  // partial reply -> don't poison BmsService state
        }
    }

    // 5. Digest. Per-IC PEC is checked inside; update returns false iff NO
    //    module had both its LTCs PEC-clean (the sleep/blackout signature).
    const std::uint32_t now_ms = osKernelGetTickCount();
    const bool any_fresh = BmsService::instance().update_from_ltc_response(
        reply, sizeof(reply), now_ms);

    // Count PEC-clean ICs so the retry loop can stop early on a fully-clean read.
    std::uint16_t mask  = BmsService::instance().ltc_online_mask();
    std::uint8_t  clean = 0;
    while (mask != 0u) { clean = static_cast<std::uint8_t>(clean + (mask & 1u)); mask >>= 1; }
    return { any_fresh, clean };
}

// Turn every discharge FET off and wait for it to actually happen, so the
// cell-voltage conversion is not corrupted by bleed current in the harness.
// See config::BalanceQuiesceMs for why ADCV's DCP=0 alone is not enough here.
//
// Returns true if it quiesced (and therefore owes a restore). A no-op when
// nothing is balancing, which is the common case outside Charge.
bool quiesce_balancing() noexcept {
    using namespace ams;
    if (!s_balance_active) return false;

    std::uint8_t zeros[config::LtcChainLength][6];
    const auto   off = ltc6811::pack_cfga_payload(0u);
    for (std::uint8_t i = 0; i < config::LtcChainLength; ++i) {
        for (std::size_t k = 0; k < 6; ++k) zeros[i][k] = off[k];
    }

    if (!ltc6820::Bus::default_instance().write_chain_command(
            ltc6811::CmdWRCFGA, zeros)) {
        // Could not prove discharge is off. Measure anyway -- stale cell data
        // starves the safety predicates, which is worse than a noisy read --
        // but do not claim a restore is owed, and count the error.
        ++g_ltc_spi_err_count;
        return false;
    }
    ++g_balance_quiesce_count;
    osDelay(config::BalanceQuiesceMs);
    return true;
}

// Put the balancing mask back after a measurement.
void restore_balancing() noexcept {
    using namespace ams;
    if (!ltc6820::Bus::default_instance().write_chain_command(
            ltc6811::CmdWRCFGA, s_last_cfga)) {
        // Balancing stays off until the next 1 Hz update rewrites it. Harmless:
        // the policy is stateless, so it simply resumes next cycle.
        ++g_ltc_spi_err_count;
        s_balance_active = false;
    }
}

// One ADOW conversion pass (open-wire). Mirrors attempt_voltage_poll's ADCV ->
// RDCV shape but issues ADOW with the given PUP, TWICE, so the pull-up/down
// current settles before the read (datasheet "Open Wire Check"; the cell-domain
// twin of the #482 mux first-select warm-up). Fills `reply` (4*GroupBytes) with
// RDCVA..D. Returns false on any bus error. UNVALIDATED ON HARDWARE (bench down)
// -- the ADOW encoding + timing need a real-chain check (see open_wire.hpp).
bool adow_pass(ams::ltc6820::Bus& bus, bool pull_up, std::uint8_t* reply) noexcept {
    using namespace ams;
    constexpr std::size_t SegBytes   = 8;
    constexpr std::size_t GroupBytes = config::LtcChainLength * SegBytes;

    // Two ADOW conversions: the first settles the PUP current, the second is the
    // one whose result the RDCV* reads pick up.
    for (std::uint8_t n = 0; n < 2; ++n) {
        const auto adow = ltc6811::pack_command(
            ltc6811::adow_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                              pull_up, /*discharge_permit=*/false,
                              ltc6811::CellSel::All));
        if (!bus.send_command(adow.data())) { ++g_ltc_spi_err_count; return false; }
        osDelay(config::AdcvSettleMs);
    }

    // RDCV warm-up (#214): no-op RDCFGA burns the stale-MOSI sample so the
    // back-to-back RDCV* reads keep bit-sync.
    const auto rdcfga = ltc6811::pack_command(ltc6811::CmdRDCFGA);
    std::uint8_t warmup[8 * config::LtcChainLength];
    if (!bus.read_register_group(rdcfga.data(), warmup, sizeof warmup)) {
        ++g_ltc_spi_err_count; return false;
    }

    static constexpr std::uint16_t RdcvCmds[4] = {
        ltc6811::CmdRDCVA, ltc6811::CmdRDCVB, ltc6811::CmdRDCVC, ltc6811::CmdRDCVD,
    };
    for (std::uint8_t g = 0; g < 4; ++g) {
        const auto cmd = ltc6811::pack_command(RdcvCmds[g]);
        if (!bus.read_register_group(cmd.data(), reply + g * GroupBytes, GroupBytes)) {
            ++g_ltc_spi_err_count; return false;
        }
    }
    return true;
}

// Open-wire scan: ADOW PUP=1 then PUP=0, hand both RDCV responses to
// BmsService::update_open_wire (which owns the per-IC 9/10 decode + the
// open_wire detector). Caller must have quiesced balancing already -- bleed
// current corrupts the pull-up/down delta. Gated by config::CellOpenWireCheck.
//
// RETRIED (up to config::OpenWireRetries) within THIS poll: update_open_wire
// needs both passes PEC-clean per IC to judge it, so a single transient PEC
// glitch on the affected IC would otherwise skip it and slip the CellOpenWire
// fault to the next poll (+BmsPollVoltMs, blowing the < 500 ms budget). If any
// IC was skipped we re-run the two-pass scan so the glitch is absorbed in-poll;
// a persistently-silent IC is caught by BmsModuleOffline instead.
void attempt_open_wire_poll() noexcept {
    using namespace ams;
    constexpr std::size_t GroupBytes = config::LtcChainLength * 8u;
    auto& bus = ltc6820::Bus::default_instance();
    std::uint8_t pu[4 * GroupBytes];
    std::uint8_t pd[4 * GroupBytes];
    for (std::uint8_t attempt = 0; attempt <= config::OpenWireRetries; ++attempt) {
        if (!adow_pass(bus, /*pull_up=*/true,  pu)) continue;   // bus error -> retry
        if (!adow_pass(bus, /*pull_up=*/false, pd)) continue;
        if (BmsService::instance().update_open_wire(pu, pd, sizeof pu)) {
            return;   // every IC judged this attempt -> done
        }
        // else: an IC PEC-glitched -> retry gives it another clean read
    }
}

void run_voltage_poll() {
    using namespace ams;

    // 0. If the last polls failed, the chain has probably hit T_SLEEP and is
    //    deaf to ordinary commands -- wake + reconfigure before trying again.
    if (s_consecutive_poll_failures >= RecoverAfterFailedPolls) {
        recover_chain();
    }

    // 0b. Stop bleeding before measuring. The bleed return path is the harness,
    //     not the board, so current flowing during a conversion shifts the
    //     shared tap node: the bled cell reads low and BOTH its neighbours read
    //     high, by 9-36 mV against a 50 mV selection threshold. Left in, the
    //     selection rule chases its own artifact.
    const bool quiesced = quiesce_balancing();

    // 1. Attempt the read up to (1 + VoltPollRetries) times. Each attempt
    //    digests whatever ICs are PEC-clean (refreshing those modules), so a
    //    brief EMI burst that corrupts one attempt often clears on an immediate
    //    re-read before the affected module drifts toward BmsStale, and retries
    //    give stragglers more chances. Stop early once the whole chain reads
    //    clean. A chain that fails EVERY attempt (dead / asleep / continuously
    //    swamped) still counts as one failed poll, feeding the T_SLEEP recovery
    //    and the staleness predicate exactly as before.
    bool any_fresh = false;
    for (std::uint8_t attempt = 0; attempt <= config::VoltPollRetries; ++attempt) {
        const VoltAttempt r = attempt_voltage_poll();
        any_fresh = any_fresh || r.any_module_fresh;
        if (r.clean_ltcs >= config::LtcChainLength) break;  // whole chain clean
    }

    // 1b. Open-wire scan (config::CellOpenWireCheck) while balancing is STILL
    //     quiesced -- ADOW's pull-up/down delta is corrupted by bleed current,
    //     same as the voltage measurement. Runs on the 200 ms poll cadence so an
    //     open faults in < 500 ms. Adds two ADOW passes to the poll -- validate
    //     the task keeps up on the HIL bench.
    if (config::CellOpenWireCheck) attempt_open_wire_poll();

    // 2. Resume bleeding. Done here rather than waiting for the 1 Hz balance
    //    update, which would otherwise leave discharge off for three polls out
    //    of four.
    if (quiesced) restore_balancing();

    if (any_fresh) {
        s_consecutive_poll_failures = 0;
    } else {
        ++s_consecutive_poll_failures;
    }
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
    // Operator balance master switch (#336): OFF / ON / AUTO on 0x103, with
    // the freshness dead-man resolved here (stale / never-seen -> Off).
    const auto       veh    = VehicleService::instance().snapshot();
    const auto       op_cmd = VehicleService::effective_balance_cmd(
        osKernelGetTickCount(), veh.last_balance_override_tick, veh.balance_cmd);
    // Per-module enable (0x104), freshness-resolved (stale/never -> all modules
    // enabled). Layered UNDER op_cmd inside compute_mask.
    const std::uint8_t mod_enable = VehicleService::effective_balance_modules_mask(
        osKernelGetTickCount(), veh.last_balance_modules_tick, veh.balance_modules_mask);
    // Balancing gates on config::BalanceTempsTrusted, NOT TempFaultsTrusted:
    // "trust these temps enough to balance on" is a different question from
    // "trust them enough to open the contactors on". Coupling the two meant the
    // WarioCharger 0x103 toggle was accepted and then produced an all-zero mask
    // forever. See the residual-risk note on BalanceTempsTrusted -- the
    // BalanceTempMax lockout inside compute_mask still applies.
    const auto       mask   = balance::compute_mask(
        state, fsm_curr, /*temps_trusted=*/config::BalanceTempsTrusted, op_cmd,
        mod_enable);

    std::uint8_t per_ic[config::LtcChainLength][6];
    bool         any_dcc = false;

    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        // LTC_1 (upper, chain index 2m) owns module cells 0..8 (9 cells).
        std::uint16_t dcc_upper = 0;
        for (std::uint8_t c = 0; c < config::CellsPerLtcUpper; ++c) {
            if (mask.cell[m][c]) {
                dcc_upper = static_cast<std::uint16_t>(dcc_upper | (1u << c));
            }
        }
        // LTC_2 (lower, chain index 2m+1) owns module cells 9..18 (10 cells).
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

    // Cache what is now on the chain so the next voltage poll can quiesce and
    // restore it. s_balance_active gates that work away entirely when nothing
    // is discharging, which is every cycle outside Charge.
    for (std::uint8_t i = 0; i < config::LtcChainLength; ++i) {
        for (std::size_t k = 0; k < 6; ++k) s_last_cfga[i][k] = per_ic[i][k];
    }
    s_balance_active = any_dcc;

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

    // Warm-up mux select (the mux-domain twin of the #214 RDCV warm-up). The
    // FIRST WRCOMM/STCOMM after the voltage poll's cell-read burst can be
    // dropped -- slaves re-sync on CS edges after the multi-ms idle -- so the
    // sweep's first channel (Adg731ChannelMap[0] = S1) never latches and every
    // mux stays on its previous channel. Result: temp1 / slot 0 reads rail
    // (~VREF2) on ALL modules at once, a false open on every sweep. Confirmed on
    // the bench via a 32-channel raw dump: 9/10 muxes' S1 came alive the instant
    // this warm-up landed (the 10th was a genuine hardware open).
    //
    // The throwaway select deliberately targets an UNPOPULATED address (S32, the
    // last channel -- NC on every mux per pcbs/BMS_LITE) and its reply is never
    // read: whichever select the chain drops is spent on a pin that carries no
    // NTC, so a dropped warm-up can never cost a real temperature, and if it DOES
    // land the mux just sits on a dead pin until the real sweep's first select
    // moves it. Either way the sweep's first populated channel latches.
    constexpr std::uint8_t WarmUpAddr = 31u;   // S32, unpopulated on all muxes
    {
        const auto warm = ltc6811::pack_adg731_select(WarmUpAddr);
        for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
            for (std::size_t k = 0; k < 6; ++k) per_ic_payload[ic][k] = warm[k];
        }
        (void)bus.write_chain_command(ltc6811::CmdWRCOMM, per_ic_payload);
        (void)bus.stcomm();
        osDelay(1);
    }

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

        // BmsPollTask woke to service an isoSPI poll -> housekeeping liveness (#411).
        ams::fw_health::poke(ams::fw_health::Housekeeping);

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
