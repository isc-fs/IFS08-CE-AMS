// SPDX-License-Identifier: proprietary
//
// Drives the LTC6811-1 daisy-chain to acquire cell voltages and, via the
// per-LTC ADG731 32:1 mux, cell temperatures. Wire format: ltc6811.hpp /
// ltc6820.hpp. Task table: docs/ARCHITECTURE.md §2.
//
// Cadence:
//   * PollVDue every BmsPollVoltMs (200 ms) -> ADCV + RDCVA/B/C/D
//   * PollTDue every BmsPollTempMs (250 ms) -> 20-channel mux sweep
//
// Two osTimers raise event-flag bits and this task wakes on osEventFlagsWait.
// It is the only user of the SPI bus, so the HAL_SPI calls need no mutex.

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

// FSM state mirror, written by MainTask on every transition. No lock needed: a
// volatile 8-bit read is coherent on Cortex-M7, and a coherent snapshot at the
// call point is all this needs.
extern "C" volatile std::uint8_t g_state_telemetry;

// Latched fault reason, written by MainTask on the transition into ERROR. Same
// single-writer / 8-bit-read contract as g_state_telemetry, and acu_can_task
// consumes it the same way. BalanceController needs it because a latched
// CELL-DATA fault means the cell voltages it ranks are untrustworthy -- see
// balance::is_cell_data_fault.
extern "C" volatile std::uint8_t g_fault_reason_telemetry;

namespace {

osTimerId_t s_volt_timer = nullptr;
osTimerId_t s_temp_timer = nullptr;

// Bus-level SPI failures (HAL_OK != 0). Volatile so it can be read via the
// symbol. The per-IC PEC-error counter is separate and lives in bms_service.cpp.
volatile std::uint32_t g_ltc_spi_err_count = 0;

// Voltage-poll round-trip time: last cycle, and worst case since boot. Lets the
// HIL operator confirm the poll fits inside its 200 ms budget without a scope.
// extern "C" so the pit-diag stream can surface them via AcuCanTask.
}  // close anonymous namespace temporarily for the extern "C" decls

extern "C" volatile std::uint32_t g_bms_volt_poll_ms  = 0;
extern "C" volatile std::uint32_t g_bms_volt_poll_max = 0;

// DCC mask from the last balance cycle, exposed for pit-diag. Bit c of
// g_balance_dcc_bits[m] == 1 iff cell c of module m was selected for discharge
// that cycle. 19 cells fit the low 19 bits; bits 19..31 are always 0.
extern "C" volatile std::uint32_t g_balance_dcc_bits[5] = {0, 0, 0, 0, 0};
extern "C" volatile std::uint32_t g_balance_cycles_total_pub  = 0;
extern "C" volatile std::uint32_t g_balance_cycles_active_pub = 0;

// Counts the times run_voltage_poll re-woke and reconfigured the chain after
// consecutive failed polls. Zero on a healthy bus; climbing means the chain is
// repeatedly dropping out (the inverter-EMI T_SLEEP case this recovery exists
// for). extern "C" so the pit-diag stream can surface it.
extern "C" volatile std::uint32_t g_ltc_chain_recover_count = 0;

// Counts the times the voltage poll cleared the DCC bits before measuring. It
// climbs at the poll rate while balancing is actually discharging and stays flat
// otherwise, so the bench can confirm from CAN alone that the quiesce runs.
extern "C" volatile std::uint32_t g_balance_quiesce_count = 0;

// Counts the times the quiesce could NOT be proven (both WRCFGA attempts failed)
// and the poll measured with balancing still live. Those cell voltages carry the
// 9-36 mV bleed displacement, so the balance selector skips that cycle while the
// safety predicates still consume them -- the right split. Climbing alongside
// g_ltc_spi_err_count means isoSPI trouble is corrupting the balancing input;
// climbing on its own means a chain that fails only on writes.
extern "C" volatile std::uint32_t g_balance_quiesce_fail_count = 0;

// BENCH DIAGNOSTIC (config::AdowRawDiag): raw ADOW pull-up / pull-down per-cell
// readings (flat 95 = 5 modules x 19 cells), dumped over pit-diag so the ADOW
// encoding and timing can be debugged on a real chain. Single-writer
// (BmsPollTask); AcuCanTask reads them for the emit. Non-volatile on purpose --
// a torn 16-bit read is harmless in a diagnostic. Stays 0 unless AdowRawDiag.
extern "C" std::uint16_t g_adow_diag_pu[ams::config::BmsModuleCount *
                                        ams::config::CellsPerModule] = {};
extern "C" std::uint16_t g_adow_diag_pd[ams::config::BmsModuleCount *
                                        ams::config::CellsPerModule] = {};

// BENCH DIAGNOSTIC (config::AdcModeCrossCheck): the same 95 cells re-measured in
// config::AdcXCheckAdcMode. Diffed against the live 0x680 grid, it separates a
// settling-limited tap (mode-dependent) from a genuinely low cell (mode-
// independent). Same ownership and tearing argument as the ADOW grids above.
// Zero until the first sweep completes; 0xFFFF on a cell whose IC PEC-failed.
extern "C" std::uint16_t g_adc_xcheck_mv[ams::config::BmsModuleCount *
                                         ams::config::CellsPerModule] = {};

namespace {

// Balance-update counters, surfaced for HIL: total update cycles, and cycles
// where at least one DCC bit was set.
volatile std::uint32_t g_balance_cycles_total  = 0;
volatile std::uint32_t g_balance_cycles_active = 0;
std::uint32_t          s_volt_poll_count       = 0;

// Polls since the last ADC-mode cross-check sweep. Separate from
// s_volt_poll_count, which is the balance-update counter and wraps every
// BalanceUpdatePolls -- reusing it would fire the sweep on that cadence instead.
std::uint32_t          s_xcheck_poll_count     = 0;

// Last CFGA payload actually written to the chain, and whether it asserted any
// DCC bit. The mask is recomputed only every BalanceUpdatePolls (800 ms) but
// PERSISTS on the chain between writes, so a measurement that quiesces it must
// put it back: clearing it every 200 ms poll and not restoring would destroy
// three quarters of the balancing duty.
std::uint8_t s_last_cfga[ams::config::LtcChainLength][6] = {};
bool         s_balance_active = false;

// Last mask compute_mask produced, fed back next cycle so the selector can apply
// hysteresis (BalanceStopDeltaMv). Without it the policy re-ranks from scratch
// every BalanceUpdatePolls and a cell hovering near the threshold toggles
// instead of accumulating bleed time. Held here rather than inside compute_mask
// so that function stays pure and host-testable.
ams::balance::Mask s_prev_balance_mask = {};

// Set by quiesce_balancing() when it could not prove discharge was off, i.e. the
// cell voltages this poll produced were taken under bleed. Consumed and cleared
// by maybe_run_balance_update(), which then skips the update rather than re-rank
// the pack on displaced readings. Single-writer, same thread, so a plain bool
// suffices.
bool         g_balance_quiesce_fail = false;

// ---------------------------------------------------------------------------
// Chain-sleep recovery.
//
// The LTC6811 drops into T_SLEEP (~2 s) after its last VALID command, and a
// sleeping IC ignores every normal command -- only the CS-low wake pulse train
// (Bus::wakeup) brings it back. One wakeup at boot is not enough: any
// interruption longer than T_SLEEP (inverter switching noise corrupting commands
// through a torque event, say) would put the chain to sleep permanently with no
// way back.
//
// So count consecutive failed polls and, once the chain looks gone, re-wake and
// reconfigure before every subsequent attempt. Retrying every poll rather than
// once is deliberate: while the disturbance lasts the wake will not take, and
// the chain should come back on the first poll after it clears. Waking an
// already-awake chain is harmless (the CS pulses carry no command), so a false
// positive costs ~500 us, not correctness.
// ---------------------------------------------------------------------------

// Consecutive failed voltage polls before recovery kicks in. At 2, recovery runs
// on the 3rd poll, >= 400 ms after the last good one at BmsPollVoltMs (200 ms).
// That is ALREADY past BmsStaleMs (350 ms), so by the time recovery fires the
// modules have gone stale and BmsModuleOffline has had cause to fire: recovery
// restores the data, it does not prevent the fault. Not 1: an isolated PEC
// glitch must not clear DCC bits mid-balance.
constexpr std::uint32_t RecoverAfterFailedPolls = 2;

std::uint32_t s_consecutive_poll_failures = 0;

// Re-establish the chain after a suspected T_SLEEP: wake it, then restore CFGR.
// The reconfigure is NOT optional -- sleeping resets CFGR to defaults, which
// re-enables the GPIO pull-downs (shorting the ADG731 mux / NTC divider the ADAX
// temperature path reads) and drops REFON. DCC is written all-zero, i.e.
// discharge off; maybe_run_balance_update restores the real mask next cycle.
void recover_chain() noexcept {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();
    bus.wakeup();

    // The chain slept, so CFGR is back at defaults and nothing is discharging.
    // Drop the balance cache: restoring a pre-sleep mask would re-assert bits the
    // controller has not re-derived since. The next 800 ms update recomputes it.
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
// Voltage poll: one ADCV broadcast (all 10 LTCs convert their cells) followed by
// 4 register-group reads (RDCVA/B/C/D) concatenated into the 320-byte buffer
// BmsService digests in one call.
// ---------------------------------------------------------------------------
// Result of one voltage-poll attempt.
struct VoltAttempt {
    bool         any_module_fresh;  // >= 1 module had BOTH its LTCs PEC-clean
    std::uint8_t clean_ltcs;        // count of PEC-clean ICs this attempt
};

// One voltage-poll attempt: ADCV -> settle -> warm-up -> RDCVA/B/C/D -> digest.
// update_from_ltc_response refreshes last_rx_tick for whichever modules came back
// PEC-clean, so repeating this (the retry loop below) gives each module more
// chances to report. Returns {false, 0} on any bus-level failure.
VoltAttempt attempt_voltage_poll() {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();

    // 1. ADCV broadcast, all cells. Discharge-permit = false during normal data
    //    acquisition; it is flipped only for balancing windows.
    const auto adcv = ltc6811::pack_command(
        ltc6811::adcv_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                          /*discharge_permit=*/false,
                          ltc6811::CellSel::All));
    if (!bus.send_command(adcv.data())) {
        ++g_ltc_spi_err_count;
        return { false, 0 };
    }

    // 2. ADC settling. Norm-7kHz converts all 12 channels in ~2.3 ms, rounded up
    //    to 3 ms (config::AdcvSettleMs).
    osDelay(config::AdcvSettleMs);

    // 3. Warm-up command before RDCVA. Over the multi-ms idle between ADCV +
    //    settle and the first RDCV, MOSI drifts to its idle-high level long
    //    enough that slaves which re-sync on CS edges (the Pi Pico LTC6820
    //    emulator on the HIL bench, for one) sample a stray HIGH as bit 7 of
    //    byte 0 of RDCVA, and PEC then mismatches on every IC. A no-op RDCFGA
    //    absorbs that stale-MOSI sample into a command whose reply is discarded;
    //    the RDCV* reads then run back-to-back with MOSI continuously driven, so
    //    bit-sync holds. Costs ~1.3 ms (84 bytes at 515.625 kHz SCK).
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

    // 5. Digest. Per-IC PEC is checked inside; the update returns false iff NO
    //    module had both its LTCs PEC-clean -- the sleep/blackout signature.
    const std::uint32_t now_ms = osKernelGetTickCount();
    const bool any_fresh = BmsService::instance().update_from_ltc_response(
        reply, sizeof(reply), now_ms);

    // Count PEC-clean ICs so the retry loop can stop early on a fully-clean read.
    std::uint16_t mask  = BmsService::instance().ltc_online_mask();
    std::uint8_t  clean = 0;
    while (mask != 0u) { clean = static_cast<std::uint8_t>(clean + (mask & 1u)); mask >>= 1; }
    return { any_fresh, clean };
}

// Turn every discharge FET off and wait for it to take effect, so bleed current
// in the harness cannot corrupt the cell-voltage conversion. See
// config::BalanceQuiesceMs for why ADCV's DCP=0 alone is not enough.
//
// Returns true if it quiesced, and therefore owes a restore. A no-op when
// nothing is balancing, which is every cycle outside Charge.
bool quiesce_balancing() noexcept {
    using namespace ams;
    if (!s_balance_active) return false;

    std::uint8_t zeros[config::LtcChainLength][6];
    const auto   off = ltc6811::pack_cfga_payload(0u);
    for (std::uint8_t i = 0; i < config::LtcChainLength; ++i) {
        for (std::size_t k = 0; k < 6; ++k) zeros[i][k] = off[k];
    }

    // RETRY once before giving up. WRCFGA is idempotent -- it writes an absolute
    // DCC mask, not a delta -- and costs ~1.3 ms at 515.625 kHz, cheap next to
    // the alternative of measuring the whole pack under full bleed.
    //
    // DCP=0 on the ADCV does NOT rescue that case. Per LTC6811 datasheet Table 53
    // it suppresses discharge only on the cell being measured and its immediate
    // neighbours: during the CELL1/7 window S1/S2 are off but S3/S4/S5 stay ON.
    // Roughly half the selected cells therefore keep pulling ~165 mA through the
    // shared tap harness for the whole conversion -- exactly the 9-36 mV
    // displacement (bled cell low, both neighbours high) that BalanceQuiesceMs
    // exists to eliminate. The quiesce is the ONLY full stop; treat it that way.
    bool quiesced = false;
    for (std::uint8_t attempt = 0; attempt < 2u && !quiesced; ++attempt) {
        quiesced = ltc6820::Bus::default_instance().write_chain_command(
            ltc6811::CmdWRCFGA, zeros);
        if (!quiesced) ++g_ltc_spi_err_count;
    }

    if (!quiesced) {
        // Still cannot prove discharge is off. Measure anyway: starving the
        // safety predicates of cell data is worse than a noisy read. But flag the
        // poll so the BALANCE SELECTOR ignores it -- the balancing policy must
        // not act on voltages it knows were taken under bleed, or it chases its
        // own artifact at the same order of magnitude as BalanceDeltaMv (50 mV).
        g_balance_quiesce_fail = true;
        ++g_balance_quiesce_fail_count;
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
        // Balancing stays off until the next 800 ms update rewrites it. Harmless:
        // the policy re-derives the mask, so it simply resumes next cycle.
        ++g_ltc_spi_err_count;
        s_balance_active = false;
    }
}

// One ADOW conversion pass (open-wire). Same ADCV -> RDCV shape as
// attempt_voltage_poll, but issues ADOW with the given PUP TWICE so the
// pull-up/down current settles before the read (datasheet "Open Wire Check" --
// the cell-domain twin of the mux first-select warm-up). Fills `reply`
// (4*GroupBytes) with RDCVA..D. Returns false on any bus error.
// UNVALIDATED ON HARDWARE: the ADOW encoding and timing have never been checked
// on a real chain (see open_wire.hpp).
bool adow_pass(ams::ltc6820::Bus& bus, bool pull_up, std::uint8_t* reply) noexcept {
    using namespace ams;
    constexpr std::size_t SegBytes   = 8;
    constexpr std::size_t GroupBytes = config::LtcChainLength * SegBytes;

    // Two ADOW conversions: the first settles the PUP current, the second
    // produces the result the RDCV* reads pick up.
    for (std::uint8_t n = 0; n < 2; ++n) {
        const auto adow = ltc6811::pack_command(
            ltc6811::adow_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                              pull_up, /*discharge_permit=*/false,
                              ltc6811::CellSel::All));
        if (!bus.send_command(adow.data())) { ++g_ltc_spi_err_count; return false; }
        osDelay(config::AdcvSettleMs);
    }

    // RDCV warm-up: no-op RDCFGA burns the stale-MOSI sample so the
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

// Open-wire scan: ADOW PUP=1 then PUP=0, then hand both RDCV responses to
// BmsService::update_open_wire, which owns the per-IC 9/10 decode and the
// open_wire detector. The caller must have quiesced balancing already -- bleed
// current corrupts the pull-up/down delta. Gated by config::CellOpenWireCheck.
//
// RETRIED up to config::OpenWireRetries times within THIS poll: update_open_wire
// needs both passes PEC-clean per IC to judge it, so one transient glitch would
// otherwise skip that IC and slip the CellOpenWire fault to the next poll
// (+BmsPollVoltMs, blowing the < 500 ms budget). Re-running the two-pass scan
// absorbs the glitch in-poll; a persistently silent IC is caught instead by
// BmsModuleOffline.
void attempt_open_wire_poll() noexcept {
    using namespace ams;
    constexpr std::size_t GroupBytes = config::LtcChainLength * 8u;
    auto& bus = ltc6820::Bus::default_instance();
    std::uint8_t pu[4 * GroupBytes];
    std::uint8_t pd[4 * GroupBytes];
    for (std::uint8_t attempt = 0; attempt <= config::OpenWireRetries; ++attempt) {
        if (!adow_pass(bus, /*pull_up=*/true,  pu)) continue;   // bus error -> retry
        if (!adow_pass(bus, /*pull_up=*/false, pd)) continue;
        // Attempt 0 overwrites, clearing the prior poll; retries OR-accumulate so
        // a later glitch cannot erase an open an earlier attempt confirmed.
        if (BmsService::instance().update_open_wire(pu, pd, sizeof pu,
                                                    /*accumulate=*/attempt != 0u)) {
            return;   // every IC judged this attempt -> done
        }
        // else: an IC PEC-glitched -> retry gives it another clean read
    }
}

// BENCH DIAGNOSTIC (config::AdowRawDiag): run one two-pass ADOW scan and stash
// the raw per-cell PU/PD readings in the diag globals for the pit-diag dump.
// Independent of CellOpenWireCheck so ADOW can be debugged while live detection
// is off. Caller quiesces balancing (same reason as attempt_open_wire_poll).
void capture_adow_diag() noexcept {
    using namespace ams;
    constexpr std::size_t GroupBytes = config::LtcChainLength * 8u;
    auto& bus = ltc6820::Bus::default_instance();
    std::uint8_t pu[4 * GroupBytes];
    std::uint8_t pd[4 * GroupBytes];
    if (adow_pass(bus, /*pull_up=*/true, pu) && adow_pass(bus, /*pull_up=*/false, pd)) {
        BmsService::capture_adow_raw(pu, pd, sizeof pu, g_adow_diag_pu, g_adow_diag_pd);
    }
}

// BENCH DIAGNOSTIC (config::AdcModeCrossCheck): re-measure every cell in a second
// ADC mode and stash it for the pit-diag dump.
//
// It reads the SAME taps and register groups as the live poll; only the
// conversion time differs, and that is the whole point. A tap with excess series
// resistance has not settled when a fast conversion samples it, so it reads low
// there and true in a slow one, while a genuinely discharged cell reads the same
// in both. See config::AdcModeCrossCheck for why sum-of-cells cannot answer this.
//
// Writes only the diagnostic grid -- BmsService state is untouched, so the
// snapshot feeding the safety predicates never sees a cross-check reading. The
// caller must have quiesced balancing: bleed current displaces this measurement
// exactly as it displaces the live one, and a displaced comparison would invent
// a tap fault.
void capture_adc_mode_crosscheck() noexcept {
    using namespace ams;
    constexpr std::size_t GroupBytes = config::LtcChainLength * 8u;
    auto& bus = ltc6820::Bus::default_instance();

    const auto adcv = ltc6811::pack_command(
        ltc6811::adcv_cmd(static_cast<ltc6811::AdcMode>(config::AdcXCheckAdcMode),
                          /*discharge_permit=*/false,
                          ltc6811::CellSel::All));
    if (!bus.send_command(adcv.data())) {
        ++g_ltc_spi_err_count;
        return;
    }
    osDelay(config::AdcXCheckSettleMs);

    // Same stale-MOSI warm-up the live poll needs: the settling delay leaves MOSI
    // idle long enough for a CS-edge-resyncing slave to sample a stray HIGH as
    // the first bit of RDCVA. See attempt_voltage_poll step 3.
    const auto rdcfga = ltc6811::pack_command(ltc6811::CmdRDCFGA);
    std::uint8_t warmup_reply[8 * config::LtcChainLength];
    if (!bus.read_register_group(rdcfga.data(), warmup_reply, sizeof(warmup_reply))) {
        ++g_ltc_spi_err_count;
        return;
    }

    static constexpr std::uint16_t RdcvCmds[4] = {
        ltc6811::CmdRDCVA, ltc6811::CmdRDCVB,
        ltc6811::CmdRDCVC, ltc6811::CmdRDCVD,
    };
    std::uint8_t reply[4 * GroupBytes] = {};
    for (std::uint8_t g = 0; g < 4; ++g) {
        const auto cmd = ltc6811::pack_command(RdcvCmds[g]);
        if (!bus.read_register_group(cmd.data(), reply + g * GroupBytes, GroupBytes)) {
            ++g_ltc_spi_err_count;
            return;   // partial reply -> leave the previous sweep in place
        }
    }
    BmsService::capture_cell_raw(reply, sizeof reply, g_adc_xcheck_mv);
}

void run_voltage_poll() {
    using namespace ams;

    // 0. If the last polls failed, the chain has probably hit T_SLEEP and is deaf
    //    to ordinary commands -- wake and reconfigure before trying again.
    if (s_consecutive_poll_failures >= RecoverAfterFailedPolls) {
        recover_chain();
    }

    // 0b. Stop bleeding before measuring. The bleed return path is the harness,
    //     not the board, so current flowing during a conversion shifts the shared
    //     tap node: the bled cell reads low and BOTH its neighbours read high, by
    //     9-36 mV against a 50 mV selection threshold. Left in, the selection
    //     rule chases its own artifact.
    const bool quiesced = quiesce_balancing();

    // 1. Read up to (1 + VoltPollRetries) times. Each attempt digests whatever
    //    ICs are PEC-clean, refreshing those modules, so a brief EMI burst that
    //    corrupts one attempt often clears on an immediate re-read before the
    //    affected module drifts toward BmsStale, and retries give stragglers more
    //    chances. Stop early once the whole chain reads clean. A chain that fails
    //    EVERY attempt (dead / asleep / continuously swamped) counts as one
    //    failed poll, feeding the T_SLEEP recovery and the staleness predicate.
    bool any_fresh = false;
    for (std::uint8_t attempt = 0; attempt <= config::VoltPollRetries; ++attempt) {
        const VoltAttempt r = attempt_voltage_poll();
        any_fresh = any_fresh || r.any_module_fresh;
        if (r.clean_ltcs >= config::LtcChainLength) break;  // whole chain clean
    }

    // 1b. Open-wire scan (config::CellOpenWireCheck), while balancing is STILL
    //     quiesced: bleed current corrupts ADOW's pull-up/down delta exactly as
    //     it corrupts the voltage measurement. Runs on the 200 ms poll cadence so
    //     an open faults in < 500 ms. It adds two ADOW passes to the poll --
    //     UNVALIDATED: confirm the task keeps up on the HIL bench.
    if (config::CellOpenWireCheck) attempt_open_wire_poll();
    // Bench-only raw ADOW dump (dead-code-eliminated when AdowRawDiag is false).
    if (config::AdowRawDiag) capture_adow_diag();

    // 1c. Bench-only second-mode sweep, every AdcXCheckPolls (25) polls = ~5 s,
    //     because a filtered conversion costs ~AdcXCheckSettleMs. Runs AFTER the
    //     live read so module freshness is never delayed by it; only the NEXT
    //     poll starts late, which the AdcXCheckPollBodyBudgetMs static_assert
    //     bounds against BmsStaleMs. Still inside the quiesce window, for the
    //     same reason as the ADOW scan.
    if (config::AdcModeCrossCheck && s_xcheck_poll_count++ == 0u) {
        capture_adc_mode_crosscheck();
    }
    if (s_xcheck_poll_count >= config::AdcXCheckPolls) s_xcheck_poll_count = 0;

    // 2. Resume bleeding. Done here rather than at the next 800 ms balance
    //    update, which would leave discharge off for three polls out of four.
    if (quiesced) restore_balancing();

    if (any_fresh) {
        s_consecutive_poll_failures = 0;
    } else {
        ++s_consecutive_poll_failures;
    }
}

// ---------------------------------------------------------------------------
// Cell balancing. Once every BalanceUpdatePolls voltage cycles (800 ms):
// snapshot BmsState, ask BalanceController what to discharge, pack the DCC bits
// into 10 WRCFGA payloads, broadcast them. Outside fsm::State::Charge the
// controller returns an all-zero mask, so the next WRCFGA clears whatever was
// set before.
// ---------------------------------------------------------------------------
void maybe_run_balance_update() {
    using namespace ams;

    if (++s_volt_poll_count < config::BalanceUpdatePolls) return;
    s_volt_poll_count = 0;

    // The quiesce could not be proven this cycle, so the snapshot to be ranked
    // was measured with cells still bleeding: the bled cell reads ~9-36 mV LOW
    // and both its neighbours read high, against a 50 mV BalanceDeltaMv.
    // Re-picking on that inverts the very comparison the selector exists to make.
    // Hold the previous mask -- it is already on the chain, and the next clean
    // cycle re-derives it -- and wait one 800 ms window, immaterial on a
    // multi-hour C/101 balancer.
    if (g_balance_quiesce_fail) {
        g_balance_quiesce_fail = false;
        return;
    }

    const auto       state    = BmsService::instance().snapshot();
    const fsm::State fsm_curr =
        static_cast<fsm::State>(g_state_telemetry);
    // Operator balance master switch: OFF / ON / AUTO on 0x103. The freshness
    // dead-man resolves here: stale or never seen -> Off.
    const auto       veh    = VehicleService::instance().snapshot();
    const auto       op_cmd = VehicleService::effective_balance_cmd(
        osKernelGetTickCount(), veh.last_balance_override_tick, veh.balance_cmd);
    // Per-module enable (0x104), freshness-resolved: stale or never seen -> all
    // modules enabled. Layered UNDER op_cmd inside compute_mask.
    const std::uint8_t mod_enable = VehicleService::effective_balance_modules_mask(
        osKernelGetTickCount(), veh.last_balance_modules_tick, veh.balance_modules_mask);
    // Balancing gates on config::BalanceTempsTrusted, NOT TempFaultsTrusted:
    // "trust these temps enough to balance on" is a different question from
    // "trust them enough to open the contactors on", and coupling the two makes
    // the WarioCharger 0x103 toggle produce an all-zero mask forever. See the
    // residual-risk note on BalanceTempsTrusted; the BalanceTempMax lockout
    // inside compute_mask still applies either way.
    // A latched CELL-DATA fault (open wire / OV / UV) stops balancing for BOTH
    // Auto and the operator On override: once one has fired, the voltages
    // compute_mask ranks are not trustworthy.
    const auto       fault    = static_cast<safety::FaultReason>(g_fault_reason_telemetry);
    const auto       mask   = balance::compute_mask(
        state, fsm_curr, /*temps_trusted=*/config::BalanceTempsTrusted, op_cmd,
        fault, mod_enable, &s_prev_balance_mask);
    s_prev_balance_mask = mask;

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

        // Publish the per-module DCC selection so pit-diag can show which cell is
        // balancing right now. dcc_upper covers module cells 0..8 and dcc_lower
        // covers 9..18, so shift the lower half by CellsPerLtcUpper to make one
        // uint32 mirror the module's cell-index layout.
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
    // restore it. s_balance_active gates that work away entirely when nothing is
    // discharging, which is every cycle outside Charge.
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
// 20 channels x ~3 ms ~= 60 ms of wire time, ~100 ms once HAL and tick jitter
// are counted -- inside the 250 ms BmsPollTempMs cadence either way.
// ---------------------------------------------------------------------------
// Per-channel sweep-failure tracking. `last_mask` covers ONLY the most recent
// sweep (cleared at its start). `sticky_mask` is OR-accumulated across every
// sweep since boot, which catches intermittent NTC / mux failures that do not
// show up on each cycle; clear it by writing 0.
//
// extern "C" so the pit-diag stream can surface last_mask via AcuCanTask. These
// MUST live outside the anonymous namespace: an extern "C" object declared
// inside an unnamed namespace still gets internal linkage, so the symbol would
// not be exported and acu_can_task.cpp's reference would fail to link.
}  // close anonymous namespace for the extern "C" decls

extern "C" volatile std::uint32_t g_temp_sweep_last_mask   = 0;
extern "C" volatile std::uint32_t g_temp_sweep_sticky_mask = 0;

namespace {

// Sweep resume state. A sweep can PAUSE after any channel to let a due voltage
// poll run, which bounds the voltage poll's jitter to ~one channel (~3 ms)
// instead of a whole ~100 ms sweep -- the precondition that makes BmsStaleMs
// (350 ms) safe against nuisance trips. s_temp_ch is the next channel to sweep
// (0 = start a fresh sweep); s_temp_sweep_fail accumulates the failure mask
// across the pauses so it is not lost mid-sweep. Single-writer (BmsPollTask), so
// no sync is needed, and isoSPI stays single-owner: the sweep interleaves with
// the voltage poll on the SAME task rather than blocking it.
std::uint8_t  s_temp_ch         = 0;
std::uint32_t s_temp_sweep_fail = 0;

void run_temperature_poll() {
    using namespace ams;

    auto& bus = ltc6820::Bus::default_instance();

    // The same mux-select payload is broadcast to every LTC each step. The ADG731
    // ignores the bits it cannot address (only ch < 32 is used).
    std::uint8_t per_ic_payload[config::LtcChainLength][6];

    // Warm-up mux select, the mux-domain twin of the RDCV warm-up. Runs ONLY at
    // the start of a sweep (s_temp_ch == 0), never on a resume after a yield.
    // Without it the FIRST WRCOMM/STCOMM after the voltage poll's cell-read burst
    // can be dropped -- slaves re-sync on CS edges after the multi-ms idle -- so
    // the sweep's first channel (Adg731ChannelMap[0] = S1) never latches and every
    // mux stays on its previous channel. temp1 / slot 0 then reads rail (~VREF2)
    // on ALL modules at once: a false open on every sweep. Bench-confirmed with a
    // 32-channel raw dump, where 9 of 10 muxes' S1 came alive the moment this
    // warm-up landed (the 10th was a genuine hardware open). The throwaway select
    // targets an UNPOPULATED address (S32, NC on every mux) and its reply is
    // discarded, so a dropped warm-up can never cost a real temperature.
    if (s_temp_ch == 0) {
        s_temp_sweep_fail = 0;
        constexpr std::uint8_t WarmUpAddr = 31u;   // S32, unpopulated on all muxes
        const auto warm = ltc6811::pack_adg731_select(WarmUpAddr);
        for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
            for (std::size_t k = 0; k < 6; ++k) per_ic_payload[ic][k] = warm[k];
        }
        (void)bus.write_chain_command(ltc6811::CmdWRCOMM, per_ic_payload);
        (void)bus.stcomm();
        osDelay(1);
    }

    // Resumable sweep: continues from s_temp_ch so a yield mid-sweep doesn't
    // restart it. Each channel: WRCOMM/STCOMM select -> settle -> ADAX -> RDAUXA.
    for (; s_temp_ch < config::TempsPerLtc; ++s_temp_ch) {
        const std::uint8_t ch_idx = s_temp_ch;
        const std::uint8_t mux_ch = config::Adg731ChannelMap[ch_idx];
        const auto sel = ltc6811::pack_adg731_select(mux_ch);

        for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
            for (std::size_t k = 0; k < 6; ++k) per_ic_payload[ic][k] = sel[k];
        }

        // 1. WRCOMM: load the select word into every IC's COMM reg.
        if (!bus.write_chain_command(ltc6811::CmdWRCOMM, per_ic_payload)) {
            ++g_ltc_spi_err_count; s_temp_sweep_fail |= (1u << ch_idx); continue;
        }
        // 2. STCOMM: shift COMM register out -> mux receives.
        if (!bus.stcomm()) {
            ++g_ltc_spi_err_count; s_temp_sweep_fail |= (1u << ch_idx); continue;
        }
        // 3. Mux + NTC voltage-divider settling (~80 ns transition, divider
        //    << 1 ms; the 1 ms tick is plenty).
        osDelay(1);

        // 4. ADAX(Gpio1) broadcast -> AUX-ADC conversion on every IC.
        const auto adax_cmd = ltc6811::pack_command(
            ltc6811::adax_cmd(static_cast<ltc6811::AdcMode>(config::AdcMode),
                              ltc6811::AuxSel::Gpio1));
        if (!bus.send_command(adax_cmd.data())) {
            ++g_ltc_spi_err_count; s_temp_sweep_fail |= (1u << ch_idx); continue;
        }
        osDelay(config::AdaxSettleMs);

        // 5. RDAUXA: read AUX1..AUX3 + PEC per IC.
        constexpr std::size_t Reply = config::LtcChainLength * 8u;
        std::uint8_t reply[Reply] = {};
        const auto rdauxa = ltc6811::pack_command(ltc6811::CmdRDAUXA);
        if (!bus.read_register_group(rdauxa.data(), reply, sizeof(reply))) {
            ++g_ltc_spi_err_count; s_temp_sweep_fail |= (1u << ch_idx); continue;
        }

        (void)BmsService::instance().update_temperature(ch_idx, reply, sizeof(reply));

        // YIELD to a due voltage poll, then resume this sweep immediately, so no
        // channel's cadence slips. This is what bounds the voltage poll's jitter
        // to ~one channel (~3 ms) instead of a whole sweep, the precondition that
        // makes BmsStaleMs (350 ms) safe from nuisance trips. Re-arm PollTDue so
        // the task finishes this sweep right after the V-poll.
        if (s_temp_ch + 1u < config::TempsPerLtc &&
            (osEventFlagsGet(bms_eventsHandle) & ams::events::bms::PollVDue) != 0u) {
            ++s_temp_ch;
            osEventFlagsSet(bms_eventsHandle, ams::events::bms::PollTDue);
            return;
        }
    }

    g_temp_sweep_last_mask    = s_temp_sweep_fail;
    g_temp_sweep_sticky_mask |= s_temp_sweep_fail;
    s_temp_ch = 0;   // sweep complete -> fresh warm-up next time
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

        // BmsPollTask woke to service an isoSPI poll -> housekeeping liveness.
        ams::fw_health::poke(ams::fw_health::Housekeeping);

        if ((evt & osFlagsError) != 0u) {
            // Event group went away; back off and keep waiting. Defensive only:
            // the group is statically allocated for the life of the app.
            osDelay(50);
            continue;
        }

        if (evt & ams::events::bms::PollVDue) {
            const std::uint32_t t0 = osKernelGetTickCount();
            run_voltage_poll();
            // Balancing piggybacks on the V-poll cadence. The controller gates on
            // FSM state internally, so this is a no-op outside Charge.
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
