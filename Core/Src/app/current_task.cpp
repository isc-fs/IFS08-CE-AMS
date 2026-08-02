// SPDX-License-Identifier: proprietary
//
// Periodic ADC poll of TWO current sensors on ADC3:
//   - PF7/PF8 / ADC3_INP3+INN3 -> pack current (Bourns SSA-2-250A read
//     in ADC DIFFERENTIAL mode; OUT_P=PF7, OUT_N=PF8)
//   - PC1 / ADC3_INP11 -> DCDC current (separate single-ended sensor)
//
// 50 ms period. Each cycle:
//   1. Reconfigure ADC3 regular channel for INP3 differential (pack),
//      start, poll, get
//   2. Disconnect check: re-read INP3 SINGLE-ENDED (OUT_P leg) and test
//      it sits in the plausible window; debounce N cycles -> sensor_fault
//   3. Feed both into CurrentService::update_from_adc
//   4. Reconfigure ADC3 regular channel for INP11 single-ended (DCDC),
//      start, poll, get
//   5. Feed into CurrentService::update_dcdc_from_adc
//   6. On HAL error at any step: skip that channel's update so the
//      corresponding last_*_update_tick does not advance -> SafetyTask
//      trips on staleness for the pack channel (IStaleMs = 200 ms) and
//      forces ERROR. DCDC staleness is informational only (no FSM impact).
//
// The channel-swap costs ~10 us of HAL overhead per swap; total cycle
// budget at 50 ms is comfortable. If timing ever gets tight we can
// move to ADC3 scan mode (two ranks via CubeMX) and drop the runtime
// reconfigure.
//
// First-time calibration via HAL_ADCEx_Calibration_Start runs once at
// task entry -- BOTH single-ended and differential offset/linearity,
// since this revision mixes a differential channel (pack) and a
// single-ended one (DCDC). CubeMX configures ADC3 rank 1 = Channel 3
// differential (PF7/PF8); the rank-1 channel + single/diff selection is
// re-set on every read so the order and mode are deterministic.

#include "app/current_task.h"

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "soc_estimator.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern ADC_HandleTypeDef hadc3;
}

// Pack state of charge, 0..100 %, or ams::soc::Unknown (0xFF) when there is no
// trustworthy estimate. Written only by CurrentSensorTask, read by AcuCanTask
// for CAN 0x130 -- same single-writer / 8-bit-atomic contract as
// g_state_telemetry. TELEMETRY ONLY: no safety predicate reads this, and
// nothing downstream of it can influence the FSM, the contactors or AMS_OK.
extern "C" volatile std::uint8_t g_soc_percent = ams::soc::Unknown;

namespace {

// Failed-conversion counters for telemetry. Separate for pack vs DCDC
// so the bench can localise which channel is the problem.
volatile std::uint32_t g_current_adc_fail      = 0;
volatile std::uint32_t g_current_adc_dcdc_fail = 0;

// Disconnect debounce: consecutive cycles the OUT_P single-ended leg
// read landed outside the plausible window. Only after
// CurrentDisconnectConfirm in a row do we assert sensor_fault, so a
// single glitch during the diff->SE channel reconfigure can't latch a
// sticky Error. Exposed for telemetry/bench visibility.
volatile std::uint8_t  g_current_disconnect_streak = 0;

// ---------------------------------------------------------------------------
// State of charge -- Coulomb counting anchored on OCV. TELEMETRY ONLY; see the
// safety contract at the top of soc_estimator.hpp. Owned by this task (single
// writer), published as a plain byte for AcuCanTask to read.
// ---------------------------------------------------------------------------
ams::soc::CoulombCounter s_soc;
std::uint32_t            s_soc_last_tick = 0;
std::uint32_t            s_rest_since    = 0;   // tick the pack last went quiet
bool                     s_resting       = false;

// Published SoC percent, or soc::Unknown. Single-writer (this task), read by
// AcuCanTask -- an 8-bit read is atomic on Cortex-M7, same contract as
// g_state_telemetry.
void update_soc() noexcept {
    using namespace ams;

    const auto        cur = CurrentService::instance().snapshot();
    const std::uint32_t now = osKernelGetTickCount();

    // A faulted or stale current sensor makes the integral meaningless: the
    // charge that flowed while we could not measure it is simply unknown, and
    // continuing to integrate a stale value would fabricate it. Drop the anchor
    // and wait for a fresh OCV rest.
    // Unsigned tick subtraction, wrap-safe -- same form the safety predicate
    // uses for IStaleMs. This task is the writer, so a stale timestamp means an
    // ADC conversion failed and update_from_adc was never called.
    const std::uint32_t age = now - cur.last_update_tick;
    if (cur.sensor_fault || age > config::IStaleMs) {
        s_soc.invalidate();
        s_resting = false;
        g_soc_percent = soc::Unknown;
        s_soc_last_tick = now;
        return;
    }

    // Track how long the pack has been quiet, for the OCV rest gate.
    const std::int32_t mag = cur.filtered_mA < 0 ? -cur.filtered_mA : cur.filtered_mA;
    if (mag <= static_cast<std::int32_t>(config::SocRestCurrentMa)) {
        if (!s_resting) { s_resting = true; s_rest_since = now; }
    } else {
        s_resting = false;
    }

    // Integrate first, so the charge moved since the last sample is counted
    // even on the cycle an anchor lands.
    if (s_soc_last_tick != 0u) {
        s_soc.update(cur.filtered_mA, now - s_soc_last_tick);
    }
    s_soc_last_tick = now;

    // (Re-)anchor whenever the pack has been genuinely at rest long enough.
    // This is what stops Coulomb drift accumulating without bound -- sensor
    // offset integrates linearly, so an unanchored counter is only as good as
    // its zero-point calibration times the hours since it started.
    //
    // Anchor off the MINIMUM cell: usable pack charge is set by the weakest
    // cell, and that is the number an operator cares about. It also matches the
    // TFM model's framing, which estimates a single cell's SoC.
    const auto bms = BmsService::instance().snapshot();
    if (bms.module_online_mask == config::AllModulesMask &&
        bms.first_full_poll_done &&
        s_resting &&
        soc::ocv_anchor_valid(cur.filtered_mA, now - s_rest_since)) {
        s_soc.anchor(soc::ocv_to_soc_permille(bms.min_cell_mV));
    }

    g_soc_percent = s_soc.soc_percent();
}

// One-shot single-channel read on ADC3. Reconfigures rank 1 to the
// requested channel and single/differential mode, starts, polls, gets
// the value. Returns false on any HAL failure -- caller bumps its own
// fail counter and skips the CurrentService update so freshness doesn't
// advance.
bool read_adc3_channel(std::uint32_t channel, std::uint32_t single_diff,
                       std::uint16_t& out_raw) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    cfg.SingleDiff   = single_diff;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    cfg.OffsetSign   = ADC3_OFFSET_SIGN_NEGATIVE;
    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK)    return false;
    if (HAL_ADC_Start(&hadc3)              != HAL_OK)    return false;
    if (HAL_ADC_PollForConversion(&hadc3, 5) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc3);
        return false;
    }
    out_raw = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc3));
    (void)HAL_ADC_Stop(&hadc3);
    return true;
}

}  // namespace

extern "C" void ams_current_sensor_task_run(void *argument) {
    (void)argument;

    // Calibrate before first use. Offset + linearity for BOTH the
    // single-ended (DCDC / INP11) and differential (pack / INP3+INN3)
    // signal paths -- on STM32H7 the two have independent calibration
    // factors. Results are applied internally; nothing to consume.
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_DIFFERENTIAL_ENDED);

    std::uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        last_wake += ams::config::CurrentPeriodMs;
        osDelayUntil(last_wake);

        // --- Pack current (PF7/PF8 / ADC3_INP3+INN3, differential) ---
        std::uint16_t raw_pack = 0;
        if (read_adc3_channel(ADC_CHANNEL_3, ADC_DIFFERENTIAL_ENDED, raw_pack)) {
            // Disconnect check: read OUT_P (PF7 / CH3) SINGLE-ENDED and
            // test it sits in the plausible window. With the internal
            // pull-down an open connector collapses OUT_P toward 0 V.
            // A failed SE read (or an in-window read) clears the streak
            // so we never fault on a missing sample -- only a sustained
            // out-of-window leg latches sensor_fault. INN3/PF8 can't be
            // sampled independently in a differential pair, so we watch
            // the OUT_P leg; an OUT_N-only break is caught instead by
            // the over-limit predicate (skewed differential).
            std::uint16_t raw_legp = 0;
            const bool se_ok = read_adc3_channel(ADC_CHANNEL_3, ADC_SINGLE_ENDED, raw_legp);
            if (se_ok && !ams::CurrentService::leg_voltage_plausible(raw_legp)) {
                if (g_current_disconnect_streak < ams::config::CurrentDisconnectConfirm) {
                    ++g_current_disconnect_streak;
                }
            } else {
                g_current_disconnect_streak = 0;
            }
            const bool sensor_fault =
                g_current_disconnect_streak >= ams::config::CurrentDisconnectConfirm;

            ams::CurrentService::instance().update_from_adc(
                raw_pack, osKernelGetTickCount(), sensor_fault);
        } else {
            ++g_current_adc_fail;
        }

        // --- DCDC current (PC1 / ADC3_INP11, single-ended) ---
        std::uint16_t raw_dcdc = 0;
        if (read_adc3_channel(ADC_CHANNEL_11, ADC_SINGLE_ENDED, raw_dcdc)) {
            ams::CurrentService::instance().update_dcdc_from_adc(
                raw_dcdc, osKernelGetTickCount());
        } else {
            ++g_current_adc_dcdc_fail;
        }

        // --- State of charge (TELEMETRY ONLY) ---
        // Runs here rather than in MainTask because this task already owns the
        // current samples and is NOT realtime-critical. Nothing in the safety
        // path reads the result: it reaches CAN 0x130 and stops there. If every
        // line below misbehaved the AMS would fault, precharge and open the
        // contactors exactly as it does today.
        update_soc();
    }
}
