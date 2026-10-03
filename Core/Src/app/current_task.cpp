// SPDX-License-Identifier: proprietary
//
// Pack-current acquisition: the Bourns SSA-2-250A on PF7/PF8 (ADC3_INP3/INN3,
// read in ADC DIFFERENTIAL mode; OUT_P = PF7, OUT_N = PF8).
//
// ADC3 free-runs with its hardware oversampler (config::CurrentAdcFracBits for
// the numbers): every ~80 us it delivers the mean of 64 conversions, and DMA
// writes those samples into one of two capture buffers with no CPU work. Each
// CurrentPeriodMs (50 ms) cycle:
//   1. Stop the running capture and note how many samples it holds.
//   2. Disconnect check: read INP3 SINGLE-ENDED (the OUT_P leg) and test it
//      sits in the plausible window; debounce N cycles -> sensor_fault.
//   3. Restart the capture into the other buffer. The pause between captures
//      is the stop, one single-ended oversampled read and the restart: ~0.1 ms
//      in 50 ms.
//   4. Feed the mean of the finished capture -- the average pack current over
//      the cycle -- into CurrentService::update_from_q4. Still one value per
//      cycle, the cadence the over-current filter and IStaleMs are sized for,
//      but now every amp-second of the cycle is in it, where a single sample
//      would catch or miss a pulse by chance.
//   5. Split the finished capture into EleWindowsPerCycle 10 ms windows and
//      push one EleRecord per window (mean / min / max) to ELEnnnn.BIN.
//      TELEMETRY ONLY.
//   6. Integrate the capture's mean into the charge totals, and advance the
//      SoC filter. TELEMETRY ONLY.
//   On any HAL failure, or a capture with no samples, update_from_q4 is not
//   called, so last_update_tick does not advance -> SafetyTask trips on
//   staleness (IStaleMs = 200 ms) and forces ERROR, as before.
//
// First-time calibration via HAL_ADCEx_Calibration_Start runs once at task
// entry -- BOTH single-ended and differential, since the disconnect check
// reads the same channel single-ended. CubeMX configures ADC3 (oversampling,
// continuous mode, DMA on DMA1_Stream1); the channel and its single/diff mode
// are re-set before every start so the mode is deterministic.

#include "app/current_task.h"

#include "ams_config.hpp"
#include "app/sd_logger_task.h"
#include "bin_log.hpp"
#include "bms_service.hpp"
#include "current_capture.hpp"
#include "current_service.hpp"
#include "log_record.hpp"
#include "soc_estimator.hpp"
#include "vehicle_service.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <cmath>

extern "C" {
extern ADC_HandleTypeDef hadc3;
}

// Pack state of charge, 0..100 %, or ams::soc::Unknown (0xFF) when there is no
// trustworthy estimate. Written only by CurrentSensorTask, read by AcuCanTask
// for CAN 0x130 -- same single-writer / 8-bit-atomic contract as
// g_state_telemetry. TELEMETRY ONLY: no safety predicate reads this, and
// nothing downstream of it can influence the FSM, the contactors or AMS_OK.
extern "C" volatile std::uint8_t g_soc_percent = ams::soc::Unknown;

// The same estimate at full resolution, for the SD log. 1 % of an 18 Ah pack is
// 648 A*s, which hides everything the filter does between log rows; 1 ppm is
// 0.0648 A*s. g_soc_ppm / g_soc_sig_ppm (1-sigma, sqrt of the filter variance)
// are meaningful only while g_soc_flags has soc::flags::Valid. g_soc_seeds is a
// wrapping count of seeds, so a reader spots a re-seed between two rows without
// anyone having to clear a flag. Single writer: this task. TELEMETRY ONLY.
extern "C" volatile std::uint32_t g_soc_ppm     = 0;
extern "C" volatile std::uint32_t g_soc_sig_ppm = 0;
extern "C" volatile std::uint8_t  g_soc_flags   = 0;
extern "C" volatile std::uint8_t  g_soc_seeds   = 0;

// Monotonic charge totals (soc::ChargeTally) for the SD log, mA*s, wrapping.
// Single writer: this task. TELEMETRY ONLY.
extern "C" volatile std::uint32_t g_q_dis_mAs = 0;
extern "C" volatile std::uint32_t g_q_chg_mAs = 0;
extern "C" volatile std::uint16_t g_q_gaps    = 0;

namespace {

static_assert(ams::config::EleWindowsPerCycle * 10u == ams::config::CurrentPeriodMs,
              "ELE windows must be 10 ms");

// Failed captures (HAL error, or a cycle that produced no samples).
volatile std::uint32_t g_current_adc_fail = 0;

// Capture buffers. DMA1 cannot reach DTCM, so they live in AXI SRAM
// (.adc_dma, RAM_D1); the D-cache is off on this part, so no cache
// maintenance. The DMA fills s_capture[s_active] while the task reduces the
// other one.
alignas(32) std::uint16_t s_capture[2][ams::config::CurrentCaptureCapacity]
    __attribute__((section(".adc_dma")));
std::uint8_t  s_active          = 0;
bool          s_capture_running = false;
std::uint32_t s_capture_start   = 0;     // tick the running capture started
std::uint16_t s_ele_seq         = 0;

// Disconnect debounce: consecutive cycles the OUT_P single-ended leg
// read landed outside the plausible window. Only after
// CurrentDisconnectConfirm in a row do we assert sensor_fault, so a
// single glitch during the diff->SE channel reconfigure can't latch a
// sticky Error. Exposed for telemetry/bench visibility.
volatile std::uint8_t  g_current_disconnect_streak = 0;

// ---------------------------------------------------------------------------
// State of charge -- EKF over the equivalent-circuit model. TELEMETRY ONLY; see
// the safety contract at the top of soc_estimator.hpp. Owned by this task
// (single writer), published as a plain byte for AcuCanTask to read.
// ---------------------------------------------------------------------------
ams::soc::KalmanSoc s_soc;
std::uint32_t       s_soc_last_tick = 0;

ams::soc::ChargeTally s_charge;
std::uint32_t         s_charge_last_tick = 0;

// Publish the full-resolution estimate for the SD log. sigma is the square root
// of the filter variance, in the same ppm units as the estimate.
void publish_soc(std::uint8_t flags) noexcept {
    if (s_soc.valid()) {
        g_soc_ppm     = static_cast<std::uint32_t>(s_soc.soc() * 1e6 + 0.5);
        g_soc_sig_ppm = static_cast<std::uint32_t>(std::sqrt(s_soc.variance()) * 1e6 + 0.5);
        flags         = static_cast<std::uint8_t>(flags | ams::soc::flags::Valid);
    } else {
        g_soc_ppm     = 0;
        g_soc_sig_ppm = 0;
    }
    g_soc_flags = flags;
}

// State-of-charge update, run once per CurrentPeriodMs (50 ms).
//
// Structure is the textbook EKF pair: PREDICT from the current integral
// (which is exactly Coulomb counting), then CORRECT against the measured cell
// voltage through the equivalent-circuit model. The gain schedules itself off
// the OCV slope, so no rest gate and no hand-written blend -- the filter leans
// on voltage where the curve is steep and on the integral where it is flat.
//
// TELEMETRY ONLY. Nothing below can influence the FSM, the contactors or
// AMS_OK; the single byte it publishes is read only by AcuCanTask for 0x130.
void update_soc() noexcept {
    using namespace ams;

    const auto          cur = CurrentService::instance().snapshot();
    const std::uint32_t now = osKernelGetTickCount();

    // Unsigned tick subtraction, wrap-safe -- same form the safety predicate
    // uses for IStaleMs. This task is the writer, so a stale timestamp means an
    // ADC conversion failed and update_from_q4 was never called.
    const std::uint32_t age = now - cur.last_update_tick;
    if (cur.sensor_fault || age > config::IStaleMs) {
        // Charge that moved while we could not measure it is simply unknown,
        // and predicting through it would fabricate it. Drop the estimate
        // rather than publish a number we cannot stand behind.
        s_soc.invalidate();
        g_soc_percent   = soc::Unknown;
        publish_soc(0u);
        s_soc_last_tick = now;
        return;
    }

    // --- predict (Coulomb counting) ---
    if (s_soc_last_tick != 0u) {
        s_soc.predict(cur.filtered_mA, now - s_soc_last_tick);
    }
    s_soc_last_tick = now;

    // --- correct (voltage residual) ---
    // Needs a trustworthy cell voltage, so require the whole chain online and
    // at least one complete poll. Without it we keep predicting, which degrades
    // gracefully to plain Coulomb counting rather than to nothing.
    // Four fields, not the whole ~690 B BmsState: this task's stack is small.
    const auto bms = BmsService::instance().soc_inputs();
    const bool cells_trustworthy =
        bms.module_online_mask == config::AllModulesMask && bms.first_full_poll_done;

    std::uint8_t flags = 0;
    if (cells_trustworthy) {
        // Seed on the first good sample. Unlike the pure-CC path this does NOT
        // wait for the pack to rest: P starts wide and the correction step
        // walks the estimate in, which is the whole advantage of the filter.
        // A seed taken under load is a poor guess, and that is fine -- the
        // I^2 term in R makes the filter discount it until the pack quietens.
        if (!s_soc.valid()) {
            s_soc.seed(bms.min_cell_mV);
            g_soc_seeds = static_cast<std::uint8_t>(g_soc_seeds + 1u);
        }
        // Minimum cell: usable pack charge is set by the weakest element.
        // avg_tempC drives R_int -- we cannot know the min cell's own
        // temperature, and the pack average is the honest representative.
        flags = s_soc.correct(bms.min_cell_mV, cur.filtered_mA, bms.avg_tempC)
                    ? soc::flags::Corrected : soc::flags::CorrectionSkipped;
    } else {
        flags = soc::flags::CoulombOnly;
    }

    g_soc_percent = s_soc.soc_percent();
    publish_soc(flags);
}

// Select ADC3_INP3 (the only channel) in the given single/differential mode.
// The ADC must be stopped: the mode bit can only change while it is disabled,
// which HAL_ADC_Stop / HAL_ADC_Stop_DMA leave it.
bool configure_channel(std::uint32_t single_diff) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = ADC_CHANNEL_3;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_47CYCLES_5;   // KEEP in sync with AMS.ioc
    cfg.SingleDiff   = single_diff;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    cfg.OffsetSign   = ADC3_OFFSET_SIGN_NEGATIVE;
    return HAL_ADC_ConfigChannel(&hadc3, &cfg) == HAL_OK;
}

// Start a differential capture into buffer `buf`. One-shot DMA: if the task
// is late the DMA stops at the end of the buffer and the ADC keeps converting
// into an overwritten data register, which is harmless.
bool start_capture(std::uint8_t buf) noexcept {
    if (!configure_channel(ADC_DIFFERENTIAL_ENDED)) return false;
    return HAL_ADC_Start_DMA(&hadc3, reinterpret_cast<std::uint32_t*>(s_capture[buf]),
                             ams::config::CurrentCaptureCapacity) == HAL_OK;
}

// Stop the running capture; returns how many samples it wrote. The DMA count
// is read before the stop, so a sample landing during the stop is simply not
// counted.
std::uint16_t stop_capture() noexcept {
    const std::uint32_t left = __HAL_DMA_GET_COUNTER(hadc3.DMA_Handle);
    (void)HAL_ADC_Stop_DMA(&hadc3);
    return (left >= ams::config::CurrentCaptureCapacity)
               ? 0u
               : static_cast<std::uint16_t>(ams::config::CurrentCaptureCapacity - left);
}

// One single-ended oversampled read of the OUT_P leg, as a Q4 code. The ADC
// is in continuous mode, so take the first result and stop.
bool read_leg_q4(std::uint16_t& out_q4) noexcept {
    if (!configure_channel(ADC_SINGLE_ENDED))  return false;
    if (HAL_ADC_Start(&hadc3) != HAL_OK)       return false;
    if (HAL_ADC_PollForConversion(&hadc3, 2) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc3);
        return false;
    }
    out_q4 = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc3));
    (void)HAL_ADC_Stop(&hadc3);
    return true;
}

// DC-bus voltage and its age for ELE. AcuCanTask, the writer, runs at this
// task's priority and can time-slice in mid-copy, so take two copies and, if
// their 0x100 ticks differ, a third (0x100 arrives every ~10 ms, so the third
// is clean). TELEMETRY ONLY.
void dc_bus_sample(std::uint32_t now, std::uint16_t& volts, std::uint16_t& age) noexcept {
    const auto& svc = ams::VehicleService::instance();
    const ams::VehicleState a = svc.snapshot();
    ams::VehicleState       v = svc.snapshot();
    if (a.last_dc_bus_tick != v.last_dc_bus_tick) v = svc.snapshot();   // a 0x100 landed mid-copy
    volts = v.dc_bus_V;
    age   = ams::log_csv::age_ms(now, v.last_dc_bus_tick);
}

// Split a finished capture into its 10 ms windows and push one ELE record
// each. TELEMETRY ONLY.
void publish_ele(const std::uint16_t* buf, std::uint16_t n,
                 std::uint32_t t_start, std::uint32_t t_stop, bool sensor_fault) noexcept {
    using namespace ams;
    std::uint16_t dc_v = 0, dc_age = 0;
    dc_bus_sample(t_stop, dc_v, dc_age);
    const std::uint8_t flags = static_cast<std::uint8_t>(
        (sensor_fault ? bin_log::ele_flag::SensorFault : 0u) |
        (n >= config::CurrentCaptureCapacity ? bin_log::ele_flag::Overrun : 0u));

    for (std::uint8_t i = 0; i < config::EleWindowsPerCycle; ++i) {
        const std::uint16_t b = current_capture::window_begin(n, config::EleWindowsPerCycle, i);
        const std::uint16_t e = current_capture::window_begin(n, config::EleWindowsPerCycle,
                                                              static_cast<std::uint8_t>(i + 1u));
        const current_capture::Window w = current_capture::reduce(buf, b, e);
        if (w.count == 0u) continue;
        (void)sd_ele_push(current_capture::make_record(
            w, current_capture::tick_at(t_start, t_stop, e, n), s_ele_seq++, flags,
            dc_v, dc_age));   // best-effort: a full ring drops the record, never blocks
    }
}

}  // namespace

extern "C" void ams_current_sensor_task_run(void *argument) {
    (void)argument;

    // Calibrate before first use, both signal paths: the capture is
    // differential, the disconnect check single-ended, and on STM32H7 the two
    // have independent calibration factors.
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3,
                                ADC_CALIB_OFFSET_LINEARITY,
                                ADC_DIFFERENTIAL_ENDED);

    s_capture_running = start_capture(s_active);
    s_capture_start   = osKernelGetTickCount();
    std::uint32_t last_wake = s_capture_start;

    for (;;) {
        last_wake += ams::config::CurrentPeriodMs;
        osDelayUntil(last_wake);

        // --- 1. Stop the capture that ran through the last cycle ---
        const std::uint8_t  done    = s_active;
        const std::uint32_t t_start = s_capture_start;
        const std::uint16_t n       = s_capture_running ? stop_capture() : 0u;
        const std::uint32_t t_stop  = osKernelGetTickCount();

        // --- 2. Disconnect check: OUT_P (PF7 / CH3) single-ended ---
        // With the internal pull-down an open connector collapses OUT_P toward
        // 0 V. A failed read (or an in-window read) clears the streak so we
        // never fault on a missing sample -- only a sustained out-of-window leg
        // latches sensor_fault. INN3/PF8 can't be sampled independently in a
        // differential pair, so we watch the OUT_P leg; an OUT_N-only break is
        // caught instead by the over-limit predicate (skewed differential).
        std::uint16_t legp_q4 = 0;
        const bool se_ok = read_leg_q4(legp_q4);

        // --- 3. Restart into the other buffer straight away ---
        s_active          = static_cast<std::uint8_t>(s_active ^ 1u);
        s_capture_running = start_capture(s_active);
        s_capture_start   = osKernelGetTickCount();

        if (se_ok && !ams::CurrentService::leg_voltage_plausible(
                         ams::CurrentService::q4_to_raw(legp_q4))) {
            if (g_current_disconnect_streak < ams::config::CurrentDisconnectConfirm) {
                ++g_current_disconnect_streak;
            }
        } else {
            g_current_disconnect_streak = 0;
        }
        const bool sensor_fault =
            g_current_disconnect_streak >= ams::config::CurrentDisconnectConfirm;

        if (n == 0u) {
            // No samples: HAL failure or a capture that never started. Leave
            // last_update_tick alone so the staleness predicate sees it.
            ++g_current_adc_fail;
        } else {
            const std::uint16_t* buf = s_capture[done];

            // --- 4. Safety path: the cycle's mean current ---
            const std::uint32_t mean_q4 = ams::current_capture::capture_mean_q4(buf, n);
            ams::CurrentService::instance().update_from_q4(mean_q4, t_stop, sensor_fault);

            // --- 5. ELE windows (TELEMETRY ONLY) ---
            publish_ele(buf, n, t_start, t_stop, sensor_fault);

            // --- 6. Monotonic charge totals: the capture's mean current over
            // the time since the previous capture ended. A faulted sensor's
            // reading is not charge.
            if (s_charge_last_tick != 0u) {
                if (sensor_fault) {
                    s_charge.skip();
                } else {
                    s_charge.add(ams::CurrentService::adc_q4_to_mA(mean_q4),
                                 t_stop - s_charge_last_tick);
                }
                g_q_dis_mAs = s_charge.discharge_mAs();
                g_q_chg_mAs = s_charge.charge_mAs();
                g_q_gaps    = s_charge.gaps();
            }
            s_charge_last_tick = t_stop;
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
