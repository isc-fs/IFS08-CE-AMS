// SPDX-License-Identifier: proprietary
//
// State-of-charge estimation -- Coulomb counting with OCV-based anchoring.
// Pure logic; no HAL, no FreeRTOS, so the host unit-test build exercises it
// directly.
//
// SAFETY CONTRACT: SoC is TELEMETRY ONLY. No safety predicate reads it, and
// nothing here can influence the FSM, the contactors or AMS_OK. If this whole
// file produced garbage the AMS would behave exactly as it does today. That is
// deliberate and must stay true -- a charge estimate is an operator convenience,
// not a protection.
//
// Method
// ------
// Coulomb counting integrates pack current to track charge moved:
//
//     SoC(t) = SoC(t0) - (1/Q) * integral of I dt
//
// It is exact over short horizons and drifts without bound over long ones,
// because sensor offset integrates linearly. So it needs (a) an anchor to start
// from and (b) periodic re-anchoring. OCV supplies both: at rest, a cell's
// terminal voltage IS its open-circuit voltage, and the VTC6 OCV curve maps that
// to SoC directly. Under load the terminal voltage includes I*R_int and the
// mapping is invalid -- hence the rest gate in ocv_anchor_valid().
//
// This is the CC half of the estimator described in the TFM (raulmoranguerra/
// TFM_RMG). The GRU there predicts a bounded residual ON TOP of exactly this
// signal (cc_skip, +/-0.2 clipped), so Coulomb counting is a prerequisite for
// the learned model, not an alternative to it -- and per that work's own
// figures, CC alone is already within ~3 pp of the network (6.89 % vs 3.91 %
// MAE). Getting CC right is most of the value.

#pragma once

#include "ams_config.hpp"

#include <cstdint>

namespace ams::soc {

// Sentinel published when no trustworthy estimate exists (never anchored, or
// the anchor was invalidated). Distinct from 0 %, which is a real reading.
inline constexpr std::uint8_t Unknown = 0xFFu;

// --- VTC6 open-circuit voltage curve ---------------------------------------
// Fitted from the cell datasheet + bench characterisation in the TFM
// (BMS_DL/sim/params.py: OCV_SOC_BP / OCV_V_BP). Piecewise linear between
// breakpoints, which is what the pack simulator itself uses -- so firmware and
// the training-data generator share one curve rather than drifting apart.
//
// NOTE the curve is FLAT in the middle: 3.4676 V -> 3.6551 V spans SoC 0.30 ->
// 0.50, i.e. 188 mV for 20 points, ~9.4 mV per point. Near the top it steepens
// to ~2.3 mV per point (3.9936 -> 4.2200 over 0.90 -> 1.00). That asymmetry is
// why an OCV anchor taken mid-pack is far less precise than one taken near full,
// and why a millivolt of measurement error costs ~0.1 SoC points in the middle
// but ~0.04 near the top.
inline constexpr std::uint8_t  OcvPoints          = 10u;
inline constexpr std::uint16_t OcvCellMv[OcvPoints] = {
    2500, 2695, 2891, 3282, 3468, 3655, 3843, 3936, 3994, 4220
};
inline constexpr std::uint16_t OcvSocPermille[OcvPoints] = {
       0,   50,  100,  200,  300,  500,  700,  800,  900, 1000
};

// Map a rested cell voltage to SoC in permille (0..1000). Clamps outside the
// curve rather than extrapolating -- below 2500 mV the cell is past the
// protection cutoff anyway, and above 4220 mV it is over the charge limit, so
// in both cases the rail is the honest answer.
[[nodiscard]] inline std::uint16_t ocv_to_soc_permille(std::uint16_t cell_mV) noexcept {
    if (cell_mV <= OcvCellMv[0])               return OcvSocPermille[0];
    if (cell_mV >= OcvCellMv[OcvPoints - 1u])  return OcvSocPermille[OcvPoints - 1u];

    for (std::uint8_t i = 1; i < OcvPoints; ++i) {
        if (cell_mV <= OcvCellMv[i]) {
            const std::int32_t v0 = OcvCellMv[i - 1u];
            const std::int32_t v1 = OcvCellMv[i];
            const std::int32_t s0 = OcvSocPermille[i - 1u];
            const std::int32_t s1 = OcvSocPermille[i];
            const std::int32_t span = v1 - v0;          // > 0, breakpoints strictly rise
            const std::int32_t num  = (static_cast<std::int32_t>(cell_mV) - v0) * (s1 - s0);
            return static_cast<std::uint16_t>(s0 + (num + span / 2) / span);   // round-to-nearest
        }
    }
    return OcvSocPermille[OcvPoints - 1u];
}

// --- Rest gate --------------------------------------------------------------
// An OCV anchor is only meaningful when the pack is genuinely at rest. Two
// conditions, both required:
//
//   |I| below RestCurrentMa      -- no I*R_int term corrupting terminal voltage
//   rested for RestSettleMs      -- diffusion relaxation has decayed
//
// The settle time matters more than people expect: the ohmic part of the
// polarisation recovers in microseconds but the concentration gradient takes
// minutes. Anchoring 10 s after a hard discharge reads several tens of mV low,
// which on the flat part of the curve is several SoC points of error baked in.
[[nodiscard]] inline bool ocv_anchor_valid(std::int32_t  current_mA,
                                           std::uint32_t rested_ms) noexcept {
    const std::int32_t mag = current_mA < 0 ? -current_mA : current_mA;
    return mag <= static_cast<std::int32_t>(config::SocRestCurrentMa) &&
           rested_ms >= config::SocRestSettleMs;
}

// --- Coulomb counter --------------------------------------------------------
// Charge is tracked in milliamp-seconds as a signed 64-bit accumulator.
//
// Why mA*s and not permille directly: integrating straight into permille throws
// away every sample smaller than one part in a thousand of pack capacity. At
// 18 Ah that is 64.8 A*s per point -- a 5 A current for a 50 ms tick moves
// 0.25 A*s, so ~260 consecutive samples would round to zero and the counter
// would sit still through a real discharge. Accumulating the raw charge and
// converting only on read keeps every sample.
//
// Why 64-bit: 18 Ah = 64.8e6 mA*s, which fits in int32 -- but the accumulator
// takes signed contributions over hours and the intermediate mA*ms product
// (before the /1000) reaches 1e9 for a 200 A sample over a 5 s gap. int64 makes
// the arithmetic unarguable for the sake of 4 bytes.
class CoulombCounter {
public:
    // Anchor to a known SoC. Called on a valid OCV rest reading; also the only
    // way out of the Unknown state.
    void anchor(std::uint16_t soc_permille) noexcept {
        charge_mAs_ = permille_to_mAs(soc_permille);
        anchored_   = true;
    }

    // Integrate one current sample. `current_mA` follows the firmware-wide
    // convention (+ = DISCHARGE, see CurrentState::filtered_mA), so a positive
    // current REMOVES charge.
    //
    // Rejects implausible dt: a gap longer than SocMaxIntegrationGapMs means the
    // task was starved or the counter was just anchored, and integrating across
    // it would invent charge that may never have flowed. Skipping the sample
    // loses at most that interval's charge, which is the conservative error.
    void update(std::int32_t current_mA, std::uint32_t dt_ms) noexcept {
        if (!anchored_) return;
        if (dt_ms == 0u || dt_ms > config::SocMaxIntegrationGapMs) return;

        // mA * ms / 1000 -> mA*s. Done in int64 before the divide so small
        // currents at short dt are not truncated to zero.
        const std::int64_t delta_mAs =
            (static_cast<std::int64_t>(current_mA) * static_cast<std::int64_t>(dt_ms)) / 1000;
        charge_mAs_ -= delta_mAs;                     // + current = discharge

        // Clamp to the physical range. A counter that has drifted past the rails
        // is already wrong; letting it run away makes the next anchor's
        // correction look like a fault rather than a re-sync.
        const std::int64_t full = permille_to_mAs(1000u);
        if (charge_mAs_ < 0)    charge_mAs_ = 0;
        if (charge_mAs_ > full) charge_mAs_ = full;
    }

    // Invalidate -- e.g. the current sensor faulted, so the integral can no
    // longer be trusted. Publishes Unknown until re-anchored.
    void invalidate() noexcept { anchored_ = false; }

    [[nodiscard]] bool anchored() const noexcept { return anchored_; }

    [[nodiscard]] std::uint16_t soc_permille() const noexcept {
        if (!anchored_) return 0u;
        const std::int64_t full = permille_to_mAs(1000u);
        if (full <= 0) return 0u;
        return static_cast<std::uint16_t>((charge_mAs_ * 1000 + full / 2) / full);
    }

    // 0..100 %, or Unknown when there is no trustworthy estimate. This is what
    // reaches CAN 0x130.
    [[nodiscard]] std::uint8_t soc_percent() const noexcept {
        if (!anchored_) return Unknown;
        return static_cast<std::uint8_t>((soc_permille() + 5u) / 10u);
    }

private:
    static constexpr std::int64_t permille_to_mAs(std::uint16_t permille) noexcept {
        // capacity_mAh * 3600 / 1000 * permille  == capacity_mAh * 18 / 5 * permille
        return static_cast<std::int64_t>(config::PackCapacityMah) * 36
             * static_cast<std::int64_t>(permille) / 10;
    }

    std::int64_t charge_mAs_ = 0;
    bool         anchored_   = false;
};

// ---------------------------------------------------------------------------
// SoC -> OCV, and the curve slope. The Coulomb counter only needs the inverse
// map (voltage -> SoC) for anchoring; the Kalman filter needs the forward map
// for its measurement prediction and the SLOPE for its observation matrix.
// ---------------------------------------------------------------------------

// Forward map: rested cell voltage for a given SoC. Same breakpoints as
// ocv_to_soc_permille, walked the other way.
[[nodiscard]] inline double ocv_from_soc(double soc) noexcept {
    if (soc <= 0.0) return static_cast<double>(OcvCellMv[0]) * 1e-3;
    if (soc >= 1.0) return static_cast<double>(OcvCellMv[OcvPoints - 1u]) * 1e-3;

    const double permille = soc * 1000.0;
    for (std::uint8_t i = 1; i < OcvPoints; ++i) {
        if (permille <= static_cast<double>(OcvSocPermille[i])) {
            const double s0 = static_cast<double>(OcvSocPermille[i - 1u]);
            const double s1 = static_cast<double>(OcvSocPermille[i]);
            const double v0 = static_cast<double>(OcvCellMv[i - 1u]) * 1e-3;
            const double v1 = static_cast<double>(OcvCellMv[i]) * 1e-3;
            return v0 + (v1 - v0) * (permille - s0) / (s1 - s0);
        }
    }
    return static_cast<double>(OcvCellMv[OcvPoints - 1u]) * 1e-3;
}

// dOCV/dSoC in volts per unit SoC. THIS IS THE OBSERVATION MATRIX H, and it is
// why an EKF is the right tool here rather than a fixed blend: the gain is
// proportional to it, so the filter automatically trusts voltage where the curve
// carries information and ignores it where the curve is flat.
//
// On this fitted VTC6 curve the slope spans roughly 0.94 V/unit on the plateau
// (3468 -> 3655 mV over SoC 0.30 -> 0.50) to 2.26 V/unit at the top
// (3994 -> 4220 mV over 0.90 -> 1.00) -- a 2.4x swing in how much a millivolt
// is worth. Outside the curve the slope is zero, which correctly means "this
// measurement says nothing about SoC" and drives the gain to zero rather than
// letting a railed reading yank the estimate.
[[nodiscard]] inline double ocv_slope(double soc) noexcept {
    if (soc <= 0.0 || soc >= 1.0) return 0.0;

    const double permille = soc * 1000.0;
    for (std::uint8_t i = 1; i < OcvPoints; ++i) {
        if (permille <= static_cast<double>(OcvSocPermille[i])) {
            const double ds = (static_cast<double>(OcvSocPermille[i]) -
                              static_cast<double>(OcvSocPermille[i - 1u])) * 1e-3;
            const double dv = (static_cast<double>(OcvCellMv[i]) -
                              static_cast<double>(OcvCellMv[i - 1u])) * 1e-3;
            return dv / ds;
        }
    }
    return 0.0;
}

// Internal resistance of ONE SERIES ELEMENT, in ohms.
//
//     R_cell(T, SoC) = R_NOM * f_SoC(SoC) * max(1 + ALPHA_R*(T - 25), 0.4)
//     R_element      = R_cell / CellsInParallel
//
// The parallel divide is not cosmetic: the LTC measures a 6P GROUP, so the
// resistance in the measurement model is a sixth of a cell's. Forgetting it
// would overstate the I*R term 6x and make the filter fight a drop that is not
// there. The 0.4 floor is the simulator's own clamp, keeping the linear fit
// physical at high temperature.
[[nodiscard]] inline double r_int_element_ohm(double soc, std::int16_t tempC) noexcept {
    // f_SoC: piecewise linear over the fitted breakpoints.
    const double permille = (soc <= 0.0) ? 0.0 : (soc >= 1.0 ? 1000.0 : soc * 1000.0);
    double f_soc = static_cast<double>(config::RIntSocValMilli[config::RIntSocPoints - 1u]) * 1e-3;
    if (permille <= static_cast<double>(config::RIntSocBpPermille[0])) {
        f_soc = static_cast<double>(config::RIntSocValMilli[0]) * 1e-3;
    } else {
        for (std::uint8_t i = 1; i < config::RIntSocPoints; ++i) {
            if (permille <= static_cast<double>(config::RIntSocBpPermille[i])) {
                const double s0 = static_cast<double>(config::RIntSocBpPermille[i - 1u]);
                const double s1 = static_cast<double>(config::RIntSocBpPermille[i]);
                const double v0 = static_cast<double>(config::RIntSocValMilli[i - 1u]) * 1e-3;
                const double v1 = static_cast<double>(config::RIntSocValMilli[i]) * 1e-3;
                f_soc = v0 + (v1 - v0) * (permille - s0) / (s1 - s0);
                break;
            }
        }
    }

    const double alpha = static_cast<double>(config::RIntAlphaMicroPerK) * 1e-6;
    double f_T = 1.0 + alpha * static_cast<double>(tempC - config::RIntTempRefC);
    if (f_T < 0.4) f_T = 0.4;

    const double r_cell = static_cast<double>(config::RIntNomMicroOhm) * 1e-6 * f_soc * f_T;
    return r_cell / static_cast<double>(config::CellsInParallel);
}

// ---------------------------------------------------------------------------
// Extended Kalman filter on SoC.
// ---------------------------------------------------------------------------
// One state (SoC). Prediction IS Coulomb counting; correction is the voltage
// residual against the equivalent-circuit model:
//
//   predict:  x' = x - I*dt/Q                         P' = P + Q_proc*dt
//   measure:  h(x) = OCV(x) - I*R_element(x,T)        H = dOCV/dSoC
//   update:   K = P'H / (H P' H + R)                  x = x' + K*(z - h(x'))
//                                                     P = (1 - K H) P'
//
// Two properties worth stating because they are the reason to prefer this over
// CC-plus-periodic-anchor:
//
//  1. H = dOCV/dSoC means the gain self-schedules. On the flat plateau H is
//     small, K is small, and the filter rides Coulomb counting. Near the ends H
//     is 2.4x larger and voltage pulls harder. No hand-written blending rule.
//
//  2. R grows with I^2, so a measurement taken under load is automatically
//     distrusted -- which is what stops the filter interpreting an I*R drop as
//     lost charge. That replaces CC's binary rest gate with a continuous one.
//
// H deliberately omits the dR_int/dSoC term. It is not negligible at high
// current (~0.17 V/unit at 100 A, vs a 0.94 V/unit plateau slope), but f_SoC is
// non-monotonic and piecewise, so its derivative is discontinuous and would make
// H jump at breakpoints. Inflating R with I^2 handles the same physics smoothly:
// both say "trust voltage less under load", but only one keeps H well behaved.
//
// WHY double AND NOT float: the process-noise step is Q*dt = 1e-8 * 0.05 =
// 5e-10, while float32 epsilon at P ~ 0.04 is ~4.8e-9. Every single increment
// would round to ZERO, P would never grow, and the filter would silently go
// deaf to voltage after its first few corrections -- an EKF failure mode that
// looks like "it just stopped converging" and is very hard to spot from the
// output. A host test (variance_grows_on_predict_shrinks_on_correct) caught it.
// The Cortex-M7 here is -mfpu=fpv5-d16, i.e. hardware double precision, so this
// costs essentially nothing at the 20 Hz this runs at.
//
// TELEMETRY ONLY -- see the safety contract at the top of this file.
class KalmanSoc {
public:
    // First valid measurement seeds the state; there is no separate anchor step
    // and no rest requirement. P starts at SocEkfInitVar (sigma ~20 % SoC), so
    // early corrections pull hard and the estimate converges within seconds
    // instead of waiting minutes for the pack to go quiet.
    void seed(std::uint16_t cell_mV) noexcept {
        soc_       = static_cast<double>(ocv_to_soc_permille(cell_mV)) * 1e-3;
        var_       = config::SocEkfInitVar;
        converged_ = true;
    }

    void invalidate() noexcept { converged_ = false; }
    [[nodiscard]] bool valid() const noexcept { return converged_; }

    // Coulomb-counting prediction. `current_mA` is + = DISCHARGE, matching
    // CurrentState::filtered_mA.
    void predict(std::int32_t current_mA, std::uint32_t dt_ms) noexcept {
        if (!converged_) return;
        if (dt_ms == 0u || dt_ms > config::SocMaxIntegrationGapMs) return;

        const double dt_s      = static_cast<double>(dt_ms) * 1e-3;
        const double amps      = static_cast<double>(current_mA) * 1e-3;
        const double coulombs  = static_cast<double>(config::PackCapacityMah) * 3.6;  // mAh -> A*s
        soc_ -= amps * dt_s / coulombs;

        // Uncertainty always grows through prediction. Without this P would
        // collapse after a few corrections and the filter would stop listening
        // to voltage entirely -- the classic way an EKF goes deaf.
        var_ += config::SocEkfProcessVarPerS * dt_s;
        clamp();
    }

    // Voltage correction against the equivalent-circuit model. `cell_mV` should
    // be the MINIMUM cell: usable pack charge is set by the weakest element.
    void correct(std::uint16_t cell_mV, std::int32_t current_mA,
                 std::int16_t tempC) noexcept {
        if (!converged_) return;

        const double amps = static_cast<double>(current_mA) * 1e-3;
        const double z    = static_cast<double>(cell_mV) * 1e-3;

        // Predicted terminal voltage: OCV minus the ohmic drop. Positive
        // (discharge) current pulls the terminal BELOW OCV.
        const double h = ocv_from_soc(soc_) - amps * r_int_element_ohm(soc_, tempC);
        const double H = ocv_slope(soc_);

        // Flat curve (or railed outside it) -> this measurement carries no SoC
        // information. Bail rather than divide by a near-zero denominator.
        if (H <= 1e-6) return;

        const double R = config::SocEkfMeasVarBase +
                        config::SocEkfMeasVarPerA2 * amps * amps;
        const double S = H * var_ * H + R;
        if (S <= 0.0) return;

        const double K = var_ * H / S;
        soc_ += K * (z - h);
        var_  = (1.0 - K * H) * var_;
        if (var_ < 0.0) var_ = 0.0;      // guard against FP round-off
        clamp();
    }

    [[nodiscard]] double soc() const noexcept { return soc_; }
    [[nodiscard]] double variance() const noexcept { return var_; }

    [[nodiscard]] std::uint16_t soc_permille() const noexcept {
        if (!converged_) return 0u;
        return static_cast<std::uint16_t>(soc_ * 1000.0 + 0.5);
    }

    [[nodiscard]] std::uint8_t soc_percent() const noexcept {
        if (!converged_) return Unknown;
        return static_cast<std::uint8_t>(soc_ * 100.0 + 0.5);
    }

private:
    void clamp() noexcept {
        if (soc_ < 0.0) soc_ = 0.0;
        if (soc_ > 1.0) soc_ = 1.0;
    }

    double soc_       = 0.0;
    double var_       = config::SocEkfInitVar;
    bool  converged_ = false;
};

}  // namespace ams::soc
