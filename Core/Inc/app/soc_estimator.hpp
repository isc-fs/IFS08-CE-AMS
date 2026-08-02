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

}  // namespace ams::soc
