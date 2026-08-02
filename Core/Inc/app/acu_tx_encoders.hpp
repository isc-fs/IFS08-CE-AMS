// SPDX-License-Identifier: proprietary
//
// ECU-side TX matrix encoders. As of Phase-2b of the code-first DSL
// these are thin adapters: the byte layout for every 0x020/0x12C/0x131..
// 0x137 frame lives in Core/Inc/can/messages/acu_*.def -- shared with
// the DBC descriptors -- and these wrappers just map service fields
// into the generated `ifs08::ACU_*_t` struct and call the generated
// `ifs08::encode_ACU_*`. Value transforms (Run|Charge -> bit, mA -> dA,
// DCDC temp stub) stay at the adapter.
//
// Hand-rolled-byte parity with the previous implementation is locked
// by tests/unit/test_acu_tx_encoders.cpp; the adapter mapping by
// tests/unit/test_dsl_parity.cpp.

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"

#include "can/can_codecs.hpp"   // code-first DSL encoders

#include <array>
#include <cstdint>

namespace ams::acu_tx {

// Saturating int16 clamp.
[[nodiscard]] inline std::int16_t sat_i16(std::int32_t v) noexcept {
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return static_cast<std::int16_t>(v);
}

// Pack current convention: + = discharge. 1 dA = 100 mA. Rounds to
// nearest, then saturates to int16.
[[nodiscard]] inline std::int16_t mA_to_deciamps_i16(std::int32_t mA) noexcept {
    const std::int32_t dA = (mA >= 0) ? (mA + 50) / 100
                                      : (mA - 50) / 100;
    return sat_i16(dA);
}

namespace detail {
// Convert a sized C array (the DSL encoder's output buffer) into a
// std::array of the same size. The DLC is enforced at compile time by
// the encoder signature.
template <std::size_t N>
[[nodiscard]] inline std::array<std::uint8_t, N>
to_array(const std::uint8_t (&buf)[N]) noexcept {
    std::array<std::uint8_t, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = buf[i];
    return out;
}
}  // namespace detail

// ----- Frame builders ---------------------------------------------------

// 0x020 ok_precharge — 1 byte, 1 iff FSM in Run|Charge.
[[nodiscard]] inline std::array<std::uint8_t, 1>
encode_ok_precharge(std::uint8_t fsm_state) noexcept {
    ifs08::ACU_ok_precharge_t in{};
    in.ok_precharge = static_cast<std::uint8_t>(
        (fsm_state == 3u /*Run*/ || fsm_state == 4u /*Charge*/) ? 1u : 0u);
    std::uint8_t buf[1] = {0};
    ifs08::encode_ACU_ok_precharge(in, buf);
    return detail::to_array(buf);
}

// 0x130 SoC % — Coulomb counting anchored on the VTC6 OCV curve
// (soc_estimator.hpp), produced by CurrentSensorTask.
//
// 0xFF (ams::soc::Unknown) is the "no trustworthy estimate" sentinel,
// NOT a reading: it appears before the first OCV anchor and whenever
// the current sensor faults or goes stale. Consumers must render it as
// "unknown" rather than clamping to 100 %.
//
// TELEMETRY ONLY. Nothing in the safety path produces or consumes this
// value, and no fault predicate reads it.
[[nodiscard]] inline std::array<std::uint8_t, 1>
encode_soc(std::uint8_t soc_percent) noexcept {
    ifs08::ACU_soc_t in{};
    in.soc_percent = soc_percent;
    std::uint8_t buf[1] = {0};
    ifs08::encode_ACU_soc(in, buf);
    return detail::to_array(buf);
}

// Retained so the existing contract test keeps asserting the sentinel
// value independently of the estimator.
[[nodiscard]] inline std::array<std::uint8_t, 1>
encode_soc_stub() noexcept {
    return encode_soc(0xFFu);
}

// 0x12C v_cell_min — BE u16 mV (pack-wide cell min).
[[nodiscard]] inline std::array<std::uint8_t, 2>
encode_min_voltage(const BmsState& bms) noexcept {
    ifs08::ACU_v_cell_min_t in{};
    in.min_cell_mV = bms.min_cell_mV;
    std::uint8_t buf[2] = {0};
    ifs08::encode_ACU_v_cell_min(in, buf);
    return detail::to_array(buf);
}

// 0x131 vmin modules 0..2.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_vmin_module_a(const BmsState& bms) noexcept {
    ifs08::ACU_vmin_module_a_t in{};
    in.vmin_module0 = bms.vmin_module[0];
    in.vmin_module1 = bms.vmin_module[1];
    in.vmin_module2 = bms.vmin_module[2];
    std::uint8_t buf[6] = {0};
    ifs08::encode_ACU_vmin_module_a(in, buf);
    return detail::to_array(buf);
}

// 0x132 vmin modules 3..4.
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_vmin_module_b(const BmsState& bms) noexcept {
    ifs08::ACU_vmin_module_b_t in{};
    in.vmin_module3 = bms.vmin_module[3];
    in.vmin_module4 = bms.vmin_module[4];
    std::uint8_t buf[4] = {0};
    ifs08::encode_ACU_vmin_module_b(in, buf);
    return detail::to_array(buf);
}

// 0x133 vmax modules 0..2.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_vmax_module_a(const BmsState& bms) noexcept {
    ifs08::ACU_vmax_module_a_t in{};
    in.vmax_module0 = bms.vmax_module[0];
    in.vmax_module1 = bms.vmax_module[1];
    in.vmax_module2 = bms.vmax_module[2];
    std::uint8_t buf[6] = {0};
    ifs08::encode_ACU_vmax_module_a(in, buf);
    return detail::to_array(buf);
}

// 0x134 vmax modules 3..4.
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_vmax_module_b(const BmsState& bms) noexcept {
    ifs08::ACU_vmax_module_b_t in{};
    in.vmax_module3 = bms.vmax_module[3];
    in.vmax_module4 = bms.vmax_module[4];
    std::uint8_t buf[4] = {0};
    ifs08::encode_ACU_vmax_module_b(in, buf);
    return detail::to_array(buf);
}

// 0x135 currents — BE i16 deciamps: [accu | dcdc].
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_currents(const CurrentState& cur) noexcept {
    ifs08::ACU_currents_t in{};
    in.current_accu_dA = mA_to_deciamps_i16(cur.filtered_mA);
    in.current_dcdc_dA = mA_to_deciamps_i16(cur.dcdc_filtered_mA);
    std::uint8_t buf[4] = {0};
    ifs08::encode_ACU_currents(in, buf);
    return detail::to_array(buf);
}

// 0x136 temp_max modules 0..2 — BE i16 degC x3.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_tmax_module_a(const BmsState& bms) noexcept {
    ifs08::ACU_tmax_module_a_t in{};
    in.tmax_module0 = bms.tmax_module[0];
    in.tmax_module1 = bms.tmax_module[1];
    in.tmax_module2 = bms.tmax_module[2];
    std::uint8_t buf[6] = {0};
    ifs08::encode_ACU_tmax_module_a(in, buf);
    return detail::to_array(buf);
}

// 0x137 temp_max modules 3..4 + temp_dcdc stub — BE i16 degC x3.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_tmax_module_b(const BmsState& bms) noexcept {
    ifs08::ACU_tmax_module_b_t in{};
    in.tmax_module3 = bms.tmax_module[3];
    in.tmax_module4 = bms.tmax_module[4];
    in.tmax_dcdc    = config::DcdcTempStubValue;
    std::uint8_t buf[6] = {0};
    ifs08::encode_ACU_tmax_module_b(in, buf);
    return detail::to_array(buf);
}

}  // namespace ams::acu_tx
