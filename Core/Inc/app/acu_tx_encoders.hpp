// SPDX-License-Identifier: proprietary
//
// Pure-logic encoders for the ECU-side TX matrix shipped in fix/53.
// Header-only so the host unit-test build can exercise them without
// pulling in HAL / FreeRTOS. Byte-by-byte layouts mirror the table in
// docs/CAN_MAP.md.

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"

#include <array>
#include <cstdint>

namespace ams::acu_tx {

// Big-endian writers.
inline void be_put_u16(std::uint8_t *p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[1] = static_cast<std::uint8_t>( v       & 0xFFu);
}

inline void be_put_i16(std::uint8_t *p, std::int16_t v) noexcept {
    be_put_u16(p, static_cast<std::uint16_t>(v));
}

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

// ----- Frame builders ---------------------------------------------------

// 0x020 ok_precharge — 1 byte, 1 iff FSM in Run|Charge.
[[nodiscard]] inline std::array<std::uint8_t, 1>
encode_ok_precharge(std::uint8_t fsm_state) noexcept {
    return { static_cast<std::uint8_t>(
        (fsm_state == 3u /*Run*/ || fsm_state == 4u /*Charge*/) ? 1u : 0u) };
}

// 0x12C v_cell_min — BE u16 mV (pack-wide cell min).
[[nodiscard]] inline std::array<std::uint8_t, 2>
encode_min_voltage(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 2> out{};
    be_put_u16(out.data(), bms.min_cell_mV);
    return out;
}

// 0x131 vmin modules 0..2 — BE u16 mV x3 = 6 bytes.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_vmin_module_a(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 6> out{};
    be_put_u16(out.data() + 0, bms.vmin_module[0]);
    be_put_u16(out.data() + 2, bms.vmin_module[1]);
    be_put_u16(out.data() + 4, bms.vmin_module[2]);
    return out;
}

// 0x132 vmin modules 3..4 — 4 bytes.
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_vmin_module_b(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 4> out{};
    be_put_u16(out.data() + 0, bms.vmin_module[3]);
    be_put_u16(out.data() + 2, bms.vmin_module[4]);
    return out;
}

// 0x133 vmax modules 0..2.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_vmax_module_a(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 6> out{};
    be_put_u16(out.data() + 0, bms.vmax_module[0]);
    be_put_u16(out.data() + 2, bms.vmax_module[1]);
    be_put_u16(out.data() + 4, bms.vmax_module[2]);
    return out;
}

// 0x134 vmax modules 3..4.
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_vmax_module_b(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 4> out{};
    be_put_u16(out.data() + 0, bms.vmax_module[3]);
    be_put_u16(out.data() + 2, bms.vmax_module[4]);
    return out;
}

// 0x135 currents — BE i16 deciamps: [accu | dcdc].
[[nodiscard]] inline std::array<std::uint8_t, 4>
encode_currents(const CurrentState& cur) noexcept {
    std::array<std::uint8_t, 4> out{};
    be_put_i16(out.data() + 0, mA_to_deciamps_i16(cur.filtered_mA));
    be_put_i16(out.data() + 2, mA_to_deciamps_i16(cur.dcdc_filtered_mA));
    return out;
}

// 0x136 temp_max modules 0..2 — BE i16 degC x3.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_tmax_module_a(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 6> out{};
    be_put_i16(out.data() + 0, bms.tmax_module[0]);
    be_put_i16(out.data() + 2, bms.tmax_module[1]);
    be_put_i16(out.data() + 4, bms.tmax_module[2]);
    return out;
}

// 0x137 temp_max modules 3..4 + temp_dcdc stub — BE i16 degC x3.
[[nodiscard]] inline std::array<std::uint8_t, 6>
encode_tmax_module_b(const BmsState& bms) noexcept {
    std::array<std::uint8_t, 6> out{};
    be_put_i16(out.data() + 0, bms.tmax_module[3]);
    be_put_i16(out.data() + 2, bms.tmax_module[4]);
    be_put_i16(out.data() + 4, config::kDcdcTempStubValue);
    return out;
}

}  // namespace ams::acu_tx
