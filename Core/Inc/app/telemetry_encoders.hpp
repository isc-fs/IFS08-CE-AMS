// SPDX-License-Identifier: proprietary
//
// Pure-logic encoders for the three AMS telemetry frames on FDCAN1.
// Header-only so the host unit-test build can exercise them without
// pulling in HAL / FreeRTOS.
//
// Frame definitions, byte by byte, also documented in docs/CAN_MAP.md
// (the canonical reference for any external receiver).

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "vehicle_service.hpp"

#include "can/can_codecs.hpp"   // code-first DSL encoders (single source of layout)

#include <array>
#include <cstdint>

namespace ams::telemetry {

using Frame = std::array<std::uint8_t, 8>;

// These encoders are now thin adapters over the code-first DSL: they map
// service fields into the generated `ifs08::*_t` struct and call the
// generated `ifs08::encode_*`. The byte layout lives in exactly one
// place -- Core/Inc/can/messages/*.def -- shared with the DBC
// descriptors. Byte-for-byte behaviour is locked by the hardcoded-byte
// tests in tests/unit/test_telemetry_encoders.cpp; the adapter mapping
// by tests/unit/test_dsl_parity.cpp. Value transforms (int8 temp clip,
// AMS_OK normalise, cockpit-byte assembly) stay here, at the adapter.
namespace detail {
[[nodiscard]] inline Frame to_frame(const std::uint8_t (&b)[8]) noexcept {
    Frame f{};
    for (std::uint8_t i = 0; i < 8; ++i) f[i] = b[i];
    return f;
}
}  // namespace detail

// ---------------------------------------------------------------------------
// 0x4A0  "AMS status" -- layout in Core/Inc/can/messages/ams_status.def
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_status(std::uint8_t       state,
                                         std::uint8_t       ams_ok,
                                         const BmsState&    bms) noexcept {
    ifs08::AMS_status_t in{};
    in.fsm_state          = state;
    in.ams_ok             = ams_ok ? 1u : 0u;   // normalise any non-zero -> 1
    in.module_online_mask = bms.module_online_mask;
    in.reserved_b3        = 0;
    in.min_cell_mV        = bms.min_cell_mV;
    in.max_cell_mV        = bms.max_cell_mV;
    std::uint8_t b[8];
    ifs08::encode_AMS_status(in, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// 0x4A1  "AMS pack" -- layout in Core/Inc/can/messages/ams_pack.def
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_pack(const BmsState&     bms,
                                       const CurrentState& cur) noexcept {
    ifs08::AMS_pack_t in{};
    in.pack_voltage_mV = bms.pack_voltage_mV;
    in.filtered_mA     = cur.filtered_mA;
    std::uint8_t b[8];
    ifs08::encode_AMS_pack(in, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// 0x4A2  "AMS temps + diagnostics" -- layout in
// Core/Inc/can/messages/ams_temps.def. The int8 temp clip + the cockpit
// byte (#246) are value transforms that stay here at the adapter; the
// .def owns only the byte placement.
//
// Cockpit byte (byte 5): bit7 sentinel, bits3:2 mode_locked, bit1 TSMS,
// bit0 DASH_CHG -- assembled by the caller in safety_task.cpp.
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::int8_t clip_int8(std::int16_t v) noexcept {
    if (v >  127) return  127;
    if (v < -128) return -128;
    return static_cast<std::int8_t>(v);
}

[[nodiscard]] inline Frame encode_temps(const BmsState&     bms,
                                        const VehicleState& veh,
                                        std::uint8_t        heartbeat,
                                        std::uint8_t        tx_fail_count_lo,
                                        std::uint8_t        tsms_dash_chg_byte) noexcept {
    ifs08::AMS_temps_t in{};
    in.min_tempC          = clip_int8(bms.min_tempC);
    in.max_tempC          = clip_int8(bms.max_tempC);
    in.avg_tempC          = clip_int8(bms.avg_tempC);
    in.dc_bus_V           = veh.dc_bus_V;
    in.tsms_dash_chg_byte = tsms_dash_chg_byte;
    in.tx_fail_count_lo   = tx_fail_count_lo;
    in.heartbeat          = heartbeat;
    std::uint8_t b[8];
    ifs08::encode_AMS_temps(in, b);
    return detail::to_frame(b);
}

}  // namespace ams::telemetry
