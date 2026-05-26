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

#include <array>
#include <cstdint>

namespace ams::telemetry {

using Frame = std::array<std::uint8_t, 8>;

// ---------------------------------------------------------------------------
// 0x4A0  "AMS status" (cadence 500 ms)
//
//   byte 0   FSM state (0..5 per ams::fsm::State enum)
//   byte 1   AMS_OK GPIO read-back (0 / 1)
//   byte 2   module_online_mask (low byte of bms.module_online_mask)
//   byte 3   reserved (0)
//   bytes 4..5  min_cell_mV  big-endian uint16
//   bytes 6..7  max_cell_mV  big-endian uint16
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_status(std::uint8_t       state,
                                         std::uint8_t       ams_ok,
                                         const BmsState&    bms) noexcept {
    Frame f = {};
    f[0] = state;
    f[1] = ams_ok ? 1u : 0u;
    f[2] = bms.module_online_mask;
    f[3] = 0;  // reserved
    f[4] = static_cast<std::uint8_t>(bms.min_cell_mV >> 8);
    f[5] = static_cast<std::uint8_t>(bms.min_cell_mV & 0xFFu);
    f[6] = static_cast<std::uint8_t>(bms.max_cell_mV >> 8);
    f[7] = static_cast<std::uint8_t>(bms.max_cell_mV & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// 0x4A1  "AMS pack" (cadence 500 ms)
//
//   bytes 0..3  pack_voltage_mV  little-endian uint32  (sum-of-cells, mV)
//   bytes 4..7  filtered_mA      little-endian int32   (+ discharge, - charge)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_pack(const BmsState&     bms,
                                       const CurrentState& cur) noexcept {
    Frame f = {};
    const std::uint32_t pv = bms.pack_voltage_mV;
    f[0] = static_cast<std::uint8_t>(pv & 0xFFu);
    f[1] = static_cast<std::uint8_t>((pv >>  8) & 0xFFu);
    f[2] = static_cast<std::uint8_t>((pv >> 16) & 0xFFu);
    f[3] = static_cast<std::uint8_t>((pv >> 24) & 0xFFu);

    const std::uint32_t mA = static_cast<std::uint32_t>(cur.filtered_mA);
    f[4] = static_cast<std::uint8_t>(mA & 0xFFu);
    f[5] = static_cast<std::uint8_t>((mA >>  8) & 0xFFu);
    f[6] = static_cast<std::uint8_t>((mA >> 16) & 0xFFu);
    f[7] = static_cast<std::uint8_t>((mA >> 24) & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// 0x4A2  "AMS temps + diagnostics" (cadence 500 ms)
//
//   byte 0   min_tempC  signed int8 (clipped from BmsState int16)
//   byte 1   max_tempC  signed int8
//   byte 2   avg_tempC  signed int8
//   bytes 3..4  dc_bus_V  little-endian uint16 (volts)
//   byte 5   tsms_dash_chg_byte (cockpit-input snapshot, always-on)
//   byte 6   tx_fail_count_lo  low byte of g_telemetry_tx_fail
//   byte 7   heartbeat counter (caller supplies; wraps at 255)
//
// Cockpit byte encoding:
//   bit 7    1 (sentinel; lets the dashboard distinguish "live byte"
//             from "byte got eaten by older firmware or compiler")
//   bits 3:2 mode_locked (00=Undecided, 01=Car, 10=Charger)
//   bit 1    TSMS readback (PF9)
//   bit 0    DASH_CHG readback (PF10)
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
    Frame f = {};
    f[0] = static_cast<std::uint8_t>(clip_int8(bms.min_tempC));
    f[1] = static_cast<std::uint8_t>(clip_int8(bms.max_tempC));
    f[2] = static_cast<std::uint8_t>(clip_int8(bms.avg_tempC));
    f[3] = static_cast<std::uint8_t>(veh.dc_bus_V & 0xFFu);
    f[4] = static_cast<std::uint8_t>((veh.dc_bus_V >> 8) & 0xFFu);
    f[5] = tsms_dash_chg_byte;
    f[6] = tx_fail_count_lo;
    f[7] = heartbeat;
    return f;
}

}  // namespace ams::telemetry
