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
    f[3] = 0;
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
//   bytes 5..6  reserved (0)
//   byte 7   heartbeat counter (caller supplies; wraps at 255)
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::int8_t clip_int8(std::int16_t v) noexcept {
    if (v >  127) return  127;
    if (v < -128) return -128;
    return static_cast<std::int8_t>(v);
}

[[nodiscard]] inline Frame encode_temps(const BmsState&     bms,
                                        const VehicleState& veh,
                                        std::uint8_t        heartbeat) noexcept {
    Frame f = {};
    f[0] = static_cast<std::uint8_t>(clip_int8(bms.min_tempC));
    f[1] = static_cast<std::uint8_t>(clip_int8(bms.max_tempC));
    f[2] = static_cast<std::uint8_t>(clip_int8(bms.avg_tempC));
    f[3] = static_cast<std::uint8_t>(veh.dc_bus_V & 0xFFu);
    f[4] = static_cast<std::uint8_t>((veh.dc_bus_V >> 8) & 0xFFu);
    f[5] = 0;
    f[6] = 0;
    f[7] = heartbeat;
    return f;
}

// ---------------------------------------------------------------------------
// 0x4A3  "AMS diag" (cadence 500 ms, alongside the other three frames)
//
//   byte 0   bms_poll_task_state
//              0xA0..0xA4 with sentinel high-nibble 0xA -> osThreadGetState
//                low nibble (1 Ready, 2 Running, 3 Blocked, 4 Terminated,
//                0 means CMSIS quirk)
//              0xAF                                     -> osThreadError
//              0xFF                                     -> BmsPollTaskHandle == NULL
//              0x00                                     -> something has eliminated
//                                                          this byte (shouldn't
//                                                          happen post #142)
//   byte 1   bms_seed_count_lo  (low byte of g_bms_seed_count)
//   bytes 2..3  free_heap_bytes  big-endian uint16, free heap in bytes
//                                (saturates at 0xFFFF for > 64 KB)
//   bytes 4..7  reserved (0) -- future probes
//
// Rationale: the original in-place patches on 0x4A0[3] / 0x4A2[5..6] kept
// getting elided by -O3 (PRs #132, #134, #136, #138, #140 all lost to
// either dead-store elimination or fold-back-to-encoder). A dedicated
// encoder writes the diagnostic values into the frame from the start,
// not as a patch on top of an existing constant -- nothing to elide.
// Bench-only frame; remove once #123 closes.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_diag(std::uint8_t  bms_poll_task_state,
                                       std::uint8_t  bms_seed_count_lo,
                                       std::uint32_t free_heap_bytes) noexcept {
    Frame f = {};
    f[0] = bms_poll_task_state;
    f[1] = bms_seed_count_lo;
    const std::uint16_t fh = (free_heap_bytes > 0xFFFFu)
                                 ? 0xFFFFu
                                 : static_cast<std::uint16_t>(free_heap_bytes);
    f[2] = static_cast<std::uint8_t>(fh >> 8);
    f[3] = static_cast<std::uint8_t>(fh & 0xFFu);
    f[4] = 0;
    f[5] = 0;
    f[6] = 0;
    f[7] = 0;
    return f;
}

}  // namespace ams::telemetry
