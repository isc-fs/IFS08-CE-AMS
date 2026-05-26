// SPDX-License-Identifier: proprietary
//
// Pure-logic encoders for the runtime-toggleable pit-side diagnostic
// stream (#247). Header-only so the host unit-test build exercises the
// byte layouts without HAL / FreeRTOS.
//
// Frames are dispatched by AcuCanTask when the runtime flag is set;
// the flag transitions are owned there too. This file only owns wire
// format. See ams_config.hpp for the ID range and enable contract.

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "state_machine.hpp"

#include <array>
#include <cstdint>

namespace ams::pit_diag {

using Frame = std::array<std::uint8_t, 8>;

// ---------------------------------------------------------------------------
// Cell-voltage frame. Each frame carries 4 cells (BE u16 mV) of the
// flattened cell_mV[5][19] grid. frame_idx in [0, PitDiagCellFrames=24);
// out-of-range indices return a zero-filled frame (caller bug). The
// last frame (idx=23) only has 3 real cells (cell_indices 92..94); the
// remaining 2 bytes are PitDiagCellSentinel (0xFFFF).
//
// Decoder side: cell_index = 4 * frame_idx + slot;
//               module = cell_index / config::CellsPerModule (= 19),
//               cell   = cell_index % config::CellsPerModule.
// Any byte-pair == 0xFFFF means "no cell at this slot" -- skip.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_cell_frame(const BmsState& bms,
                                             std::uint8_t    frame_idx) noexcept {
    Frame f = {};
    if (frame_idx >= config::PitDiagCellFrames) return f;

    constexpr std::uint16_t TotalCells =
        static_cast<std::uint16_t>(config::BmsModuleCount) *
        static_cast<std::uint16_t>(config::CellsPerModule);

    for (std::uint8_t slot = 0; slot < 4; ++slot) {
        const std::uint16_t cell_index =
            static_cast<std::uint16_t>(4u * frame_idx + slot);
        std::uint16_t v_mV;
        if (cell_index < TotalCells) {
            const std::uint8_t module = static_cast<std::uint8_t>(cell_index / config::CellsPerModule);
            const std::uint8_t cell   = static_cast<std::uint8_t>(cell_index % config::CellsPerModule);
            v_mV = bms.cell_mV[module][cell];
        } else {
            v_mV = config::PitDiagCellSentinel;
        }
        f[slot * 2u]      = static_cast<std::uint8_t>((v_mV >> 8) & 0xFFu);
        f[slot * 2u + 1u] = static_cast<std::uint8_t>(v_mV & 0xFFu);
    }
    return f;
}

// ---------------------------------------------------------------------------
// NTC-temp frame. Each frame carries 8 NTC samples (i8 degC) of the
// flattened cell_tempC[5][40] grid. frame_idx in [0, PitDiagTempFrames=25).
// 200 NTCs / 8 = 25 exactly, no padding.
//
// Decoder side: temp_index = 8 * frame_idx + slot;
//               module = temp_index / config::TempsPerModule (= 40),
//               temp   = temp_index % config::TempsPerModule.
// Values clipped to int8 range (BmsState stores int16 internally for
// out-of-range sentinels). Saturated clip matches the 0x4A2 telemetry
// behaviour from telemetry_encoders.hpp -- consistent across IDs.
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::int8_t clip_int8_pit(std::int16_t v) noexcept {
    if (v >  127) return  127;
    if (v < -128) return -128;
    return static_cast<std::int8_t>(v);
}

[[nodiscard]] inline Frame encode_temp_frame(const BmsState& bms,
                                             std::uint8_t    frame_idx) noexcept {
    Frame f = {};
    if (frame_idx >= config::PitDiagTempFrames) return f;

    for (std::uint8_t slot = 0; slot < 8; ++slot) {
        const std::uint16_t temp_index =
            static_cast<std::uint16_t>(8u * frame_idx + slot);
        const std::uint8_t module = static_cast<std::uint8_t>(temp_index / config::TempsPerModule);
        const std::uint8_t temp   = static_cast<std::uint8_t>(temp_index % config::TempsPerModule);
        f[slot] = static_cast<std::uint8_t>(clip_int8_pit(bms.cell_tempC[module][temp]));
    }
    return f;
}

// ---------------------------------------------------------------------------
// FSM extended status frame (0x6C0). Lives alongside 0x4A0 but carries
// the diag fields that are HIL-only in the flight 0x4A2 layout: full
// FSM state byte, mode_locked, raw cockpit input readbacks, AMS_OK
// GPIO, and a sum of the per-IC PEC error counters as a compact "BMS
// chain health" rollup.
//
//   byte 0  fsm_state  (same encoding as 0x4A0[0]: 0..5)
//   byte 1  mode_locked (Undecided=0 / Car=1 / Charger=2)
//   byte 2  cockpit inputs: bit1=TSMS readback, bit0=DASH_CHG readback
//   byte 3  ams_ok_gpio (0/1)
//   bytes 4..5  pec_err_total  BE u16 (saturates at 0xFFFF if all 10
//                              ICs combined exceed 65 535)
//   bytes 6..7  reserved (0)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_fsm_status(std::uint8_t  fsm_state,
                                             fsm::Mode     mode_locked,
                                             bool          tsms,
                                             bool          dash_chg,
                                             std::uint8_t  ams_ok_gpio,
                                             std::uint32_t pec_err_total) noexcept {
    Frame f = {};
    f[0] = fsm_state;
    f[1] = static_cast<std::uint8_t>(mode_locked);
    f[2] = static_cast<std::uint8_t>((tsms ? 0x02u : 0u) | (dash_chg ? 0x01u : 0u));
    f[3] = ams_ok_gpio ? 1u : 0u;
    const std::uint16_t pec16 = (pec_err_total > 0xFFFFu)
                                    ? 0xFFFFu
                                    : static_cast<std::uint16_t>(pec_err_total);
    f[4] = static_cast<std::uint8_t>((pec16 >> 8) & 0xFFu);
    f[5] = static_cast<std::uint8_t>(pec16 & 0xFFu);
    f[6] = 0u;
    f[7] = 0u;
    return f;
}

// ---------------------------------------------------------------------------
// Poll-timing frame (0x6C1). The pit tool can graph these to spot SPI
// jitter / mux degradation without SWD.
//
//   bytes 0..1  bms_volt_poll_ms       BE u16 (last cycle ms; clip at 0xFFFF)
//   bytes 2..3  bms_volt_poll_max_ms   BE u16 (worst-case since boot; clip)
//   bytes 4..7  temp_sweep_last_mask   LE u32 (NTC channel failure bitset
//                                              from the last sweep; 1 bit
//                                              per channel that the LTC
//                                              chain didn't return)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_timing(std::uint32_t bms_volt_poll_ms,
                                         std::uint32_t bms_volt_poll_max_ms,
                                         std::uint32_t temp_sweep_last_mask) noexcept {
    Frame f = {};
    const auto clip16 = [](std::uint32_t v) -> std::uint16_t {
        return (v > 0xFFFFu) ? static_cast<std::uint16_t>(0xFFFFu)
                             : static_cast<std::uint16_t>(v);
    };
    const std::uint16_t poll = clip16(bms_volt_poll_ms);
    const std::uint16_t max  = clip16(bms_volt_poll_max_ms);
    f[0] = static_cast<std::uint8_t>((poll >> 8) & 0xFFu);
    f[1] = static_cast<std::uint8_t>(poll & 0xFFu);
    f[2] = static_cast<std::uint8_t>((max  >> 8) & 0xFFu);
    f[3] = static_cast<std::uint8_t>(max  & 0xFFu);
    f[4] = static_cast<std::uint8_t>(temp_sweep_last_mask        & 0xFFu);
    f[5] = static_cast<std::uint8_t>((temp_sweep_last_mask >>  8) & 0xFFu);
    f[6] = static_cast<std::uint8_t>((temp_sweep_last_mask >> 16) & 0xFFu);
    f[7] = static_cast<std::uint8_t>((temp_sweep_last_mask >> 24) & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// Command-frame decoder. Returns:
//    +1  caller should ENABLE the diag stream  (DEADBEEF)
//    -1  caller should DISABLE the diag stream (0x00000000)
//     0  not a pit-diag command (caller continues normal dispatch)
//
// The caller is responsible for checking f.bus / f.id / f.dlc *before*
// calling this, but the function is defensive against a bad id/dlc.
// ---------------------------------------------------------------------------
[[nodiscard]] inline int classify_command(const CanFrame& f) noexcept {
    if (f.id  != config::PitDiagCmdRxId)  return 0;
    if (f.dlc != config::PitDiagCmdDlc)   return 0;
    bool enable  = true;
    bool disable = true;
    for (std::uint8_t i = 0; i < config::PitDiagCmdDlc; ++i) {
        if (f.data[i] != config::PitDiagEnableMagic[i])  enable  = false;
        if (f.data[i] != config::PitDiagDisableMagic[i]) disable = false;
    }
    if (enable)  return 1;
    if (disable) return -1;
    return 0;
}

}  // namespace ams::pit_diag
