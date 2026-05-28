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
//   byte 6  fault_reason  the predicate branch that latched ERROR
//                         (ams::safety::FaultReason; 12=FSM-driven
//                         Error path; 0=not latched) (#276)
//   byte 7  fault_detail  BmsStale: module index; BmsModuleOffline:
//                         live module_online_mask; else 0
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_fsm_status(std::uint8_t  fsm_state,
                                             fsm::Mode     mode_locked,
                                             bool          tsms,
                                             bool          dash_chg,
                                             std::uint8_t  ams_ok_gpio,
                                             std::uint32_t pec_err_total,
                                             std::uint8_t  fault_reason = 0u,
                                             std::uint8_t  fault_detail = 0u) noexcept {
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
    f[6] = fault_reason;
    f[7] = fault_detail;
    return f;
}

// ---------------------------------------------------------------------------
// Poll-timing frame (0x6C1). The pit tool can graph these to spot SPI
// jitter / mux degradation directly off the wire.
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

// ---------------------------------------------------------------------------
// Balance state -- two frames carrying the full 95-bit DCC mask plus
// the per-cycle counters. dcc_bits[m] mirrors the wire-side encoding:
// bit c == 1 iff cell c of module m was selected for discharge in the
// last balance window.
//
//   0x6C2 [byte i] = packed mask bits 8*i..8*i+7, where bit b of byte i
//                    is cell (8*i + b) of the row-major flat (cell_idx =
//                    19*m + c). Covers cells 0..63.
//   0x6C3 [byte 0..3] = mask bits 64..94 (low 31 bits of byte 4..7's
//                       concatenation; bit 31 always 0 / reserved).
//          [byte 4..5] = balance_cycles_total LE u16 (mod 65536)
//          [byte 6..7] = balance_cycles_active LE u16 (mod 65536)
//
// The pit tool reconstructs by walking 95 bits and mapping bit b ->
// (module = b / 19, cell = b % 19).
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_balance_mask_a(const volatile std::uint32_t (&dcc_bits)[config::BmsModuleCount]) noexcept {
    Frame f = {};
    for (std::uint8_t cell_idx = 0; cell_idx < 64 && cell_idx < 5 * 19; ++cell_idx) {
        const std::uint8_t m = cell_idx / config::CellsPerModule;
        const std::uint8_t c = cell_idx % config::CellsPerModule;
        if (dcc_bits[m] & (1u << c)) {
            f[cell_idx / 8u] = static_cast<std::uint8_t>(f[cell_idx / 8u] | (1u << (cell_idx % 8u)));
        }
    }
    return f;
}

[[nodiscard]] inline Frame encode_balance_mask_b(const volatile std::uint32_t (&dcc_bits)[config::BmsModuleCount],
                                                 std::uint32_t cycles_total,
                                                 std::uint32_t cycles_active) noexcept {
    Frame f = {};
    // Cells 64..94 -> bytes 0..3 (low 31 bits of a 32-bit field).
    for (std::uint8_t cell_idx = 64; cell_idx < 5 * 19; ++cell_idx) {
        const std::uint8_t m = cell_idx / config::CellsPerModule;
        const std::uint8_t c = cell_idx % config::CellsPerModule;
        const std::uint8_t bit_pos = cell_idx - 64u;   // 0..30
        if (dcc_bits[m] & (1u << c)) {
            f[bit_pos / 8u] = static_cast<std::uint8_t>(f[bit_pos / 8u] | (1u << (bit_pos % 8u)));
        }
    }
    const std::uint16_t ct = (cycles_total  > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(cycles_total);
    const std::uint16_t ca = (cycles_active > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(cycles_active);
    f[4] = static_cast<std::uint8_t>(ct & 0xFFu);
    f[5] = static_cast<std::uint8_t>((ct >> 8) & 0xFFu);
    f[6] = static_cast<std::uint8_t>(ca & 0xFFu);
    f[7] = static_cast<std::uint8_t>((ca >> 8) & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// Boot diag -- "why did we boot, how did init go". Lets the pit tool
// distinguish a clean cold boot from a watchdog reset or a CAN-trigger
// BL jump, directly off the wire.
//
//   bytes 0..3  jump_reason  LE u32 (RTC->BKP2R contents at boot;
//                            matches config::JumpReason enum -- see
//                            ams_config.hpp). 0 = no jump reason
//                            recorded (clean cold POR).
//   byte 4      g_app_init_progress (0..7 milestone counter)
//   bytes 5..7  g_fdcan1_start_result LE u24 (low 24 bits of HAL status;
//                                            0 = HAL_OK, !=0 = failure)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_boot_diag(std::uint32_t jump_reason,
                                            std::uint8_t  app_init_progress,
                                            std::uint32_t fdcan1_start_result) noexcept {
    Frame f = {};
    f[0] = static_cast<std::uint8_t>(jump_reason         & 0xFFu);
    f[1] = static_cast<std::uint8_t>((jump_reason >>  8) & 0xFFu);
    f[2] = static_cast<std::uint8_t>((jump_reason >> 16) & 0xFFu);
    f[3] = static_cast<std::uint8_t>((jump_reason >> 24) & 0xFFu);
    f[4] = app_init_progress;
    f[5] = static_cast<std::uint8_t>(fdcan1_start_result        & 0xFFu);
    f[6] = static_cast<std::uint8_t>((fdcan1_start_result >>  8) & 0xFFu);
    f[7] = static_cast<std::uint8_t>((fdcan1_start_result >> 16) & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// Crash post-mortem -- surfaces the in-RAM trail left by the FreeRTOS
// stack-overflow + malloc-failed hooks (see freertos.c). If a previous
// session crashed, the engineer reads this frame off the CAN bus and
// learns what happened; on a clean session every byte stays at 0.
//
//   byte 0      stack_overflow_seen  (0 if g_stack_overflow_task_addr
//                                     is still 0, else 1)
//   byte 1      stack_overflow_watermark low byte (saturates at 0xFF;
//                                                  0xFF on the "API
//                                                  call itself failed"
//                                                  sentinel)
//   bytes 2..5  stack_overflow_task_addr LE u32 (the failing task's
//                                                xTaskHandle value)
//   bytes 6..7  malloc_failed_count LE u16 (saturates at 0xFFFF)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_post_mortem(std::uint32_t stack_overflow_task_addr,
                                              std::uint32_t stack_overflow_watermark,
                                              std::uint32_t malloc_failed_count) noexcept {
    Frame f = {};
    f[0] = (stack_overflow_task_addr != 0u) ? 1u : 0u;
    f[1] = (stack_overflow_watermark > 0xFFu)
               ? 0xFFu
               : static_cast<std::uint8_t>(stack_overflow_watermark);
    f[2] = static_cast<std::uint8_t>(stack_overflow_task_addr        & 0xFFu);
    f[3] = static_cast<std::uint8_t>((stack_overflow_task_addr >>  8) & 0xFFu);
    f[4] = static_cast<std::uint8_t>((stack_overflow_task_addr >> 16) & 0xFFu);
    f[5] = static_cast<std::uint8_t>((stack_overflow_task_addr >> 24) & 0xFFu);
    const std::uint16_t mfc = (malloc_failed_count > 0xFFFFu)
                                  ? 0xFFFFu
                                  : static_cast<std::uint16_t>(malloc_failed_count);
    f[6] = static_cast<std::uint8_t>(mfc & 0xFFu);
    f[7] = static_cast<std::uint8_t>((mfc >> 8) & 0xFFu);
    return f;
}

// ---------------------------------------------------------------------------
// Firmware identification frame. Lets the pit tool answer "what's
// flashed?" off the CAN bus -- no need to crack the image open to
// read __firmware_info.
//
//   byte 0      fw_version_major
//   byte 1      fw_version_minor
//   byte 2      fw_version_patch
//   bytes 3..6  git_hash[0..3] (first 4 bytes of the 8-byte hash)
//   byte 7      bl_node_id  (firmware_info.reserved[0])
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_fw_id(std::uint8_t        major,
                                        std::uint8_t        minor,
                                        std::uint8_t        patch,
                                        const std::uint8_t* git_hash_4,
                                        std::uint8_t        bl_node_id) noexcept {
    Frame f = {};
    f[0] = major;
    f[1] = minor;
    f[2] = patch;
    f[3] = git_hash_4[0];
    f[4] = git_hash_4[1];
    f[5] = git_hash_4[2];
    f[6] = git_hash_4[3];
    f[7] = bl_node_id;
    return f;
}

// ---------------------------------------------------------------------------
// Per-IC PEC error counts (#258). 0x6C0[4..5] already carries the sum,
// which is enough to say "the chain is unhealthy" but not "which IC is
// the problem". The 10 ICs (LtcChainLength) split across two frames as
// saturating uint8 -- "error count > 255" already means catastrophic
// chain failure for diagnostic purposes; the high bits aren't carrying
// useful information past that. Reset only on cold boot (counts live
// in bms_service.cpp's extern array; no per-session clear).
//
//   0x6C7  PitDiag_pec_per_ic_a: 8 bytes = ICs 0..7
//   0x6C8  PitDiag_pec_per_ic_b: bytes 0..1 = ICs 8..9, bytes 2..7 = 0
//
// Chain-to-module mapping:
//   IC index 0 = module 0 upper (cells 0..9)
//   IC index 1 = module 0 lower (cells 10..18)
//   IC index 2 = module 1 upper, etc.
// So 0x6C7 byte 0 spike = "module 0's top LTC6811 is misbehaving".
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::uint8_t sat_u8(std::uint32_t v) noexcept {
    return (v > 0xFFu) ? 0xFFu : static_cast<std::uint8_t>(v);
}

[[nodiscard]] inline Frame encode_pec_err_count_a(
    const volatile std::uint32_t (&counts)[config::LtcChainLength]) noexcept {
    Frame f = {};
    constexpr std::uint8_t frame_a_lim =
        (config::LtcChainLength < 8u) ? config::LtcChainLength : 8u;
    for (std::uint8_t i = 0; i < frame_a_lim; ++i) {
        f[i] = sat_u8(counts[i]);
    }
    return f;
}

[[nodiscard]] inline Frame encode_pec_err_count_b(
    const volatile std::uint32_t (&counts)[config::LtcChainLength]) noexcept {
    Frame f = {};
    for (std::uint8_t i = 8; i < config::LtcChainLength; ++i) {
        f[i - 8u] = sat_u8(counts[i]);
    }
    return f;
}

}  // namespace ams::pit_diag
