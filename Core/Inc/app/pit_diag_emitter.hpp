// SPDX-License-Identifier: proprietary
//
// Pure-logic encoders for the runtime-toggleable pit-side diagnostic
// stream. Header-only so the host unit-test build exercises the
// byte layouts without HAL / FreeRTOS.
//
// Frames are dispatched by AcuCanTask when the runtime flag is set;
// the flag transitions are owned there too. This file only owns wire
// format. See ams_config.hpp for the ID range and enable contract.

#pragma once

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "state_machine.hpp"

#include "can/can_codecs.hpp"   // code-first DSL encoders (single layout source)

#include <array>
#include <cstdint>

namespace ams::pit_diag {

using Frame = std::array<std::uint8_t, 8>;

// The fixed-layout frames (0x6C0..0x6C8) are now thin adapters over the
// generated ifs08::encode_PIT_* -- the byte layout lives once in
// Core/Inc/can/messages/pit_*.def. The parameterised cell/temp grid
// frames (encode_cell_frame / encode_temp_frame) stay hand-rolled until
// the DSL grows a multiplexed-array representation. Value
// transforms (saturation, bit assembly) stay here at the adapter; the
// hardcoded-byte tests in test_pit_diag_emitter.cpp are the parity gate.
namespace detail {
[[nodiscard]] inline Frame to_frame(const std::uint8_t (&b)[8]) noexcept {
    Frame f{};
    for (std::uint8_t i = 0; i < 8; ++i) f[i] = b[i];
    return f;
}
}  // namespace detail

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
    const auto& A = ifs08::ALL_ARRAYS[0];   // 0x680 cell grid
    if (frame_idx >= A.frame_count) return Frame{};
    std::uint8_t b[8];
    can_dsl::encode_array_window(
        frame_idx, A.per_frame, A.elem_bits, A.big_endian, A.total_elems,
        config::PitDiagCellSentinel,
        [&](unsigned flat) -> std::uint64_t {
            const std::uint8_t m = static_cast<std::uint8_t>(flat / config::CellsPerModule);
            const std::uint8_t c = static_cast<std::uint8_t>(flat % config::CellsPerModule);
            // A module past its freshness window (isoSPI silent) keeps its last
            // good voltages in cell_mV. Emit the no-cell sentinel instead so a
            // disconnected chain reads as "no data", never a frozen value.
            if ((bms.module_online_mask & (1u << m)) == 0u) {
                return config::PitDiagCellSentinel;
            }
            // Cells straddling an ADOW-confirmed open tap: the shared node is
            // floating, so BOTH readings are displaced (one high, one low) and
            // neither is recoverable -- only their sum survives. Emit "no data"
            // rather than a number that looks like a measurement and is not.
            // cell_mV keeps the raw split; recompute_summaries_ still uses it for
            // the pair average, and the raw values remain visible on the ADOW
            // diagnostic grid (0x6D0/0x6E8) when config::AdowRawDiag is on.
            if ((bms.cell_open_cells[m] & (1u << c)) != 0u) {
                return config::PitDiagCellSentinel;
            }
            return bms.cell_mV[m][c];
        },
        b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// BENCH DIAGNOSTIC: dump any flat 95-cell uint16 mV grid with the SAME window
// layout as the 0x680 cell grid, so one decoder reads every grid the firmware
// publishes. Used for the raw ADOW pull-up / pull-down readings
// (AdowDiagPuBaseId / AdowDiagPdBaseId) and the second-mode sweep
// (AdcXCheckBaseId). 0xFFFF = no cell / PEC-skipped this scan.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_raw_grid_frame(const std::uint16_t* grid95,
                                                  std::uint8_t frame_idx) noexcept {
    const auto& A = ifs08::ALL_ARRAYS[0];   // reuse 24 x 4-cell BE-u16 window
    if (grid95 == nullptr || frame_idx >= A.frame_count) return Frame{};
    std::uint8_t b[8];
    can_dsl::encode_array_window(
        frame_idx, A.per_frame, A.elem_bits, A.big_endian, A.total_elems,
        /*sentinel=*/0xFFFFu,
        [&](unsigned flat) -> std::uint64_t { return grid95[flat]; },
        b);
    return detail::to_frame(b);
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
    const auto& A = ifs08::ALL_ARRAYS[1];   // 0x6A0 temp grid
    if (frame_idx >= A.frame_count) return Frame{};
    std::uint8_t b[8];
    can_dsl::encode_array_window(
        frame_idx, A.per_frame, A.elem_bits, A.big_endian, A.total_elems,
        /*sentinel*/0u,   // 25*8 == 200 exactly, no slot ever exceeds total
        [&](unsigned flat) -> std::uint64_t {
            const std::uint8_t m = static_cast<std::uint8_t>(flat / config::TempsPerModule);
            const std::uint8_t t = static_cast<std::uint8_t>(flat % config::TempsPerModule);
            // Offline module (isoSPI silent) -> emit the no-reading sentinel
            // (clips to -128) rather than the frozen last-good temperature, so a
            // disconnected chain never displays as a live temperature.
            const std::int16_t raw = ((bms.module_online_mask & (1u << m)) == 0u)
                                         ? config::NtcNoReading
                                         : bms.cell_tempC[m][t];
            return static_cast<std::uint8_t>(clip_int8_pit(raw));
        },
        b);
    return detail::to_frame(b);
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
//   byte 2  cockpit inputs: bit2=balance_override (balancing
//                           paused by a fresh 0x103 "BALO"),
//                           bit1=TSMS readback, bit0=DASH_CHG readback
//   byte 3  ams_ok_gpio (0/1)
//   bytes 4..5  pec_err_total  BE u16 (saturates at 0xFFFF if all 10
//                              ICs combined exceed 65 535)
//   byte 6  fault_reason  the predicate branch that latched ERROR
//                         (ams::safety::FaultReason; 12=FSM-driven
//                         Error path; 0=not latched)
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
                                             std::uint8_t  fault_detail = 0u,
                                             bool          balance_override = false) noexcept {
    ifs08::PIT_fsm_status_t s{};
    s.fsm_state        = fsm_state;
    s.mode_locked      = static_cast<std::uint8_t>(mode_locked);
    s.dash_chg         = dash_chg ? 1u : 0u;
    s.tsms             = tsms ? 1u : 0u;
    s.balance_override = balance_override ? 1u : 0u;
    s.ams_ok_gpio      = ams_ok_gpio ? 1u : 0u;
    s.pec_err_total    = (pec_err_total > 0xFFFFu)
                             ? 0xFFFFu : static_cast<std::uint16_t>(pec_err_total);
    s.fault_reason     = fault_reason;
    s.fault_detail     = fault_detail;
    std::uint8_t b[8];
    ifs08::encode_PIT_fsm_status(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// Poll-timing frame (0x6C1). MingoCAN can graph these to spot SPI
// jitter / mux degradation purely from the CAN stream.
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
    const auto clip16 = [](std::uint32_t v) -> std::uint16_t {
        return (v > 0xFFFFu) ? static_cast<std::uint16_t>(0xFFFFu)
                             : static_cast<std::uint16_t>(v);
    };
    ifs08::PIT_timing_t s{};
    s.bms_volt_poll_ms     = clip16(bms_volt_poll_ms);
    s.bms_volt_poll_max_ms = clip16(bms_volt_poll_max_ms);
    s.temp_sweep_last_mask = temp_sweep_last_mask;
    std::uint8_t b[8];
    ifs08::encode_PIT_timing(s, b);
    return detail::to_frame(b);
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
// MingoCAN reconstructs by walking 95 bits and mapping bit b ->
// (module = b / 19, cell = b % 19).
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_balance_mask_a(const volatile std::uint32_t (&dcc_bits)[config::BmsModuleCount]) noexcept {
    std::uint64_t mask = 0;
    for (std::uint8_t cell_idx = 0; cell_idx < 64 && cell_idx < 5 * 19; ++cell_idx) {
        const std::uint8_t m = cell_idx / config::CellsPerModule;
        const std::uint8_t c = cell_idx % config::CellsPerModule;
        if (dcc_bits[m] & (1u << c)) mask |= (std::uint64_t{1} << cell_idx);
    }
    ifs08::PIT_balance_mask_a_t s{};
    s.dcc_mask_lo = mask;
    std::uint8_t b[8];
    ifs08::encode_PIT_balance_mask_a(s, b);
    return detail::to_frame(b);
}

[[nodiscard]] inline Frame encode_balance_mask_b(const volatile std::uint32_t (&dcc_bits)[config::BmsModuleCount],
                                                 std::uint32_t cycles_total,
                                                 std::uint32_t cycles_active) noexcept {
    // Cells 64..94 -> bits 0..30 of a 32-bit LE field (bytes 0..3).
    std::uint32_t mask = 0;
    for (std::uint8_t cell_idx = 64; cell_idx < 5 * 19; ++cell_idx) {
        const std::uint8_t m = cell_idx / config::CellsPerModule;
        const std::uint8_t c = cell_idx % config::CellsPerModule;
        const std::uint8_t bit_pos = cell_idx - 64u;   // 0..30
        if (dcc_bits[m] & (1u << c)) mask |= (std::uint32_t{1} << bit_pos);
    }
    ifs08::PIT_balance_mask_b_t s{};
    s.dcc_mask_hi   = mask;
    s.cycles_total  = (cycles_total  > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(cycles_total);
    s.cycles_active = (cycles_active > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(cycles_active);
    std::uint8_t b[8];
    ifs08::encode_PIT_balance_mask_b(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// Boot diag -- "why did we boot, how did init go". Lets MingoCAN
// distinguish a clean cold boot from a watchdog reset or a CAN-trigger
// BL jump from CAN alone.
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
    ifs08::PIT_boot_diag_t s{};
    s.jump_reason         = jump_reason;
    s.app_init_progress   = app_init_progress;
    s.fdcan1_start_result = fdcan1_start_result;  // low 24 bits land on the wire
    std::uint8_t b[8];
    ifs08::encode_PIT_boot_diag(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// Crash post-mortem -- surfaces the in-RAM trail left by the FreeRTOS
// stack-overflow + malloc-failed hooks (see freertos.c). If a previous
// session crashed, the engineer reads this frame and learns what
// happened from CAN alone; on a clean session every byte stays at 0.
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
    ifs08::PIT_post_mortem_t s{};
    s.stack_overflow_seen      = (stack_overflow_task_addr != 0u) ? 1u : 0u;
    s.stack_overflow_watermark = (stack_overflow_watermark > 0xFFu)
                                     ? 0xFFu : static_cast<std::uint8_t>(stack_overflow_watermark);
    s.stack_overflow_task_addr = stack_overflow_task_addr;
    s.malloc_failed_count      = (malloc_failed_count > 0xFFFFu)
                                     ? 0xFFFFu : static_cast<std::uint16_t>(malloc_failed_count);
    std::uint8_t b[8];
    ifs08::encode_PIT_post_mortem(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// Firmware identification frame. Lets MingoCAN answer "what's
// flashed?" purely over CAN (no separate readout path needed).
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
    ifs08::PIT_fw_id_t s{};
    s.fw_version_major = major;
    s.fw_version_minor = minor;
    s.fw_version_patch = patch;
    s.git_hash_0 = git_hash_4[0];
    s.git_hash_1 = git_hash_4[1];
    s.git_hash_2 = git_hash_4[2];
    s.git_hash_3 = git_hash_4[3];
    s.bl_node_id = bl_node_id;
    std::uint8_t b[8];
    ifs08::encode_PIT_fw_id(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// Per-IC PEC error counts. 0x6C0[4..5] already carries the sum,
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
//   IC index 0 = module 0 upper (first LTC, cells 0..8)
//   IC index 1 = module 0 lower (second LTC, cells 9..18)
//   IC index 2 = module 1 upper, etc.
// So 0x6C7 byte 0 spike = "module 0's top LTC6811 is misbehaving".
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::uint8_t sat_u8(std::uint32_t v) noexcept {
    return (v > 0xFFu) ? 0xFFu : static_cast<std::uint8_t>(v);
}

[[nodiscard]] inline Frame encode_pec_err_count_a(
    const volatile std::uint32_t (&counts)[config::LtcChainLength]) noexcept {
    ifs08::PIT_pec_per_ic_a_t s{};
    s.pec_ic0 = sat_u8(counts[0]); s.pec_ic1 = sat_u8(counts[1]);
    s.pec_ic2 = sat_u8(counts[2]); s.pec_ic3 = sat_u8(counts[3]);
    s.pec_ic4 = sat_u8(counts[4]); s.pec_ic5 = sat_u8(counts[5]);
    s.pec_ic6 = sat_u8(counts[6]); s.pec_ic7 = sat_u8(counts[7]);
    std::uint8_t b[8];
    ifs08::encode_PIT_pec_per_ic_a(s, b);
    return detail::to_frame(b);
}

[[nodiscard]] inline Frame encode_pec_err_count_b(
    const volatile std::uint32_t (&counts)[config::LtcChainLength]) noexcept {
    ifs08::PIT_pec_per_ic_b_t s{};
    s.pec_ic8 = sat_u8(counts[8]);
    s.pec_ic9 = sat_u8(counts[9]);
    std::uint8_t b[8];
    ifs08::encode_PIT_pec_per_ic_b(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// FDCAN1 comms health (0x6C9). Surfaces the FDCAN1 TX-path counters
// that were previously RAM-only so the CAN-only HIL bench can CONFIRM a
// Bus-Off recovery fired (count > 0 after an outage) and see TX-enqueue
// pressure -- the AMS analogue of the bootloader's bl_health
// fdcan_recovery_count. Both full u32 (no saturation): the recovery count
// ticks at most ~10/s (one per FdcanBusOffRetryMs) and acu_tx_fail is a
// monotonic enqueue-failure tally.
//
//   bytes 0..3  fdcan1_busoff_recovery_count  LE u32 (Stop/Start attempts;
//                                              0 = no Bus-Off this session)
//   bytes 4..7  acu_tx_fail                    LE u32 (ECU-TX matrix
//                                              AddMessageToTxFifoQ failures)
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_comms_health(std::uint32_t busoff_recovery_count,
                                               std::uint32_t acu_tx_fail) noexcept {
    ifs08::PIT_comms_health_t s{};
    s.fdcan1_busoff_recovery_count = busoff_recovery_count;
    s.acu_tx_fail                  = acu_tx_fail;
    std::uint8_t b[8];
    ifs08::encode_PIT_comms_health(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// 0x6CB PIT_balance_health -- balance-quiesce success/failure counts.
//   bytes 0..3  balance_quiesce_ok    LE u32  (DCC cleared before measuring)
//   bytes 4..7  balance_quiesce_fail  LE u32  (both WRCFGA attempts failed ->
//                                             that poll measured under bleed)
// The RATIO is the diagnostic; a bare fail count says nothing without knowing
// how many quiesces were attempted.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_balance_health(std::uint32_t quiesce_ok,
                                                 std::uint32_t quiesce_fail) noexcept {
    ifs08::PIT_balance_health_t s{};
    s.balance_quiesce_ok   = quiesce_ok;
    s.balance_quiesce_fail = quiesce_fail;
    std::uint8_t b[8];
    ifs08::encode_PIT_balance_health(s, b);
    return detail::to_frame(b);
}

// ---------------------------------------------------------------------------
// 0x6CA AMS_fw_health -- UNGATED firmware-health, ECU-0x704 parity.
// Emitted always-on (NOT part of the pit-diag scan); this adapter just packs
// the gathered fields. free_heap/min_free_heap are clamped to u16 -- the heap
// is 64 KB so they always fit today, but the clamp stops a future larger heap
// from wrapping silently.
//   [0..1] free_heap      BE u16   [4] task_liveness bitfield   [6] uptime_s u8
//   [2..3] min_free_heap  BE u16   [5] reset_cause enum         [7] last_fault
// ---------------------------------------------------------------------------
[[nodiscard]] inline Frame encode_fw_health(std::uint32_t free_heap,
                                            std::uint32_t min_free_heap,
                                            std::uint8_t  task_liveness,
                                            std::uint8_t  reset_cause,
                                            std::uint8_t  uptime_s,
                                            std::uint8_t  last_fault) noexcept {
    ifs08::AMS_fw_health_t s{};
    s.free_heap     = (free_heap     > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(free_heap);
    s.min_free_heap = (min_free_heap > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(min_free_heap);
    s.task_liveness = task_liveness;
    s.reset_cause   = reset_cause;
    s.uptime_s      = uptime_s;
    s.last_fault    = last_fault;
    std::uint8_t b[8];
    ifs08::encode_AMS_fw_health(s, b);
    return detail::to_frame(b);
}

}  // namespace ams::pit_diag
