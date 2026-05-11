// SPDX-License-Identifier: proprietary
//
// All compile-time constants for the AMS firmware live here. Thresholds,
// CAN IDs, periods, addresses. No runtime tunables -- this is the FS
// rules expressed in code. Any change goes through a feat/fix branch and
// a labelled PR per docs/CONTRIBUTING.md.

#pragma once

#include <cstdint>

namespace ams::config {

// ---------------------------------------------------------------------------
// Pack thresholds (FS rules). See docs/ARCHITECTURE.md §6 (SafetyTask)
// and docs/COMMISSIONING.md §1 for the procedure to finalise these.
//
// COMMISSION: these are placeholder defaults. Finalise per cell
// datasheet + FS rules before v1.0.0.
// ---------------------------------------------------------------------------

inline constexpr std::uint16_t kCellUVmV =  2800;  // under-voltage   -- COMMISSION
inline constexpr std::uint16_t kCellOVmV =  4200;  // over-voltage    -- COMMISSION
inline constexpr std::int16_t  kCellUTC  =   -10;  // under-temp °C   -- COMMISSION
inline constexpr std::int16_t  kCellOTC  =    60;  // over-temp °C    -- COMMISSION
inline constexpr std::int32_t  kImaxMa   = 200000; // |I| max mA      -- COMMISSION

inline constexpr std::uint32_t kIStaleMs       =  200;  // current sensor stale
inline constexpr std::uint32_t kBmsStaleMs     = 1500;  // any BMS module silent
inline constexpr std::uint32_t kVcuStaleMs     =  200;  // VCU 0x100 stale
inline constexpr std::uint32_t kPrechargeMaxMs    = 1500;  // precharge timeout
inline constexpr std::uint32_t kTransitionHoldMs  =  100;  // hold + verify

inline constexpr std::uint8_t  kAllModulesMask = 0x1F;  // 5 modules present

// ---------------------------------------------------------------------------
// Task periods (ms). See docs/ARCHITECTURE.md §2 task table.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t kSafetyPeriodMs    =  10;
inline constexpr std::uint32_t kStatePeriodMs     =  20;
inline constexpr std::uint32_t kCurrentPeriodMs   =  50;
inline constexpr std::uint32_t kAcuHeartbeatMs    = 100;
inline constexpr std::uint32_t kBmsPollVoltMs     = 250;
inline constexpr std::uint32_t kBmsPollTempMs     = 500;
inline constexpr std::uint32_t kTelemetryPeriodMs = 500;

// ---------------------------------------------------------------------------
// CAN map. Source of truth: docs/CAN_MAP.md. Frame-byte layout lives with
// the encode/decode helpers in can_frame.hpp.
// ---------------------------------------------------------------------------

// 5 BMS slave modules. Each module has its own CANID; module N's CANID
// is (kBmsVoltPollBase + N * kBmsModuleStride). The legacy protocol
// then assigns offsets RELATIVE TO that per-module CANID:
//
//   offset  1..5   -> voltage response frames (frame_idx = offset - 1)
//   offset 20      -> temperature poll request (TX from AMS)
//   offset 21..25  -> temperature response frames (frame_idx = offset - 21)
//
// classify() in bms_service.cpp walks the 5 CANIDs and matches the
// incoming id against the (canid, canid + 30) range with `m = id - canid`.
// See docs/CAN_MAP.md for the full table.
inline constexpr std::uint8_t  kBmsModuleCount      = 5;
inline constexpr std::uint16_t kBmsModuleStride     = 0x1E;
inline constexpr std::uint16_t kBmsVoltPollBase     = 0x12C;  // module 0 CANID
inline constexpr std::uint16_t kBmsTempPollOffset   = 20;     // CANID + 20
inline constexpr std::uint8_t  kBmsVoltRespOffsetLo = 1;
inline constexpr std::uint8_t  kBmsVoltRespOffsetHi = 5;
inline constexpr std::uint8_t  kBmsTempRespOffsetLo = 21;
inline constexpr std::uint8_t  kBmsTempRespOffsetHi = 25;
inline constexpr std::uint16_t kBmsModuleAddrRange  = 30;     // legacy guard

inline constexpr std::uint8_t  kCellsPerModule   = 19;
inline constexpr std::uint8_t  kTempsPerModule   = 38;

// Accumulator bus (FDCAN1).
inline constexpr std::uint32_t kAcuRxDcBusId     = 0x100;   // extended
inline constexpr std::uint32_t kAcuRxStartBtnId  = 0x600;   // standard
inline constexpr std::uint32_t kAcuRxChargerId   = 0x18FF50E7;  // extended
inline constexpr std::uint32_t kAcuTxStateId     = 0x020;   // extended
inline constexpr std::uint32_t kAcuTxMinVoltId   = 0x12C;   // extended
inline constexpr std::uint32_t kAcuTxCurrentId   = 0x450;
inline constexpr std::uint32_t kAcuTxCurrWarnId  = 0x500;
inline constexpr std::uint32_t kAcuTxCurrOverId  = 0x501;
inline constexpr std::uint32_t kAcuTxCurrNormId  = 0x502;

// Precharge target: DC bus must reach this fraction of pack voltage.
inline constexpr float         kPrechargeRatio   = 0.95f;

// Current sensor calibration. Pack current measured via a Hall-effect
// transducer on PF11 -> ADC1 channel 2. The legacy AMS used:
//   - 2.5 V output at zero current
//   - 5.7 mV per ampere sensitivity (sign: + = discharge, - = charge)
// .ioc puts ADC1 in 12-bit + 64x oversampling + right-shift 6, so the
// effective sample is 12-bit (0..4095) referenced to Vref ~ 3.3 V.
//
// COMMISSION: kCurrentZeroMv and kCurrentMvPerAmpe1 MUST be calibrated
// per real-hardware procedure in docs/COMMISSIONING.md §2 before
// v1.0.0. Real Hall units drift; the unit-test-passing defaults are
// only for bring-up.
inline constexpr std::uint16_t kAdcVrefMv          = 3300;
inline constexpr std::uint16_t kAdcMaxCount        = 4095;
inline constexpr std::int32_t  kCurrentZeroMv      = 2500;  // COMMISSION
inline constexpr std::int32_t  kCurrentMvPerAmpe1  = 57;    // COMMISSION (5.7 mV/A x10)
inline constexpr std::uint8_t  kCurrentFilterShift = 4;     // tau ~ 16 samples

// IIR low-pass filter coefficient is encoded as a shift so the filter
// is `filtered = filtered - (filtered >> shift) + (raw >> shift)`.

// BMS poll payloads (TX from AMS -> BMS slaves on FDCAN2).
// Voltage poll carries the cell-balancing target in mV (big-endian);
// the legacy code sets this to the desired upper-balance limit. Send a
// safe default that won't trigger aggressive balancing on bring-up;
// real value tuned during commissioning (feat/18).
inline constexpr std::uint16_t kBmsBalancingTargetmV = 3700;

// ---------------------------------------------------------------------------
// Backup register flag (RTC_BKP_DR0): latched ERROR state across resets.
// See docs/ARCHITECTURE.md §1 invariant 5.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t kBkpErrorReg     = 0;
inline constexpr std::uint32_t kBkpErrorMagic   = 0xA115EE51u;

}  // namespace ams::config
