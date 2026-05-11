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
// Pack thresholds (FS rules). See docs/ARCHITECTURE.md §6 (SafetyTask).
// ---------------------------------------------------------------------------

inline constexpr std::uint16_t kCellUVmV =  2800;  // under-voltage
inline constexpr std::uint16_t kCellOVmV =  4200;  // over-voltage
inline constexpr std::int16_t  kCellUTC  =   -10;  // under-temperature °C
inline constexpr std::int16_t  kCellOTC  =    60;  // over-temperature  °C
inline constexpr std::int32_t  kImaxMa   = 200000; // |I| absolute max  mA

inline constexpr std::uint32_t kIStaleMs       =  200;  // current sensor stale
inline constexpr std::uint32_t kBmsStaleMs     = 1500;  // any BMS module silent
inline constexpr std::uint32_t kVcuStaleMs     =  200;  // VCU 0x100 stale
inline constexpr std::uint32_t kPrechargeMaxMs = 1500;  // precharge timeout

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

// 5 BMS slave modules, addressed by a 0x1E offset from the master.
inline constexpr std::uint8_t  kBmsModuleCount   = 5;
inline constexpr std::uint16_t kBmsModuleStride  = 0x1E;
inline constexpr std::uint16_t kBmsVoltPollBase  = 0x12C;  // +N*stride per module
inline constexpr std::uint16_t kBmsTempPollBase  = 0x140;
inline constexpr std::uint16_t kBmsVoltRespBase  = 0x12D;  // +N*stride, then 5 frames
inline constexpr std::uint16_t kBmsTempRespBase  = 0x14D;

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

// ---------------------------------------------------------------------------
// Backup register flag (RTC_BKP_DR0): latched ERROR state across resets.
// See docs/ARCHITECTURE.md §1 invariant 5.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t kBkpErrorReg     = 0;
inline constexpr std::uint32_t kBkpErrorMagic   = 0xA115EE51u;

}  // namespace ams::config
