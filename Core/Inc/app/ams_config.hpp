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
// Backup-register usage. RTC_BKP_DR0 is owned by the bootloader (it
// reads BL_BOOT_REQ_MAGIC there on every boot, see issue #54 / the
// stm32-can-bootloader memmap). Application uses RTC_BKP_DR1 for the
// ERROR latch so the two never share a word.
// See docs/ARCHITECTURE.md §1 invariant 5.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t kBkpErrorReg     = 1;
inline constexpr std::uint32_t kBkpErrorMagic   = 0xA115EE51u;

// Bootloader handshake. The bootloader reads RTC_BKP_DR0 on every
// boot; if it equals kBlBootReqMagic it stays in BL mode awaiting CAN
// traffic. Application must not touch RTC_BKP_DR0 except via the
// dedicated Bootloader::request_reboot() path.
inline constexpr std::uint32_t kBlBootReqReg    = 0;
inline constexpr std::uint32_t kBlBootReqMagic  = 0xB00710ADu;

// CAN frame the app listens for to trigger a deliberate reboot into
// the bootloader. Standard 11-bit ID, very high arbitration priority,
// well below the BMS namespace (0x12C+). On FDCAN2 -- same bus the
// bootloader listens on, so the host stays on one bus for the whole
// reboot-and-flash workflow.
//
// 4-byte payload magic exists so a stray same-ID frame can't
// accidentally reboot the car. All four bytes must match exactly.
inline constexpr std::uint32_t kBlBootReqCanId      = 0x002u;
inline constexpr std::uint8_t  kBlBootReqPayload[4] = { 0xB0, 0x07, 0xAD, 0x11 };
inline constexpr std::uint8_t  kBlBootReqDlc        = 4;

// Application flash base. Must match STM32H733XG_FLASH.ld's FLASH
// ORIGIN and the bootloader's BL_APP_BASE.
inline constexpr std::uint32_t kAppFlashBase    = 0x08020000u;

// AMS telemetry TX on FDCAN1. Three single-purpose 8-byte frames at
// 500 ms cadence each. See docs/CAN_MAP.md for the byte layouts.
inline constexpr std::uint32_t kAmsTelemStatusId = 0x4A0u;  // state + cell-V extremes
inline constexpr std::uint32_t kAmsTelemPackId   = 0x4A1u;  // pack V + current
inline constexpr std::uint32_t kAmsTelemTempsId  = 0x4A2u;  // temp extremes + dc bus + heartbeat

// ---------------------------------------------------------------------------
// LTC6811-1 + isoSPI BMS chain. New AMS PCB drives a chain of 10 LTCs
// (5 BMS modules × 2 LTCs each) via SPI1 + LTC6820 isoSPI master. See
// docs/BMS_LTC6811.md for the wire protocol and slot mapping.
// ---------------------------------------------------------------------------
inline constexpr std::uint8_t  kLtcsPerModule       = 2;
inline constexpr std::uint8_t  kCellsPerLtcUpper    = 10;  // LTC_1 (top of module)
inline constexpr std::uint8_t  kCellsPerLtcLower    =  9;  // LTC_2 (bottom of module)
inline constexpr std::uint8_t  kLtcChainLength      = 10;  // kBmsModuleCount * kLtcsPerModule
inline constexpr std::uint8_t  kTempsPerLtc         = 20;  // ADG731 channels populated
inline constexpr std::uint8_t  kTempMuxChannelsUsed = 20;  // of 32 on ADG731

}  // namespace ams::config
