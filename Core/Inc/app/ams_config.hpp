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

// Window after osKernelStart during which SafetyTask suppresses the
// freshness / range predicates that depend on external data sources
// (BMS responses, ADC samples, VCU heartbeat). At t=0 every service's
// `last_*_tick` is 0; without a grace, the first SafetyTask iteration
// would fault and withhold the watchdog refresh -> IWDG reset within
// ~100 ms. The immediate-safety predicates (FORCE_ERROR, SDC open)
// remain active during the grace; only data-presence predicates are
// suppressed. Must be >= the longest service warm-up:
//   - BmsPollTask first voltage poll: kBmsPollVoltMs (250 ms)
//   - CurrentTask first ADC sample:   kCurrentPeriodMs (50 ms)
//   - AcuCanTask first VCU 0x100:     uncontrolled, but typically
//                                     present from the vehicle bus
// 2000 ms gives generous margin for all of the above and slow CAN
// startup; tune down if a faster boot becomes critical.
inline constexpr std::uint32_t kSafetyBootGraceMs = 2000;
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

// 5 BMS slave modules. Each module is a pair of LTC6811-1 ICs on
// the isoSPI daisy-chain (see kLtcsPerModule / kLtcChainLength
// below). Module 0 == chain slots 0,1 ; module 4 == chain slots 8,9.
// The legacy CAN-poll constants (CANID, stride, response-offset
// ranges) were retired in v1.2.0 along with the FDCAN2 BMS path --
// see commit history of #72 if archaeology is needed.
inline constexpr std::uint8_t  kBmsModuleCount      = 5;
inline constexpr std::uint8_t  kCellsPerModule      = 19;
inline constexpr std::uint8_t  kTempsPerModule      = 40;  // 20 per LTC * 2 LTCs

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

// Cell-balancing parameters (#74). Passive balancing driven from
// the Charge state only: pick the cells with the largest excess
// over the pack minimum, cap the simultaneous count per module so
// dissipation per board stays bounded, and inhibit entirely when
// the warmest NTC says the pack is already hot.
//
// COMMISSION before v1.0.0 against the dissipation budget of the
// BMS_LITE balance resistors and the airflow available in the
// accumulator box.
inline constexpr std::uint16_t kBalanceDeltaMv     = 50;    // mV above min to start balancing
inline constexpr std::int16_t  kBalanceTempMax     = 50;    // degC; abort balancing if max_tempC > this
inline constexpr std::uint8_t  kBalanceMaxActive   = 4;     // cells per module discharging at once
inline constexpr std::uint32_t kBalanceUpdatePolls = 4;     // = 1 Hz at kBmsPollVoltMs = 250 ms

// LTC6811 ADCV / ADAX mode + settling budget. Mode 2 ("Normal",
// 7 kHz first stage) is the canonical choice for race-pack
// metrology: ~2.3 ms to convert all 12 cell channels with the
// default filter. We round up to 3 ms in software (kAdcvSettleMs)
// so jitter from the FreeRTOS tick doesn't clip the conversion.
// ADAX (AUX) under the same mode finishes in ~200 us per channel
// pair plus a settling allowance.
inline constexpr std::uint8_t  kAdcMode          = 2;   // ams::ltc6811::AdcMode::Norm7kHz
inline constexpr std::uint32_t kAdcvSettleMs     = 3;
inline constexpr std::uint32_t kAdaxSettleMs     = 1;

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
// the bootloader. Standard 11-bit ID, very high arbitration priority.
// On FDCAN1 (accumulator bus) since v1.2.0 (#73) -- BmsRxTask was
// retired with the FDCAN2 BMS path, and AcuCanTask is the only RX
// consumer now. The bootloader itself still drives FDCAN2 after
// reset, but the in-band trigger lives on the same bus the pit-tool
// already uses for VCU telemetry.
//
// 4-byte payload magic exists so a stray same-ID frame can't
// accidentally reboot the car. All four bytes must match exactly.
inline constexpr std::uint32_t kBlBootReqCanId      = 0x002u;
inline constexpr std::uint8_t  kBlBootReqPayload[4] = { 0xB0, 0x07, 0xAD, 0x11 };
inline constexpr std::uint8_t  kBlBootReqDlc        = 4;

// Jump-reason log. Bootloader::request_reboot() stamps this word into
// RTC_BKP_DR2 right before NVIC_SystemReset so the post-mortem (BL
// `read-dtc` / next app boot) can tell apart a CAN-triggered jump
// from a fault-triggered jump etc. Cleared on POR by the BL preamble,
// preserved across IWDG / NVIC resets (same backup-domain semantics
// as the boot magic and error latch).
//
// Slot 0: BL boot-request magic. Slot 1: AMS ErrorLatch. Slot 2:
// jump reason. Slot 3+ reserved for future use.
inline constexpr std::uint32_t kBkpJumpReasonReg = 2;

enum class JumpReason : std::uint32_t {
    kNone           = 0x00000000u,
    kCanTrigger     = 0x4A554D50u,  // 'JUMP' -- pit-tool sent the boot frame
    kFaultLatch     = 0x46415554u,  // 'FAUT' -- safety supervisor forced it
    kManualRequest  = 0x4D414E55u,  // 'MANU' -- operator-issued, no fault
};

// AMS node ID on the stm32-can-bootloader multi-node bus. Must match
// the value the BL was compiled with (-DBL_NODE_ID=<n>). Embedded in
// the firmware_info `reserved[0]` slot so the pit-tool can verify at
// flash time that the app it's writing matches the BL it's talking
// to. Changing this requires re-building both halves.
inline constexpr std::uint32_t kAmsNodeId = 0x02u;

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

// ADG731 channel index (0..31) for each of the 20 temperature
// indices we sweep. Extracted from pcbs/BMS_LITE/LTC_1.kicad_sch
// (#71 schematic walk): NTC_1..NTC_10 sit on S1..S10, NTC_11..NTC_20
// sit on S17..S26, with S11..S16 and S27..S31 unpopulated. The
// 0-indexed channel passed to pack_adg731_select is one less than
// the schematic's "S<n>" pin number. LTC_2's mux (U5) mirrors this
// map -- some channels may be physically unconnected on the current
// BMS_LITE revision; cell_tempC slot 20..39 will read open-circuit
// in that case, see docs/COMMISSIONING.md §3.
inline constexpr std::uint8_t kAdg731ChannelMap[kTempsPerLtc] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,    // S1..S10  -> NTC_1..NTC_10
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25,   // S17..S26 -> NTC_11..NTC_20
};

// NTC + voltage-divider parameters (COMMISSION before v1.0.0).
//
// Each NTC is wired between the ADG731 'S' input and the LTC ground
// reference, with a pull-up resistor between V_REF (LTC6811 VREF2 ~
// 3.0 V, buffered by U6) and the same 'S' node. The selected channel
// is steered to the LTC's GPIO1 input, read via RDAUXA (AUX1 in
// 100-uV units). Thermistor resistance is recovered from the
// observed AUX voltage:
//
//   R_ntc = kNtcSeriesR * V_aux / (kNtcVrefMv - V_aux)
//
// Temperature is then derived via the Beta model:
//
//   1/T = 1/T0 + (1/B) * ln(R_ntc / R0)
//
// with T0 = 298.15 K (25 degC), R0 = kNtcR25.
//
// Placeholder values match the BMS_LITE BOM (Murata NCP15XH103J,
// B = 3380 K, R25 = 10 kOhm, series resistor 10 kOhm). Real bench
// calibration during BMS_LITE bring-up may shift these slightly --
// procedure in docs/COMMISSIONING.md §3.
inline constexpr std::uint32_t kNtcBeta      = 3380;   // K
inline constexpr std::uint32_t kNtcR25       = 10000;  // Ohm
inline constexpr std::uint32_t kNtcSeriesR   = 10000;  // Ohm
inline constexpr std::uint16_t kNtcVrefMv    = 3000;   // LTC6811 VREF2
inline constexpr float         kNtcT0Kelvin  = 298.15f;

// Plausibility window for accepted NTC readings. Anything outside
// this range is dropped (slot left at its previous value) so an
// unpopulated mux channel can't drive max_tempC into orbit and trip
// safety::kForceError on a clean pack.
inline constexpr std::int16_t kNtcMinValidC = -40;
inline constexpr std::int16_t kNtcMaxValidC = 150;

}  // namespace ams::config
