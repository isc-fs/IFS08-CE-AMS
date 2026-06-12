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

inline constexpr std::uint16_t CellUnderVoltageMv =  2800;  // under-voltage   -- COMMISSION
inline constexpr std::uint16_t CellOverVoltageMv =  4200;  // over-voltage    -- COMMISSION
inline constexpr std::int16_t  CellUnderTempC  =   -10;  // under-temp °C   -- COMMISSION
inline constexpr std::int16_t  CellOverTempC  =    60;  // over-temp °C    -- COMMISSION
inline constexpr std::int32_t  CurrentMaxMa   = 200000; // |I| max mA      -- COMMISSION

inline constexpr std::uint32_t IStaleMs       =  200;  // pack current sensor stale (safety-critical)
inline constexpr std::uint32_t DcdcIStaleMs   =  500;  // DCDC current sensor stale (informational; not safety-gated -- the HW front-end is a separate single-ended sensor on PC1 and DCDC failure is recoverable)
inline constexpr std::uint32_t BmsStaleMs     = 1500;  // any BMS module silent
inline constexpr std::uint32_t VcuStaleMs     =  200;  // VCU 0x100 stale
// At the moment Start->Precharge fires (TSMS+DASH_CHG asserted), the
// FSM checks "have we heard a VCU 0x100 frame in the last VcuFreshMs?"
// to decide whether the pack is in the car (Run target) or at the
// charger (Charge target). Looser than VcuStaleMs because a slow VCU
// bring-up shouldn't be misclassified as charger. Mode is locked at
// the moment of transition and never re-evaluated.
inline constexpr std::uint32_t VcuFreshMs     = 1000;

// Precharge deadline (#302 follow-up). If the bus doesn't reach the
// precharge target within this window the FSM latches Error and opens
// every contactor, bounding how long the precharge contactor + resistor
// can be held closed. This protects the precharge resistor (rated for
// transient duty, not steady-state) for ANY stuck-precharge cause:
// a stuck contactor, no charger, a bus fault, or -- the case that
// reopened this -- a car with a dead VCU that locks Charger mode and
// would otherwise sit in Precharge forever, since `dc_bus_V` (the
// precharge-complete input) comes only from the VCU's 0x100. A normal
// precharge completes well under 1 s; this is the failsafe ceiling.
// TransitionHoldMs stays removed (Transition is a one-step passthrough).
inline constexpr std::uint32_t PrechargeMaxMs = 5000;  // COMMISSION (resistor thermal limit)

inline constexpr std::uint8_t  AllModulesMask = 0x1F;  // 5 modules present

// ---------------------------------------------------------------------------
// Task periods (ms). See docs/ARCHITECTURE.md §2 task table.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t SafetyPeriodMs    =  10;

// Window after osKernelStart during which SafetyTask suppresses the
// freshness / range predicates that depend on external data sources
// (BMS responses, ADC samples, VCU heartbeat). At t=0 every service's
// `last_*_tick` is 0; without a grace, the first SafetyTask iteration
// would fault and withhold the watchdog refresh -> IWDG reset within
// ~100 ms. The immediate-safety predicates (FORCE_ERROR, SDC open)
// remain active during the grace; only data-presence predicates are
// suppressed. Must be >= the longest service warm-up:
//   - BmsPollTask first voltage poll: BmsPollVoltMs (250 ms)
//   - CurrentSensorTask first ADC sample: CurrentPeriodMs (50 ms)
//   - AcuCanTask first VCU 0x100:     uncontrolled, but typically
//                                     present from the vehicle bus
// 2000 ms gives generous margin for all of the above and slow CAN
// startup; tune down if a faster boot becomes critical.
inline constexpr std::uint32_t SafetyBootGraceMs = 2000;

// Cell V/T range debounce (#279). The cell-voltage / temperature range
// predicates must persist for this many consecutive SafetyTask
// evaluations (× SafetyPeriodMs = 10 ms) before they latch the sticky
// ERROR. A single sub-threshold sample -- a torn lock-free snapshot
// read, or an unsettled first poll / emulator-default value at boot --
// must not permanently fault the chip. Sized to span MORE than one
// BmsPollVoltMs (250 ms = 25 ticks) cycle: a transient that clears on
// the next voltage poll never reaches the count, while a sustained
// real under/over condition does. Cell V/T are slow-by-nature faults
// (a cell cannot leave its valid window for one tick and return), so
// the ~300 ms confirmation is well within their response budget. Only
// the cell V/T branches are debounced -- force-error, BMS offline/stale,
// current-over-limit, and VCU-stale still latch on the first tick.
inline constexpr std::uint16_t CellFaultConfirmTicks = 30;  // ~300 ms

inline constexpr std::uint32_t StatePeriodMs     =  20;
inline constexpr std::uint32_t CurrentPeriodMs   =  50;
inline constexpr std::uint32_t AcuHeartbeatMs    = 100;
inline constexpr std::uint32_t BmsPollVoltMs     = 250;
inline constexpr std::uint32_t BmsPollTempMs     = 500;
inline constexpr std::uint32_t TelemetryPeriodMs = 500;

// ---------------------------------------------------------------------------
// CAN map. Source of truth: docs/CAN_MAP.md. Frame-byte layout lives with
// the encode/decode helpers in can_frame.hpp.
// ---------------------------------------------------------------------------

// 5 BMS slave modules. Each module is a pair of LTC6811-1 ICs on
// the isoSPI daisy-chain (see LtcsPerModule / LtcChainLength
// below). Module 0 == chain slots 0,1 ; module 4 == chain slots 8,9.
// The legacy CAN-poll constants (CANID, stride, response-offset
// ranges) were retired in v1.2.0 along with the FDCAN2 BMS path --
// see commit history of #72 if archaeology is needed.
inline constexpr std::uint8_t  BmsModuleCount      = 5;
inline constexpr std::uint8_t  CellsPerModule      = 19;
inline constexpr std::uint8_t  TempsPerModule      = 40;  // 20 per LTC * 2 LTCs

// Accumulator bus (FDCAN1).
//
// Retired in fix/48 (TSMS+DASH_CHG FSM): AcuRxStartBtnId (0x600) and
// AcuRxChargerId (0x18FF50E7, ext) were the legacy Start->{Precharge,
// Charge} triggers. Both replaced by physical GPIOs (TSMS_Pin /
// DASH_CHG_Pin). With those gone, the firmware is standard-frame-only
// on FDCAN1 -- the HW global filter rejects extended at the gate (#231
// follow-up); see app_init_task.cpp::HAL_FDCAN_ConfigGlobalFilter.
//
// ACU RX (FDCAN1): VCU 0x100 DC-bus heartbeat (LE uint16 V). The
// car/charger mode lock at Start->Precharge consumes its freshness.
inline constexpr std::uint32_t AcuRxDcBusId     = 0x100;   // standard; VCU DC bus heartbeat

// Operator charge-mode request (#305). The charger has NO comms with the
// AMS, so an external operator tool asserts this frame to declare "we are
// on the charger". Charger mode requires BOTH a fresh request AND VCU
// absence (see the mode lock in safety_task.cpp). That removes the
// dead-VCU-car ambiguity: a car with a dead VCU does NOT send this, so it
// stays Car mode and faults on VcuStale instead of silently charging.
// Magic payload so bus noise / a stray frame can't trigger a HV-mode
// change; freshness-checked so only an actively-asserted request counts.
inline constexpr std::uint32_t ChargeModeReqId      = 0x101u;  // standard; operator -> AMS. COMMISSION (confirm vs ECU map)
inline constexpr std::uint8_t  ChargeModeReqDlc     = 4u;
inline constexpr std::uint8_t  ChargeModeReqMagic[4] = { 0x43u, 0x48u, 0x52u, 0x47u };  // "CHRG"
inline constexpr std::uint32_t ChargeReqFreshMs     = 1000;   // must be this recent at the mode lock

// Operator balance-control override (#336). The ChargerDisplayWario pit
// tool can pause autonomous cell balancing during Charge (e.g. for a
// clean cell-V snapshot). Magic-gated like 0x101: "BALO" suppresses
// balancing, "BALX" resumes auto. Re-sent ~2 Hz while ON; if the frame
// goes stale (> BalanceOverrideFreshMs) the AMS reverts to autonomous.
// Only affects balancing (which runs in Charge only) -- never an AIR /
// safety path.
inline constexpr std::uint32_t BalanceOverrideReqId    = 0x103u;  // standard; operator -> AMS. COMMISSION (confirm vs ECU map)
inline constexpr std::uint8_t  BalanceOverrideReqDlc   = 4u;
inline constexpr std::uint8_t  BalanceOverrideOnMagic[4]  = { 0x42u, 0x41u, 0x4Cu, 0x4Fu };  // "BALO" -> suppress
inline constexpr std::uint8_t  BalanceOverrideOffMagic[4] = { 0x42u, 0x41u, 0x4Cu, 0x58u };  // "BALX" -> resume auto
inline constexpr std::uint32_t BalanceOverrideFreshMs  = 5000;   // revert to auto if silent this long

// ACU TX (FDCAN1) -- the ECU's FDCAN2 peripheral is wired to AMS
// FDCAN1, so the ECU sees these frames and forwards them to real-time
// telemetry. All standard 11-bit IDs, big-endian payloads.
// See docs/CAN_MAP.md for full byte layouts.
//
// Cadences (acu_can_task per-frame deadline scheduler):
//   EcuFastTxMs (50 ms)   -> 0x135 currents
//   EcuMidTxMs  (100 ms)  -> 0x020 ok_precharge + 0x12C vmin
//                              + 0x131..0x134 per-module V
//   EcuSlowTxMs (250 ms)  -> 0x136..0x137 per-module temps
//
// 0x130 (SOC) deferred: no SOC estimator in firmware yet.
inline constexpr std::uint32_t AcuTxOkPrechargeId    = 0x020;  // 1 byte: 1 if FSM in Run|Charge
inline constexpr std::uint32_t AcuTxMinVoltId        = 0x12C;  // BE u16 mV (pack-wide cell min)
inline constexpr std::uint32_t AcuTxSocId            = 0x130;  // 1 byte SoC % [DEFERRED -- no estimator yet; ID reserved + stub encoder available, not scheduled for TX]
inline constexpr std::uint32_t AcuTxVminModuleAId    = 0x131;  // BE u16 mV x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxVminModuleBId    = 0x132;  // BE u16 mV x2 (modules 3..4)
inline constexpr std::uint32_t AcuTxVmaxModuleAId    = 0x133;  // BE u16 mV x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxVmaxModuleBId    = 0x134;  // BE u16 mV x2 (modules 3..4)
inline constexpr std::uint32_t AcuTxCurrentsId       = 0x135;  // BE i16 deciamps x2 (accu, dcdc)
inline constexpr std::uint32_t AcuTxTempMaxModuleAId = 0x136;  // BE i16 degC x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxTempMaxModuleBId = 0x137;  // BE i16 degC x3 (mod 3, 4, dcdc-stub)

// Legacy 0x450 current frame retired in fix/53 -- superseded by 0x135
// (signed deciamps + DCDC in the same frame). AcuTxCurrWarn/Over/
// NormId reserved for future use.
inline constexpr std::uint32_t AcuTxCurrentWarnId       = 0x500;
inline constexpr std::uint32_t AcuTxCurrentOverLimitId       = 0x501;
inline constexpr std::uint32_t AcuTxCurrentNormalId       = 0x502;

inline constexpr std::uint32_t EcuFastTxMs           = 50;
inline constexpr std::uint32_t EcuMidTxMs            = 100;
inline constexpr std::uint32_t EcuSlowTxMs           = 250;

// ---------------------------------------------------------------------------
// Pit-side diagnostic stream (#247). Runtime-toggleable full-grid telemetry
// for pit-stop debugging when the accumulator is plugged into can0 (car
// stationary in the pit, or accumulator on the charger). Off by default.
//
// Enable contract:
//   RX 0x7F0 [4] DE AD BE EF  -> stream ON
//   RX 0x7F0 [4] 00 00 00 00  -> stream OFF
//   TX 0x7F1 [1] {01|00}      -> one-shot ACK after a state transition
//
// State lives in RAM only -- reboot clears it (no risk of leaving diag on
// through a track session). Stream continues through FSM::Error so that
// charging-fault diagnostics survive a trip.
//
// Stream IDs (1 Hz cadence; ~50 frames/scan ~= 1 % bus load at 500 kbps):
//   0x680..0x697   24 frames of 4 cells each (BE u16 mV); last frame has
//                  3 real cells + 1 sentinel 0xFFFF for cells 95..95.
//                  Decoder: cell_index = 4 * (id - 0x680) + slot;
//                           module = cell_index / 19, cell = cell_index % 19.
//   0x6A0..0x6B8   25 frames of 8 NTC temps each (i8 degC); covers all
//                  200 NTCs row-major over cell_tempC[5][40].
//   0x6C0          FSM extended status (see pit_diag_emitter.hpp layout).
//   0x6C1          Poll timing (last/max V-poll ms, last temp-sweep mask).
inline constexpr std::uint32_t PitDiagCmdRxId            = 0x7F0u;
inline constexpr std::uint32_t PitDiagAckTxId            = 0x7F1u;
inline constexpr std::uint32_t PitDiagCellBaseId         = 0x680u;
inline constexpr std::uint32_t PitDiagTempBaseId         = 0x6A0u;
inline constexpr std::uint32_t PitDiagFsmStatusId        = 0x6C0u;
inline constexpr std::uint32_t PitDiagTimingId           = 0x6C1u;
inline constexpr std::uint32_t PitDiagBalanceMaskAId     = 0x6C2u;  // DCC bits cells 0..63
inline constexpr std::uint32_t PitDiagBalanceMaskBId     = 0x6C3u;  // DCC bits cells 64..94 + cycle counts
inline constexpr std::uint32_t PitDiagBootDiagId         = 0x6C4u;  // reset reason + app_init_progress
inline constexpr std::uint32_t PitDiagPostMortemId       = 0x6C5u;  // stack overflow + malloc fail
inline constexpr std::uint32_t PitDiagFwIdId             = 0x6C6u;  // semver + git hash[0..3] + BL node id
inline constexpr std::uint32_t PitDiagPecPerIcAId        = 0x6C7u;  // per-IC PEC count: ICs 0..7 (saturating u8)
inline constexpr std::uint32_t PitDiagPecPerIcBId        = 0x6C8u;  // per-IC PEC count: ICs 8..9 + reserved
inline constexpr std::uint8_t  PitDiagCmdDlc             = 4u;
inline constexpr std::uint8_t  PitDiagEnableMagic[4]     = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
inline constexpr std::uint8_t  PitDiagDisableMagic[4]    = { 0x00u, 0x00u, 0x00u, 0x00u };
inline constexpr std::uint32_t PitDiagScanPeriodMs       = 1000u;  // 1 Hz scan when enabled
inline constexpr std::uint8_t  PitDiagCellFrames         = 24u;    // ceil(95 / 4)
inline constexpr std::uint8_t  PitDiagTempFrames         = 25u;    // 200 / 8 exactly
inline constexpr std::uint16_t PitDiagCellSentinel       = 0xFFFFu;

// Stub sentinel for the temp_dcdc byte (0x137 data[4..5]) while the
// DCDC temperature sensor is not wired. INT16_MIN = "not available".
inline constexpr std::int16_t  DcdcTempStubValue     = -32768;

// Precharge target: DC bus must reach this fraction of pack voltage.
inline constexpr float         PrechargeRatio   = 0.95f;

// DC-bus collapse detector (#330). In Run (Car mode) the bus equals the
// pack voltage; if the AIRs open externally -- e.g. a cockpit SDC shutdown
// the AMS can't sense -- the VCU-reported dc_bus_V collapses. When it
// falls below BusCollapsePercent of the pack (cell-sum) for
// BusCollapseConfirmTicks consecutive 10 ms safety ticks, the FSM treats
// the contactors as opened externally and de-energises to Start, so a
// re-arm re-runs precharge instead of reclosing AIR+ onto a discharged
// DC-link. COMMISSION both against the real pack/inverter: the percent
// trades false-trip immunity (must sit below worst-case loaded sag of
// dc_bus_V vs the cell-sum) against inrush protection (higher = trips
// earlier, before the link discharges enough to make a no-precharge
// reclose damaging); the debounce rejects a single anomalous 0x100 frame.
inline constexpr std::uint32_t BusCollapsePercent     = 50;  // COMMISSION (% of pack)
inline constexpr std::uint16_t BusCollapseConfirmTicks = 20;  // COMMISSION (~200 ms @ 10 ms)

// Current sensor calibration. Pack current measured via a Bourns
// SSA-2-250A shunt sensor (datasheet: pcbs/ssa-2.pdf). The sensor is a
// 2-wire amplified-differential device: its OUT_P / OUT_N pins carry a
// bipolar +/- 5 mV/A differential signal (250 A nominal, 500 A max
// unclipped, output clips at +/- 2.62 V, common-mode ~1.44 V).
//
// HW revision (feat/current-sensor-diff): the external carrier diff amp
// (old MCP6001R, gain x4, Vref/2 bias) is REMOVED. OUT_P and OUT_N now
// wire straight to the STM32 and the ADC reads them in DIFFERENTIAL
// mode:
//   OUT_P = PF7 = ADC3_INP3   (positive input)
//   OUT_N = PF8 = ADC3_INN3   (negative input, hardware-paired to INP3)
//
// In STM32H7 differential mode the conversion encodes V(INP) - V(INN)
// over the range -Vref .. +Vref onto codes 0 .. 4095, with the
// zero-difference point at mid-scale (code ~= 2048). The sensor's
// common-mode (1.44 V) cancels in the subtraction, so only the bipolar
// +/- 5 mV/A differential remains:
//
//   V(INP) - V(INN) = 5 mV/A * I        (discharge -> +, charge -> -)
//   raw            ~= 2048 + (V_diff / LSB_diff),  LSB_diff = 2*Vref/4095
//
// Net sensitivity is now the bare sensor 5 mV/A (no x4 gain), so
// CurrentMvPerAmpe1 drops from 200 (20 mV/A) to 50 (5 mV/A). Zero
// current reads code ~2048 (CurrentZeroCount), NOT a mid-rail voltage:
// the natural reference in differential mode is the mid code, so the
// zero point is a COMMISSION *count* offset rather than a voltage.
//
// Observable range: the differential pair spans ~+/- Vref, i.e. well
// beyond the sensor's own +/- 2.62 V (~+/-524 A) clip and beyond the
// rail headroom set by the 1.44 V common-mode. Unlike the old x4 +
// 1.65 V single-ended front-end (which clipped firmware-side at only
// +/- 82.5 A, below the 200 A safety threshold), the CurrentMaxMa
// over-current check is now genuinely reachable.
//
// DCDC supply current uses an Allegro ACS758 Hall-effect sensor (a
// different part from the pack SSA-2) on PC1 = ADC3_INP11 (was PF8),
// read SINGLE-ENDED through a unity buffer (gain 1). The ACS758 is
// RATIOMETRIC: both its zero offset and its sensitivity scale with Vcc.
// At the 5 V datasheet rating it is 40 mV/A with offset 0.5*Vcc = 2.5 V;
// powered from 3.3 V here, both scale by 3.3/5:
//
//   offset      = 0.5 * 3.3 V        = 1.65 V
//   sensitivity = 40 mV/A * 3.3/5    = 26.4 mV/A
//   V(PC1) = 1.65 V + 26.4 mV/A * I     (sign per IP+ -> IP- wiring)
//
// Hence DcdcCurrentZeroMv = 1650 (= Vcc/2, which is also ADC mid-scale
// since the ACS758 shares the 3.3 V rail) and DcdcCurrentMvPerAmpe1 =
// 264 (26.4 mV/A x10). Converted by adc_to_mA_dcdc. DCDC is
// informational only -- not part of any safety predicate -- and the
// sign MUST be confirmed on the bench (the ACS758 IP direction sets it).
//
// COMMISSION: CurrentZeroCount and CurrentMvPerAmpe1 (pack), and
// DcdcCurrentZeroMv / DcdcCurrentMvPerAmpe1 (DCDC), MUST be calibrated
// per docs/COMMISSIONING.md §2. The pack values below are the HIL-bench
// commissioned figures (#348, against feat/current-sensor-diff): a DAC
// injection verified at exactly 5 mV/A measured the firmware reading a
// stable 0.924x (7.6 % low) with a +0.6 A zero. Folding that effective
// gain into the (COMMISSION) sensitivity gives CurrentMvPerAmpe1 = 46
// (50 / 0.924, residual +0.4 %) and the zero into CurrentZeroCount =
// 2050 (raw at 0 A). The nominal ideal would be 50 / 2048; the converter
// math is unchanged -- this just absorbs the measured ADC/VREF gain.
inline constexpr std::uint16_t AdcVrefMv          = 3300;
inline constexpr std::uint16_t AdcMaxCount        = 4095;
// Pack channel (differential ADC3_INP3/INN3 = PF7/PF8). HIL-commissioned.
inline constexpr std::int32_t  CurrentZeroCount   = 2050;  // diff zero @ 0 A (HIL #348)  COMMISSION
inline constexpr std::int32_t  CurrentMvPerAmpe1  = 46;    // COMMISSION (eff 5.4 mV/A x10; HIL #348 gain trim)
// DCDC channel (single-ended ADC3_INP11 = PC1; Allegro ACS758 @ 3.3 V).
inline constexpr std::int32_t  DcdcCurrentZeroMv     = 1650; // ACS758 offset = Vcc/2 @ 3.3 V  COMMISSION
inline constexpr std::int32_t  DcdcCurrentMvPerAmpe1 = 264;  // COMMISSION (26.4 mV/A x10 ratiometric @ 3.3 V)
inline constexpr std::uint8_t  CurrentFilterShift = 4;     // tau ~ 16 samples

// Current-sensor disconnect detection. PF7/PF8 carry a weak internal
// pull-down (GPIO PUPDR, set in stm32h7xx_hal_msp.c) -- the SSA-2's
// low-impedance op-amp output overrides it when connected, but an open
// connector lets the legs collapse toward 0 V. We read OUT_P (PF7) in
// SINGLE-ENDED mode each cycle and check it sits in a plausible window:
// a connected sensor holds OUT_P at the ~1.44 V common-mode plus the
// per-leg half-swing (+/- 2.5 mV/A), i.e. ~0.94..1.94 V across +/-200 A;
// a disconnect drags it to ~0 V, out of the window -> sensor_fault ->
// the CurrentSensorFault predicate latches Error. (An OUT_N-only break
// instead skews the differential huge and trips CurrentOverLimit, so
// between the two predicates every disconnect mode is covered.)
//
// COMMISSION: widen/trim the window to the real OUT_P swing + pull-down
// droop measured on the carrier; confirm the disconnect actually trips
// on the bench (the pull-down behaviour is board/VREF specific).
inline constexpr std::int32_t  CurrentLegPlausMinMv = 700;   // COMMISSION (below -> disconnected)
inline constexpr std::int32_t  CurrentLegPlausMaxMv = 2300;  // COMMISSION (above -> open/rail)
inline constexpr std::uint8_t  CurrentDisconnectConfirm = 3; // consecutive out-of-window reads (~150 ms @ 50 ms)

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
inline constexpr std::uint16_t BalanceDeltaMv     = 50;    // mV above min to start balancing
inline constexpr std::int16_t  BalanceTempMax     = 50;    // degC; abort balancing if max_tempC > this
inline constexpr std::uint8_t  BalanceMaxActive   = 4;     // cells per module discharging at once
inline constexpr std::uint32_t BalanceUpdatePolls = 4;     // = 1 Hz at BmsPollVoltMs = 250 ms

// LTC6811 ADCV / ADAX mode + settling budget. Mode 2 ("Normal",
// 7 Hz first stage) is the canonical choice for race-pack
// metrology: ~2.3 ms to convert all 12 cell channels with the
// default filter. We round up to 3 ms in software (AdcvSettleMs)
// so jitter from the FreeRTOS tick doesn't clip the conversion.
// ADAX (AUX) under the same mode finishes in ~200 us per channel
// pair plus a settling allowance.
inline constexpr std::uint8_t  AdcMode          = 2;   // ams::ltc6811::AdcMode::Norm7kHz
inline constexpr std::uint32_t AdcvSettleMs     = 3;
inline constexpr std::uint32_t AdaxSettleMs     = 1;

// ---------------------------------------------------------------------------
// Backup-register usage. RTC_BKP_DR0 is owned by the bootloader (it
// reads BL_BOOT_REQ_MAGIC there on every boot, see issue #54 / the
// stm32-can-bootloader memmap). Application uses RTC_BKP_DR1 for the
// ERROR latch so the two never share a word.
// See docs/ARCHITECTURE.md §1 invariant 5.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t BkpErrorReg     = 1;
inline constexpr std::uint32_t BkpErrorMagic   = 0xA115EE51u;

// Bootloader handshake. The bootloader reads RTC_BKP_DR0 on every
// boot; if it equals BlBootReqMagic it stays in BL mode awaiting CAN
// traffic. Application must not touch RTC_BKP_DR0 except via the
// dedicated Bootloader::request_reboot() path.
inline constexpr std::uint32_t BlBootReqReg    = 0;
inline constexpr std::uint32_t BlBootReqMagic  = 0xB00710ADu;

// CAN frame the app listens for to trigger a deliberate reboot into
// the bootloader. Standard 11-bit ID, very high arbitration priority.
// On FDCAN1 (accumulator bus) since v1.2.0 (#73) -- BmsRxTask was
// retired with the FDCAN2 BMS path, and AcuCanTask is the only RX
// consumer now. The bootloader itself still drives FDCAN2 after
// reset, but the in-band trigger lives on the same bus MingoCAN
// already uses for VCU telemetry.
//
// 4-byte payload magic exists so a stray same-ID frame can't
// accidentally reboot the car. All four bytes must match exactly.
inline constexpr std::uint32_t BlBootReqCanId      = 0x002u;
inline constexpr std::uint8_t  BlBootReqPayload[4] = { 0xB0, 0x07, 0xAD, 0x11 };
inline constexpr std::uint8_t  BlBootReqDlc        = 4;

// Jump-reason log. Bootloader::request_reboot() stamps this word into
// RTC_BKP_DR2 right before NVIC_SystemReset so the post-mortem (BL
// `read-dtc` / next app boot) can tell apart a CAN-triggered jump
// from a fault-triggered jump etc. Cleared on POR by the BL preamble,
// preserved across IWDG / NVIC resets (same backup-domain semantics
// as the boot magic and error latch).
//
// Slot 0: BL boot-request magic. Slot 1: AMS ErrorLatch. Slot 2:
// jump reason. Slot 3+ reserved for future use.
inline constexpr std::uint32_t BkpJumpReasonReg = 2;

enum class JumpReason : std::uint32_t {
    None           = 0x00000000u,
    CanTrigger     = 0x4A554D50u,  // 'JUMP' -- MingoCAN sent the boot frame
    FaultLatch     = 0x46415554u,  // 'FAUT' -- safety supervisor forced it
    ManualRequest  = 0x4D414E55u,  // 'MANU' -- operator-issued, no fault
};

// AMS node ID on the stm32-can-bootloader multi-node bus. Must match
// the value the BL was compiled with (-DBL_NODE_ID=<n>). Embedded in
// the firmware_info `reserved[0]` slot so MingoCAN can verify at
// flash time that the app it's writing matches the BL it's talking
// to. Changing this requires re-building both halves.
//
// 2026-05-18: BL team adopted node ID 0x01 on MLC1 (matches the value
// already in NVM from factory). This flip aligns the firmware-side
// firmware_info.reserved[0] with the BL it now talks to. Confirmed
// by IFS08_HIL#30 turning A-002/A-003 green.
inline constexpr std::uint32_t AmsNodeId = 0x01u;

// Application flash base. Must match STM32H733XG_FLASH.ld's FLASH
// ORIGIN and the bootloader's BL_APP_BASE.
inline constexpr std::uint32_t AppFlashBase    = 0x08020000u;

// AMS telemetry TX on FDCAN1. Three single-purpose 8-byte frames at
// 500 ms cadence each. See docs/CAN_MAP.md for the byte layouts.
inline constexpr std::uint32_t AmsTelemStatusId = 0x4A0u;  // state + cell-V extremes
inline constexpr std::uint32_t AmsTelemPackId   = 0x4A1u;  // pack V + current
inline constexpr std::uint32_t AmsTelemTempsId  = 0x4A2u;  // temp extremes + dc bus + heartbeat
inline constexpr std::uint32_t AmsTelemDiagId   = 0x4A3u;  // diagnostic probes (#123); pure-fn encoder

// ---------------------------------------------------------------------------
// LTC6811-1 + isoSPI BMS chain. New AMS PCB drives a chain of 10 LTCs
// (5 BMS modules × 2 LTCs each) via SPI1 + LTC6820 isoSPI master. See
// docs/BMS_LTC6811.md for the wire protocol and slot mapping.
// ---------------------------------------------------------------------------
inline constexpr std::uint8_t  LtcsPerModule       = 2;
inline constexpr std::uint8_t  CellsPerLtcUpper    = 10;  // LTC_1 (top of module)
inline constexpr std::uint8_t  CellsPerLtcLower    =  9;  // LTC_2 (bottom of module)
inline constexpr std::uint8_t  LtcChainLength      = 10;  // BmsModuleCount * LtcsPerModule
inline constexpr std::uint8_t  TempsPerLtc         = 20;  // ADG731 channels populated
inline constexpr std::uint8_t  TempMuxChannelsUsed = 20;  // of 32 on ADG731

// ADG731 channel index (0..31) for each of the 20 temperature
// indices we sweep. Extracted from pcbs/BMS_LITE/LTC_1.kicad_sch
// (#71 schematic walk): NTC_1..NTC_10 sit on S1..S10, NTC_11..NTC_20
// sit on S17..S26, with S11..S16 and S27..S31 unpopulated. The
// 0-indexed channel passed to pack_adg731_select is one less than
// the schematic's "S<n>" pin number. LTC_2's mux (U5) mirrors this
// map -- some channels may be physically unconnected on the current
// BMS_LITE revision; cell_tempC slot 20..39 will read open-circuit
// in that case, see docs/COMMISSIONING.md §3.
inline constexpr std::uint8_t Adg731ChannelMap[TempsPerLtc] = {
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
//   R_ntc = NtcSeriesR * V_aux / (NtcVrefMv - V_aux)
//
// Temperature is then derived via the Beta model:
//
//   1/T = 1/T0 + (1/B) * ln(R_ntc / R0)
//
// with T0 = 298.15 K (25 degC), R0 = NtcR25.
//
// Placeholder values match the BMS_LITE BOM (Murata NCP15XH103J,
// B = 3380 K, R25 = 10 Ohm, series resistor 10 Ohm). Real bench
// calibration during BMS_LITE bring-up may shift these slightly --
// procedure in docs/COMMISSIONING.md §3.
inline constexpr std::uint32_t NtcBeta      = 3380;   // K
inline constexpr std::uint32_t NtcR25       = 10000;  // Ohm
inline constexpr std::uint32_t NtcSeriesR   = 10000;  // Ohm
inline constexpr std::uint16_t NtcVrefMv    = 3000;   // LTC6811 VREF2
inline constexpr float         NtcT0Kelvin  = 298.15f;

// Plausibility window for accepted NTC readings. Anything outside
// this range is dropped (slot left at its previous value) so an
// unpopulated mux channel can't drive max_tempC into orbit and trip
// safety::ForceError on a clean pack.
inline constexpr std::int16_t NtcMinValidC = -40;
inline constexpr std::int16_t NtcMaxValidC = 150;

}  // namespace ams::config
