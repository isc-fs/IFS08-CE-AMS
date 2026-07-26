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

// Cell OPEN-WIRE detection (LTC6811 ADOW). Historically the AMS relied on
// software cell-mV plausibility for a broken sense wire (docs/BMS_LTC6811.md);
// this arms the datasheet open-wire check instead, which catches an open that
// still reads in-range. Runs the two-pass ADOW conversion in BmsPollTask and
// faults FaultReason::CellOpenWire in ANY state.
//
// DEFAULT OFF, exactly like TempFaultsTrusted: the ADOW register encoding and
// conversion timing are not yet validated against a real LTC chain (the bench
// was down when this landed), and a wrong threshold/timing would false-fault a
// healthy pack and trap it in Error. Flip to true ONLY after HIL validation of
// the ADOW path end-to-end. The pure detector (open_wire.hpp) is host-tested
// independently of this flag.
inline constexpr bool          CellOpenWireCheck   = false;
inline constexpr std::uint16_t CellOpenWireDeltaMv = 400;   // datasheet open threshold

// Implausible-cell bounds for the balancing tap-artifact guard (see
// recompute_summaries_ / BmsState::tap_fault_mask). A real cell in a live pack
// physically cannot sit outside this window -- a reading beyond it is a
// MEASUREMENT error, not a true voltage. When two PHYSICALLY-ADJACENT cells
// straddle a shifted shared tap node (balancing current through a high-R tap),
// one reads impossibly high and its neighbour compensates low while the
// tap-immune PAIR SUM stays in the normal window; the guard then feeds the pair
// AVERAGE to the safety aggregates instead of the impossible individual value.
// These bounds are deliberately WIDER than CellOver/UnderVoltageMv: a genuine
// over-/under-voltage (e.g. 4200 < v < 4400) has a NORMAL neighbour (sum runs
// high, not conserved) and is never masked -- it still faults.
inline constexpr std::uint16_t CellImplausibleMaxMv = 4400;  // > any charging cell (4.2 V CV limit)
inline constexpr std::uint16_t CellImplausibleMinMv = 1500;  // < any cell in a connected live pack
// Minimum intra-pair split (|hi - lo|) for the tap-artifact guard to engage.
// Series-adjacent cells track closely even in a badly imbalanced pack; a split
// this large only appears when a shared tap node has shifted. Requiring it (on
// top of the implausible-cell + conserved-sum tests) means a single-cell glitch
// beside a NORMAL neighbour is never averaged away -- it faults conservatively.
inline constexpr std::uint16_t TapArtifactMinSplitMv = 800;
inline constexpr std::int16_t  CellUnderTempC  =   -10;  // under-temp °C   -- COMMISSION
inline constexpr std::int16_t  CellOverTempC  =    60;  // over-temp °C    -- COMMISSION
// Cell TEMPERATURE fault gate. NTC temps are read through the per-LTC ADG731
// 32:1 mux; that path is NOT yet trusted on flight (the mux-select word was
// wrong -- fixed on the bench harness, not yet validated in the flight path).
// While false, the CellUnderTemp / CellOverTemp predicates are SUPPRESSED so
// the FSM never faults on unvalidated temperatures. Cell VOLTAGE protection is
// unaffected. Flip to true once the mux fix ships to flight and temps are
// bench-validated end-to-end.
inline constexpr bool          TempFaultsTrusted = false;
inline constexpr std::int32_t  CurrentMaxMa   = 60000; // |I| max mA      -- COMMISSION

inline constexpr std::uint32_t IStaleMs       =  200;  // pack current sensor stale (safety-critical)
inline constexpr std::uint32_t DcdcIStaleMs   =  500;  // DCDC current sensor stale (informational; not safety-gated -- the HW front-end is a separate single-ended sensor on PC1 and DCDC failure is recoverable)
inline constexpr std::uint32_t BmsStaleMs     = 1000;  // any BMS module silent (1000 ms fault-response window)
inline constexpr std::uint32_t VcuStaleMs     =  200;  // VCU 0x100 stale
// Charger heartbeat (0x101) stale window while in Charger mode. The WarioCharger
// re-sends 0x101 at >= 2 Hz (<= 500 ms period), so 1000 ms tolerates one missed
// heartbeat before faulting Charge -> Error. Charging is not a 10 ms-critical
// response, so this avoids false trips on a single dropped frame. Matches
// ChargeReqFreshMs. See safety_predicates FaultReason::ChargerStale.
inline constexpr std::uint32_t ChargerStaleMs =  1000; // charger 0x101 stale (Charger mode only)
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

// BmsStale confirmation debounce. BmsStale (a BMS module silent past
// BmsStaleMs) is a timeout, but it currently latches ERROR on the FIRST
// SafetyTask tick it crosses. Require it to persist this many consecutive
// evaluations (x SafetyPeriodMs = 10 ms) first, so a far module that flickers
// just past the window under a brief EMI burst and then reports on its next
// voltage poll (<= 250 ms later) does not spuriously open the contactors.
// SAFETY TRADEOFF -- COMMISSION: this ADDS up to BmsStaleConfirmTicks x 10 ms
// to the detection of a GENUINELY lost module (25 -> 1500 ms window + 250 ms
// confirm = 1750 ms worst case). Sized to span one 250 ms voltage-poll cycle so
// a recovering module gets exactly one more chance. Sustained loss (a dead
// chain through a whole torque event) still latches -- the confirm only delays
// it, it does not prevent it. Set to 0 to restore first-tick latching.
inline constexpr std::uint16_t BmsStaleConfirmTicks = 25;  // ~250 ms  COMMISSION

inline constexpr std::uint32_t StatePeriodMs     =  20;
inline constexpr std::uint32_t CurrentPeriodMs   =  50;
inline constexpr std::uint32_t AcuHeartbeatMs    = 100;
inline constexpr std::uint32_t BmsPollVoltMs     = 250;
inline constexpr std::uint32_t BmsPollTempMs     = 500;
inline constexpr std::uint32_t TelemetryPeriodMs = 500;
inline constexpr std::uint32_t RelayStatusPeriodMs = 100;  // 0x4A4 contactor snapshot (always-on)

// ---------------------------------------------------------------------------
// Datalogging to microSD (SDMMC1 + FatFs). Strictly off the safety path:
// SdLoggerTask (low priority) owns the card; SafetyTask only pushes a
// LogRecord into a lock-free ring every LogSamplePeriodMs. Best-effort -- a
// missing/failed card or a full ring degrades to "no log", NEVER a fault and
// never a blocking call on the 10 ms loop. The boot-path SDMMC init is
// decoupled (CubeMX: MX_SDMMC1_SD_Init not auto-called) so an absent card
// can't brick the node -- see AMS #407. Logger: Core/Src/app/sd_logger_task.cpp.
// ---------------------------------------------------------------------------

// SafetyTask captures a record this often. 250 ms = 4 Hz, matched to the
// BmsPollVoltMs voltage poll so every sample carries a FRESH cell-voltage
// frame (no oversampling of the 2-4 Hz BMS data; design option B). ~6 KB/s
// of full per-cell CSV. Must be a multiple of SafetyPeriodMs.
inline constexpr std::uint32_t LogSamplePeriodMs = 250;

// Lock-free ring depth (LogRecords). Each record now carries the FULL
// 95-cell + 200-temp matrices (~620 B), so the ring is ~10 KB of BSS at
// depth 16. MUST be a power of two. 16 @ 4 Hz ~= 4 s of buffer; the
// shared-SD-mutex-with-yielding (#406 pull) lets the logger drain between
// the extractor's reads, so the ring rarely saturates. Bump if RAM allows.
inline constexpr std::uint32_t LogRingCapacity   = 16;

// SdLoggerTask drain cadence, and how often it f_syncs the active file
// (bounds data lost on a power-cut to <= LogSyncPeriodMs of samples).
inline constexpr std::uint32_t LogDrainPeriodMs  = 50;
inline constexpr std::uint32_t LogSyncPeriodMs   = 1000;

// Seal (rotate) the active file once it reaches this size. Bounded, rotated
// files make #406 listing / CRC / resume / "only new logs" tractable.
// Size cap per file. A real 314-column row is ~1.35 kB (76 B of scalars +
// 95 cells + 200 temps), so at 4 Hz the card takes ~5.3 KiB/s and 4 MiB is
// ~13 MINUTES of logging -- not the ~4 min this comment used to claim.
inline constexpr std::uint32_t LogFileMaxBytes   = 4u * 1024u * 1024u;  // 4 MiB (~13 min/file at full per-cell rows)

// Time cap per file. Without this, a file is only sealed to .CSV on the size
// cap, so any run shorter than ~13 min leaves a .TMP that no tool treats as a
// finished log (the #406 extractor lists sealed files only). Rotating on time
// as well means an ordinary bench or test session produces real .CSV files
// while it runs, instead of one perpetual .TMP.
//
// 5 min ~= 1.6 MB per file: short enough that little is at risk if power is
// cut mid-file, long enough not to litter the card.
inline constexpr std::uint32_t LogFileMaxMs      = 5u * 60u * 1000u;    // 5 min

// 8.3 names (LFN off in ffconf.h; no RTC wall-clock -- #406/#407). The active
// file is written as ".TMP" and renamed to ".CSV" on seal, so the extractor
// only ever sees finished logs. Index is a rotation counter, not a timestamp.
inline constexpr char          LogActiveNameFmt[] = "LOG%04lu.TMP";
inline constexpr char          LogSealedNameFmt[] = "LOG%04lu.CSV";
// CRC-32 sidecar written beside a sealed CSV (#406): 8 ASCII hex digits.
// Kept out of the CSV itself so the log stays directly spreadsheet-openable.
inline constexpr char          LogCrcNameFmt[]    = "LOG%04lu.CRC";

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

// Operator balance-control override (#336, extended to a 3-state master
// switch). The ChargerDisplayWario pit tool commands cell balancing on
// 0x103, magic-gated like 0x101:
//   "BALO" -> OFF   force balancing off
//   "BALN" -> ON    force balancing on in ANY FSM state -- operator override
//                   of the Charge-only default. Still honours the temp-trust
//                   gate + thermal lockout in balance::compute_mask (the
//                   operator overrides the ENABLE decision, never the safety
//                   guards).
//   "BALX" -> AUTO  defer to the autonomous policy (balances in Charge when
//                   imbalanced).
// Dead-man: WarioCharger re-sends the active command ~2 Hz. If the frame goes
// stale (> BalanceOverrideFreshMs) OR was never seen, the effective command
// falls back to OFF, so a dead WarioCharger link never leaves the pack
// bleeding. Only ever affects balancing -- never an AIR / safety path.
inline constexpr std::uint32_t BalanceOverrideReqId  = 0x103u;  // standard; operator -> AMS. COMMISSION (confirm vs ECU map)
inline constexpr std::uint8_t  BalanceOverrideReqDlc = 4u;
inline constexpr std::uint8_t  BalanceCmdOffMagic [4] = { 0x42u, 0x41u, 0x4Cu, 0x4Fu };  // "BALO" -> OFF
inline constexpr std::uint8_t  BalanceCmdOnMagic  [4] = { 0x42u, 0x41u, 0x4Cu, 0x4Eu };  // "BALN" -> ON (any state)
inline constexpr std::uint8_t  BalanceCmdAutoMagic[4] = { 0x42u, 0x41u, 0x4Cu, 0x58u };  // "BALX" -> AUTO
inline constexpr std::uint32_t BalanceOverrideFreshMs = 5000;   // fall back to OFF if silent this long

// Effective operator balancing command. VehicleService resolves the raw last-
// seen command through the freshness dead-man into one of these (stale/never
// -> Off); balance::compute_mask consumes it.
enum class BalanceCmd : std::uint8_t { Off = 0, Auto = 1, On = 2 };

// 0x104 Operator_balance_modules -- PER-MODULE balancing enable, layered UNDER
// the 0x103 master switch. magic "BALM" (bytes 0..3) + byte 4 = 5-bit enable
// mask (bit m == 1 -> module m may balance). Re-send ~2 Hz. Dead-man: when the
// frame is stale (> BalanceModulesFreshMs) OR was never seen, EVERY module is
// enabled (BalanceModulesDefaultMask) -- so the pack behaves as before (global
// OFF/ON/AUTO only), and the 0x103 dead-man remains the safety net that stops
// balancing on a dead link. Only ever narrows which modules balance; never an
// AIR / safety path.
inline constexpr std::uint32_t BalanceModulesReqId   = 0x104u;  // standard; operator -> AMS
inline constexpr std::uint8_t  BalanceModulesReqDlc  = 5u;
inline constexpr std::uint8_t  BalanceModulesMagic[4] = { 0x42u, 0x41u, 0x4Cu, 0x4Du };  // "BALM"
inline constexpr std::uint32_t BalanceModulesFreshMs = 5000;    // stale -> all modules enabled
inline constexpr std::uint8_t  BalanceModulesDefaultMask = AllModulesMask;  // 0x1F, all enabled

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

// FDCAN1 Bus-Off recovery rate-limit. On sustained TX errors the M_CAN
// latches Bus_Off (CCCR.INIT set), which halts BOTH TX and RX -- the node
// goes silent and stays deaf until a software Stop->Start. AcuCanTask
// polls HAL_FDCAN_GetProtocolStatus every loop pass (a cheap PSR read)
// and, on Bus_Off, issues a Stop/Start no more than once per
// FdcanBusOffRetryMs. The spacing matters: the M_CAN's automatic recovery
// rejoins only after 128*11 consecutive recessive bits (~2.8 ms of idle
// bus at 500 kbps), so a Stop/Start every poll would keep restarting that
// sequence and the node would never finish rejoining. 100 ms mirrors the
// bootloader's BL_FDCAN_BUSOFF_RETRY_MS (../stm32-can-bootloader). See
// ams::can_recovery::should_attempt_recovery and acu_can_task.cpp.
inline constexpr std::uint32_t FdcanBusOffRetryMs    = 100;

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
inline constexpr std::uint32_t PitDiagCommsHealthId      = 0x6C9u;  // FDCAN1 Bus-Off recovery count + ECU-TX fail (#331)
// UNGATED firmware-health frame (#411): always-on 1 Hz, NEVER gated by the
// pit-diag arm (0x7F0). ID sits right after the gated 0x6C0..0x6C9 block but
// is emitted regardless of arm state -- parity with ECU 0x704 for passive
// liveness ("is the AMS app alive?" without transmitting an arm frame).
inline constexpr std::uint32_t FwHealthId                = 0x6CAu;
inline constexpr std::uint32_t FwHealthPeriodMs          = 1000u;  // 1 Hz
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
//
// Flight-carrier re-cal (2026-06-20): on the assembled-car AMS the zero
// measured 2054 (HIL carrier was 2050) -- the offset tracks VREF+, so it
// is board-specific. The 46 gain read back EXACT against an aux-PSU known
// current, so only the zero moved. Re-measure per carrier.
inline constexpr std::uint16_t AdcVrefMv          = 3300;
inline constexpr std::uint16_t AdcMaxCount        = 4095;
// Pack channel (differential ADC3_INP3/INN3 = PF7/PF8). HIL-commissioned.
inline constexpr std::int32_t  CurrentZeroCount   = 2054;  // diff zero @ 0 A (flight carrier; HIL #348 was 2050)  COMMISSION
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
// Simultaneous dischargers per module. This is a BOARD DISSIPATION limit, not
// a policy one -- compute_mask is stateless and re-picks the top-N by excess
// every second, so every imbalanced cell is bled either way. Raising this makes
// balancing proportionally FASTER; it does not unlock cells that were stuck.
//
// BMS_LITE per-cell bleed path (schematic, per-cell sheet): external TSM2323
// PMOS switching R71 || R72 = 47R || 47R = 23.5 R, both 2512.
//
//   cell V   current   W per cell   W per 2512 (2 W part)
//   4.2 V    179 mA    0.75 W       0.37 W   (~19 % of rating)
//   4.0 V    170 mA    0.68 W       0.34 W
//   3.7 V    157 mA    0.58 W       0.29 W
//
// Board / pack totals at 4.2 V (worst case):
//
//   MaxActive    per module    all 5 modules
//        4        3.0 W          15 W        (previous)
//        8        6.0 W          30 W        <-- here
//       19       14.3 W          71 W        (all cells; not attempted)
//
// The resistors are the comfortable part -- they are 2 W devices running at
// ~0.37 W, under a fifth of rating. The constraint is heat OUT OF THE
// ACCUMULATOR BOX, and that is unchanged by the part rating: 8 simultaneous
// dischargers is 6.0 W per module and 30 W across the pack however good the
// resistors are.
//
// COMMISSION: still not measured. Watch cell/board temperature on a bench run
// at this setting before trusting it in a sealed box, and note that the
// BalanceTempMax lockout that would catch an overheating board reads the same
// unvalidated NTC path (see BalanceTempsTrusted).
inline constexpr std::uint8_t  BalanceMaxActive   = 8;     // cells per module discharging at once

// Never discharge two PHYSICALLY ADJACENT cells at once, so the 2512 balance
// resistors never form a hot cluster on the board. Adjacency is derived from
// the BMS_LITE layout in balance::physically_adjacent (consecutive index within
// an LTC half). Measured pad temperature is ~71 C at 8/module concentrated;
// spreading keeps neighbours cold over the multi-hour C/101 balancing session.
//
// May reduce the active count below BalanceMaxActive when imbalanced cells
// cluster -- that is the intended, safe outcome (less heat, skipped cells bleed
// on later cycles). The index->board-position map is BENCH-VERIFIED (2026-07-22,
// IR on the real pack -- see physically_adjacent).
inline constexpr bool          BalanceSpreadNoAdjacent = true;

// Whether the cell-temperature path is trusted ENOUGH TO BALANCE ON.
//
// Deliberately separate from TempFaultsTrusted, which arms the FSM cell-temp
// FAULTS. The two ask different questions:
//
//   TempFaultsTrusted   -- "do we trust these temps enough to OPEN THE
//                          CONTACTORS on them?"  Answer: not yet. A misread
//                          from the unvalidated ADG731 mux path would trip the
//                          car for no reason, so the faults stay disarmed.
//   BalanceTempsTrusted -- "do we trust these temps enough to let balancing
//                          run?"  Balancing's only thermal protection is the
//                          BalanceTempMax lockout, which reads the same path.
//
// Coupling them meant the WarioCharger balance toggle (0x103) was accepted and
// then produced an all-zero mask forever -- balancing could never run, in any
// FSM state, on any image. Splitting them lets the operator switch work while
// temp FAULTS stay disarmed.
//
// ---------------------------------------------------------------------------
// RESIDUAL RISK -- read before changing this to true on a car.
// ---------------------------------------------------------------------------
// With this true and TempFaultsTrusted false, passive balancing dissipates into
// the cells while its ONLY thermal guard reads a path we have not validated.
// Unpopulated NTC slots read a plausible-looking ~25 C, so a genuinely hot cell
// whose sensor is mis-routed by the mux would NOT raise max_tempC and would NOT
// trip the lockout. The mitigations are: the 5 s operator dead-man
// (BalanceOverrideFreshMs) bounds an unattended run, only BalanceMaxActive
// cells per module bleed at once, and only cells >BalanceDeltaMv above the pack
// minimum are selected at all.
//
// Balance with cell temperatures observed by some other means until the ADG731
// mux path is validated end-to-end and TempFaultsTrusted itself goes true --
// at which point this flag becomes redundant and should be deleted.
inline constexpr bool          BalanceTempsTrusted = true;   // COMMISSION -- see residual risk above
inline constexpr std::uint32_t BalanceUpdatePolls = 4;     // = 1 Hz at BmsPollVoltMs = 250 ms

// Settle time after clearing the DCC bits and before starting a cell-voltage
// conversion, so no bleed current is flowing while the cells are measured.
//
// WHY THIS EXISTS -- the ADCV DCP=0 bit is not sufficient on this board.
// DCP=0 does make the LTC6811 suspend its own S-pin switch for the conversion,
// but BMS_LITE does not bleed through that switch: each cell drives an EXTERNAL
// TSM2323 PMOS whose gate sits behind R167 (10k) / C32 (10n), tau ~100 us. The
// conversion starts immediately on ADCV, and the first channels convert in a
// few hundred microseconds -- the same order as the gate turn-off -- so the
// earliest cells can be sampled while current is still flowing.
//
// WHY IT MATTERS: the bleed current does not return through the sense path on
// the board (on-board sensing is close to Kelvin), it returns through the
// harness. 179 mA across a plausible 50-200 mOhm of tap/connector/fuse
// impedance is 9-36 mV, and it has OPPOSITE SIGN on the bled cell (reads low)
// and its neighbours (read HIGH, because the shared tap node moves). Against
// BalanceDeltaMv = 50 mV that is a first-order corruption of the very signal
// balancing selects on -- observed on the bench as neighbouring cells reading
// high whenever balancing is active.
//
// 2 ms is ~20x the gate RC and covers the LTC input-filter settle, at a cost of
// under 1 % of balancing duty. Cheap insurance on a C/101 balancer where a
// wrong selection wastes hours.
inline constexpr std::uint32_t BalanceQuiesceMs   = 2;

// FDCAN1 TX FIFO slots kept free for the flight telemetry matrix while a LOGFS
// reply is being shipped (#449).
//
// A pull is a MULTI-MINUTE operation. pump_diag_tx() used to fill the 16-deep
// FIFO to zero free slots, and it runs before the telemetry scheduler in the
// same loop pass -- so for the whole transfer the flight matrix found no slots
// and send_or_fail dropped pack currents / voltages / temps SILENTLY (the only
// evidence being g_acu_tx_fail, which is itself best-effort). Diag also wins
// arbitration: 0x011/0x012 out-prioritise every AMS telemetry ID.
//
// It fails safe -- a dropped ok_precharge means no R2D, never a spurious one --
// but an invisible telemetry blackout is a debugging trap. Reserving 6 of 16
// still lets diag move ~10 frames per 1 ms pass, far more than the ~1 frame per
// pass the transfer actually needs.
inline constexpr std::uint8_t  DiagTxReservedSlots = 6;

// LTC6811 ADCV / ADAX mode + settling budget. Mode 2 ("Normal",
// 7 Hz first stage) is the canonical choice for race-pack
// metrology: ~2.3 ms to convert all 12 cell channels with the
// default filter. We round up to 3 ms in software (AdcvSettleMs)
// so jitter from the FreeRTOS tick doesn't clip the conversion.
// ADAX (AUX) under the same mode finishes in ~200 us per channel
// pair plus a settling allowance.
inline constexpr std::uint8_t  AdcMode          = 2;   // ams::ltc6811::AdcMode::Norm7kHz
inline constexpr std::uint32_t AdcvSettleMs     = 3;

// Voltage-poll retry budget. A voltage poll is re-attempted up to this many
// EXTRA times if it does not come back fully PEC-clean, before being counted a
// failed poll. Absorbs brief EMI bursts (e.g. inverter switching noise) that
// corrupt one read but clear on an immediate re-read, so a transient does not
// starve a module toward BmsStale. Each attempt is ~5 ms (ADCV + settle +
// reads); at 2 retries the worst case (3 attempts) is ~15 ms, well inside the
// 50 ms voltage-poll budget. Each attempt digests whatever ICs are clean, so
// retries give the stragglers more chances. A dead/asleep chain fails every
// attempt and still counts as failed. 0 = no retry (legacy behaviour).
inline constexpr std::uint8_t  VoltPollRetries  = 2;
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
// jump reason. Slot 3: last-fault sentinel (#411). Slot 4+ reserved.
inline constexpr std::uint32_t BkpJumpReasonReg = 2;

enum class JumpReason : std::uint32_t {
    None           = 0x00000000u,
    CanTrigger     = 0x4A554D50u,  // 'JUMP' -- MingoCAN sent the boot frame
    FaultLatch     = 0x46415554u,  // 'FAUT' -- safety supervisor forced it
    ManualRequest  = 0x4D414E55u,  // 'MANU' -- operator-issued, no fault
};

// --- Firmware-health frame (0x6CA, #411) -----------------------------------
// Slot 3 holds the last-fault sentinel: the HardFault handler stamps a
// LastFault code here, the 0x6CA health frame surfaces it on byte [7], and a
// clean boot clears it. Same backup-domain persistence as slots 0-2 (survives
// IWDG / NVIC reset, cleared on POR).
inline constexpr std::uint32_t BkpLastFaultReg = 3;

// reset_cause byte [5] -- mirrors the ECU 0x704 reset_cause enum exactly.
enum class ResetCause : std::uint8_t {
    Unknown = 0u, PowerOn = 1u, Pin = 2u, Software = 3u,
    Iwdg = 4u, Wwdg = 5u, LowPower = 6u,
};

// last_fault byte [7] sentinel, latched in BkpLastFaultReg across a reset.
// 0 = clean; the rest map the HardFault / RTOS-hook fault classes.
enum class LastFault : std::uint8_t {
    None = 0u, HardFault = 1u, StackOverflow = 2u, MallocFail = 3u, AssertFail = 4u,
};

// AMS node ID on the stm32-can-bootloader multi-node bus. Must match
// the value the BL was compiled with (-DBL_NODE_ID=<n>). Embedded in
// the firmware_info `reserved[0]` slot so MingoCAN can verify at
// flash time that the app it's writing matches the BL it's talking
// to. Changing this requires re-building both halves.
//
// 2026-05-18: originally 0x01 (MLC1 single-node bring-up; factory NVM),
// confirmed by IFS08_HIL#30 turning A-002/A-003 green.
// #403: the shared-bus role map (can-flasher provision) is ECU=1,
// AMS=2, uDV=3 -- 0x01 is the ECU's slot, so on a shared bus the AMS
// collided with the ECU. The AMS moves to 0x02. The flight board's BL
// MUST be re-provisioned to node 2 (-DBL_NODE_ID=2 and/or NVM provision)
// before flashing this firmware -- both halves change together. Bench
// (feat/bms-stub-charge) already runs at 0x02.
inline constexpr std::uint32_t AmsNodeId = 0x02u;

// Application flash base. Must match STM32H733XG_FLASH.ld's FLASH
// ORIGIN and the bootloader's BL_APP_BASE.
inline constexpr std::uint32_t AppFlashBase    = 0x08020000u;

// AMS telemetry TX on FDCAN1. Three single-purpose 8-byte frames at
// 500 ms cadence each. See docs/CAN_MAP.md for the byte layouts.
inline constexpr std::uint32_t AmsTelemStatusId = 0x4A0u;  // state + cell-V extremes
inline constexpr std::uint32_t AmsTelemPackId   = 0x4A1u;  // pack V + current
inline constexpr std::uint32_t AmsTelemTempsId  = 0x4A2u;  // temp extremes + dc bus + heartbeat
inline constexpr std::uint32_t AmsTelemDiagId   = 0x4A3u;  // diagnostic probes (#123); pure-fn encoder
inline constexpr std::uint32_t AmsRelayStatusId = 0x4A4u;  // contactor + AMS_OK GPIO read-backs (always-on)

// ---------------------------------------------------------------------------
// LTC6811-1 + isoSPI BMS chain. New AMS PCB drives a chain of 10 LTCs
// (5 BMS modules × 2 LTCs each) via SPI1 + LTC6820 isoSPI master. See
// docs/BMS_LTC6811.md for the wire protocol and slot mapping.
// ---------------------------------------------------------------------------
inline constexpr std::uint8_t  LtcsPerModule       = 2;
inline constexpr std::uint8_t  CellsPerLtcUpper    =  9;  // LTC_1 (first in chain) -- 9 cells -> module 0..8 (#423)
inline constexpr std::uint8_t  CellsPerLtcLower    = 10;  // LTC_2 (second in chain) -- 10 cells -> module 9..18 (#423)
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
//   R_ntc = NtcPullupOhm * V_aux / (NtcVrefMv - V_aux)
//
// Temperature then comes from the manufacturer R-T table
// (Core/Inc/app/ntc_table.hpp, generated from docs/ntc_rt_table.csv), NOT a
// single-beta Steinhart fit.
//
// NTC divider + part. Conversion is a TABLE lookup (ntc_table.hpp), not a
// single-beta Steinhart fit -- see that header for why.
//
// NtcPullupOhm was NtcSeriesR = 10000, which was wrong: the divider pull-up on
// BMS_LITE is R145 / R170 = 6.8 kOhm. Combined with a beta borrowed from a
// different part (3380 vs the fitted 3950) the two errors partially cancelled
// and the whole path read ~7 degC COLD at 50 degC. Renamed as well as
// corrected so a stale "series resistor" mental model cannot survive the fix.
inline constexpr std::uint32_t NtcPullupOhm = 6800;   // R145 / R170 pull-up to VREF2
inline constexpr std::uint16_t NtcVrefMv    = 3000;   // LTC6811 VREF2 nominal

// Open-circuit detection threshold, in AUX millivolts.
//
// A disconnected NTC leaves the divider node pulled up through NtcPullupOhm
// toward VREF2. Ideally it rails to ~3.0 V, but a partially-railed open -- mux
// leakage, a long or damp sense harness, or a high-impedance fault -- can settle
// a few hundred millivolts below the rail (~2.7-2.95 V observed). Read literally
// that decodes to a very cold BUT in-range temperature (2.9 V ~= -35 degC), so a
// bare `>= NtcVrefMv` open test lets a real disconnect masquerade as a plausible
// cold reading and slip past the presence check.
//
// So treat anything at or above NtcOpenMv as OPEN rather than cold. 2800 mV maps
// to ~-20 degC on the 6.8 k / VREF2 divider -- colder than any operating cell
// (and well below the ~-10 degC a cold-soaked pack could see), leaving >=140 mV
// margin so a genuinely cold NTC never trips it while a floating node reliably
// does. Must stay below NtcVrefMv (keeps the divider denominator positive in
// ntc_mV_to_tempC). See ntc_mV_to_tempC.
inline constexpr std::uint16_t NtcOpenMv    = 2800;   // >= this AUX mV => open, not cold
static_assert(NtcOpenMv < NtcVrefMv, "open threshold must sit below VREF2");

// Plausibility window for accepted NTC readings. Anything outside
// this range is dropped (slot left at its previous value) so an
// unpopulated mux channel can't drive max_tempC into orbit and trip
// safety::ForceError on a clean pack.
// Sentinel stored in cell_tempC for a channel that has never produced a valid
// reading -- unpopulated mux input, open/shorted NTC, or a PEC-failed poll.
//
// This used to be a seeded 25 degC, which is the single most dangerous value it
// could have been: unpopulated channels read as comfortably room temperature,
// so max_tempC looked healthy no matter what the pack was doing, and every
// threshold built on it was defeated regardless of how accurate the conversion
// was. A sentinel makes "no data" distinguishable from "cool".
inline constexpr std::int16_t NtcNoReading = -32768;   // INT16_MIN

// Temperature-sensor DISCONNECT fault (FS rule: a disconnected temp sensor must
// open the SDC). A sensor that has read valid at least once and then reads OPEN
// (rail voltage -> NtcNoReading) for TempDisconnectPolls consecutive temp polls
// is treated as disconnected and latches ERROR, exactly like a missing module
// faults on the voltage side.
//
// This does NOT depend on temperature ACCURACY -- an open NTC reads the rail
// regardless of the beta/pull-up calibration -- so it is armed independently of
// TempFaultsTrusted (which gates the range over/under-temp faults).
//
// The debounce tolerates a single anomalous mux read: a genuine open is stable,
// a one-off is not. 2 polls x BmsPollTempMs (500 ms) ~= 1 s to detect, tunable.
// COMMISSION: confirm the detection time meets the applicable rule window.
inline constexpr bool         TempSensorPresenceCheck = true;
inline constexpr std::uint8_t TempDisconnectPolls     = 2;

// REQUIRED temperature channels: cell-temp slots that MUST be present in every
// online module. A required slot reading OPEN faults immediately -- WITHOUT the
// "seen valid once" latch -- so a sensor whose switch is already open at power-on
// (or after a reset that cleared the seen-valid state) is still caught. This is
// what makes the disconnect deterministic for scrutineering, where an
// open-at-boot channel is otherwise indistinguishable from an unpopulated one.
//
// Slot numbering: the ADG731 sweep stores LTC_1 (upper) temps in slots 0..19 and
// LTC_2 (lower) in 20..39. "Temperature 1 of LTC_1" = slot 0. The scrutineering
// demo switches exactly that channel on each module, so slot 0 is required.
//
// FULL populated map (all 40 slots) -- verified from the BMS_LITE schematic:
// LTC_1 mux U4 carries NTC_1..NTC_20 (S1..S10 + S17..S26) -> slots 0..19, and
// LTC_2 mux U5 carries NTC_21..NTC_40 (same channels) -> slots 20..39. Every
// slot on both LTCs is populated on all 5 modules, so ANY open cell-temp sensor
// -- at boot or after -- now opens the SDC, per the FS rule.
//
// CONSEQUENCE (COMMISSION / HIL): this enforces all 40 slots on every online
// module immediately. A genuinely open channel on the flight harness (e.g. the
// known M3 upper-LTC open) will now correctly LATCH ERROR at boot until it is
// repaired -- that is the intended behaviour, but it means the harness must be
// healthy for the pack to arm. Validate on the bench before flight.
inline constexpr std::uint8_t RequiredTempSlots[]   = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9,   // LTC_1 NTC_1..NTC_10
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,   // LTC_1 NTC_11..NTC_20
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,   // LTC_2 NTC_21..NTC_30
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,   // LTC_2 NTC_31..NTC_40
};

// Minimum valid cell-temp channels before balancing may run at all.
//
// COMMISSION: deliberately LOW. Its job today is to catch a completely dead
// temperature path, not to guarantee coverage -- and setting it above the
// number of channels actually populated would silently disable balancing,
// which is the exact failure mode this whole area keeps producing. The board
// has up to 200 channels (5 modules x 40) but how many are fitted is unknown
// and the LTC_2 half may not be wired at all.
//
// RAISE THIS to the measured populated count once a bench sweep establishes it
// (M1). BmsState::valid_temp_channels is what to read.
inline constexpr std::uint16_t BalanceMinValidTempCh = 5;

inline constexpr std::int16_t NtcMinValidC = -40;
inline constexpr std::int16_t NtcMaxValidC = 150;

}  // namespace ams::config
