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
// Pack thresholds (FS rules). See docs/ARCHITECTURE.md §3 (task architecture)
// and docs/COMMISSIONING.md §1 for the procedure to finalise these.
//
// COMMISSION: these are placeholder defaults. Finalise per cell
// datasheet + FS rules before the car runs.
// ---------------------------------------------------------------------------

inline constexpr std::uint16_t CellUnderVoltageMv =  2800;  // under-voltage   -- COMMISSION
inline constexpr std::uint16_t CellOverVoltageMv =  4200;  // over-voltage    -- COMMISSION

// Cell OPEN-WIRE detection via the LTC6811 ADOW command: a two-pass (pull-up /
// pull-down) conversion in BmsPollTask that faults CellOpenWire in ANY state.
// Detector: open_wire.hpp. Command encoding: ltc6811.hpp.
//
// The ONLY predicate that can see a broken cell tap. An open node floats, so the
// two cells sharing it rail high and low with their SUM conserved -- exactly the
// signature the tap-artifact guard in recompute_summaries_ averages back into
// range. Cell over/under-voltage therefore cannot fire on an open tap: on the
// bench, a cell reading 2364 mV reached the FSM as 3823 mV.
//
// Margin on a live pack: a real open reads about -4000 mV against the 400 mV
// threshold, roughly 10x. A healthy pack stays inside -130..+50 mV.
//
// GAP: only INTERIOR conductors are hardware-validated. The endpoint rules
// (C(0) via CELL_PU(1)==0, C(N) via CELL_PD(N)==0) test for EXACT zero and have
// never run on hardware -- an endpoint open reading a few mV instead of 0 would
// be missed. That is ~2 of every 10 conductors per IC.
inline constexpr bool          CellOpenWireCheck   = true;
inline constexpr std::uint16_t CellOpenWireDeltaMv = 400;   // datasheet open threshold
// ADOW retries within a single voltage poll (mirrors VoltPollRetries). Both ADOW
// passes must be PEC-clean on an IC to judge its open-wire state, so one PEC
// glitch would skip that IC and slip the CellOpenWire fault to the NEXT poll
// (+BmsPollVoltMs), pushing detection past 500 ms. Re-running the two-pass scan
// when any IC was skipped absorbs the glitch IN THE SAME poll, keeping detection
// < 500 ms. 1 retry (2 attempts) bounds the added time at ~2 x two ADOW passes.
inline constexpr std::uint8_t  OpenWireRetries     = 1;
// BENCH DIAGNOSTIC: dump the raw ADOW pull-up / pull-down per-cell readings over
// pit-diag, to debug the ADOW encoding and conversion timing on a real chain
// (compare PU vs PD on a known open). Runs its own two-pass ADOW scan in
// BmsPollTask INDEPENDENT of CellOpenWireCheck, so it can be debugged with live
// detection off. Keep FALSE on dev/flight -- the code stays compiled
// (dead-code-eliminated when false, so CI still type-checks it) but emits
// nothing. Blocks: PU @ AdowDiagPuBaseId, PD @ AdowDiagPdBaseId, same 24-frame
// 4-cell BE-u16 layout as the 0x680 cell grid.
inline constexpr bool          AdowRawDiag         = false;

// BENCH DIAGNOSTIC: re-measure the whole pack in a SECOND ADC mode, so a
// settling-limited tap can be told apart from a genuinely low cell.
//
// Each cell input sits behind an RC filter at the LTC pin. Series resistance in
// the tap conductor -- a cold crimp, a corroded ring terminal, a cracked joint --
// raises that time constant, so a fast conversion samples before the input has
// settled and the cell reads LOW, while a slow one settles fully and reads true.
//
//   good tap            same value in both modes
//   resistive tap       fast mode reads low, filtered mode reads true
//   genuinely low cell  same value in both modes, both low
//
// RDSTATA cannot make this call: sum-of-cells is referenced to the same C0 node
// as cell 1, so an offset there shifts SC and cell 1 by equal amounts and the sum
// still reconciles. Comparing modes does not depend on C0.
//
// Keep FALSE on dev/flight -- the code stays compiled (dead-code-eliminated when
// false, so CI still type-checks it) but emits nothing. Block: AdcXCheckBaseId,
// same 24-frame 4-cell BE-u16 layout as the 0x680 cell grid, 0xFFFF = PEC-skipped.
inline constexpr bool          AdcModeCrossCheck   = false;
// Mode to compare against config::AdcMode. 3 = Filt26Hz, the datasheet's answer
// to high source impedance and so the widest contrast against the 7 kHz live
// mode. 0 (Slow422Hz, 12.8 ms) is the cheap alternative if 26 Hz costs too much
// poll time on a given chain.
inline constexpr std::uint8_t  AdcXCheckAdcMode    = 3;   // ltc6811::AdcMode::Filt26Hz
// Conversion budget for that mode: Filt26Hz needs ~201 ms for all 12 channels,
// rounded up for FreeRTOS tick jitter the same way AdcvSettleMs does.
inline constexpr std::uint32_t AdcXCheckSettleMs   = 210;
// Polls between cross-check sweeps. The sweep runs AFTER the normal read, so
// module freshness is never delayed by it -- only the next poll slips, stretching
// the worst-case gap between fresh readings to about BmsPollVoltMs +
// AdcXCheckSettleMs. That must stay under BmsStaleMs or the sweep would trip
// BmsModuleOffline on its own; the static_assert below holds the line. 25 polls
// = 5 s, enough to watch a suspect cell over a few minutes without spending a
// fifth of the task's time in filtered conversions.
inline constexpr std::uint32_t AdcXCheckPolls      = 25;

// Implausible-cell bounds for the balancing tap-artifact guard (see
// recompute_summaries_ / BmsState::tap_fault_mask). A cell in a live pack
// physically cannot sit outside this window, so a reading beyond it is a
// MEASUREMENT error, not a true voltage. When two PHYSICALLY-ADJACENT cells
// straddle a shifted shared tap node (balancing current through a high-R tap),
// one reads impossibly high and its neighbour compensates low while the
// tap-immune PAIR SUM stays in the normal window; the guard then feeds the pair
// AVERAGE to the safety aggregates instead of the impossible individual value.
// Deliberately WIDER than CellOver/UnderVoltageMv: a genuine over-/under-voltage
// (e.g. 4200 < v < 4400) sits beside a NORMAL neighbour, so the sum runs high
// rather than conserved, and it is never masked -- it still faults.
inline constexpr std::uint16_t CellImplausibleMaxMv = 4400;  // > any charging cell (4.2 V CV limit)
inline constexpr std::uint16_t CellImplausibleMinMv = 1500;  // < any cell in a connected live pack
// Minimum intra-pair split (|hi - lo|) before the tap-artifact guard engages.
// Series-adjacent cells track closely even in a badly imbalanced pack; a split
// this large only appears when a shared tap node has shifted. With the
// implausible-cell and conserved-sum tests, requiring it means a single-cell
// glitch beside a NORMAL neighbour is never averaged away -- it faults.
inline constexpr std::uint16_t TapArtifactMinSplitMv = 800;
inline constexpr std::int16_t  CellUnderTempC  =   -10;  // under-temp °C   -- COMMISSION
inline constexpr std::int16_t  CellOverTempC  =    60;  // over-temp °C    -- COMMISSION
// Cell TEMPERATURE fault gate. NTC temps come through the per-LTC ADG731 32:1
// mux, a path NOT yet trusted on flight (the mux-select word was wrong; fixed on
// the bench harness, not yet validated in the flight path). While false, the
// CellUnderTemp / CellOverTemp predicates are SUPPRESSED so the FSM never faults
// on unvalidated temperatures. Cell VOLTAGE protection is unaffected. Set true
// once the mux fix ships to flight and temps are bench-validated end-to-end.
inline constexpr bool          TempFaultsTrusted = false;
// Pack over-current trip. |filtered_mA| above this latches CurrentOverLimit,
// which opens the AIRs and drops AMS_OK.
//
// 185 A is the 6P continuous rating of the cells: 95S6P of VTC6 at 30 A
// continuous each, so a series element sustains 6 x 30 = 180 A. The sensor is a
// Bourns SSA-2-250A, so the limit sits inside what the front end can measure.
//
// NO debounce on this check -- the smoothing comes entirely from the filter
// feeding it. filtered_mA is a first-order IIR with CurrentFilterShift=4 at
// CurrentPeriodMs=50, i.e. tau ~ 800 ms, so trip time is tau*ln(I/(I-limit)):
//
//     200 A -> 2.1 s     250 A -> 1.1 s     300 A -> 0.8 s     400 A -> 0.5 s
//
// A brief inrush therefore rides through while a sustained overload still opens
// the SDC in about a second. Corollary: currents between the cell rating and
// this limit never trip at all, by design -- a slow overload is caught by the
// cell temperature path, not here.
//
// COMMISSION: derived from the cell datasheet, NOT measured against the car's
// real draw or validated against the contactor and fuse ratings. Confirm the
// inverter peak and the SDC element ratings before trusting it.
inline constexpr std::int32_t  CurrentMaxMa   = 185000; // 6P continuous -- COMMISSION

inline constexpr std::uint32_t IStaleMs       =  200;  // pack current sensor stale (safety-critical)
inline constexpr std::uint32_t DcdcIStaleMs   =  500;  // DCDC current sensor stale (informational; not safety-gated -- the HW front-end is a separate single-ended sensor on PC1 and DCDC failure is recoverable)
// Staleness window for a single BMS module. "Stop measuring ANY voltage/
// temperature" includes a whole module going silent, which must open the SDC in
// < 500 ms (FS rule). A silent module drops off module_online_mask when its
// freshness exceeds this, firing BmsModuleOffline (immediate, not debounced).
// Worst case = 350 ms staleness crossed at the 2nd voltage poll after loss
// (~400 ms) + a 10 ms safety tick ~= 410 ms. 350 > BmsPollVoltMs (200 ms), so
// ONE missed poll is tolerated (age 200 <= 350) and two consecutive misses
// (400 ms) trip it. SAFE ONLY with bounded poll jitter -- the temp sweep does
// not head-of-line-block the voltage poll (see run_temperature_poll's
// yield-to-PollVDue). COMMISSION: confirm no nuisance trips and the exact
// margin on the HIL bench before flight.
inline constexpr std::uint32_t BmsStaleMs     = 350;   // any BMS module silent (< 500 ms fault-response)
inline constexpr std::uint32_t VcuStaleMs     =  200;  // VCU 0x100 stale
// Charger heartbeat (0x101) stale window while in Charger mode. WarioCharger
// re-sends 0x101 at >= 2 Hz (<= 500 ms period), so 1000 ms tolerates one missed
// heartbeat before faulting Charge -> Error; charging is not a 10 ms-critical
// response, so a single dropped frame should not trip it. Matches
// ChargeReqFreshMs. See safety_predicates FaultReason::ChargerStale.
inline constexpr std::uint32_t ChargerStaleMs =  1000; // charger 0x101 stale (Charger mode only)
// At Start->Precharge (TSMS+DASH_CHG asserted) the FSM asks "has a VCU 0x100
// frame arrived within VcuFreshMs?" to decide whether the pack is in the car
// (Run target) or at the charger (Charge target). Looser than VcuStaleMs so a
// slow VCU bring-up is not misclassified as charger. The mode is locked at that
// transition and never re-evaluated.
inline constexpr std::uint32_t VcuFreshMs     = 1000;

// Precharge deadline. If the bus does not reach the precharge target within this
// window the FSM latches Error and opens every contactor, bounding how long the
// precharge contactor + resistor can be held closed. The resistor is rated for
// transient duty, not steady-state, and this covers ANY stuck-precharge cause: a
// stuck contactor, no charger, a bus fault, or a car with a dead VCU that locks
// Charger mode and would otherwise sit in Precharge forever, since `dc_bus_V`
// (the precharge-complete input) comes only from the VCU's 0x100. A normal
// precharge completes well under 1 s; this is the failsafe ceiling.
inline constexpr std::uint32_t PrechargeMaxMs = 5000;  // COMMISSION (resistor thermal limit)

inline constexpr std::uint8_t  AllModulesMask = 0x1F;  // 5 modules present

// ---------------------------------------------------------------------------
// Task periods (ms). See docs/ARCHITECTURE.md §3 task table.
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t SafetyPeriodMs    =  10;

// Window after osKernelStart in which SafetyTask suppresses the freshness /
// range predicates that depend on external data (BMS responses, ADC samples,
// VCU heartbeat). At t=0 every service's `last_*_tick` is 0, so without a grace
// the first SafetyTask iteration would fault, withhold the watchdog refresh and
// IWDG-reset within ~100 ms. The immediate-safety predicates (FORCE_ERROR, SDC
// open) stay active; only data-presence predicates are suppressed.
//
// Must be >= the longest service warm-up:
//   BmsPollTask first voltage poll      BmsPollVoltMs (200 ms)
//   CurrentSensorTask first ADC sample  CurrentPeriodMs (50 ms)
//   AcuCanTask first VCU 0x100          uncontrolled, but typically already
//                                       present on the vehicle bus
// 2000 ms leaves generous margin for all of the above and slow CAN startup;
// tune down if a faster boot becomes critical.
inline constexpr std::uint32_t SafetyBootGraceMs = 2000;

// Cell V/T range debounce. The cell-voltage / temperature range predicates must
// persist this many consecutive SafetyTask evaluations (× SafetyPeriodMs =
// 10 ms) before latching the sticky ERROR, so a single sub-threshold sample -- a
// torn lock-free snapshot read, or an unsettled first poll / emulator-default
// value at boot -- cannot permanently fault the chip.
//
// 25 ticks ~= 250 ms, deliberately MORE than one BmsPollVoltMs (200 ms) cycle: a
// transient that clears on the next voltage poll never reaches the count, while
// a sustained real under/over condition does. Worst case for an out-of-range
// cell: <= 200 ms (next poll) + 250 ms (confirm) + a 10 ms tick ~= 460 ms,
// inside the < 500 ms budget. (An open tap that reads plausibly IN-range is
// caught only by ADOW, config::CellOpenWireCheck.)
//
// Only the cell V/T branches are debounced -- force-error, BMS offline/stale,
// current-over-limit and VCU-stale still latch on the first tick.
inline constexpr std::uint16_t CellFaultConfirmTicks = 25;  // ~250 ms

// BmsStale confirmation debounce. BmsStale (a BMS module silent past BmsStaleMs)
// is a timeout; requiring it to persist this many consecutive evaluations
// (x SafetyPeriodMs = 10 ms) stops a far module that flickers just past the
// window under a brief EMI burst -- and reports on its next voltage poll
// (<= 250 ms later) -- from spuriously opening the contactors. 25 x 10 ms =
// 250 ms still spans more than one 200 ms voltage poll. Set to 0 to latch on the
// first tick.
//
// SECONDARY path: a genuinely lost module is caught FASTER by BmsModuleOffline,
// which drops it off module_online_mask the moment its freshness exceeds
// BmsStaleMs (350 ms) at a voltage poll and is immediate, not debounced
// (~410 ms worst case, < 500 ms). This per-module confirm only gates the
// freshness-loop predicate that BmsModuleOffline pre-empts, so it adds nothing
// to the module-loss detection budget.
inline constexpr std::uint16_t BmsStaleConfirmTicks = 25;  // ~250 ms  COMMISSION

inline constexpr std::uint32_t StatePeriodMs     =  20;
inline constexpr std::uint32_t CurrentPeriodMs   =  50;
inline constexpr std::uint32_t AcuHeartbeatMs    = 100;
// Cell-voltage poll period. An out-of-range cell must fault in < 500 ms (FS
// rule), and the range check needs one poll to observe it plus a confirm
// debounce spanning > one poll for glitch immunity: 200 ms poll +
// CellFaultConfirmTicks (~250 ms) + a 10 ms tick ~= 460 ms worst case.
// Knock-on: balance updates run ~1.25 Hz (BalanceUpdatePolls x 200 ms) and
// LogSamplePeriodMs (250) does not land exactly on a poll -- both non-safety.
inline constexpr std::uint32_t BmsPollVoltMs     = 200;
// Temperature sweep period. A disconnected temp sensor must fault in < 500 ms
// (FS rule); with TempDisconnectPolls = 1 the worst-case detect is one sweep
// cadence + the ~100 ms sweep + a 10 ms safety tick ~= 360 ms. The mux sweep
// shares BmsPollTask with the 200 ms BmsPollVoltMs voltage poll -- confirm the
// task keeps up (sweep ~100 ms) on the HIL bench.
inline constexpr std::uint32_t BmsPollTempMs     = 250;
inline constexpr std::uint32_t TelemetryPeriodMs = 500;
inline constexpr std::uint32_t RelayStatusPeriodMs = 100;  // 0x4A4 contactor snapshot (always-on)

// ---------------------------------------------------------------------------
// Datalogging to microSD (SDMMC1 + FatFs). Strictly off the safety path:
// SdLoggerTask (low priority) owns the card; SafetyTask only pushes a
// LogRecord into a lock-free ring every LogSamplePeriodMs. Best-effort -- a
// missing/failed card or a full ring degrades to "no log", NEVER a fault and
// never a blocking call on the 10 ms loop. The boot-path SDMMC init is
// decoupled (CubeMX: MX_SDMMC1_SD_Init not auto-called) so an absent card
// can't brick the node. Logger: Core/Src/app/sd_logger_task.cpp.
// ---------------------------------------------------------------------------

// SafetyTask captures a record this often. 250 ms = 4 Hz, matched to the
// BmsPollVoltMs voltage poll so every sample carries a FRESH cell-voltage frame
// rather than oversampling the 2-4 Hz BMS data. ~6 KB/s of full per-cell CSV.
// Must be a multiple of SafetyPeriodMs.
inline constexpr std::uint32_t LogSamplePeriodMs = 250;

// Lock-free ring depth (LogRecords). Each record carries the FULL 95-cell +
// 200-temp matrices (~620 B), so the ring is ~10 KB of BSS at depth 16. MUST be
// a power of two. 16 @ 4 Hz ~= 4 s of buffer; the shared SD mutex yields, so the
// logger drains between the extractor's reads and the ring rarely saturates.
// Bump if RAM allows.
inline constexpr std::uint32_t LogRingCapacity   = 16;

// SdLoggerTask drain cadence, and how often it f_syncs the active file
// (bounds data lost on a power-cut to <= LogSyncPeriodMs of samples).
inline constexpr std::uint32_t LogDrainPeriodMs  = 50;
inline constexpr std::uint32_t LogSyncPeriodMs   = 1000;

// Size cap per file: seal (rotate) the active file once it reaches this size.
// Bounded, rotated files make LOGFS listing / CRC / resume / "only new logs"
// tractable. A real 314-column row is ~1.35 kB (76 B of scalars + 95 cells +
// 200 temps), so at 4 Hz the card takes ~5.3 KiB/s and 4 MiB is ~13 MINUTES.
inline constexpr std::uint32_t LogFileMaxBytes   = 4u * 1024u * 1024u;  // 4 MiB (~13 min/file at full per-cell rows)

// Time cap per file. On the size cap alone, any run shorter than ~13 min leaves
// a .TMP that no tool treats as a finished log (the LOGFS extractor lists sealed
// files only). Rotating on time as well means an ordinary bench or test session
// produces real .CSV files while it runs, instead of one perpetual .TMP.
//
// 5 min ~= 1.6 MB per file: short enough that little is at risk if power is cut
// mid-file, long enough not to litter the card.
inline constexpr std::uint32_t LogFileMaxMs      = 5u * 60u * 1000u;    // 5 min

// 8.3 names (LFN off in ffconf.h; no RTC wall-clock). The active file is written
// as ".TMP" and renamed to ".CSV" on seal, so the extractor only ever sees
// finished logs. The index is a rotation counter, not a timestamp.
inline constexpr char          LogActiveNameFmt[] = "LOG%04lu.TMP";
inline constexpr char          LogSealedNameFmt[] = "LOG%04lu.CSV";
// CRC-32 sidecar written beside a sealed CSV: 8 ASCII hex digits.
// Kept out of the CSV itself so the log stays directly spreadsheet-openable.
inline constexpr char          LogCrcNameFmt[]    = "LOG%04lu.CRC";

// ---------------------------------------------------------------------------
// CAN map. Source of truth: docs/CAN_MAP.md. Frame-byte layout lives with
// the encode/decode helpers in can_frame.hpp.
// ---------------------------------------------------------------------------

// 5 BMS slave modules. Each module is a pair of LTC6811-1 ICs on the isoSPI
// daisy-chain (see LtcsPerModule / LtcChainLength below): module 0 == chain
// slots 0,1 ; module 4 == chain slots 8,9.
inline constexpr std::uint8_t  BmsModuleCount      = 5;
inline constexpr std::uint8_t  CellsPerModule      = 19;
inline constexpr std::uint8_t  TempsPerModule      = 40;  // 20 per LTC * 2 LTCs

// ---------------------------------------------------------------------------
// Pack energy + state-of-charge (soc_estimator.hpp). TELEMETRY ONLY -- no
// safety predicate reads any of this.
// ---------------------------------------------------------------------------
// Usable capacity of ONE SERIES ELEMENT, which is what Coulomb counting
// integrates against: the pack is 95S6P of Sony/Murata VTC6 (3.0 Ah nominal), so
// each of the 95 series positions is a 6-parallel group of 6 x 3.0 = 18.0 Ah.
// Every series element carries the full pack current, so pack SoC and element
// SoC are the same number -- no scaling by series count.
//
// Sourced from the TFM pack model (raulmoranguerra/TFM_RMG, BMS_DL/sim/params.py:
// CELL_CAPACITY_AH = 3.0, N_PARALLEL = 6, 95S6P), corroborated by this
// firmware's own "C/101 balancer" phrasing, which back-solves to ~18 Ah at the
// 179 mA balance current.
//
// COMMISSION: nominal datasheet capacity, NOT measured on this pack. Real usable
// capacity falls with age and with temperature; a 10 % error here is a 10 %
// proportional error in every Coulomb-counted SoC. Replace with a measured
// full-discharge figure once one exists.
inline constexpr std::uint32_t PackCapacityMah     = 18000;  // 6P x 3.0 Ah -- COMMISSION

// OCV anchoring gate. Terminal voltage only equals open-circuit voltage at rest;
// under load it carries I*R_int (~20 mOhm/cell fitted, so 10 A moves a cell
// ~200 mV -- worth ~20 SoC points on the flat part of the curve). Both
// conditions must hold before an anchor is taken.
//
// 500 mA is comfortably above sensor noise and the balancing bleed
// (~179 mA/cell), yet small enough that the residual I*R error is ~10 mV, about
// 1 SoC point mid-curve.
inline constexpr std::uint32_t SocRestCurrentMa    = 500;
// 5 min: the ohmic part of the polarisation recovers in microseconds, but the
// concentration gradient relaxes over minutes, so anchoring earlier reads low.
// COMMISSION: not characterised on this cell -- if anchors land consistently
// below a known-good reference, this is the first number to raise.
inline constexpr std::uint32_t SocRestSettleMs     = 300000;  // 5 min -- COMMISSION

// Longest gap the integrator accepts in one step; beyond it the sample is
// dropped rather than extrapolated. A gap that long means the task was starved
// or the counter was just anchored, and inventing charge across it is worse than
// losing that interval. 10x CurrentPeriodMs (50 ms).
inline constexpr std::uint32_t SocMaxIntegrationGapMs = 500;

// ---------------------------------------------------------------------------
// SoC Kalman filter -- equivalent-circuit measurement model.
// ---------------------------------------------------------------------------
// Coulomb counting alone re-anchors only after SocRestSettleMs at rest, which
// during a race session may never happen -- leaving pure integrator drift for
// the whole run. The EKF corrects CONTINUOUSLY from the voltage residual, and
// the Kalman gain does the weighting for free: it is proportional to dOCV/dSoC,
// so it leans on voltage where the OCV curve is steep (near full/empty) and on
// Coulomb counting across the flat plateau.
//
// Cell model, taken verbatim from the fitted VTC6 parameters in the TFM pack
// simulator (raulmoranguerra/TFM_RMG, BMS_DL/sim/params.py + pack_model.py):
//
//     R_int(T, SoC) = R_NOM * f_SoC(SoC) * f_T(T)
//     f_T(T)        = max(1 + ALPHA_R * (T - 25 degC), 0.4)     <- LINEAR, not exp
//
// Same characterisation that produced the network's training data, so firmware
// and dataset stay on one model.
inline constexpr std::uint32_t RIntNomMicroOhm    = 20060;   // R_NOM 0.020060 ohm, per CELL
inline constexpr std::int32_t  RIntAlphaMicroPerK = -19926;  // ALPHA_R -0.019926 /K
inline constexpr std::int16_t  RIntTempRefC       = 25;      // T_REF 298.15 K
inline constexpr std::uint8_t  CellsInParallel    = 6;       // 95S6P -- element R = cell R / 6

// f_SoC(SoC) piecewise-linear resistance shape (F_SOC_BP / F_SOC_VAL), x1000.
// Non-monotonic by nature: internal resistance rises at both extremes.
inline constexpr std::uint8_t  RIntSocPoints          = 7;
inline constexpr std::uint16_t RIntSocBpPermille[RIntSocPoints] =
    {   0,  100,  200,  500,  800,  900, 1000 };
inline constexpr std::uint16_t RIntSocValMilli[RIntSocPoints] =
    {1033, 1043, 1030,  986, 1051, 1006, 1009 };

// --- Filter tuning. All COMMISSION: derived from first principles below, none
// --- fitted against a measured SoC reference on this pack.
//
// P0 -- initial variance. sigma = 0.2 (20 % SoC): the first voltage reading is a
// hint, not a fix. Lets the filter converge quickly from a cold start without a
// rest period, which is the whole point over plain CC.
inline constexpr double SocEkfInitVar        = 0.04;
// Q -- process noise per second. Coulomb counting is very good over short
// horizons; this is the slow leak that stops P collapsing to zero and the filter
// going deaf to voltage. sigma grows ~0.6 % SoC over an hour.
inline constexpr double SocEkfProcessVarPerS = 1.0e-8;
// R at zero current -- dominated by OCV CURVE FIT error, not ADC noise. The LTC
// measures to ~1 mV but the fitted curve is worth ~10 mV, so sigma = 10 mV.
inline constexpr double SocEkfMeasVarBase    = 1.0e-4;
// R growth with current, scaled by I^2 because the error is PROPORTIONAL to
// current: an uncertain R_int (say 20 % of 20 mOhm/6 per element) is ~0.67 mOhm
// x I volts of unmodelled drop, so at 100 A ~67 mV, var ~ 4.5e-3 ->
// k = 4.5e-3 / 100^2. This is what makes the filter distrust voltage under load
// instead of chasing I*R as if it were charge.
inline constexpr double SocEkfMeasVarPerA2   = 4.5e-7;

// Accumulator bus (FDCAN1). Standard-frame-only: the HW global filter rejects
// extended IDs at the gate (app_init_task.cpp::HAL_FDCAN_ConfigGlobalFilter).
// The Start->{Precharge, Charge} triggers are physical GPIOs (TSMS_Pin /
// DASH_CHG_Pin), not CAN frames.
//
// ACU RX (FDCAN1): VCU 0x100 DC-bus heartbeat (LE uint16 V). The car/charger
// mode lock at Start->Precharge consumes its freshness.
inline constexpr std::uint32_t AcuRxDcBusId     = 0x100;   // standard; VCU DC bus heartbeat

// Operator charge-mode request. The charger has NO comms with the AMS, so an
// external operator tool asserts this frame to declare "we are on the charger".
// Charger mode requires BOTH a fresh request AND VCU absence (the mode lock in
// safety_task.cpp), which removes the dead-VCU-car ambiguity: a car with a dead
// VCU does NOT send this, so it stays Car mode and faults on VcuStale instead of
// silently charging. The magic payload keeps bus noise or a stray frame from
// triggering an HV-mode change; the freshness check means only an
// actively-asserted request counts.
inline constexpr std::uint32_t ChargeModeReqId      = 0x101u;  // standard; operator -> AMS. COMMISSION (confirm vs ECU map)
inline constexpr std::uint8_t  ChargeModeReqDlc     = 4u;
inline constexpr std::uint8_t  ChargeModeReqMagic[4] = { 0x43u, 0x48u, 0x52u, 0x47u };  // "CHRG"
inline constexpr std::uint32_t ChargeReqFreshMs     = 1000;   // must be this recent at the mode lock

// Operator balance-control override -- a 3-state master switch. The
// ChargerDisplayWario pit tool commands cell balancing on 0x103, magic-gated
// like 0x101:
//   "BALO" -> OFF   force balancing off
//   "BALN" -> ON    force balancing on in ANY FSM state, overriding the
//                   Charge-only default. Still honours the temp-trust gate and
//                   thermal lockout in balance::compute_mask -- the operator
//                   overrides the ENABLE decision, never the safety guards.
//   "BALX" -> AUTO  defer to the autonomous policy (balances in Charge when
//                   imbalanced).
// Dead-man: WarioCharger re-sends the active command ~2 Hz. If the frame goes
// stale (> BalanceOverrideFreshMs) OR was never seen, the effective command
// falls back to OFF, so a dead WarioCharger link never leaves the pack bleeding.
// Only ever affects balancing -- never an AIR / safety path.
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
// the 0x103 master switch. Magic "BALM" (bytes 0..3) + byte 4 = 5-bit enable
// mask (bit m == 1 -> module m may balance). Re-sent ~2 Hz. Dead-man: stale
// (> BalanceModulesFreshMs) OR never seen enables EVERY module
// (BalanceModulesDefaultMask), so the pack falls back to the global OFF/ON/AUTO
// switch alone and the 0x103 dead-man remains the safety net that stops
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
// 0x021 ACU_discharge_interlock: the two facts only the AMS can observe (FSM in
// Start, SDC complete), published so the ECU can decide whether to secure an
// interrupted DC-link discharge. See acu_discharge_interlock.def.
inline constexpr std::uint32_t AcuTxDischargeInterlockId = 0x021;
inline constexpr std::uint32_t AcuTxMinVoltId        = 0x12C;  // BE u16 mV (pack-wide cell min)
inline constexpr std::uint32_t AcuTxSocId            = 0x130;  // 1 byte SoC % [DEFERRED -- no estimator yet; ID reserved + stub encoder available, not scheduled for TX]
inline constexpr std::uint32_t AcuTxVminModuleAId    = 0x131;  // BE u16 mV x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxVminModuleBId    = 0x132;  // BE u16 mV x2 (modules 3..4)
inline constexpr std::uint32_t AcuTxVmaxModuleAId    = 0x133;  // BE u16 mV x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxVmaxModuleBId    = 0x134;  // BE u16 mV x2 (modules 3..4)
inline constexpr std::uint32_t AcuTxCurrentsId       = 0x135;  // BE i16 deciamps x2 (accu, dcdc)
inline constexpr std::uint32_t AcuTxTempMaxModuleAId = 0x136;  // BE i16 degC x3 (modules 0..2)
inline constexpr std::uint32_t AcuTxTempMaxModuleBId = 0x137;  // BE i16 degC x3 (mod 3, 4, dcdc-stub)

// Reserved for future use -- not transmitted. Pack current is published on
// 0x135 (signed deciamps + DCDC in the same frame).
inline constexpr std::uint32_t AcuTxCurrentWarnId       = 0x500;
inline constexpr std::uint32_t AcuTxCurrentOverLimitId       = 0x501;
inline constexpr std::uint32_t AcuTxCurrentNormalId       = 0x502;

inline constexpr std::uint32_t EcuFastTxMs           = 50;
inline constexpr std::uint32_t EcuMidTxMs            = 100;
inline constexpr std::uint32_t EcuSlowTxMs           = 250;

// FDCAN1 Bus-Off recovery rate-limit. On sustained TX errors the M_CAN latches
// Bus_Off (CCCR.INIT set), halting BOTH TX and RX -- the node goes silent and
// stays deaf until a software Stop->Start. AcuCanTask polls
// HAL_FDCAN_GetProtocolStatus every loop pass (a cheap PSR read) and, on
// Bus_Off, issues a Stop/Start no more than once per FdcanBusOffRetryMs. The
// spacing matters: the M_CAN's automatic recovery rejoins only after 128*11
// consecutive recessive bits (~2.8 ms of idle bus at 500 kbps), so a Stop/Start
// every poll would keep restarting that sequence and the node would never finish
// rejoining. 100 ms mirrors the bootloader's BL_FDCAN_BUSOFF_RETRY_MS
// (../stm32-can-bootloader). See ams::can_recovery::should_attempt_recovery and
// acu_can_task.cpp.
inline constexpr std::uint32_t FdcanBusOffRetryMs    = 100;

// ---------------------------------------------------------------------------
// Pit-side diagnostic stream. Runtime-toggleable full-grid telemetry for
// pit-stop debugging with the accumulator plugged into can0 (car stationary in
// the pit, or accumulator on the charger). Off by default.
//
// Enable contract:
//   RX 0x7F0 [4] DE AD BE EF  -> stream ON
//   RX 0x7F0 [4] 00 00 00 00  -> stream OFF
//   TX 0x7F1 [1] {01|00}      -> one-shot ACK after a state transition
//
// State lives in RAM only, so a reboot clears it and diag cannot be left on
// through a track session. The stream continues through FSM::Error so
// charging-fault diagnostics survive a trip.
//
// Stream IDs (1 Hz cadence; ~50 frames/scan ~= 1 % bus load at 500 kbps):
//   0x680..0x697   24 frames of 4 cells each (BE u16 mV); the last frame has
//                  3 real cells + 1 sentinel 0xFFFF.
//                  Decoder: cell_index = 4 * (id - 0x680) + slot;
//                           module = cell_index / 19, cell = cell_index % 19.
//   0x6A0..0x6B8   25 frames of 8 NTC temps each (i8 degC); all 200 NTCs,
//                  row-major over cell_tempC[5][40].
//   0x6C0          FSM extended status (see pit_diag_emitter.hpp layout).
//   0x6C1          Poll timing (last/max V-poll ms, last temp-sweep mask).
inline constexpr std::uint32_t PitDiagCmdRxId            = 0x7F0u;
inline constexpr std::uint32_t PitDiagAckTxId            = 0x7F1u;
inline constexpr std::uint32_t PitDiagCellBaseId         = 0x680u;
// Raw-ADOW diagnostic blocks (config::AdowRawDiag only). 0x6D0..0x6FF is free
// (after the 0x6C0..0x6CA status block). PU = 0x6D0..0x6E7, PD = 0x6E8..0x6FF;
// each 24 frames of 4 cells (BE u16 mV), 0xFFFF = PEC-skipped this scan.
inline constexpr std::uint32_t AdowDiagPuBaseId          = 0x6D0u;
inline constexpr std::uint32_t AdowDiagPdBaseId          = 0x6E8u;   // = 0x6D0 + 24
// ADC-mode cross-check grid (config::AdcModeCrossCheck only). 0x700..0x717, the
// first free block after ADOW fills 0x6D0..0x6FF. Same 24-frame 4-cell BE-u16
// layout as 0x680, so the same decoder reads it; diff it against 0x680 to see
// which cells move with conversion time. 0xFFFF = PEC-skipped this sweep.
inline constexpr std::uint32_t AdcXCheckBaseId           = 0x700u;
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
inline constexpr std::uint32_t PitDiagCommsHealthId      = 0x6C9u;  // FDCAN1 Bus-Off recovery count + ECU-TX fail
// Balance-quiesce health: how often the pre-measurement DCC clear succeeded vs
// failed. Published because DCP=0 does not cover a failed quiesce (LTC6811
// Table 53 suppresses discharge only on the measured cell and its neighbours),
// so a failing quiesce means cell voltages are sampled under bleed -- and the
// balance selector ranks exactly those numbers.
inline constexpr std::uint32_t PitDiagBalanceHealthId    = 0x6CBu;
// UNGATED firmware-health frame: always-on 1 Hz, NEVER gated by the pit-diag arm
// (0x7F0). The ID sits right after the gated 0x6C0..0x6C9 block but is emitted
// regardless of arm state -- parity with ECU 0x704 for passive liveness ("is the
// AMS app alive?" without transmitting an arm frame).
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

// DC-bus collapse detector. In Run (Car mode) the bus equals the pack voltage,
// so if the AIRs open externally -- e.g. a cockpit SDC shutdown the AMS cannot
// sense -- the VCU-reported dc_bus_V collapses. Below BusCollapsePercent of the
// pack (cell-sum) for BusCollapseConfirmTicks consecutive 10 ms safety ticks,
// the FSM treats the contactors as opened externally and de-energises to Start,
// so a re-arm re-runs precharge instead of reclosing AIR+ onto a discharged
// DC-link. COMMISSION both against the real pack/inverter: the percent trades
// false-trip immunity (must sit below the worst-case loaded sag of dc_bus_V vs
// the cell-sum) against inrush protection (higher trips earlier, before the link
// discharges enough to make a no-precharge reclose damaging); the debounce
// rejects a single anomalous 0x100 frame.
inline constexpr std::uint32_t BusCollapsePercent     = 50;  // COMMISSION (% of pack)
inline constexpr std::uint16_t BusCollapseConfirmTicks = 20;  // COMMISSION (~200 ms @ 10 ms)

// Re-arm gate: the DC link must be at or below this before the FSM will leave
// Start for another precharge.
//
// Opening the shutdown circuit de-energises the discharge relay (NC) so the
// bleed connects and the link drains; closing it again re-energises the relay
// and the discharge STOPS part-way. Arming into what is left is a no-op
// precharge: dc_bus >= 95 % of pack is already true on entry, so the FSM leaves
// Precharge on the next step and the resistor never carries meaningful current.
// That 95 % check is the only evidence the AMS has that the precharge resistor
// and contactor work; satisfied by residual charge it proves nothing.
//
// Absolute volts, not a fraction of pack: this is the touch-safe DC limit the
// discharge must reach, and it does not scale with pack voltage. Must sit at or
// above the ECU's own release threshold, or the two disagree about the boundary
// and the AMS waits on a link the ECU has stopped draining.
inline constexpr std::uint16_t DcBusDischargedV       = 60;   // COMMISSION (rulebook)

// Current sensor calibration.
//
// PACK: Bourns SSA-2-250A shunt sensor (datasheet: pcbs/ssa-2.pdf), a 2-wire
// amplified-differential device. OUT_P / OUT_N carry a bipolar +/- 5 mV/A signal
// (250 A nominal, 500 A max unclipped, clips at +/- 2.62 V, common-mode
// ~1.44 V) and wire straight to the STM32 -- no external diff amp -- so ADC3
// reads them differentially:
//
//   OUT_P = PF7 = ADC3_INP3   (positive input)
//   OUT_N = PF8 = ADC3_INN3   (negative input, hardware-paired to INP3)
//
//   V(INP) - V(INN) = 5 mV/A * I        (discharge -> +, charge -> -)
//   raw            ~= 2048 + (V_diff / LSB_diff),  LSB_diff = 2*Vref/4095
//
// STM32H7 differential mode maps V(INP) - V(INN) over -Vref..+Vref onto codes
// 0..4095, zero difference at mid-scale (code ~= 2048). The 1.44 V common-mode
// cancels in the subtraction, leaving the bare 5 mV/A -- hence
// CurrentMvPerAmpe1 = 50 nominal, and a zero that is a COMMISSION *count* offset
// (CurrentZeroCount), not a voltage. The pair spans ~+/- Vref, well past the
// sensor's own +/- 2.62 V (~+/- 524 A) clip, so the CurrentMaxMa over-current
// check is genuinely reachable.
//
// DCDC: Allegro ACS758 Hall-effect sensor (a different part from the pack
// SSA-2) on PC1 = ADC3_INP11, read SINGLE-ENDED through a unity buffer. It is
// RATIOMETRIC -- zero offset and sensitivity both scale with Vcc -- so its 5 V
// datasheet ratings (40 mV/A, offset 0.5*Vcc = 2.5 V) scale by 3.3/5 on this
// 3.3 V rail:
//
//   offset      = 0.5 * 3.3 V     = 1.65 V     -> DcdcCurrentZeroMv     = 1650
//   sensitivity = 40 mV/A * 3.3/5 = 26.4 mV/A  -> DcdcCurrentMvPerAmpe1 = 264
//   V(PC1) = 1.65 V + 26.4 mV/A * I     (sign per IP+ -> IP- wiring)
//
// Converted by adc_to_mA_dcdc. DCDC is informational only -- no safety predicate
// reads it -- and the sign MUST be confirmed on the bench (the ACS758 IP
// direction sets it).
//
// COMMISSION: CurrentZeroCount and CurrentMvPerAmpe1 (pack), and
// DcdcCurrentZeroMv / DcdcCurrentMvPerAmpe1 (DCDC), MUST be calibrated per
// docs/COMMISSIONING.md §2. The pack values below are HIL-bench commissioned: a
// DAC injection at exactly 5 mV/A read back a stable 0.924x (7.6 % low) with a
// +0.6 A zero, so folding that gain into the (COMMISSION) sensitivity gives
// CurrentMvPerAmpe1 = 46 (50 / 0.924, residual +0.4 %) and CurrentZeroCount =
// 2050 (raw at 0 A). The converter math is unchanged; this absorbs the measured
// ADC/VREF gain. On the assembled car the zero measured 2054 instead -- it
// tracks VREF+, so it is board-specific, while the 46 gain read back EXACT
// against an aux-PSU known current. Re-measure the zero per carrier.
inline constexpr std::uint16_t AdcVrefMv          = 3300;
inline constexpr std::uint16_t AdcMaxCount        = 4095;
// Pack channel (differential ADC3_INP3/INN3 = PF7/PF8). HIL-commissioned.
inline constexpr std::int32_t  CurrentZeroCount   = 2054;  // diff zero @ 0 A (flight carrier; HIL bench read 2050)  COMMISSION
inline constexpr std::int32_t  CurrentMvPerAmpe1  = 46;    // COMMISSION (eff 5.4 mV/A x10, HIL gain trim)
// DCDC channel (single-ended ADC3_INP11 = PC1; Allegro ACS758 @ 3.3 V).
inline constexpr std::int32_t  DcdcCurrentZeroMv     = 1650; // ACS758 offset = Vcc/2 @ 3.3 V  COMMISSION
inline constexpr std::int32_t  DcdcCurrentMvPerAmpe1 = 264;  // COMMISSION (26.4 mV/A x10 ratiometric @ 3.3 V)
inline constexpr std::uint8_t  CurrentFilterShift = 4;     // tau ~ 16 samples

// Current-sensor disconnect detection. PF7/PF8 carry a weak internal pull-down
// (GPIO PUPDR, set in stm32h7xx_hal_msp.c): the SSA-2's low-impedance op-amp
// output overrides it when connected, but an open connector lets the legs
// collapse toward 0 V. Each cycle OUT_P (PF7) is also read SINGLE-ENDED and
// checked against a plausible window -- a connected sensor holds OUT_P at the
// ~1.44 V common-mode plus the per-leg half-swing (+/- 2.5 mV/A), i.e.
// ~0.94..1.94 V across +/- 200 A, while a disconnect drags it to ~0 V. Out of
// window -> sensor_fault -> the CurrentSensorFault predicate latches Error. An
// OUT_N-only break instead skews the differential hugely and trips
// CurrentOverLimit, so between the two predicates every disconnect mode is
// covered.
//
// COMMISSION: widen/trim the window to the real OUT_P swing + pull-down
// droop measured on the carrier; confirm the disconnect actually trips
// on the bench (the pull-down behaviour is board/VREF specific).
inline constexpr std::int32_t  CurrentLegPlausMinMv = 700;   // COMMISSION (below -> disconnected)
inline constexpr std::int32_t  CurrentLegPlausMaxMv = 2300;  // COMMISSION (above -> open/rail)
inline constexpr std::uint8_t  CurrentDisconnectConfirm = 3; // consecutive out-of-window reads (~150 ms @ 50 ms)

// IIR low-pass filter coefficient is encoded as a shift so the filter
// is `filtered = filtered - (filtered >> shift) + (raw >> shift)`.

// Cell-balancing parameters. Passive balancing, driven from the Charge state
// only: pick the cells with the largest excess over the pack minimum, cap the
// simultaneous count per module so per-board dissipation stays bounded, and
// inhibit entirely when the warmest NTC says the pack is already hot.
//
// COMMISSION against the dissipation budget of the BMS_LITE balance resistors
// and the airflow available in the accumulator box.
inline constexpr std::uint16_t BalanceDeltaMv     = 50;    // mV above min to start balancing
// Hysteresis STOP threshold: a cell already discharging keeps discharging until
// it falls within this of the floor, instead of being re-ranked from scratch.
//
// compute_mask is stateless and re-picks the top-K every BalanceUpdatePolls
// (~800 ms), so without hysteresis a cell sitting near BalanceDeltaMv toggles
// continuously -- selected, bleeds a little, drops below the single threshold,
// dropped, recovers, selected again. The bleed is real but the duty is a
// fraction of what the operator sees on the mask, and the churn makes the DCC
// pattern unreadable on the bench.
//
// 20 mV against a 50 mV start gives a 30 mV band, comfortably wider than the
// 9-36 mV harness-IR artifact a bled cell shows against its neighbours, so a
// cell does not drop out merely because it is measured while its own bleed
// displaces the shared tap.
//
// MUST stay below BalanceDeltaMv -- a stop threshold at or above the start
// threshold latches a cell on forever. The static_assert below enforces it.
inline constexpr std::uint16_t BalanceStopDeltaMv = 20;
static_assert(BalanceStopDeltaMv < BalanceDeltaMv,
              "stop threshold must be below start, or a selected cell never releases");
inline constexpr std::int16_t  BalanceTempMax     = 50;    // degC; abort balancing if max_tempC > this
// Simultaneous dischargers per module. A BOARD DISSIPATION limit, not a policy
// one -- compute_mask is stateless and re-picks the top-N by excess every
// second, so every imbalanced cell is bled either way. Raising this makes
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
//   board / pack totals at 4.2 V (worst case):
//   MaxActive    per module    all 5 modules
//        4        3.0 W          15 W
//        8        6.0 W          30 W        <-- here
//       19       14.3 W          71 W        (all cells; not attempted)
//
// The resistors are the comfortable part: 2 W devices running at ~0.37 W, under
// a fifth of rating. The real constraint is heat OUT OF THE ACCUMULATOR BOX,
// which the part rating does not change -- 8 simultaneous dischargers is 6.0 W
// per module and 30 W across the pack however good the resistors are.
//
// COMMISSION: BOARD temperature in the sealed accumulator is still not measured
// (the ~71 C below is a single pad on an open bench). Watch it at this setting
// before trusting it in a sealed box, and note that the BalanceTempMax lockout
// that would catch an overheating board reads the same unvalidated NTC path
// (see BalanceTempsTrusted).
inline constexpr std::uint8_t  BalanceMaxActive   = 8;     // cells per module discharging at once

// Never discharge two PHYSICALLY ADJACENT cells at once, so the 2512 balance
// resistors never form a hot cluster on the board. Adjacency comes from the
// BMS_LITE layout in balance::physically_adjacent (consecutive index within an
// LTC half); that index->board-position map is bench-verified by IR on the real
// pack. Measured pad temperature is ~71 C at 8/module concentrated; spreading
// keeps neighbours cold over the multi-hour C/101 balancing session.
//
// May reduce the active count below BalanceMaxActive when imbalanced cells
// cluster -- the intended, safe outcome (less heat; skipped cells bleed on
// later cycles).
inline constexpr bool          BalanceSpreadNoAdjacent = true;

// Whether the cell-temperature path is trusted ENOUGH TO BALANCE ON. Kept
// separate from TempFaultsTrusted, which arms the FSM cell-temp FAULTS, because
// the two ask different questions:
//
//   TempFaultsTrusted   -- trust these temps enough to OPEN THE CONTACTORS?
//                          Not yet: a misread from the unvalidated ADG731 mux
//                          path would trip the car for no reason.
//   BalanceTempsTrusted -- trust them enough to let balancing run? Balancing's
//                          only thermal protection is the BalanceTempMax
//                          lockout, which reads that same path.
//
// Coupled, the WarioCharger balance toggle (0x103) was accepted and then
// produced an all-zero mask forever, so balancing could never run in any FSM
// state. Splitting them lets the operator switch work while temp FAULTS stay
// disarmed.
//
// RESIDUAL RISK -- read before setting this true on a car. With this true and
// TempFaultsTrusted false, passive balancing dissipates into the cells while its
// ONLY thermal guard reads an unvalidated path. Unpopulated NTC slots read a
// plausible ~25 C, so a genuinely hot cell whose sensor is mis-routed by the mux
// would NOT raise max_tempC and would NOT trip the lockout. Mitigations only:
// the 5 s operator dead-man (BalanceOverrideFreshMs) bounds an unattended run,
// at most BalanceMaxActive cells per module bleed at once, and only cells
// > BalanceDeltaMv above the pack minimum are selected. Balance with cell
// temperatures observed by some other means until the ADG731 mux path is
// validated end-to-end and TempFaultsTrusted itself goes true -- at which point
// this flag is redundant and should be deleted.
inline constexpr bool          BalanceTempsTrusted = true;   // COMMISSION -- see residual risk above
inline constexpr std::uint32_t BalanceUpdatePolls = 4;     // = 800 ms at BmsPollVoltMs = 200

// Settle time after clearing the DCC bits and before starting a cell-voltage
// conversion, so no bleed current flows while the cells are measured.
//
// The ADCV DCP=0 bit is not sufficient on this board: it suspends the LTC6811's
// own S-pin switch, but BMS_LITE does not bleed through that switch -- each cell
// drives an EXTERNAL TSM2323 PMOS whose gate sits behind R167 (10k) / C32 (10n),
// tau ~100 us. Conversion starts immediately on ADCV and the first channels
// convert in a few hundred microseconds, the same order as the gate turn-off, so
// the earliest cells can be sampled while current is still flowing.
//
// It matters because the bleed current returns through the harness, not the
// sense path (on-board sensing is close to Kelvin). 179 mA across a plausible
// 50-200 mOhm of tap/connector/fuse impedance is 9-36 mV, with OPPOSITE SIGN on
// the bled cell (reads low) and its neighbours (read HIGH, because the shared
// tap node moves). Against BalanceDeltaMv = 50 mV that is a first-order
// corruption of the very signal balancing selects on -- observed on the bench as
// neighbouring cells reading high whenever balancing is active.
//
// 2 ms is ~20x the gate RC and covers the LTC input-filter settle, at under 1 %
// of balancing duty.
inline constexpr std::uint32_t BalanceQuiesceMs   = 2;

// FDCAN1 TX FIFO slots kept free for the flight telemetry matrix while a LOGFS
// reply is being shipped.
//
// A pull is a MULTI-MINUTE operation, and pump_diag_tx() runs before the
// telemetry scheduler in the same loop pass. Left unbounded it fills the 16-deep
// FIFO to zero free slots, so for the whole transfer the flight matrix finds no
// slot and send_or_fail drops pack currents / voltages / temps SILENTLY (the
// only evidence being g_acu_tx_fail, which is itself best-effort). Diag also
// wins arbitration: 0x011/0x012 out-prioritise every AMS telemetry ID.
//
// It fails safe -- a dropped ok_precharge means no R2D, never a spurious one --
// but an invisible telemetry blackout is a debugging trap. Reserving 6 of 16
// still lets diag move ~10 frames per 1 ms pass, far more than the ~1 frame per
// pass the transfer actually needs.
inline constexpr std::uint8_t  DiagTxReservedSlots = 6;

// LTC6811 ADCV / ADAX mode + settling budget. Mode 2 ("Normal", 7 kHz first
// stage) is the canonical choice for race-pack metrology: ~2.3 ms to convert all
// 12 cell channels with the default filter. AdcvSettleMs rounds that up to 3 ms
// so FreeRTOS tick jitter cannot clip the conversion. ADAX (AUX) under the same
// mode finishes in ~200 us per channel pair plus a settling allowance.
inline constexpr std::uint8_t  AdcMode          = 2;   // ams::ltc6811::AdcMode::Norm7kHz
inline constexpr std::uint32_t AdcvSettleMs     = 3;

// Worst-case gap between two fresh readings when a cross-check sweep runs. The
// sweep is issued AFTER the live read has landed, and BmsPollTask is driven by
// an osTimerPeriodic whose flag stays pending through an overrun -- so a long
// sweep does not push the schedule out, it only makes the next iteration start
// late. The gap is therefore one sweep plus one poll body, NOT a full
// BmsPollVoltMs on top. Cross BmsStaleMs here and the diagnostic would fault the
// pack it is measuring.
//
// The body budget is an allowance, not a measurement: a poll is ADCV + settle +
// warm-up + four register reads, plus ADOW when CellOpenWireCheck is on. Confirm
// it against the live last/max V-poll figures on PitDiagTimingId rather than
// trusting this number.
inline constexpr std::uint32_t AdcXCheckPollBodyBudgetMs = 40;
static_assert(AdcXCheckSettleMs + AdcXCheckPollBodyBudgetMs < BmsStaleMs,
              "ADC cross-check sweep would push a module past BmsStaleMs");
static_assert(AdcXCheckAdcMode != AdcMode,
              "cross-check must use a different mode than the live poll");

// Voltage-poll retry budget: EXTRA attempts if a poll does not come back fully
// PEC-clean, before it counts as a failed poll. Absorbs brief EMI bursts (e.g.
// inverter switching noise) that corrupt one read but clear on an immediate
// re-read, so a transient does not starve a module toward BmsStale. Each attempt
// is ~5 ms (ADCV + settle + reads), so 2 retries (3 attempts) is ~15 ms, well
// inside the 50 ms voltage-poll budget. Each attempt digests whatever ICs are
// clean, giving the stragglers more chances. A dead or asleep chain fails every
// attempt and still counts as failed. 0 = no retry.
inline constexpr std::uint8_t  VoltPollRetries  = 2;
inline constexpr std::uint32_t AdaxSettleMs     = 1;

// ---------------------------------------------------------------------------
// Backup-register usage. RTC_BKP_DR0 is owned by the bootloader (it
// reads BL_BOOT_REQ_MAGIC there on every boot -- see the
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

// CAN frame the app listens for to trigger a deliberate reboot into the
// bootloader. Standard 11-bit ID, very high arbitration priority, on FDCAN1
// (accumulator bus), where AcuCanTask is the only RX consumer. The bootloader
// itself drives FDCAN2 after reset, but the in-band trigger lives on the same
// bus MingoCAN already uses for VCU telemetry.
//
// The 4-byte payload magic stops a stray same-ID frame rebooting the car. All
// four bytes must match exactly.
inline constexpr std::uint32_t BlBootReqCanId      = 0x002u;
inline constexpr std::uint8_t  BlBootReqPayload[4] = { 0xB0, 0x07, 0xAD, 0x11 };
inline constexpr std::uint8_t  BlBootReqDlc        = 4;

// Jump-reason log. Bootloader::request_reboot() stamps this word into
// RTC_BKP_DR2 right before NVIC_SystemReset, so the post-mortem (BL `read-dtc`
// or the next app boot) can tell a CAN-triggered jump from a fault-triggered
// one. Cleared on POR by the BL preamble, preserved across IWDG / NVIC resets
// (same backup-domain semantics as the boot magic and error latch).
//
// Slot 0: BL boot-request magic. Slot 1: AMS ErrorLatch. Slot 2: jump reason.
// Slot 3: last-fault sentinel. Slot 4+ reserved.
inline constexpr std::uint32_t BkpJumpReasonReg = 2;

enum class JumpReason : std::uint32_t {
    None           = 0x00000000u,
    CanTrigger     = 0x4A554D50u,  // 'JUMP' -- MingoCAN sent the boot frame
    FaultLatch     = 0x46415554u,  // 'FAUT' -- safety supervisor forced it
    ManualRequest  = 0x4D414E55u,  // 'MANU' -- operator-issued, no fault
};

// --- Firmware-health frame (0x6CA) ------------------------------------------
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
    // The other spin-forever Cortex-M fault handlers. Distinguishing them costs
    // one byte on 0x6CA and tells you whether the last crash was a bad pointer
    // (MemManage/BusFault) or a bad instruction (UsageFault) -- which is the
    // difference between suspecting the MPU config and suspecting the toolchain.
    MemManage = 5u, BusFault = 6u, UsageFault = 7u,
};

// AMS node ID on the stm32-can-bootloader multi-node bus. Must match the value
// the BL was compiled with (-DBL_NODE_ID=<n>). Embedded in the firmware_info
// `reserved[0]` slot so MingoCAN can verify at flash time that the app it is
// writing matches the BL it is talking to.
//
// The shared-bus role map (provisioned by can-flasher) is ECU=1, AMS=2, uDV=3,
// so the AMS uses 0x02 and does not collide with the ECU. Both halves change
// together: a board's BL MUST be provisioned to node 2 (-DBL_NODE_ID=2 and/or
// NVM provision) before flashing this firmware.
inline constexpr std::uint32_t AmsNodeId = 0x02u;

// Application flash base. Must match STM32H733XG_FLASH.ld's FLASH
// ORIGIN and the bootloader's BL_APP_BASE.
inline constexpr std::uint32_t AppFlashBase    = 0x08020000u;

// AMS telemetry TX on FDCAN1. Three single-purpose 8-byte frames at
// 500 ms cadence each. See docs/CAN_MAP.md for the byte layouts.
inline constexpr std::uint32_t AmsTelemStatusId = 0x4A0u;  // state + cell-V extremes
inline constexpr std::uint32_t AmsTelemPackId   = 0x4A1u;  // pack V + current
inline constexpr std::uint32_t AmsTelemTempsId  = 0x4A2u;  // temp extremes + dc bus + heartbeat
inline constexpr std::uint32_t AmsTelemDiagId   = 0x4A3u;  // diagnostic probes; pure-fn encoder
inline constexpr std::uint32_t AmsRelayStatusId = 0x4A4u;  // contactor + AMS_OK GPIO read-backs (always-on)

// ---------------------------------------------------------------------------
// LTC6811-1 + isoSPI BMS chain. New AMS PCB drives a chain of 10 LTCs
// (5 BMS modules × 2 LTCs each) via SPI1 + LTC6820 isoSPI master. See
// docs/BMS_LTC6811.md for the wire protocol and slot mapping.
// ---------------------------------------------------------------------------
inline constexpr std::uint8_t  LtcsPerModule       = 2;
inline constexpr std::uint8_t  CellsPerLtcUpper    =  9;  // LTC_1 (first in chain) -- 9 cells -> module 0..8
inline constexpr std::uint8_t  CellsPerLtcLower    = 10;  // LTC_2 (second in chain) -- 10 cells -> module 9..18
inline constexpr std::uint8_t  LtcChainLength      = 10;  // BmsModuleCount * LtcsPerModule
inline constexpr std::uint8_t  TempsPerLtc         = 20;  // ADG731 channels populated
inline constexpr std::uint8_t  TempMuxChannelsUsed = 20;  // of 32 on ADG731

// ADG731 channel index (0..31) for each of the 20 temperature indices swept.
// From pcbs/BMS_LITE/LTC_1.kicad_sch: NTC_1..NTC_10 sit on S1..S10,
// NTC_11..NTC_20 on S17..S26, with S11..S16 and S27..S31 unpopulated. The
// 0-indexed channel passed to pack_adg731_select is one less than the
// schematic's "S<n>" pin number. LTC_2's mux (U5) mirrors this map -- some
// channels may be physically unconnected on the current BMS_LITE revision, in
// which case cell_tempC slots 20..39 read open-circuit; see
// docs/COMMISSIONING.md §3.
inline constexpr std::uint8_t Adg731ChannelMap[TempsPerLtc] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,    // S1..S10  -> NTC_1..NTC_10
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25,   // S17..S26 -> NTC_11..NTC_20
};

// NTC + voltage-divider parameters (COMMISSION).
//
// Each NTC is wired between the ADG731 'S' input and the LTC ground reference,
// with a pull-up resistor between V_REF (LTC6811 VREF2 ~ 3.0 V, buffered by U6)
// and the same 'S' node. The selected channel is steered to the LTC's GPIO1
// input and read via RDAUXA (AUX1 in 100-uV units). Thermistor resistance is
// recovered from the observed AUX voltage:
//
//   R_ntc = NtcPullupOhm * V_aux / (NtcVrefMv - V_aux)
//
// Temperature then comes from the manufacturer R-T table
// (Core/Inc/app/ntc_table.hpp, generated from docs/ntc_rt_table.csv), NOT a
// single-beta Steinhart fit -- see that header for why.
//
// NtcPullupOhm is a PULL-UP to VREF2, not a series resistor: on BMS_LITE it is
// R145 / R170 = 6.8 kOhm.
inline constexpr std::uint32_t NtcPullupOhm = 6800;   // R145 / R170 pull-up to VREF2
inline constexpr std::uint16_t NtcVrefMv    = 3000;   // LTC6811 VREF2 nominal

// Open-circuit detection threshold, in AUX millivolts.
//
// A disconnected NTC leaves the divider node pulled up through NtcPullupOhm
// toward VREF2. Ideally it rails to ~3.0 V, but a partially-railed open -- mux
// leakage, a long or damp sense harness, a high-impedance fault -- can settle a
// few hundred millivolts below the rail (~2.7-2.95 V observed). Read literally
// that decodes to a very cold BUT in-range temperature (2.9 V ~= -35 degC), so a
// bare `>= NtcVrefMv` test would let a real disconnect masquerade as a plausible
// cold reading and slip past the presence check.
//
// So anything at or above NtcOpenMv counts as OPEN rather than cold. 2800 mV
// maps to ~-20 degC on the 6.8 k / VREF2 divider -- colder than any operating
// cell, and well below the ~-10 degC a cold-soaked pack could see, leaving
// >= 140 mV margin so a genuinely cold NTC never trips it while a floating node
// reliably does. Must stay below NtcVrefMv to keep the divider denominator
// positive in ntc_mV_to_tempC.
inline constexpr std::uint16_t NtcOpenMv    = 2800;   // >= this AUX mV => open, not cold
static_assert(NtcOpenMv < NtcVrefMv, "open threshold must sit below VREF2");

// Plausibility window for accepted NTC readings (NtcMinValidC / NtcMaxValidC at
// the end of this file). Anything outside it is dropped, leaving the slot at its
// previous value, so an unpopulated mux channel cannot drive max_tempC into
// orbit and trip safety::ForceError on a clean pack.
//
// NtcNoReading is the sentinel stored in cell_tempC for a channel that has never
// produced a valid reading -- unpopulated mux input, open/shorted NTC, or a
// PEC-failed poll. It must NOT be a plausible temperature: a seeded 25 degC
// would make unpopulated channels read as comfortably room temperature, so
// max_tempC looks healthy whatever the pack is doing and every threshold built
// on it is defeated however accurate the conversion is. A sentinel keeps "no
// data" distinguishable from "cool".
inline constexpr std::int16_t NtcNoReading = -32768;   // INT16_MIN

// Temperature-sensor DISCONNECT fault (FS rule: a disconnected temp sensor must
// open the SDC). A sensor that has read valid at least once and then reads OPEN
// (rail voltage -> NtcNoReading) for TempDisconnectPolls consecutive temp polls
// is treated as disconnected and latches ERROR, exactly as a missing module
// faults on the voltage side.
//
// This does NOT depend on temperature ACCURACY -- an open NTC reads the rail
// whatever the beta/pull-up calibration -- so it is armed independently of
// TempFaultsTrusted, which gates the range over/under-temp faults.
//
// TempDisconnectPolls = 1 detects on the FIRST open poll, because the FS rule
// requires the SDC to open in < 500 ms: 1 x 250 ms + the ~100 ms sweep + a 10 ms
// safety tick ~= 360 ms. TRADE-OFF: there is no single-anomalous-read debounce,
// so a one-off mux glitch reading >= NtcOpenMv can nuisance-latch ERROR. The mux
// first-select warm-up and the NtcOpenMv (2800 mV) / plausibility gate reject
// the known transients, but do not eliminate this. COMMISSION: watch for
// spurious TempSensorDisconnected trips on the HIL bench; if a genuine glitch
// source appears, raise BmsPollTempMs headroom and reinstate a 2-poll debounce
// that still fits < 500 ms rather than widening the window.
inline constexpr bool         TempSensorPresenceCheck = true;
inline constexpr std::uint8_t TempDisconnectPolls     = 1;

// REQUIRED temperature channels: cell-temp slots that MUST be present in every
// online module. A required slot reading OPEN faults immediately -- WITHOUT the
// "seen valid once" latch -- so a sensor whose switch is already open at power-on
// (or after a reset that cleared the seen-valid state) is still caught. That is
// what makes the disconnect deterministic for scrutineering, where an
// open-at-boot channel is otherwise indistinguishable from an unpopulated one.
//
// Slot numbering: the ADG731 sweep stores LTC_1 (upper) temps in slots 0..19 and
// LTC_2 (lower) in 20..39, so "temperature 1 of LTC_1" = slot 0 -- the channel
// the scrutineering demo switches on each module. All 40 slots are listed, per
// the BMS_LITE schematic: LTC_1 mux U4 carries NTC_1..NTC_20 (S1..S10 + S17..S26)
// -> slots 0..19 and LTC_2 mux U5 carries NTC_21..NTC_40 (same channels) ->
// slots 20..39, populated on all 5 modules. So ANY open cell-temp sensor, at boot
// or after, opens the SDC per the FS rule.
//
// CONSEQUENCE (COMMISSION / HIL): a genuinely open channel on the flight harness
// (e.g. the known M3 upper-LTC open) LATCHES ERROR at boot until it is repaired.
// Intended, but it means the harness must be healthy for the pack to arm.
// Validate on the bench before flight.
//
// Slot 0 is safe to require only because the ADG731 first-select drop that makes
// temp 1 read open on the first sweep is absorbed by the mux warm-up (the
// throwaway select to unpopulated S32 in BmsPollTask). Without that warm-up,
// requiring slot 0 would false-fault every module at boot.
inline constexpr std::uint8_t RequiredTempSlots[]   = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9,   // LTC_1 NTC_1..NTC_10
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,   // LTC_1 NTC_11..NTC_20
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,   // LTC_2 NTC_21..NTC_30
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,   // LTC_2 NTC_31..NTC_40
};

// Minimum valid cell-temp channels before balancing may run at all.
//
// COMMISSION: deliberately LOW. Its job today is to catch a completely dead
// temperature path, not to guarantee coverage -- setting it above the number of
// channels actually populated would silently disable balancing, the exact
// failure mode this area keeps producing. The board has up to 200 channels
// (5 modules x 40) but how many are fitted is unknown, and the LTC_2 half may
// not be wired at all.
//
// RAISE THIS to the measured populated count once a bench sweep establishes it.
// BmsState::valid_temp_channels is what to read.
inline constexpr std::uint16_t BalanceMinValidTempCh = 5;

inline constexpr std::int16_t NtcMinValidC = -40;
inline constexpr std::int16_t NtcMaxValidC = 150;

}  // namespace ams::config
