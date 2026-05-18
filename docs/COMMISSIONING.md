# AMS commissioning procedure

Finalises every calibration constant that placeholder values in
`Core/Inc/app/ams_config.hpp` ship with. **Every item marked
`COMMISSION` in the source must be measured and updated before the
car runs on track.**

This procedure assumes the firmware is flashed and the AMS is
connected to the live pack and instrumented bench (logging analyzer,
calibrated current source, thermal chamber if available).

---

## 1. Cell voltage and temperature limits

Source: FS rules + the cell datasheet for the specific chemistry used
this season. Replace in `ams_config.hpp`:

| Constant | Default | Action |
|---|---|---|
| `kCellUVmV` | 2800 | set to **datasheet minimum + 100 mV margin** |
| `kCellOVmV` | 4200 | set to **datasheet maximum − 100 mV margin** |
| `kCellUTC`  | −10  | set to **datasheet minimum operating °C** |
| `kCellOTC`  | 60   | set to **the lower of: datasheet max OR FS rule** |

Sign off the values in the project log together with the rules version
they're derived from.

---

## 2. Current sensor calibration

The pack current path uses a **Bourns SSA-2-250A** shunt sensor
(datasheet at `pcbs/ssa-2.pdf`). The sensor's raw differential output
is `±5 mV/A` around a common-mode voltage of `+1.44 V`. A discrete
difference amplifier on the carrier (MCP6001R, gain ×4, R12 biased
to `Vref/2 = 1.65 V`) converts the differential signal to a
single-ended `S_CURRENT` routed to `PF7` (ADC3 ch 3, 12-bit):

> `S_CURRENT = 4 × (OUTP − OUTN) + 1.65 V`

Nominal calibration:
- **Zero current** → S_CURRENT = 1650 mV (`kCurrentZeroMv`)
- **Sensitivity** at the ADC pin = 5 mV/A × 4 = 20 mV/A
  (`kCurrentMvPerAmpe1` = 200, i.e. 20 mV/A × 10)
- **Sign convention**: discharge → positive `(OUTP − OUTN)` → S_CURRENT
  rises above 1.65 V → positive mA. Charge does the opposite.
- **Observable range**: bipolar `±82.5 A` (constrained by the 0–3.3 V
  ADC rail). Currents beyond that clip at the rail and become
  indistinguishable; `kImaxMa = 200 A` is therefore a defensive-only
  predicate on this HW revision — see §2.4.

Tolerance of the Vref/2 divider and R10..R13 mismatch can shift the
zero point by tens of mV; calibrate before v1.0.0.

### 2.1 Zero-offset

1. Disconnect the pack from anything that draws current.
2. Read the raw ADC value via debugger (`hadc3` → start → poll → get).
3. Compute the implied voltage `v_mV = raw * 3300 / 4095`.
4. If `v_mV` differs from `1650` by more than 30 mV, update
   `kCurrentZeroMv` in `ams_config.hpp` to the measured value.

### 2.2 Sensitivity

1. Connect a calibrated current source in series with the pack.
2. Run **+10 A** discharge. Note the new ADC raw.
3. Run **−10 A** charge (regen or external charger). Note ADC raw.
4. Sensitivity = `(v_at_+10A_mV − v_zero_mV) / 10`, in mV/A. (Note
   the sign — discharge raises voltage on this HW revision.)
5. Update `kCurrentMvPerAmpe1` (scaled ×10): set it to
   `round(sensitivity_mV_per_A × 10)`. Nominal is 200.
6. Confirm symmetry by comparing the −10 A reading; if `|v_at_-10A_mV
   − v_zero_mV|` differs from `|v_at_+10A_mV − v_zero_mV|` by > 2 %,
   log a non-linearity warning in the project log and consider a
   per-direction calibration table (out of v1.0.0 scope).

### 2.3 Absolute limit

`kImaxMa` defaults to 200 000 mA (200 A). On the current HW revision
the ADC clips at ±82.5 A, so the predicate `|filtered_mA| > kImaxMa`
never trips in practice — it's a defensive-only check. If you want a
real over-limit safety, lower `kImaxMa` to e.g. 75 000 mA (75 A, with
10 % margin from the clipping rail). Otherwise leave at the FS-rules
value and treat clipping at the rail as the de-facto trip.

### 2.4 Charge-current observability caveat

The diff-amp gain (×4) plus the Vref/2 bias means the full 3.3 V rail
maps to ±82.5 A. Real currents above that are not observable. Two
HW design choices that affect this:
- **Lower the gain** (e.g. ×2) to widen the range to ±165 A at the
  cost of doubling the LSB per ampere (10 mV/A noise floor).
- **Add a second sensor** on `S_CURRENT_DCDC` (PF8 → ADC3 ch 7,
  currently configured as analog input but not read by firmware).
Both are tracked as v1.5 follow-ups.

---

## 3. Precharge target and timing

`kPrechargeRatio` defaults to 0.95 (DC bus must reach 95 % of pack
voltage before AIR+ closes). Real cars typically end up at 0.95–0.98.

### 3.1 Verify on the bench

1. Force AMS into `Precharge` via debugger (set `state = Precharge`,
   set `state_entry_tick = now`).
2. Log `bms.pack_voltage_mV` and `veh.dc_bus_V` every 100 ms while the
   precharge resistor charges the bus capacitors.
3. Confirm `dc_bus_V` reaches `0.95 * pack_V` well within the
   `kPrechargeMaxMs` (1500 ms) timeout. Typical: 200–800 ms.
4. If the timeout fires, either the precharge resistor is too large
   (slow ramp) or the DC bus has an unexpected leak. Investigate
   before increasing `kPrechargeMaxMs` — the default is generous.

### 3.2 Transition hold

`kTransitionHoldMs` defaults to 100 ms. Increase if you see the bus
slumping after AIR+ closes (cap discharge, inrush). Decrease if 100 ms
is too long for the driver to wait. Don't go below 20 ms — that's
within FreeRTOS scheduling jitter.

---

## 3b. NTC thermistor calibration (LTC6811 + ADG731)

200 NTCs (5 modules × 40 per module = 20 per LTC × 2 LTCs) are
selected through the ADG731 32:1 mux on each BMS_LITE board,
buffered onto LTC6811 GPIO1, sampled with ADAX(GPIO1), and converted
into °C in `BmsService::update_temperature` using the Beta model:

```
R_ntc = kNtcSeriesR * V_aux / (kNtcVrefMv - V_aux)
1/T   = 1/T0 + (1/B) * ln(R_ntc / R_25)
T_°C  = T - 273.15
```

`ams_config.hpp` ships placeholder values matching the BMS_LITE BOM
(Murata NCP15XH103J, β = 3380 K, R₂₅ = 10 kΩ, series resistor 10 kΩ,
LTC6811 VREF2 ≈ 3.0 V). Verify on the bench before relying on
`max_tempC`:

1. Bring the pack to a known soak temperature (use the ambient probe
   on the same board if you have one — or 25 °C in still air after a
   30-minute warm-up).
2. Read `BmsState.cell_tempC[m][t]` for every (m, t) via the
   telemetry frame on FDCAN1 or directly via SWD.
3. The median across all 200 NTCs should sit within ±2 °C of the
   reference. If it doesn't, tune `kNtcBeta` first (typical
   correction is +/-5%), then `kNtcR25` (typical +/-2%).
4. Re-run the soak after each tweak. Two soaks at the two extremes
   (e.g. 25 °C and 60 °C with a hot-air gun on one cell) gives a
   two-point fit that pins both β and R₂₅.

`kAdg731ChannelMap` (20 entries) is the lookup from temperature
index `t` (0..19) to ADG731 channel (0..31). Current map matches the
schematic walk of `pcbs/BMS_LITE/LTC_1.kicad_sch` (commit `<#71>`):
S1..S10 → ch 0..9 → NTC_1..NTC_10, S17..S26 → ch 16..25 → NTC_11..NTC_20.

> **BMS_LITE Rev A caveat**: the LTC_2 schematic shows only 10 of
> 20 mux channels (S1..S10 → NTC_21..NTC_30) labelled. If your board
> mirrors LTC_1 in copper but not yet in schematic, all 20 slots
> work; if 10 channels are physically open, the second half of
> `cell_tempC[m][20..39]` will read as "skip" (sentinel) on every
> sweep and stay at the 25 °C ctor default. Confirm with a continuity
> meter on one assembled board before flashing v1.0.0.

---

## 3c. Cell balancing (LTC6811 WRCFGA / passive)

Passive balancing runs only in `fsm::State::Charge`. Once per
`kBalanceUpdatePolls` voltage-poll cycles (~1 Hz at the default 250 ms
voltage cadence) BmsPollTask snapshots `BmsState`, runs the
`ams::balance::compute_mask` policy, packs the per-IC DCC bits into
WRCFGA payloads, and broadcasts.

Tunables in `ams_config.hpp`:

| Constant | Default | Effect |
|---|---:|---|
| `kBalanceDeltaMv` | 50 mV | Discharge any cell where `v > min_cell_mV + delta`. Smaller = tighter balance, more heat. |
| `kBalanceMaxActive` | 4 | Max cells per module discharging simultaneously. Drives per-board dissipation. |
| `kBalanceTempMax` | 50 °C | Inhibit all balancing if `max_tempC > this`. Don't add heat when the pack is already warm. |
| `kBalanceUpdatePolls` | 4 | Cycles between WRCFGA updates. Smaller = more reactive, larger = less SPI traffic. |

Procedure:

1. With the pack in Charge state on the bench (charger attached,
   `kAcuRxChargerId` frame live, FSM in Charge), watch
   `g_balance_cycles_active` climb whenever any cell sits above the
   threshold. `g_balance_cycles_total` increments unconditionally so
   you can compute the active fraction.
2. Verify per-board dissipation with a clamp meter on the supply
   rail to one BMS_LITE during a known-imbalanced soak. If the
   resistor stack runs above its thermal budget, drop
   `kBalanceMaxActive` first, then raise `kBalanceDeltaMv` to
   tolerate a wider equilibrium voltage band.
3. Confirm `WRCFGA -> RDCFGA` round-trip reads back the DCC bits we
   intended (HIL: cell at 4150 mV in module 2 with the rest at
   4100 mV should set DCC for that cell on the corresponding chain
   IC; RDCFGA reflects it).

---

## 3d. MainTask boot-grace window

`kSafetyBootGraceMs` defaults to 2000 ms. While `now_tick <
kSafetyBootGraceMs`, the data-presence predicates inside
[`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp)
return `false` unconditionally. The watchdog is fed normally during
this window, so the chip stays alive while BmsPollTask,
CurrentSensorTask, and AcuCanTask are still spinning up their first
sample.

**Immediate-safety predicates stay active during the grace** —
`force_error_set` still trips a fault on the very first MainTask
iteration. The grace only suppresses the freshness / range checks
that would always fail at t = 0 because no service has published
yet.

Defaults are sized for the slowest startup path among the data
producers:

| Producer | First write to its service |
|---|---|
| BmsPollTask voltage poll | ~250 ms (kBmsPollVoltMs) |
| CurrentSensorTask ADC sample | 50 ms (kCurrentPeriodMs) |
| AcuCanTask VCU 0x100 ingest | depends on the vehicle bus; typically present immediately |
| BmsPollTask temperature sweep | ~500 ms (kBmsPollTempMs) |

2000 ms covers the worst of these plus a comfortable margin for a
slow CAN bus startup. Tune down only if a faster Start state matters
more than the margin; tune up if a planned producer takes longer
than 2 s to publish on a bench / vehicle.

> Setting `kSafetyBootGraceMs = 0` reverts to the pre-v1.2.0
> behaviour, where the very first MainTask iteration faulted on
> freshness and the chip entered a watchdog-reset loop. Don't do
> this — the loop is unrecoverable without reflashing.

---

## 4. BMS freshness window

`kBmsStaleMs` defaults to 1500 ms. The slave's typical response is
< 20 ms; 1500 ms tolerates 7 missed polls in a row. If the CAN bus
load goes up enough that legitimate misses cluster, raise to 2500 ms.
Don't go below 750 ms — you'll get nuisance FORCE_ERRORs during
normal CAN burst windows.

---

## 5. Watchdog window

The IWDG runs at LSI ~32 kHz with ±47 % tolerance. Defaults:

- `prescaler = 32`
- `reload    = 100`
- `nominal timeout = 100 ms` (range ~52 ms to ~190 ms)

MainTask refreshes every 10 ms on the clean path (and on the
latched-fault path; see ARCHITECTURE.md §1 invariant 5), so even at
the LSI fast corner (52 ms) we have 5× margin. **Do not increase
the reload** without re-evaluating the MainTask period.

If the watchdog ever resets during normal operation, the failure
mode is almost certainly NOT the LSI tolerance — it's MainTask
being preempted by an unexpected higher-priority task, or
BmsPollTask's SPI call blocking past 10 ms. Investigate before
touching the IWDG constants.

---

## 6. Fan duty cycles

`kFanDuty[]` in `safety_task.cpp` (MainTask's anonymous namespace):

| State | Default % | Tune by |
|---|---|---|
| Run    | 40 | thermal soak test at peak discharge |
| Charge | 75 | thermal soak test at max charge rate |

Increase the Run duty if the pack hits `kCellOTC − 5°C` during a 22-
minute autocross run. The 75 % charge default is conservative; you
can drop to 60 % if the charger is itself temperature-limited.

---

## 7. Acceptance test

Before tagging v1.0.0, the AMS must pass the following on the actual
vehicle:

1. Cold-boot from VBAT-only → AIRs read open via clamp meter
2. Press start button with healthy pack → reach `Run` within 2 s
3. Charger plug-in from `Run` → `Charge`, fan to 75 %
4. Force-open any BMS module (pull its isoSPI cable from the chain) →
   `Error` within `kBmsStaleMs + one V-poll period`, AIRs open, backup
   register flag set
5. Power-cycle after step 4 → AMS comes up in `Error`, AIRs stay
   open, requires the manual reset procedure (TBD: define gesture)
6. 30-minute capture of telemetry frames `0x4A0` / `0x4A1` / `0x4A2`
   on FDCAN1; confirm cadence is 500 ms ± 5 ms, no dropped frames,
   no garbage decode

Sign off each step in the project log with date, scrutineer name, and
the firmware commit SHA.

---

## 8. Operator reset procedure

**Pending decision.** Options:

- Reset-only via power cycle (simplest, current default)
- Long-press charge button to clear the latch from `App_InitTask`
- Dedicated UART command

Lock this in before commissioning. The choice affects `App_InitTask`
boot logic.
