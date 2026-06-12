# AMS commissioning procedure

Finalises every calibration constant that placeholder values in
`Core/Inc/app/ams_config.hpp` ship with. **Every item marked
`COMMISSION` in the source must be measured and updated before the
car runs on track.**

This procedure assumes the firmware is flashed and the AMS is
connected to the live pack and instrumented bench (logging analyzer,
calibrated current source, thermal chamber if available).

> **Sign-off sheet:** [`COMMISSIONING_CHECKLIST.md`](COMMISSIONING_CHECKLIST.md)
> lists every `COMMISSION` constant (current default, units, what to
> measure) with tick-boxes and a final-value column — use it as the
> record for the bench session; this document is the how-to behind it.

---

## 1. Cell voltage and temperature limits

Source: FS rules + the cell datasheet for the specific chemistry used
this season. Replace in `ams_config.hpp`:

| Constant | Default | Action |
|---|---|---|
| `CellUnderVoltageMv` | 2800 | set to **datasheet minimum + 100 mV margin** |
| `CellOverVoltageMv` | 4200 | set to **datasheet maximum − 100 mV margin** |
| `CellUnderTempC`  | −10  | set to **datasheet minimum operating °C** |
| `CellOverTempC`  | 60   | set to **the lower of: datasheet max OR FS rule** |

Sign off the values in the project log together with the rules version
they're derived from.

---

## 2. Current sensor calibration

The pack current path uses a **Bourns SSA-2-250A** shunt sensor
(datasheet at `pcbs/ssa-2.pdf`). It is a 2-wire amplified-differential
device: its `OUT_P` / `OUT_N` pins carry a bipolar `±5 mV/A`
differential signal around a common-mode of `+1.44 V`.

**HW revision `feat/current-sensor-diff`:** the external carrier diff
amp (old MCP6001R ×4 + Vref/2 bias) is **removed**. `OUT_P` and `OUT_N`
now wire straight to the STM32 and the ADC reads them in **differential
mode**:

> `OUT_P = PF7 = ADC3_INP3`  ·  `OUT_N = PF8 = ADC3_INN3`

In STM32H7 differential mode the conversion encodes `V(INP) − V(INN)`
over `−Vref … +Vref` onto codes `0 … 4095`, with the zero-difference
point at **mid-scale (code ≈ 2048)**. The sensor common-mode (1.44 V)
cancels in the subtraction, leaving only the bipolar `±5 mV/A`:

> `raw ≈ 2048 + (5 mV/A × I) / LSB_diff`,  `LSB_diff = 2·Vref/4095 ≈ 1.61 mV`

Nominal (ideal) calibration:
- **Zero current** → ADC code ≈ 2048 (`CurrentZeroCount`, a *count*, not
  a voltage — the natural reference in differential mode is the mid code)
- **Sensitivity** = the bare sensor 5 mV/A (no ×4 gain anymore):
  ideal `CurrentMvPerAmpe1 = 50` (i.e. 5 mV/A × 10)

> **HIL-commissioned values (#348)** — `ams_config.hpp` currently ships
> the bench-measured figures, not the ideal ones. A DAC injection
> verified at exactly 5 mV/A measured the firmware reading a stable
> **0.924× (7.6 % low)** with a **+0.6 A** zero (6-boot spread 0.1 A).
> Folding that effective ADC/VREF gain into the COMMISSION constants:
> **`CurrentMvPerAmpe1 = 46`** (50 / 0.924, residual +0.4 %) and
> **`CurrentZeroCount = 2050`** (raw at 0 A) → the over-current trip
> lands at **200 A real** and telemetry is accurate. Re-measure on each
> new carrier; the gain is board-specific (VREF+ tolerance).
- **Sign convention**: discharge → `OUT_P` above `OUT_N` → raw above
  mid-scale → positive mA. Charge does the opposite.
- **Observable range**: the differential pair spans ≈ `±Vref`, well
  beyond the sensor's own `±2.62 V` (≈ `±524 A`) clip. Unlike the old
  ×4 + 1.65 V front-end (which clipped firmware-side at only `±82.5 A`),
  `CurrentMaxMa = 200 A` is now **genuinely reachable** — see §2.3.

The sensor DC offset (≤ ±0.4 mV) plus the ADC offset shift the zero code
by a few LSB; calibrate before v1.0.0.

### 2.1 Zero-offset

1. Disconnect the pack from anything that draws current.
2. Read the raw differential ADC code for the pack channel (CH3,
   `ADC_DIFFERENTIAL_ENDED`) — see the firmware's `read_adc3_channel`.
3. If the raw code differs from `2048` by more than ~20 counts, update
   `CurrentZeroCount` in `ams_config.hpp` to the measured value.

### 2.2 Sensitivity

1. Connect a calibrated current source in series with the pack.
2. Run **+10 A** discharge. Note the new raw code `r_plus`.
3. Run **−10 A** charge (regen or external charger). Note `r_minus`.
4. Counts-per-amp = `(r_plus − CurrentZeroCount) / 10`. Convert to mV/A:
   `mV_per_A = counts_per_amp × LSB_diff` where `LSB_diff = 2·3300/4095 ≈
   1.612 mV`. Nominal result ≈ 5 mV/A.
5. Update `CurrentMvPerAmpe1` (scaled ×10): set it to
   `round(mV_per_A × 10)`. Nominal is 50.
6. Confirm symmetry by comparing the −10 A reading; if `|r_minus −
   CurrentZeroCount|` differs from `|r_plus − CurrentZeroCount|` by > 2 %,
   log a non-linearity warning and consider a per-direction calibration
   table (out of v1.0.0 scope).

### 2.3 Absolute limit

`CurrentMaxMa` defaults to 200 000 mA (200 A). With the differential
front-end this threshold maps to a raw code (`2050 + ~571 ≈ 2621`) well
inside `0…4095`, so the predicate `|filtered_mA| > CurrentMaxMa` is now a
**real, reachable** over-current trip (no longer defeated by an 82.5 A
clip). Keep it at the FS-rules value unless a tighter pack limit applies.

### 2.4 DCDC current channel

DCDC supply current uses an **Allegro ACS758** Hall-effect sensor (a
different part from the pack SSA-2) on `PC1 = ADC3_INP11` (was `PF8`),
read **single-ended** through a unity buffer (gain 1). The ACS758 is
**ratiometric** — both its zero offset and its sensitivity scale with
`Vcc`. At the 5 V datasheet rating it is 40 mV/A with offset
`0.5·Vcc = 2.5 V`; powered from **3.3 V** here, both scale by `3.3/5`:

> offset = `0.5 × 3.3 V` = **1.65 V**
> sensitivity = `40 mV/A × 3.3/5` = **26.4 mV/A**
> `V(PC1) = 1.65 V + 26.4 mV/A × I`

Converted by `adc_to_mA_dcdc` using its own constants, both `COMMISSION`:
- `DcdcCurrentZeroMv` (nominal **1650 mV** — `Vcc/2`, which also equals
  ADC mid-scale because the ACS758 shares the 3.3 V rail)
- `DcdcCurrentMvPerAmpe1` (nominal **264**, i.e. 26.4 mV/A × 10)

DCDC is **informational only** — not part of any safety predicate
(`DcdcIStaleMs` staleness has no FSM impact). Calibrate by the same
zero-then-sensitivity procedure as §2.1–2.2 but against the PC1
single-ended reading (`v_mV = raw × 3300 / 4095`). **Confirm the sign on
the bench** — the ACS758's IP+→IP− conductor direction sets whether
discharge reads positive or negative.

### 2.5 Disconnect detection (pack channel)

A disconnected pack sensor is *not* caught by staleness (the ADC keeps
converting) or by the differential reading (a floating diff pair reads
≈ 0, i.e. looks like 0 A). To detect it, **PF7/PF8 carry a weak internal
pull-down** (GPIO PUPDR, set in `stm32h7xx_hal_msp.c` — no board parts).
The SSA-2's low-impedance op-amp output overrides the pull when
connected; an open connector lets the legs collapse toward 0 V. Each
50 ms cycle the firmware reads **OUT_P (PF7) single-ended** and checks it
sits in `[CurrentLegPlausMinMv, CurrentLegPlausMaxMv]`; after
`CurrentDisconnectConfirm` consecutive out-of-window reads it asserts
`sensor_fault` → the `CurrentSensorFault` predicate (reason **8**) latches
Error, opens the AIRs, drops `AMS_OK`. (An OUT_N-only break instead skews
the differential and trips `CurrentOverLimit` — between the two, every
disconnect mode is covered.)

Nominal window `700..2300 mV` brackets the connected OUT_P swing
(≈ 0.94–1.94 V across ±200 A) with margin; disconnect reads ≈ 0 V.
**COMMISSION:** on the carrier, measure the connected OUT_P range and the
pull-down droop, set the window to enclose the former and exclude ~0 V,
and **confirm on the bench that physically unplugging the sensor latches
reason 8** (the PUPDR-in-analog behaviour is board/VREF specific). Tune
`CurrentDisconnectConfirm` if the channel-reconfigure glitch ever shows.

| Const | Default | Meaning |
|---|---|---|
| `CurrentLegPlausMinMv` | 700 | below → disconnected (pulled to 0 V) |
| `CurrentLegPlausMaxMv` | 2300 | above → open / stuck-rail |
| `CurrentDisconnectConfirm` | 3 | consecutive out-of-window reads (~150 ms) |

---

## 3. Precharge target and timing

Two constants govern precharge, both tagged `COMMISSION`:

- **`PrechargeRatio`** (default 0.95) — in **Car** mode, AIR+ closes once
  `dc_bus_V ≥ 95 % × pack_voltage`. Real cars typically end up at
  0.95–0.98. (Charger mode has no `dc_bus_V`; it proceeds on a still-fresh
  `0x101` charge request — see § 3.3.)
- **`PrechargeMaxMs`** (default 5000 ms) — the hard ceiling on how long
  Precharge may run before the FSM latches `Error` and opens every
  contactor. This protects the precharge resistor (transient-duty rated)
  for **any** stuck cause, including a dead-VCU car that mislocks Charger
  and could otherwise sit in Precharge forever.

### 3.1 Verify the Car precharge ramp

1. Force AMS into `Precharge` (Car mode): VCU `0x100` fresh, then assert
   TSMS + a DASH_CHG press.
2. Log `bms.pack_voltage_mV` and `veh.dc_bus_V` every 100 ms (the pit-diag
   stream carries both) while the precharge resistor charges the bus
   capacitors.
3. Confirm `dc_bus_V` reaches `0.95 × pack_V`. Typical: 200–800 ms.
4. If the bus never reaches target, the precharge resistor is too large
   (slow ramp) or the DC bus has an unexpected leak — investigate the ramp.

### 3.2 Commission `PrechargeMaxMs`

Measure the **worst-case** healthy precharge time on the real pack +
resistor (cold resistor, full pack voltage, coldest expected bus caps),
then set `PrechargeMaxMs` comfortably **above** it (so a healthy ramp
never trips) but **below** the resistor's transient thermal limit from its
datasheet (so a genuinely stuck precharge opens before the resistor is
damaged). The 5000 ms default is a placeholder — confirm it against the
measured ramp and the resistor's pulse-energy rating.

### 3.3 Transition

Transition is a single FSM-step passthrough: the AIR+ close +
precharge-contactor open is emitted on the `Precharge → Transition` edge,
and the next FSM step commits to `Run` (Car mode) or `Charge` (Charger
mode). A **Car-only** bus-still-at-target guard remains, so a failed
contactor swap (bus slumps the moment the precharge contactor opens) lands
in Error rather than energising on a degraded bus. Charger mode skips the
guard — there is no VCU `dc_bus_V` during a charge, and the charger
soft-starts its own output.

### 3.4 Bus-collapse detector (`BusCollapsePercent`, `BusCollapseConfirmTicks`)

In `Run` (Car mode) the VCU `dc_bus_V` tracks the pack. A **cockpit SDC
shutdown** opens the AIRs without the AMS sensing it (#330) — so the AMS
watches for the bus collapsing while it still thinks it's in `Run`, and
falls back to `Start` (non-latching) so a re-arm re-runs precharge instead
of reclosing AIR+ onto a discharged DC-link. Two `COMMISSION` constants:

- **`BusCollapsePercent`** (default 50) — `dc_bus_V` below this % of the
  pack (cell-sum) counts as "collapsed." Pick it **above** the worst-case
  loaded sag of `dc_bus_V` vs the cell-sum (so hard acceleration/regen
  never false-trips) but **high enough** that it fires before the DC-link
  discharges to a voltage where a no-precharge reclose would be damaging.
  Measure both on the bench: log `dc_bus_V / pack_voltage_mV` under maximum
  load (sag floor), and the link's discharge curve after an AIR-open
  (how far it falls in `BusCollapseConfirmTicks`), then set the percent
  comfortably between them.
- **`BusCollapseConfirmTicks`** (default 20 ≈ 200 ms @ 10 ms) — consecutive
  collapsed ticks before de-energising. Long enough to reject a single
  anomalous `0x100` frame; short enough to trip before a released shutdown
  recloses the AIRs. The DC-link discharge is gradual, so a few hundred ms
  is safe.

Verify on the bench: in `Run`, force the AIRs open (or inject a low
`dc_bus_V` via the VCU/HIL fixture); confirm the FSM returns to `Start`
(not Error) after the debounce, AMS_OK stays HIGH, and a DASH_CHG re-arm
runs a full precharge.

---

## 3b. NTC thermistor calibration (LTC6811 + ADG731)

200 NTCs (5 modules × 40 per module = 20 per LTC × 2 LTCs) are
selected through the ADG731 32:1 mux on each BMS_LITE board,
buffered onto LTC6811 GPIO1, sampled with ADAX(GPIO1), and converted
into °C in `BmsService::update_temperature` using the Beta model:

```
R_ntc = NtcSeriesR * V_aux / (NtcVrefMv - V_aux)
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
   telemetry frame on FDCAN1 (the per-NTC grid is on pit-diag
   `0x6A0..0x6B8`).
3. The median across all 200 NTCs should sit within ±2 °C of the
   reference. If it doesn't, tune `NtcBeta` first (typical
   correction is +/-5%), then `NtcR25` (typical +/-2%).
4. Re-run the soak after each tweak. Two soaks at the two extremes
   (e.g. 25 °C and 60 °C with a hot-air gun on one cell) gives a
   two-point fit that pins both β and R₂₅.

`Adg731ChannelMap` (20 entries) is the lookup from temperature
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
`BalanceUpdatePolls` voltage-poll cycles (~1 Hz at the default 250 ms
voltage cadence) BmsPollTask snapshots `BmsState`, runs the
`ams::balance::compute_mask` policy, packs the per-IC DCC bits into
WRCFGA payloads, and broadcasts.

Tunables in `ams_config.hpp`:

| Constant | Default | Effect |
|---|---:|---|
| `BalanceDeltaMv` | 50 mV | Discharge any cell where `v > min_cell_mV + delta`. Smaller = tighter balance, more heat. |
| `BalanceMaxActive` | 4 | Max cells per module discharging simultaneously. Drives per-board dissipation. |
| `BalanceTempMax` | 50 °C | Inhibit all balancing if `max_tempC > this`. Don't add heat when the pack is already warm. |
| `BalanceUpdatePolls` | 4 | Cycles between WRCFGA updates. Smaller = more reactive, larger = less SPI traffic. |

Procedure:

1. With the pack in Charge state on the bench (charger attached,
   `AcuRxChargerId` frame live, FSM in Charge), watch
   `g_balance_cycles_active` climb whenever any cell sits above the
   threshold. `g_balance_cycles_total` increments unconditionally so
   you can compute the active fraction.
2. Verify per-board dissipation with a clamp meter on the supply
   rail to one BMS_LITE during a known-imbalanced soak. If the
   resistor stack runs above its thermal budget, drop
   `BalanceMaxActive` first, then raise `BalanceDeltaMv` to
   tolerate a wider equilibrium voltage band.
3. Confirm `WRCFGA -> RDCFGA` round-trip reads back the DCC bits we
   intended (HIL: cell at 4150 mV in module 2 with the rest at
   4100 mV should set DCC for that cell on the corresponding chain
   IC; RDCFGA reflects it).

---

## 3d. MainTask boot-grace window

`SafetyBootGraceMs` defaults to 2000 ms. While `now_tick <
SafetyBootGraceMs`, the data-presence predicates inside
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
| BmsPollTask voltage poll | ~250 ms (BmsPollVoltMs) |
| CurrentSensorTask ADC sample | 50 ms (CurrentPeriodMs) |
| AcuCanTask VCU 0x100 ingest | depends on the vehicle bus; typically present immediately |
| BmsPollTask temperature sweep | ~500 ms (BmsPollTempMs) |

2000 ms covers the worst of these plus a comfortable margin for a
slow CAN bus startup. Tune down only if a faster Start state matters
more than the margin; tune up if a planned producer takes longer
than 2 s to publish on a bench / vehicle.

> Setting `SafetyBootGraceMs = 0` reverts to the pre-v1.2.0
> behaviour, where the very first MainTask iteration faulted on
> freshness and the chip entered a watchdog-reset loop. Don't do
> this — the loop is unrecoverable without reflashing.

---

## 4. BMS freshness window

`BmsStaleMs` defaults to 1500 ms. The slave's typical response is
< 20 ms; 1500 ms tolerates 7 missed polls in a row. If the CAN bus
load goes up enough that legitimate misses cluster, raise to 2500 ms.
Don't go below 750 ms — you'll get nuisance FORCE_ERRORs during
normal CAN burst windows.

---

## 5. Watchdog window

The IWDG runs at LSI ~32 Hz with ±47 % tolerance. Defaults:

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

`FanDuty[]` in `safety_task.cpp` (MainTask's anonymous namespace):

| State | Default % | Tune by |
|---|---|---|
| Run    | 40 | thermal soak test at peak discharge |
| Charge | 75 | thermal soak test at max charge rate |

Increase the Run duty if the pack hits `CellOverTempC − 5°C` during a 22-
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
   `Error` within `BmsStaleMs + one V-poll period`, AIRs open, backup
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
