# AMS commissioning procedure

Finalises every calibration constant that placeholder values in
`Core/Inc/app/ams_config.hpp` ship with. **Every item marked
`COMMISSION` in the source must be measured and updated before the
car runs on track.**

This procedure assumes the firmware is flashed (over the CAN bootloader
— see `AmsNodeId` / `AppFlashBase` in `ams_config.hpp`) and the AMS is
connected to the live pack and an instrumented bench: a CAN analyser on
FDCAN1, a calibrated current source, and a thermal chamber or hot-air
gun if available.

> **Sign-off sheet:** [`COMMISSIONING_CHECKLIST.md`](COMMISSIONING_CHECKLIST.md)
> lists every `COMMISSION` constant (current default, units, what to
> measure) with tick-boxes and a final-value column — use it as the
> record for the bench session; this document is the how-to behind it.

**The source is the authority.** `ams_config.hpp` carries the physical
reasoning for every number inline, usually in more detail than this page.
If the two disagree, the header wins and this page is stale — fix it.

---

## 0. How to read a `COMMISSION` tag

A `COMMISSION` marker does not mean "this number is wrong". It means
**nobody has measured it against this car**. Three distinct situations
hide under the one tag, and they need different work:

| Kind | Example | What commissioning owes you |
|---|---|---|
| Derived from a datasheet, never measured | `CurrentMaxMa`, `PackCapacityMah` | A measurement, or an explicit decision to trust the datasheet |
| Measured on one carrier, board-specific | `CurrentZeroCount` | A **re-measurement on this board** — the old value is not portable |
| Reasoned from first principles, unfitted | the SoC EKF variances | A sanity check against a reference; often "leave it" is the right answer |

The third kind is the trap. `SocEkfInitVar` and friends are marked
`COMMISSION` because they were derived, not fitted — but SoC is
**telemetry only** (`soc_estimator.hpp` says so in its first line; no
safety predicate reads it), so a wrong value costs accuracy, never
safety. Spend the bench day on the first two kinds.

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

### 1.1 The temperature limits are currently disarmed

`config::TempFaultsTrusted` is **false**. While it is, the
`CellUnderTemp` / `CellOverTemp` predicates are suppressed
(`safety_predicates.hpp:227` gates both branches on it), so setting
`CellOverTempC` correctly changes **nothing on a real car**.

The reason is honest and worth understanding before you flip it: the NTC
path runs through the per-LTC ADG731 mux, and that path is not validated
end-to-end on flight hardware. A mis-routed channel would fault the car
for no reason. Cell **voltage** protection is unaffected.

Two consequences for the bench session:

- Commissioning `CellOverTempC` is still worth doing — you are setting
  the number that will be live the day the mux path is validated — but
  do not report "over-temp protection commissioned". It is not armed.
- The **disconnect** side of the temperature path *is* armed
  independently (`TempSensorPresenceCheck`, §3b.3). An open NTC reads the
  divider rail regardless of how badly the conversion is calibrated, so
  that check does not depend on accuracy and does not wait for
  `TempFaultsTrusted`.

### 1.2 Debounce

A cell voltage/temperature breach must persist `CellFaultConfirmTicks`
(25 ticks ≈ 250 ms at `SafetyPeriodMs` = 10 ms) before it latches Error.
Combined with the 200 ms voltage poll the worst case is ≈ 460 ms, inside
the < 500 ms FS fault-response budget. The confirm window deliberately
spans **more than one** `BmsPollVoltMs` cycle so a single anomalous poll
cannot latch the car. If you change either number, redo that arithmetic.

---

## 2. Current sensor calibration

The pack current path uses a **Bourns SSA-2-250A** shunt sensor
(datasheet at `pcbs/ssa-2.pdf`). It is a 2-wire amplified-differential
device: its `OUT_P` / `OUT_N` pins carry a bipolar `±5 mV/A`
differential signal around a common-mode of `+1.44 V`.

`OUT_P` and `OUT_N` wire straight to the STM32 and the ADC reads them in
**differential mode**:

> `OUT_P = PF7 = ADC3_INP3`  ·  `OUT_N = PF8 = ADC3_INN3`

In STM32H7 differential mode the conversion encodes `V(INP) − V(INN)`
over `−Vref … +Vref` onto codes `0 … 4095`, with the zero-difference
point at **mid-scale (code ≈ 2048)**. The sensor common-mode (1.44 V)
cancels in the subtraction, leaving only the bipolar `±5 mV/A`:

> `raw ≈ 2048 + (5 mV/A × I) / LSB_diff`,  `LSB_diff = 2·Vref/4095 ≈ 1.61 mV`

Nominal (ideal) calibration:
- **Zero current** → ADC code ≈ 2048 (`CurrentZeroCount`, a *count*, not
  a voltage — the natural reference in differential mode is the mid code)
- **Sensitivity** = the bare sensor 5 mV/A: ideal `CurrentMvPerAmpe1 = 50`
  (i.e. 5 mV/A × 10)

> **`ams_config.hpp` ships bench-measured figures, not the ideal ones.**
> A DAC injection verified at exactly 5 mV/A measured the firmware
> reading a stable **0.924× (7.6 % low)** with a **+0.6 A** zero. Folding
> that effective ADC/VREF gain into the COMMISSION constants gives
> **`CurrentMvPerAmpe1 = 46`** (50 / 0.924, residual +0.4 %). The shipped
> **`CurrentZeroCount = 2054`** is the flight carrier's zero; the HIL
> bench carrier read 2050 on the same firmware.
>
> **The zero is board-specific — re-measure it on every new carrier.**
> It tracks VREF+, which is a per-board tolerance term. The gain is a
> VREF+ term too, but it read back exact against an aux-PSU known current
> on the second carrier, so in practice only the zero moves.

- **Sign convention**: discharge → `OUT_P` above `OUT_N` → raw above
  mid-scale → positive mA. Charge does the opposite.
- **Observable range**: the differential pair spans ≈ `±Vref`, well
  beyond the sensor's own `±2.62 V` (≈ `±524 A`) clip. The over-current
  threshold is therefore genuinely reachable in the ADC's range — see
  §2.3.

The sensor DC offset (≤ ±0.4 mV) plus the ADC offset shift the zero code
by a few LSB; calibrate before a flight build is tagged.

### 2.1 Zero-offset

1. Disconnect the pack from anything that draws current.
2. Read the raw differential ADC code for the pack channel (CH3,
   `ADC_DIFFERENTIAL_ENDED`) — see `current_task.cpp`.
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
   table.

### 2.3 Absolute limit (`CurrentMaxMa`)

`CurrentMaxMa` is **185 000 mA (185 A)**, and the number is a *cell*
rating, not a fuse or contactor rating: the pack is 95S6P of VTC6 rated
30 A continuous per cell, so a series element sustains `6 × 30 = 180 A`.
The sensor is a 250 A part, so the limit sits inside what the front end
can actually measure.

**There is no debounce on this predicate.** All the smoothing comes from
the filter feeding it: `filtered_mA` is a first-order IIR with
`CurrentFilterShift = 4` at `CurrentPeriodMs = 50`, i.e. `tau ≈ 800 ms`.
Trip time is `tau · ln(I / (I − limit))`:

| Sustained current | Time to open the SDC |
|---|---|
| 200 A | 2.1 s |
| 250 A | 1.1 s |
| 300 A | 0.8 s |
| 400 A | 0.5 s |

So a brief inrush rides through while a sustained overload still opens
the contactors in about a second.

**The corollary is the part people get wrong:** currents between the cell
rating and this limit *never trip at all*, by design. Protection against
a slow overload is the cell **temperature** path, not this one — and that
path is currently disarmed (§1.1). Note that gap when you sign off.

**COMMISSION:** this figure is derived from the cell datasheet and has
**not** been measured against the car's real draw, nor validated against
the contactor and fuse ratings. Confirm the inverter peak current and the
ratings of every element in the shutdown circuit before trusting it.

### 2.4 DCDC current channel

DCDC supply current uses an **Allegro ACS758** Hall-effect sensor (a
different part from the pack SSA-2) on `PC1 = ADC3_INP11`, read
**single-ended** through a unity buffer (gain 1). The ACS758 is
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
sits in `[CurrentLegPlausMinMv, CurrentLegPlausMaxMv]`
(`current_service.cpp:69`); after `CurrentDisconnectConfirm` consecutive
out-of-window reads (`current_task.cpp:205`) it asserts `sensor_fault` →
the `CurrentSensorFault` predicate (reason **8**,
`safety_predicates.hpp:52`) latches Error, opens the AIRs, drops
`AMS_OK`. (An OUT_N-only break instead skews the differential and trips
`CurrentOverLimit` — between the two, every disconnect mode is covered.)

Nominal window `700..2300 mV` brackets the connected OUT_P swing
(≈ 0.94–1.94 V across the sensor's ±200 A working range) with margin;
disconnect reads ≈ 0 V.

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

## 3. Precharge and re-arm

Constants tagged `COMMISSION` in this area:

- **`PrechargeRatio`** (0.95, not formally tagged but bench-confirm it) —
  in **Car** mode, AIR+ closes once `dc_bus_V ≥ 95 % × pack_voltage`.
  Real cars typically end up at 0.95–0.98. (Charger mode has no
  `dc_bus_V`; it proceeds on a still-fresh `0x101` charge request — see
  §3.3.)
- **`PrechargeMaxMs`** (5000 ms) — the hard ceiling on how long Precharge
  may run before the FSM latches `Error` and opens every contactor
  (`state_machine.hpp:280`). This protects the precharge resistor
  (transient-duty rated) for **any** stuck cause, including a dead-VCU
  car that mislocks Charger and could otherwise sit in Precharge forever.
- **`DcBusDischargedV`** (60 V) — the re-arm gate. See §3.5.

### 3.0 The freshness requirement in `precharge_target_reached`

`precharge_target_reached` takes `dc_bus_fresh` and returns false without
it (`state_machine.hpp:111`). This is **required, not advisory**, and the
reason is the single most important thing to understand before you
bench-test precharge:

`VehicleState` holds the **last received** `dc_bus_V`. When the VCU stops
publishing `0x100` the number does not go away — it **freezes**. Frozen
at pack voltage it satisfies the 95 % criterion forever, *including after
the link has actually bled to zero*, where closing AIR+ means full pack
voltage across the contactor with nothing to limit the inrush.

`VcuStale` cannot save you here: it is gated on `vcu_required` (false in
Start, so the value may already be arbitrarily old when the operator
presses) and needs `VcuStaleMs` = 200 ms, while the FSM steps every
20 ms. Precharge → Transition would fire on the frozen reading roughly
ten steps before the fault could reopen the AIRs. Freshness has to be
*part of the criterion*, not a separate fault racing it.

**Bench-test it explicitly:** enter Precharge in Car mode, then stop the
`0x100` publisher while the bus is above target. The FSM must **stay in
Precharge** and time out to Error via `PrechargeMaxMs` — it must not
advance to Transition.

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

In **Charger** mode the resistor is never in the loop (§3.3), so there
`PrechargeMaxMs` is simply the failsafe ceiling on how long the FSM waits
for a fresh `0x101` before latching Error. Set the constant for whichever
of the two constraints is tighter.

### 3.3 Charger mode skips the resistor

On `Start → Precharge` the FSM emits `CloseAirN` alone in Charger mode,
versus `CloseAirN | ClosePrecharge` in Car mode
(`state_machine.hpp:258-263`) — the precharge resistor never enters the
charge loop, because the charger voltage-matches its own output and
soft-starts.

Consequently the precharge-complete criterion is mode-specific
(`state_machine.hpp:300-304`): Charger proceeds on `0x101` **freshness**
(`charge_requested`), Car on the resistor-thermal-bounded `dc_bus_V` ramp
(`precharge_target_reached`). So `PrechargeRatio` and the `BusCollapse*`
sag margins are **Car-mode concerns only**.

Transition is a single FSM-step passthrough: the AIR+ close + precharge
contactor open is emitted on the `Precharge → Transition` edge, and the
next step commits to `Run` or `Charge`. A **Car-only** bus-still-at-target
guard remains (`state_machine.hpp:323`), so a failed contactor swap (bus
slumps the moment the precharge contactor opens) lands in Error rather
than energising on a degraded bus. Charger skips the guard — there is no
VCU `dc_bus_V` during a charge.

### 3.4 Bus-collapse detector (`BusCollapsePercent`, `BusCollapseConfirmTicks`)

In `Run` (Car mode) the VCU `dc_bus_V` tracks the pack. A **cockpit SDC
shutdown** opens the AIRs without the AMS sensing it — so the AMS watches
for the bus collapsing while it still thinks it's in `Run`, and falls
back to `Start` (non-latching) so a re-arm re-runs precharge instead of
reclosing AIR+ onto a discharged DC-link. Two `COMMISSION` constants:

- **`BusCollapsePercent`** (50) — `dc_bus_V` below this % of the pack
  (cell-sum) counts as "collapsed" (`bus_below_collapse`,
  `state_machine.hpp:160`). Pick it **above** the worst-case loaded sag
  of `dc_bus_V` vs the cell-sum (so hard acceleration/regen never
  false-trips) but **high enough** that it fires before the DC-link
  discharges to a voltage where a no-precharge reclose would be damaging.
  Measure both on the bench: log `dc_bus_V / pack_voltage_mV` under
  maximum load (sag floor), and the link's discharge curve after an
  AIR-open, then set the percent comfortably between them.
- **`BusCollapseConfirmTicks`** (20 ≈ 200 ms @ 10 ms) — consecutive
  collapsed ticks before de-energising. Long enough to reject a single
  anomalous `0x100` frame; short enough to trip before a released
  shutdown recloses the AIRs.

Note that `bus_below_collapse` deliberately does **not** take
`dc_bus_fresh`: it is only consumed in Run, where mode is locked to Car,
so `vcu_required` is true and `VcuStale` bounds staleness at 200 ms — the
same 200 ms its own debounce already spends. Both stale outcomes are safe
(a false collapse de-energises to Start without latching; a missed one is
caught by `VcuStale`), so it has no race to lose. That asymmetry with
§3.0 is intentional, not an oversight.

Verify on the bench: in `Run`, force the AIRs open (or inject a low
`dc_bus_V` via the VCU/HIL fixture); confirm the FSM returns to `Start`
(not Error) after the debounce, AMS_OK stays HIGH, and a DASH_CHG re-arm
runs a full precharge.

### 3.5 DC-link discharge interlock (`DcBusDischargedV`)

**Read this before commissioning `DcBusDischargedV`, because the number
only makes sense once you know why the interlock exists.**

The bleed relay is **normally-closed** and wired into the shutdown
circuit with **no software control**. Opening the SDC de-energises it, so
the bleed resistor connects and the DC link drains. But closing the SDC
again re-energises the relay and **the discharge stops part-way**,
stranding the link at a voltage nobody can predict from how long ago the
SDC was cycled.

The AMS cannot restart it. Its own leg of the loop (`AMS_OK`) latches in
**hardware** — a self-holding relay plus an RST_BMS button the driver
cannot reach — so firmware cannot pulse it low to re-open the SDC. So the
AMS publishes the two facts only it can observe —
`0x021 ACU_discharge_interlock` (`fsm_in_start`, `tsms`) — and the ECU,
which owns both a DC-link measurement and a normally-closed relay in
series with the bleed relay's coil, decides.

The AMS side consumes `0x100` byte 2 bit 0 (`discharge_engaged`) and
gates `Start → Precharge` through `fsm::rearm_permitted`
(`state_machine.hpp:144-151`). Two independent refusals, because they
fail differently:

- **`discharge_engaged`** — the ECU says the bleed resistor is connected
  across the link. Closing a contactor now would put pack current through
  a transient-duty resistor. Hard interlock, honoured whatever the
  voltage reads.
- **`dc_bus_V > DcBusDischargedV`** — the link is still charged, so a
  precharge would be a *no-op*: the 95 % completion criterion is already
  satisfied on entry, the resistor never carries meaningful current, and
  that 95 % check is the only evidence the AMS ever gets that the
  precharge resistor and contactor actually work. Satisfied by residual
  charge, it proves nothing.

The second refusal is enforced **only** once the ECU has shown it speaks
the protocol (`ecu_discharge_capable`), because an ECU that cannot drain
a stranded link also cannot clear the block — enforcing it against older
ECU firmware would brick the car rather than protect it.

A blocked press holds in `Start` rather than latching: the driver waits
out the discharge and presses again, no reset. The press **is** consumed
on a blocked attempt, deliberately — carrying it would let a press made
while the link was live arm the car by itself seconds later, when nobody
is expecting it.

**Commissioning `DcBusDischargedV`** (60 V): it is absolute volts, not a
fraction of pack, because the number is the touch-safe DC limit the
discharge is *required* to reach and it does not scale with pack voltage.
Take it from the current rulebook. **It must sit at or above the ECU's
own release threshold** — if the AMS demands a lower voltage than the ECU
drains to, the two disagree about the boundary and the AMS waits forever
on a link the ECU has stopped draining. Confirm the ECU's threshold
directly with whoever owns that firmware; do not assume it. At the time
of writing it is `DischargeReleaseV` = 10 V in `IFS08-CE-ECU`
`ecu_config.hpp`, comfortably below 60 V — but that is a number in
another repository and re-checking it is part of this step.

> **The interlock is live against ECU `dev` and inert against ECU `main`.**
> `dev` sends `0x100` at DLC 3, so `ecu_discharge_capable` latches and both
> refusals bite. `main` still sends DLC 2, where the bit reads 0 and nothing
> here changes. **Check which image is on the car before you rely on either
> statement**, and note that neither path has run on hardware: the AMS side
> is verified only by `tests/unit/test_state_machine.cpp` and the ECU side
> only by its SIL.
>
> Until the pairing is bench-proven, treat a stranded DC link as a manual
> hazard regardless of which image is flashed.

---

## 3b. NTC thermistor calibration (LTC6811 + ADG731)

200 NTCs (5 modules × 40 per module = 20 per LTC × 2 LTCs) are selected
through the ADG731 32:1 mux on each BMS_LITE board, buffered onto LTC6811
GPIO1, sampled with ADAX(GPIO1), and converted into °C in
`BmsService::update_temperature`.

### 3b.1 The conversion is a table lookup, not a Beta fit

```
R_ntc = NtcPullupOhm * V_aux / (NtcVrefMv - V_aux)
T     = ntc::lookup(R_ntc)          // Core/Inc/app/ntc_table.hpp
```

`ntc_table.hpp` is generated from `docs/ntc_rt_table.csv` — the
manufacturer's R-T appendix for the fitted part, a **Fenghua
CMFB103F3950FANT** (R25 = 10 kΩ ±1 %, B25/50 = 3950 K). It covers
−55…+125 °C in 1 °C steps and is exact at every listed point.

**There are no `NtcBeta` or `NtcR25` constants.** A single-beta
Steinhart approximation is only accurate near its fitting interval — at
−40 °C it misestimates R by tens of percent, worth several degrees — and
the header explains the concrete failure that motivated the table: two
wrong constants (a Murata beta of 3380 against the fitted 3950, and a
10 kΩ pull-up against the real 6.8 kΩ) partially cancelled, leaving the
path reading ~7 °C **cold** at 50 °C. `BalanceTempMax = 50` therefore
tripped at ~56 °C true and `CellOverTempC = 60` at ~65 °C true — both
above the cell's rated envelope. Fixing either constant alone makes it
worse, which is why the pull-up and the table had to land together.

The lesson for commissioning: **do not "tune" one of these constants to
make a reading match.** Two errors can cancel. Verify the divider
resistance with a meter and the part number against the BOM, then trust
the table.

| Constant | Default | What it is |
|---|---|---|
| `NtcPullupOhm` | 6800 | R145 / R170 divider pull-up to VREF2 — **measure it** |
| `NtcVrefMv` | 3000 | LTC6811 VREF2 nominal, buffered by U6 — **measure it** |

Cross-check: `tools/bms_monitor.py` runs the same conversion on the
bare-LTC bench harness (`RPULL_KOHM = 6.8`, `VREF2_MV = 3000`).

### 3b.2 Bench procedure

1. Bring the pack to a known soak temperature (use an ambient probe on
   the same board if you have one — or 25 °C in still air after a
   30-minute warm-up).
2. Read `BmsState.cell_tempC[m][t]` for every (m, t) — the per-NTC grid
   is on pit-diag `0x6A0..0x6B8` (25 frames × 8 temps = 200), armed by
   sending the magic on `0x7F0`.
3. The median across all 200 NTCs should sit within ±2 °C of the
   reference. If it does not, the fault is in the **divider** or the
   **part**, not in a fitting constant — check `NtcPullupOhm` and
   `NtcVrefMv` against the board before anything else.
4. Re-soak at a second point (e.g. 25 °C and 60 °C with a hot-air gun on
   one cell) to confirm the curve, not just the offset.

`Adg731ChannelMap` (20 entries) is the lookup from temperature index `t`
(0..19) to ADG731 channel (0..31), from the schematic walk of
`pcbs/BMS_LITE/LTC_1.kicad_sch`: S1..S10 → ch 0..9 → NTC_1..NTC_10,
S17..S26 → ch 16..25 → NTC_11..NTC_20. The 0-indexed channel passed to
`pack_adg731_select` is one less than the schematic's `S<n>` pin number.
LTC_2's mux (U5) mirrors this map. Slot numbering across a module:
**LTC_1 (upper) → slots 0..19, LTC_2 (lower) → slots 20..39.**

### 3b.3 Disconnect detection — armed, and it will stop the car

Independently of `TempFaultsTrusted`, the firmware faults a
**disconnected** temperature sensor (FS rule: an open temp sensor must
open the SDC). This works regardless of calibration accuracy, because an
open NTC reads the divider rail whatever the conversion constants say.

| Constant | Default | Meaning |
|---|---|---|
| `NtcOpenMv` | 2800 | AUX mV at or above this ⇒ **open**, not cold |
| `TempSensorPresenceCheck` | true | arms the disconnect fault |
| `TempDisconnectPolls` | 1 | consecutive open polls before latching |
| `NtcNoReading` | −32768 | sentinel for "never produced a valid reading" |
| `RequiredTempSlots` | all 40 | slots that must be present on every online module |

Two design points a new member needs:

- **`NtcOpenMv` is below VREF2 on purpose.** A partially-railed open —
  mux leakage, a long or damp sense harness, a high-impedance fault —
  settles a few hundred mV below the rail (~2.7–2.95 V observed). Read
  literally that decodes to a very cold *but in-range* temperature
  (2.9 V ≈ −35 °C), so a bare `>= NtcVrefMv` test would let a real
  disconnect masquerade as a plausible cold reading. 2800 mV maps to
  ≈ −20 °C on the 6.8 k / VREF2 divider — colder than any operating cell
  — leaving ≥ 140 mV of margin either way.
- **The sentinel is `NtcNoReading`, not 25 °C.** An unconverted channel
  seeded to a plausible room temperature is the single most dangerous
  value it could hold: `max_tempC` looks healthy no matter what the pack
  is doing, and every threshold built on it is defeated. A sentinel makes
  "no data" distinguishable from "cool".

**COMMISSION — this is the item most likely to stop your bench day.**
`RequiredTempSlots` lists **all 40 slots**, and a required slot reading
open faults immediately *without* the "seen valid once" latch — so a
channel already open at power-on is caught, which is what makes the
disconnect deterministic for scrutineering. The consequence: **a
genuinely open channel anywhere on the flight harness latches Error at
boot and the pack will not arm until it is repaired.** That is intended
behaviour, not a bug. Sweep the harness for continuity on every module
before you expect the car to arm.

`TempDisconnectPolls = 1` gives a worst-case detection of one sweep
cadence (`BmsPollTempMs` = 250 ms) + the ~100 ms sweep + a 10 ms safety
tick ≈ 360 ms, inside the 500 ms rule. The trade-off is an explicit one:
there is **no** single-glitch debounce left, so a one-off mux glitch
reading ≥ `NtcOpenMv` can nuisance-latch Error. Watch for that on the
bench. If a genuine glitch source appears, add headroom to
`BmsPollTempMs` and reinstate a 2-poll debounce that still fits inside
500 ms — do **not** widen `NtcOpenMv`, which would re-open the
"disconnect looks cold" hole.

Slot 0 is safe to require only because of the mux warm-up: BmsPollTask
does a throwaway select to unpopulated S32 first, absorbing the ADG731
first-select droop that would otherwise make temp 1 read open on the
first sweep of every module.

---

## 3c. Cell balancing (LTC6811 WRCFGA / passive)

Once per `BalanceUpdatePolls` voltage-poll cycles BmsPollTask snapshots
`BmsState`, runs the `ams::balance::compute_mask` policy, packs the
per-IC DCC bits into WRCFGA payloads, and broadcasts.

At `BmsPollVoltMs` = 200 ms and `BalanceUpdatePolls` = 4 that is one
update every **800 ms**.

### 3c.1 When balancing is allowed to run

`compute_mask` (`balance_controller.hpp:95`) refuses in this order, and
every gate matters:

| Gate | Refuses when |
|---|---|
| Operator command | `op_cmd == Off` (which is also the **stale / never-seen** fallback) |
| FSM state | `op_cmd == Auto` and state ≠ `Charge` |
| Cell-data fault | a latched fault says the cell voltages are wrong |
| Temp trust | `BalanceTempsTrusted` false |
| Temp data | `valid_temp_channels < BalanceMinValidTempCh` |
| Thermal lockout | `max_tempC > BalanceTempMax` |

The operator master switch is `0x103`, magic-gated (`"BALO"` → OFF,
`"BALN"` → ON in **any** FSM state, `"BALX"` → AUTO). The pit tool
re-sends it at ~2 Hz; if the frame goes stale past
`BalanceOverrideFreshMs` (5 s) or was never seen, the effective command
falls back to **OFF**, so a dead pit link never leaves the pack bleeding.
`0x104` layers a per-module enable mask underneath it; its own dead-man
defaults to **all modules enabled**, so a lost `0x104` cannot silently
disable balancing — `0x103` remains the safety net.

**The operator overrides the ENABLE decision, never the guards.** `"BALN"`
skips the Charge-state gate; it does not skip the thermal or data gates.

The two data gates exist because `max_tempC` is `INT16_MIN` when nothing
has converted, and `INT16_MIN` compares as wonderfully cool — without
them a pack with a dead temperature path would balance with no thermal
protection at all and no symptom.

> **RESIDUAL RISK — `BalanceTempsTrusted` is `true` while
> `TempFaultsTrusted` is `false`.** These are deliberately separate
> questions: *"do we trust these temps enough to open the contactors?"*
> (no) versus *"do we trust them enough to let balancing run?"* (yes,
> provisionally). Coupling them meant the pit balance toggle was accepted
> and then produced an all-zero mask forever. But with the split in
> place, balancing dissipates into the cells while its **only** thermal
> guard reads a path we have not validated. The mitigations are the 5 s
> operator dead-man, the per-module active cap, and the
> `BalanceDeltaMv` selection floor. **Balance with cell temperatures
> observed by some other means until the ADG731 path is validated
> end-to-end.** When `TempFaultsTrusted` goes true this flag becomes
> redundant and should be deleted.

### 3c.2 Tunables

| Constant | Default | Effect |
|---|---:|---|
| `BalanceDeltaMv` | 50 mV | START threshold: discharge a cell above `floor + delta`. |
| `BalanceStopDeltaMv` | 20 mV | STOP threshold (hysteresis). Must stay **below** `BalanceDeltaMv` — a `static_assert` enforces it, because a stop at or above start latches a cell on forever. |
| `BalanceMaxActive` | 8 | Max cells per module discharging at once. **Board dissipation limit, not policy.** |
| `BalanceSpreadNoAdjacent` | true | Never bleed two physically adjacent cells at once. |
| `BalanceTempMax` | 50 °C | Inhibit all balancing if `max_tempC > this`. |
| `BalanceUpdatePolls` | 4 | Voltage-poll cycles between WRCFGA updates (= 800 ms). |
| `BalanceMinValidTempCh` | 5 | Minimum converted temp channels before balancing may run. |
| `BalanceQuiesceMs` | 2 | DCC-clear settle before a measurement. See §3c.4. |

Why the hysteresis: `compute_mask` is **stateless** and re-picks the
top-K every cycle, so without a separate stop threshold any cell sitting
near `BalanceDeltaMv` toggles on and off continuously — selected, bleeds
a little, drops below the single threshold, dropped, recovers, selected
again. The bleed is real but the duty is a fraction of what the operator
sees on the mask, and the DCC pattern is unreadable on the bench. The
20 mV stop against a 50 mV start gives a 30 mV band, comfortably wider
than the 9–36 mV harness-IR artifact a bled cell shows against its
neighbours.

Why the floor is the **second**-lowest cell and not the minimum: a
disconnected tap reads spuriously low, and if that one bad cell set the
floor then every real cell would sit more than `BalanceDeltaMv` above it
and the **whole stack** would start balancing off a single faulty
reading.

### 3c.3 Dissipation budget

BMS_LITE per-cell bleed path (per-cell schematic sheet): an external
TSM2323 PMOS switching R71 ‖ R72 = 47R ‖ 47R = **23.5 Ω**, both 2512.

| cell V | current | W per cell | W per 2512 (2 W part) |
|---|---|---|---|
| 4.2 V | 179 mA | 0.75 W | 0.37 W (~19 % of rating) |
| 4.0 V | 170 mA | 0.68 W | 0.34 W |
| 3.7 V | 157 mA | 0.58 W | 0.29 W |

At `BalanceMaxActive` = 8 that is **6.0 W per module, 30 W across the
pack** at the 4.2 V worst case.

**The resistors are the comfortable part** — 2 W devices running under a
fifth of rating. The real constraint is heat **out of the sealed
accumulator box**, and that is unchanged by how good the resistors are.
Raising `BalanceMaxActive` makes balancing proportionally faster; it does
not unlock cells that were stuck, because the selector is stateless and
re-picks every cycle anyway.

**COMMISSION:** board temperature in the sealed accumulator is **not
measured**. The ~71 °C figure behind `BalanceSpreadNoAdjacent` is a
single pad on an **open bench**. Measure board temperature at this
setting in the real box before trusting it — and note the circularity:
the `BalanceTempMax` lockout that would catch an overheating board reads
the same unvalidated NTC path (§3c.1).

Procedure:

1. With the pack in Charge on the bench (or `"BALN"` asserted on `0x103`),
   watch `g_balance_cycles_active` climb whenever any cell sits above the
   threshold. `g_balance_cycles_total` increments unconditionally, so the
   ratio is the active fraction. Both are published on the pit-diag
   stream.
2. Verify per-board dissipation with a clamp meter on the supply rail to
   one BMS_LITE during a known-imbalanced soak. If the board runs above
   its thermal budget, drop `BalanceMaxActive` first, then raise
   `BalanceDeltaMv` to tolerate a wider equilibrium band.
3. Confirm `WRCFGA → RDCFGA` round-trips the DCC bits you intended.

### 3c.4 Quiesce before measure — do not "optimise" this away

`BalanceQuiesceMs` = 2 ms of settle after clearing the DCC bits and
before starting a cell-voltage conversion, so no bleed current flows
while the cells are measured.

**The LTC6811 ADCV `DCP=0` bit is not sufficient on this board.** DCP=0
does make the LTC suspend its own S-pin switch for the conversion, but
BMS_LITE does not bleed through that switch — each cell drives an
external TSM2323 PMOS whose gate sits behind R167 (10 k) / C32 (10 n),
`tau ≈ 100 µs`. The conversion starts immediately on ADCV and the first
channels convert in a few hundred microseconds, the same order as the
gate turn-off, so the earliest cells can be sampled while current is
still flowing.

Why it matters: the bleed current does **not** return through the sense
path (on-board sensing is close to Kelvin) — it returns through the
harness. 179 mA across a plausible 50–200 mΩ of tap/connector/fuse
impedance is **9–36 mV**, with **opposite sign** on the bled cell (reads
low) and its neighbours (read high, because the shared tap node moves).
Against `BalanceDeltaMv` = 50 mV that is a first-order corruption of the
very signal balancing selects on — observed on the bench as neighbouring
cells reading high whenever balancing is active.

2 ms is ~20× the gate RC and covers the LTC input-filter settle, at a
cost of under 1 % of balancing duty.

**Bench check:** the quiesce can *fail* (the DCC-clear write does not
land). `g_balance_quiesce_count` / `g_balance_quiesce_fail_count` are
published on pit-diag `0x6CB` precisely because DCP=0 does not cover a
failed quiesce — a failing quiesce means cell voltages are being sampled
under bleed, and the balance selector ranks exactly those numbers.
Confirm the fail count stays at zero across a long balancing session.

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
| BmsPollTask voltage poll | ~200 ms (`BmsPollVoltMs`) |
| CurrentSensorTask ADC sample | 50 ms (`CurrentPeriodMs`) |
| BmsPollTask temperature sweep | ~250 ms (`BmsPollTempMs`) |
| AcuCanTask VCU `0x100` ingest | uncontrolled — depends on the vehicle bus |

2000 ms covers the worst of these plus a comfortable margin for a slow
CAN bus startup. Tune down only if a faster Start state matters more than
the margin; tune up if a planned producer takes longer than 2 s to
publish on a bench / vehicle.

> **Do not set `SafetyBootGraceMs = 0`.** With no grace the very first
> MainTask iteration faults on freshness and withholds the watchdog
> refresh, so the chip enters an IWDG reset loop within ~100 ms — and the
> loop is unrecoverable without reflashing over the CAN bootloader.

---

## 4. BMS freshness window

`BmsStaleMs` is **350 ms**, and it is a fault-response-budget number, not
a comfort margin. "Stop measuring ANY voltage or temperature" includes a
whole module going silent, which must open the SDC in **< 500 ms** per FS
rule.

How the budget spends:

- A silent module drops off `module_online_mask` when its freshness
  exceeds `BmsStaleMs` at a voltage poll, firing `BmsModuleOffline`
  (immediate, **not** debounced).
- Worst case: 350 ms staleness crossed at the 2nd voltage poll after loss
  (~400 ms) + one 10 ms safety tick ≈ **410 ms**.
- `350 > BmsPollVoltMs` (200 ms), so **one** missed poll is tolerated
  (age 200 ≤ 350); two consecutive misses (400 ms) trip it.

That tolerance is what stops a single dropped poll opening the
contactors, and it is the reason the number is not simply "as small as
possible".

**This is only safe with bounded poll jitter.** It relies on the
temperature sweep no longer head-of-line-blocking the voltage poll (see
`run_temperature_poll`'s yield-to-`PollVDue`). **COMMISSION:** confirm on
the HIL bench that there are no nuisance trips and measure the actual
margin before flight.

`BmsStaleConfirmTicks` (25 ≈ 250 ms) is a **secondary** path and also
`COMMISSION`. It debounces the per-module freshness-loop predicate so a
far module that flickers just past the window under a brief EMI burst,
then reports on its next voltage poll, does not spuriously open the
contactors. It does **not** add to the module-loss detection budget,
because `BmsModuleOffline` pre-empts it. 250 ms still spans more than one
200 ms voltage poll. Setting it to 0 restores first-tick latching.

---

## 5. Watchdog window

The IWDG runs at LSI ~32 Hz with ±47 % tolerance. Values in `main.c`:

- `prescaler = 32` (`IWDG_PRESCALER_32`)
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

Cooling fans are wired permanently on. There is no firmware fan-duty
constant to commission and no fan PWM in the build.

---

## 7. Acceptance test

Before tagging a flight release, the AMS must pass the following on the
actual vehicle:

1. Cold-boot from VBAT-only → AIRs read open via clamp meter.
2. Press start (TSMS held + DASH_CHG press) with a healthy pack → reach
   `Run`. Note that `SafetyBootGraceMs` (2 s) bounds how early this can
   happen after power-on.
3. From `Start` with the VCU `0x100` heartbeat absent and a fresh `0x101`
   charge request, press DASH_CHG (TSMS held) → mode locks Charger →
   `Precharge` (AIR− only, precharge resistor skipped) → `Transition` →
   `Charge`; confirm AIR+ closes and the precharge contactor never closes.
   (Charge cannot be entered from `Run` — mode is locked at Start→Precharge
   and never re-evaluated.)
4. Force-open any BMS module (pull its isoSPI cable from the chain) →
   `Error` in **< 500 ms** (expect ≈ 410 ms via `BmsModuleOffline`), AIRs
   open, backup register flag set.
5. Open one cell-temperature channel on any online module → `Error` in
   **< 500 ms** (expect ≈ 360 ms), reason `TempSensorDisconnected`.
6. Stop the VCU `0x100` publisher mid-Precharge with the bus above target
   → the FSM must stay in Precharge and time out to Error, never advance
   to Transition (§3.0).
7. Power-cycle after step 4 → AMS comes up in `Error`, AIRs stay open,
   requires the manual reset procedure (§8). Confirm the flight image was
   **not** built with `AMS_HIL_CLEAR_ERROR_LATCH`
   ([`HIL_BUILD.md`](HIL_BUILD.md)), which would defeat this step.
8. 30-minute capture of telemetry frames `0x4A0` / `0x4A1` / `0x4A2`
   on FDCAN1; confirm cadence is 500 ms ± 5 ms, no dropped frames,
   no garbage decode.

Sign off each step in the project log with date, scrutineer name, and
the firmware commit SHA (also readable from `0x6C6`, see
[`HIL_BUILD.md`](HIL_BUILD.md)).

---

## 8. Operator reset procedure

**Still an open decision.** What the firmware does today: `ErrorLatch`
lives in `RTC->BKP_DR1` behind magic `0xA115EE51` and is deliberately
sticky, so a latched fault survives a reset and the board boots straight
into `Error`. There is **no in-firmware clear path** on a flight build —
only physical loss of backup-domain power clears it, and whether the
flight carrier even has a VBAT source is unconfirmed (see
[`HIL_BUILD.md`](HIL_BUILD.md)).

Options that have been discussed:

- Reset-only via backup-domain power loss (what happens today by default)
- Long-press the charge button to clear the latch from `App_InitTask`
- A dedicated magic-gated CAN command, like the existing `0x002`
  bootloader-reboot frame

Lock this in before commissioning. The choice affects `App_InitTask` boot
logic, and step 7 of the acceptance test cannot be signed off without a
defined gesture.
