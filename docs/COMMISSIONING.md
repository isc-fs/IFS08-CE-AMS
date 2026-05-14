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

The Hall transducer on `PF11` (ADC1 ch 2) is assumed to be **2.5 V at
zero, 5.7 mV/A sensitivity**. Real units drift; calibrate before
v1.0.0.

### 2.1 Zero-offset

1. Disconnect the pack from anything that draws current.
2. Read the raw ADC value via debugger (`hadc1` → start → poll → get).
3. Compute the implied zero-voltage `v_mV = raw * 3300 / 4095`.
4. If `v_mV` differs from 2500 by more than 30 mV, update
   `kCurrentZeroMv` in `ams_config.hpp` to the measured value.

### 2.2 Sensitivity

1. Connect a calibrated current source in series with the pack.
2. Run **+10 A** discharge. Note the new ADC raw.
3. Run **−10 A** charge (regen or external charger). Note ADC raw.
4. Sensitivity = `(v_zero_mV − v_at_+10A_mV) / 10`, in mV/A.
5. Update `kCurrentMvPerAmpe1` (scaled ×10): set it to
   `round(sensitivity_mV_per_A × 10)`.
6. Confirm symmetry by comparing the −10 A reading; if it deviates >
   2 %, log a non-linearity warning in the project log and consider a
   per-direction calibration table (out of v1.0.0 scope).

### 2.3 Absolute limit

`kImaxMa` defaults to 200 000 mA (200 A). Set it to the lower of:
- The pack discharge rating from the FS rules
- The contactor's continuous rating
- The cell datasheet's `I_max_continuous`

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

The SafetyTask refreshes every 10 ms on the clean path, so even at
the LSI fast corner (52 ms) we have 5× margin. **Do not increase the
reload** without re-evaluating the SafetyTask period.

If the watchdog ever resets during normal operation, the failure
mode is almost certainly NOT the LSI tolerance — it's SafetyTask
being preempted or blocking on a mutex. Investigate before touching
the IWDG constants.

---

## 6. Fan duty cycles

`kFanDuty[]` in `state_task.cpp`:

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
4. Manual SDC open during `Run` → AIRs open within 50 ms (scope)
5. Force-open any BMS slave (unplug its CAN cable) → `Error` within
   `kBmsStaleMs + 20 ms`, AIRs open, backup register flag set
6. Power-cycle after step 5 → AMS comes up in `Error`, AIRs stay
   open, requires the manual reset procedure (TBD: define gesture)
7. 30-minute UART log capture with the line format from
   `telemetry_task.cpp`; confirm no drops, no garbage

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
