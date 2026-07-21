# Commissioning checklist — AMS v1.6.2

One-page sign-off sheet for every `COMMISSION`-tagged constant in
[`Core/Inc/app/ams_config.hpp`](../Core/Inc/app/ams_config.hpp). These are
**placeholder defaults** — each must be confirmed or measured on the real
pack / inverter / BMS_LITE hardware **before a flight build is tagged**.
Procedures are in [`COMMISSIONING.md`](COMMISSIONING.md); this sheet is the
checklist + record.

> **Workflow:** measure → set the value in `ams_config.hpp` → rebuild →
> re-run the affected host tests + the relevant HIL row (#399) → tick the
> box and record the final value, who, and the date. **🔒 = safety-gating**
> (feeds a predicate that opens the AIRs / latches Error, or a contactor
> sequence). Two of these (`BusCollapse*`) also move HIL row **C-049**.

**Build / verify after edits:**
`cmake --build build && cmake -B build-tests -S tests/unit && cmake --build build-tests && ctest --test-dir build-tests`

---

## 1. Cell voltage & temperature limits  🔒  (`COMMISSIONING.md` §1)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `CellUnderVoltageMv` | 2800 | mV | Cell datasheet min discharge cutoff (+ margin for sag). | | ☐ |
| `CellOverVoltageMv` | 4200 | mV | Cell datasheet max charge voltage (− margin). | | ☐ |
| `CellOverTempC` | 60 | °C | Cell datasheet max temp (− margin); cross-check with `BalanceTempMax`. | | ☐ |

> **Under-temp is not commissioned** for this pack — `CellUnderTempC` keeps
> its default (−10 °C). The predicate still exists but isn't a measured
> bench item.

> Debounce: a breach must persist `CellFaultConfirmTicks` (30 ≈ 300 ms)
> before latching — confirm that window is acceptable for your cell, not a
> value to tune blind.

## 2. Pack current sensor  🔒  (`COMMISSIONING.md` §2)

Bourns SSA-2-250A shunt read **differentially** on `PF7`/`PF8`
(ADC3_INP3/INN3). **Calibrate the two cal constants before trusting the
over-current trip.**

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `CurrentZeroCount` | 2054 | ADC counts | Zero-current differential code (pack open / 0 A). HIL-commissioned (#348) at 2050; flight-carrier re-cal (2026-06-20, e729f1f) moved it to 2054 — the offset tracks VREF+, so it is board-specific (ideal 2048). Record the actual mid-scale offset per carrier. | | ☐ |
| `CurrentMvPerAmpe1` | 46 | 0.1·mV/A | Sensitivity ×10. HIL-commissioned (#348): ideal 50 (5 mV/A) trimmed for a measured 0.924× ADC/VREF gain → trip at 200 A real. Re-measure per carrier. | | ☐ |
| `CurrentMaxMa` | 200000 | mA | Over-current trip = pack/fuse continuous rating (− margin). Now genuinely reachable on this HW rev. | | ☐ |

DCDC current (informational, not safety-gated) is an Allegro **ACS758**
Hall sensor single-ended on `PC1` (ADC3_INP11), ratiometric @ 3.3 V →
26.4 mV/A at a 1.65 V (Vcc/2) offset. Calibrate `DcdcCurrentZeroMv`
(1650) and `DcdcCurrentMvPerAmpe1` (264), and confirm the sign, per
`COMMISSIONING.md` §2.4.

## 3. Precharge & bus-collapse  🔒  (`COMMISSIONING.md` §3, §3.4)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `PrechargeRatio`*¹ | 0.95 | fraction | DC bus must reach this × pack before AIR+ closes. 0.95–0.98 typical. | | ☐ |
| `PrechargeMaxMs` | 5000 | ms | **Car mode:** **above** the measured worst-case healthy resistor-precharge time, **below** the precharge resistor's transient/pulse-energy rating. **Charger mode** skips the resistor (see note), so here it's the failsafe ceiling on how long the FSM waits for a fresh `0x101` before latching Error. Set for the tighter of the two. | | ☐ |
| `BusCollapsePercent` | 50 | % of pack | **Above** worst-case loaded sag of `dc_bus_V` vs cell-sum (so hard accel/regen never false-trips), **high enough** to fire before a no-precharge reclose is damaging. Measure both, set between. **→ moves HIL C-049.** | | ☐ |
| `BusCollapseConfirmTicks` | 20 (~200 ms) | 10-ms ticks | Long enough to reject one anomalous `0x100`; short enough to trip before a released cockpit shutdown recloses the AIRs. | | ☐ |

*¹ `PrechargeRatio` isn't formally `COMMISSION`-tagged but is bench-confirmed in §3 — verify it for this pack.*

> **Charger mode skips the resistor** (58f56c1): on Start→Precharge the FSM
> closes only AIR− (emits `CloseAirN`; Car emits `CloseAirN | ClosePrecharge`),
> so the resistor never enters the charge loop — the charger voltage-matches
> its output before asserting `0x101`. Charger's precharge-complete criterion
> is `0x101` freshness (`charge_requested`), not the resistor-thermal-bounded
> `dc_bus_V` ramp (`precharge_target_reached`) used in Car mode. So
> `PrechargeRatio` / `BusCollapse*` sag margins are Car-mode concerns; the
> Charger path is gated only by 0x101 freshness + `PrechargeMaxMs`.

## 4. NTC thermistor model  🔒 (feeds cell-temp)  (`COMMISSIONING.md` §3)

`R_ntc = NtcSeriesR · V_aux / (NtcVrefMv − V_aux)`, then Beta model.
Defaults match the BMS_LITE BOM (Murata NCP15XH103J); confirm against the
fitted parts + a reference-temperature reading.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `NtcBeta` | 3380 | K | Thermistor datasheet B-value (25/85). | | ☐ |
| `NtcR25` | 10000 | Ω | Thermistor R₂₅. | | ☐ |
| `NtcSeriesR` | 10000 | Ω | Measured pull-up resistor value. | | ☐ |
| `NtcVrefMv` | 3000 | mV | Measured LTC6811 VREF2 (buffered). | | ☐ |

> Validate by reading a few cells at a known reference temperature and
> confirming the decoded °C; adjust if off.

## 5. Cell balancing  (`COMMISSIONING.md` §3c)

Not AIR-gating, but `BalanceMaxActive` is bounded by the BMS_LITE balance
resistor dissipation + module airflow.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `BalanceDeltaMv` | 50 | mV | Imbalance above pack-min that starts a cell bleeding. | | ☐ |
| `BalanceTempMax` | 50 | °C | Abort balancing above this `max_tempC` (≤ `CellOverTempC`). | | ☐ |
| `BalanceMaxActive` | 8 | cells/module | Max simultaneous dischargers per module. 47R\|\|47R = 23.5R per cell → 0.75 W/cell @4.2 V, so 8 = 6.0 W per module / 30 W pack. Resistors (2512, 1 W) run ~37 % — the limit is heat out of the box. **Measure board temperature at this setting.** | | ☐ |

## 6. CAN map confirmation  (`COMMISSIONING.md` §7 / `CAN_MAP.md`)

| Constant | Default | How to determine | Final | ✓ |
|---|---|---|---|---|
| `ChargeModeReqId` | `0x101` | Confirm the operator charge-request ID is free in the ECU/VCU CAN map (no collision). | | ☐ |

---

## Sign-off

| Field | Value |
|---|---|
| Firmware version / SHA | `1.6.2` / `__________` |
| Pack / accumulator s/n | `__________` |
| Commissioned by | `__________` |
| Date | `__________` |
| All boxes ticked, host tests green, affected HIL rows re-run | ☐ |

After sign-off: tag the release (current: **v1.6.2**) and open the
`dev → main` release PR (this also unblocks the host pit-diag release,
#321).
