# Commissioning checklist — AMS v1.6.0

One-page sign-off sheet for every `COMMISSION`-tagged constant in
[`Core/Inc/app/ams_config.hpp`](../Core/Inc/app/ams_config.hpp). These are
**placeholder defaults** — each must be confirmed or measured on the real
pack / inverter / BMS_LITE hardware **before a flight build is tagged**.
Procedures are in [`COMMISSIONING.md`](COMMISSIONING.md); this sheet is the
checklist + record.

> **Workflow:** measure → set the value in `ams_config.hpp` → rebuild →
> re-run the affected host tests + the relevant HIL row (#317) → tick the
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
| `CellUnderTempC` | −10 | °C | Cell datasheet min charge/discharge temp. | | ☐ |
| `CellOverTempC` | 60 | °C | Cell datasheet max temp (− margin); cross-check with `BalanceTempMax`. | | ☐ |

> Debounce: a breach must persist `CellFaultConfirmTicks` (30 ≈ 300 ms)
> before latching — confirm that window is acceptable for your cell, not a
> value to tune blind.

## 2. Pack current sensor  🔒  (`COMMISSIONING.md` §2)

Bourns SSA-2-250A shunt on `PF7`/ADC3. **Calibrate the two cal constants
before trusting the over-current trip.**

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `CurrentZeroMv` | 1650 | mV | Zero-current ADC reading (pack open / 0 A). Record the actual offset. | | ☐ |
| `CurrentMvPerAmpe1` | 200 | 0.1·mV/A | Sensitivity ×10 (20 mV/A nominal). Inject a known current, solve mV/A. | | ☐ |
| `CurrentMaxMa` | 200000 | mA | Over-current trip = pack/fuse continuous rating (− margin). | | ☐ |

## 3. Precharge & bus-collapse  🔒  (`COMMISSIONING.md` §3, §3.4)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `PrechargeRatio`*¹ | 0.95 | fraction | DC bus must reach this × pack before AIR+ closes. 0.95–0.98 typical. | | ☐ |
| `PrechargeMaxMs` | 5000 | ms | **Above** the measured worst-case healthy precharge time, **below** the precharge resistor's transient/pulse-energy rating. | | ☐ |
| `BusCollapsePercent` | 50 | % of pack | **Above** worst-case loaded sag of `dc_bus_V` vs cell-sum (so hard accel/regen never false-trips), **high enough** to fire before a no-precharge reclose is damaging. Measure both, set between. **→ moves HIL C-049.** | | ☐ |
| `BusCollapseConfirmTicks` | 20 (~200 ms) | 10-ms ticks | Long enough to reject one anomalous `0x100`; short enough to trip before a released cockpit shutdown recloses the AIRs. | | ☐ |

*¹ `PrechargeRatio` isn't formally `COMMISSION`-tagged but is bench-confirmed in §3 — verify it for this pack.*

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
| `BalanceMaxActive` | 4 | cells/module | Max simultaneous dischargers per module — set against the resistor power budget + airflow. | | ☐ |

## 6. CAN map confirmation  (`COMMISSIONING.md` §7 / `CAN_MAP.md`)

| Constant | Default | How to determine | Final | ✓ |
|---|---|---|---|---|
| `ChargeModeReqId` | `0x101` | Confirm the operator charge-request ID is free in the ECU/VCU CAN map (no collision). | | ☐ |

---

## Sign-off

| Field | Value |
|---|---|
| Firmware version / SHA | `1.6.0` / `__________` |
| Pack / accumulator s/n | `__________` |
| Commissioned by | `__________` |
| Date | `__________` |
| All boxes ticked, host tests green, affected HIL rows re-run | ☐ |

After sign-off: tag **v1.6.0** and open the `dev → main` release PR (this
also unblocks the host pit-diag release, #321).
