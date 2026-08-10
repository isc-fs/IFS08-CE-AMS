# Commissioning checklist

One-page sign-off sheet for every `COMMISSION`-tagged constant in
[`Core/Inc/app/ams_config.hpp`](../Core/Inc/app/ams_config.hpp). These are
**placeholder or single-carrier defaults** — each must be confirmed or measured
on the real pack / inverter / BMS_LITE hardware **before a flight build is
tagged**. Procedures are in [`COMMISSIONING.md`](COMMISSIONING.md); this sheet
is the checklist + record.

> **Workflow:** measure → set the value in `ams_config.hpp` → rebuild →
> re-run the host tests + the affected HIL bench rows → tick the box and record
> the final value, who, and the date.
>
> **🔒 = safety-gating** — feeds a predicate that opens the AIRs / latches
> Error, or a contactor sequence. **📡 = telemetry only**, wrong values cost
> accuracy and never safety.

**Build / verify after edits:**

```bash
cmake --build build
cmake -B build-tests -S tests/unit && cmake --build build-tests
./build-tests/ams_unit_tests    # expect "476 Tests 0 Failures 0 Ignored"
```

`ctest` reports `1/1 Test ... Passed` — that is the single Unity *runner*, not
the case count. Run the binary directly for the real total.

> **To generate this list yourself** (and catch anything this sheet has
> drifted from): `grep -n COMMISSION Core/Inc/app/ams_config.hpp`. The header
> is the authority; if it disagrees with this page, the header wins.

---

## 1. Cell voltage & temperature limits  🔒  (`COMMISSIONING.md` §1)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `CellUnderVoltageMv` | 2800 | mV | Cell datasheet min discharge cutoff (+ margin for sag). | | ☐ |
| `CellOverVoltageMv` | 4200 | mV | Cell datasheet max charge voltage (− margin). | | ☐ |
| `CellOverTempC` | 60 | °C | Cell datasheet max temp (− margin); cross-check with `BalanceTempMax`. | | ☐ |
| `CellUnderTempC` | −10 | °C | Cell datasheet min operating temp. Not a measured bench item for this pack. | | ☐ |

> **⚠ The temperature limits are NOT ARMED.** `config::TempFaultsTrusted` is
> `false`, so the `CellUnderTemp` / `CellOverTemp` predicates are suppressed
> — setting these changes nothing on a real car until the ADG731 mux path is
> validated end-to-end. Commission the values anyway, but do not report
> "over-temp protection commissioned". Cell **voltage** protection is
> unaffected, and the temp **disconnect** check (§4) is armed independently.

> Debounce: a breach must persist `CellFaultConfirmTicks` (25 ≈ 250 ms) before
> latching. With `BmsPollVoltMs` = 200 ms the worst case is ≈ 460 ms, inside
> the < 500 ms budget. Confirm that window suits your cell — not a value to
> tune blind.

## 2. Pack current sensor  🔒  (`COMMISSIONING.md` §2)

Bourns SSA-2-250A shunt read **differentially** on `PF7`/`PF8`
(ADC3_INP3/INN3). **Calibrate the two cal constants before trusting the
over-current trip.**

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `CurrentZeroCount` | 2054 | ADC counts | Zero-current differential code (pack open / 0 A). **Board-specific — the offset tracks VREF+, so re-measure on every carrier.** Ideal is 2048; the flight carrier read 2054, an earlier bench carrier 2050. | | ☐ |
| `CurrentMvPerAmpe1` | 46 | 0.1·mV/A | Sensitivity ×10. Ideal 50 (5 mV/A), trimmed for a bench-measured 0.924× ADC/VREF gain. Re-measure per carrier (it read back exact on the second carrier, but it is a VREF+ term too). | | ☐ |
| `CurrentMaxMa` | 185000 | mA | Over-current trip. Today's value is the **cell** rating (95S6P VTC6 at 30 A each → 6 × 30 = 180 A), derived from the datasheet and **never measured against the car's real draw**. Confirm inverter peak + contactor and fuse ratings. | | ☐ |
| `CurrentLegPlausMinMv` | 700 | mV | Disconnect window floor on OUT_P read single-ended. Measure the connected swing + pull-down droop on the carrier. | | ☐ |
| `CurrentLegPlausMaxMv` | 2300 | mV | Disconnect window ceiling (above → open / stuck-rail). | | ☐ |

> **No debounce on the over-current trip** — smoothing is entirely the IIR
> (`CurrentFilterShift` = 4 at 50 ms → τ ≈ 800 ms). Trip time is
> `τ·ln(I/(I−limit))`: 200 A → 2.1 s, 250 A → 1.1 s, 400 A → 0.5 s.
> **Corollary:** currents between the cell rating and this limit never trip at
> all, by design. The intended protection against a slow overload is the cell
> temperature path — which is currently disarmed (§1). Record that gap.

> **Bench-confirm the disconnect actually latches** (`CurrentSensorFault`,
> reason 8) by physically unplugging the sensor. The detection relies on a
> weak internal pull-down on PF7/PF8 in analog mode, which is board/VREF
> specific. `CurrentDisconnectConfirm` = 3 reads ≈ 150 ms.

### 2b. DCDC current  📡 (`COMMISSIONING.md` §2.4)

Allegro **ACS758** Hall sensor, single-ended on `PC1` (ADC3_INP11),
ratiometric @ 3.3 V → 26.4 mV/A at a 1.65 V (Vcc/2) offset. Informational
only — `DcdcIStaleMs` has no FSM impact.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `DcdcCurrentZeroMv` | 1650 | mV | Measured 0 A output = Vcc/2. | | ☐ |
| `DcdcCurrentMvPerAmpe1` | 264 | 0.1·mV/A | Sensitivity ×10, ratiometric at the real rail voltage. | | ☐ |
| — sign | — | — | **Confirm on the bench**: the IP+→IP− conductor direction sets whether discharge reads positive. | | ☐ |

## 3. Precharge, bus-collapse & re-arm  🔒  (`COMMISSIONING.md` §3)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `PrechargeRatio`*¹ | 0.95 | fraction | DC bus must reach this × pack before AIR+ closes. 0.95–0.98 typical. | | ☐ |
| `PrechargeMaxMs` | 5000 | ms | **Car:** **above** the measured worst-case healthy resistor-precharge time, **below** the resistor's transient/pulse-energy rating. **Charger:** the resistor is skipped, so it is the failsafe ceiling on waiting for a fresh `0x101`. Set for the tighter of the two. | | ☐ |
| `BusCollapsePercent` | 50 | % of pack | **Above** worst-case loaded sag of `dc_bus_V` vs cell-sum (so hard accel/regen never false-trips), **high enough** to fire before a no-precharge reclose is damaging. Measure both, set between. | | ☐ |
| `BusCollapseConfirmTicks` | 20 (~200 ms) | 10-ms ticks | Long enough to reject one anomalous `0x100`; short enough to trip before a released cockpit shutdown recloses the AIRs. | | ☐ |
| `DcBusDischargedV` | 60 | V | Touch-safe DC limit from the current rulebook. **Must sit at or above the ECU's own discharge-release threshold** — confirm it directly with the ECU owner, or the two disagree and the AMS waits forever. Absolute volts, not a fraction of pack. | | ☐ |

*¹ `PrechargeRatio` isn't formally `COMMISSION`-tagged but is bench-confirmed
in §3 — verify it for this pack.*

> **Charger mode skips the resistor.** On Start→Precharge the FSM emits only
> `CloseAirN` (Car emits `CloseAirN | ClosePrecharge`), so the resistor never
> enters the charge loop — the charger voltage-matches its own output.
> Charger's precharge-complete criterion is `0x101` freshness, not the
> `dc_bus_V` ramp. `PrechargeRatio` and the `BusCollapse*` sag margins are
> **Car-mode concerns only**.

> **⚠ Whether `DcBusDischargedV` does anything depends on the ECU image.**
> ECU `dev` sends `0x100` at DLC 3, so `ecu_discharge_capable` latches and the
> block is live; ECU `main` still sends DLC 2, where it is inert. Commission
> the number either way — it must sit at or above the ECU's own release
> threshold — and treat a stranded DC link as a manual hazard until the
> pairing is bench-proven, which it is not. Re-verify against the ECU repo
> rather than trusting this note.

| Bench test | How | ✓ |
|---|---|---|
| Precharge freshness gate (`COMMISSIONING.md` §3.0) | Stop the `0x100` publisher mid-Precharge with the bus above target. The FSM must stay in Precharge and time out to Error — never advance to Transition. A frozen `dc_bus_V` satisfies the 95 % criterion forever. | ☐ |

## 4. NTC thermistor path  🔒 (`COMMISSIONING.md` §3b)

```
R_ntc = NtcPullupOhm · V_aux / (NtcVrefMv − V_aux)   →   table lookup
```

Conversion is a **table lookup** (`Core/Inc/app/ntc_table.hpp`, generated from
`docs/ntc_rt_table.csv`) for the fitted **Fenghua CMFB103F3950FANT**
(R25 = 10 kΩ, B25/50 = 3950 K). **There are no `NtcBeta` / `NtcR25`
constants** — do not try to reintroduce a single-beta fit, and do not "tune"
a constant to make one reading match. Two wrong constants once cancelled here
and left the whole path reading ~7 °C cold at 50 °C.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `NtcPullupOhm` | 6800 | Ω | **Measure** the R145 / R170 divider pull-up. | | ☐ |
| `NtcVrefMv` | 3000 | mV | **Measure** the buffered LTC6811 VREF2. | | ☐ |
| `NtcOpenMv` | 2800 | mV | At or above ⇒ **open**, not cold. Must stay below `NtcVrefMv`. Set so a partially-railed open (~2.7–2.95 V) is caught while a genuinely cold NTC is not. | | ☐ |
| `TempDisconnectPolls` | 1 | polls | Consecutive open polls before latching Error. 1 gives ≈ 360 ms detection; there is **no** single-glitch debounce left. | | ☐ |
| `RequiredTempSlots` | all 40 | slots | Slots required present on every online module. **Consequence: any open channel on the flight harness latches Error at boot until repaired.** Sweep the harness for continuity before expecting the pack to arm. | | ☐ |

> Validate by soaking at a known reference temperature and confirming the
> decoded °C across all 200 channels (pit-diag `0x6A0..0x6B8`, armed via
> `0x7F0`). If the median is off, the fault is in the **divider or the part**,
> not in a fitting constant.

> **Watch for spurious `TempSensorDisconnected` trips on the bench.** If a
> genuine glitch source appears, add headroom to `BmsPollTempMs` and reinstate
> a 2-poll debounce that still fits < 500 ms — do **not** widen `NtcOpenMv`,
> which re-opens the "a disconnect looks merely cold" hole.

## 5. BMS chain freshness  🔒  (`COMMISSIONING.md` §4)

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `BmsStaleMs` | 350 | ms | Sized so a silent module opens the SDC in < 500 ms (worst case ≈ 410 ms) while tolerating exactly **one** missed 200 ms poll. **Confirm no nuisance trips and measure the real margin on the HIL bench.** Safe only with bounded poll jitter. | | ☐ |
| `BmsStaleConfirmTicks` | 25 (~250 ms) | 10-ms ticks | Debounce on the secondary per-module freshness predicate, so an EMI flicker does not open the contactors. Does **not** add to the module-loss budget (`BmsModuleOffline` pre-empts it). 0 = first-tick latching. | | ☐ |

## 6. Cell balancing  (`COMMISSIONING.md` §3c)

Not AIR-gating, but bounded by BMS_LITE balance-resistor dissipation and the
airflow available in a sealed accumulator box.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `BalanceDeltaMv` | 50 | mV | START threshold: imbalance above the pack floor that starts a cell bleeding. | | ☐ |
| `BalanceTempMax` | 50 | °C | Abort balancing above this `max_tempC` (keep ≤ `CellOverTempC`). | | ☐ |
| `BalanceMaxActive` | 8 | cells/module | Max simultaneous dischargers per module. 47R‖47R = 23.5 Ω per cell → 0.75 W/cell @ 4.2 V, so 8 = **6.0 W per module / 30 W per pack**. The 2512 resistors (2 W) run ~19 % — comfortable. **The real limit is heat out of the sealed box, which the part rating does not change. Measure board temperature at this setting in the real box** — the ~71 °C reference figure is a single pad on an open bench. | | ☐ |
| `BalanceMinValidTempCh` | 5 | channels | Minimum converted temp channels before balancing may run. Deliberately LOW — it catches a totally dead temp path, it does not guarantee coverage. **Raise to the measured populated count** once a bench sweep establishes it (read `BmsState::valid_temp_channels`). | | ☐ |
| `BalanceTempsTrusted` | true | bool | See the residual-risk note below before shipping this on a car. | | ☐ |

> **⚠ RESIDUAL RISK — `BalanceTempsTrusted` is `true` while
> `TempFaultsTrusted` is `false`.** Balancing dissipates into the cells while
> its *only* thermal guard (`BalanceTempMax`) reads a path we have not
> validated. Mitigations: the 5 s operator dead-man (`BalanceOverrideFreshMs`),
> the per-module active cap, and the `BalanceDeltaMv` selection floor.
> **Balance with cell temperatures observed by some other means** until the
> ADG731 path is validated end-to-end.

> Related non-`COMMISSION` constants worth understanding before you touch the
> ones above: `BalanceStopDeltaMv` (20 mV hysteresis STOP threshold; a
> `static_assert` keeps it below the start threshold), `BalanceSpreadNoAdjacent`
> (never bleed two physically adjacent cells), `BalanceUpdatePolls` (4 polls =
> 800 ms at `BmsPollVoltMs` = 200), and `BalanceQuiesceMs` (2 ms DCC-clear
> settle before measuring — see §3c.4, this is load-bearing).

| Bench test | How | ✓ |
|---|---|---|
| Balance quiesce health | `g_balance_quiesce_fail_count` on pit-diag `0x6CB` must stay at zero across a long balancing session. A failing quiesce means cell voltages are sampled under bleed — and the balance selector ranks exactly those numbers. | ☐ |
| Per-board dissipation | Clamp meter on the supply rail to one BMS_LITE during a known-imbalanced soak, at the final `BalanceMaxActive`. | ☐ |

## 7. State of charge  📡  (`soc_estimator.hpp`)

**Telemetry only — no safety predicate reads any of this.** A wrong value costs
accuracy, never safety. Do not spend bench time here at the expense of §1–§5.

| Constant | Default | Unit | How to determine | Final | ✓ |
|---|---|---|---|---|---|
| `PackCapacityMah` | 18000 | mAh | Usable capacity of **one series element** (6P × 3.0 Ah). Nominal datasheet figure, **not measured on this pack**. A 10 % error here is a 10 % proportional error in every Coulomb-counted SoC. Replace with a measured full-discharge figure once one exists. | | ☐ |
| `SocRestSettleMs` | 300000 | ms | Rest time before an OCV anchor is taken. Not characterised on this cell — if anchors land consistently below a known-good reference, this is the first number to raise. | | ☐ |
| `SocEkfInitVar` | 0.04 | — | Derived from first principles (σ = 20 % SoC), never fitted. | | ☐ |
| `SocEkfProcessVarPerS` | 1.0e-8 | /s | Derived, never fitted. | | ☐ |
| `SocEkfMeasVarBase` | 1.0e-4 | — | Derived (σ = 10 mV OCV curve-fit error), never fitted. | | ☐ |
| `SocEkfMeasVarPerA2` | 4.5e-7 | /A² | Derived from R_int uncertainty, never fitted. | | ☐ |

> For the EKF variances, "leave them" is usually the right answer — record that
> decision rather than tuning blind. They are tagged `COMMISSION` because they
> were reasoned, not measured, and the tag should not imply they are wrong.

## 8. CAN map confirmation  (`CAN_MAP.md`)

Both are operator→AMS request IDs that must not collide with anything in the
ECU / VCU map.

| Constant | Default | How to determine | Final | ✓ |
|---|---|---|---|---|
| `ChargeModeReqId` | `0x101` | Confirm the charge-request ID is free in the ECU/VCU CAN map. Magic-gated (`"CHRG"`) + freshness-checked. | | ☐ |
| `BalanceOverrideReqId` | `0x103` | Confirm the balance master-switch ID is free. Magic-gated (`"BALO"`/`"BALN"`/`"BALX"`), 5 s dead-man to OFF. | | ☐ |

---

## Pre-flight build check

| Check | ✓ |
|---|---|
| Flight image built **without** `AMS_HIL_CLEAR_ERROR_LATCH` (see [`HIL_BUILD.md`](HIL_BUILD.md)) — otherwise the sticky `ErrorLatch` contract is defeated | ☐ |
| `python3 scripts/check_flash_layout.py build/AMS.elf` → `PASS` | ☐ |
| DBC regenerated and matching (`tools/dbc_dump.cpp` → `docs/dbc/ams.dbc`) | ☐ |
| `0x6C6` reports the expected git hash on the flashed unit | ☐ |
| Bootloader node ID matches `config::AmsNodeId` (`0x02`) | ☐ |

## Sign-off

| Field | Value |
|---|---|
| Firmware version (`VERSION`) / commit SHA | `__________` / `__________` |
| Pack / accumulator s/n | `__________` |
| Commissioned by | `__________` |
| Date | `__________` |
| All boxes ticked, host tests green, affected HIL rows re-run | ☐ |
| Known gaps recorded in the project log (temp faults disarmed; ECU discharge interlock present but the pairing is unproven on a bench; over-current is datasheet-derived) | ☐ |

After sign-off: tag the release and open the `dev → main` release PR.
