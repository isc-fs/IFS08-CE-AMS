# Balancing strategy — brief for the firmware lead

*IFS08-CE-AMS @ `fc8bc15` (v2.1.0) · BMS_LITE v3 (`production/bom.csv`, 2026-06-30)*

> **A point-in-time analysis, not a living spec.** This is kept because the
> reasoning is worth more than the conclusion: it shows how the balancing
> numbers were arrived at, and which of them rest on a measurement nobody has
> taken. The dates and the version stamp are deliberate — this page's job is
> history, which is the documented exception to the no-dates rule.
>
> Its recommendations were acted on. The thermal-guard and ADOW-throughput
> findings now live in [`../BMS_LTC6811.md`](../BMS_LTC6811.md) and
> [`../FMEA.md`](../FMEA.md); trust those for current behaviour and read this
> for *why*.

---

## 1. THE ANSWER

**Keep the strategy exactly as shipped: greedy top-8-by-excess per module, floored on the 2nd-lowest cell, 50 mV deadband, non-adjacent spreading, quiesced before every measurement.** It is the right policy for this hardware and I found no tuning change worth the regression risk. Make one behavioural fix — a latched *cell-data* fault (open-wire / OV / UV) should stop balancing even under the operator `On` override, which today it does not — plus a set of comment/doc corrections where the repo's own numbers are wrong by up to 20×. Everything else that looks like a lever (`BalanceDeltaMv`, `BalanceMaxActive`, hysteresis, the adjacency rule) is either already at the right value or cannot be moved without a measurement nobody has taken.

The real finding is not in the algorithm. It is that **the only thermal guard on balancing is physically incapable of seeing the thing that gets hot**, and that **the ADOW scan shipped today cost ~14% of balancing throughput** — neither of which is written down anywhere.

---

## 2. WHAT THE HARDWARE PERMITS

**23.5 Ω is confirmed fitted for the v3 run.** `R81` = `R82` = 47 Ω, 2512, verified in all 24 cell sheets (`cell1.kicad_sch:1533`, `:2209`) and in `production/bom.csv:20` (48 × 2512 @ 47), which carries the same 2026-06-30 timestamp as the shipped `production/BMS_PCB_v3.zip`. 47 ‖ 47 = 23.5 Ω.

### Per cell

| Cell V | I = V/23.5 | P per cell | P per 2512 | % of 2 W |
|---|---|---|---|---|
| 4.20 V (charge limit) | 178.7 mA | 0.751 W | 0.375 W | 18.8% |
| 4.00 V | 170.2 mA | 0.681 W | 0.341 W | 17.0% |
| 3.85 V (nominal) | 163.8 mA | 0.631 W | 0.315 W | 15.8% |

Arithmetic: 4.2² / 23.5 = 17.64 / 23.5 = 0.7506 W; per resistor (I/2)²·47 = (0.08935)²·47 = 0.3753 W — exactly half, no parallel factor-of-2 error in `ams_config.hpp:606-616`.

**The FET is not in the picture.** At the actual gate drive (below) Rds(on) interpolates to ~48 mΩ from the datasheet maxima — 0.2% of the 23.5 Ω loop, dissipating 1.5 mW against a 670 mW rating at 70 °C.

**The gate drive is not marginal either, and the brief's premise was wrong.** `R169` is **10k**, not the 100k stated in the brief and in `analysis/open_tap_currentpath.html` (`cell1.kicad_sch:1614`; `bom.csv:25` lists 24 × 10k 0603). With `R84` = 3.3k, Vgs = −Vcell × 10/13.3 = **−0.752 × Vcell = −3.16 V at 4.2 V**, against a ±8 V maximum. Even with the sense domain driven to its clamp, |Vgs| ≤ 6.02 V — 25% margin. *Do not "correct" R169 upward*: at 100k the ratio becomes 0.968 and the same fault yields 7.74 V, a 3% margin.

### Per module / per pack

- **Peak, at `BalanceMaxActive` = 8, 4.2 V:** 8 × 0.751 = **6.0 W per module**, **30.0 W across 5 modules**.
- **Time-averaged** (see duty below): 8 × 0.751 × 0.80 = **4.8 W per module**, **24 W pack**. The 6.0 W figure the config quotes is a peak; a sealed box responds to the average.

### Duty cycle — the number nobody has written down

A selected cell discharges only **~80–83% of wall-clock time**, not the ~99% the docs imply. `quiesce_balancing()` runs at `bms_poll_task.cpp:404` and `restore_balancing()` only at `:433`, so the OFF window is the whole quiesce → measure → **ADOW** → restore span, on **every 200 ms poll**.

At 515.625 kHz SCK (`main.c:533`, prescaler 256 on 132 MHz pll1_q_ck) = 15.5 µs/byte; a chain transaction is 4 + 8×10 = 84 bytes = 1.303 ms:

| Phase | Cost |
|---|---|
| quiesce WRCFGA + `osDelay(2)` | 3.30 ms |
| ADCV + settle(3) + RDCFGA warm-up + 4× RDCV | 9.58 ms |
| **two ADOW passes** (`adow_pass` ×2, each 2 conversions + warm-up + 4 RDCV) | **25.28 ms** |
| restore WRCFGA | 1.30 ms |
| **OFF total** | **39.47 ms of 200 ms → 80.3% duty** |

Taking `osDelay(n)` as n−1 ticks gives 33.5 ms → 83.3%. **Band: 80–83%.**

Without ADOW the OFF window is 14.18 ms → **92.9%**. So the v2.1.0 open-wire scan **cut effective bleed from ~166 mA to ~145 mA — a 13.6% relative loss of balancing capacity**. That is the correct price of the feature and it is worth paying; it just needs to be on the record.

### What the real limiting factor is

Not the resistors (18.8% of rating). Not the FET (0.2% of the loop). Not the gate drive (25% margin). **It is heat out of the sealed accumulator box, and it has never been measured.** The constraint chain is:

> resistor value (hard, 23.5 Ω) → duty cycle (firmware, ~80%) → `BalanceMaxActive` + spreading (thermal policy) → **board-to-air out of the sealed box (unmeasured)**

The board is 208.00 × 44.37 mm (`Edge.Cuts` extents), 4-layer, all 35 µm — but `In2.Cu` has **zero** copper zones and `In1.Cu` is split into two zones tens of volts apart, so effective uniform spreading thickness is ~34.7 µm, i.e. **one-ounce equivalent, not four**. Free-air natural convection plus radiation lands around h ≈ 14 W/m²K → ~22 K whole-board rise at 6.0 W, with an unknown but comparable local pad delta on top. Inside a sealed box with warm walls, convection is suppressed and that estimate is optimistic. The single measured datum — ~71 °C at a 2512 pad (`ams_config.hpp:633`, commit `4ef0d45`) — **records no ambient, no enclosure state, and was taken with 8 pads *concentrated*, i.e. with the spreading mitigation deliberately defeated.** It cannot be scaled to the accumulator.

### Throughput — honest, and gated on a number that is in neither repo

**Cell capacity in Ah does not exist anywhere in either tree.** The firmware's "C/101 balancer" phrasing (`ams_config.hpp:634`) back-solves to ~18 Ah at 179 mA, but that is inference from a comment, not a datasheet. Charge removed per selected cell at ~145 mA effective is **145 mAh/h**:

| Assumed capacity | %SoC/h | Time to close one 50 mV deadband at top-of-charge* |
|---|---|---|
| 18 Ah | 0.81 %/h | ~5.2 h |
| 12 Ah | 1.21 %/h | ~3.5 h |
| 6 Ah | 2.42 %/h | ~1.7 h |

\* assuming ~12 mV/%SoC NMC top-of-charge slope — no cell datasheet exists in `BMS_LITE/Datasheets/`, so this is a textbook figure, and on the mid-pack plateau (~5 mV/%SoC) every number roughly triples. A 500 mV spread, if it is real, is a **multi-day** job at any of these capacities. Recovering the ADOW duty cost would improve these by ~15% (52 h → 45 h at 18 Ah / 500 mV) — real, but not enough to justify weakening an open-wire safety scan.

---

## 3. WHAT TO CHANGE

Only changes I can justify quantitatively. All of them are small.

| Constant / site | Current | Recommended | Why | Risk |
|---|---|---|---|---|
| `compute_mask` fault gate (`balance_controller.hpp:84-93`) | `op_cmd == On` balances in **any** FSM state, including `Error` | Refuse to balance while a latched **cell-data** fault (`CellOpenWire`, `CellOverVoltage`, `CellUnderVoltage`) is present — for `On` as well as `Auto`. Leave non-cell faults (IMD, contactor) overridable. | This is the only surviving path where the selector can bleed a healthy cell indefinitely. `compute_mask` reads **raw** `s.cell_mV[m][c]` (`:176`); the tap-artifact guard writes its corrected values into a **local** `agg_v[]` (`bms_service.cpp:153-186`) that never reaches the selector. A split-tap open (e.g. 4600/3000) is masked for OV but still presents 4600 mV to the selector, which will pick it top-of-list forever while the adjacency rule starves its physical neighbour. ADOW now latches this in <500 ms in every state — but `On` walks straight past the latch. The header already states the intended principle at `:88-89` ("the operator overrides the ENABLE decision, never the guards"); this just finishes implementing it. | Removes a pit escape hatch: post-fault manual rebalancing now requires clearing the fault first. Contactors are already open in `Error`, so nothing time-critical is lost. Scope the refusal to cell-data faults so the rest of the workflow survives. |
| `ams_config.hpp:698-699` and `docs/BMS_LTC6811.md:437` | "at a cost of under 1 % of balancing duty" | "~20 % of balancing duty (39.5 ms of every 200 ms poll, of which 25.3 ms is the ADOW scan)" | **Wrong by 20×.** Both count only `BalanceQuiesceMs` (2/200 = 1%) and ignore the 37.5 ms of WRCFGA + voltage read + ADOW + restore that they hold discharge off for. Anyone sizing a balancing session off these docs under-estimates wall-clock by ~25%. | None — comment only. |
| `docs/BMS_LTC6811.md` DCC → physical-cell map | 10 / 9 split | 9 / 10, matching `CellsPerLtcUpper = 9` / `CellsPerLtcLower = 10` (`ams_config.hpp:844-845`) | The doc is **inverted relative to the shipping code**. The code is right and IR-bench-verified on the real pack (`balance_controller.hpp:54-58`, 2026-07-22). Anyone re-deriving `physically_adjacent` from the doc moves the seam and lets two genuinely adjacent 2512s fire together. | None — doc only. Real commissioning hazard removed. |
| `ams_config.hpp:625-627` (the `BalanceMaxActive` COMMISSION note) | "the `BalanceTempMax` lockout that would catch an overheating board reads the same unvalidated NTC path" | "the `BalanceTempMax` lockout **cannot** catch an overheating board — there is no board thermistor. All 20 NTCs per LTC are off-board **cell** sensors through the ADG731 mux, so `max_tempC` never observes the 2512 band." | `production/bom.csv` has 34 lines and **zero** NTC/thermistor entries (only `ADG731BSUZ` at line 34). The current wording implies a guard that may misread; the truth is there is no guard at all. The board can pass 100 °C with the lockout reading a happy 45 °C cell. | None — comment only, but it is the single most misleading sentence in the balancing config. |
| `ams_config.hpp:676` `BalanceUpdatePolls` comment | "= 1 Hz at `BmsPollVoltMs` = 250 ms" | "= 1.25 Hz at `BmsPollVoltMs` = 200 ms (800 ms window)" | `BmsPollVoltMs` is 200 (`:217`). The stale 1 Hz figure is repeated in ≥4 places, including the argument for why hysteresis is unnecessary. 25% understated. | None. |
| `ams_config.hpp:606-616` dissipation table | 179 mA / 0.75 W presented as continuous | Add a duty row: **~145 mA / 0.61 W time-averaged at 80% duty**; keep 179 mA / 0.75 W labelled as peak | Throughput planning off the peak figure is ~20% optimistic; box-heat planning off the peak is ~20% pessimistic (4.8 W/module, 24 W pack averaged). Both matter and they pull in opposite directions. | None. |

---

## 4. WHAT TO LEAVE ALONE

**`BalanceDeltaMv` = 50 mV.** Do not lower it to recover the ~4 %SoC deadband loss. The firmware's own measured harness IR error is 9–36 mV (`ams_config.hpp:688-695`, 179 mA across 50–200 mΩ of tap/connector/fuse impedance, **opposite sign** on the bled cell and its neighbours). A 25 mV deadband sits inside the measurement error and the selector starts chasing its own artifact. This is a quantified reason to leave a knob alone.

**`BalanceMaxActive` = 8.** Cannot usefully be raised: with non-adjacency on halves of 9 and 10, the maximum independent set is 5 + 5 = 10, so 8 is already near the ceiling, and the two spare slots buy 25% throughput against an unmeasured box thermal limit. Cannot be justifiably lowered either — the resistors are at 18.8% of rating and the constraint that would justify lowering it has not been measured. **8 is the right value to hold until someone puts an IR camera on the box.**

**`BalanceSpreadNoAdjacent` = true.** The geometry argues *for* it, contrary to a superficial reading. Centre-to-centre, the forbidden adjacent-cell resistor sits at **5.956 mm** while the unavoidable same-cell partner sits at **9.20–9.40 mm** — the one the policy blocks is 1.55× *closer*. In a contiguous run the policy removes 4 of the 5 nearest heat sources (2 at 5.956 mm, 2 at 11.13 mm), leaving only the 9.4 mm partner; a 2-D log-spreading estimate puts the saving at **5–10 K of pad temperature** against a `BalanceTempMax` band of 50 °C. It also self-limits: when imbalance clusters, the greedy breaks early (`balance_controller.hpp:191`) and delivers *less* than 6.0 W — as few as 7 cells with all 19 eligible, and 2 cells for a 4-cell cluster. That is the intended safe outcome.

**No hysteresis on the selection threshold.** A cell within ±4 mV of the threshold toggles at 1.25 Hz, but by definition it needs almost no further bleed, so throughput loss is negligible. Adding hysteresis means making `compute_mask` stateful — it is currently a pure function with direct host unit tests, and that is worth more than the fix.

**No target-based / proportional discharge.** Total charge removed per module per hour is fixed at `BalanceMaxActive` × 145 mA regardless of how it is allocated. Greedy top-K-by-excess is already the optimal allocation for the objective. There is nothing to win.

**The 2nd-lowest-cell floor.** This is load-bearing and correct: a single disconnected tap reads spuriously low, and if the floor were the true minimum, every real cell would sit >50 mV above it and **the whole stack would discharge off one faulty reading**. Cost is one deadband on a genuinely weak cell. Right trade, keep it.

**The quiesce sequence, including ADOW inside it.** ADOW's pull-up/down delta is corrupted by bleed current exactly as the voltage measurement is; it belongs inside the window. The 13.6% throughput cost is the correct price for <500 ms open-wire detection on a 350 V sealed pack. Document it; do not optimise it before it is validated on the HIL.

---

## 5. HARDWARE FLAGS

**The cells 2 / 5 / 9 gerber corruption did not happen. I checked the shipped file directly.**

`analysis/cell_corruption_report.html` claims `production/BMS_PCB_v3.zip` shipped with a ⌀3.2 mm mask opening (no dam) on Q10/Q6/D23/D15 and 0.25 mm sliver pads on Q13. I extracted the zip and parsed `BMS_LITE-F_Mask.gts`:

- The mask apertures flashed at **Q13 (cell 2), Q10 (cell 5), Q6 (cell 9) are byte-identical to Q14 (cell 1)** — three `RoundRect` 1.325 × 0.60 mm openings each, same aperture definition, same count.
- The **only** ⌀3.2 mm circular aperture in the entire layer is `%ADD23C,3.200000`, flashed **exactly 4 times**, at (51.00, −79.51), (51.00, −116.12), (251.03, −79.51), (251.03, −116.12) — the four board-corner **M3 mounting holes**.

That aperture is numbered **D23**, which is precisely the string the report attributes to *designator* D23. The report is an aperture-index/designator confusion in its own rendering script, not a fab defect. `analysis/ghost_balancing_chain.html` independently reached the same conclusion ("real JLC gerbers: all 24 pads identical").

**Firmware implication: none. Do not build a per-cell workaround for indices 1 / 4 / 8.** If someone has proposed masking those cells from balancing, kill it.

**Ghost balancing is real and firmware cannot mitigate it.** The chain in `ghost_balancing_chain.html` starts at "no isoSPI master connected" — i.e. the AMS is *unpowered* while modules are plugged in. The LTC6811 sleeps after ~2 s, VREG collapses, the S-pin hold-off pull-ups lose power, and FETs can turn on. There is no firmware in the loop. This is a **storage and handling procedure**, not a backlog item. The one open measurement is the S-pin voltage in sleep (U1 pin 23 for cell 2) against a healthy cell. Note the old LTC6802 was immune because it has no deep sleep — this is a monitor-change consequence, not a board defect. The one thing firmware does right here is already right: after `recover_chain` the cached mask is **dropped** rather than restored, because a slept chain has reset CFGR.

**The tap fuses are not on BMS_LITE at all**, and the 0.5 A BSMD rating quoted in the ghost analysis appears nowhere in the schematics (`PCB_VOLTAGE.kicad_sch` carries a generic `Fuse` value). At 23.5 Ω the balance current is 179 mA against that claimed 0.5 A hold — a 2.8× margin, and the fuse-blowing narrative loses its mechanism entirely. **That is itself evidence the boards which blew fuses had the older ~10 Ω effective resistors fitted** (420 mA vs a 0.5 A hold is marginal, and derates as the area heats). Which brings us to:

**Confirm the fitted resistor per module, physically.** The v3 run is unambiguously 47 Ω (24 cell sheets + `bom.csv:20` + the `.kicad_pcb`). But the archive shows the effective value has been 10 Ω (Feb–Mar, 24 × 10R), 50 Ω (Mar 11), 10 Ω again (Apr–Jun, 48 × 20R), and 23.5 Ω only from Jun 30. **If any older BMS_IFS08 board is still in service it bleeds at 420 mA — 1.76 W per 2512 (88% of a 2 W part) and ~14 W per module against a firmware budget written for 6 W.** A DMM across R81/R82 on each module settles it in 30 seconds. (For the record: the brief's "10 Ω × 2 → 5 Ω → 770 mA" gloss is itself wrong — the stale docs describe 20 Ω × 2 = 10 Ω *equivalent* at 420 mA. No 5 Ω revision ever existed.)

**The 2 W resistor rating has no paper trail.** `bom.csv:20` gives value and footprint with an **empty** LCSC part field; the R81/R82 symbols carry no MPN (unlike Q14, D32, C34, which all do); and there is no 2512 resistor datasheet in `Datasheets/`. The 2 W rests solely on commit `5f961a6` recording that you confirmed the fitted parts by inspection — which for "what is on the board" is stronger evidence than a BOM line, so **this is a traceability finding, not a thermal-margin finding**. It matters only because a re-order or CM substitution has nothing to catch it. Close it with the purchase order or a reel-label photo, and back-fill the MPN into the symbols and the BOM. Note the thermal conclusion does not move either way: at 23.5 Ω, `BalanceMaxActive` = 8 is set by the 6.0 W/module box-heat budget, which is rating-independent.

**D32 / PDZ7.5B is not a gate clamp.** It sits across the LTC6811 sense inputs C(n)/C(n−1), on the IC side of the 100 Ω R83 — verified by rebuilding the netlist geometrically across all 24 sheets. The "7.5 V Zener against ±8 V Vgs" pairing in the brief is wrong; Vgs is set entirely by the 10k/3.3k divider (§2). The relevant margin is Vz(max) 7.60 V against the LTC6811's 8 V C(n)-to-C(n−1) absolute max: 0.40 V at 25 °C, ~0.15 V at 70 °C ambient once self-heating is included. **The tighter number is that the LTC6811's *specified* input range is C(n−1) to C(n−1)+5 V — 8 V is a damage limit, not an operating limit.** For the strongest plausible single fault (harness off by one tap position, 8.4 V into R83), the clamp holds: Iz ≈ 6–8 mA, C pin at 7.63 V / 7.79 V. And ADOW has *more* margin than normal operation, not less — the LTC injects 70–130 µA, 50× below the Zener's 5 mA test point.

**The firmware-index → board-position starting offset is undocumented.** `balance_controller.hpp:44-58` establishes the map is monotonic and contiguity is IR-verified, but the board has 24 balance channels (LTC_1 = cell1..12, LTC_2 = cell13..24) while firmware uses 9 + 10 = 19, leaving 5 unused. **Nothing states which board cell is firmware index 0.** Anyone mapping a lit resistor back to a cell index in the pit needs that written down.

---

## 6. OPEN QUESTIONS

### Must be MEASURED — reasoning cannot settle these

1. **Peak 2512 pad temperature and whole-board temperature at `BalanceMaxActive` = 8, with `BalanceSpreadNoAdjacent` ON, inside the sealed box, with logged ambient — as a full-field IR frame, not a single pad reading.** The existing 71 °C datum recorded no ambient, no enclosure state, and used the *concentrated* case, which is not the shipped configuration. The board-average-minus-ambient vs pad-minus-board-average split from one IR image resolves the entire question. **This is the only unbounded risk in the whole system.**
2. **`g_bms_volt_poll_ms` / `g_bms_volt_poll_max`, with balancing active and ADOW live.** Already instrumented and published on pit-diag (`pit_timing.def:4-5`, emitted at `acu_can_task.cpp:325-326`). Reading it converts every duty and throughput number in §2 from estimate to measurement for **zero** engineering cost. My prediction: ~40 ms with balancing active, ~36 ms without. If it reads materially higher, the 80% duty figure is optimistic and throughput is worse than stated.
3. **Cell capacity in Ah.** Not in either repo. Every time-to-balance figure scales linearly with it. Get it from the cell datasheet and put it in `ams_config.hpp`.
4. **Cell OCV/SoC slope near 4.0–4.2 V.** No cell datasheet in `BMS_LITE/Datasheets/`. Without it, "how long to close 500 mV" cannot be answered better than ±3×.
5. **Resistance across R81/R82 on one board per module.** Settles whether any pre-v3 (10 Ω effective) board is in service. 30 seconds with a DMM.
6. **ADOW on the HIL, including the endpoint conductors.** `bms_poll_task.cpp:424-425` flags this itself ("validate the task keeps up on the HIL bench"), and the C(0)/C(N) endpoint rule uses exact-zero comparisons that are bench-unvalidated (`ams_config.hpp:51-54`). Shipped today, unvalidated on a real chain.
7. **S-pin voltage on U1 during LTC sleep**, cell 2 vs a healthy cell. Closes the last unproven link in the ghost-balancing chain.
8. **Internal accumulator air temperature during a balancing run.** The 60 °C figure everyone uses is pure assumption, and it is the term the final pad temperature is most sensitive to. Also worth 60 seconds: check whether the board is conduction-coupled to the enclosure through its four M3 mounting holes — a bolted metal standoff path would dominate every thermal number above.

### Can be REASONED — already settled, do not spend bench time

- **Fitted value is 47 Ω for the v3 run** — BOM + 24 sheets + PCB all agree, and the BOM postdates the change.
- **The FET is not a constraint** — 48 mΩ at the actual Vgs, 0.2% of the loop, 1.5 mW of a 670 mW rating.
- **Vgs margin is fine** — −3.16 V of ±8 V, fixed by the 10k/3.3k divider.
- **The 2512s are not the constraint at 47 Ω** — 0.375 W each, 18.8% of rating.
- **The 2/5/9 gerber corruption did not ship** — verified above from the actual `.gts`.
- **`BalanceMaxActive` cannot usefully exceed 10** under the non-adjacency rule.
- **The adjacency policy is worth keeping** — centre-to-centre geometry decides it.
- **`BalanceDeltaMv` cannot go below ~50 mV** — the firmware's own measured IR artifact is 9–36 mV.

---

## 7. THE ONE THING

**Book one 30-minute bench session: run the pack at `BalanceMaxActive` = 8 in the shipped configuration (spread ON, ADOW live), inside the sealed box, and capture two things — a full-field IR frame of the 2512 band with the ambient logged, and `g_bms_volt_poll_ms` / `_max` off pit-diag.**

Not a code change. Every remaining decision is gated on those two readings and nothing else:

- The IR frame plus ambient tells you whether `BalanceMaxActive` = 8 is safe in a sealed accumulator. Right now the only guard that would catch an overheating board **does not exist in hardware**, the one measurement anyone has is from a defeated-mitigation configuration with no ambient recorded, and `ams_config.hpp:624` still says "COMMISSION: still not measured" nine lines above a comment claiming it was measured. This is the only place in the balancing system where being wrong damages a 350 V sealed box.
- `g_bms_volt_poll_ms` costs nothing — the instrumentation is already shipped and already on CAN — and it turns the entire §2 throughput analysis from arithmetic into fact, including whether the ADOW scan that went live today is actually affordable.

If that session says the box is fine, ship the §3 table and change nothing else. If it says the box is not fine, `BalanceMaxActive` is the knob, and you will finally have a number to turn it against.