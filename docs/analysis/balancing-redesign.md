# Balancing redesign — design for review

*IFS08-CE-AMS @ `027f453` (v3.0.2) · BMS_LITE v3 · 2026-10-02*

> **Status: proposal, not implemented.** This is the design to review before
> any code is written. It succeeds
> [`balancing-strategy-brief.md`](balancing-strategy-brief.md) (v2.1.0), whose
> recommendations have since shipped. Like the brief it is a point-in-time
> analysis, so dates and versions appear on purpose: this folder's job is
> history.

---

## 1. Summary

The cell-selection rule is right and stays: largest excess first, second-lowest
floor, 50 / 20 mV hysteresis, no two adjacent cells, at most 8 per module,
quiesced before every measurement. What is wrong is **when** it decides, **what
it watches** for heat, and **how little it tells anyone**:

1. **It ranks cells while current flows.** The balancing path never sees pack
   current, so under charge or load it bleeds whichever groups have the highest
   I·R, not the highest state of charge.
2. **Its only thermal guard cannot see the part that gets hot.** The board has
   no thermistor. The LTC6811 on that board does measure its own die
   temperature, and the firmware does not read it.
3. **It never says why it is not balancing**, never says when it is done, and
   never checks that a bleed path actually works.
4. **It is allowed in states where it should not run.** Decided for this
   redesign: **balancing only in Start and Charge, never in any other state.**

The redesign wraps the existing pure `compute_mask()` in a stateful, still
hardware-free `BalanceController` that adds: a state gate, current-aware
picking, a die-temperature board guard, state/reason telemetry, and (second
step) a bleed-path self-test.

---

## 2. Decisions already taken

| # | Decision | Effect |
|---|---|---|
| D1 | **Balancing runs only in `Start` and `Charge`.** Never in `Precharge`, `Transition`, `Run` or `Error`, whatever `0x103` says. | Closes FMEA **SEASON-3** (forced balancing while driving). Removes balancing in `Error`: the pit-rebalance path after a fault now requires clearing the fault first. |
| D2 | **The WarioCharger only displays data; it does not control the charger.** The charger runs its own profile. | Nothing in the system can taper charge current for balancing. The AMS's only lever on charging is the contactors. See §6. |
| D1a | **`Auto` only in `Charge`; `On` in `Start` or `Charge`.** | No autonomous bleeding in the pit; balancing in `Start` always needs an operator command. |
| D3 | Design reviewed as a document first; implementation follows in staged PRs (§8). | — |

**D1 per command:** `Auto` runs **only in `Charge`**; `On` (operator force)
runs in `Start` or `Charge`. No autonomous balancing while the car sits in
`Start`.

---

## 3. How it works today

Every `BalanceUpdatePolls` × `BmsPollVoltMs` = 4 × 200 ms = **800 ms**,
`BmsPollTask::maybe_run_balance_update()` calls `balance::compute_mask()` and
writes the result with one `WRCFGA` to the chain.

**Gates** (any one → all-zero mask):

| Gate | Source |
|---|---|
| operator command `Off` (incl. 5 s dead-man on `0x103`) | `VehicleService::effective_balance_cmd` |
| `Auto` and FSM ≠ `Charge` (`On` skips this — runs in any state) | `compute_mask` |
| latched cell-data fault (`CellOpenWire`, `CellOverVoltage`, `CellUnderVoltage`) | `balance::is_cell_data_fault` |
| `BalanceTempsTrusted == false` | `ams_config.hpp` (currently `true`) |
| `valid_temp_channels < BalanceMinValidTempCh` (5) | `compute_mask` |
| `max_tempC > BalanceTempMax` (50 °C, cell NTCs, no hysteresis) | `compute_mask` |

**Selection** per module: floor = second-lowest cell in the pack; a cell
qualifies above floor + 50 mV (stays selected down to floor + 20 mV); greedy by
excess, incumbents win ties, never two physically adjacent cells
(`physically_adjacent`, IR-verified), at most `BalanceMaxActive` = 8.
Per-module enable from `0x104` is layered underneath.

**Measurement protection:** every 200 ms voltage poll clears all DCC bits,
waits `BalanceQuiesceMs` = 2 ms, measures (ADCV + open-wire ADOW), then
restores. A failed quiesce skips the next balance update
(`g_balance_quiesce_fail`).

**Hardware fail-safes (verified against the LTC6811 datasheet):**
- With `DCTO = 0000` (as configured, `ltc6811::pack_cfga_payload`), the
  watchdog resets **CFGR0–5 including every DCC bit 2 s after the last valid
  command, regardless of the DTEN pin**. An AMS hang or an isoSPI loss stops
  discharge within 2 s.
- Die thermal shutdown at **150 °C** also resets the configuration.

**Numbers carried over from the brief** (still valid): 23.5 Ω per cell
(47 Ω ‖ 47 Ω, 2512), 179 mA / 0.75 W peak at 4.2 V, ~80 % bleed duty because of
the per-poll ADOW scan (~145 mA effective), 6.0 W per module and 30 W across the
pack at 8 active cells.

---

## 4. Findings

### F1 — Cells are ranked under current (high)

`bms_poll_task.cpp` never reads `CurrentService`. Under current, each group
reads `OCV + I·R`, and `R` differs between groups (cell spread, welds, busbar
segments inside a tap). That difference is ranked as if it were state of charge.

- `Auto` re-picks throughout `Charge`, including the constant-current phase.
- `On` runs in every state today, including `Run` (FMEA SEASON-3, which
  already notes it *"ranks cells by I·R rather than SoC under load, so it bleeds
  the healthiest ones"*).
- Scale, **assuming** ~3–4 mΩ per 6P VTC6 group (no cell datasheet is in the
  repo): a ±20 % resistance spread is ~±7 mV at 10 A — a third of the 20 mV stop
  threshold — and far beyond the 50 mV start threshold at driving currents.

### F2 — The only thermal guard reads cell NTCs, not the board (high)

There is no thermistor on BMS_LITE (brief §5). `BalanceTempMax` reads the cell
NTCs through the ADG731 path whose faults are still disarmed
(`TempFaultsTrusted = false`) while `BalanceTempsTrusted = true` lets balancing
rely on it. FMEA **BALANCE-3** (board temperature in a sealed box never
measured) is still open.

**The fix is already on the board.** Each LTC6811 measures its own die
temperature with `ADSTAT`:

- `ITMP` in Status Register Group A; **T(°C) = ITMP × 100 µV / 7.5 mV − 273**
  (datasheet, *Internal Die Temperature*).
- `ADSTAT` = `0x0468 | MD<<7 | CHST`; `CHST` 0 = all, 1 = SC, 2 = ITMP.
  At 7 kHz all four values convert in ~0.8 ms (datasheet Table 9).
- The die sits on the balancing board, one LTC per resistor row, so it is a
  real (lagging, offset) board-temperature signal per row, available with
  firmware only.

### F3 — Inhibition is silent (medium)

Six gates collapse into the same all-zero mask. "Balanced", "blocked by a
fault", "too hot" and "no thermal data" look identical on pit-diag
(`0x6C2/0x6C3` show the mask; `0x6CB` only the quiesce health).

### F4 — No notion of done, no accounting (medium)

No `Done` state, no record of how long each cell bled or how much charge was
removed, no convergence trend. A cell that is stuck looks the same as one that
is converging.

### F5 — Nobody checks that a bleed path works (medium)

An open FET, resistor or tap fuse leaves a cell selected forever and never
converging; a stuck-on FET drains a cell silently. The physics gives a free
check: a working bleed displaces its own cell's reading by **9–36 mV** (the
artifact the quiesce exists to remove, FMEA BALANCE-1).

### F6 — Thermal lockout has no hysteresis (low)

`max_tempC > 50` toggles at the update rate around the threshold.

---

## 5. Redesign

### 5.1 Structure

```
BmsPollTask ──inputs──▶ BalanceController::step(...)  ──mask──▶ WRCFGA
  BmsState snapshot          │  stateful, HAL-free, host-tested
  pack current (filtered)    │  owns: last clean pick, hold age, per-LTC derate,
  FSM state, fault reason    │        inhibit reasons, accounting, self-test plan
  0x103 / 0x104 (resolved)   ▼
  die temps / SC per LTC   compute_mask(...)   (unchanged, pure: picks cells)
```

`compute_mask()` keeps its signature and tests. Every new rule lives in
`BalanceController`, which is a plain class with no RTOS or HAL dependency, so
it is tested the same way. `BmsPollTask` gains only the `ADSTAT` read and the
call into the controller.

### 5.2 State gate (D1)

Allowed iff FSM ∈ {`Start`, `Charge`} **and** the command permits it there:
`On` → `Start` or `Charge`; `Auto` → `Charge`. Any other state → mask 0, reason
`StateNotAllowed`. This replaces "`On` runs anywhere".

### 5.3 Current-aware picking (F1)

| Rule | Proposed default (COMMISSION) |
|---|---|
| A sample is **clean** when \|I\| ≤ `BalanceRankMaxCurrentMa` continuously for `BalanceRelaxMs` | 2000 mA, 30 s |
| On a clean sample: re-pick with `compute_mask()` and remember the decision | — |
| Not clean: **keep bleeding the last clean decision** for at most `BalanceHoldMaxMs` | 10 min |
| \|I\| > `BalanceLoadMaxCurrentMa` → mask 0 immediately, reason `HighCurrent` | 10 A |
| No clean sample yet / hold expired → mask 0, reason `WaitingCleanSample` | — |

- Uses `CurrentState::filtered_mA` (sign: + = discharge); the absolute value is
  what matters.
- 30 s relaxation is a conservative start; the logged CSV (`I_filt_mA` and cell
  voltages at 4 Hz) will show how fast the groups actually relax, and the
  constant should be tuned from that.
- **Consequence:** during a high-current constant-current phase there are no new
  picks; the hold expires after 10 min and balancing pauses until current drops
  (CV taper, end of charge, or `Start`). Fewer bleed hours, but on the right
  cells. A later option is per-group resistance compensation (`R` estimated from
  ΔV/ΔI at current steps), which would allow picking under current; not
  proposed now.

### 5.4 Board-temperature guard from the LTC die (F2, F6)

- Every `BalanceDieTempPolls` voltage polls (proposal: 10 → every 2 s), inside
  the quiesce window after the cell read: `ADSTAT` (`CHST` = 0, 7 kHz, ~0.8 ms)
  then `RDSTATA` (~1.3 ms). Cost ~2 ms every 2 s — negligible against the
  ~40 ms already quiesced per poll.
- Per module, take the hotter of its two LTC dies, `T_die`. Allowed active
  cells for that module:
  `BalanceMaxActive × clamp((T_hard − T_die) / (T_hard − T_soft), 0, 1)`,
  rounded down, with `BalanceDieTempHystC` hysteresis on the way back up.
- Proposed defaults: `T_soft` = 60 °C, `T_hard` = 80 °C, hysteresis 5 °C —
  **COMMISSION**: the die runs below the pads by an unknown offset, so the
  limits must be set from one IR session that records die temperature next to
  the pad temperatures (this is also the measurement BALANCE-3 asks for).
- Invalid reading (`0xFF0x` redundancy code, outside −40..150 °C, read failure)
  → that module's mask 0, reason `DieTempInvalid`. A failing sensor never
  permits more heat.
- The cell-NTC `BalanceTempMax` lockout stays, gains hysteresis
  (`BalanceTempHystC`, 5 °C), and becomes the second line instead of the only
  one.
- **Bonus, diagnostic only at first:** the same conversion returns `SC`, the sum
  of the LTC's cells measured through an independent 20:1 path
  (`SC × 20 × 100 µV`). Comparing it with the sum of that LTC's cell readings
  gives a per-LTC plausibility check that does not depend on the ADCV path.
  Publish the difference first; decide on a tolerance and an action once real
  data exists.

### 5.5 State, reasons and accounting (F3, F4)

**State:** `Off`, `Blocked`, `Waiting` (no clean sample), `Active`, `Holding`
(bleeding a held decision), `Done`.

**Inhibit reasons** (bitfield, all that apply): `OpOff`, `StateNotAllowed`,
`CellDataFault`, `TempsUntrusted`, `NoThermalData`, `PackHot`, `BoardHot`,
`DieTempInvalid`, `HighCurrent`, `WaitingCleanSample`, `QuiesceFailHold`,
`AllModulesDisabled`.

**Done:** a clean pick selects nothing and the spread (max − floor) is
≤ `BalanceDeltaMv`. Leaves `Done` when a clean spread exceeds it again.

**Accounting:** per-cell bleed time (s), and estimated charge removed
(V / 23.5 Ω × time × measured duty) per cell and per module; spread at each
clean pick, giving a measured convergence rate in mV/h. No time-to-balance in
hours is promised: that needs the cell OCV slope, which is not in the repo.

**Telemetry:**
- New pit-diag frame **`0x6CC PIT_balance_status`** (1 Hz, gated like the other
  pit frames; `0x6CC`–`0x6CF` are free): state (u8), inhibit bits (u16), active
  cells pack-wide (u8), spread at last clean pick (u16 mV), hottest die (i8 °C),
  clean-sample age (u8, s, saturating). Declared in a `.def`, so the DBC follows.
- Optional `0x6CD`: hottest die per module (5 × i8) + per-module allowed count.
- SD log: append `bal_state`, `bal_inhibit`, `bal_active` to the LOG CSV
  (column additions; log readers that index by header name are unaffected).

### 5.6 Bleed-path self-test and convergence monitor (F5) — second step

- **Self-test:** every `BalanceSelfTestMin` (proposal: 10 min) while `Active`
  on a clean sample, pick one selected cell per module; on the next poll measure
  once with only that cell's DCC left on, then the normal quiesced read. A
  working path reads lower by more than `BalanceSelfTestMinMv` (proposal 5 mV;
  9–36 mV predicted — set from bench data). Three consecutive failures →
  `BleedPathFault` for that cell: reported, excluded from selection.
- **Convergence:** a selected cell whose excess does not fall over
  `BalanceConvergeWindow` of its own bleed time → `NotConverging` (report only).
- **Stuck-on suspicion:** an unselected cell drifting down against the pack
  median faster than `BalanceDriftMvPerH` → `StuckOnSuspect` (report only).
- Cost: one extra ADCV + reads (~11 ms) every 10 min.
- Held for a second PR because it changes the poll sequence and its thresholds
  need bench data.

### 5.7 What does not change

`compute_mask()` selection rules, `BalanceDeltaMv` 50 / `BalanceStopDeltaMv`
20, `BalanceMaxActive` 8, `BalanceSpreadNoAdjacent`, the second-lowest floor,
the quiesce including ADOW, the cell-data fault gate, the `0x103` 5 s dead-man,
`0x104` per-module enable, `DCTO = 0000`. The brief's §4 reasoning for each
still holds.

---

## 6. Charging and balancing (D2)

The charger is standalone and runs its own profile; nothing in the system can
reduce its current for balancing. That matters at the top of charge:

- A cell more than ~50 mV above the rest reaches `CellOverVoltageMv` (4200 mV)
  before the pack is full → `CellOverVoltage` latches → `Error`.
- With D1, **no balancing runs in `Error`**. An imbalanced pack can therefore
  end every charge in `Error` and never be balanced unless someone clears the
  fault and balances in `Start`.

Options, not decided:

| Option | What | Cost |
|---|---|---|
| A | **Set the charger's CV voltage below 95 × 4.20 V** (e.g. 95 × 4.15 = 394 V) so the top cell keeps margin while the CV taper gives clean picks. | Procedure only. Gives up a little capacity. |
| B | **Graceful end of charge in the AMS:** when the hottest cell reaches `ChargeEndMv` (e.g. 4170 mV, below the 4200 mV OV fault), end the charge by returning to `Start` (contactors open, not latched), then balance in `Start` at rest (clean). | FSM change: safety-critical surface, two reviews. |
| C | **Balance in `Start` before charging** whenever the clean spread is large (`On`, operator-driven). | Procedure only. Slow (~5 h per 50 mV at 18 Ah). |

**Needed to decide:** the charger model, its profile (CC/CV? end current?), and
the CV setpoint currently used.

---

## 7. FMEA impact

| Item | Effect |
|---|---|
| SEASON-3 (forced balancing while driving) | **Closed** by the state gate (D1), backed by the 10 A current gate. |
| BALANCE-3 (board temperature unmeasured) | **Mitigated** by the die-temperature guard; stays open until the IR session calibrates `T_soft`/`T_hard`. |
| BALANCE-1 (failed quiesce) | Unchanged (already handled). |
| New: die sensor fails | Fail-safe: invalid reading inhibits that module. |
| New: never a clean sample | Balancing never runs, but visibly (`WaitingCleanSample`), not silently. |
| New: self-test false positive | Report-only for convergence/stuck-on; exclusion only after 3 consecutive self-test failures. |

---

## 8. Rollout

Each step: host tests for every new rule, CI, bench check; `safety-critical`
label wherever `ams_config.hpp` changes.

1. **PR A — controller + state gate + reasons.** `BalanceController` skeleton
   wrapping `compute_mask`, D1 state gate, inhibit reasons and state, `0x6CC`,
   SD columns, thermal-lockout hysteresis. Behaviour change: no balancing outside
   `Start`/`Charge`.
2. **PR B — current-aware picking** (§5.3). Thresholds as COMMISSION constants.
3. **PR C — die-temperature guard** (§5.4): `ADSTAT`/`RDSTATA` in the LTC
   driver, per-module derate, `SC` plausibility published.
4. **PR D — bleed-path self-test and convergence monitor** (§5.6).
5. **Optional PR E — end-of-charge strategy** (§6), once the charger is known.

---

## 9. Measurements needed

| # | Measurement | Unblocks |
|---|---|---|
| M1 | **IR frame of the 2512 band in the sealed box at 8 active cells, with die temperatures logged alongside** (and ambient). | `T_soft`/`T_hard` (PR C); closes BALANCE-3. |
| M2 | `g_bms_volt_poll_ms` / `_max` with balancing active (already on pit-diag). | Confirms the ~80 % duty and the ADSTAT/self-test cost. |
| M3 | One logged charge session: `I_filt_mA` + cell voltages through CC → CV → end. | `BalanceRankMaxCurrentMa`, `BalanceRelaxMs` (PR B); §6 choice. |
| M4 | Charger model, profile and CV setpoint. | §6. |
| M5 | Cell datasheet (capacity, DC resistance, OCV curve). | F1 numbers; any time-to-balance estimate. |
| M6 | Bleed displacement of a selected cell, DCC on vs off (bench). | `BalanceSelfTestMinMv` (PR D). |
