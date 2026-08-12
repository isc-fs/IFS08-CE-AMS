# AMS firmware — failure modes and effects

A register of the ways this firmware can fail to protect the pack, and what
the code actually does about each one today.

**How to read it.** Every entry is written against the source, and every
entry names the symbol it came from. References are by **file + symbol**, not
line number — line numbers rot faster than anything else in a doc, and a
stale line number sends you to the wrong function with full confidence.
Open the file, jump to the symbol, and check the claim before you act on it.
If the code disagrees with a sentence here, the code wins and the sentence is
a bug — fix it.

**There are no RPN scores.** Occurrence and detection ratings would be
invented numbers: nothing in this repo measures how often a contactor welds
or how likely an isoSPI glitch is, so a product of three guesses is a guess
wearing a lab coat. Entries carry a priority band instead, which is
engineering judgement and says so:

| Band | Meaning |
|---|---|
| **Open** | A gap in protection. Close it, or write down that you accept it, before the pack is energised. |
| **Accepted** | Understood, currently lived with, and the reason is recorded here. Re-read the reason before you rely on it. |
| **Latent** | Real but needs an unlikely precondition. Listed so nobody rediscovers it from scratch. |

## The Open items, at a glance

| ID | One line |
|---|---|
| [COMMISSION-1](#commission-1--placeholder-thresholds-can-ship-unchanged--open) | Nothing checks the cell V/T thresholds were ever changed from placeholders. |
| [COMMISSION-2](#commission-2--current-calibration-is-per-carrier-and-unverified-in-firmware--open) | Current zero/gain are per-carrier; the sensor check catches disconnection, not mis-scaling. |
| [COMMISSION-3](#commission-3--the-over-current-trip-is-not-a-thermal-protection-and-the-thermal-protection-is-disarmed--open) | **Cell temperature faults are disarmed, and the current trip was never meant to cover slow overload.** |
| [LATCH-1](#latch-1--every-backup-register-write-is-unverified--open) | Every `ErrorLatch` write is fire-and-forget; the read is an exact-match compare. |
| [LATCH-3](#latch-3--ams_hil_clear_error_latch-has-no-build-system-guard--open) | The bench latch-wipe flag has no build-system guard against reaching flight. |
| [RELAY-2](#relay-2--a-welded-precharge-contactor-is-invisible-to-the-swap-check--open) | The contactor-swap check reads bus voltage, so a welded precharge contactor passes it. |
| [DISCHARGE-1](#discharge-1--both-halves-exist-the-pairing-is-unproven--open) | **Both halves of the discharge interlock now exist, but no bench has run them on one bus.** |
| [BALANCE-1](#balance-1--a-failed-quiesce-corrupts-the-readings-the-selector-ranks--open) | A failed balance quiesce feeds bleed-displaced cell voltages to the safety predicates and the open-wire scan. |
| [BALANCE-3](#balance-3--board-temperature-in-a-sealed-box-is-unmeasured--open) | Balance-board temperature in a sealed accumulator has never been measured. |
| [BMS-2](#bms-2--open-wire-detection-covers-interior-conductors-only--open) | Open-wire detection is live, but the endpoint conductors are bench-unvalidated. |
| [TICK-1](#tick-1--tick_age-reports-a-wrapped-stale-reading-as-fresh--open) | At the 32-bit tick wrap, `tick_age()` reports every stale service as fresh. |
| [WATCHDOG-2](#watchdog-2--a-looping-but-wrong-maintask-holds-the-airs-closed--open) | Nothing supervises a `MainTask` that keeps looping while computing wrong answers. |

---

## 0. What actually protects the pack

Before the failure modes, the three things that do the protecting. Almost
every entry below is a way one of them is weaker than it looks.

1. **The predicate set** (`safety_predicates.hpp` — `evaluate_fault_detail`)
   runs every 10 ms in `MainTask` and is the only thing that decides "fault".
   It is pure logic over three snapshots, so it is fully host-testable and
   carries no HAL.
2. **Local relay drive.** `MainTask` opens the contactors and drops `AMS_OK`
   inline in the same tick that latched the fault (`safety_task.cpp` —
   `SafetyTask::latch_error_`). No CAN frame, no other task, and no external
   node is in that path. Telemetry loss is an observability failure, never a
   control failure.
3. **The sticky `ErrorLatch`** (RTC backup register) so a latched fault
   survives a reset and the next boot comes up already in `Error`.

Everything else — telemetry, pit-diag, SoC, balancing — is off the safety
path by design. `soc_estimator.hpp` opens with that contract in as many
words.

### The fault-response budget

The FS rule is a shutdown within 500 ms. The arithmetic behind that, from
`ams_config.hpp`:

- Cell voltage faults: 200 ms voltage poll (`BmsPollVoltMs`) + 25 confirming
  10 ms evaluations (`CellFaultConfirmTicks`) ≈ **460 ms worst case**. There
  is almost no margin. If you change either constant, redo this sum — the
  comments in `ams_config.hpp` carry it.
- Module loss: `BmsStaleMs` = 350 ms crossed at the second poll after loss
  (~400 ms) + one 10 ms tick ≈ **410 ms**, then `BmsStaleConfirmTicks` = 25
  (~250 ms) on the `BmsStale` branch. `BmsModuleOffline` (mask mismatch) is
  **not** debounced and latches on the first tick.
- Everything else immediate-danger — current over-limit, current stale, VCU
  stale, charger stale, open-wire — latches on the first tick.

---

## 1. Commissioned constants are trusted, never checked

### COMMISSION-1 — Placeholder thresholds can ship unchanged · **Open**

`ams_config.hpp`: `CellUnderVoltageMv` = 2800, `CellOverVoltageMv` = 4200,
`CellUnderTempC` = −10, `CellOverTempC` = 60. All tagged `COMMISSION` and all
described in the file's own header as placeholder defaults to be finalised
against the cell datasheet and the FS rules.

Nothing in the firmware checks that they were ever changed. There is no
`static_assert`, no runtime plausibility window, and no boot frame that emits
the active thresholds so the bench can read them back before energising.

**Effect.** Set too low, the under-voltage protection is simply absent for
the range between the real cell limit and the configured one — the safety
function is lost silently, and the pack looks healthy on telemetry the whole
time. Set too high, the car false-trips under load and the team learns to
distrust the AMS, which is its own hazard.

**What would close it.** A compile-time assertion that the values differ from
the placeholders, plus a boot-time frame carrying the active thresholds so
the pit can confirm them against the cell datasheet before the first run.

### COMMISSION-2 — Current calibration is per-carrier and unverified in firmware · **Open**

`CurrentZeroCount` = 2054 and `CurrentMvPerAmpe1` = 46 (`ams_config.hpp`,
both `COMMISSION`). The zero tracks the ADC's VREF+, so it is a property of
the specific carrier board, not of the design — swap the board or the
reference components and both numbers move.

The firmware does check *something*: `CurrentService` runs a leg-voltage
plausibility window (`CurrentLegPlausMinMv` = 700 mV, `CurrentLegPlausMaxMv`
= 2300 mV) debounced over `CurrentDisconnectConfirm` = 3 consecutive reads,
and asserts `sensor_fault` → `FaultReason::CurrentSensorFault`. That catches
a **disconnected or railed** sensor. It does not catch a *mis-scaled* one:
a wrong zero or gain sits comfortably inside that window.

**Effect.** Pack current reads systematically wrong. Under-scaled, the pack
exceeds `CurrentMaxMa` without the predicate seeing it. Over-scaled, the car
trips at a load it should survive.

**What would close it.** A zero-count plausibility check (the raw
differential code should sit near mid-scale at zero current) wired into the
existing `CurrentSensorFault` predicate, plus emitting the commissioned
zero/gain at boot so the pit can tie them to the carrier's part number.

### COMMISSION-3 — The over-current trip is not a thermal protection, and the thermal protection is disarmed · **Open**

This one is a compound of two verified facts and it is the most important
paragraph in this document.

`CurrentMaxMa` = 185 000 (185 A) is the **6P continuous rating** of the
cells: 95S6P of VTC6 at 30 A each. The comment in `ams_config.hpp` states the
corollary plainly — *currents between the cell rating and this limit never
trip at all, by design*, and protection against a slow overload is meant to
be the cell temperature path.

The cell temperature path is off. `TempFaultsTrusted` = false, and
`evaluate_fault_detail` wraps both `CellUnderTemp` and `CellOverTemp` in that
flag. The reason is recorded and legitimate: NTC temperatures come through
the per-LTC ADG731 32:1 mux, whose select word was wrong and whose fix is not
yet validated on the flight path, so faulting on those numbers would trip the
car for no reason.

**Effect, today: the AMS will not open the contactors for any cell
temperature, at any value.** A sustained overload between the cell rating and
185 A is caught by neither branch. Cell *voltage* protection is unaffected
and still works.

There is one surviving thermal-adjacent protection: `TempSensorPresenceCheck`
= true arms `FaultReason::TempSensorDisconnected`, which fires when a channel
that had previously read valid goes open past `TempDisconnectPolls`
(`BmsService::recompute_summaries_` sets `temp_disconnect_mask`). That is a
*presence* check — an open NTC reads the rail regardless of calibration — so
it is deliberately armed independently of the trust flag. It protects against
a lost sensor, not against a hot cell.

**What would close it.** Validating the ADG731 mux path end-to-end on flight
hardware and flipping `TempFaultsTrusted`. Until then this gap must be in the
written safety case, not just in a config comment.

---

## 2. The sticky-error contract is assumed, not enforced

`ErrorLatch` is the mechanism that stops a car that faulted from being
re-armed by a power cycle. Four separate ways it can quietly fail to do that.

### LATCH-1 — Every backup-register write is unverified · **Open**

`error_latch.cpp` is four functions and no error handling anywhere:

- `ErrorLatch::init()` calls `HAL_PWR_EnableBkUpAccess()`, which returns
  `void`. If the backup-domain write protection does not actually lift, every
  subsequent access is silently wrong.
- `ErrorLatch::set()` is a bare `*bkp_register() = Magic` with no read-back.
- `ErrorLatch::is_set()` is an **exact** compare against
  `BkpErrorMagic` = `0xA115EE51` in `BkpErrorReg` = 1. Any single flipped bit
  in the stored word reads as "not latched".

**Effect.** A fault latches in RAM, the relays open, `AMS_OK` drops — and the
persistent half silently does not happen. The next boot comes up in `Start`
on a pack that faulted. If the underlying fault is persistent it re-latches
after the 2 s boot grace, which is the saving grace; if it was
intermittent, nothing remembers it.

**What would close it.** Write-then-read-back on every `ErrorLatch::set()`,
and a defined response on mismatch. Note the ordering question too:
`latch_error_()` opens the relays *first* and writes the latch last, which is
the right order for immediate safety and the wrong order for persistence.

### LATCH-2 — The backup domain has no VBAT on the bench carrier, and flight is unconfirmed · **Accepted**

Recorded in `app_init_task.cpp`: the backup domain only persists across a
power-off when the carrier has a VBAT source (coin cell or supercap). The
bench carrier has none, so a power cycle wipes the latch there regardless of
any build flag. Flight-hardware VBAT is **unconfirmed**.

**Effect.** "The latch survives a power cycle" is a property of the *board*,
not of this firmware, and on at least one board it is false. Read the
invariant as *survives a warm reset* — which is what the code guarantees —
and treat power-cycle persistence as unproven until someone confirms the
flight carrier's VBAT with a meter.

### LATCH-3 — `AMS_HIL_CLEAR_ERROR_LATCH` has no build-system guard · **Open**

`CMakeLists.txt` declares `option(AMS_HIL_CLEAR_ERROR_LATCH ... OFF)` and
passes it through as a compile definition. `app_init_task.cpp` guards the
`ErrorLatch::clear()` call with `#if defined(...)`, and the comment there
says in capitals that it must never be compiled into a flight build.

There is no assertion that enforces it. Nothing fails the build, nothing
refuses to flash, and nothing in the running image says the flag was set.
A copy-pasted bench build script is all it takes.

**Effect.** The sticky-error contract is defeated at every boot. A pack that
faulted comes up in `Start` forever after, and the only symptom is the
absence of one you were relying on.

**What would close it.** A CMake-level failure when the flag is combined with
a release/flight configuration, or a sentinel in the firmware-info block so
the flashing tool and the health frame can both see it.

### LATCH-4 — A boot-time chain glitch latches a sticky do-not-drive · **Accepted**

`app_init_task.cpp` runs LTC chain discovery **once**: wake the chain, issue
one `RDCFGA`, count PEC-clean segments, and if the count is not exactly
`LtcChainLength` (10) call `ErrorLatch::set()` and `Relays::open_all()`.
There is no retry.

**Effect.** A single transient isoSPI corruption during boot produces a
sticky `Error` that a warm reset cannot clear — on flight hardware with
working VBAT, the car is undriveable until someone power-cycles the
accumulator.

This is deliberately conservative: the AMS cannot reason about cells it
cannot observe, so refusing to leave `Error` is correct behaviour for a
genuinely broken chain. The cost is that a glitch is indistinguishable from a
break. A bounded retry (a small number of attempts with a short delay before
latching) would keep the conservative outcome for a real break and absorb the
glitch.

---

## 3. Contactors: nothing confirms they moved

### RELAY-1 — Read-backs report the coil command, not the contact · **Accepted**

`relay_driver.cpp` provides `is_air_negative_closed()`,
`is_air_positive_closed()` and `is_precharge_closed()`. All three are
`HAL_GPIO_ReadPin` on the **output** pin: they tell you what the firmware
drove, never what the contactor did. A shorted drive FET, an open coil, or a
welded contact all read "as commanded".

These read-backs *are* now consumed — `MainTask` emits them on
`0x4A4 AMS_relay_status` every `RelayStatusPeriodMs` = 100 ms, and the frame
comment in `safety_task.cpp` says exactly this: "confirm what we drive the
coils to, not that the contactor physically closed." No FSM gate reads them;
`state_machine.hpp` contains no call to any `is_*_closed()`.

**Effect.** There is no electrical feedback path from the contactors into the
safety logic. Every contactor failure is detected *functionally* — by the DC
bus not doing what it should — or not at all.

### RELAY-2 — A welded precharge contactor is invisible to the swap check · **Open**

The `Precharge → Transition` edge emits `CloseAirP | OpenPrecharge` in one
tick. On the next FSM step, `Transition` re-checks
`precharge_target_reached()` before committing to `Run`. That check reads the
**bus voltage**. If the precharge contactor failed to open, the bus is still
high — for the wrong reason — and the check passes.

**Effect.** AIR+ closes with the precharge resistor still bridged. The
resistor, rated for transient duty, sits across the closed path. On the next
arm cycle the inrush limiting is already gone.

**What would close it.** Reading `is_precharge_closed()` in `Transition` at
least tells you the *coil* was de-energised, which catches a firmware or
driver fault though not a mechanical weld. Genuine coverage needs a contact
or current measurement the board does not currently have.

### RELAY-3 — A contactor that never closes costs 5 s of resistor soak · **Accepted**

The only bound on a stuck precharge is `PrechargeMaxMs` = 5000 ms
(`state_machine.hpp`, `Precharge` case). Below the target voltage the FSM
sits there for the full window before latching `Error`.

**Effect.** The precharge resistor carries current for up to 5 s. Its rating
is transient duty. This is the failsafe ceiling, not a comfortable margin,
and `PrechargeMaxMs` is tagged `COMMISSION` against the resistor's actual
thermal limit — confirm that number against the part before trusting it.

### RELAY-4 — `latch_error_()` cannot fail loudly · **Latent**

`SafetyTask::latch_error_()` calls `Relays::open_all()`,
`Relays::set_ams_ok(false)` and `ErrorLatch::set()` with no return checks —
correctly, because `HAL_GPIO_WritePin` returns `void` and the build is
`-fno-exceptions`. `open_all()` is a single BSRR write across all three relay
pins, so it is atomic and interrupt-safe, and the driver documents the
invariant that keeps it that way: **all three relay outputs must stay on the
same GPIO port**. Split them across ports on a future carrier and the atomic
open silently stops being atomic.

**Effect.** If the GPIO peripheral is in an undefined state the relays may not
open and nothing notices — and see WATCHDOG-1 below for why the watchdog will
not rescue you here.

---

## 4. DC-link discharge interlock

### DISCHARGE-1 — Both halves exist; the pairing is unproven · **Open**

Read this one in full before touching the FSM's `Start` case.

**The hardware situation.** The bleed relay is normally-closed and wired into
the shutdown circuit with no software control. Opening the SDC de-energises
it, the bleed connects, and the link drains. Close the SDC again before the
link has drained and the relay re-energises, the bleed disconnects, and **the
discharge stops part-way**. The link is then stranded at a voltage nobody can
predict from how long ago the SDC was cycled.

**Why the AMS cannot fix it.** Its own leg of the loop is `AMS_OK`, which
latches in hardware (see AMSOK-1). The AMS cannot pulse the SDC low to
restart a discharge. So it publishes the two facts only it can see —
`0x021 ACU_discharge_interlock` carrying `fsm_in_start` and `tsms`, emitted
every `EcuMidTxMs` = 100 ms from `AcuCanTask` — and the ECU, which owns both
a DC-link measurement and a normally-closed relay in series with the bleed
relay's coil, makes the decision. The message definition
(`Core/Inc/can/messages/acu_discharge_interlock.def`) carries the full
contract, including why the ECU must latch on entry rather than evaluate
continuously:
securing the discharge connects the bleed, which is the very thing `tsms`
reports on, so a continuous evaluation would falsify its own trigger.

**The AMS consumer side is implemented.** `VehicleService::update_from_frame`
reads `0x100` byte 2 bit 0 into `discharge_engaged`, and latches
`ecu_discharge_capable` the first time it sees the frame at DLC ≥ 3.
`fsm::rearm_permitted` refuses to leave `Start` while `discharge_engaged` is
set, and — only once `ecu_discharge_capable` has latched — also requires
`dc_bus_V ≤ DcBusDischargedV` (60 V). Charger mode is exempt: the inverter is
not in the charge loop and `dc_bus_V` is VCU-only, so gating it would make
charging unarmable. A blocked attempt holds in `Start` rather than latching,
and **consumes the DASH_CHG press** deliberately, so a press made while the
link was live cannot arm the car by itself seconds later when the discharge
finishes.

**The ECU producer side is implemented too**, on `IFS08-CE-ECU` `dev`: it
mirrors `0x021` field for field, supplies the third term from its own DC-link
measurement, latches the hold and releases at its `DischargeReleaseV` = 10 V —
below the 60 V gate here, so this repo's re-arm condition clears before the ECU
lets go — and gives up after 30 s with a fault if the link does not fall. It
sends `0x100` at DLC 3, so `ecu_discharge_capable` latches against that image.

**The gap is that the two have never met.** The AMS side is exercised only by
`tests/unit/test_state_machine.cpp`, the ECU side only by its SIL, and no bench
has had both boards on one bus. A textual diff of the two `.def` files is not a
test: the first ECU implementation of `0x021` decoded bit 0 correctly and read
it as a complete discharge request, ignoring `tsms` entirely. Layout agreement
does not imply semantic agreement.

**Which image is on the car decides whether any of it runs.** ECU `main` still
sends `0x100` at DLC 2, where `ecu_discharge_capable` never latches,
`discharge_engaged` reads 0, and `rearm_permitted` returns true
unconditionally. That fallback is deliberate — refusing to arm over a link the
other end cannot drain would brick the car — but it means the protection is
absent, silently, against older ECU firmware. Watch the DLC of `0x100` to tell.

**What that costs you when it is inert.** Arming onto a stranded link is
possible, and the failure is quiet rather than dramatic: with the link already above 95 % of
pack, `precharge_target_reached()` is satisfied on entry, the FSM leaves
`Precharge` on the next 20 ms step, and the resistor never carries meaningful
current. That 95 % check is the **only evidence the AMS ever gets that the
precharge resistor and contactor work**. Satisfied by residual charge, it
proves nothing — and it proves nothing silently, on a perfectly normal-looking
arm sequence.

### PRECHARGE-1 — Why `precharge_target_reached` requires freshness · **Accepted (do not regress)**

Not an open failure mode — a fixed one, recorded here because the fix looks
like a redundant check and someone will eventually try to delete it.

`VehicleState` holds the **last received** `dc_bus_V`. When the VCU stops
publishing `0x100`, that number does not go away, it freezes. Frozen at pack
voltage it satisfies the 95 % criterion **forever**, including after the link
has actually bled to zero — where closing AIR+ means the full pack voltage
across the contactor with nothing limiting the inrush.

So `precharge_target_reached(bms, veh, dc_bus_fresh)` takes freshness as a
required argument and returns false without it. `MainTask` computes
`dc_bus_fresh` against `VcuStaleMs` (200 ms), not the looser `VcuFreshMs`
used for the mode lock, and treats a never-seen VCU (tick 0) as not fresh.

The `VcuStale` predicate alone cannot cover this: it is gated on
`vcu_required`, which is false in `Start`, so the value may already be
arbitrarily old at the moment the operator presses; and it needs 200 ms while
the FSM steps every 20 ms. `Precharge → Transition` would fire on the frozen
reading roughly ten steps before the fault could reopen the AIRs. **Freshness
has to be part of the criterion, not a separate fault racing it.**

`bus_below_collapse()` deliberately does *not* take freshness, and the reason
is worth understanding: it is consumed only in `Run`, where the mode is locked
to Car, so `vcu_required` is true and `VcuStale` bounds staleness at the same
200 ms its own debounce already spends. Both of its stale outcomes are safe —
a false collapse de-energises to `Start` without latching, a missed one is
caught by `VcuStale` — so it has no race to lose.

### AMSOK-1 — `AMS_OK` latches in hardware and can never be a temporary interlock · **Accepted (hard constraint)**

The AMS's leg of the shutdown circuit is a **self-holding relay (K5) plus an
`RST_BMS` button the driver cannot reach** — recorded in
`docs/ARCHITECTURE.md` §1 invariant 8 and `docs/CAN_MAP.md`. This is a hardware fact, not
visible anywhere in the firmware, and it constrains the firmware absolutely.

Once `Relays::set_ams_ok(false)` runs, the loop stays open until somebody
physically presses that button. Driving PB4 high again does nothing.
`MainTask` re-drives the pin from `safety::ams_ok_asserted()` on every 10 ms
tick, so the *pin* will go high again once the latch clears in RAM — the
*loop* will not.

**The failure mode is a future change, not current code.** Any design that
drops `AMS_OK` momentarily — to open the SDC so the link bleeds, to
de-energise on a TSMS release, to implement any kind of soft interlock —
bricks the car until someone opens the accumulator. `AMS_OK` is health-only:
driven by the predicate set, never by an operator input.

The FSM is shaped by this one fact and it is worth seeing how. A TSMS drop is
a **non-latching** de-energise to `Start`, precisely so `AMS_OK` stays up: the
FS rule requires the driver to stop and restart the tractive system unaided,
and `AMS_OK` sits *upstream* of TSMS in the loop, so latching on a TSMS drop
would open the upstream element and reclosing TSMS could no longer restore
it. The one exception is Charger mode, where scrutineering forbids
re-activating the charge output after the SDC opens — there a TSMS drop
latches `Error` (`FaultReason::ChargerTsmsOpen`) and the same irreversibility
is the desired interlock.

---

## 5. Balancing

Balancing is not on the safety path — but it manipulates the pack while the
safety path is measuring it, and that coupling is where its failure modes
live.

### BALANCE-1 — A failed quiesce corrupts the readings the selector ranks · **Open**

**Why a quiesce is needed at all.** BMS_LITE does not bleed through the
LTC6811's own S-pin switch. Each cell drives an **external TSM2323 PMOS**
whose gate sits behind a 10 k / 10 n RC, τ ≈ 100 µs. So the ADCV `DCP=0` bit
does not stop the bleed — and per LTC6811 datasheet Table 53 it would not be
enough anyway, since it suppresses discharge only on the cell being measured
and its immediate neighbours. Roughly half the selected cells keep pulling
~165 mA for the whole conversion. `run_voltage_poll` therefore calls
`quiesce_balancing()` first: clear every DCC bit, then wait
`BalanceQuiesceMs` = 2 ms (~20× the gate RC) before converting.

**Why it matters numerically.** The bleed current does not return through the
sense path — on-board sensing is close to Kelvin — it returns through the
**harness**. 179 mA across a plausible 50–200 mΩ of tap, connector and fuse
impedance is **9–36 mV**, and it has *opposite sign* on the bled cell (reads
low) and on its neighbours (read high, because the shared tap node moves).
Against `BalanceDeltaMv` = 50 mV that is a first-order corruption of the very
signal the selector ranks on. It has been observed on the bench.

**The failure.** `quiesce_balancing()` retries `WRCFGA` once and, if both
attempts fail, sets `g_balance_quiesce_fail`, bumps
`g_balance_quiesce_fail_count` (published on pit-diag `0x6CB`) and returns
false — the poll then measures **anyway**. `maybe_run_balance_update()` reads
the flag and skips that balance window rather than re-ranking on the bad
snapshot.

That protects the selector. Three things it does not protect:

1. **The safety predicates still consume the displaced readings**, and that
   is deliberate — the comment in `quiesce_balancing()` says stale cell data
   starves the predicates, which is worse than a noisy read. But the
   consequence is real: during a failed quiesce `max_cell_mV` can read up to
   ~36 mV high on a neighbour of a bled cell. Balancing runs where cells sit
   near the top of charge, i.e. near `CellOverVoltageMv` = 4200. The 250 ms
   cell debounce does not help, because the displacement persists for as long
   as the bleed does. The tap-artifact guard in
   `BmsService::recompute_summaries_` does not cover it either: that guard
   needs an intra-pair split of at least `TapArtifactMinSplitMv` = 800 mV
   *and* one reading outside the implausible-cell window, and 36 mV is more
   than an order of magnitude short of triggering it.
2. **The open-wire scan runs under bleed too.** `run_voltage_poll` calls
   `attempt_open_wire_poll()` unconditionally when `CellOpenWireCheck` is
   set, not conditionally on the quiesce having succeeded — and the comment
   there assumes balancing "is STILL quiesced". `CellOpenWire` faults in
   **any** state and is **not** debounced. The margin saves it in practice —
   `CellOpenWireDeltaMv` = 400 mV against a 9–36 mV artifact, an order of
   magnitude — but nothing in the code enforces that margin, so it is
   arithmetic doing a gate's job.
3. **A persistently failing quiesce freezes balancing on a stale mask.**
   Each window returns early, `s_prev_balance_mask` never advances, and
   `restore_balancing()` is skipped (it only runs when the quiesce
   succeeded), so the chain keeps bleeding the *last* selection
   indefinitely. The only symptom is a climbing counter on `0x6CB`.

**What would close it.** Gate `attempt_open_wire_poll()` on the quiesce
result, and give a repeated quiesce failure a real reaction rather than a
counter — the cheapest is to command an all-zero DCC mask so a chain that
cannot be quiesced also cannot bleed.

### BALANCE-2 — The thermal guard reads a path nobody trusts · **Accepted**

`BalanceTempsTrusted` = true while `TempFaultsTrusted` = false. The two flags
answer different questions on purpose — "trust these temps enough to *open
the contactors*" versus "trust them enough to *let balancing run*" — and
coupling them meant the operator's balance switch was accepted and then
produced an all-zero mask forever.

The residual risk is written out in `ams_config.hpp` and is worth repeating:
passive balancing dumps heat into the cells while its **only** thermal
protection is the `BalanceTempMax` = 50 °C lockout, which reads the
unvalidated ADG731 mux path. A genuinely hot cell whose sensor is mis-routed
by the mux would not raise `max_tempC` and would not trip the lockout.

The mitigations are real but bounded: the 5 s operator dead-man
(`BalanceOverrideFreshMs`, folded in by
`VehicleService::effective_balance_cmd` so a stale link arrives as `Off`),
at most `BalanceMaxActive` = 8 cells per module bleeding at once, and
`BalanceSpreadNoAdjacent` = true so no two physically adjacent 2512 resistors
are ever on together. `compute_mask` also refuses outright when
`valid_temp_channels` is below `BalanceMinValidTempCh` — without that check a
pack with a dead temperature path would balance with `max_tempC` at
`INT16_MIN`, which compares as wonderfully cool.

**Balance with cell temperatures observed by some other means until the mux
path is validated.**

### BALANCE-3 — Board temperature in a sealed box is unmeasured · **Open**

`BalanceMaxActive` = 8 gives 6.0 W per module and 30 W across the pack at
4.2 V. The resistors are comfortable — 2 W 2512 parts at ~0.37 W, under a
fifth of rating. The constraint is **heat out of the accumulator box**, and
that is unchanged by the part rating. The ~71 °C pad figure in
`ams_config.hpp` is a single pad on an **open bench**. Nobody has measured a
sealed box at this setting, and the lockout that would catch an overheating
board reads the same unvalidated NTC path as BALANCE-2.

### BALANCE-4 — What the policy is now, so you do not re-derive it · **Accepted**

Short notes on `balance_controller.hpp`, because each one exists to prevent a
specific failure and reads like an arbitrary choice otherwise:

- **The floor is the second-lowest cell in the pack, not the minimum.** A
  disconnected tap reads spuriously low; if the floor were the true minimum,
  every real cell would sit more than `BalanceDeltaMv` above it and the
  **whole stack** would start discharging off one faulty reading. The true
  minimum still drives the UV predicate, so a genuinely low cell still
  faults.
- **Hysteresis, via a `previous` mask passed in.** A cell already discharging
  holds its slot until it comes within `BalanceStopDeltaMv` = 20 mV of the
  floor; one that was not must clear the wider 50 mV. `compute_mask` is a
  pure function re-evaluated from scratch every window, so without this
  anything near the threshold toggles on and off and never accumulates useful
  bleed time. A `static_assert` enforces stop < start.
- **A latched cell-data fault stops balancing for the operator override too**
  (`is_cell_data_fault`: open-wire, over-voltage, under-voltage). The
  selector reads **raw** `cell_mV`; the tap-artifact correction lives in a
  local inside `recompute_summaries_` and never reaches it. So a split tap
  presents its high half at full value here and the greedy picks it first,
  every cycle, forever. Faults elsewhere — current, VCU, contactor — leave
  cell data intact and stay overridable so the pit keeps its manual rebalance
  path.

---

## 6. BMS chain and cell observation

### BMS-1 — No runtime chain-length re-check · **Accepted**

Chain length is verified once, at boot. During operation
`BmsService::update_from_ltc_response` walks all ten ICs and records which
ones were PEC-clean; `module_online_mask` is then re-derived from per-module
`last_rx_tick` freshness against `BmsStaleMs`, so a module that stops
answering drops off the mask and fires `BmsModuleOffline` (immediate, not
debounced). Per-IC PEC tallies are published on pit-diag `0x6C7`/`0x6C8`.

What is *not* checked at runtime is chain **continuity** — whether the ten
responses still correspond to the ten physical ICs in order. A daisy-chain
desync that still produces PEC-clean segments would not be caught by the
freshness path.

**Effect.** Cell voltages could be attributed to the wrong module. The
per-module detail byte on a fault would point at the wrong place, and
balancing would discharge the wrong cells.

### BMS-2 — Open-wire detection covers interior conductors only · **Open**

`CellOpenWireCheck` is **enabled** and the ADOW two-pass scan runs on the
200 ms voltage poll, so an open faults in under 500 ms in any state. This is
the only predicate that can see a broken cell tap at all: an open node floats,
one of the two cells sharing it rails high and the other low with their sum
conserved — exactly the signature the tap-artifact guard averages back into
range. Measured on the bench, a cell reading 2364 mV reached the FSM as
3823 mV. Cell over/under-voltage therefore **cannot** fire on an open tap.

Margin on a live pack is good: a real open reads about −4000 mV against the
400 mV threshold, roughly 10×, while a healthy pack stays inside
−130..+50 mV.

**The gap, stated in `ams_config.hpp` and unchanged here: only INTERIOR
conductors are hardware-validated.** The endpoint rules — C(0) via
`CELL_PU(1) == 0`, C(N) via `CELL_PD(N) == 0` — test for **exact** zero and
have never run on hardware. An endpoint open reading a few millivolts instead
of zero would be missed. That is roughly **2 of every 10 conductors per IC**.

### BMS-3 — `first_full_poll_done` is set once and never cleared · **Latent**

`evaluate_fault_detail` gates the cell V/T range checks on
`first_full_poll_done`, which `BmsService` sets when every module has
reported at least once and never clears.

**Effect.** If a module never comes online at all, the flag stays false and
the cell range checks stay suppressed for the whole session. That sounds
alarming and is not, because `module_online_mask` catches the absent module
first: `BmsModuleOffline` is evaluated *before* the range checks and latches
immediately. The gate is a boot-time guard against sentinels and a
partially-populated grid, and the module-presence path is what actually
protects an offline module.

### BMS-4 — Broadcast commands have no acknowledgement · **Latent**

`ADCV`, `ADAX`, `WRCOMM` and `STCOMM` produce no reply, so PEC cannot
validate them. `send_command()` returning true means the SPI transfer
completed, not that the chain received what you sent. A bit flip that still
completes cleanly is undetectable at the point of transmission.

**Effect.** The chain executes a different command, or none. Downstream reads
return stale or misaligned data. In practice a corrupted `ADCV` shows up as
PEC failures on the following reads (the conversion never happened), so the
failure is usually loud — but it is loud by accident, not by design.

---

## 7. Timebase

FreeRTOS ticks are 32-bit at 1 kHz, so they wrap at ~49.7 days. A race car
power-cycles constantly; a bench rig or a pack left on a charger does not.

### TICK-1 — `tick_age()` reports a wrapped stale reading as fresh · **Open**

This is the one wrap consequence that fails **dangerous**, so it gets its own
entry.

`safety::tick_age(now, last)` returns `(now >= last) ? (now - last) : 0`. The
clamp is deliberate and fixes a real bug: a producer that updates its
`last_*_tick` between `MainTask`'s `now` sample and the snapshot read would
otherwise underflow to ~4e9 and trip staleness at the boot-grace boundary.

But the clamp cannot tell "the producer just reported" from "the counter
wrapped". After a wrap, `now` is small and `last` is huge, so **a service
that has been silent since before the wrap reports age 0** — permanently
fresh. `tick_age` feeds `BmsStale`, `CurrentStale`, `VcuStale` and
`ChargerStale`, so all four staleness protections go quiet at once.

`VehicleService` repeats the same clamp in `charge_requested`,
`effective_balance_cmd` and `effective_balance_modules_mask`.

**What would close it.** Modular distance arithmetic — plain unsigned
subtraction compared directly against the threshold is already wrap-correct —
with the producer-ahead case handled by a bounded window rather than an
unbounded clamp. Whatever the fix, it needs a wrap-boundary unit test; the
existing tests all run inside one epoch.

### TICK-2 — The other wrap consequences fail safe · **Latent**

Recorded so nobody spends an afternoon on them thinking they are TICK-1:

- **Precharge timeout** (`state_machine.hpp`): raw `now_tick - state_entry_tick`.
  The comment argues the subtraction cannot underflow because `SafetyTask`
  owns both values — true within an epoch, false across a wrap. Across one,
  the difference becomes enormous and the timeout fires **immediately**:
  a spurious `Error` latch, not a disabled timeout.
- **`dc_bus_fresh`** (`safety_task.cpp`): raw subtraction, so a wrap reads
  *not* fresh and precharge refuses to complete → `PrechargeMaxMs` → `Error`.
- **`module_online_mask`** (`bms_service.cpp`): raw subtraction, so a wrap
  drops every module → `BmsModuleOffline` → `Error`.

All three latch `Error` spuriously, which is an availability failure, not a
safety one. TICK-1 is the only one that removes protection.

### CONCURRENCY-1 — Lock-free snapshots and the 0xFF fingerprint · **Latent**

No mutex is taken anywhere in app code. Each service has a single writer, and
`snapshot()` is a plain struct copy; 32-bit aligned loads and stores are
atomic on the Cortex-M7. The design tolerates a briefly inconsistent
multi-field read, and the range predicates are debounced to absorb one.

Worth knowing: under the current priority assignment a torn read is close to
unreachable. `MainTask` is the **only** `osPriorityRealtime` thread; the
producers are `AboveNormal` (`AcuCanTask`, `CurrentSensorTask`) and `Normal`
(`BmsPollTask`), so none of them can preempt `MainTask` mid-copy, and
`MainTask` does not block inside its snapshot sequence.

The fingerprint is already built: `module_below()` and friends return
`NoOffendingModule` = `0xFF` on the fault-detail byte when the summary
min/max disagrees with the per-module aggregates — the signature of a
snapshot copied across two poll cycles. **If you ever see `0x6C0[7] == 0xFF`
on a cell-range fault, treat it as evidence that something changed about task
priorities or about who writes service state**, not as routine noise.

---

## 8. Watchdog and liveness

### WATCHDOG-1 — The IWDG is not a fault-escape mechanism · **Accepted**

Read this before reasoning about any fault path, because the intuitive model
is wrong.

`MainTask` refreshes the IWDG on **both** branches: on the clean path after
the FSM step, and on the fault path immediately after latching. That is
deliberate and `ARCHITECTURE.md` invariant 5 states why — the relays are
already open and the latch persists, so staying alive is safe and lets an
operator read telemetry instead of watching the node self-reset in a 100 ms
loop.

**The consequence: "the fault path withholds the refresh, so the IWDG resets
the chip and `MX_GPIO_Init` re-opens the relays" is not true.** Any argument
that leans on a reset as the backstop for a failed relay write, a failed
latch write, or an inconsistent hardware state is unsound. The IWDG covers
exactly one thing: `MainTask` stopping.

The genuinely loud paths are elsewhere — the FreeRTOS stack-overflow and
malloc-failed hooks in `freertos.c` open all relays, set the latch, and then
**spin**, so the IWDG does reset the node with the latch set.

### WATCHDOG-2 — A looping-but-wrong `MainTask` holds the AIRs closed · **Open**

The IWDG detects a task that stopped iterating. It cannot detect one that
keeps iterating and computing the wrong answer, or one stuck on a resource
inside the loop body while still reaching the refresh.

`fw_health::poke(MainStepped)` is called at the top of every iteration, but
`sample_liveness()` is consumed only by the 1 Hz `0x6CA` health frame in
`AcuCanTask`. **It is observability, not a supervisor** — nothing in that
path opens a relay.

**Effect.** In `Run` with the contactors closed, a `MainTask` that iterates
without evaluating correctly keeps the AIRs closed indefinitely, which is the
gap in "no single stuck task can prevent an AIR open."

**What would close it.** An independent supervisor — a second timer ISR that
checks the iteration counter advanced and calls `ams_relays_open_all_c()`
(which exists precisely for callers that cannot include the C++ header) if it
did not.

---

## 9. CAN: mostly observability, but four IDs are control

Telemetry OUT of the AMS is observability only, and CAN-1 below explains why a
dropped frame is survivable. Traffic IN is a different matter: four IDs change
what the node does, and two of them are not state-gated.

| ID | Effect | Gated on |
|---|---|---|
| `0x002` | Opens all relays, writes the bootloader magic, resets the MCU | **FSM in Start or Error** (`Bootloader::reboot_allowed_in`) |
| `0x7E0` LOGFS opcodes | Reads the SD card, can hold the bus for minutes | FSM in Start or Error (`logfs_allowed_in`), NACK `0x15` otherwise |
| `0x103` `"BALN"` | Forces balancing on in **any** state | magic payload + 5 s freshness only |
| `0x104` `"BALM"` | Selects which modules may balance | magic payload + 5 s freshness only |
| `0x7F0` | Arms the pit-diag stream (~50 frames/scan) | magic payload only; adds bus load in any state |

`0x103`/`0x104` are the remaining gap: balancing forced on in `Run` bleeds
through resistors rated for transient duty while the pack is under load, and
nothing in firmware refuses it. The magic payload stops bus noise from doing it
by accident; it does not stop a tool from doing it on purpose at the wrong time.


### CAN-1 — Telemetry drops are silent, and that is survivable · **Accepted**

`send_telem()` in `safety_task.cpp` and `send_or_fail()` in `acu_can_task.cpp`
are non-blocking: a full 16-deep TX FIFO bumps a counter
(`g_telemetry_tx_fail`, `g_acu_tx_fail`) and the frame is gone. The telemetry
failure count is surfaced on `0x4A2` at a 500 ms cadence, which lags by
definition.

**Why this is not a safety failure.** Read the ordering in
`SafetyTask::run()`: the predicate is evaluated, `latch_error_()` opens the
relays and drops `AMS_OK`, and *then* telemetry is encoded and sent. The
frames report state that has already been acted on locally. A dropped frame
costs the ECU and the dash up to 500 ms of blindness on a fault transition —
an observability gap, never a control one.

Two mitigations already exist. `DiagTxReservedSlots` = 6 keeps six of the
sixteen FIFO slots free for the flight telemetry matrix while a multi-minute
log pull is running — without it, the diag stream filled the FIFO and the
flight matrix was dropped for the whole transfer with no evidence but a
best-effort counter. And only the pit-diag burst uses the blocking send
variant; the flight matrix stays non-blocking so a transient FIFO bump never
stalls the 50/100/250 ms cadences.

### CAN-2 — RX queue drops are counted but not acted on · **Latent**

The FDCAN1 RX ISR drains the hardware FIFO into `acu_rx_queue` with a
non-blocking put and bumps `g_acu_rx_isr_drop` on a full queue. No predicate
reads that counter.

**Effect.** Dropped frames age out through the normal freshness paths: a lost
`0x100` shows up as `VcuStale` after 200 ms, a lost `0x101` as
`ChargerStale`. Saturation is transient — `AcuCanTask` runs at `AboveNormal`
and drains promptly — so recovery is automatic. The realistic cost is
operability (a charger-mode lock missed during the arming window), not
safety.

### CAN-3 — Extended frames are rejected in hardware only · **Latent**

`App_InitTask` configures the FDCAN1 global filter to `FDCAN_REJECT` extended
frames, so nothing 29-bit reaches the queue. `VehicleService::update_from_frame`
checks `f.bus` but does **not** check `f.extended`.

**Effect.** None today — the hardware gate is correct and everything the AMS
listens for fits in 11 bits. It is a defence-in-depth gap: a filter
misconfiguration, or a CubeMX regeneration that resets the filter, would
silently start feeding extended frames into the ID comparisons.

---

## 10. Fault attribution

Small but it will save you an afternoon on the bench.

`g_fault_reason_telemetry` and `g_fault_detail_telemetry` are written **once**,
on the first transition into the latched state, and surfaced on pit-diag
`0x6C0[6]` and `0x6C0[7]`. They record the branch that latched, not the most
recent condition — which is what you want for post-mortem and is easy to
misread as live status.

`FaultReason` values are a **stable wire contract: append only, never
renumber.** Value 12 (`FsmError`) has no enum slot; it is a bare constant
because the FSM-driven Error path (precharge timeout, input drop) is
attributed by `fsm_error_reason()`, which distinguishes the generic case from
`ChargerTsmsOpen` (15).

The detail byte carries the offending module index for cell-range faults, the
live `module_online_mask` for `BmsModuleOffline`, the module index for
`BmsStale`, and the offending-module mask for `TempSensorDisconnected` and
`CellOpenWire`. `0xFF` means no module matched — see CONCURRENCY-1.

---

## 11. What is not covered here

Honest limits of this register:

- **No hardware FMEA.** Contactor, fuse, IMD, HVD, cell and busbar failure
  modes are out of scope. This document covers what the *firmware* does and
  fails to do.
- **No HIL evidence.** Every entry was verified by reading source. Where a
  claim rests on bench measurement, it says so and the number comes from a
  comment in the source, not from a run performed for this document.
- **No coverage claim.** "Detected by predicate X" means the code path
  exists and was read. It does not mean it has been exercised against the
  real failure on real hardware. Where a path is explicitly unvalidated —
  open-wire endpoints, the ADG731 temperature path, the ECU discharge half —
  the entry says so.
