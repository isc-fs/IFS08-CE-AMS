# Onboarding — IFS08-CE-AMS

Welcome to the Accumulator Management System firmware. This is the
**Day-1 guide**: it gets you from zero to a working toolchain, gives you
the mental model, and sequences the rest of the docs so you don't have to
guess the reading order. Budget ~2 hours to go from here to "can read a
PR." Keep [`GLOSSARY.md`](GLOSSARY.md) open in another tab — it defines
every acronym below.

---

## 0. Before anything else: this pack can kill you

The accumulator is **95 cells in series**
([`ams_config.hpp`](../Core/Inc/app/ams_config.hpp): `BmsModuleCount = 5`,
`CellsPerModule = 19`). At the 4200 mV over-voltage limit that is
**~400 V DC** fully charged, ~350 V nominal, behind roughly 18 Ah of
usable capacity — about 6 kWh with no fuse fast enough to save you.

Three things follow, and none of them are negotiable:

- **DC at these voltages does not let go.** There is no zero crossing.
  Treat every conductor downstream of the AIRs as live until *you*
  measured it dead, with your own meter, on the actual conductor.
- **A discharged DC-link is not a safe DC-link.** Opening the shutdown
  circuit starts the bleed; closing it again *stops* the bleed part-way
  and strands whatever charge is left. See §5 — the firmware has a whole
  interlock about this, and the interlock is currently inert.
- **The firmware is not your personal protective equipment.** Nothing in
  this repo is a substitute for the accumulator's isolation procedure.
  Never work on live HV alone.

> **Safety-critical firmware.** This code opens and closes the contactors
> on that pack. Read [`ARCHITECTURE.md`](ARCHITECTURE.md) §1 (safety
> invariants) before you change anything in the safety path — and when in
> doubt, ask a reviewer. A wrong line here is not a bug report, it is an
> incident.

---

## 1. What the AMS actually does

The AMS is the brain of the high-voltage battery. In one breath: it
**watches every cell**, **drives the contactors**, and **kills the pack**
the instant anything goes out of bounds.

- **Watches** — 95 cell voltages + 200 NTC temperatures, read over isoSPI
  from a daisy-chain of 10 LTC6811-1 monitors (5 modules × 2 ICs) driven
  through an LTC6820 bridge on SPI1; plus pack current from a Bourns
  SSA-2-250A sensor read *differentially* on ADC3 (PF7/PF8), and DCDC
  current single-ended on PC1.
- **Drives** — three contactors — AIR− (PB6), AIR+ (PB5), precharge (PB7)
  — and the `AMS_OK` leg of the shutdown circuit (SDC) on PB4. All
  active-high; CubeMX writes them LOW *before* configuring them as
  outputs, so the pack is isolated for the whole reset-to-first-task
  window ([`relay_driver.hpp`](../Core/Inc/app/relay_driver.hpp)).
- **Kills** — a 10 ms supervisor evaluates the fault predicates in
  [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp) and,
  on a confirmed breach, opens every contactor, drops `AMS_OK`, and
  latches `Error`.

### The fault-response contract — read this carefully

The invariant people quote is "AIRs open within 10 ms." That is the
**supervisor's own** budget, not the wall-clock detection time. `MainTask`
runs every 10 ms and drives the GPIOs *inline in the same iteration* that
evaluated the predicate — no queue, no second task — so once the bad
number is in a service, the contactors move within one tick. What the
data cost to *get* there is a separate number, and it is the one that
matters for the Formula Student "< 500 ms" rules:

| Fault | Debounced? | Worst case, sensor → AIRs open |
|---|---|---|
| VCU stale, charger stale, current stale, current sensor fault, cell open-wire | no — first tick | producer period + 10 ms |
| A whole module goes silent | no — `BmsModuleOffline` fires the moment freshness exceeds `BmsStaleMs` at a poll | ~410 ms (350 ms + the poll that observes it + a tick) |
| Cell under/over-voltage | yes, `CellFaultConfirmTicks` = 25 | ~460 ms (200 ms poll + 250 ms confirm + tick) |
| Disconnected temp sensor | no, `TempDisconnectPolls` = 1 | ~360 ms (250 ms sweep + ~100 ms + tick) |
| Pack over-current | no debounce — but the input is IIR-filtered, τ ≈ 800 ms | 250 A → ~1.1 s; 400 A → ~0.5 s |

Two honest consequences a newcomer must internalise:

1. **Cell range faults are deliberately slow.** The debounce exists
   because the `BmsState` snapshot can be read mid-update (see §3) and a
   single torn read must never latch a sticky `Error`. A cell cannot
   leave its window for one 10 ms tick and come back, so ~250 ms of
   confirmation costs nothing real.
2. **Over-current does not trip fast, by design.** The filter is the
   only smoothing, and `CurrentMaxMa = 185000` is the cells' 6P
   continuous rating. Currents *between* the cell rating and that limit
   never trip at all — slow-overload protection is the temperature path,
   not this one. Read the comment on `CurrentMaxMa`; it does the
   arithmetic for you.

### Two contexts, decided once per power cycle

| Context | Locked when | What it means |
|---|---|---|
| **Car** | anything that is not Charger — including a car with a dead VCU | Precharge through the resistor; completes on `dc_bus_V ≥ 95 %` of pack **and** a fresh `0x100` |
| **Charger** | a fresh operator `0x101` "CHRG" request **and** VCU `0x100` silent | **Skips the resistor** — closes only AIR− on entry, AIR+ on the proceed; proceeds while `0x101` is still fresh |

Charger mode is deliberately hard to enter: it needs a *positive*
assertion (`0x101`), not merely VCU absence. A car whose VCU died sends
no `0x101`, so it locks **Car** and faults on `VcuStale` instead of
silently charging (`safety_task.cpp` mode lock,
`state_machine.hpp` `Mode`).

Charger also skips the precharge contactor because that contactor sits in
**parallel with AIR+**: closing it while the charger is sourcing current
would route the full charge current through a transient-rated resistor.
The charger voltage-matches its own output before asserting `0x101`, so
there is no inrush to limit.

The two operator inputs are physical: **TSMS** (PF9) is a *held* level;
**DASH_CHG** (PF10) is a *momentary press*, edge-detected and latched by
`MainTask` until the 20 ms FSM step consumes it. `Run`/`Charge` are
sustained by **TSMS alone** — releasing DASH_CHG does nothing, and
level-checking it would fault `Run` instantly.

**PF9 is more than a switch.** It reads the shutdown circuit as a whole:
any open SDC element pulls it low. So `tsms == 1` also means "the
discharge relay is energised and the bleed is disconnected" — which is
exactly why it is published on `0x021` (see §5).

A TSMS drop is a **normal operator stop, not a fault**: `Run` → `Start`,
contactors open, nothing latched, and the driver re-arms with another
press. That is load-bearing for the FS rule that a driver must be able to
stop and restart the tractive system unaided — and it works only because
`AMS_OK` is health-only. `AMS_OK` sits *upstream* of TSMS in the loop, so
if a TSMS drop latched `Error` it would drop `AMS_OK`, open the upstream
relay, and re-closing TSMS could no longer restore the loop. **Charger
mode is the exception**: there, a TSMS drop *does* latch `Error`, because
the scrutineering sheet forbids re-activating a charger output once the
SDC has opened.

For the full picture: [`ARCHITECTURE.md`](ARCHITECTURE.md) is the
as-built reference, [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md) is the
gate-by-gate state machine, and [`DEEP_DIVE.md`](DEEP_DIVE.md) walks the
whole codebase.

---

## 2. Set up your toolchain

You need two toolchains: one to **cross-compile** the firmware for the
STM32, and the host compiler to **run the unit tests** (which is the
fastest way to confirm your setup works).

**Prerequisites**

| Tool | Version | Notes |
|---|---|---|
| `arm-none-eabi-gcc` | 14.x | The cross-compiler (CI pins 14.2.Rel1). macOS: `brew install --cask gcc-arm-embedded`. Linux: distro package or the Arm GNU Toolchain tarball. |
| CMake | ≥ 3.22 | `brew install cmake` / distro package. |
| Ninja *(optional)* | any | Faster than Make; used by the CMake presets. CI uses Makefiles. |
| A C++17 host compiler | Clang (macOS) / GCC (Linux) | Only needed for the host tests. |
| Network, first configure only | — | `tests/unit/CMakeLists.txt` `FetchContent`s Unity v2.6.1 from GitHub. |

**Clone and verify** — a fresh clone leaves you on `main` (the release
branch). Switch to `dev` immediately; all work branches off `dev`.

```bash
git clone https://github.com/isc-fs/IFS08-CE-AMS.git
cd IFS08-CE-AMS
git checkout dev

# 1) Host unit tests — the fastest "is my setup sane?" check.
cmake -B build-tests -S tests/unit
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
#   → the Unity runner prints "<N> Tests 0 Failures 0 Ignored / OK".
#     Any non-zero failure count means stop and fix, not "probably fine".

# 2) Cross-compile the firmware.
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
arm-none-eabi-size build/AMS.elf                            # flash/RAM footprint
python3 scripts/check_flash_layout.py build/AMS.elf         # sector-0 / overflow guard

# 3) Only if you touched Core/Inc/can/messages/*.def — regenerate the DBC.
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump
/tmp/dbc_dump > docs/dbc/ams.dbc
```

CI (`.github/workflows/build-tests.yml`) runs **three** jobs on every
push to a `feat/**` or `fix/**` branch and every PR into `dev`: the
cross-compile (plus the flash-layout guard), the host tests, and a
**"DBC matches code"** job that regenerates `docs/dbc/ams.dbc` and
`diff`s it against the committed file. That third one is the one people
forget — change a `.def`, don't regenerate, and CI goes red.

`scripts/check_flash_layout.py` is not cosmetic: the app must start
exactly at `0x08020000` and stay out of sector 0 (bootloader) and sector
7 (BL NVM), or the CAN bootloader will jump into nothing.

> **Bench build.** The HIL rig uses a third build with
> `-DAMS_HIL_CLEAR_ERROR_LATCH=ON`, which wipes the sticky error latch on
> boot so a faulted bench session restarts clean. It must **never** reach
> a flight build — full detail in [`HIL_BUILD.md`](HIL_BUILD.md).

---

## 3. The mental model (read this before the code)

The firmware is one **realtime supervisor task** plus a few lower-priority
producers feeding it data through lock-free services:

```
BmsPollTask       ─┐  (isoSPI: 95 cells @200 ms, 200 temps @250 ms, balancing)
CurrentSensorTask ─┤→  single-writer services  →  MainTask (10 ms)
AcuCanTask        ─┘  (FDCAN1 RX: 0x100 VCU, 0x101/0x103/0x104        │
                       operator, 0x002 boot trigger, 0x7F0 pit-diag)  ▼
                                     evaluate fault predicates → step FSM (20 ms)
                                     → drive contactors + AMS_OK inline
                                     → telemetry + SD log + IWDG refresh
```

`SdLoggerTask` (low priority, microSD) and `App_InitTask` (runs once)
round out the thread set. AcuCanTask also owns all periodic TX to the
ECU on its own 50/100/250 ms deadlines.

Three ideas do most of the work:

1. **One safety timeline.** `MainTask` (the CubeMX thread is still named
   `SafetyTask`) is the only realtime-priority task. It snapshots the
   services, runs the predicates, steps the FSM, and drives the relays —
   all in one timeline, so there is no cross-task race in the safety
   path, and no event-flag hand-off to lose. Producers run at strictly
   lower priority and cannot preempt it.
2. **Lock-free single-writer services.** `BmsService` (written by
   BmsPollTask), `CurrentService` (CurrentSensorTask), `VehicleService`
   (AcuCanTask). Each has exactly one writer and many readers; no mutex
   is taken. **Do not read this as "reads are atomic."** Individual
   32-bit aligned words are atomic on the Cortex-M7, but
   `BmsService::snapshot()` returns a copy of a ~620-byte struct while
   the writer may be part-way through updating it, so a reader *can*
   observe a mid-update mix — a torn read. The design
   tolerates it deliberately: telemetry only ever looks stale, and the
   safety path is protected by the cell V/T debounce and the
   `first_full_poll_done` gate. If you add a predicate that cannot
   survive a torn read, that is your problem to solve, not the
   service's.
3. **Pure-logic core.** The FSM
   ([`state_machine.hpp`](../Core/Inc/app/state_machine.hpp)), the fault
   predicates
   ([`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp)),
   the balancing policy, the SoC estimator, and the CAN/LTC codecs are
   HAL-free and RTOS-free pure functions — which is why the whole host
   suite runs with no hardware and almost no mocking. If you find
   yourself needing a HAL call in one of those headers, you are putting
   it in the wrong file.

**The first 2 seconds are different.** `SafetyBootGraceMs` = 2000 ms after
`osKernelStart`, the freshness / data-presence predicates are suppressed
and `AMS_OK` is held **LOW**. Both halves matter. Without the suppression
the very first supervisor iteration would fault (every service's
`last_*_tick` is still 0), withhold the watchdog refresh, and IWDG-reset
the chip in ~100 ms — before BmsPollTask has polled once. And `AMS_OK`
must stay low while the predicates are disarmed, because enabling the SDC
against inputs nobody has checked is exactly the wrong failure. So on the
bench, "AMS_OK is low for two seconds after reset" is correct behaviour,
not a fault. A sticky `ErrorLatch` from a previous boot is *not*
suppressed by the grace.

**Sticky `Error`, and its one hole.** A latch writes a magic word to RTC
backup register `BKP1R`, so `Error` survives every *warm* reset —
software, IWDG, reset pin. It does **not** survive a full LV power-cycle:
this carrier has no VBAT, so the backup domain dies with VDD. That is
accepted by design (a deliberate power-cycle *is* the manual reset, and a
persistent fault re-latches on the next post-grace evaluation) but never
assume a fault is still recorded after someone cycled the LV switch. See
[`error_latch.hpp`](../Core/Inc/app/error_latch.hpp).

---

## 4. Which files are the ground truth

When a doc and the code disagree, the code wins — but even inside the
code some files are authoritative and others are downstream:

| Question | Authoritative file |
|---|---|
| Every threshold, period, CAN ID, calibration constant | [`ams_config.hpp`](../Core/Inc/app/ams_config.hpp) — and its comments carry the physical reasoning |
| What counts as a fault | [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp) |
| What each state may do | [`state_machine.hpp`](../Core/Inc/app/state_machine.hpp) |
| How the two are wired to hardware and to time | [`safety_task.cpp`](../Core/Src/app/safety_task.cpp) |
| CAN wire format (bit layouts, DLCs, cycle times) | `Core/Inc/can/messages/*.def` — the code-first DSL |
| Pin assignment | `Core/Inc/main.h` (CubeMX-generated from `AMS.ioc`) |

**Generated — never hand-edit:** `docs/dbc/ams.dbc` (from the `.def`
files via `tools/dbc_dump.cpp`) and `ROADMAP.md` (from
`.github/roadmap.yaml`).

**CubeMX-owned — never hand-edit:** `Core/Src/main.c`,
`Core/Src/freertos.c`, `Core/Src/stm32h7xx_*.c`, `Drivers/`,
`Middlewares/`. They are regenerated from `AMS.ioc` and your edits will
vanish. Application logic lives in `Core/{Inc,Src}/app/` and is reached
through `extern "C"` `ams_*_task_run` trampolines that the CubeMX
`USER CODE BEGIN … END` blocks call. See
[`ARCHITECTURE.md`](ARCHITECTURE.md) §2.

Anything needing on-vehicle calibration is tagged `COMMISSION` in
`ams_config.hpp`. Those are **placeholder defaults**, not measured
values — treat an un-ticked `COMMISSION` constant as an open safety
question, and see [`COMMISSIONING.md`](COMMISSIONING.md) for the
procedure and [`COMMISSIONING_CHECKLIST.md`](COMMISSIONING_CHECKLIST.md)
for the sign-off sheet.

---

## 5. Five things that surprise people

These are the areas where the obvious reading of the code is wrong. Each
one exists because something bit us.

**1. `precharge_target_reached` requires freshness, not just voltage.**
`VehicleState` holds the *last received* `dc_bus_V`. When the VCU stops
publishing `0x100`, that number does not disappear — it **freezes**.
Frozen at pack voltage it satisfies the 95 % criterion forever, including
long after the link has actually bled to zero, and closing AIR+ then puts
full pack voltage across the contactor with nothing limiting the inrush.
`VcuStale` cannot save you here: it needs 200 ms while the FSM steps
every 20 ms, and it is gated on `vcu_required`, which is false in `Start`.
So freshness is *part of the criterion*, not a fault racing it
(`state_machine.hpp`, `precharge_target_reached`).

**2. The DC-link discharge interlock — present, and currently inert.**
Opening the SDC de-energises a normally-closed discharge relay, so a
bleed resistor connects and the link drains. Closing the SDC again
re-energises the relay and the discharge **stops part-way**, stranding the
link. The AMS cannot fix that itself: the discharge relay has no software
control, and the AMS's own leg of the loop (`AMS_OK`) latches in
hardware, so it can never be pulsed low. The ECU can fix it, with a
normally-closed relay in series with the discharge relay coil — but the
ECU cannot see whether the AMS is in `Start` or whether the SDC is
closed. So the AMS publishes those two facts on `0x021`
(`fsm_in_start`, `tsms`; `acu_discharge_interlock.def`) and consumes the
ECU's answer from `0x100` byte 2 bit 0 (`discharge_engaged`).
`fsm::rearm_permitted` gates `Start → Precharge` on it, and `Charger` is
exempt (the inverter is not in the charge loop, and `dc_bus_V` is
VCU-only, so gating it would make Charger unarmable).

**Gap, stated plainly: both halves exist and have never met.** The AMS side
is complete and unit-tested (`test_state_machine.cpp`). The deciding logic
and the series relay live in the ECU firmware, in a different repo, and are
implemented on its `dev` branch — but no bench has ever had both boards on
one bus, so the pairing is verified by each side's assumptions about the
other and nothing else.

Byte 2 of `0x100` is optional on the wire, and that is what decides whether
you have the protection at all: against an ECU sending DLC 2 (its `main`
still does), `vehicle_service.cpp` leaves `ecu_discharge_capable` and
`discharge_engaged` false, `rearm_permitted` returns true unconditionally,
and the interlock does nothing. That fallback is deliberate — refusing to
arm over a link the other end has no way to drain would brick the car.
**Watch the DLC of `0x100` on the bus to know which case you are in;
DLC 2 means inert.**

**3. Cell voltages must be measured with balancing OFF.** Setting the
ADCV `DCP=0` bit is **not** enough on BMS_LITE: the bleed does not go
through the LTC6811's own switch but through an external TSM2323 PMOS
whose gate has a ~100 µs RC, comparable to the first conversion. And the
bleed current returns through the *harness*, not the Kelvin sense path —
179 mA across 50–200 mΩ of tap impedance is 9–36 mV, with opposite sign
on the bled cell and its neighbours. Against a 50 mV balancing threshold
that is a first-order corruption of the exact signal the balancer selects
on. Hence `BalanceQuiesceMs`: clear the DCC bits, wait 2 ms, *then*
convert (`ams_config.hpp`, `bms_poll_task.cpp::quiesce_balancing`).

**4. The LTC cell split is 9/10, not 10/9 or 12/7.** Each module is two
LTC6811-1s. The **first** IC in the chain (even chain slot) carries
module cells **0..8** (`CellsPerLtcUpper = 9`); the **second** (odd slot)
carries **9..18** (`CellsPerLtcLower = 10`), with its tenth cell arriving
in the RDCVD group. The 9-cell IC's RDCVD group is discarded. Getting
this backwards zeroes cell 9 and silently drops cell 18
(`bms_service.cpp`, the commit block in the voltage decoder). The same
split defines physical adjacency for the balancer's
"never bleed two neighbours at once" rule.

**5. SoC is real, and is telemetry only.** `soc_estimator.hpp` implements
Coulomb counting with OCV anchoring at rest plus an EKF that corrects
continuously from the voltage residual (the Kalman gain leans on voltage
where the OCV curve is steep and on charge integration across the flat
plateau). `CurrentSensorTask` runs it; it is published on `0x130`. Its
safety contract is absolute: **no predicate reads it, and nothing it
produces can move the FSM, a contactor, or `AMS_OK`.** If the whole file
returned garbage the AMS would behave identically. Keep it that way.

---

## 6. Guided reading order

Read top-to-bottom; each builds on the last.

1. **This file** — you're here.
2. [`GLOSSARY.md`](GLOSSARY.md) — skim it, then refer back constantly.
3. [`ARCHITECTURE.md`](ARCHITECTURE.md) **§1 (safety invariants)** and
   **§3 (task architecture)** — the non-negotiable rules and what runs
   when.
4. [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md) — the six states, the Car/Charger
   mode lock, and exactly what each transition does. Read it **alongside**
   [`state_machine.hpp`](../Core/Inc/app/state_machine.hpp) with both
   windows open; the header's comments carry the reasoning the doc
   summarises.
5. [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp) and
   [`ams_config.hpp`](../Core/Inc/app/ams_config.hpp) — read the config
   header as prose, not a constants list. Almost every number has a
   paragraph explaining the physics or the rule it came from, and that
   is the densest knowledge transfer in the repo.
6. [`CAN_MAP.md`](CAN_MAP.md) — the wire protocol on FDCAN1 (telemetry,
   VCU `0x100`, operator `0x101`/`0x103`/`0x104`, boot trigger, pit-diag
   stream), then `Core/Inc/can/messages/*.def` and
   [`can_dsl_explained.html`](can_dsl_explained.html) for how the DBC is
   generated from them.
7. [`BMS_LTC6811.md`](BMS_LTC6811.md) — when you touch the isoSPI /
   LTC6811 / balancing / open-wire path.
8. [`DEEP_DIVE.md`](DEEP_DIVE.md) — the full end-to-end walkthrough when
   you want every subsystem mapped to source in one pass.
9. [`FMEA.md`](FMEA.md) — the risk register. Long, but if you are about
   to touch a subsystem, read its rows first: they cite the code and tell
   you what is already known to be weak there.
10. [`COMMISSIONING.md`](COMMISSIONING.md),
    [`COMMISSIONING_CHECKLIST.md`](COMMISSIONING_CHECKLIST.md) and
    [`HIL_BUILD.md`](HIL_BUILD.md) — when you start working on the bench.
11. [`CONTRIBUTING.md`](../CONTRIBUTING.md) — before your first PR.

For the *why* behind this architecture versus the previous bare-metal
firmware, there is a narrative read: `docs/AMS_2025_VS_2026.html`.

---

## 7. Where everything lives

```
Core/Inc/app/       hand-written headers  — ams_config.hpp (ALL tunables),
                    state_machine.hpp, safety_predicates.hpp, *_service.hpp,
                    balance_controller.hpp, soc_estimator.hpp, open_wire.hpp,
                    ltc6811.hpp, ltc6820.hpp, relay_driver.hpp, …
Core/Src/app/       hand-written .cpp     — safety_task.cpp (MainTask body),
                    bms_poll_task.cpp, acu_can_task.cpp, current_task.cpp,
                    sd_logger_task.cpp, …
Core/Inc/can/       code-first CAN DSL    — messages/*.def are the wire-format
                    source of truth; can_dsl.hpp / can_codecs.hpp expand them
Core/Src/main.c     CubeMX-generated      — DO NOT hand-edit; it creates the
Core/Src/freertos.c                         threads and calls our
                                            ams_*_task_run trampolines
Core/Inc/main.h     CubeMX-generated      — pin definitions
tests/unit/         host Unity tests      — test_*.cpp, mocks/, unity_runner.cpp
docs/               you are here          — plus dbc/ams.dbc (GENERATED)
tools/              dbc_dump.cpp (DBC generator),
                    bms_monitor.py (live per-cell isoSPI dashboard over CAN)
scripts/            check_flash_layout.py (CI flash-budget guard)
cmake/              gcc-arm-none-eabi.cmake (cross toolchain file)
.github/            workflows + roadmap.yaml (auto-tracking, CI, roadmap)
AMS.ioc             CubeMX project — the source of everything generated
STM32H733XG_FLASH.ld  app at 0x08020000, sectors 1..6
```

---

## 8. How we work (the short version)

Full detail is in [`README.md`](../README.md) and
[`CONTRIBUTING.md`](../CONTRIBUTING.md); the essentials:

- **Branch off `dev`**, never commit to `dev` or `main` directly.
  Naming: `feat/<n>` or `fix/<n>`, optionally with a slug —
  `feat/3-bms-rx-task`. The two types have independent counters.
- **Pushing a *new* branch auto-opens a tracking issue**
  (`.github/workflows/branch-issue.yml`). The bot picks the expected
  number by scanning **all** issues carrying that type's label — open
  *and* closed — and taking the highest + 1. A mismatched number does not
  block anything; it just adds a warning block to the issue asking you to
  recreate the branch. Note the number is consumed either way, so do not
  push throwaway branches to "see what happens".
- Your **first commit message on the branch** is auto-copied into the
  tracking issue's description — make it a real sentence.
- **PRs target `dev`** (`gh pr create --base dev`; the repo default is
  `main`, so omitting it mis-targets a release). Put `Closes #<issue>` in
  the **PR body only** — a merge workflow parses it there, not in commit
  messages.
- A **`dev → main` PR is a release** and only happens after the HIL
  acceptance gate is green on the same SHA.
- **Touching the safety supervisor, relays, FSM, or a predicate?** Label
  the PR `safety-critical`, list in the PR body every predicate added,
  removed or changed, attach a captured SIL FSM-transition trace as
  evidence, and confirm the `ARCHITECTURE.md` §1 invariants still hold.
  See [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Modifying the safety
  supervisor". **Review count is ambiguous in that file** — its PR
  checklist says one approving review, its safety-supervisor section says
  two, explicitly "not one". Ask a lead which governs before you rely on
  either.
- **Comments and docs describe the code as it is now.** No issue/PR
  numbers, no dates, no version stamps, no "was 2 / fixed in" narrative —
  git has that. Do write the physics, the safety contract, the
  non-obvious invariant, the datasheet pointer, and the honest gap.

---

## 9. First-week checklist

- [ ] Both builds pass locally (host tests + cross-compile), and you know
      which third CI job you can still break without noticing.
- [ ] You can explain, in one sentence each, what `MainTask`, the FSM,
      and the three services do.
- [ ] You can state the fault-response contract in your own words:
      what the 10 ms actually bounds, and why a cell over-voltage takes
      ~460 ms.
- [ ] You can trace `Start → Precharge → Run` and name what closes each
      contactor — and say why the **Charger** path closes a different set
      (hint: [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md), and §1 above).
- [ ] You understand why `Run` is sustained by TSMS and not DASH_CHG, and
      why a TSMS drop must not latch `Error` in Car mode but must in
      Charger mode.
- [ ] You've read `ams_config.hpp` end to end, know what `COMMISSION`
      means, and can name three constants that are still placeholders.
- [ ] You've read `.github/workflows/branch-issue.yml` and can predict
      what happens when you push `feat/<n>`.

When the boxes are checked, pick up a `good first issue` or ask a lead
what's unblocked. Welcome aboard.
