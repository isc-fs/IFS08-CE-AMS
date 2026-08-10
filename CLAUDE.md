# CLAUDE.md

Guidance for Claude Code (and a fast orientation for humans) working in
this repo. For the full story, humans should read
[`docs/ONBOARDING.md`](docs/ONBOARDING.md).

## What this is

Safety-critical firmware for the **Accumulator Management System (AMS)** of
the IFS08 Formula Student EV — STM32H733ZG, FreeRTOS (CMSIS-RTOS v2),
C++17 application code on CubeMX-generated HAL/RTOS C. It monitors a 95S6P
pack — 5 modules × 19 cells = 95 cells, × 40 NTC channels = 200 temps, read
over an isoSPI LTC6811-1 chain of 10 ICs (2 per module) — drives the AIRs and
the precharge contactor, and holds the shutdown circuit closed via `AMS_OK`.

**Fault response is not one tick.** The 10 ms loop evaluates the predicate
set and drives the relays inline, but the cell voltage/temperature branches
are debounced (`CellFaultConfirmTicks` = 25 ≈ 250 ms) on top of a 200 ms
voltage poll — worst case ≈ 460 ms, sized against the < 500 ms budget.
`BmsStale` is debounced too (`BmsStaleConfirmTicks` = 25 ≈ 250 ms).
Immediate-danger predicates (BMS module offline, current over-limit,
VCU stale) latch on the first tick. If you change either number, redo that
arithmetic; the comments in `ams_config.hpp` carry it.

## Build & test commands

```bash
# Host unit + SIL tests (fast — run these for any logic change).
cmake -B build-tests -S tests/unit && cmake --build build-tests && ctest --test-dir build-tests --output-on-failure
```

`ctest` reports `1/1 Test ... Passed` — that is the single Unity *runner*, not
the case count. Run `./build-tests/ams_unit_tests` directly for the real
total: it ends `476 Tests 0 Failures 0 Ignored`.

```bash
# Cross-compile the firmware.
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake && cmake --build build
python3 scripts/check_flash_layout.py build/AMS.elf   # expect "PASS: flash layout compatible with stm32-can-bootloader"

# Regenerate the CAN database after any wire-format change (CI's "DBC matches code" check enforces it).
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump > docs/dbc/ams.dbc
```

The image links at **0x08020000**, not at the reset vector: sector 0
(0x08000000–0x0801FFFF) is the CAN bootloader's and sector 7 is its NVM.
That is what `check_flash_layout.py` guards — an image that grows into either
region bricks the update path, and CI runs the check on every build.

There is no CubeIDE/Eclipse makefile — CMake end-to-end. The bench build
adds `-DAMS_HIL_CLEAR_ERROR_LATCH=ON`, which wipes the sticky `ErrorLatch`
on every boot; it must never reach a flight image (see `docs/HIL_BUILD.md`).

## Git & PR conventions

- **Branch off `dev`**; never commit directly to `dev` or `main`. Name
  branches `feat/<short-slug>` or `fix/<short-slug>` — recent branches on
  `dev` are slug-only. `branch-issue.yml` opens the tracking issue from the
  branch name; it `parseInt`s the second field looking for a
  `feat/<n>-<slug>` counter, so a slug-only branch parses to `NaN` and the
  auto-opened issue carries a numbering warning. That warning is expected
  noise, not a problem to fix (`CONTRIBUTING.md`).
- **Always open PRs with `gh pr create --base dev`.** The default branch is
  `main`, so omitting `--base dev` mis-targets a release branch. `dev → main`
  is a *release* only, gated on the HIL acceptance plan.
- **`Closes #N`** goes in the **PR body only** (parsed by
  `close-on-dev-merge.yml`), never in commit messages.
- **Do NOT add a `Co-Authored-By: Claude` trailer** to commits in this repo.
- **Never `git add -A`** — the working tree normally carries untracked
  scratch files and build directories. Stage files explicitly.
- Commit messages: imperative mood, first line < 72 chars.

## Comment style

Comments explain the code **as it is now**, to someone who has never seen this
repo. History lives in `git log`, PRs and `docs/` — not in the source.

**Do not write:**

- Issue or PR numbers (`(#279)`, `see #423`). A reader cannot look those up
  offline, and they almost never carry information the sentence needs.
- Dates (`BENCH-VERIFIED 2026-07-22`) or versions (`live since v2.1.0`). Keep
  the *fact* — "bench-verified on the real pack" — and drop the timestamp.
- Change narrative: `was 2`, `30 -> 25`, `Until this landed`, `The previous
  version`, `fixed in`. If the old behaviour matters, describe the current rule
  and why; otherwise `git blame` has it.

**Do write:**

- Physical and numerical reasoning: `47R || 47R = 23.5 ohm -> 164 mA at 3.85 V`
- Safety contracts: *"SAFETY CONTRACT: SoC is TELEMETRY ONLY. No safety
  predicate reads it"* (`soc_estimator.hpp` opens on this — SoC never
  reaches the FSM)
- Non-obvious invariants and conventions: the LTC cell split (`LTC_1` carries
  module cells 0..8, `LTC_2` carries 9..18 — `CellsPerLtcUpper` = 9,
  `CellsPerLtcLower` = 10), `+ current = discharge`, single-writer ownership
- Datasheet pointers a reader can actually follow: *"LTC6811 Table 53"*
- Honest gaps: what is measured, what is assumed, what is untested

Exceptions where a number IS the information: CAN message IDs, register
addresses, `COMMISSION` markers, and `docs/` pages whose job is project history.

Rule of thumb: if a comment answers *when* something changed or *who* asked for
it, cut it. If it answers *what this does* or *why it must be this way*, keep it.

## Hard constraints

- **No references to SWD / JTAG / OpenOCD / ST-Link / GDB / CubeProgrammer**
  in live docs or source. Flashing is described via the CAN bootloader flow.
- **Never hand-edit `Core/Src/main.c` / `freertos.c`** (CubeMX-owned,
  regenerated from `AMS.ioc`). App logic lives in `Core/{Inc,Src}/app/` and
  is invoked through `extern "C"` `ams_*_task_run` trampolines. Anything you
  do put in a CubeMX file must sit inside a `USER CODE BEGIN/END` block.
- **Never hand-edit `docs/dbc/ams.dbc` or `ROADMAP.md`** — both are
  generated. The DBC comes from the **code-first CAN DSL**
  (`Core/Inc/can/messages/*.def` → `tools/dbc_dump.cpp`); regenerate with the
  command above. `ROADMAP.md` ← `.github/roadmap.yaml`.

## Safety-critical surface (higher review bar)

Changes to any of these need the `safety-critical` label, **two approving
reviews** from team members familiar with FS safety rules — not the one an
ordinary PR needs — and an explicit confirmation in the PR that
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1 *Safety invariants* still
hold. No branch-protection rule enforces the second reviewer, so it is on the
author to go and ask for one (`CONTRIBUTING.md`):

- `Core/Src/app/safety_task.cpp` — `MainTask` (safety + FSM + relays + telemetry, 10 ms)
- `Core/Inc/app/state_machine.hpp` — the pure FSM
- `Core/Inc/app/safety_predicates.hpp` — the fault predicate set
- `Core/Inc/app/relay_driver.hpp` + `Core/Src/app/relay_driver.cpp` — contactors + `AMS_OK`
- `Core/Inc/app/ams_config.hpp` — thresholds / timings (many tagged `COMMISSION`)

## Architecture in 10 lines

- **One realtime task**, `MainTask` (the CubeMX thread is still named
  `SafetyTask` in `main.c`/`AMS.ioc`), 10 ms: snapshot services → evaluate
  (debounced) fault predicates → step the FSM every 20 ms → drive contactors
  and `AMS_OK` inline → emit telemetry @500 ms → refresh the IWDG. It is the
  only `osPriorityRealtime` thread, so no producer can preempt it.
- **Lock-free single-writer services** — `BmsService` (BmsPollTask),
  `CurrentService` (CurrentSensorTask), `VehicleService` (AcuCanTask).
  CubeMX still declares `bms_mutex`/`current_mutex`/`vehicle_mutex` in
  `main.c`, but no app code takes them: 32-bit atomic access + a single-writer
  contract instead.
- **Pure-logic core** (FSM, predicates, SoC, balancing policy, CAN/LTC
  encoders) is HAL-free → the host tests need no hardware or RTOS mocks.
- **FSM:** Start → Precharge → Transition → {Run | Charge}, plus sticky Error.
  A TSMS drop de-energises back to **Start without latching** (the driver must
  be able to stop and restart unaided) — except in Charger mode, where
  scrutineering forbids re-activating the charge output, so it latches Error.
  Run also falls back to Start on a debounced DC-bus collapse (the AIRs were
  opened externally). Mode (Car/Charger) is locked at `Start → Precharge` and
  **cleared on any return to Start**, so a re-arm re-locks and re-precharges.
- **Inputs:** TSMS (PF9, held level) + DASH_CHG (PF10, momentary **edge**).
  Start→Precharge needs TSMS held + a DASH_CHG press + `fsm::rearm_permitted`.
  Run/Charge are sustained by **TSMS alone** (releasing DASH_CHG does not
  fault). Charger precharge proceeds on a still-fresh magic-gated `0x101`
  charge request; Car on `dc_bus_V ≥ 95 %` of the cell-sum pack voltage
  **and** a fresh `0x100`. Freshness is part of that criterion, not a separate
  fault: `VehicleState` keeps the *last received* `dc_bus_V`, so a dead VCU
  freezes it — frozen at pack voltage it satisfies 95 % forever and would
  close AIR+ onto an already-bled link.
- **DC-link discharge interlock.** The bleed relay is normally-closed and
  wired into the SDC with no software control: opening the shutdown circuit
  connects the bleed and the link drains, but closing the SDC again
  re-energises the relay and the discharge **stops part-way**, stranding the
  link at an unpredictable voltage. The AMS cannot restart it — its own leg of
  the loop (`AMS_OK`) latches in hardware and cannot be pulsed low from
  firmware. So the AMS publishes the two facts only it can see —
  `0x021 ACU_discharge_interlock` (`fsm_in_start`, `tsms`) — and the ECU, which
  owns both a DC-link measurement and a normally-closed relay in series with
  the bleed relay's coil, decides. The AMS consumes `0x100` byte 2 bit 0
  (`discharge_engaged`) and refuses to leave Start while it is set
  (`fsm::rearm_permitted`), holding in Start rather than latching.
  **The ECU half is implemented** on `IFS08-CE-ECU` `dev` (`discharge.hpp`):
  it consumes `0x021`, adds its own `dc_bus` term, latches the hold and
  releases on its own measurement, and sends `0x100` at DLC 3 — so
  `ecu_discharge_capable` latches and both re-arm blocks are live. **Gap: the
  pairing has never run end to end.** The AMS side is verified only by
  `tests/unit/test_state_machine.cpp` and the ECU side only by its SIL. ECU
  `main` predates the feature and sends DLC 2, where the bit reads 0 and the
  AMS behaves as it did before.
- **Pins:** AIR− PB6, AIR+ PB5, precharge PB7, `AMS_OK` PB4 (active-high,
  HIGH = AMS not blocking the SDC); pack current ADC3 differential PF7/PF8
  (INP3/INN3), DCDC current ADC3 single-ended PC1 (INP11).
- **ErrorLatch** is sticky across resets (RTC backup register), so a latched
  fault survives a power-cycle and the board boots straight into Error.

## Where to look

| Need | File |
|---|---|
| Onboarding / reading order | [`docs/ONBOARDING.md`](docs/ONBOARDING.md) |
| Term definitions | [`docs/GLOSSARY.md`](docs/GLOSSARY.md) |
| As-built reference | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| Safety FSM, gate by gate | [`docs/FSM_OVERVIEW.md`](docs/FSM_OVERVIEW.md) |
| Full codebase walkthrough | [`docs/DEEP_DIVE.md`](docs/DEEP_DIVE.md) |
| CAN wire protocol | [`docs/CAN_MAP.md`](docs/CAN_MAP.md) |
| isoSPI / LTC6811 / balancing | [`docs/BMS_LTC6811.md`](docs/BMS_LTC6811.md) |
| Failure modes + open risks | [`docs/FMEA.md`](docs/FMEA.md) |
| Bench calibration / build flag | [`docs/COMMISSIONING.md`](docs/COMMISSIONING.md), [`docs/HIL_BUILD.md`](docs/HIL_BUILD.md) |
| Contribution conventions | [`CONTRIBUTING.md`](CONTRIBUTING.md) |
| All tunable constants | [`Core/Inc/app/ams_config.hpp`](Core/Inc/app/ams_config.hpp) |
