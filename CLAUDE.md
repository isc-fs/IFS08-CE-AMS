# CLAUDE.md

Guidance for Claude Code (and a fast orientation for humans) working in
this repo. For the full story, humans should read
[`docs/ONBOARDING.md`](docs/ONBOARDING.md).

## What this is

Safety-critical firmware for the **Accumulator Management System (AMS)** of
the IFS08 Formula Student EV — STM32H733ZG, FreeRTOS (CMSIS-RTOS v2),
C++17 application code on CubeMX-generated HAL/RTOS C. It monitors a ~350 V
pack (95 cells / 200 temps over isoSPI), drives the contactors, and opens
them within 10 ms of any fault.

## Build & test commands

```bash
# Host unit + SIL tests (fast — run these for any logic change). Expect "182 Tests 0 Failures".
cmake -B build-tests -S tests/unit && cmake --build build-tests && ctest --test-dir build-tests --output-on-failure

# Cross-compile the firmware.
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake && cmake --build build
scripts/check_flash_layout.py build/AMS.elf      # sector-0 / overflow guard (CI runs this)

# Regenerate the CAN database after any wire-format change (CI's "DBC matches generator" check enforces it).
python3 tools/gen_dbc.py > docs/dbc/ams.dbc
```

There is no CubeIDE/Eclipse makefile — CMake end-to-end. The bench build
adds `-DAMS_HIL_CLEAR_ERROR_LATCH=ON` (never in a flight image).

## Git & PR conventions

- **Branch off `dev`**; never commit directly to `dev` or `main`. Name
  branches `feat/<n>-<short-title>` or `fix/<n>-<short-title>` (independent
  counters; next = last closed issue of that type + 1).
- **Always open PRs with `gh pr create --base dev`.** The default branch is
  `main`, so omitting `--base dev` mis-targets a release branch. `dev → main`
  is a *release* only, gated on the HIL acceptance plan
  ([issue #317](https://github.com/isc-fs/IFS08-CE-AMS/issues/317)).
- **`Closes #N`** goes in the **PR body only** (parsed by
  `close-on-dev-merge.yml`), never in commit messages.
- **Do NOT add a `Co-Authored-By: Claude` trailer** to commits in this repo.
- **Never `git add -A`** — it pulls in the untracked `can-spec/` submodule
  directory. Stage files explicitly.
- Commit messages: imperative mood, first line < 72 chars.

## Hard constraints

- **No references to SWD / JTAG / OpenOCD / ST-Link / GDB / CubeProgrammer**
  in live docs or source. Flashing is described via the CAN bootloader flow.
- **Never hand-edit `Core/Src/main.c` / `freertos.c`** (CubeMX-owned,
  regenerated from `AMS.ioc`). App logic lives in `Core/{Inc,Src}/app/` and
  is invoked through `extern "C"` `ams_*_task_run` trampolines.
- **Never hand-edit `docs/dbc/ams.dbc` or `ROADMAP.md`** — both are
  generated. The DBC comes from the **code-first CAN DSL**
  (`Core/Inc/can/messages/*.def` → `tools/dbc_dump.cpp`); regenerate with
  `c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump > docs/dbc/ams.dbc`
  (CI's "DBC matches code" check enforces it). `ROADMAP.md` ← `.github/roadmap.yaml`.

## Safety-critical surface (higher review bar)

Changes to any of these need the `safety-critical` label, two reviews, and
a confirmation that [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1
invariants still hold:

- `Core/Src/app/safety_task.cpp` — `MainTask` (safety + FSM + relays + telemetry, 10 ms)
- `Core/Inc/app/state_machine.hpp` — the pure FSM
- `Core/Inc/app/safety_predicates.hpp` — the fault predicate set
- `Core/Inc/app/relay_driver.{hpp,cpp}` — contactors + `AMS_OK`
- `Core/Inc/app/ams_config.hpp` — thresholds / timings (many tagged `COMMISSION`)

## Architecture in 10 lines

- **One realtime task**, `MainTask` (thread name still `SafetyTask`), 10 ms:
  snapshot services → evaluate (debounced) fault predicates → step the FSM
  every 20 ms → drive contactors + `AMS_OK` inline → emit telemetry @500 ms
  → refresh the IWDG. The only realtime-priority task; producers can't preempt it.
- **Lock-free single-writer services** — `BmsService` (BmsPollTask),
  `CurrentService` (CurrentSensorTask), `VehicleService` (AcuCanTask).
  No mutexes (32-bit atomic access + single-writer contract).
- **Pure-logic core** (FSM, predicates, CAN/LTC encoders) is HAL-free → the
  182 host tests need no hardware or RTOS mocks.
- **FSM:** Start → Precharge → Transition → {Run | Charge}, plus sticky Error.
  Mode (Car/Charger) is locked at `Start → Precharge` and never re-evaluated.
- **Inputs:** TSMS (PF9, held level) + DASH_CHG (PF10, momentary **edge**).
  Start→Precharge needs TSMS held + a DASH_CHG press; Run/Charge are
  sustained by **TSMS alone** (releasing DASH_CHG does not fault). Charger
  precharge proceeds on a still-fresh operator `0x101`; Car on `dc_bus_V ≥ 95 %`.
- **Pins:** AIR− PB6, AIR+ PB5, precharge PB7, `AMS_OK` PB4; pack current ADC3 differential PF7/PF8 (INP3/INN3), DCDC current ADC3 single-ended PC1 (INP11).
- **ErrorLatch** is sticky across resets (RTC backup register).

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
| Bench calibration / build flag | [`docs/COMMISSIONING.md`](docs/COMMISSIONING.md), [`docs/HIL_BUILD.md`](docs/HIL_BUILD.md) |
| Contribution conventions | [`CONTRIBUTING.md`](CONTRIBUTING.md) |
| All tunable constants | [`Core/Inc/app/ams_config.hpp`](Core/Inc/app/ams_config.hpp) |
