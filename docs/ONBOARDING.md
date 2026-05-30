# Onboarding — IFS08-CE-AMS

Welcome to the Accumulator Management System firmware. This is the
**Day-1 guide**: it gets you from zero to a working toolchain, gives you
the mental model, and sequences the rest of the docs so you don't have to
guess the reading order. Budget ~2 hours to go from here to "can read a
PR." Keep [`GLOSSARY.md`](GLOSSARY.md) open in another tab — it defines
every acronym below.

> **Safety-critical firmware.** This code opens and closes the contactors
> on a ~350 V battery pack. The whole design exists to make one promise:
> *if the pack is unsafe, the AIRs open within 10 ms.* Read
> [`ARCHITECTURE.md`](ARCHITECTURE.md) §1 (safety invariants) before you
> change anything in the safety path — and when in doubt, ask a reviewer.

---

## 1. What the AMS actually does

The AMS is the brain of the high-voltage battery. In one breath: it
**watches every cell**, **drives the contactors**, and **kills the pack**
the instant anything goes out of bounds.

- **Watches** — 95 cell voltages + 200 temperatures, read over isoSPI from
  a daisy-chain of LTC6811 monitors (via an LTC6820 bridge on SPI1), plus
  the pack current from an analog shunt on ADC3 (PF7).
- **Drives** — three contactors (AIR−, AIR+, precharge) on GPIO PB5/6/7,
  and the `AMS_OK` leg of the shutdown circuit (SDC) on PB4.
- **Kills** — a 10 ms safety supervisor evaluates a set of fault
  predicates every tick; any breach latches `Error`, opens every
  contactor, and drops `AMS_OK`. The latch is sticky across resets.

It runs in **two contexts**, decided once at start-up and never changed
for that power cycle:

| Context | When | Armed by |
|---|---|---|
| **Car** | Pack installed in the vehicle (VCU heartbeat `0x100` present) | TSMS held + a DASH_CHG press; precharge completes on `dc_bus_V ≥ 95 %` of pack |
| **Charger** | Pack on the charging station (VCU absent + operator `0x101` "CHRG" request present) | TSMS held + a DASH_CHG press; precharge completes when `0x101` is still fresh (the charger soft-starts) |

The two operator inputs are physical buttons: **TSMS** (PF9) is a *held*
master switch; **DASH_CHG** (PF10) is a *momentary press* (edge-detected).
Once running, `Run`/`Charge` are sustained by **TSMS alone** — releasing
DASH_CHG does nothing; only dropping TSMS (or a fault) ends them.

For the full picture: [`ARCHITECTURE.md`](ARCHITECTURE.md) is the as-built
reference, [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md) is the gate-by-gate state
machine, and [`DEEP_DIVE.md`](DEEP_DIVE.md) walks the whole codebase.

---

## 2. Set up your toolchain

You need two toolchains: one to **cross-compile** the firmware for the
STM32, and the host compiler to **run the unit tests** (which is the
fastest way to confirm your setup works).

**Prerequisites**

| Tool | Version | Notes |
|---|---|---|
| `arm-none-eabi-gcc` | 14.x | The cross-compiler. macOS: `brew install --cask gcc-arm-embedded`. Linux: distro package or the Arm GNU Toolchain tarball. |
| CMake | ≥ 3.22 | `brew install cmake` / distro package. |
| Ninja *(optional)* | any | Faster than Make; used by the CMake presets. |
| A C++17 host compiler | Clang (macOS) / GCC (Linux) | Already present on most dev machines; only needed for the host tests. |

**Clone and verify** — a fresh clone leaves you on `main` (production).
Switch to `dev` immediately; all work branches off `dev`.

```bash
git clone https://github.com/isc-fs/IFS08-CE-AMS.git
cd IFS08-CE-AMS
git checkout dev

# 1) Host unit tests — the fastest "is my setup sane?" check (~0.5 s).
cmake -B build-tests -S tests/unit
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
#   → expect "182 Tests 0 Failures".

# 2) Cross-compile the firmware.
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
arm-none-eabi-size build/AMS.elf            # see the flash/RAM footprint
scripts/check_flash_layout.py build/AMS.elf # sector-0 / overflow guard
```

If both succeed you have a working environment. CI runs exactly these two
builds on every push, so green-locally usually means green-in-CI.

> **Bench build.** The HIL rig uses a third build with
> `-DAMS_HIL_CLEAR_ERROR_LATCH=ON`, which wipes the sticky error latch on
> boot so a faulted bench session restarts clean. It must **never** reach a
> flight build — full detail in [`HIL_BUILD.md`](HIL_BUILD.md).

---

## 3. The mental model (read this before the code)

The firmware is one **realtime supervisor task** plus a few lower-priority
producers feeding it data through lock-free services:

```
BmsPollTask      ─┐  (isoSPI: 95 cells @250ms, 200 temps @500ms, balancing)
CurrentSensorTask ┤→  volatile single-writer services  →  MainTask (10 ms)
AcuCanTask       ─┘  (FDCAN1: VCU 0x100, operator 0x101, boot trigger)        │
                                                                              ▼
                                          evaluate fault predicates → step FSM (20 ms)
                                          → drive contactors + AMS_OK + telemetry + watchdog
```

Three ideas do most of the work:

1. **One safety timeline.** `MainTask` (the thread is still named
   `SafetyTask`) is the only realtime-priority task. It snapshots the
   services, runs the fault predicates, steps the FSM, and drives the
   relays — all in one timeline, so there's no cross-task race in the
   safety path. Producers run at lower priority and can't preempt it.
2. **Lock-free single-writer services.** `BmsService`, `CurrentService`,
   `VehicleService` are each written by exactly one task and read by
   others. No mutexes — 32-bit aligned access on the Cortex-M7 is atomic,
   and the predicates tolerate one-cycle staleness.
3. **Pure-logic core.** The FSM ([`state_machine.hpp`](../Core/Inc/app/state_machine.hpp)),
   the fault predicates ([`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp)),
   and the CAN/LTC encoders are HAL-free pure functions — which is why
   there are 182 host unit tests with no hardware or RTOS mocked.

---

## 4. Guided reading order

Read top-to-bottom; each builds on the last.

1. **This file** — you're here.
2. [`GLOSSARY.md`](GLOSSARY.md) — skim it, then refer back constantly.
3. [`ARCHITECTURE.md`](ARCHITECTURE.md) **§1 (safety invariants)** and
   **§3 (task table)** — the non-negotiable rules and what runs when.
4. [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md) — the six states, the Car/Charger
   mode lock, and exactly what each transition does. Read alongside
   [`state_machine.hpp`](../Core/Inc/app/state_machine.hpp).
5. [`CAN_MAP.md`](CAN_MAP.md) — the wire protocol on FDCAN1 (telemetry,
   VCU `0x100`, operator `0x101`, boot trigger, pit-diag stream).
6. [`BMS_LTC6811.md`](BMS_LTC6811.md) — only when you touch the isoSPI /
   LTC6811 / balancing path.
7. [`DEEP_DIVE.md`](DEEP_DIVE.md) — the full end-to-end walkthrough when
   you want every subsystem mapped to source in one pass.
8. [`COMMISSIONING.md`](COMMISSIONING.md) and [`HIL_BUILD.md`](HIL_BUILD.md)
   — when you start working on the bench.
9. [`CONTRIBUTING.md`](../CONTRIBUTING.md) — before your first PR.

For the *why* behind the 2026 architecture (vs the 2025 bare-metal
firmware), there's a narrative read: `docs/AMS_2025_VS_2026.html`.

---

## 5. Where everything lives

```
Core/Inc/app/   hand-written headers  — ams_config.hpp (ALL tunables),
                state_machine.hpp, safety_predicates.hpp, *_service.hpp,
                ltc6811.hpp, relay_driver.hpp, …
Core/Src/app/   hand-written .cpp     — safety_task.cpp (MainTask body),
                bms_poll_task.cpp, acu_can_task.cpp, current_task.cpp, …
Core/Src/main.c CubeMX-generated      — DO NOT hand-edit; it calls our
                (+ freertos.c)          ams_*_task_run trampolines
tests/unit/     host Unity tests      — test_*.cpp, mocks/, CMakeLists.txt
docs/           you are here
tools/          gen_dbc.py (CAN DB generator), …
scripts/        check_flash_layout.py (CI flash-budget guard)
.github/        workflows + roadmap.yaml (auto-tracking, CI, roadmap)
```

The line between **CubeMX-owned** (regenerated from `AMS.ioc`) and
**hand-written** (`Core/{Inc,Src}/app/`) code is sacred: never put logic
in `main.c`. See [`ARCHITECTURE.md`](ARCHITECTURE.md) §2.

Every tunable constant lives in
[`ams_config.hpp`](../Core/Inc/app/ams_config.hpp). Anything needing
on-vehicle calibration is tagged `COMMISSION` — see
[`COMMISSIONING.md`](COMMISSIONING.md).

---

## 6. How we work (the short version)

Full detail is in [`README.md`](../README.md) and
[`CONTRIBUTING.md`](../CONTRIBUTING.md); the essentials:

- **Branch off `dev`**, never commit to `dev` or `main` directly.
  Naming: `feat/<n>-<short-title>` or `fix/<n>-<short-title>` (independent
  counters; next number = last closed issue of that type + 1).
- **Pushing a branch auto-opens a tracking issue.** Your first commit
  message fills its description — make it good.
- **PRs target `dev`.** Put `Closes #<issue>` in the body. A `dev → main`
  PR is a *release* and only happens after the HIL acceptance gate
  ([issue #317](https://github.com/isc-fs/IFS08-CE-AMS/issues/317)) is
  green on the same SHA.
- **Touching the safety supervisor, relays, FSM, or a predicate?** Label
  the PR `safety-critical` — it carries a higher review bar (two reviews,
  a SIL trace as evidence). See
  [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Modifying the safety supervisor".

---

## 7. First-week checklist

- [ ] Both builds pass locally (host tests + cross-compile).
- [ ] You can explain, in one sentence each, what `MainTask`, the FSM,
      and the three services do.
- [ ] You can trace `Start → Precharge → Run` and name what closes each
      contactor (hint: [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md)).
- [ ] You understand why `Run` is sustained by TSMS and not DASH_CHG.
- [ ] You've found `ams_config.hpp` and know what `COMMISSION` means.
- [ ] You've opened a throwaway `feat/<n>-…` branch, watched the tracking
      issue appear, and deleted it.

When the boxes are checked, pick up a `good first issue` or ask a lead
what's unblocked. Welcome aboard.
