# Contributing to IFS08-CE-AMS

[`README.md`](README.md) covers what the firmware is and how to build it.
This document is the conventions: branches, PRs, and the recipes for the
four changes people most often need to make.

This is safety firmware. The bar is not "it compiles and the tests pass" —
it is "a reviewer can see why it cannot open the AIRs late, or close them
early".

---

## Where to look first

| Question | File |
|---|---|
| I'm brand new — where do I start? | [`docs/ONBOARDING.md`](docs/ONBOARDING.md) |
| What does this term mean? | [`docs/GLOSSARY.md`](docs/GLOSSARY.md) |
| What is the architecture? | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| How does the safety FSM work? | [`docs/FSM_OVERVIEW.md`](docs/FSM_OVERVIEW.md) |
| Full codebase walkthrough | [`docs/DEEP_DIVE.md`](docs/DEEP_DIVE.md) |
| What CAN frames does the AMS speak? | [`docs/CAN_MAP.md`](docs/CAN_MAP.md) |
| How does the LTC6811 / isoSPI BMS path work? | [`docs/BMS_LTC6811.md`](docs/BMS_LTC6811.md) |
| What can fail, and does anything catch it? | [`docs/FMEA.md`](docs/FMEA.md) |
| What is `AMS_HIL_CLEAR_ERROR_LATCH`? | [`docs/HIL_BUILD.md`](docs/HIL_BUILD.md) |
| Which constants still need bench calibration? | [`docs/COMMISSIONING.md`](docs/COMMISSIONING.md) |
| Roadmap, and its source of truth | [`ROADMAP.md`](ROADMAP.md) (generated) ← [`.github/roadmap.yaml`](.github/roadmap.yaml) |
| What the AMS is, build commands, repo layout | [`README.md`](README.md) |
| How comments and docs must be written here | [`CLAUDE.md`](CLAUDE.md) |
| Every tunable constant | [`Core/Inc/app/ams_config.hpp`](Core/Inc/app/ams_config.hpp) |

Bench acceptance criteria for a release live in the HIL acceptance plan,
which is **outside this repo** — ask the HIL owner for the current
location.

---

## Branch naming

```
feat/<short-slug>   →  new functionality   (feat/balance-spatial-spread)
fix/<short-slug>    →  bug fix             (fix/precharge-stale-dc-bus)
```

Cut it from `dev`. The slug should be 2–4 lowercase words joined by
dashes and should name the *change*, not the file you touched — the
branch name becomes the tracking issue's title and is the only summary
most people will ever read.

**The numbering caveat, honestly.** `branch-issue.yml` still expects a
numeric counter (`feat/<n>-<slug>`): it splits the branch name on `/` and
`parseInt`s the second field, then compares it to "highest number seen in
an existing issue title + 1". A slug-only branch parses to `NaN`, so the
comparison always fails and the auto-opened issue carries a warning block
telling you to rename the branch. The issue is still created, still
labelled correctly, and still closes on merge — **the warning is expected
noise on a slug-only branch, not a problem to fix.** Recent branches on
`dev` are slug-only. Either style works; be consistent within a series.

**CI gotcha.** [`build-tests.yml`](.github/workflows/build-tests.yml)
triggers on pushes to `feat/**`, `fix/**`, `dev`, `main` and on PRs into
`dev`. A branch named anything else (`chore/…`, `docs/…`, `bench/…`) gets
**no CI on push** — you only find out it's broken when you open the PR.

---

## Commit messages

- Imperative mood: "add BmsPollTask freshness check", not "added".
- First line under 72 characters.
- The **first commit message** of a new branch is auto-copied into the
  tracking issue's description by `branch-issue.yml`, and only that first
  one — later commits do not update it, and a hand-edited description is
  never overwritten. Make it count.
- `Closes #N` goes in the **PR description only**, never in a commit
  message — `close-on-dev-merge.yml` parses the PR body.

---

## Pull requests

PRs target `dev`. `main` is the repo's default branch, so if you create a
PR from the web UI or with `gh pr create` and forget `--base dev`, you
will open a release PR by accident. The template at
[`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md)
loads automatically — fill in every section.

Required for merge:

1. **`Closes #N` in the body**, referencing the tracking issue.
   `close-on-dev-merge.yml` needs it. Comma-separated lists work
   (`Closes #75, #76, #77`), as do `Closes #1 and #2` and repeated
   `Closes #75, Closes #76`.
2. **CI green** — firmware cross-compile + flash-layout guard, the host
   unit/SIL suite, and the "DBC matches code (dbc_dump)" check.
3. **One approving review** from another team member.
4. **`safety-critical` label** if the PR touches any of:
   `Core/Src/app/safety_task.cpp` (`MainTask`),
   `Core/Inc/app/state_machine.hpp`,
   `Core/Inc/app/safety_predicates.hpp`,
   `Core/Inc/app/relay_driver.hpp` / `Core/Src/app/relay_driver.cpp`,
   or `Core/Inc/app/ams_config.hpp`. Those PRs need an explicit
   confirmation that the invariants in
   [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1 still hold.

Before requesting review, check the image still fits:
`python3 scripts/check_flash_layout.py build/AMS.elf`.

---

## Labels

| Label | Meaning |
|---|---|
| `feat` | Applied automatically to feat-branch tracking issues. |
| `fix` | Applied automatically to fix-branch tracking issues. |
| `safety-critical` | Touches safety supervisor / relays / FSM / thresholds. Triggers the extra review bar below. |
| `hardware-required` | Cannot be tested in SIL — needs the bench rig or the vehicle. |
| `blocked` | Waiting on something external (hardware, another team, a decision). |
| `protocol` | Changes a CAN frame definition. Coordinate with the VCU/ECU teams. |

Only `feat` and `fix` are applied by automation; the rest are applied by
hand.

---

## Comment and doc conventions

Enforced repo-wide — see [`CLAUDE.md`](CLAUDE.md) for the full rule and
the reasoning. The short version:

**Do not write** issue/PR numbers (`#279`), dates, version stamps
(`live since v2.1.0`), or change narrative (`was 2`, `fixed in`,
`the previous version`). None of that is checkable by a reader offline,
and `git log` already has it.

**Do write** physical and numerical reasoning
(`47R || 47R = 23.5 ohm -> 164 mA at 3.85 V`), safety contracts
(`TELEMETRY ONLY -- no safety predicate reads this`), non-obvious
invariants, datasheet pointers (`LTC6811 Table 53`), and honest gaps —
what is measured, what is assumed, what is untested.

CAN IDs, register addresses and `COMMISSION` markers are the exception:
there the number *is* the information.

---

## Adding a new CAN frame

The wire layout is code-first: there is exactly **one** place each frame's
byte layout is written down — its `.def` descriptor — from which the
struct, encoder, decoder and DBC row are all generated. Do not author a
layout in an encoder header.

1. Add (or edit) the `.def` under `Core/Inc/can/messages/` — it owns the
   ID, DLC, sender, cycle time and every field — and register it in
   `Core/Inc/can/messages/all_messages.inc`. Related thresholds and
   period constants still go in `Core/Inc/app/ams_config.hpp`.
   The declared cycle time must match the cadence group the frame is
   actually transmitted in (`EcuFastTxMs` / `EcuMidTxMs` / `EcuSlowTxMs`
   in `acu_can_task.cpp`); other teams size their receive timeouts from
   the number in the DBC.
2. If a firmware call site needs it, add a **thin adapter** in the
   relevant encoder header (`Core/Inc/app/telemetry_encoders.hpp` for the
   `0x4A*` telemetry frames, `Core/Inc/app/acu_tx_encoders.hpp` for the
   ECU TX matrix) that maps service fields into the generated
   `ifs08::<Name>_t` struct and calls `ifs08::encode_<Name>`. RX handlers
   live in the service that owns the frame (e.g. `vehicle_service.cpp`).
3. Add a unit test in `tests/unit/` covering the encode/decode round-trip
   and hardcoded-byte parity against the `.def`.
4. Regenerate and commit the DBC. Easiest is the CMake target CI uses:
   ```bash
   cmake -B build-tests -S tests/unit
   cmake --build build-tests --target dbc_dump
   ./build-tests/dbc_dump > docs/dbc/ams.dbc
   ```
   The `dbcinator` bot also regenerates it on every PR and pushes the
   result back onto your branch, so a stale DBC usually fixes itself —
   but the `DBC matches code (dbc_dump)` job fails the build if it does
   not. Never hand-edit `docs/dbc/ams.dbc`.
5. Update [`docs/CAN_MAP.md`](docs/CAN_MAP.md).
6. If the frame talks to the VCU or the ECU, label the PR `protocol` and
   confirm the change with that team before merging. A frame the AMS
   sends but nobody consumes is worth flagging in the PR as such — as is
   one whose consumer exists only on the other repo's `dev` and has never
   been proven on a shared bus, which is the discharge interlock `0x021`
   today.

> The BMS is **not** on CAN. Cell voltages and temperatures reach the AMS
> only over isoSPI — see the next section.

---

## Adding a new LTC6811 / isoSPI command

1. Declare the 11-bit command code in `Core/Inc/app/ltc6811.hpp` as a
   `Cmd<Name>` constant (e.g. `CmdWRCFGA = 0x0001`) alongside the
   existing set. Per-IC payload builders (`pack_cfga_payload`,
   `pack_adg731_select`) live in the same header.
2. The pure-logic encode/decode goes in `Core/Src/app/ltc6811.cpp`. **No
   HAL dependency** — it must compile in the host test build, which is
   the only place this logic is ever actually verified.
3. Add a test in `tests/unit/test_ltc6811_decode.cpp` with at least a
   datasheet-derived vector and one PEC-corrupt negative case. A decoder
   that accepts a corrupt frame is how bad cell data reaches the safety
   predicates.
4. The transport call site goes in `bms_poll_task.cpp` and uses
   `ams::ltc6820::Bus::default_instance()`. `BmsPollTask` is the **single
   owner of the isoSPI bus** — there is no bus mutex, so do not call the
   `Bus` from another task. Add a `Bus` method only if the call shape is
   genuinely new; `send_command`, `read_register_group`,
   `write_chain_command` and `stcomm` already cover most reads/writes.
5. Update [`docs/BMS_LTC6811.md`](docs/BMS_LTC6811.md) §5 (command set)
   with the new command and its cadence.
6. If the command touches balancing, respect the
   **quiesce-before-measure** rule: every balancing FET must be off
   (all-zero DCC mask, settled for `BalanceQuiesceMs`) before any
   cell-voltage conversion. The LTC's own `DCP = 0` is not enough —
   BMS_LITE bleeds through an external PMOS, and per LTC6811 Table 53
   `DCP = 0` only suppresses discharge on the measured cell and its
   neighbours. Bleed current returns through the harness, so ~179 mA
   across 50–200 mΩ of tap/connector impedance shifts the shared tap
   node by 9–36 mV against a 50 mV balancing threshold: the bled cell
   reads low and both neighbours read high. `docs/BMS_LTC6811.md` §8 has
   the rest.
7. If the command changes a safety predicate (e.g. enabling the LTC's
   onboard UV/OV detection), label the PR `safety-critical`.

---

## Adding a new task

1. Confirm it is necessary. Prefer a method on an existing service —
   tasks cost stack, context switches, and one more thing that can be
   starved. There are already seven threads.
2. Implement in `Core/Src/app/<name>_task.cpp`, exposing a C-callable
   `void ams_<name>_task_run(void *argument)` entry point declared in a
   matching `Core/Inc/app/<name>_task.h`.
3. Wire it up in **`Core/Src/main.c`**, not `freertos.c`: CubeMX emits
   the thread creation and the `Start<Name>Task` body into `main()`, and
   the body must call your trampoline from inside its
   `/* USER CODE BEGIN Start<Name>Task */` block. Anything outside a
   `USER CODE` block in a CubeMX file is destroyed the next time someone
   regenerates from `AMS.ioc`. Creating the thread itself means editing
   `AMS.ioc` in CubeMX — do that, do not hand-add an `osThreadNew`.
4. Pick a priority **below** `MainTask` (`osPriorityRealtime`). The
   priority ordering is the safety argument: no producer may be able to
   delay the AIR-open decision. A new realtime-priority thread needs the
   `safety-critical` bar.
5. Use `osDelayUntil` for periodic tasks and `osMessageQueueGet` /
   `osEventFlagsWait` for event-driven ones. Never `osDelay` alone — it
   drifts by however long the body took.
6. Update [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §3 (task table:
   priority, cadence, stack) and §8 (inter-task signalling) if you add a
   queue or event flag.

---

## Modifying the safety supervisor

`Core/Src/app/safety_task.cpp`, `state_machine.hpp`,
`safety_predicates.hpp`, `relay_driver.*` and the thresholds in
`ams_config.hpp` carry a hard review bar:

- **Two approving reviews** from team members familiar with FS safety
  rules, not one. (Team convention — it is not enforced by a branch
  protection rule you can read from the repo, so it is on the author to
  ask for the second reviewer.)
- The PR description lists every safety predicate added, removed or
  changed, and a captured FSM-transition trace from SIL is attached.
- **Do not move the IWDG feed.** It is on the clean path *and* on the
  latched-fault path. The latched path stays alive on purpose: the relays
  are already open and the latch persists, so an operator can read
  telemetry out of a faulted board instead of watching the node reset in
  a ~100 ms loop. Invariants 4 and 5 in
  [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1.
- **New freshness-dependent predicates must sit behind the boot-grace
  gate** (`now_tick < config::SafetyBootGraceMs`). At t = 0 no producer
  has published yet, so an ungated freshness check latches `Error` before
  the pack is even read (invariant 7).
- **Immediate-danger predicates must stay outside that gate** so they
  trip on the first iteration: current over-limit, BMS module offline,
  VCU stale, forced error.
- Two predicates deliberately debounce instead, and both are load-bearing
  numbers, not taste: the cell voltage/temperature range checks over
  `CellFaultConfirmTicks` (25 × 10 ms ≈ 250 ms) and BMS staleness over
  `BmsStaleConfirmTicks` (25 ≈ 250 ms). The first rejects a torn snapshot
  read, the second rides out a single dropped isoSPI poll on a far
  module. Stacked on the 200 ms voltage poll, that is ≈ 460 ms worst-case
  fault response against a < 500 ms budget. **If you change either
  constant, or `BmsPollVoltMs`, redo that arithmetic in the PR** — the
  comments in `ams_config.hpp` carry the working.

---

## C++ rules (short version)

Full rules in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §10.
Highlights:

- No exceptions, no RTTI, no thread-safe statics (all three are compiler
  flags in `CMakeLists.txt`). Banned: `std::string`, `std::vector`,
  `<iostream>`, `<thread>`.
- No heap allocation after `osKernelStart`. FreeRTOS allocates at task /
  queue / event-group / timer creation only.
- One service or task per `.hpp` + `.cpp` pair. **Pure-logic modules
  (FSM, predicates, balancing, SoC, encoders, ISO-TP, open-wire) are
  header-only** so the host test build gets them for free — that is why
  the test suite needs no hardware and no RTOS mocks. Keep new logic
  header-only if you possibly can.
- Singletons via a `static` local in `instance()`, constructed by an
  explicit `init()` from `App_InitTask`. With `-fno-threadsafe-statics`
  there is no guard mutex, so first touch must not race.
- `enum class` for state types. `safety::FaultReason` values are a **wire
  contract** on pit-diag: append only, never renumber.
- `constexpr` for every threshold, ID and period, collected in
  `ams_config.hpp`. Tag anything needing on-vehicle tuning `COMMISSION`
  and add it to [`docs/COMMISSIONING.md`](docs/COMMISSIONING.md).
- **Single-writer per shared struct. No mutexes in app code.** The
  services (`BmsService`, `CurrentService`, `VehicleService`) each have
  exactly one writing task and use 32-bit atomic access instead. CubeMX
  still declares `bms_mutex` / `current_mutex` / `vehicle_mutex` in
  `main.c` and `scoped_mutex.hpp` still exists, but nothing takes them.
  If you find yourself wanting a lock, the ownership design is wrong —
  talk to the safety reviewer.
- Task entry points are `extern "C"` trampolines.

---

## Local builds and tests

CMake end-to-end — the firmware target pulls the CubeMX source list from
`cmake/stm32cubemx/CMakeLists.txt`; the host tests use
`tests/unit/CMakeLists.txt`. There is no CubeIDE/Eclipse makefile.

```bash
# Host unit + SIL tests — run these for any logic change.
cmake -B build-tests -S tests/unit
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
./build-tests/ams_unit_tests        # real case count: "476 Tests 0 Failures"

# Firmware (cross-compile)
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
arm-none-eabi-size build/AMS.elf
python3 scripts/check_flash_layout.py build/AMS.elf   # sector-0 / overflow guard

# HIL bench build (auto-clears ErrorLatch on boot; see docs/HIL_BUILD.md)
cmake -B build-hil -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
                   -DAMS_HIL_CLEAR_ERROR_LATCH=ON
cmake --build build-hil
```

`ctest` reports `1/1 Test ... Passed` because the whole Unity suite is a
single runner executable; run the binary directly for the case count.

The host build adds `-Wextra -Wpedantic` on top of the firmware's
`-Wall`. Since the header-only logic modules are compiled by both, that
is where the stricter warnings actually bite.

CI never produces an image with HIL flags enabled — the bench build is
operator-driven only.
