# Contributing to IFS08-CE-AMS

The README explains the day-to-day branch flow. This document covers the
conventions that go beyond "open a feat branch and PR to dev".

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
| What is `AMS_HIL_CLEAR_ERROR_LATCH`? When do I use it? | [`docs/HIL_BUILD.md`](docs/HIL_BUILD.md) |
| What is the current phase plan? | [`ROADMAP.md`](ROADMAP.md) (auto-generated) |
| Source of truth for the roadmap | [`.github/roadmap.yaml`](.github/roadmap.yaml) |
| Day-to-day Git flow | [`README.md`](README.md) |
| Bench acceptance criteria for a release | HIL plan — [issue #399](https://github.com/isc-fs/IFS08-CE-AMS/issues/399) (v1.6.2; supersedes #317) |

---

## Branch naming

```
feat/<n>   →  new functionality          (feat/1, feat/2, ...)
fix/<n>    →  bug fix                    (fix/1, fix/2, ...)
```

The counters are independent. The next number is the last closed issue of
that type plus one — check the Issues tab filtered by label.

Optionally, append a short slug: `feat/3-bms-rx-task`. The branch-issue
workflow accepts both `[feat/3]` and `[feat/3-bms-rx-task]` titles.

---

## Commit messages

- Imperative mood: "add BmsRxTask freshness check", not "added".
- Keep the first line under 72 characters.
- The **first commit message** of a new branch is auto-copied into the
  tracking issue's description by the `branch-issue.yml` workflow — make
  it count.
- Use `Closes #N` only in the **PR description**, not in commit messages
  (the `close-on-dev-merge.yml` workflow parses the PR body).

---

## Pull requests

PRs target `dev`. The template at
[`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md) is
loaded automatically — fill in every section.

Required for merge:

1. **`Closes #N` line in the body**, referencing the tracking issue. The
   `close-on-dev-merge.yml` workflow needs it to auto-close on merge.
2. **CI green** — firmware cross-compile, the full host unit + SIL suite, and the "DBC matches code (dbc_dump)" check.
3. **One approving review** from another team member.
4. **`safety-critical` label** if the PR touches `MainTask`
   (the consolidated safety + FSM + telemetry task in
   `safety_task.cpp`), `Relays` (`relay_driver.{hpp,cpp}` —
   contactors + AMS_OK), the FSM, or any safety
   predicate. PRs with this label need explicit confirmation that
   `docs/ARCHITECTURE.md` invariants still hold.

---

## Labels

| Label | Meaning |
|---|---|
| `feat` | Applied automatically to feat-branch tracking issues. |
| `fix` | Applied automatically to fix-branch tracking issues. |
| `safety-critical` | Touches safety supervisor / relays / FSM. Triggers extra review. |
| `hardware-required` | Cannot be tested in SIL — needs bench rig or vehicle. |
| `blocked` | Waiting on something external (DBC update, hardware, decision). |
| `protocol` | Changes a CAN frame definition. Coordinate with VCU/BMS teams. |

---

## Adding a new CAN frame

The wire layout is code-first: there is exactly **one** place each frame's
byte layout is written down — its `.def` descriptor — from which the
struct, encoder, decoder, and DBC row are all generated. Don't author the
layout in the encoder headers.

1. Add (or edit) the frame's `.def` descriptor under
   `Core/Inc/can/messages/` — the single source of truth for the ID, DLC,
   and every field — and register it in
   `Core/Inc/can/messages/all_messages.inc`. (Any related thresholds or a
   period constant still go in `Core/Inc/app/ams_config.hpp`.)
2. If a firmware call site needs it, add a **thin adapter** in the
   relevant encoder header (`Core/Inc/app/telemetry_encoders.hpp`,
   `Core/Inc/app/acu_tx_encoders.hpp`) that maps service fields into the
   generated `ifs08::*_t` struct and calls the generated `ifs08::encode_*`.
   RX handlers still live in the per-service file that owns the frame
   (e.g. `vehicle_service.cpp` for an RX frame).
3. Add a unit test in `tests/unit/` covering the encode/decode round-trip
   and hardcoded-byte parity.
4. Regenerate and commit the DBC. The `DBC matches code (dbc_dump)` CI
   check — plus the `dbcinator` bot — enforces `docs/dbc/ams.dbc` against
   the descriptors; a stale DBC fails CI. Regenerate with:
   ```bash
   c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump \
     && /tmp/dbc_dump > docs/dbc/ams.dbc
   ```
5. Update `docs/CAN_MAP.md` with the new entry.
6. If the frame talks to the VCU, label the PR `protocol` and
   confirm the change with the VCU team before merging.

> The BMS no longer rides on CAN since v1.2.0 — BMS work goes through
> the isoSPI / LTC6811 path. See the next section.

---

## Adding a new LTC6811 / isoSPI command

1. If the command isn't already declared, add its 11-bit code to
   `Core/Inc/app/ltc6811.hpp` as a `kCmd<Name>` constant. Per-IC
   payload-builder helpers (like `pack_cfga_payload`,
   `pack_adg731_select`) also live in this header.
2. The pure-logic encode/decode lives in `Core/Src/app/ltc6811.cpp`.
   No HAL dependency — it must compile in the host unit-test build.
3. Add a unit test in `tests/unit/test_ltc6811_decode.cpp` with at
   least the datasheet-derived test vector and one PEC-corrupt
   negative case.
4. The transport call site lives in `bms_poll_task.cpp` (or a new
   helper) and uses `ams::ltc6820::Bus::default_instance()`. Add a
   `Bus` method only if the call shape is genuinely new
   (broadcast-write with per-IC payload, post-cmd dummy clocks,
   etc.) — most reads / writes are already covered by
   `send_command`, `read_register_group`, and `write_chain_command`.
5. Update [`docs/BMS_LTC6811.md`](docs/BMS_LTC6811.md) §5 with the
   new command + cadence row.
6. If the new command changes a safety predicate (e.g. enabling the
   LTC's onboard UV/OV detection), label the PR `safety-critical`
   and follow that section's bar.

---

## Adding a new task

1. Confirm the task is necessary — prefer adding a method to an existing
   service first. Tasks cost stack and context-switch overhead.
2. Update `docs/ARCHITECTURE.md` § 3 (task table) with priority, period,
   and role.
3. Update `docs/ARCHITECTURE.md` § 8 (inter-task signalling) if the task
   introduces a new queue or event flag.
4. Implement in `Core/Src/app/<name>_task.cpp` with an
   `extern "C"` entry point in `freertos.cpp`.
5. Use `osDelayUntil` for periodic tasks, `osMessageQueueGet` /
   `osEventFlagsWait` for event-driven ones. Never `osDelay` in
   isolation — it drifts.

---

## Modifying the safety supervisor

This is the only file in the repo with a hard review bar.

- The PR description must list every safety predicate added, removed, or
  changed.
- The IWDG-feed point in `MainTask` is on the clean path **and** on
  the latched-fault path (the latter stays alive so the operator
  can read telemetry from a latched state — see invariant 5 in
  [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1, and PR #107
  for the loop bug this avoids). Don't move it.
- New predicates that depend on a producer's freshness MUST live
  behind the boot-grace gate (`now_tick < kSafetyBootGraceMs`) so
  the chip doesn't latch ERROR at t = 0 (invariant 7 in
  [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §1).
- New predicates that are **immediate-danger** (current over-limit,
  BMS module offline) must stay outside the grace gate so they trip on
  the first iteration. Note the cell V/T **range** predicates are the
  exception — they debounce over `CellFaultConfirmTicks` (~300 ms, #296)
  so a torn snapshot read can't latch a spurious ERROR.
- Two approving reviews from team members familiar with FS safety rules,
  not one.
- A captured FSM-transition trace from SIL is attached as evidence.

---

## C++ rules (short version)

Full rules in `docs/ARCHITECTURE.md` § 10. Highlights:

- No exceptions, no RTTI, no `std::string`, no `std::vector`, no
  `<iostream>`, no `<thread>`.
- No runtime heap allocation after `osKernelStart`.
- One service per file pair, singleton via `instance()`.
- `enum class` for FSM states, `constexpr` for every constant.
- Single-writer / many-reader shared state — no mutexes in app
  code (the per-service mutexes were retired in refactor/19 phase 1).
- Task entry points are `extern "C"`.

---

## Local builds and tests

The build system is CMake end-to-end — the firmware target uses the
CubeMX-generated `cmake/stm32cubemx/CMakeLists.txt`; the host tests
use `tests/unit/CMakeLists.txt`. There is no CubeIDE / Eclipse-
managed makefile.

```bash
# Host unit + SIL tests
cmake -B build-tests -S tests/unit
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure

# Firmware (cross-compile)
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
arm-none-eabi-size build/AMS.elf
scripts/check_flash_layout.py build/AMS.elf   # sector-0 / overflow guard

# HIL bench build (auto-clears ErrorLatch on boot; see docs/HIL_BUILD.md)
cmake -B build-hil -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
                   -DAMS_HIL_CLEAR_ERROR_LATCH=ON
cmake --build build-hil
```

CI (`.github/workflows/build-tests.yml`) runs the host-test and
firmware-cross-compile builds on every push. The HIL bench build is
operator-driven; CI never produces an image with HIL flags enabled.
