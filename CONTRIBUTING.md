# Contributing to IFS08-CE-AMS

The README explains the day-to-day branch flow. This document covers the
conventions that go beyond "open a feat branch and PR to dev".

---

## Where to look first

| Question | File |
|---|---|
| What is the architecture? | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| What CAN frames does the AMS speak? | [`docs/CAN_MAP.md`](docs/CAN_MAP.md) |
| What is the current phase plan? | [`ROADMAP.md`](ROADMAP.md) (auto-generated) |
| Source of truth for the roadmap | [`.github/roadmap.yaml`](.github/roadmap.yaml) |
| Day-to-day Git flow | [`README.md`](README.md) |

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
2. **CI green** — build, unit tests, SIL tests (once they exist).
3. **One approving review** from another team member.
4. **`safety-critical` label** if the PR touches `SafetyTask`,
   `RelayDriver`, the FSM, or any safety predicate. PRs with this label
   need explicit confirmation that `docs/ARCHITECTURE.md` invariants still
   hold.

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

1. Add the `constexpr` ID, DLC, and any related thresholds to
   `Core/Inc/app/ams_config.hpp`.
2. Add encode/decode free functions in `Core/Src/app/can_frames.cpp`
   (or the per-service file).
3. Add a unit test in `tests/unit/` covering encode/decode round-trip and
   edge cases.
4. Update `docs/CAN_MAP.md` with the new entry.
5. If the frame talks to the VCU or BMS, label the PR `protocol` and
   confirm the change with the other team before merging.

---

## Adding a new task

1. Confirm the task is necessary — prefer adding a method to an existing
   service first. Tasks cost stack and context-switch overhead.
2. Update `docs/ARCHITECTURE.md` § 2 (task table) with priority, period,
   and role.
3. Update `docs/ARCHITECTURE.md` § 4 if the task introduces a new queue
   or event flag.
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
- The IWDG-feed point in `SafetyTask::step` must remain on the clean path
  only — feeding on the fault path defeats the whole watchdog design.
- Two approving reviews from team members familiar with FS safety rules,
  not one.
- A captured FSM-transition trace from SIL is attached as evidence.

---

## C++ rules (short version)

Full rules in `docs/ARCHITECTURE.md` § 11. Highlights:

- No exceptions, no RTTI, no `std::string`, no `std::vector`, no
  `<iostream>`, no `<thread>`.
- No runtime heap allocation after `osKernelStart`.
- One service per file pair, singleton via `instance()`.
- `enum class` for FSM states, `constexpr` for every constant.
- `ScopedMutex` always — no raw `osMutexAcquire`.
- Task entry points are `extern "C"`.

---

## Local builds and tests

```bash
# Tests (host)
cmake -B build -DBUILD_UNIT_TESTS=ON -DBUILD_SIL_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure

# Firmware (CubeIDE-generated makefile)
make -C Debug -j
```

Production firmware build is driven by STM32CubeIDE. The host-side CMake
exists only to run unit and SIL tests.
