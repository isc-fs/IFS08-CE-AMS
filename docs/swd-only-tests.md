# SWD-only tests — parking lot

The MLC carriers on the production HIL do not expose SWD / JTAG / OCD.
Tests below cannot run on that bench because their enable path requires
GDB+OpenOCD (to halt a task, write a backup register, or inject an ADC
fault). They are explicitly **not** part of the v1.5.0 HIL acceptance
plan ([#245](../../issues/245)) and are not counted against any HIL
scoreboard.

They still have value during firmware-development bring-up on a
Nucleo + ST-LINK, so the contracts they describe are recorded here so
the work isn't lost.

## Why they were dropped

Each of these has appeared in earlier revs of the HIL acceptance plan
as "skipped, pending fixture". They aren't pending a fixture — they
require a hardware capability (SWD on the MLC carrier) that the
production bench will never gain. Keeping them on the HIL scoreboard
made every sweep look ~8 % worse than it actually was and let stale
skip-reasons rot in the test source for months. The HIL team flagged
this in [#245 comments](../../issues/245) and we agreed to strike
them from the HIL plan and park the contracts here instead.

## The parked tests

### B-011 — IWDG fires when MainTask hangs

Halt SafetyTask via GDB breakpoint past the IWDG window (~100 ms). The
IWDG should reset the chip; on the next boot the supervisor comes up
clean. Validates the watchdog-refresh contract.

How to run on a Nucleo + ST-LINK:
1. OpenOCD attached, `mon halt`.
2. Set breakpoint inside `ams_safety_task_run` after the
   `last_wake += SafetyPeriodMs;` line.
3. Continue. Verify the chip resets within ~100 ms of the breakpoint hit.
4. After reset, attach again, dump `RTC->BKP2R` to confirm the JumpReason
   indicates an unexpected reset.

### B-012 — Watchdog reset reopens all relays

Same setup as B-011, but with a logic analyser on PB5 / PB6 / PB7.
Within 100 µs of the IWDG-triggered reset, the relay outputs should all
go LOW (Relays::open_all). Verifies the relay-open path doesn't depend
on the FreeRTOS scheduler running.

Note: on the production HIL, relay-open observability is already covered
by F-066 via the TCA9555 readback path — no SWD or LA needed for that
specific check. B-012 only adds value during firmware-dev bring-up
to confirm the IWDG-reset case specifically.

### B-013 — ErrorLatch clears on boot under HIL_CLEAR_ERROR_LATCH

Write `RTC->BKP1R = 0xA115EE51` via GDB before power-cycling. Boot the
firmware with `AMS_HIL_CLEAR_ERROR_LATCH=ON`. App_InitTask should clear
the backup register before SafetyTask first looks at the latch; the FSM
comes up in Start, not Error.

The HIL-bench equivalent of this (which doesn't need SWD) is rolled into
F-076 / F-080 in [#245](../../issues/245): force the AMS into Error via
a deliberate fault stim (drop TSMS mid-Run), power-cycle, observe via
CAN telemetry. That covers the same firmware path without needing to
pre-set the backup register externally.

### B-016 / B-023 — Current sensor staleness

Halt `CurrentSensorTask` via GDB so `last_update_tick` stops advancing.
After `IStaleMs`, the predicate should fire and trip the FSM to Error.

Doubly-blocked on the production HIL:
1. No SWD to halt the task.
2. The freshness predicate is **relaxed under `AMS_BMS_HIL_STUB`** in
   `safety_predicates.hpp:71` — the assert path doesn't fire on the
   stub build at all. Even a flight-build HIL session can't easily
   reach this code path without GDB.

### B-025 — sensor_fault from ADC failure path

Force `HAL_ADC_PollForConversion` to fail via GDB-injected error. The
predicate's `sensor_fault` check should trip and force Error.

In production this code path is exercised by genuine ADC misbehaviour
(electrical fault, peripheral lock-up). It can't be cleanly induced
from the bench side without SWD.

### C-044 — Error survives reset (flight latch-stickiness)

The flight contract: `RTC->BKP1R = 0xA115EE51` set on Error must survive
a watchdog reset and any subsequent boot. Verifying this requires either
a) GDB to write the magic + reset, or b) a flight build (no
`AMS_HIL_CLEAR_ERROR_LATCH`) and a way to externally reset the chip
without losing the VBAT-backed backup domain.

On the production HIL the `AMS_HIL_CLEAR_ERROR_LATCH=ON` build wipes
the latch on every boot, which is the right behaviour for bench
iteration. C-044's flight-side contract gets covered by unit tests
(`tests/unit/test_error_latch.cpp`) and the firmware-dev Nucleo+ST-LINK
checklist below.

### D-042 — JumpReason directly from RTC->BKP2R

The earlier rev demanded reading `RTC->BKP2R` directly via SWD after
a CAN-trigger reboot, asserting it equals `0x4A554D50` ('JUMP').

**Superseded by D-052** in [#245](../../issues/245): same contract, but
read via the pit-diag `0x6C4[0..3]` frame (`encode_boot_diag`). No SWD
needed; the firmware ships the byte on can0.

## Firmware-dev checklist (Nucleo + ST-LINK)

These contracts should still be verified during a firmware-dev session
before a release tag. Suggested cadence: once per v1.x.0 release on a
Nucleo H7 dev board with ST-LINK attached, run-through that takes ~30
minutes:

1. **B-011** — set the SafetyTask breakpoint, confirm chip resets.
2. **B-012** — with the LA on PB5/6/7, confirm all three go LOW
   within 100 µs of the IWDG reset.
3. **B-013** — pre-set `BKP1R`, power-cycle, confirm `0x4A0[0]` comes
   up as Start.
4. **B-016 / B-023** — halt CurrentSensorTask, confirm staleness
   predicate trips after `IStaleMs`.
5. **B-025** — patch `HAL_ADC_Start` to return an error via GDB
   `set var`, confirm `sensor_fault` predicate trips.
6. **C-044** — flight build (`AMS_HIL_CLEAR_ERROR_LATCH=OFF`),
   pre-set BKP1R + magic, reset, confirm FSM comes up in Error.

A short writeup of each session goes in the release-notes PR alongside
the tag.

## When the bench gains SWD

If the production HIL ever grows an SWD interface, these tests can be
revived in #245. Until then they live here.
