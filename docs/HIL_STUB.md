# `AMS_BMS_HIL_STUB` build flag

Compile-time switch that lets you run the AMS firmware on a bench rig
that **doesn't have a real LTC6811 chain attached**. It exists so the
HIL session can exercise the FSM, the relay path, the FDCAN1
telemetry path, and the bootloader-trigger path without the
guaranteed safety-fault that a missing chain would otherwise cause.

> ⚠️ **Never compiled into a flight build.** The flag intentionally
> fakes the BMS data source so the safety predicates see nominal-
> healthy data they aren't checking against real hardware. A build
> with `AMS_BMS_HIL_STUB` defined is **unsafe to connect to a real
> high-voltage pack** — the supervisor will not detect a cell-V /
> cell-T / freshness fault. CI guarantees nothing about this flag;
> the convention is "use it on the bench, build flight images
> without it".

---

## When to use it

| Scenario | Use the flag? |
|---|---|
| Bench rig with no LTC6811 chain, just the MCU + relay GPIO breakout. | **Yes.** Without it, chain-length discovery latches ERROR before MainTask's first iteration and you can't exercise anything past boot. |
| Real BMS_LITE pack on the bench (5 modules, full isoSPI chain). | **No.** You want the real safety gates to be live. |
| Vehicle. | **Never.** |
| HIL-001 .. HIL-008 (boot + memory-layout tests, no BMS dependency). | Optional. Either works. Without the flag is a more faithful test. |
| HIL-009 .. HIL-040 (the BMS-dependent tests). | **No.** These tests need the real chain wired up — that's what they validate. |
| HIL-041 .. HIL-047 (boot-trigger tests). | Either. The trigger path doesn't depend on chain health. |
| HIL-056 .. HIL-061 (v1.2.0 LTC-specific). | **No.** They exist specifically to exercise the chain. |

---

## What it does

The flag relocates the stub to the **data source** (refactor/19
phase 2, PR #119). Two sites in the code, both guarded by
`#if defined(AMS_BMS_HIL_STUB)`:

### 1. `BmsPollTask` body — seed a nominal-healthy snapshot

`Core/Src/app/bms_poll_task.cpp`:

```cpp
#if defined(AMS_BMS_HIL_STUB)
    for (;;) {
        ams::BmsService::instance().seed_for_hil_stub(osKernelGetTickCount());
        osDelay(ams::config::kBmsPollVoltMs);
    }
#else
    // ... real ADCV / RDCV[A-D] / mux sweep / balance WRCFGA ...
#endif
```

Under the flag, the BmsPollTask body collapses to a 250 ms loop that
calls `BmsService::seed_for_hil_stub(now_tick)`. The seeder stamps a
nominal-healthy snapshot into `BmsState`:

- `module_online_mask = kAllModulesMask` (all 5 modules present)
- `last_rx_tick[m] = now_tick` (freshness check passes)
- `cell_mV[m][c] = 3750` (mid-pack nominal, in range)
- `cell_tempC[m][t] = 25` (ambient, in range)
- summaries (`min/max/avg`, `pack_voltage_mV`) recomputed from arrays

The **real LTC/SPI/balance code path is compiled out** — literally
not in the binary when the flag isn't defined. Not "guarded at
runtime", not "skipped with an early return". The flight binary
contains no reference to `seed_for_hil_stub`.

### 2. `App_InitTask` — clear ErrorLatch + skip LTC chain discovery

`Core/Src/app/app_init_task.cpp`:

```cpp
#if defined(AMS_BMS_HIL_STUB)
    ams::ErrorLatch::clear();   // wipe any latch from a previous session
#endif

#if !defined(AMS_BMS_HIL_STUB)
    auto& ltc_bus = ams::ltc6820::Bus::default_instance();
    ltc_bus.configure(&hspi1, ...);
    ltc_bus.wakeup();
    // ... RDCFGA -> count_pec_valid_segments -> ErrorLatch + Relays::open_all
    // on mismatch
#else
    (void)hspi1;
#endif
```

`ErrorLatch::clear()` defends against a VBAT-backed `RTC_BKP_DR1`
surviving a long power-cycle on the bench (most bench carriers have
a coin cell + bulk caps that hold the backup domain for 30+
seconds). On a flight binary the latch is meant to be sticky;
clearing it on every boot would defeat the safety contract.

Skipping LTC chain discovery prevents the guaranteed-fail path that
would re-latch immediately after the clear.

### What does NOT change under the flag

The **safety predicate is HIL-agnostic**:

`Core/Inc/app/safety_predicates.hpp` has no `#if defined` block.
The predicate evaluates the same checks (module online mask,
freshness, cell V/T range, current overlimit, VCU heartbeat) on
flight and stub builds alike. The stub flag changes what the data
*looks like* (real chain vs seeded fake), not what the predicate
*checks*.

This means: the cell V/T threshold logic in
`safety_predicates.hpp` is exercised on the bench every time you
boot a stub build — the seeded `cell_mV = 3750` is well within the
`[kCellUVmV, kCellOVmV]` window, so the predicate sees "healthy"
and lets the FSM out of Start. If you want to exercise the
predicate's *fault* paths under stub, set
`force_error_set = true` via whatever hook your bench uses (today
only `App_InitTask` writes it, on LTC discovery failure — which is
skipped under the flag).

---

## How to enable

The flag is **not defined anywhere in the committed build system** —
it's passed on the command line so the bench operator opts in
explicitly, build by build:

```bash
cmake -B build-hil \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_CXX_FLAGS="-DAMS_BMS_HIL_STUB=1"
cmake --build build-hil
```

Use a distinct build directory (`build-hil`) so the flight build
(`build/`) is never confused with a stub image.

---

## How to verify a flight image is NOT stub-built

Three independent checks, any one of them is sufficient:

1. **Symbol presence**: a flight binary has no `seed_for_hil_stub`
   symbol at all (the method isn't compiled under the flight build).
   ```bash
   arm-none-eabi-nm build/AMS.elf | grep -c seed_for_hil_stub
   # Flight build: 0
   # Stub build:   1
   ```
2. **Inspection on the bench**: power-cycle a chip with `RTC->BKP1R`
   pre-loaded to `0xA115EE51` (the ERROR magic). A flight image
   comes up in Error. A stub image comes up in Start (because
   App_InitTask clears the latch).
3. **CI provenance**: the `Firmware cross-compile` workflow does
   **not** pass `-DAMS_BMS_HIL_STUB`. Any image whose SHA matches a
   CI artifact is, by construction, not a stub build.

The flight-release procedure should require check 1 (or 3) on every
SHA before tagging.

---

## Operating tips

- The flag does NOT disable the LTC6820 SPI peripheral itself —
  `MX_SPI1_Init()` still runs. You can probe PA4 / PA5 / PA6 / PA7
  in a stub build; you just won't see any traffic because the
  BmsPollTask body that drives it isn't compiled in.
- Telemetry frames 0x4A0/4A1/4A2 emit with the seeded values
  (3750 mV nominal, 25 °C). Use this to sanity-check that your
  bench-side decoder is reading the layout correctly.
- The boot-grace window (`kSafetyBootGraceMs = 2000`) is unchanged
  in a stub build. It still suppresses data-dependent predicates
  for the first 2 s; that's harmless because the stub's seeded
  values pass the predicate anyway. The seeder runs once on each
  250 ms tick, so by t = 250 ms the BmsState is fresh and predicate-
  clean even outside the grace window.

---

## History

- **PR #107** introduced the flag, originally as a predicate-side
  bypass (`#if !defined(AMS_BMS_HIL_STUB)` block in
  `safety_predicates.hpp` that compiled out the BMS predicates).
- **PR #110** added the `App_InitTask` LTC-discovery skip and
  `ErrorLatch::clear()`.
- **PR #114** added a pre-scheduler `RTC_BKP_DR1` clear in
  `main.c` to win a priority race with `SafetyTask`. Three
  coordinated sites total.
- **PR #119 (refactor/19 phase 2)** relocated the stub to the
  data source: `BmsPollTask::seed_for_hil_stub` replaces the
  predicate-side guard. `safety_predicates.hpp` becomes HIL-
  agnostic. The pre-scheduler clear in `main.c` is retired (no
  longer needed; the race it was working around went away with
  the predicate guard). `App_InitTask`'s `ErrorLatch::clear()`
  stays as the single bench-only clear site. Two coordinated
  sites total — the simplest the integration has ever been.

---

## See also

- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) §1 — safety invariants
  the flag deliberately relaxes.
- [`docs/BMS_LTC6811.md`](BMS_LTC6811.md) §7 — chain-length
  discovery on boot (the path this flag skips).
- [`docs/HIL_TESTS.md`](HIL_TESTS.md) — which tests should and
  must not run on a stub build.
