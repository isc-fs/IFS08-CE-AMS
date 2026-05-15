# `AMS_BMS_HIL_STUB` build flag

Compile-time switch that lets you run the AMS firmware on a bench rig
that **doesn't have a real LTC6811 chain attached**. It exists so the
HIL session can exercise the FSM, the relay path, the FDCAN1
telemetry path, and the bootloader-trigger path without the
guaranteed safety-fault that a missing chain would otherwise cause.

> ⚠️ **Never compiled into a flight build.** The flag intentionally
> disables the chain-discovery safety gate and the BMS-related safety
> predicates. A build with `AMS_BMS_HIL_STUB` defined is **unsafe to
> connect to a real high-voltage pack** — the supervisor will not
> shut the pack down on a cell-V / cell-T / freshness fault. CI
> guarantees nothing about this flag; the convention is "use it on
> the bench, build flight images without it".

---

## When to use it

| Scenario | Use the flag? |
|---|---|
| Bench rig with no LTC6811 chain, just the MCU + relay GPIO breakout. | **Yes.** Without it, chain-length discovery latches ERROR before SafetyTask's first tick and you can't exercise anything past boot. |
| Real BMS_LITE pack on the bench (5 modules, full isoSPI chain). | **No.** You want the real safety gates to be live. |
| Vehicle. | **Never.** |
| HIL-001 .. HIL-008 (boot + memory-layout tests, no BMS dependency). | Optional. Either works. Without the flag is a more faithful test. |
| HIL-009 .. HIL-040 (the BMS-dependent tests). | **No.** These tests need the real chain wired up — that's what they validate. |
| HIL-041 .. HIL-047 (boot-trigger tests). | Either. The trigger path doesn't depend on chain health. |
| HIL-056 .. HIL-061 (v1.2.0 LTC-specific). | **No.** They exist specifically to exercise the chain. |

---

## What it disables

Three sites in the code, all guarded by `#ifdef AMS_BMS_HIL_STUB`:

### 1. `ErrorLatch::clear()` on every boot

`Core/Src/app/app_init_task.cpp`:

```cpp
#if defined(AMS_BMS_HIL_STUB)
    ams::ErrorLatch::clear();
#endif
```

The flight contract says ERROR is sticky across resets via
`RTC->BKP1R`, cleared only by full backup-domain power loss
(invariant 5 in [`ARCHITECTURE.md`](ARCHITECTURE.md) §1). On a
bench carrier with a VBAT coin cell + bulk caps that contract holds
*too well* — a stale latch from a previous bench session survives
power-cycles indefinitely and there's no SWD on most bench rigs to
clear it externally. The stub build wipes the latch on every boot
so the operator isn't locked out.

### 2. Skip LTC chain discovery

`Core/Src/app/app_init_task.cpp`:

```cpp
#if !defined(AMS_BMS_HIL_STUB)
    auto& ltc_bus = ams::ltc6820::Bus::default_instance();
    ltc_bus.configure(&hspi1, ...);
    ltc_bus.wakeup();
    // ... RDCFGA -> count_pec_valid_segments -> ErrorLatch + FORCE_ERROR
    // on mismatch
#else
    (void)hspi1;
#endif
```

Without the flag, App_InitTask issues `RDCFGA` over isoSPI, expects
`kLtcChainLength = 10` PEC-clean replies, and latches ERROR on
mismatch (see [`BMS_LTC6811.md`](BMS_LTC6811.md) §7 +
[`HIL_TESTS.md`](HIL_TESTS.md) HIL-056). On a bench with no chain
that path returns 0 ICs and the AMS would never leave Error — the
flag short-circuits the gate.

### 3. Skip BMS-side safety predicates

`Core/Inc/app/safety_predicates.hpp`:

```cpp
#if !defined(AMS_BMS_HIL_STUB)
    // module_online_mask + per-module freshness checks
    // cell V range checks
    // cell T range checks
#endif
```

The SafetyTask predicate set normally drives FORCE_ERROR on:
module-online mismatch, BMS freshness expiry, cell undervoltage,
cell overvoltage, cell overtemp, cell undertemp. All of these read
`BmsState`, which is never populated in a stub build. The flag
removes them from the predicate set so the FSM can actually leave
`Start` and exercise the rest of the system. The non-BMS predicates
(FORCE_ERROR, SDC open, current overlimit, current-sensor stale,
VCU stale) stay active.

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

1. **Symbol presence**: a flight binary has no
   `ams_app_init_task_run.cold` branch that calls
   `ErrorLatch::clear()`.
   ```bash
   arm-none-eabi-objdump -d build/AMS.elf \
     | grep -A2 'ams_app_init_task_run' \
     | grep -c 'ErrorLatch.*clear'
   # Flight build: 0
   # Stub build:   1 or more
   ```
2. **Inspection on the bench**: power-cycle a chip with `RTC->BKP1R`
   pre-loaded to `0xA115EE51` (the ERROR magic). A flight image
   comes up in Error. A stub image comes up in Start.
3. **CI provenance**: the `Firmware cross-compile` workflow does
   **not** pass `-DAMS_BMS_HIL_STUB`. Any image whose SHA matches a
   CI artifact is, by construction, not a stub build.

The flight-release procedure should require check 1 (or 3) on every
SHA before tagging.

---

## Operating tips

- The flag does NOT disable the LTC6820 SPI peripheral itself —
  `MX_SPI1_Init()` still runs. You can probe PA4 / PA5 / PA6 / PA7
  in a stub build; you just won't see any traffic because
  BmsPollTask's run-time work is gated upstream by the predicate set
  staying happy enough to leave Start.
- `BmsService::snapshot()` returns the zero-initialised BmsState in a
  stub build (with `cell_tempC[][]` defaulted to 25 °C via the ctor).
  Telemetry frames 0x4A0/4A1/4A2 still emit with these placeholder
  values.
- The boot-grace window (`kSafetyBootGraceMs = 2000`) is unchanged in
  a stub build. It still suppresses data-dependent predicates for the
  first 2 s; that's harmless because the stub build already
  suppresses those predicates indefinitely.

---

## See also

- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) §1 — invariant 5 (ERROR latch
  semantics that this flag deliberately violates).
- [`docs/BMS_LTC6811.md`](BMS_LTC6811.md) §7 — chain-length discovery
  on boot (the predicate this flag skips).
- [`docs/HIL_TESTS.md`](HIL_TESTS.md) — which tests should and must
  not run on a stub build.
