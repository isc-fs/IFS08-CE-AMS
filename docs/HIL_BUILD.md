# `AMS_HIL_CLEAR_ERROR_LATCH` build flag

Compile-time switch that wipes the sticky `ErrorLatch` on every boot so
a HIL bench session that ended in a faulted state comes back clean. It
is the **only** HIL-specific build flag and is **never compiled into a
flight build**.

> ⚠️ This flag defeats the flight contract that an `ErrorLatch` is
> sticky across resets. A binary built with `AMS_HIL_CLEAR_ERROR_LATCH`
> is **unsafe to connect to a real high-voltage pack**.

The HIL bench drives the real `BmsPollTask` against a Pi Pico
LTC6820 + LTC6811 emulator (IFS08_HIL
[`feat/pico-ltc-emulator`](https://github.com/isc-fs/IFS08_HIL/tree/feat/pico-ltc-emulator),
wired on MLC2 J8). Same isoSPI traffic, same PEC validation, same
predicate inputs — flight and bench run the **same BMS path**. The only
thing the bench needs that flight does not is the boot-time latch wipe
below.

---

## What the flag does

A single concern, guarded by `#if defined(AMS_HIL_CLEAR_ERROR_LATCH)`.

`App_InitTask` calls `ErrorLatch::clear()` immediately after
`ErrorLatch::init()`:

```cpp
ams::ErrorLatch::init();
#if defined(AMS_HIL_CLEAR_ERROR_LATCH)
    ams::ErrorLatch::clear();   // wipe any latch from a previous bench session
#endif
```

The `ErrorLatch` lives in `RTC->BKP_DR1` (magic `0xA115EE51`). The
backup domain is VBAT-backed, so the latch survives 30+ seconds of
power-off on most bench carriers (coin cell + bulk caps) — it outlives a
bench power cycle. Without this clear, the unit boots straight into
`Error` whenever the previous session ended with the latch set, and the
bench has no out-of-band clear path (the bench is CAN-only). The clear
lets every test start from a known-clean state.

On flight the latch is intentionally sticky — once set, only physical
backup-domain power loss clears it. Clearing it every boot would defeat
that contract, which is exactly why this flag must **never** be in a
flight build.

---

## How to enable

`AMS_HIL_CLEAR_ERROR_LATCH` is a CMake `option()` that defaults to
**OFF** — flight is the safe-by-default build:

```bash
# Flight (default) -- flag OFF
cmake -B build \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build

# HIL bench (auto-clears ErrorLatch on boot)
cmake -B build-hil \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DAMS_HIL_CLEAR_ERROR_LATCH=ON
cmake --build build-hil
```

CI never enables it: the `Firmware cross-compile` workflow builds with
the flag OFF, so any image whose SHA matches a CI artifact is, by
construction, a flight (latch-sticky) build. Use a distinct build
directory per flavour (`build/` for flight, `build-hil/` for the bench)
so binaries don't get confused.

### Build provenance in `0x6C6` (`-DGIT_HASH`)

`firmware_info` (pit-diag `0x6C6`) carries the build's git short hash so
the bench / MingoCAN can confirm exactly which commit is flashed. It's
captured from `git rev-parse` at configure time — but a build with **no
`.git` tree** (a Docker/CI image, a release tarball, the HIL bench's
rsync copy) can't derive it and falls back to zeros. Pass it in
explicitly so `0x6C6[3..6]` reflects the real commit:

```bash
cmake -B build-hil \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DAMS_HIL_CLEAR_ERROR_LATCH=ON \
      -DGIT_HASH=$(git -C <repo-with-.git> rev-parse --short=8 HEAD)
```

`GIT_HASH` must be hexadecimal; the first 8 chars are used (matching
`--short=8`). This lets the A-013 / G-096 / F-075 provenance assertions
verify the hash instead of treating it as log-only.

---

## History

The legacy `AMS_BMS_HIL_STUB` umbrella flag — which used to stub the BMS
data source, swap the `0x4A2` telemetry layout, and relax the
current-sensor freshness predicate — was **retired once the Pi Pico
LTC6820/LTC6811 emulator landed**. With a real LTC chain on the bench,
flight and HIL now share the same BMS path, and the only HIL-only
behaviour that remains is the boot-time latch wipe described above.

The diagnostics the old stub layout used to overlay onto `0x4A2` now
live on the pit-diag CAN stream (`0x680..0x6C8`): 24 cell-voltage frames
(`0x680..0x697`), 25 temperature frames (`0x6A0..0x6B8`), and status
frames (`0x6C0..0x6C8`). See [`CAN_MAP.md`](CAN_MAP.md).

---

## `AMS_TEMP_STUB` build flag (no NTC sensor PCB)

A second bench-only CMake `option()` (default **OFF**) for bringing up
the BMS + AMS **before the NTC temperature sensor PCB exists**. When ON,
`run_temperature_poll()` **skips the ADG731/ADAX mux sweep entirely** and
pins every `cell_tempC` slot to `config::TempStubValueC` (25 °C) via
`BmsService::set_all_temperatures()`, so the over/under-temp predicate
sees an in-range pack and passes.

> ⚠️ Like `AMS_HIL_CLEAR_ERROR_LATCH`, this defeats a real safety check
> (cell over/under-temperature) and must **never** be in a flight image.
> Defaults OFF; CI builds with it OFF.

Cell-**voltage** safety is unaffected: `first_full_poll_done` is armed by
the voltage poll, so cell UV/OV/offline/stale faults are detected
normally — you're only neutralising the temperature branch.

```bash
# Bench bring-up: stub temps AND auto-clear the latch
cmake -B build-hil \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DAMS_HIL_CLEAR_ERROR_LATCH=ON \
      -DAMS_TEMP_STUB=ON
cmake --build build-hil
```

Retire the flag once the sensor PCB is in: a flag-OFF build runs the real
mux sweep and the temp predicate goes live again. Guarded by
`#if defined(AMS_TEMP_STUB)` in `bms_poll_task.cpp`.

---

## See also

- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) §1 — safety invariants; the
  bench and flight share the same BMS path.
- [`docs/CAN_MAP.md`](CAN_MAP.md) — the pit-diag stream
  (`0x680..0x6C8`) carries the bench observability the old stub layout
  used to provide.
- [`docs/BMS_LTC6811.md`](BMS_LTC6811.md) §7 — chain-length discovery on
  boot (runs on every build).
- HIL acceptance plan — [issue #317](https://github.com/isc-fs/IFS08-CE-AMS/issues/317)
  (the living bench-test matrix, blocks A–G, that gates a `dev → main` release).
- IFS08_HIL [`feat/pico-ltc-emulator`](https://github.com/isc-fs/IFS08_HIL/tree/feat/pico-ltc-emulator) —
  the Pi Pico LTC6820/LTC6811 emulator that replaced the data stub.
