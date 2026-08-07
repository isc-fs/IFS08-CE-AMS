# `AMS_HIL_CLEAR_ERROR_LATCH` build flag

Compile-time switch that wipes the sticky `ErrorLatch` on every boot so
a HIL bench session that ended in a faulted state comes back clean. It
is the **only** HIL-specific build flag and is **never compiled into a
flight build**.

> ⚠️ This flag defeats the flight contract that an `ErrorLatch` is
> sticky across resets. A binary built with `AMS_HIL_CLEAR_ERROR_LATCH`
> is **unsafe to connect to a real high-voltage pack**.

The HIL bench drives the real `BmsPollTask` against a Pi Pico
LTC6820 + LTC6811 emulator. Same isoSPI traffic, same PEC validation,
same predicate inputs — flight and bench run the **same BMS path**. The
only thing the bench needs that flight does not is the boot-time latch
wipe below.

---

## What the flag does

A single concern, guarded by `#if defined(AMS_HIL_CLEAR_ERROR_LATCH)`
in `App_InitTask` (`Core/Src/app/app_init_task.cpp`), after the latch
and firmware-health boot capture are initialised:

```cpp
ams::ErrorLatch::init();
ams::fw_health::capture_reset_cause();
ams::fw_health::latch_boot_fault();
#if defined(AMS_HIL_CLEAR_ERROR_LATCH)
    ams::ErrorLatch::clear();   // wipe any latch from a previous bench session
#endif
```

The `ErrorLatch` lives in `RTC->BKP_DR1` behind magic `0xA115EE51`.

### What it actually recovers from — and what it does not

This is the part that is easy to get backwards, so be precise about it:

- `RTC->BKP_DR1` survives a **warm** reset — the `0x002` CAN trigger →
  `NVIC_SystemReset`, an IWDG timeout, a fault-driven reset. **That is
  what this flag clears.** Without it, a warm reset after a faulted run
  boots straight back into `Error` and the bench, which is CAN-only, has
  no out-of-band clear path.
- Surviving a **power cycle** additionally requires the carrier to have a
  VBAT source (coin cell / supercap). **The MLC bench carrier has none**,
  so a power-cycle wipes `BKP_DR1` there regardless of this flag.
- **Flight-hardware VBAT is unconfirmed.** That leaves a real gap: if the
  flight carrier also lacks a VBAT source, the sticky-error-across-a-
  power-cycle contract does not hold in hardware either, and the flag is
  not what would be defeating it. Confirm the flight carrier's VBAT
  before relying on step 7 of the acceptance test in
  [`COMMISSIONING.md`](COMMISSIONING.md) §7.

It also lets the bench recover from a transient LTC chain-discovery
glitch on the emulator without a power cycle.

On flight the latch is intentionally sticky: a latched precharge / SDC /
cell fault must **not** be wiped by a reboot. Clearing it every boot
defeats that contract, which is exactly why this flag must never be in a
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

CI never enables it: the `Firmware cross-compile (arm-none-eabi)` job in
`.github/workflows/build-tests.yml` builds with the flag OFF, so any
image whose SHA matches a CI artifact is, by construction, a flight
(latch-sticky) build. Use a distinct build directory per flavour
(`build/` for flight, `build-hil/` for the bench) so binaries don't get
confused.

**Verifying which flavour is on a board** is not possible over CAN — the
flag leaves no runtime signature. Use the git hash in `0x6C6` below and
match it against a known-provenance build; that is the only evidence
available.

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

`GIT_HASH` must be hexadecimal (CMake hard-errors otherwise); the first 8
chars are used, matching `--short=8`. This lets the HIL provenance
assertions verify the hash instead of treating it as log-only.

---

## Bench observability

There is no HIL-only telemetry layout. Everything the bench needs is on
the ordinary pit-diag CAN stream, armed by sending the magic on `0x7F0`
(`DE AD BE EF` on, four zero bytes off):

| Range | Content |
|---|---|
| `0x680..0x697` | 24 cell-voltage frames (4 cells per frame, BE u16) |
| `0x6A0..0x6B8` | 25 temperature frames (8 temps per frame) |
| `0x6C0..0x6C9` | status frames (PEC counts, comms health, provenance) |
| `0x6CB` | balance-quiesce health (success / fail counts) |

`0x6CA` (firmware health, 1 Hz) is **ungated** — it is emitted regardless
of arm state, so you can ask "is the AMS app alive?" without
transmitting an arm frame. See [`CAN_MAP.md`](CAN_MAP.md) for byte
layouts.

Because flight and bench share one BMS path and one telemetry stream, a
bench observation is evidence about flight behaviour — which is the whole
reason the old data-stub flag was removed.

---

## See also

- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) §1 — safety invariants; the
  bench and flight share the same BMS path.
- [`docs/CAN_MAP.md`](CAN_MAP.md) — the pit-diag stream byte layouts.
- [`docs/BMS_LTC6811.md`](BMS_LTC6811.md) §7 — chain-length discovery on
  boot (runs on every build).
- [`docs/COMMISSIONING.md`](COMMISSIONING.md) — the bench calibration
  session this flag exists to make repeatable.
- The HIL acceptance plan (the living bench-test matrix that gates a
  `dev → main` release) and the Pi Pico LTC6820/LTC6811 emulator live
  outside this repo — ask the HIL owner for the current locations rather
  than trusting a link here.
