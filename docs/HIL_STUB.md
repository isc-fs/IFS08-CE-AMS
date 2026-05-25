# `AMS_BMS_HIL_STUB` build flag

Compile-time switch that relaxes a small set of safety / observability
invariants so the HIL bench can boot the firmware in a controlled,
observable state. It is **never compiled into a flight build**.

> ⚠️ The flag deliberately relaxes the current-sensor freshness
> check and clears a sticky `ErrorLatch` on every boot. A binary with
> `AMS_BMS_HIL_STUB` defined is **unsafe to connect to a real
> high-voltage pack**.

The HIL bench drives the real `BmsPollTask` against a Pi Pico
LTC6820 + LTC6811 emulator (IFS08_HIL
[`feat/pico-ltc-emulator`](https://github.com/isc-fs/IFS08_HIL/tree/feat/pico-ltc-emulator),
wired on MLC2 J8). Same SPI traffic, same PEC validation, same
predicate inputs — flight and bench differ only in who's at the
other end of the chain. The flag covers the small set of bench-only
relaxations listed below. Issue #205 tracks splitting it into three
orthogonal sub-options.

---

## What the flag does

Three orthogonal concerns, all guarded by `#if defined(AMS_BMS_HIL_STUB)`:

### 1. Boot-time `ErrorLatch::clear()` in `App_InitTask`

`Core/Src/app/app_init_task.cpp`:

```cpp
#if defined(AMS_BMS_HIL_STUB)
    ams::ErrorLatch::clear();   // wipe any latch from a previous session
#endif
```

VBAT-backed `RTC_BKP_DR1` survives 30+ seconds of power-off on most
bench carriers (coin cell + bulk caps). Without this clear the unit
boots into `Error` whenever a previous session ended with the latch
set, and there's no SWD attached on the bench to clear it
externally. On flight the latch is intentionally sticky; clearing it
every boot would defeat the contract.

### 2. Bench-only diagnostic counters + `0x7FF` boot trace

`Core/Src/app/app_init_task.cpp` emits an FDCAN `0x7FF` frame at
each `App_InitTask` milestone (post-clear, post-filter,
post-Start, etc.) so the bench can confirm init progress without
SWD. Sibling globals `g_app_init_progress`, `g_fdcan1_start_result`
also gated on this flag.

### 3. `0x4A2` telemetry-frame layout swap

`Core/Inc/app/telemetry_encoders.hpp` reshapes bytes 3..5 of the
`0x4A2` "AMS temps + diagnostics" frame:

| Build | Bytes 3..4 | Byte 5 |
|---|---|---|
| Flight | `dc_bus_V` little-endian | reserved (0) |
| HIL_STUB | `bms_poll_task_state`, `acu_rx_total_lo` | `tsms_dash_chg_byte` |

The bench injects `dc_bus_V` from the host so its observability
isn't affected; the freed bytes carry per-loop diagnostic probes.

### 4. Current-sensor freshness predicate relax

`Core/Inc/app/safety_predicates.hpp`:

```cpp
#if !defined(AMS_BMS_HIL_STUB)
    if (in.now_tick - in.current.last_update_tick > config::IStaleMs) return true;
#endif
```

The bench has no real Bourns SSA-2-250A wired to `S_CURRENT`, so the
freshness check would trip ~`IStaleMs` after boot. `sensor_fault`
and the `|Imax|` envelope still apply, so a bench fixture that
injects synthetic current frames still gets meaningful safety
behaviour.

### What does NOT change under the flag (post-#207)

- **BMS data source**: real LTC SPI on every build. Flight runs against
  the actual chain; HIL runs against the Pico emulator.
- **Safety predicate** (cell V / T range, freshness, current-overlimit,
  VCU heartbeat): identical inputs, identical checks. Only the
  current-sensor freshness gate is skipped.

---

## How to enable

The flag is **not defined anywhere in the committed build system** —
it's passed on the command line so the bench operator opts in
explicitly:

```bash
cmake -B build-hil \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_CXX_FLAGS="-DAMS_BMS_HIL_STUB=1"
cmake --build build-hil
```

Use a distinct build directory (`build-hil`) so the flight build
(`build/`) is never confused with a stub image. #205 tracks
elevating this to a CMake `option()`.

---

## How to verify a flight image is NOT stub-built

1. **Symbol presence**: a flight binary has no `g_app_init_progress`
   symbol (it's gated on the flag).
   ```bash
   arm-none-eabi-nm build/AMS.elf | grep -c g_app_init_progress
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

## See also

- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) §1 — safety invariants
  the flag deliberately relaxes.
- [`docs/BMS_LTC6811.md`](BMS_LTC6811.md) §7 — chain-length
  discovery on boot (now runs on every build).
- [`docs/HIL_TESTS.md`](HIL_TESTS.md) — bench tests.
- IFS08_HIL [`feat/pico-ltc-emulator`](https://github.com/isc-fs/IFS08_HIL/tree/feat/pico-ltc-emulator) —
  the Pi Pico LTC6820/LTC6811 emulator that replaced the data stub.
- #205 — split this umbrella flag into three sub-options.
- #207 — the PR that removed the data-source stub.
