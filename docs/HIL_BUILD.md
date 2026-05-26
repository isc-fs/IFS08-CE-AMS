# HIL bench build flags

The HIL bench rig drives a real LTC6820/LTC6811 chain via the Pi
Pico emulator (IFS08_HIL `feat/pico-ltc-emulator`) on MLC2 J8.
Flight and bench therefore run the **same** BMS, current-sensor,
predicate, FSM, and telemetry code paths. The only remaining
build-time divergence is one CMake `option()`.

---

## 1. Why it exists

The ErrorLatch lives in `RTC->BKP_DR1` and persists across reset
(deliberately — once latched, only a power cycle or explicit
operator action should clear it). On the bench that gets in the
way: a faulted test would leave the latch set, and there's no
in-band CAN command to clear it during a test sweep. The flag
makes the AMS wipe the latch as part of `App_InitTask`, so the
next test starts clean.

It must **never** be compiled into a flight image. Doing so would
turn a sticky persistent latch into a self-clearing one, defeating
the entire point of `RTC->BKP_DR1`.

---

## 2. The flag

| Flag | Effect |
|---|---|
| `AMS_HIL_CLEAR_ERROR_LATCH` | `App_InitTask` calls `ErrorLatch::clear()` immediately after `ErrorLatch::init()`. No other code path is gated on this flag. |

Source: `Core/Src/app/app_init_task.cpp`, gated by
`#if defined(AMS_HIL_CLEAR_ERROR_LATCH)`.

The legacy `AMS_BMS_HIL_STUB` flag was retired once the Pico
emulator landed — there is no longer any code path that relaxes
safety invariants for the bench. If you find a stale reference to
`AMS_BMS_HIL_STUB`, please open an issue.

---

## 3. Building

```bash
cmake -B build-hil -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
                   -DAMS_HIL_CLEAR_ERROR_LATCH=ON
cmake --build build-hil
```

CMake echoes the resolved value at configure time:

```
-- AMS_HIL_CLEAR_ERROR_LATCH = ON
```

Defaults to `OFF`. The CI workflow
(`.github/workflows/build-tests.yml`) never sets it on, so a
release image straight off `main` is always safe-by-default.

---

## 4. Verifying a flight image isn't HIL-built

The pit-diag firmware ID frame (`0x6C6`, see
[`CAN_MAP.md`](CAN_MAP.md)) carries the build's semver + git short
hash. Cross-reference the hash against a CI build artefact (which
is always compiled `OFF`) — if it matches a CI commit, the image
came from a pipeline that cannot have HIL flags enabled.

---

## 5. Related

- [`ARCHITECTURE.md`](ARCHITECTURE.md) §1 — safety invariants, which
  are the same in flight and HIL builds.
- [`CAN_MAP.md`](CAN_MAP.md) — the pit-diag stream (`0x680..0x6C8`)
  that the bench uses for runtime observability.
