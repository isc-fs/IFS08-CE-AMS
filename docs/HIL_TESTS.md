# AMS HIL test plan

Hardware-in-the-loop tests for the IFS08-CE-AMS firmware. Each test
exercises a specific behaviour documented in
[`ARCHITECTURE.md`](ARCHITECTURE.md), [`CAN_MAP.md`](CAN_MAP.md), or the
bootloader contract at
[`isc-fs/stm32-can-bootloader`](https://github.com/isc-fs/stm32-can-bootloader).
Tests are independent and can be re-run individually; the **Test
execution order** section below recommends a sequence so failures
surface in dependency order (boot before safety before FSM before
soak).

This document is the source of truth for the v1.1.0-bootloader
hardware-acceptance gate. Sign-off for issues
[#53](https://github.com/isc-fs/IFS08-CE-AMS/issues/53) and
[#54](https://github.com/isc-fs/IFS08-CE-AMS/issues/54) requires every
test in the [Acceptance criteria](#acceptance-criteria) table to be
green on the same firmware SHA.

---

## 1. Bench rig requirements

### 1.1 Hardware

| Item | Notes |
|---|---|
| Target board | STM32H733ZGTx (custom AMS PCB or Nucleo-H733ZG with the AMS pinout broken out) |
| Programmer | ST-Link V3 or V2-1 over SWD; `STM32_Programmer_CLI` on the host PATH |
| USB-CAN adapter | SocketCAN-compatible (canable, PCAN-USB FD with `pcan-driver`, Kvaser with kernel driver). Two channels preferred (one per bus); single-channel is OK if the test plan is run in two halves |
| FDCAN1 wiring | PD0/PD1 on the target, terminated 120 Ω at each end. Carries ACU traffic, AMS telemetry, **and the bootloader-trigger frame** (v1.2.0+; the trigger moved off FDCAN2 in #73). |
| FDCAN2 wiring | PB12/PB13, terminated 120 Ω. Reserved for the bootloader's flash workflow only; the app does not start FDCAN2 in v1.2.0. |
| BMS_LITE stack or LTC emulator | Real pack of 5 BMS_LITE modules (10 LTC6811-1, isoSPI daisy-chained) wired into the AMS SPI1 + LTC6820 master port; OR an LTC6811 emulator (e.g. Analog's eval kit, or a host-controlled rig that synthesises the chain replies) on the same connector. |
| isoSPI cabling | Twisted-pair to the first BMS_LITE; transformer-coupled on both ends. Must withstand cable pulls (HIL-057). |
| Logic analyser on SPI1 | PA4 (CS), PA5/6/7 (SCK/MISO/MOSI). Several Block D + Block F tests need it. |
| ADC current input | Analog source on PF7 (ADC3 ch3). A bench DAC or a calibrated voltage source 0–3.3 V |
| GPIO inputs | Switch / signal generator on PG7 (`Charge_Button`). The legacy `DIGITAL1` SDC sense on PE9 was retired in PR #117 — the AMS no longer GPIO-senses the SDC. |
| GPIO output read-back | DMM or logic analyser on PB6 (`RELAY_AIR_N`), PB5 (`RELAY_AIR_P`), PB7 (`RELAY_PRECHARGE`), PB4 (`AMS_OK`), PB9 (`TIM17_CH1` fan PWM). Pin map aligned with the v1.2 daughterboard schematic in PR #117. |
| Power | 5 V or 3.3 V regulated rail to the target; VBAT-only test capability for HIL-006 |

**HV must not be connected.** Every test below validates the MCU's
behaviour using the relay-control GPIO pins. The relay coils may be
populated for realism, but no high-voltage source / battery pack.

### 1.2 Tooling (host-side)

| Tool | Use |
|---|---|
| `arm-none-eabi-gcc 14.x` | Cross-compile the AMS app and the bootloader |
| `STM32_Programmer_CLI` | Flash sector 0 (bootloader) and sectors 1..6 (app) via ST-Link |
| `python-can` | Drive the BMS emulator and inject ACU-bus stimuli |
| `cansend` / `candump` | Quick one-off frames and bus capture |
| `screen` or `picocom` | UART2 telemetry capture (default 115200 8N1) |
| `arm-none-eabi-gdb` + OpenOCD | Fault-injection (hang MainTask, etc.), backtrace on hardfault |
| `scripts/check_flash_layout.py` | Pre-flight sanity for any custom build |

### 1.3 Fixture firmware

- **Bootloader image** built from `isc-fs/stm32-can-bootloader` `main`
  branch with `BL_NODE_ID` set to **any value ≠ 2** (so the BL's
  hardware filter doesn't accidentally match `0x002` boot-trigger
  frames). Confirm in `Core/Inc/bl_config.h` of that build.
- **AMS image** built from the firmware SHA under test (default:
  latest `dev`). `scripts/check_flash_layout.py build/AMS.elf` must
  PASS before the image is flashed.
- **BMS source**: either the real BMS_LITE pack (5 modules, 10 LTC6811-1,
  20 NTCs / module) or an LTC6811 chain emulator. The emulator must
  respond to the AMS-issued commands (ADCV → settle → RDCV[A-D];
  WRCOMM → STCOMM → ADAX(Gpio1) → RDAUXA; WRCFGA / RDCFGA) with
  PEC-correct replies, and must allow per-IC value injection for the
  Block D + Block F tests. Wire format reference:
  [`BMS_LTC6811.md`](BMS_LTC6811.md).

---

## 2. Test execution order

Run blocks in this order. A red light in an earlier block almost
always means later blocks will also fail; do not chase later failures
until the earlier ones are green.

1. **[Block A — Boot & bring-up](#block-a--boot--bring-up)** (HIL-001…009)
2. **[Block B — Safety supervisor](#block-b--safety-supervisor)** (HIL-010…019)
3. **[Block C — FSM](#block-c--fsm)** (HIL-020…029)
4. **[Block D — Communications](#block-d--communications)** (HIL-030…040)
5. **[Block E — Bootloader integration](#block-e--bootloader-integration)** (HIL-041…047)
6. **[Block F — Soak & fuzz](#block-f--soak--fuzz)** (HIL-050…055)

Each block has a "block precondition" listing what must already be
verified before running tests in that block.

---

## Block A — Boot & bring-up

**Block precondition**: bench rig wired per §1.1, both probes
enumerated, no firmware on the chip yet (or known image to be
replaced).

### HIL-001 — Power-on relays default open

**Goal**: prove invariant 2 (relays default to open on any reset)
holds without any software involvement.

| | |
|---|---|
| Steps | 1. Erase the entire flash with `STM32_Programmer_CLI -e all`.<br>2. Power-cycle the board.<br>3. Within 50 ms of power-on, measure PB6, PB5, PB7. |
| Pass | All three pins read 0 V (LOW). |
| Fail mode | Any pin asserted → either the GPIOs are not in their reset state (HW issue) or a stray default-flashed image is closing relays. |
| Capture | Multimeter / logic-analyser sample on PB5/6/7. |
| Duration | 2 min |

### HIL-002 — Bootloader programmed and responsive

**Goal**: the bootloader image is correctly flashed to sector 0 and
its CAN protocol responds.

| | |
|---|---|
| Steps | 1. Build the BL with `BL_NODE_ID=<chosen>`.<br>2. `STM32_Programmer_CLI -c port=SWD -w bootloader.hex 0x08000000`.<br>3. Power-cycle. Wait 3 s.<br>4. Send `DISCOVER` (TYPE=7, dst=0xF) on FDCAN2 with `cansend`. |
| Pass | BL responds with `DISCOVER` reply carrying its node ID, major, minor version. |
| Fail mode | No reply → BL didn't boot. WRP enabled accidentally, or BL crash early. Pull SWD, attach GDB, inspect. |
| Capture | `candump -tA can0` for 5 s after reset; expect BL DISCOVER reply within ~1 s (after auto-jump window expires without app present). |
| Duration | 5 min |

### HIL-003 — Application flashes via bootloader

**Goal**: the AMS image can be installed in-system via the BL's
FLASH_ERASE / FLASH_WRITE / FLASH_VERIFY protocol.

| | |
|---|---|
| Steps | 1. Build the AMS app from the SHA under test.<br>2. `scripts/check_flash_layout.py build/AMS.elf` → PASS.<br>3. Drive `can-flasher` (or hand-rolled `python-can` sequence) to:<br>&nbsp;&nbsp;&nbsp;&nbsp;a. CONNECT<br>&nbsp;&nbsp;&nbsp;&nbsp;b. FLASH_ERASE 0x08020000..0x080DFFFF<br>&nbsp;&nbsp;&nbsp;&nbsp;c. FLASH_WRITE in chunks ≤ 256 B until full image is in<br>&nbsp;&nbsp;&nbsp;&nbsp;d. FLASH_VERIFY with the expected CRC32 (computed host-side)<br>&nbsp;&nbsp;&nbsp;&nbsp;e. RESET |
| Pass | Every step ACKs. FLASH_VERIFY ACK includes the metadata write. |
| Fail mode | Any NACK → log the NACK code + offset. Most common: NACK on FLASH_WRITE if the BL is session-gated and we forgot CONNECT first. |
| Capture | Full `candump` trace tagged with timestamps. |
| Duration | 10 min |

### HIL-004 — App boots, reaches Start

**Goal**: post-flash reset hands off cleanly to the app and the FSM
comes up in `Start`.

| | |
|---|---|
| Preconditions | HIL-003 passed (app flashed). |
| Steps | 1. `screen /dev/cu.usbmodemX 115200` attached.<br>2. Hard-reset the target.<br>3. Wait 2 s. |
| Pass | UART emits `AMS s=S …` line (state=Start = 'S') within 1 s of reset. PB5/6/7 read LOW (relays open per HIL-001 + initial fan duty 0 % per state_task `FanDuty[Start]`). |
| Fail mode | UART silent → app crashed or VTOR misaligned. Pull SWD, read stack, check `SCB->VTOR`. |
| Capture | UART log file, GPIO read-back. |
| Duration | 2 min |

### HIL-005 — SCB->VTOR set by app (direct-flash path)

**Goal**: prove the app's `USER CODE BEGIN 1` sets `SCB->VTOR` itself
so direct-flash workflows (no BL) also work.

| | |
|---|---|
| Steps | 1. With BL still in sector 0, attach GDB, halt at `main()` very early.<br>2. Force `SCB->VTOR = 0x08000000` (wrong value).<br>3. Step past `USER CODE BEGIN 1`.<br>4. Read `SCB->VTOR`. |
| Pass | After step 3, `SCB->VTOR` reads `0x08020000`. |
| Fail mode | Reads anything else → USER CODE BEGIN 1 didn't execute, or got optimised out. |
| Capture | GDB transcript. |
| Duration | 5 min |

### HIL-006 — ErrorLatch persistence across resets

**Goal**: confirm the ERROR latch survives software / watchdog resets
(per invariant 5) and is cleared only by full backup-domain power loss.

| | |
|---|---|
| Preconditions | BL + app installed; AMS comes up in `Start` per HIL-004. |
| Steps | 1. Attach GDB while running in Start. Write `RTC->BKP1R = 0xA115EE51` (BkpErrorMagic).<br>2. `NVIC_SystemReset()` from GDB.<br>3. Read UART. |
| Pass | UART says `s=E` on the post-reset boot. |
| Steps cont. | 4. From a healthy `Error`, power-cycle (full VDD removal, VBAT only if VBAT is wired — otherwise full power-down).<br>5. Read UART. |
| Pass | UART says `s=S` (latch cleared because backup domain lost power). |
| Fail mode | Step 3 still reads `s=S` → backup-domain access not unlocked (`HAL_PWR_EnableBkUpAccess` missing or wrong register). |
| Capture | GDB + UART. |
| Duration | 10 min |

### HIL-007 — Sector 0 WRP (optional, production gate)

**Goal**: the BL's WRP-protect-sector-0 path actually blocks accidental
overwrites of the BL.

| | |
|---|---|
| Steps | 1. With BL installed, drive `CMD_OB_APPLY_WRP` over CAN per the BL protocol (token + sector-bitmap arg).<br>2. After the BL-driven reset, attempt `STM32_Programmer_CLI -w wrongimage.bin 0x08000000`. |
| Pass | The write fails with a flash-protection error. The BL still boots after the failed write attempt. |
| Skip if | Not running production-grade testing (development bench leaves WRP off). |
| Duration | 15 min |

### HIL-008 — Flash-layout sanity on the installed image

**Goal**: the image flashed in HIL-003 actually conforms to the layout
spec, not just the .elf on the host.

| | |
|---|---|
| Steps | 1. After HIL-003, dump flash via `STM32_Programmer_CLI -r dump.bin 0x08020000 0xC0000`.<br>2. Verify the first 4 bytes (`dump.bin[0..3]`) are the initial-MSP value (will be in DTCM range, 0x2002…).<br>3. Verify `dump.bin[4..7]` is the reset-vector address (will be in app text range, 0x080203…). |
| Pass | Both fields look sensible (MSP points into DTCM, reset vector into app text). |
| Fail mode | Garbage → the FLASH_WRITE chunking got out of order or skipped sectors. |
| Duration | 5 min |

### HIL-009 — LTC6820 wakeup pulse + chain-length discovery on boot

**Goal**: prove `App_InitTask` wakes the LTC6811 daisy-chain and
discovers exactly `LtcChainLength = 10` ICs before the safety
supervisor's first tick. Replaces the legacy FDCAN2 filter test
(v1.2.0+ the BMS no longer rides on CAN, so the only filter that
matters is FDCAN1's, exercised passively by every other test in this
block).

| | |
|---|---|
| Preconditions | Real BMS_LITE stack of 5 modules wired into the AMS isoSPI port, OR a 10-IC LTC6811 emulator. PA4 CS available on a logic analyser. |
| Steps | 1. Attach LA to PA4 (CS) and SPI1 SCK; arm on falling-edge of CS, 5 ms timebase.<br>2. Power-cycle the AMS. Wait 2 s.<br>3. Read GDB-visible `g_state_telemetry` (should be `'S'` = Start) and `BmsService::instance().snapshot().ltc_online_mask`. |
| Pass | LA shows ten CS-low pulses ≥ 10 µs each on power-up (the chain wakeup train). After the train, one RDCFGA round-trip clocks ~84 bytes. `ltc_online_mask` reads `0x3FF` (all 10 ICs PEC-clean). `g_state_telemetry == 'S'`. |
| Fail mode | Fewer than 10 wakeup pulses → `LtcChainLength` mismatch in code. RDCFGA absent → `App_InitTask` hit the early-exit. Mask not `0x3FF` → ERROR latched; HIL-056 covers that failure mode in its own test. |
| Capture | LA screenshot of CS + SCK; GDB transcript. |
| Duration | 15 min |

---

## Block B — Safety supervisor

**Block precondition**: HIL-004 passed (AMS boots and runs).

### HIL-010 — MainTask 10 ms cadence

**Goal**: MainTask actually runs at its declared 10 ms period.

| | |
|---|---|
| Steps | 1. (One-time firmware tweak in a debug build, or via the existing telemetry timestamp) Capture `osKernelGetTickCount` deltas across MainTask iterations.<br>2. Run for 60 s. |
| Pass | Mean period = 10 ms ± 1 ms; max period ≤ 12 ms; no period > 15 ms. |
| Fail mode | Long periods → priority inversion, or another task hogging CPU. |
| Capture | Time-series log of periods. |
| Duration | 10 min |

### HIL-011 — IWDG resets the chip if MainTask hangs

**Goal**: invariant 4 — watchdog discipline enforces a HW reset within
~100 ms if MainTask stops feeding it.

| | |
|---|---|
| Steps | 1. Attach GDB. Set a breakpoint inside `MainTask::run()`.<br>2. Hit the breakpoint, halt the CPU.<br>3. Wait 200 ms in wall-clock time.<br>4. Continue. |
| Pass | The chip resets within ~100 ms of the halt. After resume, the reset is observable (UART shows fresh boot line; reset-cause register has IWDG bit set). |
| Fail mode | Chip continues running past 200 ms → watchdog disabled (check `hiwdg` init), or refresh path is on the wrong branch. |
| Capture | GDB transcript + reset-cause register read. |
| Duration | 10 min |

### HIL-012 — Watchdog reset re-opens relays

**Goal**: after a watchdog reset (from HIL-011 or any synthetic), the
chip boots through `MX_GPIO_Init` and re-asserts PB5/6/7 to PIN_RESET
within milliseconds.

| | |
|---|---|
| Steps | 1. Start in Run (need a full happy-path setup, see Block C). Verify PB5/PB6 are HIGH.<br>2. Halt MainTask via GDB. Wait for IWDG reset.<br>3. Probe PB5/6/7 immediately after reset. |
| Pass | All three read LOW within 20 ms of the reset edge. |
| Fail mode | Pins stay HIGH → MX_GPIO_Init isn't writing them before pin direction is configured, or boot is slow. |
| Capture | Logic analyser on PB5/6/7 + a reset-edge signal (e.g. NRST). |
| Duration | 10 min |

### HIL-013 — FORCE_ERROR event flag opens AIRs

**Goal**: any task can post `ForceError` on `safety_events` and the
supervisor responds within one period.

| | |
|---|---|
| Steps | 1. Bring AMS up to Run.<br>2. Via GDB, call `osEventFlagsSet(safety_eventsHandle, 0x1)` (ForceError = bit 0).<br>3. Within 15 ms, read PB5/6/7. |
| Pass | All three pins are LOW. UART next line says `s=E`. `RTC->BKP1R` reads `0xA115EE51`. |
| Fail mode | Pins still HIGH after 50 ms → MainTask not consuming the flag, or relay write path broken. |
| Duration | 5 min |

### HIL-014 — Cell undervoltage trips ERROR

**Goal**: a single cell below `CellUnderVoltageMv` (2800 mV) trips the predicate
set within one V-poll cycle + one MainTask period.

| | |
|---|---|
| Steps | 1. AMS in Run with healthy LTC6811 chain (real BMS_LITE pack or emulator).<br>2. Force module 2 / cell 5 to ~2700 mV on the emulator's next ADCV cycle (or, on a real pack, GDB-inject `cell_mV[2][5] = 2700` directly into `BmsService` between two polls).<br>3. Within 280 ms of the injection (one V-poll period + one Safety period), sample PB5/6/7 + read UART. |
| Pass | Pins LOW within 280 ms. UART says `s=E`. |
| Fail mode | Pins stay HIGH after 500 ms → threshold value mismatch, or `BmsService::update_from_ltc_response` dropped the value via PEC. |
| Capture | LA + GPIO scope. |
| Duration | 5 min |

### HIL-015 — Cell overvoltage trips ERROR

Same shape as HIL-014, value = 4250 mV (above `CellOverVoltageMv` = 4200).

### HIL-016 — Cell overtemperature trips ERROR

Same shape; temp frame with temp[10] = 65 °C (above `CellOverTempC` = 60).

### HIL-017 — BMS module staleness trips ERROR

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Disconnect module 3's isoSPI input (cable pull) so chain slots 6 and 7 stop replying with PEC-clean data.<br>3. Wait for `BmsStaleMs` (1500 ms) + one V-poll period (250 ms). |
| Pass | Pins LOW within ~1.8 s of the cable pull. UART says `s=E`. `g_ltc_pec_err_count[6]` and `[7]` climb, but `last_rx_tick[3]` stops advancing — that's what trips the freshness predicate. |
| Fail mode | Pins stay HIGH after 2.5 s → freshness check broken, or `update_from_ltc_response` is advancing `last_rx_tick` even on PEC-fail. |
| Duration | 10 min |

### HIL-018 — Current sensor staleness trips ERROR

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Inject a constant voltage on PF7 ≈ 2.5 V (zero current) for the warm-up.<br>3. Halt CurrentSensorTask via GDB (it's the only writer to `last_update_tick`).<br>4. Wait `IStaleMs` (200 ms) + 50 ms. |
| Pass | Pins LOW within 300 ms. UART says `s=E`. |
| Fail mode | Pins stay HIGH → freshness threshold mis-applied. |
| Duration | 5 min |

### HIL-019 — *(retired)* SDC open trips ERROR

The `sdc_closed` GPIO-sense predicate was retired in PR #117: the
v1.2 daughterboard doesn't route a dedicated SDC sense input
(invariant 8 in [`ARCHITECTURE.md`](ARCHITECTURE.md) §1). The AMS is
*part of* the SDC via `AMS_OK` (PB4), not a sensor of it. The
equivalent coverage now lives in HIL-014 (`force_error_set` trips
ERROR) and HIL-013 (cell undervoltage) — pick either to verify the
"any-state → Error via predicate" path.

| | |
|---|---|
| Status | Retired — no longer applicable on v1.2 daughterboard |
| Replacement | HIL-014 (force-error injection) or HIL-013 (cell predicate) |

---

## Block C — FSM

**Block precondition**: Block B passed. BMS emulator emits healthy
frames continuously.

### HIL-020 — Start → Precharge on start button

| | |
|---|---|
| Steps | 1. Verify AMS in Start (`s=S`).<br>2. Send 0x600 standard frame with byte 0 = 1 on FDCAN1.<br>3. Within 50 ms, read PB6 and PB7; UART next line. |
| Pass | PB6 (AIR-) HIGH, PB7 (Precharge) HIGH, PB5 (AIR+) LOW. UART says `s=P`. |
| Fail mode | Wrong pin state → relay-action flag mismatch in `state_machine.hpp`. |
| Duration | 5 min |

### HIL-021 — Start → Charge on charger detect

| | |
|---|---|
| Steps | 1. AMS in Start.<br>2. Send extended frame `0x18FF50E7` on FDCAN1 (any payload).<br>3. Within 50 ms, sample pins; UART. |
| Pass | PB6 HIGH, PB5 HIGH, PB7 LOW. UART says `s=C`. |
| Duration | 5 min |

### HIL-022 — Precharge → Transition on DC bus target

| | |
|---|---|
| Steps | 1. AMS in Precharge (after HIL-020 sequence).<br>2. Emit 0x100 ext frame on FDCAN1 with `dc_bus_V` = 350 (≥ 0.95 × pack_V, where pack is healthy at ~356 V from emulator).<br>3. Within 50 ms, sample pins. |
| Pass | PB5 HIGH (AIR+ closed), PB7 LOW (Precharge open), PB6 still HIGH. UART says `s=T`. |
| Duration | 5 min |

### HIL-023 — Precharge timeout → ERROR

| | |
|---|---|
| Steps | 1. AMS in Precharge.<br>2. Continuously emit 0x100 with `dc_bus_V` = 50 (well below target).<br>3. Wait `PrechargeMaxMs` (1500 ms) + 50 ms. |
| Pass | All pins LOW within 1.6 s. UART says `s=E`. `RTC->BKP1R` = `0xA115EE51`. |
| Duration | 5 min |

### HIL-024 — Transition → Run after hold

| | |
|---|---|
| Steps | 1. AMS in Transition (after HIL-022).<br>2. Keep 0x100 emitting `dc_bus_V` ≥ 350.<br>3. Wait `TransitionHoldMs` (100 ms) + 30 ms.<br>4. Sample pins + UART. |
| Pass | PB6 HIGH, PB5 HIGH, PB7 LOW. UART says `s=R`. Fan duty PB9 PWM = 40 % (measure with scope). |
| Duration | 5 min |

### HIL-025 — Transition voltage drop → ERROR

| | |
|---|---|
| Steps | 1. AMS in Transition (immediately after HIL-022).<br>2. Before the hold elapses, drop `dc_bus_V` in 0x100 to 100.<br>3. Within 50 ms, sample pins. |
| Pass | All pins LOW. UART says `s=E`. |
| Duration | 5 min |

### HIL-026 — Run is terminal (charger toggle ignored)

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Send `0x18FF50E7` charger-detect frame.<br>3. Wait 200 ms.<br>4. Sample pins + UART. |
| Pass | Pins unchanged from Run state (PB6 HIGH, PB5 HIGH, PB7 LOW). UART continues `s=R`. |
| Fail mode | UART shows `s=C` → the Run→Charge transition wasn't fully removed in `fsm::step`. |
| Duration | 5 min |

### HIL-027 — Charge is terminal (charger toggle ignored)

| | |
|---|---|
| Steps | 1. AMS in Charge.<br>2. Stop emitting charger-detect frames for 2 s (would have set `charger_detected=false` in a non-sticky model).<br>3. Sample pins + UART. |
| Pass | Pins unchanged, UART continues `s=C`. |
| Duration | 5 min |

### HIL-028 — ERROR is sticky within a boot

| | |
|---|---|
| Steps | 1. Trip ERROR via any predicate (HIL-013 cell UV or HIL-014 force-error).<br>2. Clear the fault (restore healthy cell value or clear `force_error_set`).<br>3. Wait 5 s. |
| Pass | UART continues `s=E`. Pins all LOW. |
| Duration | 5 min |

### HIL-029 — ERROR persists across software reset

| | |
|---|---|
| Steps | 1. After HIL-028, while in ERROR, send the boot-trigger? **No — that's a separate path.** Use `NVIC_SystemReset` via GDB.<br>2. Wait for boot. |
| Pass | UART comes up immediately in `s=E`. (Tests the BKP1R latch + boot-time read in App_InitTask.) |
| Steps cont. | 3. Power-cycle (full VDD removal). |
| Pass | UART comes up in `s=S` (latch cleared because backup domain lost power). |
| Duration | 10 min |

---

## Block D — Communications

**Block precondition**: HIL-009 (filters) and HIL-020 (basic FSM transitions) passed.

### HIL-030 — ADCV broadcast cadence on SPI1

**Goal**: confirm `BmsPollTask::run_voltage_poll` issues an ADCV
broadcast every 250 ms, well within the budget.

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. LA on PA4 (CS) + SPI1 SCK for 5 s. Decode SPI as 8-bit MSB-first, Mode 3.<br>3. Look for the ADCV command (first two bytes match `adcv_cmd(Norm7kHz, false, All)` = `0x0360`). |
| Pass | ADCV appears 4× per second (cadence 250 ms ± 25 ms). After each ADCV the bus is silent for ~3 ms, then four RDCV*-shaped 84-byte reads happen back-to-back. `g_bms_volt_poll_ms` (GDB) sits ≤ 10 ms; `g_bms_volt_poll_max` ≤ 15 ms across the run. |
| Fail mode | Slower / irregular → BmsPollTask priority inversion, osTimer mis-configured, or one of the four `read_register_group` calls is failing and aborting the cycle. |
| Capture | LA screenshot; GDB snapshot of `g_bms_volt_poll_ms` / `_max`. |
| Duration | 10 min |

### HIL-031 — Temperature mux-sweep cadence

**Goal**: confirm the 20-channel ADG731 sweep runs every 500 ms and
that all 20 mux selections actually leave the LTC's GPIO port.

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. LA on PA4 + SCK for 5 s.<br>3. For each 500 ms window, count complete `WRCOMM → STCOMM → ADAX(Gpio1) → RDAUXA` cycles. |
| Pass | 20 cycles per window, ≥ 9 windows over 5 s (= 500 ms cadence ± 100 ms). Each cycle:<br>· WRCOMM transmits `cmd(2) + PEC(2) + 8 × LtcChainLength = 84 B`<br>· STCOMM transmits `cmd(2) + PEC(2) + 30 dummy B = 34 B`<br>· ADAX is 4 B<br>· RDAUXA is `cmd(4) + 80 B` = 84 B. |
| Fail mode | Fewer than 20 cycles/window → one mux step is stalling or aborting. Investigate which channel via `g_ltc_spi_err_count`. |
| Capture | LA + `g_ltc_spi_err_count` before / after. |
| Duration | 15 min |

### HIL-032 — RDCV[A-D] decode round-trip

**Goal**: a synthesised cell-voltage chain reply lands in
`BmsService::snapshot().cell_mV` at the right slot.

| | |
|---|---|
| Steps | 1. With the BMS emulator (or a real pack at known voltages) producing `cell_mV[2][7] = 3711`, `cell_mV[4][18] = 3733`, AMS in Run.<br>2. GDB-read `BmsService::instance().snapshot().cell_mV[2][7]` and `[4][18]`. |
| Pass | Within ±5 mV (LTC6811 LSB is 100 µV, decoder rounds to mV). `ltc_online_mask` = `0x3FF`. |
| Fail mode | Wrong slot → cell mapping in `update_from_ltc_response` is off; cross-check against `BMS_LTC6811.md §2`. Wrong value → endianness / 100 µV→mV conversion error. |
| Duration | 10 min |

### HIL-033 — RDAUXA decode round-trip (temperature)

**Goal**: feed a known AUX1 voltage on one LTC channel and confirm
the °C value lands in `cell_tempC`.

| | |
|---|---|
| Steps | 1. With a calibrated bench DAC (or pot) drive the LTC's GPIO1 input on module 2 LTC_1 directly with 1.5 V (= 25 °C nominal under the placeholder Beta-model constants).<br>2. AMS in Run; wait 1 s for a full temperature sweep.<br>3. GDB-read `cell_tempC[2][k]` for the temp-index `k` whose `Adg731ChannelMap[k]` matches the mux address you selected. |
| Pass | Reads 25 °C ± 2 °C. |
| Fail mode | Wrong slot → channel-index mapping (`Adg731ChannelMap`) wrong or LTC_1 vs LTC_2 swap. Wrong value → β / R₂₅ / V_ref off; tune per `COMMISSIONING.md §3b`. |
| Duration | 20 min |

### HIL-034 — PEC-error counter rises on injected corruption

**Goal**: corrupting a single RDCV* reply on the wire increments
`g_ltc_pec_err_count[ic]` but does NOT cause a state transition.

| | |
|---|---|
| Steps | 1. AMS in Run; healthy chain.<br>2. With a benchtop LTC6811 emulator, schedule one cell-voltage reply for IC 4 to be returned with its last PEC byte XOR `0x01`.<br>3. GDB-read `g_ltc_pec_err_count[4]` before / after, and `g_state_telemetry`. |
| Pass | `g_ltc_pec_err_count[4]` incremented by exactly 1. `g_state_telemetry` unchanged (still `'R'`). `last_rx_tick[2]` (module 2 = IC 4's module) did NOT advance for that cycle; next clean cycle advances it normally. |
| Fail mode | State transitioned → PEC error is incorrectly tripping `FORCE_ERROR`. Counter didn't move → decoder isn't checking PEC. |
| Duration | 10 min |

### HIL-035 — ACU 0x100 DC bus parsing

| | |
|---|---|
| Steps | 1. Send 0x100 ext frame with payload `0x2C 0x01 0 0 0 0 0 0` (little-endian → 300 V).<br>2. GDB-read `VehicleService::instance().snapshot().dc_bus_V`. |
| Pass | Reads 300. |
| Duration | 5 min |

### HIL-036 — ACU 0x600 start button parsing

| | |
|---|---|
| Steps | 1. Send 0x600 with byte 0 = 1.<br>2. GDB-read `VehicleService::instance().snapshot().start_button`. |
| Pass | Reads 1. |
| Duration | 5 min |

### HIL-037 — ACU charger-detect frame

| | |
|---|---|
| Steps | 1. Send `0x18FF50E7` ext.<br>2. GDB-read `CurrentService::instance().snapshot().charger_detected`. |
| Pass | Reads true. |
| Duration | 5 min |

### HIL-038 — ACU TX min cell V cadence (Run state)

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. `candump` FDCAN1 ext-ID filter on `0x12C`. |
| Pass | Frame appears every 500 ms ± 100 ms. Payload bytes [0..1] decode to the BMS-emulator's min cell V (big-endian). |
| Duration | 5 min |

### HIL-039 — ACU TX current cadence (Run state)

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Inject 2.4 V on PF7 (≈ +17.5 A discharge).<br>3. `candump` FDCAN1 std-ID filter on `0x450`. |
| Pass | Frame appears every 250 ms ± 50 ms. Payload [1] approximately 17 (= 17 A & 0xFF). |
| Duration | 5 min |

### HIL-040 — Charger-suppress: TX 0x12C stops in Charge; isoSPI polling continues

| | |
|---|---|
| Steps | 1. AMS in Charge (after HIL-021 / HIL-037).<br>2. `candump` FDCAN1 for 2 s. LA on PA4 / SCK for the same 2 s. |
| Pass | No 0x12C ext frames on FDCAN1 (legacy min-V telemetry suppressed when charging). 0x450 (current) still emits every 250 ms. isoSPI traffic continues unaffected — ADCV cadence (HIL-030) and mux sweep (HIL-031) are unchanged; balancing now lives on the LTC chain so the legacy "no FDCAN2 BMS poll during charge" rule no longer applies. |
| Duration | 5 min |

---

## Block E — Bootloader integration

**Block precondition**: Block A passed (BL + app installed). Block B
and Block C optionally verified — they make this block easier to debug.

### HIL-041 — Boot-trigger round-trip

**Goal**: the headline use case. Running app receives the boot-trigger
frame, opens AIRs, reboots, BL takes over.

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Probe PB5/6/7 + UART.<br>3. `cansend can0 002#B007AD11` (FDCAN1 — the trigger moved off FDCAN2 in v1.2.0 (#73)). |
| Pass | Within 15 ms: PB5/6/7 all LOW (Relays::open_all fired). Within 1 s: BL DISCOVER reply on FDCAN2 confirms BL is alive (the BL still operates on FDCAN2 after the magic-reset; only the in-band trigger moved). |
| Fail mode | Pins stay HIGH → boot-trigger not matching, AcuCanTask not dispatching, or `request_reboot` is not actually running.<br>BL doesn't respond → BL didn't see the BKP0R magic, or BL is in auto-jump and hands back to app. |
| Capture | Logic analyser on PB5/6/7 + NRST + `candump` of FDCAN1 (trigger) and FDCAN2 (BL reply). |
| Duration | 10 min |

### HIL-042 — Wrong-bus trigger ignored

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Send the exact `002#B007AD11` payload **on FDCAN2** (the app no longer listens on FDCAN2 in v1.2.0 (#73); only the bootloader claims the bus post-reset). |
| Pass | No reboot, no relay change, UART continues to emit Run telemetry. |
| Fail mode | Reboot happens → the `CanFrame::bus` check in `matches_trigger` is broken, or FDCAN2 RX-FIFO0 notification got re-activated somewhere in app code. |
| Duration | 5 min |

### HIL-043 — Wrong-payload trigger ignored

Parametric. Repeat with each of these payloads on FDCAN1 id 0x002, DLC=4:

| Payload | Description |
|---|---|
| `00 07 AD 11` | byte 0 zeroed |
| `B0 00 AD 11` | byte 1 zeroed |
| `B0 07 00 11` | byte 2 zeroed |
| `B0 07 AD 00` | byte 3 zeroed |
| `FF FF FF FF` | all wrong |
| `B0 07 AD 12` | byte 3 off by one |

**Pass**: none cause a reboot. AMS continues Run telemetry.

**Duration**: 5 min.

### HIL-044 — Wrong-DLC trigger ignored

| | |
|---|---|
| Steps | Send `002#B007AD` (DLC=3), then `002#B007AD1100` (DLC=5), then `002#B007AD11FFFFFFFF` (DLC=8). |
| Pass | None cause a reboot. |
| Duration | 5 min |

### HIL-045 — Pre-reboot relay-open timing

**Goal**: prove `Relays::open_all()` runs **before** `NVIC_SystemReset`,
so AIRs are open for the entire reset window.

| | |
|---|---|
| Steps | 1. AMS in Run. PB5/PB6 HIGH.<br>2. Send the trigger.<br>3. Capture PB5/6/7 + NRST on a logic analyser with µs resolution. |
| Pass | PB5/6/7 fall to LOW **at least 5 ms before** NRST asserts (the `osDelay(10)` in `request_reboot` enforces this). |
| Fail mode | Pins fall after NRST → the order in `request_reboot` is wrong. |
| Capture | LA trace. |
| Duration | 10 min |

### HIL-046 — BKP0R cleared by BL (one-shot)

| | |
|---|---|
| Steps | 1. After HIL-041, AMS is in BL mode awaiting commands.<br>2. From host, send `RESET` opcode (or just power-cycle).<br>3. Wait for boot. |
| Pass | The reboot brings up the **app**, not the BL. (BL cleared BKP0R on consuming the magic; subsequent reset has no trigger.) |
| Fail mode | Boots into BL again → BL isn't clearing BKP0R, or the app is re-writing the magic somehow. |
| Duration | 5 min |

### HIL-047 — Flood of malformed triggers + one valid

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. From host, hammer FDCAN2 with 100 frames of `002#B007AD12` (one byte off) over 1 s.<br>3. Send one valid `002#B007AD11`. |
| Pass | The 100 malformed frames cause **zero** reboots. The single valid frame reboots within 15 ms. |
| Fail mode | Any malformed frame causes a reboot → magic check is incomplete. |
| Capture | UART (count of bad-frame counter increments) + LA on PB5/6/7. |
| Duration | 10 min |

---

## Block F — Soak & fuzz

**Block precondition**: every test in Blocks A–E green on the same SHA.

### HIL-050 — 30-minute soak in Run

| | |
|---|---|
| Steps | 1. AMS in Run with healthy BMS, healthy current sensor.<br>2. Capture telemetry frames 0x4A0/4A1/4A2 + `candump` to file for 30 minutes. |
| Pass | No resets. Telemetry frame cadence stays at 500 ms ± 20 ms across the entire run. No `g_*_dropped_*` counter increases beyond a low background (< 1 per minute). State byte in 0x4A0 remains `0x03` (Run). |
| Fail mode | Reset → check reset-cause reg (IWDG? brownout? hardfault?). Drift in cadence → priority inversion. |
| Capture | CAN trace (≥ 30 min of telemetry). |
| Duration | 35 min |

### HIL-051 — Boot-trigger reliability

| | |
|---|---|
| Steps | Repeat HIL-041 ten times consecutively (each round: bring up to Run via #020→#024, trigger reboot, BL respond, re-flash optional). |
| Pass | All 10 round-trips succeed; no hung state requires manual power-cycle. |
| Duration | 30 min |

### HIL-052 — isoSPI fuzz (DEPRECATED for v1.2.0+)

> Originally a CAN-bus fuzz on FDCAN2; the BMS no longer rides on CAN
> in v1.2.0. The equivalent stress is HIL-058 (PEC-error storm) plus
> HIL-057 (cable pull). Kept here under the legacy ID so historic
> sign-offs stay readable; do not re-run.

### HIL-053 — ACU bus fuzz

Same shape on FDCAN1 with extended + standard IDs.

### HIL-054 — Brownout recovery

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Drop VDD to 1.5 V briefly (200 ms), restore to 3.3 V.<br>3. Observe behaviour. |
| Pass | AMS resets (BOR pinned the chip), boots fresh, comes up in Start (or Error if `BKP1R` survived — VBAT-dependent). |
| Duration | 10 min |

### HIL-055 — *(retired)* SDC chatter

Retired alongside HIL-019: no SDC GPIO-sense input on the v1.2
daughterboard. The equivalent "predicate-thrash" coverage is now
HIL-013-chatter (rapidly toggling a cell voltage across the UV
threshold), HIL-014-chatter (toggling `force_error_set`), and the
balance-on/off boundary in HIL-022/023.

| | |
|---|---|
| Status | Retired — no SDC GPIO sense on v1.2 daughterboard |
| Replacement | HIL-014 force-error chatter, or HIL-013 cell-UV chatter |

### HIL-056 — Chain-length mismatch on boot

**Goal**: missing module on power-up trips ERROR before the safety
supervisor's first tick. This is the boot-time invariant the v1.2.0
firmware enforces in `App_InitTask`.

| | |
|---|---|
| Preconditions | Real BMS_LITE pack OR LTC6811 emulator. |
| Steps | 1. Disconnect module 5's isoSPI input cable (or program the emulator to leave chain slots 8 + 9 silent).<br>2. Power-cycle the AMS. Wait 1 s.<br>3. Read GDB-visible `g_state_telemetry`, `RTC->BKP1R`, and PB5/6/7. |
| Pass | `g_state_telemetry == 'E'` (= Error). `RTC->BKP1R == 0xA115EE51`. PB5/6/7 all LOW within 1 s of power-on. Re-attach the cable + power-cycle once more → backup-domain power gone → comes up in Start (latch cleared). |
| Fail mode | Boot to Start with 8 ICs discovered → `count_pec_valid_segments` is forgiving where it shouldn't be. Stays in Start past 2 s → `App_InitTask` skipped the chain-length gate. |
| Capture | GDB transcript + LA on PA4 + UART log. |
| Duration | 15 min |

### HIL-057 — isoSPI cable unplugged mid-Run

**Goal**: complement to HIL-017. Yanking the chain while running
trips ERROR through the freshness predicate (not through the boot
gate, which is one-shot).

| | |
|---|---|
| Steps | 1. AMS in Run, all 5 modules online.<br>2. Pull the master-end isoSPI cable.<br>3. Within `BmsStaleMs + 250 ms` = 1750 ms, sample PB5/6/7 + UART. |
| Pass | Pins LOW within 1.8 s; UART says `s=E`. `g_ltc_pec_err_count[*]` and `g_ltc_spi_err_count` both rise during the window. |
| Fail mode | Pins stay HIGH past 2.5 s → freshness predicate broken or `last_rx_tick` is being touched by something other than a clean poll. |
| Duration | 10 min |

### HIL-058 — PEC-error storm: counters climb, no spurious state change

**Goal**: confirm the architecture's "transient PEC noise must not
trip FORCE_ERROR" invariant against a flood of bad PECs.

| | |
|---|---|
| Steps | 1. AMS in Run. Healthy chain.<br>2. With an LTC6811 emulator (or signal-injection rig that XORs one byte on every reply), corrupt **every** RDCV*/RDAUXA reply for 5 s.<br>3. Throughout: GDB-snapshot `g_state_telemetry` every 100 ms; record `g_ltc_pec_err_count` totals. |
| Pass | `g_state_telemetry` stays `'R'` for the first ~1.5 s (= `BmsStaleMs`), then transitions to `'E'` via the freshness predicate (no module's `last_rx_tick` could advance). `g_ltc_pec_err_count[*]` total climbs by ~80 (= 4 groups × 10 ICs × ~2 polls). No hardfault, no IWDG reset. |
| Fail mode | Transition to `'E'` happens earlier than 1.5 s → some path other than freshness is tripping. Hardfault → driver doesn't tolerate corrupt input. |
| Duration | 15 min |

### HIL-059 — ADG731 mux address bit-order

**Goal**: confirm `ltc6811::pack_adg731_select` matches the
datasheet's 8-bit serial format (EN bit, A4..A0, don't-cares).

| | |
|---|---|
| Steps | 1. Connect a calibrated bench resistor (e.g. 10 kΩ, gives V_aux ≈ 1.5 V) to the LTC_1 mux input wired to ADG731 channel 5 (= S6 on the schematic, → temp-index 5 in `Adg731ChannelMap`).<br>2. AMS in Run. Wait 1 s for a full mux sweep.<br>3. GDB-read `cell_tempC[m][5]` for the module whose LTC_1 carries the rigged channel. |
| Pass | Reads ~25 °C (matches the 10 kΩ NTC-equivalent at room temperature). |
| Fail mode | Reads 25 °C on the wrong slot index (e.g. slot 4 or 6) → mux-address packing has a bit-flip vs the ADG731 truth table. |
| Capture | LA on the LTC GPIO/COMM pin during STCOMM; verify the 8-bit shift contains `1<<7 | (5<<1)` = `0x8A`. |
| Duration | 20 min |

### HIL-060 — Balancing FET activation in Charge

**Goal**: the full WRCFGA→RDCFGA round-trip — an unbalanced cell in
Charge gets its DCC bit set, observable on the next round-trip read.

| | |
|---|---|
| Steps | 1. AMS in Charge (after HIL-021/HIL-037), all modules healthy.<br>2. With the emulator, set `cell_mV[2][7] = 4150` while every other cell sits at 4100 mV. Wait 2 s (one balance window + one V-poll).<br>3. Issue an RDCFGA over the chain (via GDB invoking `Bus::read_register_group(pack_command(CmdRDCFGA), …)`).<br>4. Decode the reply: chain slot 4 (module 2 LTC_1) should have `DCC[8]` set in CFGR4. |
| Pass | DCC bit for cell channel 8 of chain slot 4 reads `1`. Other DCC bits across the chain read `0`. `g_balance_cycles_active` is incrementing. |
| Fail mode | DCC bit set on the wrong cell channel → cell-to-DCC mapping in `maybe_run_balance_update` is off (cross-check §8 of `BMS_LTC6811.md`). Bit unset → balance policy lockout (check `max_tempC`, `BalanceDeltaMv`). |
| Duration | 25 min |

### HIL-061 — Balancing inhibited above BalanceTempMax

**Goal**: thermal-lockout rule — even with imbalance, no DCC bits
should be set once `max_tempC > BalanceTempMax`.

| | |
|---|---|
| Steps | 1. Start from HIL-060's state: one cell at 4150 mV, DCC bit set.<br>2. Heat one NTC on any module past `BalanceTempMax` (default 50 °C) with a hot-air gun. Wait for the next temperature sweep + balance window (1 s).<br>3. RDCFGA round-trip; decode DCC bits across the chain. |
| Pass | All DCC bits read `0`. UART continues `s=C`. `g_balance_cycles_total` keeps incrementing (we did send a WRCFGA), but `g_balance_cycles_active` does NOT increase during the heated window. |
| Fail mode | DCC bit still set → `compute_mask`'s thermal lockout isn't firing, or `max_tempC` isn't being recomputed. |
| Duration | 20 min |

---

## Acceptance criteria

Two gates apply to this document, stacked on top of each other.

### v1.1.0-bootloader gate (issues #53 and #54)

The following tests **must** pass on the same firmware SHA:

| Test | Why it's mandatory |
|---|---|
| HIL-001 | Invariant 2 (relays open on power-up) |
| HIL-002 | BL is alive |
| HIL-003 | App flashes via BL |
| HIL-004 | App boots, reaches Start |
| HIL-005 | App's `SCB->VTOR` works |
| HIL-006 | ErrorLatch persistence |
| HIL-008 | Image LMA matches sector 1 |
| HIL-009 | Filters admit every expected frame |
| HIL-011 | IWDG enforces safety supervisor liveness |
| HIL-012 | Watchdog reset re-opens relays |
| HIL-013 | FORCE_ERROR opens AIRs |
| HIL-014 | Cell UV trips |
| HIL-019 | *(retired)* SDC sense predicate removed in PR #117; covered by HIL-013 / HIL-014 |
| HIL-020 | Start → Precharge |
| HIL-021 | Start → Charge |
| HIL-024 | Transition → Run |
| HIL-028 | Error stickiness |
| HIL-029 | ERROR latch survives software reset |
| HIL-041 | Boot-trigger round-trip |
| HIL-042 | Wrong-bus ignored |
| HIL-043 | Wrong-payload ignored (all six sub-cases) |
| HIL-045 | Pre-reboot relay open |
| HIL-046 | BKP0R one-shot |
| HIL-047 | Trigger flood resilience |
| HIL-050 | 30-min soak (resets disqualify the build) |

Everything else (HIL-007, HIL-010, HIL-015–018, HIL-022–023, HIL-025–027,
HIL-030–040, HIL-044, HIL-051–055) is **strongly recommended** but
not a hard gate. A test marked recommended that fails is a bug; the
build is still releasable only if the responsible engineer signs off
on the residual risk.

### v1.2.0-ltc6811 gate (LTC6811 BMS migration)

On top of the v1.1.0 mandatory list (the legacy CAN-bound BMS tests
that survived the rewrite stay applicable — they're now expressed in
isoSPI terms), the following v1.2.0 tests **must** pass on the same
SHA before the milestone tag goes on `main`:

| Test | Why it's mandatory |
|---|---|
| HIL-009 | Wakeup pulse train + chain-length discovery (10 ICs) |
| HIL-030 | ADCV cadence stays inside the 50 ms budget |
| HIL-031 | 20-channel mux sweep runs in full |
| HIL-032 | RDCV[A-D] cells land in the right slot |
| HIL-033 | RDAUXA + Beta-model produces a sane temperature |
| HIL-056 | Missing module on boot trips ERROR before relays could close |
| HIL-057 | Cable yank mid-Run trips ERROR via freshness |
| HIL-058 | PEC-error storm does NOT trip FORCE_ERROR (transient noise resilience) |
| HIL-060 | WRCFGA → RDCFGA round-trip confirms balancing FET path works |

HIL-034 (PEC error counter), HIL-059 (ADG731 bit-order), HIL-061
(balancing thermal lockout) are **strongly recommended** for the
v1.2.0 gate but not hard blockers — they validate behaviours that
also fail-safe through the existing freshness / lockout paths.

---

## Sign-off template

Copy into the project log when a session completes.

```
HIL session
-----------
Date       :  YYYY-MM-DD
Engineer   :  <name>
Firmware   :  <SHA on dev or main>
Bootloader :  <SHA + BL_NODE_ID>
Rig        :  <bench id / board serial>

Mandatory tests
---------------
HIL-001  [PASS | FAIL | SKIP] <notes>
HIL-002  [...]
...

Recommended tests
-----------------
...

Issues / observations
---------------------
- ...

Sign-off
--------
Mandatory tests all PASS:   [ ] yes  [ ] no
Authorise v1.1.0-bootloader tag on main: [ ] yes  [ ] no
```

Attach the sign-off in a comment on issue #53 (memory layout) AND
issue #54 (boot trigger). Both issues close on this sign-off + tag.

---

## Maintenance

- New test? Append at the next `HIL-NNN` integer. Don't renumber
  existing tests — historic sign-offs cite the IDs.
- Existing test no longer relevant? Mark it `DEPRECATED` in-place,
  don't delete; add a note pointing at its replacement.
- A regression slipping past a passing test is the biggest sin — open
  a `fix/N-hil-NNN-tightening` branch immediately and harden the
  test's pass criteria so the regression couldn't have slipped.
