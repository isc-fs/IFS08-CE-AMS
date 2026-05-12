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
| FDCAN1 wiring | PD0/PD1 on the target, terminated 120 Ω at each end of the bus |
| FDCAN2 wiring | PB12/PB13, terminated 120 Ω. Bootloader also lives on this bus |
| BMS slave emulator | At minimum a script that emits the 5-module poll responses on FDCAN2 with controllable cell V/T values. The host PC running `python-can` is sufficient |
| ADC current input | Analog source on PF11 (ADC1 ch2). A bench DAC or a calibrated voltage source 0–3.3 V |
| GPIO inputs | Switch / signal generator on PE9 (`DIGITAL1` / SDC) and PG7 (`Charge_Button`) |
| GPIO output read-back | DMM or logic analyser on PD3 (`RELAY_AIR_N`), PD4 (`RELAY_AIR_P`), PD5 (`RELAY_PRECHARGE`), PF13 (`AMS_OK`), PB9 (`TIM17_CH1` fan PWM) |
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
| `arm-none-eabi-gdb` + OpenOCD | Fault-injection (hang SafetyTask, etc.), backtrace on hardfault |
| `scripts/check_flash_layout.py` | Pre-flight sanity for any custom build |

### 1.3 Fixture firmware

- **Bootloader image** built from `isc-fs/stm32-can-bootloader` `main`
  branch with `BL_NODE_ID` set to **any value ≠ 2** (so the BL's
  hardware filter doesn't accidentally match `0x002` boot-trigger
  frames). Confirm in `Core/Inc/bl_config.h` of that build.
- **AMS image** built from the firmware SHA under test (default:
  latest `dev`). `scripts/check_flash_layout.py build/AMS.elf` must
  PASS before the image is flashed.
- **BMS slave emulator**: a small `python-can` script that responds to
  poll IDs at `0x12C + m * 0x1E` (voltage poll) and `0x140 + m * 0x1E`
  (temp poll) with the response frames documented in
  [`CAN_MAP.md`](CAN_MAP.md). Cell V and temp values are tunable per
  test.

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
| Steps | 1. Erase the entire flash with `STM32_Programmer_CLI -e all`.<br>2. Power-cycle the board.<br>3. Within 50 ms of power-on, measure PD3, PD4, PD5. |
| Pass | All three pins read 0 V (LOW). |
| Fail mode | Any pin asserted → either the GPIOs are not in their reset state (HW issue) or a stray default-flashed image is closing relays. |
| Capture | Multimeter / logic-analyser sample on PD3/4/5. |
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
| Pass | UART emits `AMS s=S …` line (state=Start = 'S') within 1 s of reset. PD3/4/5 read LOW (relays open per HIL-001 + initial fan duty 0 % per state_task `kFanDuty[Start]`). |
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
| Steps | 1. Attach GDB while running in Start. Write `RTC->BKP1R = 0xA115EE51` (kBkpErrorMagic).<br>2. `NVIC_SystemReset()` from GDB.<br>3. Read UART. |
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

### HIL-009 — FDCAN filter admits all expected RX frames

**Goal**: prove the `HAL_FDCAN_ConfigGlobalFilter` calls in
`App_InitTask` admit every RX frame the firmware expects on both buses.

| | |
|---|---|
| Steps | 1. Attach GDB, set a breakpoint in `HAL_FDCAN_RxFifo0Callback`.<br>2. From the host, inject one frame of every documented RX type:<br>&nbsp;&nbsp;FDCAN1 — `0x100`, `0x600`, `0x18FF50E7`<br>&nbsp;&nbsp;FDCAN2 — every `CANID(m)+1..5` and `CANID(m)+21..25` for m∈{0..4}, plus the boot-trigger `0x002`<br>3. Confirm the breakpoint fires for each frame. |
| Pass | Every injected frame triggers the callback. |
| Fail mode | Any frame missed → the global filter is rejecting it, or wrong bus mapping. |
| Capture | `candump` + GDB breakpoint counts. |
| Duration | 20 min |

---

## Block B — Safety supervisor

**Block precondition**: HIL-004 passed (AMS boots and runs).

### HIL-010 — SafetyTask 10 ms cadence

**Goal**: SafetyTask actually runs at its declared 10 ms period.

| | |
|---|---|
| Steps | 1. (One-time firmware tweak in a debug build, or via the existing telemetry timestamp) Capture `osKernelGetTickCount` deltas across SafetyTask iterations.<br>2. Run for 60 s. |
| Pass | Mean period = 10 ms ± 1 ms; max period ≤ 12 ms; no period > 15 ms. |
| Fail mode | Long periods → priority inversion, or another task hogging CPU. |
| Capture | Time-series log of periods. |
| Duration | 10 min |

### HIL-011 — IWDG resets the chip if SafetyTask hangs

**Goal**: invariant 4 — watchdog discipline enforces a HW reset within
~100 ms if SafetyTask stops feeding it.

| | |
|---|---|
| Steps | 1. Attach GDB. Set a breakpoint inside `SafetyTask::run()`.<br>2. Hit the breakpoint, halt the CPU.<br>3. Wait 200 ms in wall-clock time.<br>4. Continue. |
| Pass | The chip resets within ~100 ms of the halt. After resume, the reset is observable (UART shows fresh boot line; reset-cause register has IWDG bit set). |
| Fail mode | Chip continues running past 200 ms → watchdog disabled (check `hiwdg` init), or refresh path is on the wrong branch. |
| Capture | GDB transcript + reset-cause register read. |
| Duration | 10 min |

### HIL-012 — Watchdog reset re-opens relays

**Goal**: after a watchdog reset (from HIL-011 or any synthetic), the
chip boots through `MX_GPIO_Init` and re-asserts PD3/4/5 to PIN_RESET
within milliseconds.

| | |
|---|---|
| Steps | 1. Start in Run (need a full happy-path setup, see Block C). Verify PD3/4 are HIGH.<br>2. Halt SafetyTask via GDB. Wait for IWDG reset.<br>3. Probe PD3/4/5 immediately after reset. |
| Pass | All three read LOW within 20 ms of the reset edge. |
| Fail mode | Pins stay HIGH → MX_GPIO_Init isn't writing them before pin direction is configured, or boot is slow. |
| Capture | Logic analyser on PD3/4/5 + a reset-edge signal (e.g. NRST). |
| Duration | 10 min |

### HIL-013 — FORCE_ERROR event flag opens AIRs

**Goal**: any task can post `kForceError` on `safety_events` and the
supervisor responds within one period.

| | |
|---|---|
| Steps | 1. Bring AMS up to Run.<br>2. Via GDB, call `osEventFlagsSet(safety_eventsHandle, 0x1)` (kForceError = bit 0).<br>3. Within 15 ms, read PD3/4/5. |
| Pass | All three pins are LOW. UART next line says `s=E`. `RTC->BKP1R` reads `0xA115EE51`. |
| Fail mode | Pins still HIGH after 50 ms → SafetyTask not consuming the flag, or relay write path broken. |
| Duration | 5 min |

### HIL-014 — Cell undervoltage trips ERROR

**Goal**: a single cell below `kCellUVmV` (2800 mV) trips the predicate
set within one BmsRxTask cycle + one SafetyTask period.

| | |
|---|---|
| Steps | 1. AMS in Run with healthy emulated BMS.<br>2. BMS emulator sends next voltage frame for module 2 with cell[5] = 2700 mV.<br>3. Within 30 ms of the frame, sample PD3/4/5 + read UART. |
| Pass | Pins LOW within 30 ms (one BMS RX + one Safety period). UART says `s=E`. |
| Fail mode | Pins stay HIGH after 50 ms → threshold value mismatch, or `BmsService::update_from_frame` ignored the value. |
| Capture | `candump` + GPIO scope. |
| Duration | 5 min |

### HIL-015 — Cell overvoltage trips ERROR

Same shape as HIL-014, value = 4250 mV (above `kCellOVmV` = 4200).

### HIL-016 — Cell overtemperature trips ERROR

Same shape; temp frame with temp[10] = 65 °C (above `kCellOTC` = 60).

### HIL-017 — BMS module staleness trips ERROR

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Emulator stops responding to module 3 polls.<br>3. Wait for `kBmsStaleMs` (1500 ms) + 50 ms slack. |
| Pass | Pins LOW within 1.6 s of the last module-3 frame. UART says `s=E`. |
| Fail mode | Pins stay HIGH after 2 s → freshness check broken or `last_rx_tick[3]` not being updated. |
| Duration | 5 min |

### HIL-018 — Current sensor staleness trips ERROR

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Inject a constant voltage on PF11 ≈ 2.5 V (zero current) for the warm-up.<br>3. Halt CurrentTask via GDB (it's the only writer to `last_update_tick`).<br>4. Wait `kIStaleMs` (200 ms) + 50 ms. |
| Pass | Pins LOW within 300 ms. UART says `s=E`. |
| Fail mode | Pins stay HIGH → freshness threshold mis-applied. |
| Duration | 5 min |

### HIL-019 — SDC open trips ERROR

| | |
|---|---|
| Steps | 1. AMS in Run with PE9 HIGH (SDC closed).<br>2. Drive PE9 LOW.<br>3. Within 30 ms, sample PD3/4/5. |
| Pass | Pins LOW within 30 ms. UART says `s=E`. |
| Steps cont. | 4. Drive PE9 HIGH again. |
| Pass | Pins stay LOW (ERROR is sticky). UART continues to say `s=E`. |
| Duration | 5 min |

---

## Block C — FSM

**Block precondition**: Block B passed. BMS emulator emits healthy
frames continuously. SDC asserted (PE9 HIGH).

### HIL-020 — Start → Precharge on start button

| | |
|---|---|
| Steps | 1. Verify AMS in Start (`s=S`).<br>2. Send 0x600 standard frame with byte 0 = 1 on FDCAN1.<br>3. Within 50 ms, read PD3 and PD5; UART next line. |
| Pass | PD3 (AIR-) HIGH, PD5 (Precharge) HIGH, PD4 (AIR+) LOW. UART says `s=P`. |
| Fail mode | Wrong pin state → relay-action flag mismatch in `state_machine.hpp`. |
| Duration | 5 min |

### HIL-021 — Start → Charge on charger detect

| | |
|---|---|
| Steps | 1. AMS in Start.<br>2. Send extended frame `0x18FF50E7` on FDCAN1 (any payload).<br>3. Within 50 ms, sample pins; UART. |
| Pass | PD3 HIGH, PD4 HIGH, PD5 LOW. UART says `s=C`. |
| Duration | 5 min |

### HIL-022 — Precharge → Transition on DC bus target

| | |
|---|---|
| Steps | 1. AMS in Precharge (after HIL-020 sequence).<br>2. Emit 0x100 ext frame on FDCAN1 with `dc_bus_V` = 350 (≥ 0.95 × pack_V, where pack is healthy at ~356 V from emulator).<br>3. Within 50 ms, sample pins. |
| Pass | PD4 HIGH (AIR+ closed), PD5 LOW (Precharge open), PD3 still HIGH. UART says `s=T`. |
| Duration | 5 min |

### HIL-023 — Precharge timeout → ERROR

| | |
|---|---|
| Steps | 1. AMS in Precharge.<br>2. Continuously emit 0x100 with `dc_bus_V` = 50 (well below target).<br>3. Wait `kPrechargeMaxMs` (1500 ms) + 50 ms. |
| Pass | All pins LOW within 1.6 s. UART says `s=E`. `RTC->BKP1R` = `0xA115EE51`. |
| Duration | 5 min |

### HIL-024 — Transition → Run after hold

| | |
|---|---|
| Steps | 1. AMS in Transition (after HIL-022).<br>2. Keep 0x100 emitting `dc_bus_V` ≥ 350.<br>3. Wait `kTransitionHoldMs` (100 ms) + 30 ms.<br>4. Sample pins + UART. |
| Pass | PD3 HIGH, PD4 HIGH, PD5 LOW. UART says `s=R`. Fan duty PB9 PWM = 40 % (measure with scope). |
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
| Pass | Pins unchanged from Run state (PD3 HIGH, PD4 HIGH, PD5 LOW). UART continues `s=R`. |
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
| Steps | 1. Trip ERROR via any predicate (e.g. HIL-019).<br>2. Clear the fault (re-assert SDC).<br>3. Wait 5 s. |
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

### HIL-030 — BMS voltage poll cadence

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. `candump -tA can1` (FDCAN2) for 5 s. Filter for IDs `0x12C`, `0x14A`, `0x168`, `0x186`, `0x1A4`. |
| Pass | Each of the 5 IDs appears ~5 times per second (250 ms cadence ± 50 ms). |
| Fail mode | Slower → BmsPollTask priority issue or timer mis-configured. |
| Duration | 5 min |

### HIL-031 — BMS temperature poll cadence

Same shape as HIL-030; IDs `0x140`, `0x15E`, `0x17C`, `0x19A`, `0x1B8`; cadence 500 ms ± 100 ms.

### HIL-032 — BMS voltage response parsing

| | |
|---|---|
| Steps | 1. Emulator sends voltage frame for module 0 frame_idx 0 with cells = [3700, 3711, 3722, 3733] mV.<br>2. Attach GDB, dump `BmsService::instance().snapshot().cell_mV[0][0..3]`. |
| Pass | Values match exactly. |
| Fail mode | Endianness flip or wrong cell-index mapping. |
| Duration | 10 min |

### HIL-033 — BMS temperature response parsing

Same shape; temp frame module 2 idx 0, temps `[25, -5, 60, -20, 0, 40, -10, 30]` °C; verify `cell_tempC[2][0..7]`.

### HIL-034 — Unknown BMS frame counter

| | |
|---|---|
| Steps | 1. Send a frame with ID `0x200` (not in any module's range) on FDCAN2.<br>2. GDB-read `g_bms_rx_dropped_unknown` before / after. |
| Pass | Counter incremented by 1. |
| Duration | 5 min |

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
| Steps | 1. AMS in Run.<br>2. Inject 2.4 V on PF11 (≈ +17.5 A discharge).<br>3. `candump` FDCAN1 std-ID filter on `0x450`. |
| Pass | Frame appears every 250 ms ± 50 ms. Payload [1] approximately 17 (= 17 A & 0xFF). |
| Duration | 5 min |

### HIL-040 — Charger-suppress: TX 0x12C and BMS poll stop in Charge

| | |
|---|---|
| Steps | 1. AMS in Charge (after HIL-021 / HIL-037).<br>2. `candump` FDCAN1 and FDCAN2 for 2 s. |
| Pass | No 0x12C ext frames on FDCAN1 (suppressed when charging). BMS voltage polls on FDCAN2 also stop (legacy parity for "no balancing during charge"). 0x450 (current) still emits every 250 ms. |
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
| Steps | 1. AMS in Run.<br>2. Probe PD3/4/5 + UART.<br>3. `cansend can1 002#B007AD11` (FDCAN2). |
| Pass | Within 15 ms: PD3/4/5 all LOW (Relays::open_all fired). Within 1 s: BL DISCOVER reply on FDCAN2 confirms BL is alive. |
| Fail mode | Pins stay HIGH → boot-trigger not matching, BmsRxTask not dispatching, or `request_reboot` is not actually running.<br>BL doesn't respond → BL didn't see the BKP0R magic, or BL is in auto-jump and hands back to app. |
| Capture | Logic analyser on PD3/4/5 + NRST + `candump` of FDCAN2. |
| Duration | 10 min |

### HIL-042 — Wrong-bus trigger ignored

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Send the exact `002#B007AD11` payload **on FDCAN1** instead of FDCAN2. |
| Pass | No reboot, no relay change, UART continues to emit Run telemetry. |
| Fail mode | Reboot happens → the `CanFrame::bus` check in `matches_trigger` is broken. |
| Duration | 5 min |

### HIL-043 — Wrong-payload trigger ignored

Parametric. Repeat with each of these payloads on FDCAN2 id 0x002, DLC=4:

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
| Steps | 1. AMS in Run. PD3/4 HIGH.<br>2. Send the trigger.<br>3. Capture PD3/4/5 + NRST on a logic analyser with µs resolution. |
| Pass | PD3/4/5 fall to LOW **at least 5 ms before** NRST asserts (the `osDelay(10)` in `request_reboot` enforces this). |
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
| Capture | UART (count of bad-frame counter increments) + LA on PD3/4/5. |
| Duration | 10 min |

---

## Block F — Soak & fuzz

**Block precondition**: every test in Blocks A–E green on the same SHA.

### HIL-050 — 30-minute soak in Run

| | |
|---|---|
| Steps | 1. AMS in Run with healthy BMS, healthy current sensor, SDC closed.<br>2. UART log + `candump` to file for 30 minutes. |
| Pass | No resets. UART line cadence stays at 500 ms ± 20 ms across the entire run. No `g_*_dropped_*` counter increases beyond a low background (< 1 per minute). State remains `s=R`. |
| Fail mode | Reset → check reset-cause reg (IWDG? brownout? hardfault?). Drift in cadence → priority inversion. |
| Capture | UART log file (≥ 3000 lines). CAN trace. |
| Duration | 35 min |

### HIL-051 — Boot-trigger reliability

| | |
|---|---|
| Steps | Repeat HIL-041 ten times consecutively (each round: bring up to Run via #020→#024, trigger reboot, BL respond, re-flash optional). |
| Pass | All 10 round-trips succeed; no hung state requires manual power-cycle. |
| Duration | 30 min |

### HIL-052 — BMS bus fuzz

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. `python-can` sends 10 000 random standard-ID frames on FDCAN2 with random payloads over 60 s.<br>3. Throughout, watch UART + GPIO. |
| Pass | AMS stays in Run (no fault from a malformed frame). `g_bms_rx_dropped_unknown` increments accordingly. No hardfault. |
| Fail mode | Any reset, any spontaneous transition → `BmsService` or `Bootloader::matches_trigger` has a corner case that does the wrong thing on random input. |
| Capture | UART log + GDB attach if a reset occurs. |
| Duration | 65 min |

### HIL-053 — ACU bus fuzz

Same shape on FDCAN1 with extended + standard IDs.

### HIL-054 — Brownout recovery

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Drop VDD to 1.5 V briefly (200 ms), restore to 3.3 V.<br>3. Observe behaviour. |
| Pass | AMS resets (BOR pinned the chip), boots fresh, comes up in Start (or Error if `BKP1R` survived — VBAT-dependent). |
| Duration | 10 min |

### HIL-055 — SDC chatter

| | |
|---|---|
| Steps | 1. AMS in Run.<br>2. Drive PE9 with a 100 Hz square wave (5 ms HIGH, 5 ms LOW) for 5 s.<br>3. Observe. |
| Pass | The very first LOW edge trips ERROR (HIL-019). Subsequent edges do nothing — ERROR is sticky. AMS does not glitch, oscillate, or reset spontaneously. |
| Duration | 10 min |

---

## Acceptance criteria

For the v1.1.0-bootloader hardware-acceptance gate (issues #53 and
#54), the following tests **must** pass on the same firmware SHA:

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
| HIL-019 | SDC open trips |
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
