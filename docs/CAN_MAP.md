# AMS CAN map

Reverse-engineered from the legacy bare-metal AMS code at
`isc-fs/IFS08-CE/AMS/Core/Src/`. Source-of-truth for the FreeRTOS refactor.

The refactor MUST be byte-compatible with this map unless a CAN spec
update is explicitly agreed with the VCU and BMS teams.

---

## Bus assignment

| Bus | Role | Frame format | Filter (legacy) |
|---|---|---|---|
| **FDCAN1** | Accumulator / vehicle bus | Standard + Extended | Accept-all (FilterID1=0x0, FilterID2=0x0) |
| **FDCAN2** | BMS slave bus | Standard | Mask FilterID1=0x10, FilterID2=0x10 |

Both run classic CAN framing in the legacy firmware. The refactor may
upgrade FDCAN2 to CAN-FD if the slave modules support it; defer to Phase 3.

---

> **DEPRECATED in v1.2.0 (LTC6811-1 isoSPI).** The five BMS modules
> no longer talk to the AMS over FDCAN2 — they sit on a daisy-chained
> isoSPI link driven by an LTC6820 master and read via the LTC6811-1
> register groups. The on-MCU surface that used to ingest these
> frames (`BmsService::update_from_frame`) is now an inert stub kept
> alive only until `BmsRxTask` is retired in #73. New code goes
> through `BmsService::update_from_ltc_response` and the wire format
> is documented in `docs/BMS_LTC6811.md` (#75). The sections below
> are kept verbatim for archaeology / firmware on legacy hardware.

## Module addressing — BMS slaves (5 modules) [LEGACY]

Each BMS slave has its own CANID. **All response and request IDs for a
given module are offsets from that module's CANID**, not from a single
global base:

```
CANID(m)        = 0x12C + m * 0x1E      (m = 0..4)
voltage poll TX = CANID(m)              -- offset 0
voltage resp RX = CANID(m) + 1..5       -- 5 frames per module
temp poll    TX = CANID(m) + 20         -- offset 20
temp resp    RX = CANID(m) + 21..25     -- 5 frames per module
```

| Module | CANID | Voltage resp | Temp poll TX | Temp resp |
|---|---|---|---|---|
| 0 | 0x12C | 0x12D – 0x131 | 0x140 | **0x141 – 0x145** |
| 1 | 0x14A | 0x14B – 0x14F | 0x15E | **0x15F – 0x163** |
| 2 | 0x168 | 0x169 – 0x16D | 0x17C | **0x17D – 0x181** |
| 3 | 0x186 | 0x187 – 0x18B | 0x19A | **0x19B – 0x19F** |
| 4 | 0x1A4 | 0x1A5 – 0x1A9 | 0x1B8 | **0x1B9 – 0x1BD** |

> **Corrected 2026-05-11.** An earlier revision of this document put
> the temperature responses at `0x14D + m * 0x1E` (a fixed global
> base). That's wrong: it lands `0x21` above each module's CANID,
> well outside the legacy guard `id > CANID && id < CANID + 30`.
> Caught by unit tests in fix/3-unit-tests against the actual legacy
> parser at `isc-fs/IFS08-CE/AMS/Core/Src/class_bms.cpp:121-122`.

Module index of an incoming frame is recovered by walking the 5 CANIDs
and matching `id > CANID(m) && id < CANID(m) + 30`. Inter-module ranges
abut (module n ends at `CANID + 30` = module n+1's CANID) so a single
match is always unambiguous.

---

## TX — AMS to BMS slaves (FDCAN2)

### `0x12C + 0x1E·m` — voltage poll / balancing command

| Field | Value |
|---|---|
| Direction | TX (AMS → BMS[m]) |
| Bus | FDCAN2 |
| ID type | Standard |
| DLC | 2 |
| Period | 250 ms (`TIME_LIM_SEND_VOLTS`) |
| Suppressed when | charging (`flag_charger == 1`) |

Payload:

| Byte | Field | Notes |
|---|---|---|
| 0 | `BALANCING_V >> 8` | Big-endian. Cell-balancing target voltage (mV). |
| 1 | `BALANCING_V & 0xFF` | |

Source: `BMS_MOD::query_voltage()` in `class_bms.cpp:242–250`.

### `0x140 + 0x1E·m` — temperature poll

| Field | Value |
|---|---|
| Direction | TX (AMS → BMS[m]) |
| Bus | FDCAN2 |
| ID type | Standard |
| DLC | 2 |
| Payload | `{0x00, 0x00}` |
| Period | 500 ms (`TIME_LIM_SEND_TEMPS`) |

Source: `BMS_MOD::query_temperature()` in `class_bms.cpp:282`.

---

## RX — BMS slaves to AMS (FDCAN2)

### `0x12D – 0x131` (+ 0x1E·m) — cell voltages

5 frames per module covering 19 cells. 4 cells per frame, last frame has
3 cells (bytes 6–7 unused).

| Field | Value |
|---|---|
| Direction | RX (BMS[m] → AMS) |
| ID type | Standard |
| DLC | 8 |
| Encoding | Big-endian uint16, mV |
| Freshness timeout | 1500 ms (enforced) → `BMS_ERROR_COMMUNICATION` |

Frame-to-cell index mapping:

| Frame ID offset | Cell indices |
|---|---|
| +1 (0x12D) | 0, 1, 2, 3 |
| +2 (0x12E) | 4, 5, 6, 7 |
| +3 (0x12F) | 8, 9, 10, 11 |
| +4 (0x130) | 12, 13, 14, 15 |
| +5 (0x131) | 16, 17, 18 |

Per-cell decode: `cell_mV = (buf[2k] << 8) | buf[2k+1]` for `k ∈ {0,1,2,3}`.

Source: `BMS_MOD::parse()` in `class_bms.cpp:121–164`.

### `0x14D – 0x151` (+ 0x1E·m) — cell temperatures

5 frames per module covering 38 temperatures. 8 sensors per frame, last
frame has 6 (bytes 6–7 unused).

| Field | Value |
|---|---|
| Direction | RX (BMS[m] → AMS) |
| ID type | Standard |
| DLC | 8 |
| Encoding | int8 °C per byte |
| Freshness timeout | 1000 ms (legacy: declared but **not enforced**) |

Frame-to-temp index mapping:

| Frame ID offset | Temp indices |
|---|---|
| +21 (0x14D) | 0..7 |
| +22 (0x14E) | 8..15 |
| +23 (0x14F) | 16..23 |
| +24 (0x150) | 24..31 |
| +25 (0x151) | 32..37 |

Source: `BMS_MOD::parse()` in `class_bms.cpp:166–192`.

**Refactor decision:** enforce the 1000 ms freshness check — currently a
silent failure mode.

---

## TX — AMS to vehicle / charger (FDCAN1)

### `0x12C` — minimum cell voltage telemetry **[RETIRED — fix/48]**

Used to TX every 500 ms while in `Run`, suppressed during `Charge`. The only
reason to send it was the legacy "no balancing during charge" rule; now
that the charger no longer communicates on CAN, the suppression became
meaningless. Frame retired entirely; will return as a charge-state-only
balance TX in a future PR (the firmware comment in `acu_can_task.cpp`
marks the future re-entry point).

### `0x20` — AMS state reply

| Field | Value |
|---|---|
| Direction | TX (AMS → vehicle) |
| Bus | FDCAN1 |
| ID type | Extended |
| DLC | 1 |
| Trigger | On RX of `0x100` when `DC_BUS > 280 V` |

Byte 0:

| Value | State |
|---|---|
| 0 | CPU_POWER (running) |
| 1 | CPU_PRECHARGE |
| 2 | CPU_DISCONNECTED |
| 3 | CPU_ERROR |
| 4 | CPU_CHARGING |

Source: `CPU_MOD::parse()` / `updateState()` in `class_cpu.cpp:71` and
`module_state_machine.cpp:109–111`.

### `0x450` — current measurement

| Field | Value |
|---|---|
| Direction | TX |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 2 |
| Period | 250 ms (`TIME_LIM_SEND`) |

Payload:

| Byte | Field |
|---|---|
| 0 | 0x00 |
| 1 | `Current & 0xFF` (amps, lower byte only — legacy limitation) |

Source: `Current_MOD::query()` in `class_curent.cpp:130–136`.

**Refactor decision:** widen to a 16-bit signed mA value with a defined
sign convention (+ discharge / − charge). Coordinate with VCU.

### `0x500` — current warning, 80%–100% of `C_MAX`

| Field | Value |
|---|---|
| Direction | TX |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 1 |
| Payload | 0x00 (placeholder) |
| Trigger | `0.8 · C_MAX < current < C_MAX` |

Source: `class_curent.cpp:99`.

### `0x501` — current over-limit alert

| Field | Value |
|---|---|
| Direction | TX |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 2 |
| Trigger | `current > C_MAX` (single shot, counter increments) |

Source: `class_curent.cpp:105`.

### `0x502` — current normal (recovery)

| Field | Value |
|---|---|
| Direction | TX |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 2 |
| Payload | 0x00 |
| Trigger | current drops below threshold, repeated 5× for redundancy |

Source: `class_curent.cpp:124`.

### `0x40D – 0x412` — temperature forwarding (charger mode only)

| Field | Value |
|---|---|
| Direction | TX (AMS forwards raw FDCAN2 RX onto FDCAN1) |
| Bus | FDCAN1 |
| ID type | Extended |
| DLC | 8 |
| Trigger | `flag_charger == 1` |

Frames are passed through unmodified for the charger to consume. IDs are
shifted from `0x401–0x406` (FDCAN2 RX) into the `0x40D–0x412` range.

Source: `BMS_MOD::parse()` `class_bms.cpp:170`,
`Temperatures_MOD::parse()` `class_temperatures.cpp:86,99`.

---

## TX — AMS telemetry (FDCAN1)

Three single-purpose 8-byte frames emitted every **500 ms** by
`MainTask` (the consolidated SafetyTask + StateTask + TelemetryTask
since refactor/19 phase 3). Bench tools and the VCU consume these
instead of the legacy UART telemetry line (the UART path was
dropped in feat/22).

Encoders are pure-logic in
[`Core/Inc/app/telemetry_encoders.hpp`](../Core/Inc/app/telemetry_encoders.hpp);
unit tests in
[`tests/unit/test_telemetry_encoders.cpp`](../tests/unit/test_telemetry_encoders.cpp).

### `0x4A0` — AMS status

Standard 11-bit. DLC 8. Cadence 500 ms.

| Byte | Field | Notes |
|:---:|---|---|
| 0 | `state` | `ams::fsm::State` enum (0..5: Start, Precharge, Transition, Run, Charge, Error) |
| 1 | `ams_ok` | GPIO PB4 read-back; 0 or 1 |
| 2 | `module_online_mask` | Low byte of `BmsState.module_online_mask`. 0x1F = all 5 modules healthy |
| 3 | reserved | 0 |
| 4-5 | `min_cell_mV` | Big-endian uint16, mV |
| 6-7 | `max_cell_mV` | Big-endian uint16, mV |

### `0x4A1` — AMS pack

Standard 11-bit. DLC 8. Cadence 500 ms.

| Byte | Field | Notes |
|:---:|---|---|
| 0-3 | `pack_voltage_mV` | Little-endian uint32, mV (sum of all cells) |
| 4-7 | `filtered_mA` | Little-endian **signed** int32, mA. `+ = discharge, − = charge` |

### `0x4A2` — AMS temps + heartbeat

Standard 11-bit. DLC 8. Cadence 500 ms.

Flight layout:

| Byte | Field | Notes |
|:---:|---|---|
| 0 | `min_tempC` | Signed int8, °C. `BmsState.min_tempC` clipped to int8 range |
| 1 | `max_tempC` | Signed int8, °C |
| 2 | `avg_tempC` | Signed int8, °C |
| 3-4 | `dc_bus_V` | Little-endian uint16, V (from `VehicleState.dc_bus_V`) |
| 5 | reserved | 0 |
| 6 | `tx_fail_lo` | Low byte of `g_telemetry_tx_fail` |
| 7 | `heartbeat` | Wraparound 8-bit counter, increments per MainTask telemetry cycle (500 ms). Useful for detecting dropped frames on the receiver. |

HIL_STUB layout (`-DAMS_BMS_HIL_STUB`): bytes 0..2 and 6..7 unchanged;
bytes 3..5 carry bench diagnostic probes instead of `dc_bus_V`
(the bench injects `dc_bus_V` from the host so visibility is unaffected):

| Byte | Field | Notes |
|:---:|---|---|
| 3 | `bms_task_state_byte` | `0xA0 | (osThreadGetState(BmsPollTaskHandle) & 0x0F)`. `0xFF` if handle is NULL. |
| 4 | `acu_rx_total_lo` | Low byte of `g_acu_rx_total` (ticks on any matched ACU RX frame; AcuCanTask + queue + dispatch liveness) |
| 5 | `cockpit_byte` | `0x80 | (mode_locked << 2) | (TSMS<<1) | RST_PIL`. High bit `0x80` is a sentinel so older binaries' `0x00` stands out. mode_locked: `0`=Undecided, `1`=Car, `2`=Charger. |

---

## RX — vehicle / charger to AMS (FDCAN1)

### `0x100` — DC bus voltage from VCU

| Field | Value |
|---|---|
| Direction | RX (VCU → AMS) |
| Bus | FDCAN1 |
| ID type | Extended |
| DLC | 2 |
| Decode | `DC_BUS = (buf[1] << 8) | buf[0]` (little-endian, volts) |
| Use | precharge complete when `200 < DC_BUS < 500` |
| Freshness | 1000 s declared, not enforced |

Source: `CPU_MOD::parse()` `class_cpu.cpp:65–68`,
`parse_state()` `module_state_machine.cpp:334–335`.

**Refactor decision:** enforce a 200 ms freshness on `DC_BUS`. Stale
voltage during precharge is a real fault.

### `0x600` — start button **[RETIRED — fix/48]**

Replaced by the **TSMS** GPIO (PF9, active-high, external pull-down). The
FSM Start→Precharge transition now requires both `TSMS` and `RST_PIL`
asserted; level-polled at the 20 ms FSM cadence in `safety_task.cpp`.

### `0x401 – 0x406` — accumulator temperature sensors

| Field | Value |
|---|---|
| Direction | RX (temp module → AMS) |
| Bus | FDCAN2 (not FDCAN1 — VERIFY in legacy) |
| ID type | Standard |
| DLC | 8 |
| Encoding | int8 °C per byte |
| Total | 38 sensors across 6 frames |

Source: `Temperatures_MOD::parse()` `class_temperatures.cpp:73–134`.

### `0x18FF50E7` — charger detected **[RETIRED — fix/48]**

The charger no longer communicates over CAN; the only thing on the
charger-assembly bus is an HMI for displaying cell V/T. Car-vs-charger
context is now distinguished by VCU `0x100` heartbeat freshness at the
moment of Start→Precharge transition: heard within `kVcuFreshMs`
(1000 ms) → Car (target = Run), silent → Charger (target = Charge).
The captured mode locks for the rest of the boot cycle and never
re-evaluates.

---

## RX — bootloader-trigger command (FDCAN1)

Not from the legacy AMS — added in the refactor for in-system firmware
update via [isc-fs/stm32-can-bootloader](https://github.com/isc-fs/stm32-can-bootloader).

> Moved from FDCAN2 to FDCAN1 in v1.2.0 (#73). FDCAN2 stays the
> bootloader's working bus after reset, but the in-band reboot
> trigger now rides on the accumulator/vehicle bus alongside
> everything the pit-tool already sends.

### `0x002` — request reboot into bootloader

| Field | Value |
|---|---|
| Direction | RX (host → AMS) |
| Bus | **FDCAN1** (accumulator bus; the bootloader takes FDCAN2 over after the reset) |
| ID type | Standard 11-bit, very high arbitration priority |
| DLC | 4 |
| Payload | `{0xB0, 0x07, 0xAD, 0x11}` -- all 4 bytes must match exactly |
| Effect | `AcuCanTask` calls `ams::Bootloader::request_reboot()` which opens all relays, drains TX, writes `0xB00710AD` to `RTC->BKP0R`, and `NVIC_SystemReset()`s. The bootloader's reset handler sees the magic, clears it (one-shot), and stays in BL mode awaiting flash commands on FDCAN2. |
| Failure modes | Wrong bus, wrong ID, wrong DLC, or any byte of the payload differing → frame silently dropped, no reboot. |

Source: [`Core/Inc/app/bootloader.hpp`](../Core/Inc/app/bootloader.hpp) (`matches_trigger`), [`Core/Src/app/bootloader.cpp`](../Core/Src/app/bootloader.cpp) (`request_reboot`), dispatched in [`Core/Src/app/acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp). Constants in [`Core/Inc/app/ams_config.hpp`](../Core/Inc/app/ams_config.hpp) (`kBlBootReqCanId`, `kBlBootReqPayload`, `kBlBootReqDlc`).

---

## Timeout summary

| Signal | Timeout | Enforced (legacy) | Refactor plan |
|---|---|---|---|
| BMS voltage RX | 1500 ms | Yes → `BMS_ERROR_COMMUNICATION` | Keep, route to `FORCE_ERROR` |
| BMS temperature RX | 1000 ms | No (commented out) | **Enforce** |
| `0x100` DC bus | 1000 s | No | **Tighten to 200 ms** |
| Temperature module RX | 1000 ms | No | **Enforce** |
| Current sensor | — | — | Add: 200 ms staleness → `FORCE_ERROR` |

---

## Refactor checklist

For each TX/RX frame above, the refactor must:

- [ ] Define the ID, DLC, and byte layout as `constexpr` in
      `Core/Inc/app/ams_config.hpp`.
- [ ] Implement encode/decode as free functions on `CanFrame` (no
      class-local state).
- [ ] Cover encode/decode with unit tests (host CMake, Unity).
- [ ] Verify byte-compatibility against a captured trace from the legacy
      firmware on the bench before merging the corresponding feat branch.

Open coordination items with the VCU team:

- [ ] Widen `0x450` payload to signed 16-bit mA.
- [ ] Confirm `0x600` bus assignment (FDCAN1 vs FDCAN2).
- [ ] Confirm `0x401–0x406` bus assignment.
- [ ] Agree on a new pack-status frame on FDCAN1 at 100 ms cadence with
      voltage / current / state / fault word packed together (planned for
      Phase 4, feat/12).
