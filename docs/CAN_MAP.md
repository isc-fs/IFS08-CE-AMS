# AMS CAN map

Source-of-truth for the wire format that the firmware emits + consumes
on the **accumulator bus (FDCAN1)**. The "BMS slave bus" sections
further down are historical archaeology — see the note below.

---

## Bus assignment

**Bit rate: 500 kbps** (classic CAN, 68.75 % sample point — FDCAN kernel
clock 24 MHz, nominal prescaler 3, Tseg1 10, Tseg2 5, SJW 1). The 1 Mbps
bump (#338) was **reverted** (#351): on real hardware the bus ran near its
signal-integrity margin (~7 errors/5 min vs ≈0 at 500 k) and a mid-flash
bit error bricked chips via stm32-can-bootloader#166; doubling the bit time
restores the margin. The whole bus — VCU, ECU, charger, and the
**bootloader** — must run at 500 kbps in lockstep. No CAN-FD / bit-rate
switching (all frames classic).

| Bus | Role | Frame format | Filter |
|---|---|---|---|
| **FDCAN1** | Accumulator / vehicle / telemetry / bootloader-trigger | Standard only (extended rejected at HW filter since #236) | Accept all unmatched standard into FIFO0; reject extended; reject remote |

**The app is FDCAN1-only.** FDCAN2 was dropped from the AMS CubeMX
project in #388 (`a885bf5`) — there is no `MX_FDCAN2_Init` and no
`hfdcan2` handle; the sole residue is the PB13 pin still muxed to
`GPIO_AF9_FDCAN2` (unused). The in-band reboot trigger (`0x002`) rides on
FDCAN1 alongside everything else (see below). The
`isc-fs/stm32-can-bootloader` is a **separate sector-0 image** that brings
up its own CAN peripheral from scratch after the magic-reset jump — the
AMS app does not leave any peripheral configured for it.

---

> **DEPRECATED in v1.2.0 (LTC6811-1 isoSPI).** The five BMS modules
> no longer talk to the AMS over FDCAN2 — they sit on a daisy-chained
> isoSPI link driven by an LTC6820 master and read via the LTC6811-1
> register groups. The legacy on-MCU surface (`BmsService::update_from_frame`
> + `BmsRxTask`) was retired in #73. New code goes through
> `BmsService::update_from_ltc_response` and the wire format is
> documented in `docs/BMS_LTC6811.md` (#75). The sections marked
> `[LEGACY]` below are kept verbatim for archaeology and for firmware
> running on legacy carriers; nothing in the current build emits or
> consumes those frames.

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

## TX — AMS to ECU (FDCAN1) — ECU forwards to real-time telemetry

The ECU's FDCAN2 peripheral is wired to AMS FDCAN1; these frames feed the
ECU's onboard logic + the real-time telemetry uplink. All standard 11-bit
IDs, big-endian payloads. Cadence groups (per-frame deadline scheduler in
`acu_can_task.cpp`):

- 50 ms — `0x135` currents
- 100 ms — `0x020`, `0x12C`, `0x131..0x134`
- 250 ms — `0x136..0x137`

`0x130` (SOC) deferred — no SOC estimator in firmware yet.

### `0x020` — ok_precharge

| Field | Value |
|---|---|
| Direction | TX (AMS → ECU) |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 1 |
| Period | 100 ms |

| Byte | Field | Notes |
|:---:|---|---|
| 0 | `ok_precharge` | `1` iff FSM state ∈ {Run, Charge} (AIRs closed and ready). `0` otherwise. |

### `0x12C` — minimum cell voltage (pack-wide)

| Field | Value |
|---|---|
| Direction | TX |
| Bus | FDCAN1 |
| ID type | Standard |
| DLC | 2 |
| Period | 100 ms |

| Byte | Field |
|:---:|---|
| 0–1 | `v_cell_min` BE uint16, mV (`BmsState.min_cell_mV`) |

### `0x131` — vmin per module (modules 0..2)

| Field | Value |
|---|---|
| Direction | TX |
| ID type | Standard |
| DLC | 6 |
| Period | 100 ms |

| Bytes | Field |
|:---:|---|
| 0–1 | `vmin_module[0]` BE uint16, mV |
| 2–3 | `vmin_module[1]` BE uint16, mV |
| 4–5 | `vmin_module[2]` BE uint16, mV |

For an offline module, sentinel `0xFFFF`.

### `0x132` — vmin per module (modules 3..4)

| Field | Value |
|---|---|
| DLC | 4 | Period | 100 ms |

| Bytes | Field |
|:---:|---|
| 0–1 | `vmin_module[3]` BE uint16, mV |
| 2–3 | `vmin_module[4]` BE uint16, mV |

### `0x133` — vmax per module (modules 0..2)

Same layout as `0x131` with `vmax_module[0..2]`. Sentinel for offline module: `0x0000`.

### `0x134` — vmax per module (modules 3..4)

Same layout as `0x132` with `vmax_module[3..4]`.

### `0x135` — pack + DCDC current (signed deciamps)

| Field | Value |
|---|---|
| Direction | TX |
| ID type | Standard |
| DLC | 4 |
| Period | 50 ms |

| Bytes | Field |
|:---:|---|
| 0–1 | `current_accu` BE int16, deciamps (1 LSB = 0.1 A; `+` = discharge) |
| 2–3 | `current_dcdc` BE int16, deciamps |

Sign convention preserved from `+ = discharge, − = charge`. Pack current
is read **differentially** (PF7/PF8, ADC3_INP3/INN3); the front-end now
observes well beyond the FS-rules range (the old ×4 + 1.65 V single-ended
front-end capped firmware-side at ±82.5 A — that limit is gone).
Supersedes the retired `0x450`.

### `0x136` — temp_max per module (modules 0..2)

| Field | Value |
|---|---|
| DLC | 6 | Period | 250 ms |

| Bytes | Field |
|:---:|---|
| 0–1 | `temp_max_module[0]` BE int16, °C |
| 2–3 | `temp_max_module[1]` BE int16, °C |
| 4–5 | `temp_max_module[2]` BE int16, °C |

Sentinel for offline module: `INT16_MIN` = `0x8000`.

### `0x137` — temp_max per module (modules 3..4) + temp_dcdc

| Field | Value |
|---|---|
| DLC | 6 | Period | 250 ms |

| Bytes | Field |
|:---:|---|
| 0–1 | `temp_max_module[3]` BE int16, °C |
| 2–3 | `temp_max_module[4]` BE int16, °C |
| 4–5 | `temp_dcdc` BE int16, °C **[STUB — `INT16_MIN`]** until the DCDC temp sensor is wired |

### `0x450` — current measurement **[RETIRED — fix/53]**

Legacy 2-byte unsigned current frame. Superseded by `0x135` (signed
deciamps + DCDC current in the same frame).

### `0x20` — AMS state reply **[LEGACY DOC — superseded by `0x020`]**

The legacy AMS used the extended-ID `0x20` with 5-value state byte. The
current firmware emits `0x020` (standard) as a simple `ok_precharge`
boolean (see above). Full state mirror still lives in `0x4A0[0]` for
diagnostic consumers that want it. Kept as a doc anchor so spelunkers in
old logs can find the cross-reference.

### `0x450` — current measurement **[RETIRED — fix/53, see above]**

Legacy 2-byte unsigned current frame. Removed; `0x135` is the successor.
Kept as a doc anchor for log-archaeology.

### `0x500` / `0x501` / `0x502` — current warning / over-limit / normal **[RESERVED — not emitted]**

`AcuTxCurrentWarnId` (`0x500`), `AcuTxCurrentOverLimitId` (`0x501`), and
`AcuTxCurrentNormalId` (`0x502`) are declared in `ams_config.hpp:180–182`
but marked **reserved for future use** (comment at `ams_config.hpp:177–179`,
alongside the retired `0x450`). **No current firmware emits them** —
`acu_can_task.cpp` transmits only `0x135` for current. There is no
threshold/warning/recovery current frame on the wire today; these IDs are
placeholders for a future warning stream. (Historically these were
`class_curent.cpp` frames; that legacy file is gone.)

### `0x40D – 0x412` — temperature forwarding (charger mode only) **[RETIRED — FDCAN2 drop + isoSPI]**

This charger-mode passthrough forwarded raw **FDCAN2** temperature RX onto
FDCAN1 as extended-ID frames. It no longer exists: FDCAN2 was dropped
(#388, `a885bf5`), so there is no FDCAN2 RX to forward, and temperatures now
come from the LTC6811-1 isoSPI chain (`docs/BMS_LTC6811.md`) rather than
CAN. Nothing in the current build emits `0x40D–0x412`. (The legacy
`class_bms.cpp` / `class_temperatures.cpp` source is gone.)

---

## TX — AMS pit-diag stream (FDCAN1, runtime-toggleable)

Optional full-grid diagnostic stream gated by a runtime CAN command.
Default OFF; intended for pit-stop debugging when the accumulator is
plugged into a `candump`-grade tool — either with the pack mounted in
the car (stationary) or out of the car on the charger. **Never on
during a track session** because nothing persists the flag across
reboots (#247).

### Enable / disable

| Direction | ID | DLC | Payload | Meaning |
|---|---|---|---|---|
| RX | `0x7F0` | 4 | `DE AD BE EF` | Pit-diag stream ON |
| RX | `0x7F0` | 4 | `00 00 00 00` | Pit-diag stream OFF |
| TX | `0x7F1` | 1 | `01` or `00` | One-shot ACK after a state change |

Reboot clears the flag — every power-cycle returns to OFF. Stream
continues through `fsm::State::Error` so charging-fault diagnostics
survive a predicate trip.

### Stream layout (cadence 1 Hz when enabled)

| IDs | Frames | Layout | Cadence |
|---|---|---|---|
| `0x680..0x697` | 24 cell-V frames | 4 cells per frame, BE u16 mV. Row-major over `cell_mV[5][19]`. Last frame has 3 real cells + 2-byte sentinel `0xFFFF`. Decode: `cell_index = 4·(id - 0x680) + slot; module = cell_index / 19; cell = cell_index % 19`. | 1 Hz |
| `0x6A0..0x6B8` | 25 cell-T frames | 8 NTCs per frame, signed i8 °C each. Row-major over `cell_tempC[5][40]`. Decode: `temp_index = 8·(id - 0x6A0) + slot; module = temp_index / 40; temp = temp_index % 40`. | 1 Hz |
| `0x6C0` | 1 FSM extended status | `[0]` FSM state, `[1]` mode_locked (0/1/2), `[2]` bits `2`=balance_override (#336, balancing paused by a fresh `0x103` "BALO"), `1`=TSMS, `0`=DASH_CHG, `[3]` AMS_OK GPIO, `[4..5]` PEC error total BE u16, `[6]` fault_reason (latched-ERROR predicate branch; see below), `[7]` fault_detail (BmsStale / CellUnderVoltage / CellOverVoltage / CellOverTemp: offending module index 0..4, or `0xFF` = none matched → inconsistent/torn snapshot; BmsModuleOffline: module_online_mask; else 0) | 1 Hz |
| `0x6C1` | 1 poll timing | `[0..1]` last V-poll ms BE u16, `[2..3]` worst-case V-poll BE u16, `[4..7]` last T-sweep failure mask LE u32 | 1 Hz |
| `0x6C2` | 1 balance mask A | DCC mask bits 0..63: `[i]` bit `b` = cell `(8·i + b)` of the row-major flat (`cell_idx = 19·m + c`) selected for discharge last balance window. | 1 Hz |
| `0x6C3` | 1 balance mask B | `[0..3]` DCC mask bits 64..94 (bit 31 reserved 0), `[4..5]` `balance_cycles_total` LE u16 (mod 65536), `[6..7]` `balance_cycles_active` LE u16. Reconstruct: bit `b` → `module = b/19, cell = b%19`. | 1 Hz |
| `0x6C4` | 1 boot diag | `[0..3]` `jump_reason` LE u32 (RTC→BKP2R; `config::JumpReason`; 0 = clean cold POR), `[4]` `g_app_init_progress` (0..7 milestone), `[5..7]` `g_fdcan1_start_result` LE u24 (0 = HAL_OK). | 1 Hz |
| `0x6C5` | 1 crash post-mortem | `[0]` stack-overflow seen (0/1), `[1]` stack-overflow watermark low byte (0xFF = API-failed sentinel), `[2..5]` failing-task `xTaskHandle` LE u32, `[6..7]` `malloc_failed_count` LE u16 (sat 0xFFFF). All 0 on a clean session. | 1 Hz |
| `0x6C6` | 1 firmware ID | `[0]` fw major, `[1]` minor, `[2]` patch, `[3..6]` `git_hash[0..3]` (first 4 of 8 bytes), `[7]` `bl_node_id` (`firmware_info.reserved[0]`). | 1 Hz |
| `0x6C7` | 1 per-IC PEC count (ICs 0..7) | 8 bytes, one saturating uint8 per chain index. Maps chain index → module: IC `2m`=upper / `2m+1`=lower of module `m`. (#258) | 1 Hz |
| `0x6C8` | 1 per-IC PEC count (ICs 8..9 + reserved) | `[0]` IC 8, `[1]` IC 9, `[2..7]` reserved 0 | 1 Hz |
| `0x6C9` | 1 FDCAN1 comms health | `[0..3]` `fdcan1_busoff_recovery_count` LE u32 (Bus-Off Stop/Start recoveries this session; `0` = never went Bus-Off), `[4..7]` `g_acu_tx_fail` LE u32 (ECU-TX-matrix enqueue failures). Lets the CAN-only bench confirm a Bus-Off recovery fired. (#331) | 1 Hz |

`0x6C0[6]` fault_reason values (#276): `0`=None, `1`=ForceError, `2`=BmsModuleOffline, `3`=BmsStale, `4`=CellUnderVoltage, `5`=CellOverVoltage, `6`=CellUnderTemp, `7`=CellOverTemp, `8`=CurrentSensorFault, `9`=CurrentStale, `10`=CurrentOverLimit, `11`=VcuStale, `12`=FsmError (FSM-driven Error path — precharge timeout / Transition guard; note a TSMS drop is non-latching since #327, not an Error). Latched once at the transition into ERROR; stays put until the latch clears. These enum mappings — plus `fsm_state` and `mode_locked` — are also emitted as machine-readable `VAL_` tables in [`docs/dbc/ams.dbc`](dbc/ams.dbc) (#291).

Encoders are pure-logic in
[`Core/Inc/app/pit_diag_emitter.hpp`](../Core/Inc/app/pit_diag_emitter.hpp);
unit tests in
[`tests/unit/test_pit_diag_emitter.cpp`](../tests/unit/test_pit_diag_emitter.cpp).
Dispatch + flag ownership in
[`Core/Src/app/acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp).

Bus cost: 59 frames/scan (24 cell-V + 25 cell-T + 10 status `0x6C0..0x6C9`)
× ~12 bytes-on-wire ≈ 5.7 kbit, ~11.4 ms at 500 kbps ≈ 1.1 % bus load (1 Hz)
when enabled.

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
| 0 | `state` | `ams::fsm::State` enum (0..5: Start, Precharge, Transition, Run, Charge, Error). **ECU↔AMS state contract — see below.** |
| 1 | `ams_ok` | GPIO PB4 read-back; 0 or 1 |
| 2 | `module_online_mask` | Low byte of `BmsState.module_online_mask`. 0x1F = all 5 modules healthy |
| 3 | reserved | 0 |
| 4-5 | `min_cell_mV` | Big-endian uint16, mV |
| 6-7 | `max_cell_mV` | Big-endian uint16, mV |

> **`0x4A0[0]` is a stable cross-board contract (#342).** It is the
> supported way for external consumers (notably the ECU) to read the AMS
> FSM state — emitted **continuously, including while latched in `Error`**
> (the telemetry block runs after the fault branch in `safety_task.cpp`).
> The ECU uses it to tell **`Start`** (re-armable) apart from **`Error`**
> (latched, reset-only) — both of which read `0x020 (ok_precharge) = 0`,
> so `ok_precharge` alone is ambiguous. **Do not reorder the `fsm::State`
> enum values or move byte 0 without coordinating with the ECU** — the
> enum is the contract. (The `VAL_` table in `ams.dbc` mirrors it.)

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
| 5 | `tsms_dash_chg_byte` | Cockpit-input snapshot (#246, always-on post-#251): bit 7 = sentinel `1`, bits 3:2 = `mode_locked` (00=Undecided / 01=Car / 10=Charger), bit 1 = TSMS, bit 0 = DASH_CHG. Was reserved=0 in pre-v1.5.0 builds. |
| 6 | `tx_fail_lo` | Low byte of `g_telemetry_tx_fail` |
| 7 | `heartbeat` | Wraparound 8-bit counter, increments per MainTask telemetry cycle (500 ms). Useful for detecting dropped frames on the receiver. |

> The legacy HIL_STUB byte 3..4 overlay (which carried bench diagnostic
> probes in place of `dc_bus_V`) was retired with the `AMS_BMS_HIL_STUB`
> flag; the equivalent diagnostics now live on the pit-diag stream
> (`0x680..0x6C8`). The flight layout above is the only layout.

### `0x4A4` — AMS relay status

Standard 11-bit. DLC 8. Cadence 100 ms (`RelayStatusPeriodMs`). Always-on
contactor + SDC-output snapshot so a datalogger can watch the AIR /
precharge sequence without arming the pit-diag stream. All values are
**MCU-side GPIO read-backs** (ODR): they confirm what the firmware is
driving the coils to, not that the contactor physically closed (same
caveat as `Relays::is_*_closed()`).

| Byte | Bit | Field | Notes |
|:---:|:---:|---|---|
| 0 | 0 | `air_negative` | AIR− (PB6) commanded closed |
| 0 | 1 | `air_positive` | AIR+ (PB5) commanded closed |
| 0 | 2 | `precharge` | Precharge contactor (PB7) commanded closed |
| 0 | 3 | `ams_ok` | AMS_OK / SDC-enable (PB4) high = AMS healthy |
| 1-7 | — | reserved | Zero; room for future output state |

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

### `0x101` — operator charge-mode request (#305)

| Field | Value |
|---|---|
| Direction | RX (operator tool → AMS) |
| Bus | FDCAN1 (standard 11-bit) |
| DLC | ≥ 4 |
| Payload | bytes `[0..3]` must equal the magic `43 48 52 47` ("CHRG"); other bytes ignored |
| Use | Declares "we are on the charger." The AMS enters **Charger** mode only if a fresh request (within `ChargeReqFreshMs` = 1000 ms) is present **and** the VCU is absent, at the Start→Precharge mode lock. A still-fresh `0x101` later also gates the Charger precharge→Transition proceed (closing AIR+). |
| Source | the charger emits it automatically the moment it is connected |
| Cadence | sent periodically (≥ 2 Hz) while connected, so it stays fresh both at the mode lock and at the precharge proceed |

The charger itself has no comms with the AMS, so this auto-emitted frame
is the positive charge-detect. Without it, a car with a dead VCU locks
**Car** mode and faults on VcuStale rather than silently charging. The
magic gate prevents bus noise / a stray frame from flipping the AMS into
a HV charge mode. Handled in `vehicle_service.cpp`.

Full charge bring-up (one button): with `0x101` fresh (charger connected)
and the VCU absent, the operator presses **DASH_CHG** (PF10, a momentary
edge-detected button) **once** to leave Start→Precharge. The proceed to
Transition (close AIR+) is then gated on `0x101` *still* being fresh — the
charger's auto-emitted "I'm connected and ready" signal — since Charger
has no `dc_bus_V` to voltage-gate on, and the charger soft-starts its own
output. If `0x101` goes stale before the proceed (charger unplugged), the
precharge holds and hits the `PrechargeMaxMs` timeout → Error rather than
closing AIR+ into a disconnected charger. See `state_machine.hpp` /
`safety_task.cpp`.

### `0x103` — operator balance-control override (#336)

| Field | Value |
|---|---|
| Direction | RX (operator tool → AMS) |
| Bus | FDCAN1 (standard 11-bit) |
| DLC | ≥ 4 |
| Payload | bytes `[0..3]` = `42 41 4C 4F` ("BALO") → **suppress** autonomous balancing; `42 41 4C 58` ("BALX") → **resume** auto. Other payloads ignored. |
| Source | the ChargerDisplayWario pit tool (BALANCE ON/OFF button) |
| Cadence | re-sent ≥ 2 Hz while suppress is held |
| Freshness | reverts to autonomous if silent > `BalanceOverrideFreshMs` (5000 ms) |

Lets the pit operator pause passive cell balancing during a charge (e.g.
for a clean cell-voltage snapshot) without a firmware change. The AMS
suppresses balancing only while a fresh "BALO" is in effect; a "BALX" or a
stale override resumes autonomous balancing. **Only affects balancing —
which runs in `Charge` only — so it can never touch an AIR / safety path,
and `Error` is unaffected.** The acknowledged override state is mirrored on
pit-diag `0x6C0[2]` bit 2 so the display can confirm receipt. Magic-gated
against bus noise; handled in `vehicle_service.cpp`, consumed by
`BmsPollTask` via `VehicleService::balance_suppressed`.

### `0x600` — start button **[RETIRED — fix/48]**

Replaced by the **TSMS** GPIO (PF9, active-high, external pull-down). The
FSM Start→Precharge transition now requires `TSMS` held **and** a
**DASH_CHG** press: TSMS is level-polled, while DASH_CHG (PF10) is a
momentary button edge-detected at the 10 ms cadence in `safety_task.cpp`
(#305). Run/Charge are sustained by TSMS alone.

### `0x401 – 0x406` — accumulator temperature sensors **[RETIRED — FDCAN2 drop + isoSPI]**

On-CAN temperature RX rode the legacy **FDCAN2** bus, which no longer
exists (#388, `a885bf5`). No current firmware consumes `0x401–0x406`:
accumulator temperatures are read over the LTC6811-1 isoSPI chain
(`docs/BMS_LTC6811.md`), not CAN. (The legacy `class_temperatures.cpp`
parser is gone.)

### `0x18FF50E7` — charger detected **[RETIRED — fix/48]**

The charger no longer communicates over CAN; the only thing on the
charger-assembly bus is an HMI for displaying cell V/T. Car-vs-charger
context is now distinguished by VCU `0x100` heartbeat freshness at the
moment of Start→Precharge transition: heard within `VcuFreshMs`
(1000 ms) → Car (target = Run), silent → Charger (target = Charge).
The captured mode locks for the rest of the boot cycle and never
re-evaluates.

---

## RX — bootloader-trigger command (FDCAN1)

Not from the legacy AMS — added in the refactor for in-system firmware
update via [isc-fs/stm32-can-bootloader](https://github.com/isc-fs/stm32-can-bootloader).

> Moved from FDCAN2 to FDCAN1 in v1.2.0 (#73). The in-band reboot
> trigger rides on the accumulator/vehicle bus (FDCAN1) alongside
> everything MingoCAN already sends. The AMS app is FDCAN1-only
> (FDCAN2 dropped in #388, `a885bf5`); the stm32-can-bootloader is a
> separate sector-0 image that brings up its own CAN peripheral after
> the reset — the app does not leave anything configured for it.

### `0x002` — request reboot into bootloader

| Field | Value |
|---|---|
| Direction | RX (host → AMS) |
| Bus | **FDCAN1** (accumulator bus; the sector-0 bootloader brings up its own CAN after the reset) |
| ID type | Standard 11-bit, very high arbitration priority |
| DLC | 4 |
| Payload | `{0xB0, 0x07, 0xAD, 0x11}` -- all 4 bytes must match exactly |
| Effect | `AcuCanTask` calls `ams::Bootloader::request_reboot()` which opens all relays, drains TX, writes `0xB00710AD` to `RTC->BKP0R`, and `NVIC_SystemReset()`s. The sector-0 bootloader's reset handler sees the magic, clears it (one-shot), and stays in BL mode awaiting flash commands on the CAN peripheral it brings up. |
| Failure modes | Wrong bus, wrong ID, wrong DLC, or any byte of the payload differing → frame silently dropped, no reboot. |

Source: [`Core/Inc/app/bootloader.hpp`](../Core/Inc/app/bootloader.hpp) (`matches_trigger`), [`Core/Src/app/bootloader.cpp`](../Core/Src/app/bootloader.cpp) (`request_reboot`), dispatched in [`Core/Src/app/acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp). Constants in [`Core/Inc/app/ams_config.hpp`](../Core/Inc/app/ams_config.hpp) (`BlBootReqCanId`, `BlBootReqPayload`, `BlBootReqDlc`).

---

## RX/TX — LOGFS log extraction (FDCAN1, ISO-TP)

On-CAN retrieval of the SD datalogs, so a card never has to be pulled from
a mounted accumulator. Serves [#406](https://github.com/isc-fs/IFS08-CE-AMS/issues/406)
(MingoCAN) and [#439](https://github.com/isc-fs/IFS08-CE-AMS/issues/439) (`ui.py`)
with one protocol.

Unlike everything else in this document, LOGFS is **not** a fixed-layout
signal frame — it is a request/response protocol over **ISO-TP
(ISO 15765-2)**, so it does not appear in `ams.dbc`.

### Addressing

| Direction | ID | Note |
|---|---|---|
| Host → AMS | `0x000 + NodeID` | `0x001` today; `0x002` after [#403](https://github.com/isc-fs/IFS08-CE-AMS/issues/403) |
| AMS → host | `0x010 + NodeID` | |

> **`0x002` is shared with the boot trigger above, and that is safe.**
> `matches_trigger` requires **DLC 4** *and* the exact `B0 07 AD 11`
> payload; an ISO-TP frame is always **DLC 8**, and `0xB0` can never be a
> valid ISO-TP PCI byte (only `0x0`–`0x3` are). The trigger check runs
> first in `AcuCanTask`, so a LOGFS frame passes through it untouched.

### Message framing

Each reassembled ISO-TP message is `[msg_type][opcode][payload…]`.

Requests use **`msg_type = 0x06` (`BL_MSG_APP_CTRL`)**, *not* `0x00`
(`BL_MSG_CMD`). Under `CMD` the application and bootloader would share one
opcode registry, and `0x2x` is bootloader-adjacent (`0x20` is reserved
there for a future `CMD_MEM_READ`). Responses reuse `ACK 0x01` / `NACK 0x02`.

> **Consequence for hosts:** the bootloader *silently drops* `APP_CTRL`. A
> LOGFS command sent while the node sits in the **bootloader** yields a
> **timeout, not a NACK** — indistinguishable from a dead node. Probe with a
> bootloader command (`DISCOVER 0x03` / `GET_FW_INFO 0x04` under `CMD`),
> which the BL *does* answer, before reporting a cable fault.

### Opcodes (little-endian payloads)

| Opcode | Name | Args | ACK payload |
|---|---|---|---|
| `0x01` | CONNECT | — | `major:u8`, `minor:u8` (app diag proto, currently 1.0) |
| `0x02` | DISCONNECT | — | — |
| `0x21` | LIST | `cursor:u16` | `next_cursor:u16`, `count:u8`, `entry × count` |
| `0x22` | OPEN | `index:u16` | `handle:u16`, `size:u32`, `crc32:u32` |
| `0x23` | READ | `handle:u16`, `offset:u32`, `len:u16` | `data` (≤ 512 B) |
| `0x24` | CRC | `handle:u16` | `crc32:u32` |
| `0x25` | CLOSE | `handle:u16` | — |

`entry` is a fixed **22 bytes** — `{index:u16, size:u32, mtime:u32,
name[12]}` — so a listing is parsed by stride. `mtime` is the packed FAT
stamp (`fdate << 16 | ftime`). `next_cursor == 0xFFFF` ends the listing.

On OPEN, **`crc32 == 0` means "not available — use `LOGFS_CRC`"**, not a literal
zero CRC. Every file this firmware seals gets a `.CRC` sidecar, but cards
written before sidecars existed have none, and OPEN must never block streaming
4 MiB to compute one.

**A short READ means end-of-file**, and is an ACK, not an error — that is
the host's normal termination signal. An oversized `len` is clamped to 512
rather than rejected.

**v1 is read-only.** There is no DELETE (`0x26` is deliberately
unimplemented and NACKs as unsupported; `0x27` is reserved for a
finalize-current-log opcode, not yet a committed contract): an extraction tool that can also
erase logs is the wrong tool to hand a pit crew.

### NACK codes

Values ≤ `0x10` are the bootloader's, reused verbatim.

| Code | Meaning |
|---|---|
| `0x02` | OUT_OF_BOUNDS — malformed/truncated args |
| `0x04` | FILE_NOT_FOUND |
| `0x06` | BAD_SESSION — no CONNECT, or wrong peer |
| `0x08` | BUSY — a request is already in flight |
| `0x11` | BAD_HANDLE — stale handle (note: **not** `0x08`, which is BUSY) |
| `0x12` | NO_SD_CARD |
| `0x13` | FS_ERROR |
| `0x14` | READ_ERROR |
| `0xFE` | UNSUPPORTED |

### Session

`CONNECT` opens a session; everything except CONNECT/DISCONNECT is refused
with `BAD_SESSION` without one, so a stray frame cannot stream the card to
whoever is on the bus. Idle timeout is **10 s**, after which any open file
handle is released — an operator who unplugs mid-pull does not pin it.
A CONNECT from a second host takes over, releasing the previous handle.

A `CMD`-typed (`0x00`) frame gets **silence**, not a NACK: the application
does not answer for the bootloader's namespace.

### Files visible

Only sealed `LOGnnnn.CSV`. The active `LOGnnnn.TMP` is excluded because its
length would be stale before a host finished reading it, and `LOGnnnn.CRC`
sidecars are an implementation detail. The sidecar holds the CRC-32
accumulated as rows were written, so `CRC` answers without re-reading the
file; absent one, it falls back to streaming.

CRC-32 is **ISO-HDLC / zlib** (poly `0xEDB88320` reflected, init and final
XOR `0xFFFFFFFF`) — i.e. Python's `zlib.crc32`. Deliberately *not* the
STM32 hardware CRC default (CRC-32/MPEG-2), which is a different checksum.

### Throughput — plan for it

ISO-TP carries 7 of every 8 bytes, so a 512-byte READ is ~76 frames ≈ 19 ms
at 500 kbit/s → **~27 KB/s on an idle bus**. With `LogFileMaxBytes` at 4 MiB
that is **~2.5–3 min per file**, and the bus is not idle. Larger reads buy
~5%; per-frame overhead dominates. Hosts should show progress, let operators
pick files from `LIST` sizes, and resume via the offset-based `READ`.

Source: [`Core/Inc/app/isotp.hpp`](../Core/Inc/app/isotp.hpp),
[`diag_proto.hpp`](../Core/Inc/app/diag_proto.hpp),
[`diag_dispatch.hpp`](../Core/Inc/app/diag_dispatch.hpp),
[`logfs_server.hpp`](../Core/Inc/app/logfs_server.hpp),
[`crc32.hpp`](../Core/Inc/app/crc32.hpp); FatFs backend in
[`sd_logger_task.cpp`](../Core/Src/app/sd_logger_task.cpp), CAN route in
[`acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp).

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

- [x] ~~Widen `0x450` payload to signed 16-bit mA.~~ Done differently:
      `0x450` retired, `0x135` carries signed 16-bit deciamps (fix/53).
- [x] ~~Confirm `0x600` bus assignment.~~ Retired (fix/48); replaced by the
      TSMS/DASH_CHG GPIOs. The app is FDCAN1-only (FDCAN2 dropped, #388).
- [x] ~~Confirm `0x401–0x406` bus assignment.~~ Retired; accumulator
      temperatures now arrive over LTC6811-1 isoSPI, not CAN.
- [ ] Agree on a new pack-status frame on FDCAN1 at 100 ms cadence with
      voltage / current / state / fault word packed together (planned for
      Phase 4, feat/12).
