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

## Module addressing — BMS slaves (5 modules)

Each BMS slave is identified by a CAN-ID offset of `0x1E` (30) from the
master ID `0x12C`. The refactor must keep this scheme.

| Module index | Voltage request ID | Voltage response IDs | Temperature request ID | Temperature response IDs |
|---|---|---|---|---|
| 0 | 0x12C | 0x12D – 0x131 | 0x140 | 0x14D – 0x151 |
| 1 | 0x14A | 0x14B – 0x14F | 0x15E | 0x16B – 0x16F |
| 2 | 0x168 | 0x169 – 0x16D | 0x17C | 0x189 – 0x18D |
| 3 | 0x186 | 0x187 – 0x18B | 0x19A | 0x1A7 – 0x1AB |
| 4 | 0x1A4 | 0x1A5 – 0x1A9 | 0x1B8 | 0x1C5 – 0x1C9 |

Module index of an incoming frame is recovered as
`m = (id - base_voltage_request) / 0x1E` (legacy uses `id % CANID`, same
effect modulo collisions — TO BE VERIFIED before refactor lands).

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

### `0x12C` — minimum cell voltage telemetry

| Field | Value |
|---|---|
| Direction | TX (AMS → vehicle) |
| Bus | FDCAN1 |
| ID type | **Extended** |
| DLC | 2 |
| Period | 500 ms |
| Suppressed when | charging |

Payload (big-endian, mV):

| Byte | Field |
|---|---|
| 0 | `MIN_V >> 8` |
| 1 | `MIN_V & 0xFF` |

Source: `select_state()` in `module_state_machine.cpp:150–157`.

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

### `0x600` — start button

| Field | Value |
|---|---|
| Direction | RX (VCU → AMS) |
| Bus | FDCAN1 (legacy code accepts on FDCAN2 too — VERIFY) |
| ID type | Standard |
| DLC | 1 |
| Decode | byte 0 = 0/1, drives `start` → `transition` transition |

Source: `parse_state()` `module_state_machine.cpp:329–331`.

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

### `0x18FF50E7` — charger detected

| Field | Value |
|---|---|
| Direction | RX (charger → AMS) |
| Bus | FDCAN1 |
| ID type | Extended (29-bit) |
| Effect | Sets `flag_charger = 1` across all modules |
| Side effects | Enables temperature forwarding, alters relay/state behaviour, suppresses balancing TX |

Source: `parse_state()` `module_state_machine.cpp:350`.

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
