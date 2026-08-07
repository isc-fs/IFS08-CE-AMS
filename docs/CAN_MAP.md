# AMS CAN map

The wire format the AMS firmware emits and consumes on **FDCAN1**, the only
CAN peripheral the application uses.

## Read this first: where the truth lives

The byte layouts are **code-first**. Each message is declared once, in a
`.def` file under [`Core/Inc/can/messages/`](../Core/Inc/can/messages/), using
the macro DSL in [`can_dsl.hpp`](../Core/Inc/can/can_dsl.hpp). From that single
declaration `can_codecs.hpp` mechanically generates, in five preprocessor
passes: the typed struct, the firmware encoder, the firmware decoder, a runtime
descriptor table, and a compile-time bit-overlap `static_assert`. The DBC in
`docs/dbc/ams.dbc` is generated from the *same* descriptors by
`tools/dbc_dump.cpp`, and CI regenerates and diffs it.

So the ordering of authority is:

1. `Core/Inc/can/messages/*.def` — the layout. Change here, everything follows.
2. `Core/Inc/can/can_codecs.hpp` — the array-of-frames families (`ALL_ARRAYS`),
   which have no `.def` yet.
3. `Core/Inc/app/ams_config.hpp` — IDs, cadences, magics, DLCs.
4. This document — prose, reasoning, and the parts that are not in any of the
   above (why a frame exists, what is untested).

If this page and a `.def` disagree, the `.def` is right and this page is a bug.

```bash
# Regenerate the DBC after any wire-format change (CI enforces the diff).
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump > docs/dbc/ams.dbc
```

The generated DBC is deliberately lean: `BO_`/`SG_` rows plus
`GenMsgCycleTime`, and **no `VAL_` enum tables and no `CM_` comments**
(`dbc_dump.cpp` says so explicitly). The enum meanings — FSM state, mode,
fault reason — live only here and in the headers. Do not expect a DBC importer
to name them for you.

---

## Bus

**500 kbps classic CAN, sample point 68.75 %.** Derived, not asserted: FDCAN
kernel clock is HSE = 24 MHz (`stm32h7xx_hal_msp.c` selects
`RCC_FDCANCLKSOURCE_HSE`; `HSE_VALUE` is 24 000 000 in `stm32h7xx_hal_conf.h`),
`NominalPrescaler` 3, `NominalTimeSeg1` 10, `NominalTimeSeg2` 5, `SJW` 1
(`main.c` `MX_FDCAN1_Init`). One bit = 1 + 10 + 5 = 16 tq; 24 MHz / 3 / 16 =
500 kbps; sample point = 11/16 = 68.75 %.

The bit rate is **not** an AMS-local choice. Every node on this bus — VCU, ECU,
charger tooling — plus the `isc-fs/stm32-can-bootloader` sector-0 image must
agree, and the bootloader's rate is compiled in. Raising it is a coordinated
change across repositories, and a bit error during a flash write bricks the
node. (The team's recorded reason for staying at 500 k is a bench-measured
error rate at 1 Mbps near the signal-integrity margin. That measurement is not
reproducible from this tree — treat it as a lab observation, not a datum.)

`FrameFormat = FDCAN_FRAME_CLASSIC`, `BitRateSwitch = FDCAN_BRS_OFF` on every
TX. **No CAN-FD, no bit-rate switching, ever.**

| | |
|---|---|
| Peripheral | FDCAN1 — accumulator / vehicle / telemetry / diag / boot-trigger |
| Frame format | **Standard 11-bit only.** Extended frames are rejected at the hardware global filter |
| Filter | `HAL_FDCAN_ConfigGlobalFilter(ACCEPT_IN_RX_FIFO0, REJECT, REJECT_REMOTE, REJECT_REMOTE)` — accept all unmatched standard into FIFO0, reject extended, reject remote (`app_init_task.cpp`) |
| TX FIFO depth | 16 (`TxFifoQueueElmtsNbr`), `AutoRetransmission` **DISABLE** |
| RX FIFO depth | 32 |

Consequence of `AutoRetransmission = DISABLE`: a frame that loses arbitration
and then errors is **dropped, not retried**. Every signal on this bus is
periodic and self-refreshing, which is what makes that acceptable — nothing
here is a one-shot command whose loss matters. If you ever add one, it must
carry its own repeat/ack.

**The app is FDCAN1-only.** There is no `MX_FDCAN2_Init` and no `hfdcan2`
handle in `main.c`; the only residue is pin PB13 still muxed to
`GPIO_AF9_FDCAN2` and unused. `CanBus::Bms` survives in `can_frame.hpp` purely
as a "wrong bus" sentinel for dispatch-rejection tests. The bootloader is a
separate sector-0 image that brings up its own CAN peripheral from scratch
after the magic-reset jump — the app leaves nothing configured for it.

### Bus-Off recovery

The STM32H7 M_CAN latches Bus_Off after sustained TX errors, and Bus_Off sets
`CCCR.INIT`, which halts **both** TX and RX. The node goes silent, stops
ACKing, and **does not self-clear**. `AcuCanTask` polls
`HAL_FDCAN_GetProtocolStatus` every loop pass and issues a rate-limited
Stop→Start; the attempt count is published on `0x6C9[0..3]` so a CAN-only bench
can confirm a recovery fired without a debugger. On the car this has never been
needed only because the bus always has another ACKing node.

---

## Full ID index

Everything the firmware puts on, or takes off, the wire.

| ID | Name | Dir | DLC | Period | Layout source |
|---|---|---|---|---|---|
| `0x002` | BL_boot_trigger | RX | 4 (exact) | — | `rx_bl_boot_trigger.def` |
| `0x002` | LOGFS request (ISO-TP) | RX | 8 | — | `isotp.hpp` / `diag_proto.hpp` |
| `0x012` | LOGFS response (ISO-TP) | TX | 8 | — | `isotp.hpp` / `diag_proto.hpp` |
| `0x020` | ACU_ok_precharge | TX | 1 | 100 ms | `acu_ok_precharge.def` |
| `0x021` | ACU_discharge_interlock | TX | 1 | 100 ms | `acu_discharge_interlock.def` |
| `0x100` | VCU_dc_bus_heartbeat | RX | 3 (2 accepted) | 10 ms | `rx_vcu_dc_bus.def` |
| `0x101` | Operator_charge_request | RX | ≥ 4 | ≥ 2 Hz | `rx_operator_charge_request.def` |
| `0x103` | Operator_balance_override | RX | ≥ 4 | ≥ 2 Hz | `rx_operator_balance_override.def` |
| `0x104` | Operator_balance_modules | RX | ≥ 5 | ≥ 2 Hz | `rx_operator_balance_modules.def` |
| `0x12C` | ACU_v_cell_min | TX | 2 | 100 ms | `acu_v_cell_min.def` |
| `0x130` | ACU_soc | TX | 1 | 250 ms | `acu_soc.def` |
| `0x131` | ACU_vmin_module_a | TX | 6 | 100 ms | `acu_vmin_module_a.def` |
| `0x132` | ACU_vmin_module_b | TX | 4 | 100 ms | `acu_vmin_module_b.def` |
| `0x133` | ACU_vmax_module_a | TX | 6 | 100 ms | `acu_vmax_module_a.def` |
| `0x134` | ACU_vmax_module_b | TX | 4 | 100 ms | `acu_vmax_module_b.def` |
| `0x135` | ACU_currents | TX | 4 | 50 ms | `acu_currents.def` |
| `0x136` | ACU_tmax_module_a | TX | 6 | 250 ms | `acu_tmax_module_a.def` |
| `0x137` | ACU_tmax_module_b | TX | 6 | 250 ms | `acu_tmax_module_b.def` |
| `0x4A0` | AMS_status | TX | 8 | 500 ms | `ams_status.def` |
| `0x4A1` | AMS_pack | TX | 8 | 500 ms | `ams_pack.def` |
| `0x4A2` | AMS_temps | TX | 8 | 500 ms | `ams_temps.def` |
| `0x4A4` | AMS_relay_status | TX | 8 | 100 ms | `relay_status.def` |
| `0x680..0x697` | PitDiag cell grid (24) | TX | 8 | 1 Hz, gated | `can_codecs.hpp` `ALL_ARRAYS[0]` |
| `0x6A0..0x6B8` | PitDiag temp grid (25) | TX | 8 | 1 Hz, gated | `can_codecs.hpp` `ALL_ARRAYS[1]` |
| `0x6C0..0x6C9` | PIT status frames (10) | TX | 8 | 1 Hz, gated | `pit_*.def` |
| `0x6CA` | AMS_fw_health | TX | 8 | 1 Hz, **ungated** | `ams_fw_health.def` |
| `0x6CB` | PIT_balance_health | TX | 8 | 1 Hz, gated | `pit_balance_health.def` |
| `0x6D0..0x6E7` | ADOW raw PU grid (24) | TX | 8 | bench build only | none — `pit_diag_emitter.hpp` |
| `0x6E8..0x6FF` | ADOW raw PD grid (24) | TX | 8 | bench build only | none — `pit_diag_emitter.hpp` |
| `0x7F0` | PitDiag_cmd | RX | 4 (exact) | — | `rx_pitdiag_cmd.def` |
| `0x7F1` | PitDiag_ack | TX | 1 | one-shot | `pit_ack.def` |

Declared in `ams_config.hpp` but **never transmitted by any code path**:
`0x4A3` (`AmsTelemDiagId`), `0x500`/`0x501`/`0x502`
(`AcuTxCurrentWarnId` / `OverLimitId` / `NormalId`). They have no `.def`, no DBC
row, and no call site. Treat them as reserved IDs, not as protocol.

---

## TX — AMS to ECU (the "ECU TX matrix")

The ECU's second CAN peripheral is wired to AMS FDCAN1; these frames feed the
ECU's own logic and the real-time telemetry uplink. Scheduled by `AcuCanTask`
in three cadence groups, all driven by a deadline computed against the RX queue
wait so TX jitter stays bounded and RX latency stays low:

| Group | Constant | Frames |
|---|---|---|
| 50 ms | `EcuFastTxMs` | `0x135` |
| 100 ms | `EcuMidTxMs` | `0x020`, `0x021`, `0x12C`, `0x131`, `0x132`, `0x133`, `0x134` |
| 250 ms | `EcuSlowTxMs` | `0x136`, `0x137`, `0x130` |

All sends in this matrix are **non-blocking**. A momentarily full TX FIFO
increments `g_acu_tx_fail` (published on `0x6C9[4..7]`) rather than stalling
the cadence — losing one periodic frame is strictly better than delaying the
next one.

### `0x020` — ok_precharge

DLC 1, 100 ms. Byte 0 = `1` iff the FSM state is `Run` (3) or `Charge` (4),
else `0`.

`ok_precharge = 0` is **ambiguous on its own**: it is equally true in `Start`
(re-armable) and in `Error` (latched, reset-only). A consumer that needs to
tell those apart must read `0x4A0[0]`.

### `0x021` — discharge_interlock

DLC 1, 100 ms. Two bits, byte 0:

| Bit | Field | Meaning |
|---|---|---|
| 0 | `fsm_in_start` | FSM state is `Start`. AIR−, AIR+ and precharge are all commanded open, so any voltage on the link is left over rather than something the AMS is putting there. Deliberately excludes `Precharge`/`Transition`, where the link is *supposed* to be rising and a forced discharge would fight it |
| 1 | `tsms` | Shutdown circuit is complete (PF9 high). Any open shutdown element pulls it low, so `1` means the discharge relay is **energised** and the bleed is **disconnected** |

**Why this frame exists.** The DC-link bleed relay is normally-closed and wired
into the shutdown circuit with no software control. Opening the SDC
de-energises it, the bleed resistor connects, and the link drains. Close the
SDC again before the link has finished draining and the relay re-energises: the
discharge **stops part-way** and the link is stranded at an unpredictable
voltage, with nothing to drain it while the SDC stays closed.

The AMS cannot fix that. Its own leg of the loop, `AMS_OK`, latches in
**hardware** (self-holding relay K5 plus an RST_BMS button the driver cannot
reach), so firmware can never pulse it low to reopen the SDC. The ECU can fix
it: it owns a normally-closed relay in series with the discharge relay coil.
But the ECU cannot see either bit above. Hence this frame.

The intended ECU rule is `fsm_in_start AND tsms AND (its own dc_bus above
threshold)`, **latched on entry** and released on its own measurement falling.
The latch is not an optimisation: securing the discharge connects the bleed,
which is exactly what `tsms` reports on, so a continuously-evaluated rule
would falsify its own trigger the instant it acted.

Both bits are **raw observations, not a request**. The ECU owns the decision
because it owns the DC-link measurement that decides it; shipping a
pre-computed request would insert a stale CAN value into the middle of that
judgement.

> **The ECU half exists, and has never run against this one.** `IFS08-CE-ECU`
> `dev` mirrors this `.def` field for field, supplies the third term from its
> own DC-link measurement, latches the hold and releases at
> `DischargeReleaseV` = 10 V — below this repo's `DcBusDischargedV` = 60 V, so
> the re-arm gate clears before the ECU lets go. It gives up after 30 s and
> reports a fault if the link does not fall.
>
> What is *not* verified is the pairing: the AMS side is exercised only by
> `tests/unit/test_state_machine.cpp`, the ECU side only by its SIL, and no
> bench has ever had both boards on it. ECU `main` predates the feature and
> sends `0x100` at DLC 2, where `ecu_discharge_capable` never latches and this
> frame changes nothing.

### `0x12C` — pack-wide minimum cell voltage

DLC 2, 100 ms. Bytes 0–1: `min_cell_mV`, big-endian uint16, mV.

### `0x130` — pack state of charge

DLC 1, 250 ms. Byte 0: `0..100 %`.

**`0xFF` is the "no trustworthy estimate" sentinel, not a reading.** It appears
before the first OCV anchor and whenever the pack current sensor faults or goes
stale, because Coulomb counting cannot be trusted across an unmeasured
interval. A consumer that clamps `0xFF` to 100 % will display a full pack on a
flat one. Render it as *unknown*.

Produced by `CurrentSensorTask` via `soc_estimator.hpp`: Coulomb counting
against `PackCapacityMah` (18 000 mAh = 6P × 3.0 Ah VTC6), corrected by an EKF
whose observation matrix is dOCV/dSoC — so the gain self-schedules and goes
nearly zero on the OCV plateau where voltage carries no SoC information. The
anchor is taken off the **minimum** cell (`current_task.cpp` passes
`bms.min_cell_mV`), because usable pack charge is set by the weakest element.

**TELEMETRY ONLY — no safety predicate reads it.**

The 250 ms in the `.def` is not decorative: it must match the scheduler group
`tx_soc()` actually rides in, because it becomes `GenMsgCycleTime` in the DBC
and every consumer sizes its receive timeout from that. If `tx_soc()` moves
cadence group, this number moves with it.

### `0x131` / `0x132` — per-module minimum cell voltage

`0x131` DLC 6, 100 ms: modules 0, 1, 2 as BE uint16 mV.
`0x132` DLC 4, 100 ms: modules 3, 4.

**Offline module sentinel: `0xFFFF`** (`bms_service.cpp` leaves the aggregate at
`uint16_t` max when no cell contributed).

### `0x133` / `0x134` — per-module maximum cell voltage

Same shape as `0x131`/`0x132` with `vmax_module[]`.

**Offline module sentinel: `0x0000`** — different from vmin, because vmax is a
running maximum seeded at 0 and vmin a running minimum seeded at `0xFFFF`. Both
sentinels are "impossible reading in the direction that would look safe", which
is the property that matters: an offline module never reports a comfortable
number.

### `0x135` — pack + DCDC current

DLC 4, 50 ms.

| Bytes | Field |
|:---:|---|
| 0–1 | `current_accu_dA` BE **int16**, deciamps (1 LSB = 0.1 A) |
| 2–3 | `current_dcdc_dA` BE **int16**, deciamps |

**Sign convention: `+` = discharge, `−` = charge.** This convention holds
across the whole codebase; do not re-derive it per frame.

The DBC carries `factor = 0.1` as *display* metadata. The C++ struct holds raw
deciamps and the encoder never applies the factor — the mA→dA conversion
happens in `acu_tx_encoders.hpp` (`mA_to_deciamps_i16`, round-to-nearest then
saturate to int16) before the struct is populated. This is the DSL's global
convention: **structs hold raw wire integers; factor/offset are DBC display
only.**

Pack current is read **differentially** on PF7/PF8 (ADC3 INP3/INN3).

### `0x136` / `0x137` — per-module maximum temperature

Both DLC 6, 250 ms. BE int16 °C.

`0x136`: modules 0, 1, 2. `0x137`: modules 3, 4, then `tmax_dcdc`.

**Offline module sentinel: `INT16_MIN` (`0x8000`).**

`0x137` bytes 4–5 are a **stub**: `config::DcdcTempStubValue` = `-32768` is
transmitted unconditionally until a DCDC temperature sensor is physically
wired. It is not a reading and never has been.

---

## TX — AMS telemetry

Emitted by `MainTask` (the CubeMX thread is still named `SafetyTask`). Pure
encoders in
[`telemetry_encoders.hpp`](../Core/Inc/app/telemetry_encoders.hpp), byte-level
tests in `tests/unit/test_telemetry_encoders.cpp`.

### `0x4A0` — AMS status

DLC 8, 500 ms.

| Byte | Field | Notes |
|:---:|---|---|
| 0 | `fsm_state` | `ams::fsm::State`: 0 Start, 1 Precharge, 2 Transition, 3 Run, 4 Charge, 5 Error |
| 1 | `ams_ok` | PB4 read-back, normalised to 0/1 |
| 2 | `module_online_mask` | `0x1F` = all 5 modules healthy |
| 3 | reserved | 0 |
| 4–5 | `min_cell_mV` | BE uint16, mV |
| 6–7 | `max_cell_mV` | BE uint16, mV |

> **`0x4A0[0]` is a stable cross-board contract.** It is the supported way for
> an external consumer — notably the ECU — to read the AMS FSM state, and it is
> emitted **continuously, including while latched in `Error`** (the telemetry
> block in `safety_task.cpp` runs after the fault branch). It is what
> disambiguates `Start` from `Error`, which `0x020` cannot. **Do not reorder
> the `fsm::State` enum values and do not move byte 0 without coordinating with
> the ECU.** The enum *is* the contract, and the generated DBC does not carry a
> `VAL_` table to remind anyone.

### `0x4A1` — AMS pack

DLC 8, 500 ms.

| Bytes | Field | Notes |
|:---:|---|---|
| 0–3 | `pack_voltage_mV` | **Little-endian** uint32, mV, sum of all cells |
| 4–7 | `filtered_mA` | **Little-endian signed** int32, mA. `+` discharge, `−` charge |

Note the endianness flip against `0x4A0[4..7]`. Both are correct; the DSL
records each field's endianness individually and there is no frame-wide rule.
Always read the `.def`.

### `0x4A2` — AMS temps, DC bus, cockpit, heartbeat

DLC 8, 500 ms.

| Byte | Field | Notes |
|:---:|---|---|
| 0 | `min_tempC` | int8 °C, saturating clip of `BmsState.min_tempC` |
| 1 | `max_tempC` | int8 °C |
| 2 | `avg_tempC` | int8 °C |
| 3–4 | `dc_bus_V` | **Little-endian** uint16, V — the last value received on `0x100`, with no freshness attached |
| 5 | `tsms_dash_chg_byte` | cockpit snapshot, below |
| 6 | `tx_fail_count_lo` | low byte of `g_telemetry_tx_fail` |
| 7 | `heartbeat` | 8-bit wraparound, +1 per 500 ms telemetry cycle |

Byte 5, assembled in `safety_task.cpp`:

| Bit(s) | Meaning |
|---|---|
| 7 | sentinel, always `1` — distinguishes a live byte from a zeroed reserved one |
| 3:2 | `mode_locked`: 00 Undecided, 01 Car, 10 Charger |
| 1 | TSMS (PF9) level |
| 0 | DASH_CHG (PF10) level |

Byte 3–4 is a **frozen** value when the VCU is silent, not a stale-flagged one.
`VehicleState` keeps the last received `dc_bus_V` forever. If you are logging
this for post-analysis, correlate it against the VCU's own `0x100` presence in
the same capture; the AMS gives you no freshness bit here.

### `0x4A4` — AMS relay status

DLC 8, **100 ms** (`RelayStatusPeriodMs`) — four times faster than the rest of
the telemetry block, on its own timer, so a datalogger can watch the whole
AIR/precharge sequence without arming the pit-diag stream.

| Byte | Bit | Field |
|:---:|:---:|---|
| 0 | 0 | `air_negative` — AIR− (PB6) commanded closed |
| 0 | 1 | `air_positive` — AIR+ (PB5) commanded closed |
| 0 | 2 | `precharge` — precharge contactor (PB7) commanded closed |
| 0 | 3 | `ams_ok` — AMS_OK / SDC enable (PB4) high = AMS not blocking |
| 1–7 | — | reserved, zero |

**These are MCU-side GPIO read-backs (ODR).** They tell you what the firmware
is driving the coils to. They do **not** tell you a contactor physically
closed — there is no auxiliary-contact feedback in this design. Same caveat as
`Relays::is_*_closed()`. A welded or failed-open contactor looks identical here
to a healthy one.

---

## TX — firmware health (ungated)

### `0x6CA` — AMS_fw_health

DLC 8, **1 Hz, always on**, regardless of the pit-diag arm state. Emitted from
`AcuCanTask` so a passive listener sees AMS liveness the instant the board
powers up. Byte-for-byte parity with the ECU's own `0x704` health frame, so one
tool decodes both.

| Bytes | Field | Notes |
|:---:|---|---|
| 0–1 | `free_heap` | BE uint16, bytes, clamped |
| 2–3 | `min_free_heap` | BE uint16, bytes — min-ever, the number that actually matters |
| 4 | `task_liveness` | bitfield, below |
| 5 | `reset_cause` | enum, below |
| 6 | `uptime_s` | uint8 seconds, **wraps at 256** |
| 7 | `last_fault` | sticky fault class, below |

`task_liveness` (bit set = that task stepped in the last second; the 1 Hz emit
samples **and clears** the field, so a stalled task leaves its bit at 0):

| Bit | Task |
|---|---|
| 0 | `MainStepped` — MainTask 10 ms loop |
| 1 | `CanRx` — AcuCanTask serviced its RX queue |
| 2 | `CanTx` — AcuCanTask ran its periodic TX scheduler |
| 3 | `Housekeeping` — BmsPollTask isoSPI sweep |

`reset_cause` (`config::ResetCause`): 0 Unknown, 1 PowerOn, 2 Pin, 3 Software,
4 Iwdg, 5 Wwdg, 6 LowPower. The specific causes win over the generic Pin/POR a
reset usually also asserts; POR beats Pin because a cold boot raises both.

`last_fault` (`config::LastFault`, latched in RTC backup register 3 across a
reset, cleared on a clean boot): 0 None, 1 HardFault, 2 StackOverflow,
3 MallocFail, 4 AssertFail.

`0x6CA` is the frame to look for first when a board appears dead. It is
distinct from the *gated* `0x6C9` comms-health frame, which needs the pit-diag
stream armed.

---

## TX — pit-diag stream (gated)

A full-grid diagnostic dump, off by default, armed by a CAN command. Intended
for pit-side debugging with a `candump`-grade tool, with the pack either
stationary in the car or out of it on the charger.

The arm flag `s_pit_diag_enabled` lives in `.bss` in `acu_can_task.cpp`, so
**every power-cycle returns to OFF** — it cannot be inherited from a previous
session, and it also cannot be relied on to survive a reset mid-debug. The
stream keeps running through `fsm::State::Error`, deliberately: a charging
fault is exactly when you want the diagnostics.

### Enable / disable

| Dir | ID | DLC | Payload | Effect |
|---|---|---|---|---|
| RX | `0x7F0` | 4 (exact) | `DE AD BE EF` | stream ON |
| RX | `0x7F0` | 4 (exact) | `00 00 00 00` | stream OFF |
| TX | `0x7F1` | 1 | `01` / `00` | one-shot ACK, sent only on an actual state change |

Any other payload, or any DLC other than exactly 4, is not a pit-diag command
and falls through to normal dispatch (`classify_command`). Enabling resets the
scan clock so the first scan lands within 1 s of the command, not at whatever
residual phase was on the timer.

### Cost, and why it is the only blocking TX path

A scan is **60 frames**: 24 cell + 25 temp + 10 status (`0x6C0..0x6C9`) +
`0x6CB`. All DLC 8. A standard-ID 8-byte data frame is ~111 bits on the wire
before stuffing, so a scan is ≈6.7 kbit ≈ 13 ms of bus time — about **1.3 %
average load at 1 Hz**, up to ~1.6 % with worst-case bit stuffing.

60 frames do not fit a 16-deep TX FIFO. Without flow control, frames 17+ are
silently NACKed and only the front of the burst reaches the wire. So the
pit-diag burst — and *only* the pit-diag burst — uses a yield-while-full send
(`osDelay(1)` on a full FIFO), costing ~6 ms of AcuCanTask time per scan. The
flight TX matrix stays non-blocking on purpose: a FIFO bump there must bump a
counter, never stall a safety-adjacent cadence.

### Cell + temperature grids

| IDs | Frames | Layout |
|---|---|---|
| `0x680..0x697` | 24 | 4 cells per frame, BE uint16 mV, row-major over `cell_mV[5][19]` |
| `0x6A0..0x6B8` | 25 | 8 NTCs per frame, int8 °C, row-major over `cell_tempC[5][40]` |

Decode:

```
cell_index = 4 * (id - 0x680) + slot;   module = cell_index / 19;  cell = cell_index % 19
temp_index = 8 * (id - 0x6A0) + slot;   module = temp_index / 40;  temp = temp_index % 40
```

95 cells / 4 = 23 full frames + 3 cells, so the last cell frame's final slot
pair is the sentinel. 200 NTCs / 8 = 25 exactly, no padding.

**`0xFFFF` in a cell slot means "no measurement", and there are three separate
reasons for it** (`encode_cell_frame`):

1. Slot past cell 94 — padding in the last frame.
2. The module is offline (`module_online_mask` bit clear). `cell_mV` still
   holds the last good voltages, and emitting those would show a disconnected
   chain as live data.
3. The cell straddles an **ADOW-confirmed open tap**. When a sense node floats,
   *both* cells sharing it are displaced — one high, one low — and neither is
   recoverable; only their sum survives. Emitting a number that looks like a
   measurement and is not would be worse than emitting nothing. `cell_mV` keeps
   the raw split internally for the pair average.

Temperatures use `config::NtcNoReading` (`INT16_MIN`) for an offline module,
which clips to `-128` on the wire — so **`-128 °C` is a sentinel, not a
reading**.

The grid layouts live in `can_codecs.hpp` `ALL_ARRAYS`, not in a `.def`: they
are windowed array views selected at runtime and the DSL has no multiplexed
representation yet. `dbc_dump` still expands them into one DBC message per
frame, so they *are* in `ams.dbc`.

### Status frames

#### `0x6C0` — FSM extended status

| Byte | Field |
|:---:|---|
| 0 | `fsm_state` (same encoding as `0x4A0[0]`) |
| 1 | `mode_locked`: 0 Undecided, 1 Car, 2 Charger |
| 2 | bit 0 `dash_chg`, bit 1 `tsms`, bit 2 `balance_override` |
| 3 | `ams_ok_gpio` |
| 4–5 | `pec_err_total` BE uint16, sum over all 10 ICs, saturating |
| 6 | `fault_reason` |
| 7 | `fault_detail` |

**`0x6C0[2]` bit 2 is not "an operator sent BALO".** It is set whenever the
*effective* balance command resolves to `Off`, which includes the dead-man
fallback: no `0x103` ever seen, or the last one older than
`BalanceOverrideFreshMs`. On a freshly booted board with no pit tool connected
the bit reads **1**. See `0x103` below.

`fault_reason` = `ams::safety::FaultReason`, **append-only wire contract, never
renumber**:

| | | | |
|---|---|---|---|
| 0 None | 1 ForceError | 2 BmsModuleOffline | 3 BmsStale |
| 4 CellUnderVoltage | 5 CellOverVoltage | 6 CellUnderTemp | 7 CellOverTemp |
| 8 CurrentSensorFault | 9 CurrentStale | 10 CurrentOverLimit | 11 VcuStale |
| 12 FsmError | 13 TempSensorDisconnected | 14 ChargerStale | 15 ChargerTsmsOpen |
| 16 CellOpenWire | | | |

**6 `CellUnderTemp` and 7 `CellOverTemp` cannot occur in the current build.**
Both are gated behind `config::TempFaultsTrusted`, which is **`false`**: NTC
temperatures come through the ADG731 mux and are not yet validated end-to-end,
so the firmware does not open the AIRs on them. Do not read "no over-temp
faults in the log" as "the pack stayed cool" — read it as "temperature is not a
fault source yet". Cell *voltage* protection is unaffected.

Other notes that matter when reading a log:

- **12 `FsmError`** has no enum slot (it is `safety::FsmErrorReason`, a bare
  constant) because the FSM, not the predicate set, produces it: precharge
  timeout, or the `Transition` bus-still-up guard failing. A TSMS drop in
  **Car** mode is *not* an error at all — it de-energises to `Start` without
  latching, so the driver can stop and restart unaided.
- **15 `ChargerTsmsOpen`** is the Charger-mode counterpart, and it *does*
  latch: scrutineering forbids re-activating a charge output after the SDC
  opens, so recovery needs a full reset.
- **14 `ChargerStale`** means the `0x101` heartbeat went silent beyond
  `ChargerStaleMs` while committed to Charger mode — the charger was unplugged
  mid-charge.
- **16 `CellOpenWire`** is an LTC6811 ADOW two-pass open-wire confirmation. It
  catches a broken sense tap that still reads *in range*, which a plain
  cell-mV plausibility check cannot. It faults in **any** state.

`fault_detail` depends on the reason: `BmsStale` / `CellUnderVoltage` /
`CellOverVoltage` / `CellOverTemp` carry the offending module index 0..4, or
`0xFF` if no module matched (an inconsistent or torn snapshot);
`BmsModuleOffline` carries the live `module_online_mask`;
`TempSensorDisconnected` and `CellOpenWire` carry their module masks; otherwise
0.

#### `0x6C1` — poll timing

`[0..1]` last voltage-poll ms BE uint16, `[2..3]` worst-case since boot BE
uint16 (both clipped at `0xFFFF`), `[4..7]` last temperature-sweep failure mask
LE uint32, one bit per NTC channel the chain did not return.

#### `0x6C2` / `0x6C3` — balance mask + cycles

`0x6C2`: DCC bits for flat cells 0..63 as a 64-bit LE mask.
`0x6C3`: `[0..3]` cells 64..94 in bits 0..30 (bit 31 reserved 0), `[4..5]`
`balance_cycles_total` LE uint16, `[6..7]` `balance_cycles_active` LE uint16,
both saturating at `0xFFFF`.

Flat index is row-major `cell_idx = 19*m + c`; reconstruct with
`module = b / 19, cell = b % 19`.

#### `0x6C4` — boot diag

`[0..3]` `jump_reason` LE uint32, read from RTC backup register 2
(`config::JumpReason`: 0 none/clean cold POR, `'JUMP'` CAN-triggered BL jump,
`'FAUT'` fault-forced, `'MANU'` operator-issued). `[4]` `app_init_progress`,
a milestone counter that reaches **7** on a fully successful init — anything
less pinpoints where `app_init_task.cpp` stopped. `[5..7]`
`fdcan1_start_result` LE uint24, 0 = `HAL_OK`.

#### `0x6C5` — crash post-mortem

`[0]` stack-overflow seen (0/1), `[1]` watermark low byte (`0xFF` also encodes
"the API call itself failed"), `[2..5]` failing task's `xTaskHandle` LE uint32,
`[6..7]` `malloc_failed_count` LE uint16 saturating. **All zero on a clean
session** — that is the normal reading.

#### `0x6C6` — firmware ID

`[0..2]` semver major/minor/patch, `[3..6]` first 4 bytes of the 8-byte git
hash, `[7]` bootloader node ID from `firmware_info.reserved[0]`. Lets a tool
verify at flash time that the app matches the bootloader it is talking to.

#### `0x6C7` / `0x6C8` — per-IC PEC error counts

Saturating uint8 per chain index. `0x6C7` = ICs 0..7, `0x6C8[0..1]` = ICs 8..9,
`0x6C8[2..7]` reserved 0.

Chain index → module: IC `2m` is module `m`'s **upper** LTC (cells 0..8), IC
`2m+1` is the **lower** (cells 9..18) — `CellsPerLtcUpper` = 9,
`CellsPerLtcLower` = 10. So a spike on `0x6C7[0]` reads "module 0's upper
LTC6811 is misbehaving". `0x6C0[4..5]` says the chain is unhealthy; these say
which IC.

Counts reset only on cold boot — there is no per-session clear.

#### `0x6C9` — FDCAN1 comms health

`[0..3]` `fdcan1_busoff_recovery_count` LE uint32 (Stop/Start attempts this
session; 0 = never went Bus-Off), `[4..7]` `g_acu_tx_fail` LE uint32 (ECU-TX
matrix enqueue failures). No saturation on either.

#### `0x6CB` — balance-quiesce health

`[0..3]` `g_balance_quiesce_count` LE uint32 (DCC successfully cleared before a
measurement), `[4..7]` `g_balance_quiesce_fail_count` LE uint32 (both WRCFGA
attempts failed, so that poll measured with cells still bleeding).

**The ratio is the diagnostic.** A bare fail count means nothing without the
attempt count. `ok` climbing with `fail` flat means the quiesce is healthy —
look elsewhere. `fail` climbing means cell voltages are being sampled under
bleed, which corrupts both the balance selector (it ranks raw `cell_mV`) and
the SoC filter (it corrects on `min_cell_mV`).

Note that `DCP = 0` on the ADCV does **not** cover a failed quiesce: per
LTC6811 datasheet Table 53 it suppresses discharge only on the cell being
measured and its immediate neighbours, so roughly half the selected cells keep
bleeding through the conversion. The quiesce is the only full stop.

### `0x6D0..0x6FF` — raw ADOW grids (bench build only)

Two more 24-frame blocks with the **same** window layout as the `0x680` cell
grid, carrying the raw ADOW pull-up (`0x6D0`) and pull-down (`0x6E8`) readings
so the open-wire check can be debugged against real hardware. `0xFFFF` = no
cell, or PEC-skipped this scan.

Gated on `config::AdowRawDiag`, which is **`false`** — the block is
dead-code-eliminated on flight builds and has no `.def` and no DBC row. If you
see traffic in `0x6D0..0x6FF` on a car, someone flashed a bench build.

---

## RX — vehicle and operator frames

Dispatched in `AcuCanTask` in a fixed order that matters: pit-diag command
first (cheap classify, and it must not be mistaken for a VCU frame on dispatch
failure), then the bootloader trigger, then LOGFS, then `VehicleService`.
Anything matching nothing increments `g_acu_rx_dropped_unknown`.

### `0x100` — VCU DC-bus heartbeat

**Standard 11-bit** (the hardware filter rejects extended frames, so a
29-bit `0x100` never reaches the firmware). Declared DLC **3**, **10 ms** —
the ECU posts it from its unconditional 10 ms control body, so that is the
rate, and it is what `GenMsgCycleTime` carries for other teams' tooling. It is
*not* `VcuStaleMs`: 200 ms is this repo's tolerance for the frame going quiet,
which is a different question and deliberately twenty frames of margin.

| Bytes | Field |
|---|---|
| 0–1 | `dc_bus_V` — **little-endian** uint16, volts: `(buf[1] << 8) | buf[0]` |
| 2 bit 0 | `discharge_engaged` |
| 2 bit 1 | `dc_bus_valid` — `dc_bus_V` is a present-tense measurement. `0` = do not use the voltage in **either** direction |

`VehicleService::update_from_frame` accepts **DLC ≥ 2** and ignores byte 2 when
it is absent.

#### Freshness is part of the precharge criterion, not a separate check

`VcuStaleMs` = **200 ms**, and it is enforced: `VcuStale` (reason 11) latches
Error once the FSM has committed to **Car** mode. But that alone is not enough,
and understanding why is the single most important thing on this page.

`VehicleState` stores the **last received** `dc_bus_V`. When the VCU stops
publishing, the number does not disappear — it **freezes**. Frozen at pack
voltage, it satisfies the "bus ≥ 95 % of cell-sum pack voltage" precharge
completion test *forever*, including after the link has actually bled to zero.
Closing AIR+ then puts full pack voltage across the contactor with nothing to
limit the inrush.

`VcuStale` cannot win that race: it is gated on `vcu_required`, which is false
in `Start`, so the value may already be arbitrarily old at the moment the
operator presses; and it needs 200 ms while the FSM steps every 20 ms. So
`precharge_target_reached()` **requires `dc_bus_fresh` as its first condition**
(`state_machine.hpp`), where `dc_bus_fresh` = "a `0x100` arrived within
`VcuStaleMs`, and at least one has ever arrived".

Two different freshness windows are applied to the same frame, deliberately:

| Window | Constant | Used for |
|---|---|---|
| 200 ms | `VcuStaleMs` | `dc_bus_fresh` — gates closing AIR+, and the `VcuStale` fault |
| 1000 ms | `VcuFreshMs` | Car-vs-Charger mode lock at `Start → Precharge` |

The mode lock is looser on purpose: a slow VCU should not be misread as an
absent one and silently lock the car into Charger mode.

#### Byte 2 bit 0 — `discharge_engaged`

The ECU's report that the bleed resistor is **connected** across the link,
either because the shutdown circuit is open or because the ECU is securing an
interrupted discharge (see `0x021`). While it is set the AMS refuses to leave
`Start`, so no contactor closes: with the SDC closed and the bleed connected,
closing an AIR would put pack current through a resistor rated for transient
duty. (The gate is on the *re-arm* only — `fsm::rearm_permitted` is evaluated
on the `Start → Precharge` edge, not continuously in `Run`.)

**Byte 2 is optional on the wire, and the compatibility rule is not
symmetric.** A VCU that predates it sends DLC 2, the bit reads 0, and the AMS
behaves exactly as before rather than blocking the car. The first time the AMS
sees DLC ≥ 3 it **latches** `ecu_discharge_capable` (never cleared — a
mid-session downgrade is not modelled).

That latch gates only the *second* of two independent refusals in
`fsm::rearm_permitted`:

| Refusal | Condition | Gated on `ecu_discharge_capable`? |
|---|---|---|
| bleed connected | `discharge_engaged` set | **No** — honoured whatever the voltage reads |
| link still charged | `dc_bus_V > DcBusDischargedV` (60 V) | **Yes**, and also requires `dc_bus_fresh` |

The asymmetry is the point. Refusing to arm over a stranded link makes sense
only if the other end can actually drain it; enforcing that against an ECU that
cannot would brick the car rather than protect it. A connected bleed, by
contrast, is a hard interlock regardless.

`Charger` mode is exempt from both: the inverter is not in the charge loop and
`dc_bus_V` is VCU-only and absent during a charge, so gating on it would make
Charger unarmable.

A blocked re-arm **holds in `Start`** rather than latching Error — the driver
waits out the discharge and presses again, no reset needed. The DASH_CHG press
*is* consumed on a blocked attempt, deliberately: carrying it forward would let
a press made while the link was live arm the car by itself seconds later, when
the discharge finally completes and nobody is expecting it.

> **Which ECU is on the bus decides whether any of this runs.** `IFS08-CE-ECU`
> `dev` sends DLC 3, so `ecu_discharge_capable` latches and both refusals are
> live. ECU `main` sends DLC 2: the bit reads 0, the latch never sets, and the
> car behaves exactly as it did before this frame grew a third byte. That
> compatibility is the point of the asymmetry above, not an accident.
>
> Neither path has been exercised on hardware. Verified in this repo only by
> `tests/unit/test_state_machine.cpp`.

### `0x101` — operator charge-mode request

| | |
|---|---|
| Direction | RX (charger-side tool → AMS) |
| DLC | ≥ 4 |
| Payload | bytes `[0..3]` must equal `43 48 52 47` = `"CHRG"`; later bytes ignored |
| Freshness | `ChargeReqFreshMs` = 1000 ms; `ChargerStaleMs` = 1000 ms for the fault |
| Cadence | re-sent ≥ 2 Hz while connected |

The magic gate exists so bus noise or a stray frame cannot flip the AMS into a
HV charge mode. Handled in `vehicle_service.cpp`.

This frame does **three** distinct jobs, and they happen at different moments:

1. **Mode lock at `Start → Precharge`.** Charger mode requires a fresh `0x101`
   **and** VCU absence (`0x100` silent beyond `VcuFreshMs`). Requiring both
   means a car with a merely dead VCU does *not* silently lock Charger — it
   locks Car and faults on `VcuStale`, which is the correct, loud outcome.
   Symmetrically, a stray `0x101` while the VCU is live cannot flip a running
   car into Charger.
2. **Precharge → Transition proceed, in Charger mode only.** There is nothing
   to voltage-gate on: `dc_bus_V` comes only from the VCU, which is absent
   during a charge. A **still-fresh** `0x101` is the "charger is connected and
   ready" signal that authorises closing AIR+. If it goes stale first, precharge
   simply holds until `PrechargeMaxMs` (5000 ms) expires → Error, rather than
   closing AIR+ into a disconnected charger.
3. **Heartbeat while charging.** Once committed to Charger mode, silence beyond
   `ChargerStaleMs` latches Error with reason 14 `ChargerStale`.

Note what Charger mode does **not** do: it closes only AIR− on entry to
Precharge, then AIR+ on the proceed. The precharge contactor sits in *parallel*
with AIR+, so closing it while the charger sources current would route the full
charge current through the transient-rated resistor. The charger voltage-matches
its output to the pack before asserting `0x101`, so there is no inrush to
absorb. **The precharge resistor never enters the charge loop.**

Full charge bring-up is one button: with `0x101` fresh and the VCU absent, the
operator presses **DASH_CHG** (PF10, momentary, edge-detected at the 10 ms
cadence) once.

### `0x103` — operator balance master switch

| | |
|---|---|
| Direction | RX (pit tool → AMS) |
| DLC | ≥ 4 |
| Cadence | re-sent ≥ 2 Hz |
| Freshness | `BalanceOverrideFreshMs` = 5000 ms |

**Three** magics, not two (`config::BalanceCmd*Magic`):

| Bytes `[0..3]` | ASCII | `BalanceCmd` | Effect |
|---|---|---|---|
| `42 41 4C 4F` | `"BALO"` | `Off` (0) | suppress balancing |
| `42 41 4C 4E` | `"BALN"` | `On` (2) | force balancing on |
| `42 41 4C 58` | `"BALX"` | `Auto` (1) | autonomous balancing |

Any other payload is ignored and leaves the previous command *and* its
freshness tick in place — bus-noise safe.

**The dead-man falls back to `Off`, not to `Auto`, and "never seen" counts as
stale.** `VehicleService::effective_balance_cmd` returns `Off` when
`last_override_tick == 0` or when the last command is older than
`BalanceOverrideFreshMs`. So on a board that has never heard a `0x103`,
balancing is **off** and `0x6C0[2]` bit 2 reads **1**. That is the intended
failure direction: a dead pit-tool link must never leave a pack bleeding
unattended.

Balancing runs in `Charge` only, so this can never touch an AIR or a safety
path, and `Error` is unaffected. Consumed by `BmsPollTask` via
`VehicleService::balance_suppressed`.

### `0x104` — operator per-module balance enable

| | |
|---|---|
| Direction | RX (pit tool → AMS) |
| DLC | ≥ 5 |
| Payload | `[0..3]` = `42 41 4C 4D` = `"BALM"`; `[4]` = 5-bit mask, bit `m` = 1 → module `m` may balance |
| Cadence | re-sent ≥ 2 Hz |
| Freshness | `BalanceModulesFreshMs` = 5000 ms |

Layers **under** `0x103`: `0x103` decides Off/On/Auto for the whole pack, and
`0x104` then narrows which modules participate. A disabled module never
discharges, but its cells still count toward the pack-wide balance floor.

**This dead-man goes the opposite way: stale or never-seen re-enables *every*
module** (`BalanceModulesDefaultMask` = `0x1F`). That is not an inconsistency
with `0x103` — it is the consequence of `0x103` already being the thing that
stops balancing on a dead link. If `0x104` also failed closed, a lost frame
would silently freeze balancing off on some modules with no indication.

Resolved by `VehicleService::effective_balance_modules_mask`, consumed by
`BmsPollTask`.

### `0x002` — reboot into bootloader

| | |
|---|---|
| Direction | RX (host → AMS) |
| DLC | **exactly 4** |
| Payload | `B0 07 AD 11` — all four bytes must match |
| Effect | `ams::Bootloader::request_reboot()`: opens all relays, writes `0xB00710AD` to `RTC->BKP0R`, `NVIC_SystemReset()`. Never returns |

The sector-0 bootloader's reset handler sees the magic, clears it (one-shot),
and stays in BL mode awaiting flash commands on the CAN peripheral it brings up
itself. Wrong bus, wrong ID, wrong DLC, or any byte differing → silently
dropped, no reboot.

The check runs **before** `VehicleService`, so the AMS still reboots on request
even if `0x002` ever collides with a future frame.

Sources: [`bootloader.hpp`](../Core/Inc/app/bootloader.hpp) (`matches_trigger`),
[`bootloader.cpp`](../Core/Src/app/bootloader.cpp) (`request_reboot`);
constants `BlBootReqCanId` / `BlBootReqPayload` / `BlBootReqDlc` in
`ams_config.hpp`.

---

## RX/TX — LOGFS log extraction (ISO-TP)

On-CAN retrieval of the SD datalogs, so a card never has to be pulled from a
mounted accumulator. Unlike everything else here this is **not** a fixed-layout
signal frame — it is a request/response protocol over **ISO-TP (ISO 15765-2)**,
which is why it has no `.def` and does not appear in `ams.dbc`.

### Addressing

| Direction | ID |
|---|---|
| Host → AMS | `0x000 + NodeID` = **`0x002`** |
| AMS → host | `0x010 + NodeID` = **`0x012`** |

`AmsNodeId` = **2** (role map: ECU 1, AMS 2, uDV 3). The board's bootloader
must be provisioned to the same node — both halves change together.

> **`0x002` is shared with the boot trigger, and that is safe by construction.**
> `matches_trigger` demands DLC **4** *and* the exact `B0 07 AD 11` payload; an
> ISO-TP frame is always DLC 8, and `0xB0` can never be a valid ISO-TP PCI byte
> (only `0x0`–`0x3` are). The trigger check runs first, so a LOGFS frame passes
> through it untouched.

ISO-TP parameters: `MaxMsg` 1024 B, timeout 1000 ms, flow control
`BS = 0` / `STmin = 0` (every consecutive frame, no further FC, no inter-frame
gap).

### Message framing

Each reassembled message is `[msg_type][opcode][payload…]`, payloads
little-endian.

Requests use **`msg_type = 0x06` (`AppCtrl`)**, *not* `0x00` (`Cmd`). Under
`Cmd` the application and the bootloader would share one opcode registry, and
`0x2x` is bootloader-adjacent there. Responses reuse `Ack 0x01` / `Nack 0x02`.

> **Consequence for host tools:** the bootloader *silently drops* `AppCtrl`. A
> LOGFS command sent while the node is sitting in the **bootloader** yields a
> **timeout, not a NACK** — indistinguishable from a dead node. Probe with a
> bootloader command (`DISCOVER 0x03` / `GET_FW_INFO 0x04` under `Cmd`), which
> the BL does answer, before reporting a cable fault.
>
> Symmetrically, the application answers a `Cmd`-typed frame with **silence**,
> not a NACK: it does not speak for the bootloader's namespace.

### Opcodes

| Opcode | Name | Args | ACK payload |
|---|---|---|---|
| `0x01` | CONNECT | — | `major:u8`, `minor:u8` (app diag proto, currently 1.0) |
| `0x02` | DISCONNECT | — | — |
| `0x21` | LIST | `cursor:u16` | `next_cursor:u16`, `count:u8`, `entry × count` |
| `0x22` | OPEN | `index:u16` | `handle:u16`, `size:u32`, `crc32:u32` |
| `0x23` | READ | `handle:u16`, `offset:u32`, `len:u16` | `data` (≤ 512 B) |
| `0x24` | CRC | `handle:u16` | `crc32:u32` |
| `0x25` | CLOSE | `handle:u16` | — |
| `0x27` | FINALIZE | — | `index:u16` |

`entry` is a fixed **22 bytes** — `{index:u16, size:u32, mtime:u32, name[12]}` —
so a listing parses by stride alone. `mtime` is the packed FAT stamp
(`fdate << 16 | ftime`). `next_cursor == 0xFFFF` ends the listing. At most
**46** entries fit one reply.

Three behaviours a host must get right:

- **On OPEN, `crc32 == 0` means "not available — use `CRC`"**, not a literal
  zero CRC. Every file this firmware seals gets a `.CRC` sidecar, but cards
  written before sidecars existed have none, and OPEN must never block
  streaming 4 MiB to compute one.
- **A short READ means end-of-file**, and it is an ACK, not an error — that is
  the host's normal termination signal. An oversized `len` is clamped to 512
  rather than rejected.
- **`FINALIZE` seals the *active* log** — flush, close, rename to `.CSV`, write
  the CRC sidecar — and returns the sealed index, so the run that just happened
  is immediately listable rather than waiting on rotation. It NACKs
  `FILE_NOT_FOUND` when there is nothing to seal (no active file, or no rows
  written), so an eager operator cannot fill the card with header-only files.
  Any open read handle is released, since the file set changed.

**v1 is read-only.** `0x26` (DELETE) is deliberately unimplemented and NACKs as
unsupported: an extraction tool that can also erase logs is the wrong tool to
hand a pit crew.

### NACK codes

Values ≤ `0x10` are the bootloader's, reused verbatim so the vocabulary stays
coherent across both images.

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
| `0x15` | VEHICLE_STATE — refused: the car must be stopped with the TS off |
| `0xFE` | UNSUPPORTED |

### Vehicle-state gate

**Log extraction is permitted only in `Start` and `Error`** — the two states
with the contactors open and the tractive system down, which is the property
the rule is actually about. Any LOGFS opcode in `Precharge` / `Transition` /
`Run` / `Charge` is refused with `NACK VEHICLE_STATE (0x15)`, and any open read
handle is released.

Beyond the obvious — a pull is a multi-minute, bus-heavy operation and nobody
should be inviting it while the car can move — there is a concrete mechanism.
The diag TX id `0x012` is numerically **lower** than the VCU heartbeat `0x100`,
so every queued LOGFS frame **wins arbitration** against the heartbeat the
safety FSM depends on. `VcuStaleMs` is 200 ms and latches Error, which opens the
contactors. A single reply burst is only ~17–33 ms so the margin is wide, but
the failure mode is *"a log pull opens the AIRs"*, and that is not worth
carrying when the alternative is simply not extracting while the car is live.

`Error` is permitted deliberately, not grudgingly: "grab the log from the run
that just faulted" is the case the feature exists for, and a faulted car sits in
`Error`. With `ErrorLatch` sticky across resets, a power-cycled post-fault car
boots straight back into it. It is also the one state where the arbitration
hazard cannot bite — `VcuStale` latching Error is exactly what the gate protects
against, and in `Error` that has already happened.

The gate is checked **after** the session check, so a connected host gets a
specific reason instead of a generic refusal, and **before** anything reaches
the card. `CONNECT` / `DISCONNECT` remain available in **any** state: they cost
nothing on the bus and let a host discover *why* it is being refused rather than
facing a silent node.

### Session

`CONNECT` opens a session; everything except CONNECT/DISCONNECT is refused with
`BAD_SESSION` without one, so a stray frame cannot stream the card to whoever is
on the bus. Idle timeout is **10 s**, after which any open handle is released —
an operator who unplugs mid-pull does not pin it. A CONNECT from a second host
takes over, releasing the previous handle: a pit tool that cannot connect
because a dead session is parked is worse than one that interrupts a session
nobody is driving.

### Files visible

Only sealed `LOGnnnn.CSV`. The active `LOGnnnn.TMP` is excluded because its
length would be stale before a host finished reading it, and `LOGnnnn.CRC`
sidecars are an implementation detail. The sidecar holds the CRC-32 accumulated
as rows were written, so `CRC` answers without re-reading the file; absent one,
it falls back to streaming.

CRC-32 is **ISO-HDLC / zlib** (poly `0xEDB88320` reflected, init and final XOR
`0xFFFFFFFF`) — i.e. Python's `zlib.crc32`. Deliberately **not** the STM32
hardware CRC default (CRC-32/MPEG-2), which is a different checksum.

### Throughput — plan for it

ISO-TP carries 7 of every 8 bytes, so a 512-byte READ is ~76 frames ≈ 19 ms at
500 kbit/s → **~27 KB/s on an idle bus**. With `LogFileMaxBytes` at 4 MiB that
is **~2.5–3 min per file**, and the bus is not idle. Larger reads buy ~5 %;
per-frame overhead dominates. Hosts should show progress, let operators pick
files from `LIST` sizes, and resume via the offset-based `READ`.

The reply pump deliberately leaves `DiagTxReservedSlots` (6) of the 16-deep TX
FIFO free rather than filling it, so the flight telemetry matrix — which ships
non-blocking later in the same loop pass — always finds room. Filling to zero
blacks out telemetry for the whole multi-minute pull.

Sources: [`isotp.hpp`](../Core/Inc/app/isotp.hpp),
[`diag_proto.hpp`](../Core/Inc/app/diag_proto.hpp),
[`diag_dispatch.hpp`](../Core/Inc/app/diag_dispatch.hpp),
[`logfs_server.hpp`](../Core/Inc/app/logfs_server.hpp),
[`crc32.hpp`](../Core/Inc/app/crc32.hpp); FatFs backend in
[`sd_logger_task.cpp`](../Core/Src/app/sd_logger_task.cpp), CAN route in
[`acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp).

---

## Retired IDs

Nothing in the current build emits or consumes any of these. Listed only so
someone spelunking an old trace can find the cross-reference.

| ID(s) | Was | Replaced by |
|---|---|---|
| `0x20` (extended) | AMS state reply, 5-value state byte | `0x020` standard `ok_precharge`; full state on `0x4A0[0]` |
| `0x12C + 0x1E·m`, `0x140 + 0x1E·m` and their response ranges | BMS slave polling over the second CAN bus | LTC6811-1 isoSPI chain — see [`BMS_LTC6811.md`](BMS_LTC6811.md) |
| `0x401 – 0x406` | accumulator temperature RX | isoSPI temperature sweep |
| `0x40D – 0x412` | charger-mode temperature passthrough (extended) | nothing; there is no second bus to forward from |
| `0x450` | unsigned 2-byte current | `0x135` (signed deciamps + DCDC in one frame) |
| `0x600` | start button | TSMS (PF9, level) + DASH_CHG (PF10, momentary edge) |
| `0x18FF50E7` | charger detect | `0x101` magic-gated charge request + VCU absence |

If the legacy module base really was `0x12C` (see the caveat below), then that
ID has been **reused**: it is now the pack-wide minimum cell voltage TX frame,
and an old trace and a new one mean different things by it.

The legacy source files those frames were parsed in (`class_bms.cpp`,
`class_cpu.cpp`, `class_temperatures.cpp`, `module_state_machine.cpp`) are not
in this repository, so their exact byte layouts cannot be verified from this
tree. Earlier revisions of this page reproduced them from memory; those tables
have been removed rather than left as unverifiable claims.

---

## Adding or changing a message

1. Write or edit the `.def`. That is the whole layout. The compile-time
   bit-overlap `static_assert` in pass 5 of `can_codecs.hpp` catches a
   copy-pasted byte index; the per-field width `static_assert` in pass 1
   catches `FIELD_LE(x, uint8_t, 0, 16, …)`.
2. Add it to `all_messages.inc` if it is new.
3. Keep the struct holding **raw wire integers**. Any scaling, clipping,
   saturation or bit assembly belongs in the adapter
   (`telemetry_encoders.hpp`, `acu_tx_encoders.hpp`, `pit_diag_emitter.hpp`),
   never in the `.def`.
4. Add an ID and a cadence constant to `ams_config.hpp`, and put the send in
   the right cadence group in `acu_can_task.cpp`. **The `.def` period must
   match the group you actually send in** — it becomes `GenMsgCycleTime` and
   other teams size receive timeouts from it.
5. Regenerate the DBC (command at the top). CI diffs it.
6. Update this page.

One trap worth knowing: `send_acu` sets `tx.DataLength = dlc` as a **raw byte
count**. The STM32H7 HAL applies the register shift internally and its
`FDCAN_DLC_BYTES_N` macros are unshifted — unlike STM32G4's HAL, which
pre-shifts them. Pre-shifting here puts DLC = 0 on the wire for every frame,
which is easy to import from G4 code and hard to spot.

---

## Known gaps

- **The discharge interlock has never run end to end.** Both halves exist —
  this repo's and `IFS08-CE-ECU` `dev`'s — but each is verified only by its own
  host tests and no bench has had both boards on it. Against ECU `main`, which
  still sends `0x100` at DLC 2, the interlock is inert by design.
- **`0x680`/`0x6A0` grids have no `.def`.** Their layout lives in
  `can_codecs.hpp` `ALL_ARRAYS` because the DSL has no multiplexed-array
  representation. They do reach the DBC, expanded frame-by-frame.
- **`0x6D0`/`0x6E8` ADOW grids have no `.def` and no DBC row** and are compiled
  out of flight builds.
- **`0x137` `tmax_dcdc` is a fixed stub** (`-32768`); no DCDC temperature
  sensor is wired.
- **`0x4A3`, `0x500`, `0x501`, `0x502` are declared IDs with no producer.**
- **The generated DBC has no `VAL_` tables**, so `fsm_state`, `mode_locked`,
  `fault_reason`, `reset_cause` and `last_fault` decode as bare integers in any
  DBC-driven tool. The mappings are in this document and in the headers only.
- **`0x4A2[3..4]` `dc_bus_V` carries no freshness bit** and freezes at its last
  received value when the VCU goes silent.
- **`0x4A4` reports commanded coil state, not contactor position.** There is no
  auxiliary-contact feedback anywhere in this design, so no CAN frame can
  distinguish a welded contactor from a healthy one.
- **Fault reasons 6 and 7 (cell under/over temperature) are unreachable**
  while `config::TempFaultsTrusted` is `false`.
- **`ams_config.hpp`'s comment on `AcuTxSocId` still calls `0x130` deferred**,
  and the header comment in `acu_can_task.cpp` says the same. Both are stale —
  `tx_soc()` is scheduled in the 250 ms group. The scheduler, not the comment,
  is what runs.
- **`telemetry_encoders.hpp` labels `encode_relay_status` as `0x4A3`**; it is
  transmitted on `0x4A4` (`AmsRelayStatusId`). A comment-only error, but it
  will mislead anyone grepping for an ID.
