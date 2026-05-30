# Glossary

Every acronym and domain term used across the AMS docs and source, in one
place. Formula Student EV and STM32 firmware both come with a lot of
jargon — keep this open while you read the rest.

> New here? Start with [`ONBOARDING.md`](ONBOARDING.md). The as-built
> reference is [`ARCHITECTURE.md`](ARCHITECTURE.md).

---

## The system

| Term | Meaning |
|---|---|
| **AMS** | Accumulator Management System — *this* firmware. Owns pack safety: monitors cells, drives the contactors, kills the pack on any fault. |
| **Accumulator** | The Formula Student term for the high-voltage traction battery pack (~350 V here). |
| **Pack** | The accumulator. 95 series cells across 5 **BMS_LITE** modules. |
| **BMS** | Battery Management System — the cell-monitoring layer. On this car the BMS function is the LTC6811 chain read by the AMS over isoSPI (not a separate CAN device since v1.2.0). |
| **BMS_LITE** | The cell-monitoring daughterboard (one per module): 19 cells + 40 NTCs, an LTC6811 pair and ADG731 mux per module. |
| **VCU** | Vehicle Control Unit — the car's main controller. Sends the `0x100` DC-bus heartbeat; its presence is what tells the AMS it's in the **Car** context. |
| **ACU** | Accumulator Container Unit — the AMS's CAN-facing role on the "accumulator bus" (FDCAN1). You'll see `Acu*` in task/encoder names. |
| **Charger** | The external HV charging station. Has **no CAN comms with the AMS**; it auto-emits the operator `0x101` "CHRG" frame and soft-starts its own output. |

## Operator inputs & the shutdown circuit

| Term | Meaning |
|---|---|
| **SDC** | Shutdown Circuit — the safety loop that, when broken, opens the contactors. The AMS is a *participant* via `AMS_OK`, not a sensor of it. |
| **AMS_OK** | The AMS's leg of the SDC (GPIO **PB4**, active-high). HIGH = "AMS healthy, not blocking the SDC"; driven LOW during boot grace and the instant a fault latches. Driven every 10 ms (#299/#301). |
| **TSMS** | Tractive System Master Switch (GPIO **PF9**) — a **held** master switch. Gates `Start → Precharge`; sustains `Run`/`Charge`; its drop ends them. |
| **DASH_CHG** | Dashboard / charge button (GPIO **PF10**) — a **momentary press**, edge-detected. One press drives `Start → Precharge` (with TSMS). Not level-checked in `Run`/`Charge`. |
| **AIR** | Accumulator Isolation Relay — the main HV contactors. **AIR−** (PB6) and **AIR+** (PB5) connect the pack to the bus. |
| **Precharge** | Bringing the downstream bus capacitors up to pack voltage *through a resistor* before closing AIR+, to avoid a damaging inrush current. The **precharge relay** is on PB7. |
| **Contactor** | A high-current relay. The three here: AIR−, AIR+, precharge. |

## The state machine

| Term | Meaning |
|---|---|
| **FSM** | Finite State Machine — the pure-logic safety state machine in [`state_machine.hpp`](../Core/Inc/app/state_machine.hpp). States: Start, Precharge, Transition, Run, Charge, Error. |
| **Mode** | The Car-vs-Charger context, locked once at `Start → Precharge` and never re-evaluated. Values: Undecided, Car, Charger. |
| **Boot grace** | A window (`SafetyBootGraceMs`, 2 s) after kernel start during which data-presence/freshness predicates are suppressed, so the chip doesn't fault before the first sensor data arrives. |
| **ErrorLatch** | A sticky error flag stored in an RTC backup register (`0xA115EE51`) that survives reset — so a faulted boot comes back up in `Error` until backup-domain power is cycled (or the HIL clear-flag wipes it). |
| **Predicate** | A single fault check (cell under-voltage, BMS stale, current over-limit, …) in [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp). Any one tripping latches `Error`. |
| **Debounce** | Requiring a fault condition to persist (`CellFaultConfirmTicks`, ~300 ms) before latching — so a single torn snapshot read can't cause a spurious `Error` (#296/#279). |

## BMS / isoSPI hardware

| Term | Meaning |
|---|---|
| **LTC6811(-1)** | Analog Devices battery-stack monitor IC. Measures up to 12 cells + aux inputs; daisy-chains over isoSPI. Two per module here. |
| **LTC6820** | isoSPI ↔ SPI bridge. The STM32's SPI1 master talks to it; it drives the isolated isoSPI chain. |
| **isoSPI** | Isolated SPI — a differential, transformer-isolated serial link the LTC parts use to daisy-chain safely across the HV pack. |
| **ADG731** | 32-channel analog mux. Two per module select which of the 40 NTC thermistors the LTC's aux ADC reads during the temperature sweep. |
| **NTC** | Negative-Temperature-Coefficient thermistor — the cell temperature sensors (200 total, 40 per module). |
| **PEC / PEC15** | Packet Error Code — the 15-bit CRC the LTC6811 appends to every register group. A bad PEC means a corrupt read; the module's freshness is not advanced. |
| **DCC** | Discharge Cell Control — the LTC6811 config bits that switch a cell's onboard balancing FET to bleed off charge. |
| **Balancing** | Bleeding higher cells down (via DCC) so the pack stays matched. Policy lives in `balance_controller.hpp`. |
| **ADCV / RDCV / ADAX / RDAUXA / WRCFGA** | LTC6811 commands: start cell-voltage conversion / read cell voltages / start aux (temp) conversion / read aux / write config (balancing). See [`BMS_LTC6811.md`](BMS_LTC6811.md). |

## CAN & tooling

| Term | Meaning |
|---|---|
| **FDCAN1 / FDCAN2** | The STM32H7 CAN-FD peripherals. **FDCAN1** is the live accumulator bus (VCU, telemetry, boot trigger). **FDCAN2** is claimed by the bootloader only, post-reset. |
| **`0x100`** | VCU DC-bus voltage heartbeat. Its freshness decides Car vs Charger at the mode lock. |
| **`0x101`** | Operator charge-mode request ("CHRG" magic `43 48 52 47`). Selects Charger mode and gates the charger precharge proceed. |
| **`0x4A0/0x4A1/0x4A2`** | The AMS telemetry frames (status / pack / temps + cockpit byte), emitted every 500 ms. |
| **Pit-diag** | The runtime-toggleable diagnostic CAN stream (`0x680..0x6C8`, 58 frames) carrying every cell, temp, FSM/fault detail, and per-IC PEC count for bench debugging. |
| **DBC** | A CAN database file (`docs/dbc/ams.dbc`) describing every frame's signal layout, generated by `tools/gen_dbc.py`. Consumed by the pit-debug host tool. |
| **fault_reason** | A byte on pit-diag `0x6C0[6]` naming which predicate latched `Error` (enum 0..11; 12 = FsmError for FSM-driven errors like precharge timeout / TSMS drop). #276. |

## Build, test & bench

| Term | Meaning |
|---|---|
| **CubeMX** | ST's code generator. Owns `AMS.ioc` and the generated HAL/RTOS C. Our C++ lives in `Core/{Inc,Src}/app/` and is called via `extern "C"` trampolines. |
| **HAL** | Hardware Abstraction Layer — ST's peripheral driver library. |
| **IWDG** | Independent Watchdog — resets the chip (~100 ms) if `MainTask` stops refreshing it, defaulting the relays open. |
| **Unity** | The C unit-test framework used for the 182 host tests (`tests/unit/`). |
| **SIL** | Software-in-the-Loop — the multi-step pure-FSM scenario tests (`test_sil_scenarios.cpp`) that drive the FSM through realistic input sequences. |
| **HIL** | Hardware-in-the-Loop — the real-hardware bench rig (MLC2 carrier + Pi Pico LTC emulator + CAN/GPIO fixtures). Acceptance plan: [issue #317](https://github.com/isc-fs/IFS08-CE-AMS/issues/317). |
| **MLC2** | The bench carrier board the AMS chip sits on for HIL testing (node ID `0x01`). |
| **`AMS_HIL_CLEAR_ERROR_LATCH`** | A bench-only CMake flag that wipes the sticky ErrorLatch on boot. Never in a flight build. See [`HIL_BUILD.md`](HIL_BUILD.md). |
| **`COMMISSION`** | A tag on `ams_config.hpp` constants that must be calibrated on real hardware before flight. See [`COMMISSIONING.md`](COMMISSIONING.md). |
| **Bootloader** | The CAN bootloader ([isc-fs/stm32-can-bootloader](https://github.com/isc-fs/stm32-can-bootloader)) in flash sector 0; the app lives in sectors 1–6 and jumps to/from the BL via a backup-register handshake. |
