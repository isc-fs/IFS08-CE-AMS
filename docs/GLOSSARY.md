# Glossary

Every acronym and domain term used across the AMS docs and source, in one
place. Formula Student EV and STM32 firmware both come with a lot of
jargon — keep this open while you read the rest.

Definitions here describe how a term is used **in this codebase**, which is
not always the textbook meaning. Where the two differ, the local meaning is
what the code does, and the entry says so.

> New here? Start with [`ONBOARDING.md`](ONBOARDING.md). The as-built
> reference is [`ARCHITECTURE.md`](ARCHITECTURE.md); the gate-by-gate FSM
> walkthrough is [`FSM_OVERVIEW.md`](FSM_OVERVIEW.md).

---

## The system

| Term | Meaning |
|---|---|
| **AMS** | Accumulator Management System — *this* firmware. Owns pack safety: monitors cells, drives the contactors, and opens the shutdown circuit on any fault. |
| **Accumulator** | The Formula Student term for the high-voltage traction battery pack. |
| **Pack** | The accumulator. **95S6P**: 95 series elements, each element six cells in parallel (`CellsInParallel = 6`). At the configured cell window (`CellUnderVoltageMv` 2800 → `CellOverVoltageMv` 4200) that is roughly 266 V empty to 399 V full. Nameplate capacity `PackCapacityMah = 18000` (`COMMISSION`). |
| **Cell** | Throughout the firmware, "cell" means one *series element* — the 6P group the LTC actually measures across. This matters in `soc_estimator.hpp`: the internal resistance in the measurement model is a cell's divided by six. |
| **BMS** | Battery Management System — the cell-monitoring layer. Here the BMS function is an LTC6811-1 isoSPI chain read directly by the AMS, not a separate CAN device. |
| **BMS_LITE** | The cell-monitoring daughterboard, one per module: 19 cells, 40 NTC channels, two LTC6811s, two ADG731 muxes, and the per-cell bleed hardware. Five of them make the pack. |
| **Module** | One BMS_LITE and its 19 series elements. `BmsModuleCount = 5`, `CellsPerModule = 19`, `TempsPerModule = 40`. Modules are indexed 0..4 and `AllModulesMask = 0x1F` means all five reporting. |
| **VCU** | Vehicle Control Unit — the car's main controller. Publishes the `0x100` DC-bus heartbeat. Its *freshness at the moment of `Start → Precharge`* is what selects Car vs Charger mode. |
| **ECU** | Used in the source as the far end of the accumulator bus: the node that consumes the AMS TX matrix (`0x020`, `0x021`, `0x12C`, `0x131`–`0x137`) and that owns the DC-link discharge decision. In practice **ECU and VCU name the same physical node** in different comments — `rx_vcu_dc_bus.def` attributes `0x100` to the "VCU", while `acu_discharge_interlock.def` calls the consumer of `0x021` the "ECU". Do not read a two-node architecture into the split. |
| **ACU** | Accumulator Container Unit — the AMS's CAN-facing role on the accumulator bus. You see it as the `Acu*` prefix on task, config and encoder names (`AcuCanTask`, `AcuTx*Id`, `acu_tx_encoders.hpp`). |
| **Charger** | The external HV charging station. It has **no CAN link to the AMS at all** — no handshake, no telemetry. It voltage-matches its output to the pack and soft-starts on its own. Everything the AMS knows about "we are on a charger" comes from `0x101`, which the *operator tool* sends (below), not the charger. |
| **WarioCharger / ChargerDisplayWario** | The pit-side host that sits with the charger and speaks CAN to the AMS. It is the sender of `0x101` (charge-mode request, ≥2 Hz), `0x103` (balancing master switch) and `0x104` (per-module balancing enable). The CAN DSL labels it `Pit_Tool`. |
| **MingoCAN** | The bench CAN tool. Consumes the generated DBC and drives the bootloader flash flow. |

## The shutdown circuit, contactors, and the DC link

| Term | Meaning |
|---|---|
| **TS** | Tractive System — everything downstream of the AIRs: DC link, inverter, motor. |
| **SDC** | Shutdown Circuit — the series safety loop that, when broken anywhere, de-energises the AIR coils. The AMS is a **participant** in the loop, not a sensor of it: there is no dedicated SDC-feedback input on the board (see the note opening `safety::evaluate_fault_detail`). |
| **AMS_OK** | The AMS's leg of the SDC (**PB4**, active-high). HIGH = "AMS healthy, not blocking the SDC". `MainTask` drives it every 10 ms from `safety::ams_ok_asserted`: HIGH only once the boot grace has passed **and** no `Error` is latched; LOW during grace and the instant a fault latches. **Dropping it is not reversible from firmware** — the AMS's leg of the loop latches in *hardware*, so re-asserting PB4 does not re-close the SDC (`acu_discharge_interlock.def`). Treat AMS_OK as a one-way health output, never as a temporary interlock. |
| **TSMS** | Tractive System Master Switch (**PF9**, held level). Nominally the master switch, but the firmware reads it as **"the shutdown circuit is complete"** — any open element in the loop pulls PF9 low, so `tsms = 1` also means the discharge relay is energised and the bleed is disconnected (`safety_task.cpp`, `g_tsms_telemetry`). That second reading is what makes it publishable on `0x021`. It gates `Start → Precharge` and sustains `Run`/`Charge`. |
| **DASH_CHG** | Dashboard / charge button (**PF10**, momentary press). `MainTask` polls it at 10 ms and latches a **rising edge** until the 20 ms FSM step consumes it, so a press between steps is never lost. One press drives `Start → Precharge` (with TSMS held). It is deliberately **not** level-checked in `Run`/`Charge` — a momentary button sits low most of the time, so checking its level would fault instantly. |
| **AIR** | Accumulator Isolation Relay — the main HV contactors. **AIR−** (PB6) and **AIR+** (PB5) connect the pack to the DC link. |
| **Contactor** | A high-current relay. Three of them, all on GPIOB and all active-high: AIR− PB6, AIR+ PB5, precharge PB7. `Relays::open_all()` writes all three in one BSRR transaction, which is only valid because they share a port — see the shared-port invariant note on `Relays::open_all` in `relay_driver.cpp`. |
| **Precharge** | Raising the DC-link capacitors to pack voltage *through a resistor* before closing AIR+, so AIR+ never sees the inrush. The precharge contactor sits **in parallel with AIR+**. Bounded by `PrechargeMaxMs` (5 s, `COMMISSION`) — the resistor is rated for transient duty only, so *any* stuck precharge must end in `Error`, not in a held-closed resistor. |
| **Precharge target** | Car mode's completion test: DC bus ≥ 95 % of measured pack voltage **and** the VCU heartbeat still fresh. The freshness half is not decoration — see `precharge_target_reached` below. |
| **DC link** | The capacitor bank downstream of the AIRs (inverter side). "The link" in comments. Only the VCU measures it; the AMS sees it solely through `0x100`. |
| **Bleed resistor** | The resistor that drains the DC link when the tractive system is shut down. Distinct from the *precharge* resistor. |
| **Discharge relay** | The relay that connects the bleed resistor across the DC link. **Normally closed and wired into the SDC**, with no software control from the AMS. Opening the SDC de-energises it, so the bleed connects and the link falls; closing the SDC re-energises it and the bleed disconnects. |
| **Stranded link** | The failure this whole vocabulary exists for: the SDC is opened and closed again *before* the link has drained, so the discharge stops part-way and nothing drains the remainder while the SDC stays closed. The AMS cannot fix it (no software control of the discharge relay, and AMS_OK cannot be pulsed). See below. |
| **Discharge interlock** | The AMS↔ECU protocol around a stranded link. Detailed below. |
| **Re-arm** | Attempting `Start → Precharge` again after falling back to `Start`. Gated by `fsm::rearm_permitted`. |
| **IMD** | Insulation Monitoring Device — the FS-mandated isolation monitor. **This firmware does not read, monitor or telemeter the IMD.** It appears in the source exactly once, as an example of a fault that leaves cell data intact (a comment in `balance_controller.hpp`). The IMD is a separate SDC participant; if you are looking for IMD handling here, there is none. |
| **R2D** | Ready-to-Drive. The AMS's only contribution is `0x020 ACU_ok_precharge` (1 byte, set iff the FSM is in `Run` or `Charge`). A dropped `0x020` means no R2D — which fails safe. |

### The DC-link discharge interlock, in full

Worth reading once, because no single file tells the whole story.

Opening the shutdown circuit de-energises the normally-closed discharge relay,
the bleed resistor connects, and the DC link starts falling. If the SDC is
closed again before the link has drained, the relay re-energises, the bleed
disconnects, and **the discharge stops part-way**. What is left on the link is
then unpredictable from the AMS's side — it is not a function of how long ago
the SDC was cycled.

The AMS cannot restart that discharge. The relay has no software control, and
the AMS's own leg of the loop (`AMS_OK`) latches in hardware, so it cannot be
driven low temporarily to re-open the SDC. The ECU can, through a
normally-closed relay in series with the discharge-relay coil — but the ECU
cannot see either fact it needs to decide. Hence the protocol:

- **AMS publishes `0x021 ACU_discharge_interlock`** at 100 ms
  (`AcuCanTask`'s 100 ms TX slot): `fsm_in_start` (bit 0 — all three contactors are
  commanded open, so any voltage on the link is *left over*, not something the
  AMS is putting there) and `tsms` (bit 1 — the SDC is complete, so the bleed
  is disconnected). Both are raw observations, not a request: the ECU owns the
  decision because it owns the DC-bus measurement that decides it.
- **ECU reports back on `0x100` byte 2 bit 0** as `discharge_engaged`
  (`VehicleService::update_from_frame`) — "the bleed resistor is connected across the
  link right now".
- **AMS gates re-arm** in `fsm::rearm_permitted` (`state_machine.hpp`):
  `discharge_engaged` blocks unconditionally (closing a contactor onto a
  connected transient-duty resistor puts pack current through it), and a link
  still above `DcBusDischargedV` (60 V, `COMMISSION`) blocks because a
  precharge would be a no-op. Charger mode is exempt — the inverter is not in
  the charge loop and `dc_bus_V` is VCU-only.
- **A blocked press is consumed, not carried.** The FSM returns to `Start`, and
  `MainTask` clears `dash_chg_edge_pending` unconditionally after every step —
  so the press is spent. That is deliberate: carrying it would let a press made
  while the link was live arm the car by itself seconds later, when the
  discharge finally completes and nobody is expecting it.

**`ecu_discharge_capable`** is the compatibility latch. Byte 2 of `0x100` is
optional on the wire; an ECU that does not implement it sends DLC 2, the bit
reads 0, and the AMS arms exactly as if there were no interlock. The first
`0x100` with DLC ≥ 3
latches `ecu_discharge_capable` true, and only then does the AMS enforce its
own voltage-based block — refusing to arm over a link the other end has no way
to drain would brick the car rather than protect it.

**Status, honestly:** the AMS half is complete and unit-tested. The ECU half
lives in a different repository and is **not** verifiable from here. In this
repo nothing ever sets `discharge_engaged`, so until an ECU ships that sends
DLC ≥ 3 on `0x100`, `ecu_discharge_capable` stays false and `rearm_permitted`
always returns true — the interlock is present but inert. Do not treat it as
live protection until you have watched byte 2 on a real bus.

## The state machine

| Term | Meaning |
|---|---|
| **FSM** | Finite State Machine — the pure-logic safety state machine in [`state_machine.hpp`](../Core/Inc/app/state_machine.hpp). No HAL, no FreeRTOS, so the host tests drive it directly. States: `Start`, `Precharge`, `Transition`, `Run`, `Charge`, `Error`. |
| **Start** | Idle. All three contactors commanded open. The only state that can arm. |
| **Transition** | A one-step passthrough between `Precharge` and `Run`/`Charge`. It has no hold timer: the contactor swap (`CloseAirP` plus `OpenPrecharge`) was already emitted on the entering edge, and this step commits. In Car mode it re-checks the precharge target so a failed swap (bus slumps the moment the precharge contactor opens) lands in `Error` rather than energising the TS on a degraded bus. |
| **Mode** | The Car-vs-Charger context. Values `Undecided`, `Car`, `Charger`. Locked by `MainTask` at the exact iteration that leaves `Start`, never re-evaluated — and **cleared on any return to `Start`**, so a re-arm re-locks and re-precharges from scratch. Charger requires *both* a fresh magic-gated `0x101` **and** VCU absence, so a car with a dead VCU locks Car and faults on `VcuStale` rather than silently charging. |
| **Boot grace** | `SafetyBootGraceMs` (2000 ms) after kernel start, during which the *data-presence / freshness* predicates are suppressed. Without it, every service's `last_*_tick` is 0 at t=0, the first evaluation faults, the watchdog refresh is withheld, and the IWDG resets the chip in ~100 ms before `BmsPollTask` has polled once. Immediate-safety predicates (force-error) stay armed throughout. `AMS_OK` is also held LOW for the whole grace. |
| **ErrorLatch** | A sticky error flag in RTC backup register 1, magic `0xA115EE51`, that survives reset — so a faulted boot comes back up in `Error` until the backup domain is power-cycled (or the bench build's clear flag wipes it). |
| **Predicate** | One fault check in [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp), evaluated as an ordered chain returning the **first** matching `FaultReason`. Most latch `Error` on the first tick; the cell voltage/temperature range checks are debounced, and cell-temperature faults are suppressed entirely while `TempFaultsTrusted` is false. |
| **Debounce** | Requiring a condition to persist before latching. Two independent debouncers, both driven every 10 ms tick and both self-gating on the reason: `CellFaultConfirmTicks` = 25 (≈250 ms) for the cell V/T range checks, and `BmsStaleConfirmTicks` = 25 (≈250 ms) for `BmsStale`. Both are sized to span **more than one** 200 ms voltage poll, so a single anomalous poll can never latch, while a real condition still clears the < 500 ms fault-response budget (200 ms poll + 250 ms confirm + 10 ms tick ≈ 460 ms). |
| **`precharge_target_reached`** | The Car-mode precharge-complete test — and the reason `dc_bus_fresh` is a **required** argument, not advisory. `VehicleState` holds the *last received* `dc_bus_V`, so a dead VCU does not zero it, it **freezes** it. Frozen at pack voltage it satisfies the 95 % criterion forever, including after the link has actually bled to zero — closing AIR+ onto full pack voltage with nothing to limit the inrush. `VcuStale` cannot save you here: it needs 200 ms while the FSM steps every 20 ms, so the transition fires ~10 steps before the fault can open the AIRs. Freshness has to be *part of the criterion*, not a separate fault racing it. |
| **Bus collapse** | The signature of the AIRs being opened externally (a cockpit SDC shutdown the AMS cannot sense): the VCU keeps reporting `dc_bus_V`, but it has fallen below `BusCollapsePercent` (50 %) of pack. Debounced over `BusCollapseConfirmTicks` (20 ≈ 200 ms) in `MainTask`, Car-mode `Run` only. Confirmed, the FSM falls back to `Start` **without latching**, so a re-arm re-runs precharge instead of reclosing AIR+ onto a discharged link. |
| **Non-latching de-energise** | Falling back to `Start` with all contactors open and **no** `Error`. Used for a TSMS drop and a confirmed bus collapse. Load-bearing for the FS rule that the driver must be able to stop and restart the TS from the cockpit unaided: if a TSMS drop latched `Error` it would drop `AMS_OK`, opening the AMS's *upstream* leg of the loop, and re-closing TSMS could no longer restore it. **Charger mode is the exception** — scrutineering forbids re-activating the charge output once the SDC has opened, so a TSMS drop there latches `Error` (`ChargerTsmsOpen`) and needs a full reset. |
| **Dead-man** | The freshness fallback pattern on operator commands: a command that goes stale (or was never seen) resolves to its *safe* value rather than sticking. `0x103` balancing → `Off`; `0x104` module mask → all-enabled. Resolved in `VehicleService::effective_balance_*`, never in the consumer. |

## BMS / isoSPI hardware

| Term | Meaning |
|---|---|
| **LTC6811(-1)** | Analog Devices battery-stack monitor. Measures up to 12 cells plus auxiliary inputs and daisy-chains over isoSPI. Two per module, so `LtcChainLength = 10`. |
| **LTC cell split** | **Not uniform, and the single easiest thing to get wrong.** `LTC_1` (even chain index, first in chain) carries **9** cells → module cells **0..8**; `LTC_2` (odd index) carries **10** → module cells **9..18**. `CellsPerLtcUpper = 9`, `CellsPerLtcLower = 10`. The upper IC's `RDCVD` group is **unused** — decoding it would false-flag conductors that do not exist (see the POLL-INTEGRATION CONTRACT block in `open_wire.hpp`). |
| **LTC6820** | isoSPI ↔ SPI bridge. SPI1 master (PA4–PA7, Mode 3) talks to it; it drives the isolated chain. Also owns the chain wakeup pulse train. |
| **isoSPI** | Isolated SPI — a differential, transformer-coupled serial link the LTC parts use to daisy-chain safely across the HV pack. |
| **T_SLEEP** | The LTC6811 drops into sleep ~2 s after its last valid command, and a sleeping IC ignores everything but the CS wake-pulse train. Sleeping also **resets CFGR to defaults**, which re-enables the GPIO pull-downs that would short the ADG731 / NTC divider — so recovery must re-wake *and* re-write config, not just re-wake (`bms_poll_task.cpp`, `recover_chain`). |
| **PEC / PEC15** | Packet Error Code — the 15-bit CRC the LTC6811 appends to every register group. A bad PEC means a corrupt read; that IC's data is discarded and the module's freshness is **not** advanced, so a persistently-failing IC eventually shows up as `BmsModuleOffline` / `BmsStale`. Per-IC counters are published on pit-diag `0x6C7`/`0x6C8`. |
| **ADG731** | 32-channel analog mux, **two per module** (one per LTC), of which 20 channels each are populated — `NTC_1..10` on S1..S10 and `NTC_11..20` on S17..S26 (`Adg731ChannelMap`). The mux needs a throwaway first select (to unpopulated S32) to absorb the first-select drop, or temp slot 0 reads open. |
| **NTC** | Negative-Temperature-Coefficient thermistor — the cell temperature sensors. 40 per module, 200 total, stored row-major in `cell_tempC[5][40]` with slots 0..19 from the upper LTC and 20..39 from the lower. |
| **`TempFaultsTrusted`** | Currently **false**, and it is the single biggest honest gap in the pack-monitoring story. The NTC path runs through the ADG731 mux, which is not validated end-to-end in the flight path, so while the flag is false the `CellUnderTemp`/`CellOverTemp` predicates are **suppressed** and `balance::compute_mask` refuses to balance at all (its only thermal protection reads that same data). Cell *voltage* protection is unaffected. Note the separate `TempSensorDisconnected` predicate *is* armed — an open NTC reads the rail regardless of calibration, so presence is checkable even when accuracy is not. |
| **DCC** | Discharge Cell Control — the LTC6811 config bits that command a cell to bleed. **On BMS_LITE the DCC bit does not switch the LTC's own S-pin FET**: it gates an **external TSM2323 PMOS** driving `R71 ‖ R72 = 47 Ω ‖ 47 Ω = 23.5 Ω`, i.e. ~179 mA at 4.2 V, 0.75 W in a 2 W 2512 pair. That external gate sits behind a ~10 kΩ / 10 nF network (τ ≈ 100 µs), which is why `DCP = 0` alone is not enough — see **quiesce**. |
| **Balancing** | Passive top-balancing: bleed the high cells down until the pack matches. Policy is a pure function, `balance::compute_mask` in `balance_controller.hpp`; the WRCFGA packing and chain TX live in `BmsPollTask`. Ranks against the **second-lowest** cell in the pack, not the lowest, so one stuck-low reading cannot trigger a pack-wide bleed. Never selects two physically adjacent cells at once (`BalanceSpreadNoAdjacent`) — measured ~71 °C per pad at 8 cells/module, so spreading the heat bounds the local hot spot. |
| **Quiesce** | Clearing the DCC bits and waiting `BalanceQuiesceMs` (2 ms) **before** starting a cell-voltage or ADOW conversion, so no bleed current flows while the cells are measured. Necessary because the LTC's `DCP = 0` only suspends its *own* switch, and this board bleeds through an external PMOS whose gate turn-off (τ ≈ 100 µs) is the same order as the first channels' conversion time. The bleed returns through the harness, not the Kelvin sense path: ~179 mA across 50–200 mΩ of tap/connector impedance is 9–36 mV, with **opposite sign** on the bled cell (low) and its neighbours (high, because the shared tap node moves) — a first-order corruption of the very 50 mV signal balancing selects on. Success/failure counts are published on pit-diag `0x6CB`; a rising fail count means voltages are being sampled under bleed. |
| **Bleed** | Discharging a single cell through its balancing resistor. Also, separately, discharging the **DC link** through its bleed resistor — the two are unrelated hardware. Context tells you which. |
| **Tap / tap node** | The shared connection between two series-adjacent cells. Each LTC channel measures *across* two taps, so one bad node corrupts **two** readings. |
| **Tap artifact** | Balancing current through a high-resistance tap shifts the shared node: one cell reads impossibly high, its neighbour compensates low, and the **pair sum stays normal**. `recompute_summaries_` detects this (implausible value + conserved sum + a split ≥ `TapArtifactMinSplitMv`) and feeds the tap-immune *pair average* to the safety aggregates, so the artifact cannot false-trip over/under-voltage. Raw `cell_mV` is left untouched for the pit-diag grid — which is exactly why the balance selector, which reads raw values, needs its own cell-data fault gate. |
| **ADOW** | Start Open-Wire ADC Conversion — the LTC6811 command that pushes a small pull-up (`PUP = 1`) or pull-down (`PUP = 0`) current onto the cell inputs. Two passes, each read back with the normal `RDCV*` commands, expose a broken sense wire that would otherwise read plausibly in range. |
| **PUP** | The ADOW pull-up/pull-down select bit — **bit 6** of the command word, which is built from base `0x0228` with `MD` at bit 7, `PUP` at bit 6, `DCP` at bit 4 and `CH` in the low bits (`ltc6811.hpp`, `adow_cmd`). Getting its position wrong is silent: a swapped encoding leaves the `PUP = 1` pass correct by coincidence and emits a second pull-*up* pass for `PUP = 0`, so both passes agree and no open is ever detected. |
| **DCP** | Discharge Permitted — the ADCV/ADOW bit that lets discharge continue during a conversion. Set to 0 here, but see **quiesce**: on this board `DCP = 0` is necessary and not sufficient. |
| **Open-wire detection** | `CellOpenWireCheck` is **enabled**, faults `CellOpenWire` in *any* state, and is the only predicate that can see a broken cell tap (an open reads ~−4000 mV against a 400 mV threshold; a healthy pack stays inside −130..+50 mV). **Honest gap:** only the *interior* conductor rules are hardware-validated. The endpoint rules — C(0) via `CELL_PU(1) == 0` and C(N) via `CELL_PD(N) == 0` — test for **exact zero** and have never run on hardware, so an endpoint open reading a few mV instead of 0 would be missed. That is ~2 of every 10 conductors per IC. |
| **ADCV / ADAX / ADOW / RDCVA–D / RDAUXA–B / WRCFGA** | LTC6811 commands: start cell-voltage conversion / start auxiliary (temperature) conversion / start open-wire conversion / read cell-voltage groups / read auxiliary groups / write config (which is where DCC lives). Wire format in [`ltc6811.hpp`](../Core/Inc/app/ltc6811.hpp), protocol notes in [`BMS_LTC6811.md`](BMS_LTC6811.md). |
| **`NtcNoReading`** | The sentinel written to a temperature channel that produced no valid conversion — unpopulated, open, shorted or PEC-failed. It must be a sentinel and not a plausible number, because a plausible default makes a completely dead temperature path indistinguishable from a cool pack. Consumers therefore have to check `valid_temp_channels` before trusting `min_tempC` / `max_tempC`: with a count of 0 those mean "no thermal data", not "cold". `balance::compute_mask` enforces exactly this via `BalanceMinValidTempCh` — `max_tempC` at `INT16_MIN` compares as wonderfully cool, so without the check a pack with a dead temperature path would balance with no thermal protection and no symptom. |

## State of charge

| Term | Meaning |
|---|---|
| **SoC** | State of Charge, 0..100 %, published on `0x130` every 250 ms. `0xFF` (`soc::Unknown`) means "no trustworthy estimate". **SAFETY CONTRACT: telemetry only.** No safety predicate reads it; if `soc_estimator.hpp` produced pure garbage the AMS would behave identically. Keep it that way. |
| **Coulomb counting** | Integrating pack current to track charge moved. Exact over short horizons, unbounded drift over long ones (sensor offset integrates linearly), so it needs an anchor. Note the firmware-wide sign convention: **+ current = discharge**, so a positive current *removes* charge. |
| **OCV** | Open-Circuit Voltage. At rest a cell's terminal voltage *is* its OCV, and the fitted VTC6 curve maps it to SoC. Under load the terminal includes the I·R drop and the mapping is invalid. The curve is deliberately documented as **flat in the middle** (~9.4 mV per SoC point from 0.30–0.50) and steep at the top (~2.3 mV per point above 0.90) — which is why a millivolt of error costs ~0.1 points mid-pack and ~0.04 near full. |
| **Rest gate** | The two conditions for a valid OCV anchor: current magnitude at or below `SocRestCurrentMa` (500 mA) **and** rested for `SocRestSettleMs` (5 min, `COMMISSION`). The settle time is the surprising half — the ohmic part of polarisation recovers in microseconds, but the concentration gradient takes minutes, so anchoring 10 s after a hard discharge reads tens of mV low. |
| **EKF** | Extended Kalman Filter — the live estimator (`soc::KalmanSoc`, run by `CurrentSensorTask`). One state (SoC). Prediction *is* Coulomb counting; correction is the voltage residual against an OCV − I·R_element model. Two reasons it beats CC-plus-anchor: the observation matrix **H = dOCV/dSoC** makes the gain self-schedule (small on the flat plateau, 2.4× larger near the ends — no hand-written blending rule), and the measurement variance **R grows with I²**, so a reading taken under load is automatically distrusted instead of being gated on/off. |
| **R_int / R_element** | Internal resistance. The model computes a *cell* resistance from `RIntNomMicroOhm` × f(SoC) × f(T), then **divides by `CellsInParallel` (6)** because the LTC measures a 6P group. Forgetting the divide overstates the I·R term sixfold. |
| **Why `double`** | Not stylistic. The process-noise step is `Q·dt = 5e-10` while float32 epsilon at `P ≈ 0.04` is ~4.8e-9, so every increment would round to zero, `P` would never grow, and the filter would silently go deaf to voltage after a few corrections. The M7 here has hardware double precision, so it is free. |
| **`CoulombCounter`** | The pure Coulomb-counting class in `soc_estimator.hpp`, with `anchor()` / `ocv_anchor_valid()`. **Host-tested only — not wired into the firmware.** The live path is `KalmanSoc`. Do not assume the rest-gated anchor runs on the car. |

## CAN & tooling

| Term | Meaning |
|---|---|
| **FDCAN1 / FDCAN2** | The STM32H7 CAN-FD peripherals. **FDCAN1** is the live accumulator bus and carries everything the app does — VCU RX, ECU TX matrix, telemetry, pit-diag, the boot trigger, and the LOGFS transport. **FDCAN2** is not started by the app at all; only the bootloader drives it. All app traffic is **standard-frame only** (the hardware global filter rejects extended frames at the gate). |
| **`0x100`** | VCU DC-bus heartbeat. Bytes 0–1 LE = `dc_bus_V`; **byte 2 bit 0 = `discharge_engaged`**, optional on the wire (DLC 2 from an older ECU reads as 0). Freshness within `VcuFreshMs` (1000 ms) at the trigger selects Car mode; freshness within `VcuStaleMs` (200 ms) is what `precharge_target_reached` and the `VcuStale` predicate use. |
| **`0x101`** | Operator charge-mode request, magic `43 48 52 47` ("CHRG"), sent by the pit tool at ≥2 Hz. Two roles: with VCU absence it selects Charger mode at the lock, and a **still-fresh** `0x101` is the Charger-mode precharge proceed (the DASH_CHG press is the human "go"; `0x101` freshness says the charger is still connected). Going stale mid-charge raises `ChargerStale`. |
| **`0x103` / `0x104`** | Operator balancing control. `0x103` is the master switch — `"BALO"` off, `"BALN"` on in any state, `"BALX"` auto (Charge-only). `0x104` (`"BALM"` + a 5-bit mask) narrows which modules may balance, layered *under* `0x103`. Both magic-gated and both dead-manned. Neither can touch an AIR or any safety path. |
| **`0x020` / `0x021` / `0x12C` / `0x130`–`0x137`** | The ECU TX matrix, sent by `AcuCanTask` on three cadences. 50 ms: `0x135` currents (accu + DCDC, i16 deciamps). 100 ms: `0x020` ok_precharge, `0x021` discharge_interlock, `0x12C` pack-wide min cell, `0x131`/`0x132` per-module vmin, `0x133`/`0x134` per-module vmax. 250 ms: `0x136`/`0x137` per-module max temp and `0x130` SoC. |
| **`0x4A0` / `0x4A1` / `0x4A2`** | AMS telemetry — status (state + cell-V extremes), pack (V + current), temps (+ DC bus, heartbeat, cockpit byte) — every 500 ms. The cockpit byte at `0x4A2[5]` carries a sentinel bit 7, mode in bits 3:2, TSMS in bit 1, DASH_CHG in bit 0. |
| **`0x4A4`** | Relay status: contactor and `AMS_OK` GPIO read-backs, always on, every 100 ms — so a logger can watch the AIR/precharge sequence without arming pit-diag. These are **ODR read-backs**: they confirm what the firmware drives the coils to, never that a contactor physically closed. |
| **Pit-diag** | The runtime-armed diagnostic CAN stream. Armed by `0x7F0` with magic `DE AD BE EF` (disarm with four zero bytes), acknowledged on `0x7F1`, then scanned at 1 Hz: 24 cell frames from `0x680`, 25 temperature frames from `0x6A0`, and a status block `0x6C0`–`0x6C9` plus `0x6CB` (FSM detail, poll timing, balance masks, boot diag, post-mortem, firmware ID, per-IC PEC counts, comms health, balance-quiesce health). Sent with a blocking, yield-while-full helper, because the TX FIFO is only 16 deep and an unthrottled 60-frame burst would silently lose everything past the front. |
| **`0x6CA`** | The **ungated** firmware-health frame. Its ID sits inside the pit-diag block but it is emitted at 1 Hz *regardless* of the arm state, so a passive listener can answer "is the AMS app alive?" (heap, task liveness, reset cause, uptime, last fault) without transmitting anything. |
| **CAN DSL** | The code-first message definitions in `Core/Inc/can/messages/*.def` — one `CAN_MSG` block per frame, with bit offsets, widths, scaling and a sender. These are the **source of truth** for the wire format; the encoders and the DBC are both derived from them. |
| **DBC** | The CAN database at `docs/dbc/ams.dbc`, **generated** from the DSL by [`tools/dbc_dump.cpp`](../tools/dbc_dump.cpp) and consumed by MingoCAN. Never hand-edit it; regenerate. CI's "DBC matches code" check fails the build otherwise. |
| **fault_reason** | The byte on pit-diag `0x6C0[6]` naming which branch latched `Error`, with a detail byte at `[7]` (module index or mask). Values are an **append-only wire contract**: 0 None, 1 ForceError, 2 BmsModuleOffline, 3 BmsStale, 4/5 Cell Under/OverVoltage, 6/7 Cell Under/OverTemp, 8 CurrentSensorFault, 9 CurrentStale, 10 CurrentOverLimit, 11 VcuStale, **12 FsmError** (FSM-driven, e.g. a precharge timeout — it has no enum slot, it is set by `MainTask`), 13 TempSensorDisconnected, 14 ChargerStale, 15 ChargerTsmsOpen, 16 CellOpenWire. Note the gap: there is no 12 in `FaultReason`. |
| **Bus-Off** | The CAN error state where a node stops transmitting after too many errors. `AcuCanTask` polls for it and recovers on its own cadence — the poll runs unconditionally because a Bus-Off node receives nothing, so nothing else would wake the loop. The recovery count is published on `0x6C9`, which is how a CAN-only bench confirms a recovery fired. |
| **ISO-TP / LOGFS** | The multi-frame transport (`isotp.hpp`) and the log-file server (`logfs_server.hpp`) that ship SD-card logs over CAN, addressed like the bootloader (`0x000 + node` in, `0x010 + node` out). A pull is a multi-minute operation, which is why `DiagTxReservedSlots` (6 of 16 FIFO slots) is held back for flight telemetry throughout. |

## Build, test & bench

| Term | Meaning |
|---|---|
| **CubeMX** | ST's code generator. Owns `AMS.ioc` and the generated HAL/RTOS C (`main.c`, `freertos.c`). Application code lives in `Core/{Inc,Src}/app/` and is entered through `extern "C"` `ams_*_task_run` trampolines. Anything you must put in a generated file has to sit inside a `USER CODE BEGIN/END` block or the next regeneration deletes it. |
| **HAL** | Hardware Abstraction Layer — ST's peripheral driver library. |
| **MainTask vs SafetyTask** | Two names for one thread, and the first thing that confuses a newcomer. The CubeMX thread is `SafetyTask` (`main.c`, `AMS.ioc`); the body in `safety_task.cpp` calls itself `MainTask` because that one 10 ms loop owns the predicates, the FSM step, the relay drive, `AMS_OK` and telemetry together. Expect both names in comments. It is the only realtime-priority thread, so no producer can preempt it. |
| **Tasks** | `App_InitTask` (boot bring-up), `SafetyTask`/`MainTask` (10 ms safety loop), `BmsPollTask` (LTC chain), `CurrentSensorTask` (ADC + SoC), `AcuCanTask` (CAN RX/TX + pit-diag), `SdLoggerTask` (datalogging). |
| **Single-writer service** | The synchronisation model: `BmsService`, `CurrentService` and `VehicleService` each have exactly one writing task and many readers, with no mutex — 32-bit atomic access plus the contract. A reader can therefore observe a **torn snapshot** (fields from two different poll cycles), which is precisely what the fault debounces and the `NoOffendingModule` (0xFF) detail sentinel exist to absorb. |
| **IWDG** | Independent Watchdog. Prescaler 32 on the ~32 kHz LSI with reload 100 ≈ **100 ms**. `MainTask` refreshes it every iteration; if the loop stops, the chip resets and the relays default open. |
| **Unity** | The C unit-test framework used by the host tests in `tests/unit/`. |
| **SIL** | Software-in-the-Loop — the multi-step pure-FSM scenario tests (`test_sil_scenarios.cpp`) that drive the FSM through realistic input sequences rather than single transitions. |
| **HIL** | Hardware-in-the-Loop — the real-hardware bench rig (MLC2 carrier + a Pi Pico emulating the LTC6820/LTC6811 chain + CAN/GPIO fixtures). Same isoSPI traffic and PEC validation as flight. The `dev → main` release is gated on the HIL acceptance plan. |
| **MLC2** | The bench carrier board the AMS chip sits on for HIL testing. |
| **`AmsNodeId`** | `0x02` — the AMS's node ID on the shared bootloader bus (role map: ECU 1, AMS 2, uDV 3). It must match the value the bootloader was compiled with (`-DBL_NODE_ID`), and it is embedded in the firmware metadata so MingoCAN can refuse a mismatched flash. Changing it means rebuilding both halves. |
| **`AMS_HIL_CLEAR_ERROR_LATCH`** | A bench-only CMake option that wipes the sticky `ErrorLatch` on every boot, so a bench session comes up clean. **Never in a flight image.** See [`HIL_BUILD.md`](HIL_BUILD.md). |
| **`COMMISSION`** | A marker on `ams_config.hpp` constants that are *placeholders derived from datasheets or rules*, not values measured on this car. Every one must be validated on real hardware before flight. The sign-off sheet is [`COMMISSIONING_CHECKLIST.md`](COMMISSIONING_CHECKLIST.md); the procedures are in [`COMMISSIONING.md`](COMMISSIONING.md). |
| **Bootloader** | The CAN bootloader that owns flash **sector 0** (`0x08000000`–`0x0801FFFF`); **sector 7** (`0x080E0000`–`0x080FFFFF`) is its NVM plus app metadata, and the application gets **sectors 1–6** — `0x08020000`, 768 KB. So the app image does *not* start at the reset vector, which is why `scripts/check_flash_layout.py` runs in CI: an image that grows into either reserved region bricks the update path. The bootloader checks `RTC->BKP0R` at every reset and stays in BL mode if it holds magic `0xB00710AD`, otherwise it jumps to the app. `Bootloader::request_reboot` is the app's one-way path back: open all relays, drain the TX FIFO so any in-flight ACK reaches the wire, write the magic, `NVIC_SystemReset`. It is triggered by a single frame on FDCAN1 — ID `0x002`, DLC exactly 4, payload exactly `B0 07 AD 11`. |
