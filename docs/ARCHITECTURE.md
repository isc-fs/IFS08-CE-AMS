# AMS firmware architecture

Target hardware: STM32H733ZGTx (Cortex-M7 @ 264 MHz core, 1 MB Flash,
~1 MB RAM). RTOS: FreeRTOS via CMSIS-RTOS v2 (1000 Hz tick).
Language: C++17 for application code (no exceptions, no RTTI, no
thread-safe statics) on top of CubeMX-generated C for the HAL/RTOS
boilerplate.

This document is the **as-built** reference. The reverse-engineered
legacy protocol is in [`CAN_MAP.md`](CAN_MAP.md); the bench /
on-vehicle calibration procedure is in
[`COMMISSIONING.md`](COMMISSIONING.md).

---

## 1. Safety invariants

Everything else exists to enforce these:

1. **If the pack is unsafe, AIRs open within one safety period
   (10 ms).** "Unsafe" = any of the predicates in
   [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp).
2. **Relays default to open** on any reset, fault, or uninitialised
   state. CubeMX-generated `MX_GPIO_Init` writes `PIN_RESET` to
   PB5 / PB6 / PB7 **before** configuring them as outputs.
3. **No single stuck task can prevent an AIR open.** `MainTask`
   runs at `osPriorityRealtime`, has direct GPIO write access via
   `ams::Relays`, and owns the watchdog refresh. The same task
   owns FSM step, predicate evaluation, relay drive, and
   telemetry emission — one timeline, no cross-task race.
4. **The watchdog is fed only on `MainTask`'s clean path** (and on
   the latched-fault path; see invariant 5). A stuck supervisor →
   IWDG timeout (≈100 ms) → hardware reset → relays default open.
5. **ERROR is latched across resets** via `RTC->BKP1R` (magic
   `0xA115EE51`). Cleared only by full backup-domain power loss.
   `BKP0R` is reserved for the bootloader's boot-request handshake
   (`0xB00710AD`), `BKP2R` is reserved for the jump-reason ASCII
   tag; no two registers share a word. Once latched, MainTask keeps
   refreshing the watchdog (relays already open, latch persists) so
   the chip stays alive for diagnosis instead of self-resetting in a
   100 ms loop. See PR #107 for the loop bug this avoids.
6. **Shared sensor state has one writer per service.** Single-writer
   / many-reader contract — Cortex-M7 32-bit aligned loads/stores
   are atomic. Multi-field reads can briefly observe a mid-update
   snapshot, but the predicate and telemetry are tolerant of
   one-cycle staleness. No mutex required; the old per-service
   mutex was retired in refactor/19 phase 1 (PR #119).
7. **Boot-grace window suppresses data-presence predicates for
   `SafetyBootGraceMs` (2 s) after `osKernelStart`.** At t = 0
   every service's `last_*_tick` is 0; without a grace the first
   MainTask iteration would fault on freshness and withhold the
   watchdog refresh, triggering an IWDG reset within ~100 ms. The
   window covers BmsPollTask's first voltage poll (250 ms),
   CurrentSensorTask's first ADC sample (50 ms), and AcuCanTask's
   first VCU 0x100 (uncontrolled but typically present).
   **Immediate-safety predicates stay active during the grace**:
   `force_error_set` still trips instantly. Only data-dependent
   predicates wait. See
   [`safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp)
   and the inline comment on `SafetyBootGraceMs` in
   [`ams_config.hpp`](../Core/Inc/app/ams_config.hpp).
8. **The AMS does not sense the SDC line.** It IS part of the SDC
   via the ``AMS_OK`` output (PB4) — opening that output opens the
   SDC. The legacy ``DIGITAL1`` (PE9) input was retired in PR #117
   because the v1.2 daughterboard doesn't route it; the
   ``sdc_closed`` predicate is gone.

> The HIL bench rig uses the `AMS_BMS_HIL_STUB` build flag to
> relax a few bench-only invariants (current-sensor freshness, latch
> stickiness, telemetry layout). The legacy BMS-data-source seeder
> was retired in #207; HIL builds now drive a real LTC6820/LTC6811
> via the Pi Pico emulator (IFS08_HIL `feat/pico-ltc-emulator`).
> The flag is **never compiled into a flight build**. Full semantics
> in [`HIL_STUB.md`](HIL_STUB.md).

---

## 2. Build model

CubeMX 6.16 generates a **CMake-based** project (not Eclipse-managed-
make). The boundary between generated and hand-written code:

```mermaid
flowchart LR
    subgraph Gen["CubeMX-owned (regenerated from AMS.ioc)"]
        G1[Core/Src/main.c<br/>Core/Src/freertos.c<br/>Core/Src/stm32h7xx_*.c]
        G2[Drivers/STM32H7xx_HAL_Driver/]
        G3[Middlewares/Third_Party/FreeRTOS/]
        G4[cmake/stm32cubemx/CMakeLists.txt]
    end
    subgraph Hand["Hand-written (lives forever)"]
        H1[Core/Inc/app/*.hpp + *.h]
        H2[Core/Src/app/*.cpp]
        H3[CMakeLists.txt<br/>top-level, edited once]
        H4[tests/unit/<br/>host CMake + Unity]
    end
    Gen -. "USER CODE BEGIN ... END blocks<br/>call ams_*_task_run trampolines" .-> Hand

    classDef gen  fill:#fde68a,stroke:#a16207,color:#1c1917
    classDef hand fill:#34d399,stroke:#065f46,color:#052e16
    class G1,G2,G3,G4 gen
    class H1,H2,H3,H4 hand
```

C++ code never modifies the generated `main.c`. Instead it provides
`extern "C"` trampolines (`ams_safety_task_run`,
`ams_state_task_run`, etc.) that the CubeMX-preserved
`USER CODE BEGIN … END` blocks call. Every regen keeps the
trampolines.

Cross-compile uses `arm-none-eabi-gcc 14.x`; host tests use the
system toolchain (Clang on macOS, GCC on Linux/CI).

---

## 3. Task architecture

**6 live tasks** post-refactor/19 (PRs #119 + #120 + #121), plus
CMSIS `defaultTask` and the FreeRTOS timer-service daemon. The
collapsed `StateTask` and `TelemetryTask` thread handles are still
declared by CubeMX-generated `MX_FREERTOS_Init` for compatibility,
but their entry points immediately `osThreadExit()` — TCB and stack
return to the heap, no CPU consumed.

| Task | Priority (enum / value) | Period | Stack (words) | Implementation |
|---|---|---|---|---|
| `App_InitTask` | High (40) | once | 512 | [`app_init_task.cpp`](../Core/Src/app/app_init_task.cpp) |
| `MainTask` *(thread name still "SafetyTask")* | Realtime (48) | 10 ms | 512 | [`safety_task.cpp`](../Core/Src/app/safety_task.cpp) |
| `BmsPollTask` | Normal (24) | 250 ms / 500 ms (isoSPI poll + balance WRCFGA) | 256 | [`bms_poll_task.cpp`](../Core/Src/app/bms_poll_task.cpp) |
| `AcuCanTask` | AboveNormal (32) | RX-queue-driven (FDCAN1; dispatches boot-trigger 0x002) | 512 | [`acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp) |
| `CurrentSensorTask` | AboveNormal (32) | 50 ms | 256 | [`current_task.cpp`](../Core/Src/app/current_task.cpp) |
| `defaultTask` | Low (8) | — | 128 | (CMSIS placeholder) |
| Timer service | Normal | callback-driven | 256 | FreeRTOS daemon |
| `StateTask` *(retired)* | High (40) | n/a | 1024 | self-exiting stub; logic now in `MainTask` |
| `TelemetryTask` *(retired)* | Low (8) | n/a | 512 | self-exiting stub; logic now in `MainTask` |

### MainTask body

One timeline, tick-gated cadences:

```cpp
for (;;) {
    osDelayUntil(last_wake += 10);
    snapshot bms / current / vehicle
    fault = error_latched || evaluate_fault(...)
    if (fault) {
        if (!error_latched) latch (open relays + set ErrorLatch + fan off)
        refresh watchdog                       // stay alive for diag
    } else {
        every 20 ms: fsm::step() → apply relay actions inline
        refresh watchdog
    }
    every 500 ms: emit telemetry frames 0x4A0/0x4A1/0x4A2
}
```

No event-flag ping-pong: the FSM's output bitmask is consumed
inline by the same task that produced it. The old `safety_events`
event group is still declared by CubeMX (cosmetic cleanup
deferred) but no one reads or writes it.

### BmsPollTask body

Owns the LTC6811 isoSPI conversation end-to-end and the balance
DCC writes (`maybe_run_balance_update` runs every
`BalanceUpdatePolls` voltage cycles, inline after RDCV[A-D],
sharing the same `LTC6820::Bus`). Single owner of the chain — no
bus mutex, no producer/consumer queue. Runs on every build flavour
post-#207 — HIL_STUB no longer compiles out the real LTC path; the
bench drives a Pi Pico LTC6820/LTC6811 emulator on MLC2 J8. See
[`HIL_STUB.md`](HIL_STUB.md) for the remaining bench-only concerns
(diag counters, 0x4A2 layout, current-sensor freshness relax).

The legacy `BmsRxTask` was retired in v1.2.0 (#73) once the BMS
data path moved off FDCAN2; the bootloader-trigger frame it used
to dispatch now rides on FDCAN1 and is handled inside `AcuCanTask`.

### Why MainTask is realtime

`MainTask` is the only Realtime-priority task in the system. Its
period of 10 ms is the hard contract: worst-case latency from
"sensor reads out of range" to "AIRs open" is bounded by
`producer_period + 10 ms + GPIO_write < 15 ms`. The producer tasks
(`BmsPollTask`, `CurrentSensorTask`, `AcuCanTask`) all run at
strictly lower priority, so they cannot preempt MainTask.

---

## 4. Data flow

```mermaid
flowchart TD
  subgraph HW[STM32H733]
    FDCAN1[FDCAN1 RX/TX<br/>ACU + bootloader-trigger]
    FDCAN2[FDCAN2<br/>bootloader-claimed post-reset only]
    SPI1[SPI1 master + PA4 CS<br/>via LTC6820]
    ADC1[ADC3 ch3 PF7]
    GPIOB[GPIOB<br/>PB4 AMS_OK<br/>PB5/6/7 relays]
    TIM17[TIM17_CH1 PB9 fan]
    FDCAN1TX[FDCAN1 TX telemetry<br/>0x4A0/0x4A1/0x4A2]
    IWDG[IWDG1 ~100 ms]
  end

  subgraph Chain[LTC6811-1 daisy-chain]
    LTC[10 × LTC6811<br/>5 modules × 2]
    MUX[2 × ADG731 per module<br/>40 NTCs / module]
    LTC --- MUX
  end

  subgraph ISR[ISRs]
    RX1[FDCAN1 RX-FIFO0 cb]
  end

  subgraph Queues
    acu_rx[(acu_rx_queue 16)]
    bms_evt[[bms_events]]
  end

  subgraph State["Shared state (volatile, single-writer)"]
    BmsSvc[BmsService<br/>volatile BmsState]
    CurSvc[CurrentService<br/>volatile CurrentState]
    VehSvc[VehicleService<br/>volatile VehicleState]
  end

  subgraph Tasks
    AcuT[AcuCanTask]
    BmsPollT[BmsPollTask<br/>+ balance WRCFGA]
    CurT[CurrentSensorTask]
    MainT[MainTask<br/>safety + FSM + telemetry]
  end

  FDCAN1 --> RX1 --> acu_rx --> AcuT --> VehSvc
  AcuT -.charger.-> CurSvc
  AcuT -- "boot-trigger 0x002 → request_reboot" --> FDCAN1

  SPI1 <-->|"ADCV/RDCV* (V)<br/>WRCOMM+STCOMM+ADAX+RDAUXA (T)<br/>WRCFGA (balance)"| Chain
  BmsPollT -- "isoSPI conversation" --> SPI1
  BmsPollT --> BmsSvc

  ADC1 --> CurT --> CurSvc

  BmsSvc & CurSvc & VehSvc --> MainT

  MainT -- relays (inline) --> GPIOB
  MainT -- fan duty --> TIM17
  MainT -- refresh --> IWDG
  MainT --> FDCAN1TX

  bms_evt -- pollV / pollT --> BmsPollT

  classDef hw    fill:#1e293b,stroke:#0f172a,color:#f8fafc
  classDef chain fill:#0ea5e9,stroke:#0369a1,color:#f0f9ff
  classDef isr   fill:#fb923c,stroke:#9a3412,color:#1c1917
  classDef queue fill:#fde68a,stroke:#a16207,color:#1c1917
  classDef svc   fill:#60a5fa,stroke:#1e40af,color:#f8fafc
  classDef task  fill:#34d399,stroke:#065f46,color:#052e16
  classDef safe  fill:#ef4444,stroke:#7f1d1d,color:#fef2f2
  classDef dim   fill:#475569,stroke:#1e293b,color:#cbd5e1

  class FDCAN1,SPI1,ADC1,GPIOB,TIM17,FDCAN1TX,IWDG hw
  class FDCAN2 dim
  class LTC,MUX chain
  class RX1 isr
  class acu_rx,bms_evt queue
  class BmsSvc,CurSvc,VehSvc svc
  class AcuT,BmsPollT,CurT task
  class MainT safe
```

**Legend** — hardware (slate, FDCAN2 dimmed because the app no longer
drives it) · LTC6811 chain + ADG731 mux (cyan) · ISRs (orange) ·
queues & event groups (amber) · shared state (blue) · tasks (green) ·
`MainTask` (red, the only realtime-priority task in the system).

What changed from pre-refactor/19:

- **No service mutexes.** Single-writer / many-reader → `volatile`
  state, atomic 32-bit accesses. Old `bms_mutex` / `current_mutex` /
  `vehicle_mutex` handles still declared by CubeMX but unused.
- **No `safety_events` event group consumer.** FSM output bitmask
  is consumed inline by MainTask. Handle still declared by CubeMX
  but unused.
- **One realtime task instead of two-cooperating-tasks.**
  Safety + FSM + Telemetry collapsed; no ping-pong, no priority
  race surface.
- **HIL stub lives in `BmsPollTask`**, not in the predicate. The
  safety predicate is HIL-agnostic — same code on flight and bench.

Arrows are direction of value flow. The only direct GPIO writes
happen in `MainTask` (relays + watchdog + fan) and `App_InitTask`
(one-shot driver bring-up + LTC chain wakeup / length discovery).

The BMS transport is now isoSPI end-to-end; see
[`BMS_LTC6811.md`](BMS_LTC6811.md) for the LTC6811-1 wire protocol,
register-group layout, daisy-chain semantics, PEC15 rules, and the
ADG731 channel mapping that drives the temperature sweep.

---

## 5. Finite state machine

Six states. Transitions live in pure code at
[`state_machine.hpp`](../Core/Inc/app/state_machine.hpp) so they're
unit-tested in isolation; `MainTask` calls `fsm::step()` every 20 ms
on its 10 ms cadence and consumes the returned relay-action bitmask
inline (the old event-flag handoff to a separate `SafetyTask` was
retired in PR #120).

**Two mutually exclusive contexts:**

- **Run** = pack is installed in the car. Entered via the
  `Start → Precharge → Transition → Run` path on the start button.
- **Charge** = pack is removed from the car and on the charging
  station. Entered via `Start → Charge` on charger detection.

The two are decided **once per power cycle** at `Start` and cannot
flip at runtime — a pack physically cannot move between car and
charger while the AMS is alive. If `charger_detected` toggled in
`Run` it would indicate an electrical fault (CAN noise, miswired
charger); the FSM ignores it and the predicate set catches any real
current anomaly.

```mermaid
stateDiagram-v2
    [*] --> Boot

    state boot_check <<choice>>
    Boot --> boot_check
    boot_check --> Start : ErrorLatch clear
    boot_check --> Error : ErrorLatch set

    Start --> Precharge  : start button and healthy
    Start --> Charge     : charger detected

    Precharge --> Transition : dc_bus reached target
    Precharge --> Error      : precharge timeout

    Transition --> Run   : hold elapsed and steady
    Transition --> Error : dc_bus dropped

    Start      --> Error : safety fault
    Precharge  --> Error : safety fault
    Transition --> Error : safety fault
    Run        --> Error : safety fault
    Charge     --> Error : safety fault

    Error --> [*] : reset only

    note right of Run
        terminal context
        until reset or fault
    end note

    note left of Charge
        terminal context
        until reset or fault
    end note

    note right of Error
        sticky within a boot
        ErrorLatch set in backup register
        next boot starts here
    end note

    classDef idle    fill:#cbd5e1,stroke:#475569,color:#0f172a
    classDef bring   fill:#fde68a,stroke:#a16207,color:#1c1917
    classDef live    fill:#34d399,stroke:#065f46,color:#052e16
    classDef chg     fill:#22d3ee,stroke:#0e7490,color:#083344
    classDef fault   fill:#ef4444,stroke:#7f1d1d,color:#fef2f2

    class Start idle
    class Precharge,Transition bring
    class Run live
    class Charge chg
    class Error fault
```

**Legend** — idle (slate) · bring-up (amber) · running (green) ·
charging (cyan) · error (red).

Edge-transition relay actions (bits in `Output::safety_flags`,
consumed inline by MainTask):

| Transition | bits set in `Output::safety_flags` |
|---|---|
| Start → Precharge | `CloseAirN`, `ClosePrecharge` |
| Start → Charge | `CloseAirN`, `CloseAirP` |
| Precharge → Transition | `CloseAirP`, `OpenPrecharge` |
| any → Error | `ForceError`, `OpenAirN`, `OpenAirP`, `OpenPrecharge` |

**Key properties:**

- **ERROR is sticky within a boot.** Even if the underlying fault
  clears, the FSM stays in `Error` until reset. `ErrorLatch::set()`
  fires on every `Error` entry so the next boot also starts in
  `Error` until backup-domain power is cycled.
- **Predicate-fault evaluation runs first** on every MainTask
  iteration (10 ms cadence), so any fault from any state preempts
  the normal transition (20 ms FSM step). MainTask skips the FSM
  step on a fault iteration.
- **AIR / Precharge actions are driven inline by MainTask.** The
  FSM's `safety_flags` bitmask is interpreted by `apply_relay_actions`
  in the same iteration the FSM ran. No event-flag indirection, no
  cross-task race window. The faulted path skips relay actions but
  the relays are already open from `latch_error_()`.

---

## 6. Boot sequence

```mermaid
%%{init: {'theme':'base','themeVariables':{
  'actorBkg':'#e2e8f0','actorBorder':'#475569','actorTextColor':'#0f172a',
  'noteBkgColor':'#fee2e2','noteBorderColor':'#7f1d1d','noteTextColor':'#7f1d1d',
  'sequenceNumberColor':'#0f172a'
}}}%%
sequenceDiagram
  participant HW
  participant main as main()
  participant init as App_InitTask
  participant Main as MainTask
  participant aux as Bms/Acu/Cur

  HW->>main: Reset_Handler then SystemInit
  main->>main: HAL_Init, SystemClock_Config
  main->>HW: MX_GPIO_Init (relays driven low; PB4-7)
  main->>HW: MX_ADC1/3, MX_FDCAN1/2, MX_TIM17, MX_USART2, MX_SPI1
  main->>HW: MX_IWDG1_Init (IWDG alive pre-scheduler)
  main->>main: osKernelInitialize + create queues, tasks
  main->>main: osKernelStart

  par scheduler running
    init->>init: ErrorLatch::init (DBP unlock)
    Note over init: Under -DAMS_BMS_HIL_STUB:<br/>clear ErrorLatch (defence-in-depth)
    init->>HW: FDCAN1 filter + ActivateNotification + Start
    init->>HW: LTC6820 configure + wakeup + chain discovery
    alt chain discovered != LtcChainLength
      init->>HW: ErrorLatch::set + Relays::open_all
    end
    init->>init: osThreadExit
  and
    Main->>Main: ErrorLatch::init + Fan::init
    Main->>Main: boot in Error if ErrorLatch::is_set
    Note over Main: For t < SafetyBootGraceMs (2 s)<br/>data-presence predicates are suppressed.<br/>force_error_set still trips instantly.
    loop every 10 ms (osDelayUntil)
      Main->>Main: snapshot bms/current/vehicle
      alt fault detected
        Main->>HW: latch (Relays::open_all + ErrorLatch::set + fan off)
        Main->>HW: HAL_IWDG_Refresh (stay alive for diag)
      else clean path
        Main->>Main: every 20 ms: fsm::step → apply relay actions inline
        Main->>HW: HAL_IWDG_Refresh
      end
      Main->>HW: every 500 ms: emit 0x4A0/0x4A1/0x4A2
    end
  and aux
    aux-->>aux: BmsPoll 250/500 ms (+ balance WRCFGA)<br/>CurrentSensor 50 ms ADC<br/>AcuCan RX-queue drain
  end
```

`App_InitTask` self-deletes once peripheral bring-up is done; its
TCB and stack return to the heap. The retired `StateTask` and
`TelemetryTask` thread entry points also call `osThreadExit()`
immediately, so their stacks return to the heap on boot too.

---

## 7. Shared state

Three domain services. Each is a singleton wrapping a `volatile`
struct. **Single-writer / many-reader**, lock-free.

| Service | Writer | Readers |
|---|---|---|
| [`BmsService`](../Core/Inc/app/bms_service.hpp) | `BmsPollTask` (LTC6811 isoSPI sweeps; `update_from_ltc_response` + `update_temperature`) | MainTask, AcuCanTask, BalanceController |
| [`CurrentService`](../Core/Inc/app/current_service.hpp) | `CurrentSensorTask` (ADC), `AcuCanTask` (charger flag) | MainTask, BmsPollTask |
| [`VehicleService`](../Core/Inc/app/vehicle_service.hpp) | `AcuCanTask` | MainTask |

Concurrency model: Cortex-M7 32-bit aligned loads/stores are atomic.
Multi-field reads from a non-writer task can briefly observe a
mid-update snapshot — the predicate + telemetry are tolerant of
one-cycle staleness (any inconsistency causes at worst one extra
predicate evaluation that corrects on the next 10 ms iteration).
The mutex handles (`bms_mutexHandle` etc.) are still declared by
CubeMX in `MX_FREERTOS_Init` but no one acquires them anymore;
cleanup deferred to a future `.ioc` pass.

`BmsState` shape:

| Field | Type | Notes |
|---|---|---|
| `cell_mV` | `uint16_t [5][19]` | 95 cells; populated by `update_from_ltc_response`. |
| `cell_tempC` | `int16_t [5][40]` | 200 NTC slots; populated by `update_temperature`. Ctor-initialised to 25 °C so unpopulated channels don't dominate `max_tempC`. |
| `pack_voltage_mV`, `min/max_cell_mV`, `min/max/avg_tempC` | summaries | recomputed inside `recompute_summaries_` after every write. |
| `last_rx_tick[5]` | `uint32_t` | advances only on a poll where BOTH LTCs of the module passed PEC. |
| `module_online_mask` | `uint8_t` | sticky (once-online); compared against `AllModulesMask`. |
| `ltc_online_mask` | `uint16_t` | non-sticky per-cycle per-IC PEC-OK mask (10 bits). |

---

## 8. Inter-task signalling

Two primitives, nothing else.

**Queues** ([`AMS.ioc`](../AMS.ioc) `FREERTOS.Queues01`):

| Queue | Depth | Item | Producer | Consumer |
|---|---:|---|---|---|
| `acu_rx_queue` | 16 | `CanFrame` | FDCAN1 RX ISR | AcuCanTask |
| `acu_tx_queue` | 16 | `CanFrame` | (reserved) | (reserved) |

`bms_rx_queue` was retired in v1.2.0 (#73) along with `BmsRxTask`.
`acu_tx_queue` is declared in the .ioc but currently unused —
`AcuCanTask` and `MainTask` are sole producers on FDCAN1 TX and
call HAL directly.

**Event groups** (managed by
[`app_globals.cpp`](../Core/Src/app/app_globals.cpp) because
CubeMX 6.16 doesn't emit them from .ioc):

| Group | Bit | Set by | Cleared by |
|---|---|---|---|
| `bms_events` | `PollVDue` | osTimer 250 ms | BmsPollTask (ADCV + RDCV[A-D] over isoSPI) |
|  | `PollTDue` | osTimer 500 ms | BmsPollTask (20-channel ADG731 mux sweep) |

The `safety_events` event group was retired in refactor/19 phase 3
(PR #120): the FSM's relay-action bitmask is now consumed inline by
`MainTask` in the same iteration the FSM produced it. The handle is
still declared by CubeMX (`MX_FREERTOS_Init`) but no one reads or
writes it; cleanup deferred to a future `.ioc` pass.

---

## 9. Memory budget (as built)

| Region | Used | Capacity | % used | Notes |
|---|---:|---:|---:|---|
| FLASH | ~78 KB | 768 KB | ~10 % | app region only (sectors 1..6). Sector 0 reserved for the bootloader, sector 7 for BL NVM + app metadata. Cross-compile run summary on each CI build reports the exact byte count. |
| DTCMRAM | ~46 KB | 128 KB | ~36 % | stacks, TCBs, BSS, FreeRTOS heap_4. |
| ITCMRAM | 0 B | 64 KB | 0 % | unused. |
| RAM_D1/D2/D3 | 0 B | — | — | unused. |

The 768 KB ceiling is enforced both at link time (`STM32H733XG_FLASH.ld`
`FLASH` region is sized for the app range) and in CI (the
[`check_flash_layout.py`](../scripts/check_flash_layout.py) step in
`build-tests.yml` rejects any image that lands in sector 0 or
overflows sector 6).

FreeRTOS heap (`configTOTAL_HEAP_SIZE = 32 768`): ~14.7 KB used,
~18 KB free. Newlib reentrant structs account for ~1 KB. Three
load-bearing tasks (`App_InitTask`, `MainTask`) were originally
planned as `Static` allocation; CubeMX UI workflow makes that
brittle, so they ship `Dynamic` for now. Heap has enough headroom
that this is fine; revisitable post-commissioning.

---

## 10. C++ rules

Compiler flags (top-level [`CMakeLists.txt`](../CMakeLists.txt)):

```cmake
-std=c++17
-fno-exceptions
-fno-rtti
-fno-threadsafe-statics
-fno-use-cxa-atexit
-Wall -Wextra -Wpedantic
```

**Required patterns:**

- One service / task per `.hpp` + `.cpp` pair, named
  `<thing>_service` or `<thing>_task`.
- Singletons via `static` local in `instance()`. Construction order
  controlled by explicit `init()` calls from `App_InitTask`.
- Task entry points are `extern "C"` trampolines that call into
  the implementation TU.
- `enum class` for state types (`fsm::State`, `CanBus`).
- `constexpr` for every threshold, ID, period — collected in
  [`ams_config.hpp`](../Core/Inc/app/ams_config.hpp). Anything
  needing on-vehicle tuning is tagged `COMMISSION` (see
  [`COMMISSIONING.md`](COMMISSIONING.md)).
- **Single-writer per shared struct.** No mutexes in app code —
  Cortex-M7 atomic 32-bit accesses + the producer/consumer
  contract make them unnecessary. If you find yourself wanting a
  mutex, the design is wrong; talk to the safety reviewer.
- No `new` / `delete` in steady state. FreeRTOS allocates at task /
  queue / event-group creation only.

**Banned:** `std::string`, `std::vector`, `<iostream>`, `<thread>`,
heap allocation after `osKernelStart`.

---

## 11. Testing

Two layers, both in [`tests/unit/`](../tests/unit/) (the SIL
"scenarios" are pure-FSM multi-step tests; they don't need a
separate harness):

| Layer | Files | Coverage |
|---|---|---|
| Unit (single-step) | `test_bms_service`, `test_current_service`, `test_vehicle_service`, `test_safety_predicates`, `test_state_machine`, `test_bootloader`, `test_ltc6811_decode`, `test_telemetry_encoders`, `test_balance_controller` | ~95 tests: LTC6811 PEC15 + register decoders + chain-length walker, ADG731 channel packing, balancing policy, BMS / current / vehicle service decode + freshness, ADC scaling, each safety predicate in isolation, each FSM transition, telemetry encoders, boot-trigger frame matcher |
| SIL (multi-step) | `test_sil_scenarios` | 5 scenarios: nominal startup, precharge timeout, BMS dropout, charger path, SDC sticky |

**~95 / ~95 PASS on every push** via
`.github/workflows/build-tests.yml`, which also cross-compiles the
firmware with `arm-none-eabi-gcc` and reports flash/RAM sizes in
the run summary.

The pure-logic separation (FSM, predicates, decoders, scaling) is
the key design decision that keeps the test surface large without
mocking HAL or FreeRTOS — only `osMutexAcquire/Release` and the
mutex-handle externs are stubbed in
[`tests/unit/mocks/`](../tests/unit/mocks/).

---

## 12. File layout

```
IFS08-CE-AMS/
├── AMS.ioc                          # CubeMX (source of truth for HW)
├── CMakeLists.txt                   # firmware build (CMake)
├── README.md
├── ROADMAP.md                       # auto-generated, do not edit
├── CONTRIBUTING.md
├── STM32H733XG_FLASH.ld
├── startup_stm32h733xx.s
├── Core/
│   ├── Inc/                         # CubeMX-owned C headers
│   │   ├── FreeRTOSConfig.h, main.h, stm32h7xx_*.h
│   │   └── app/                     # HAND-WRITTEN
│   │       ├── acu_can_task.h
│   │       ├── ams_config.hpp       # ALL constexpr (incl. LTC/NTC tunables)
│   │       ├── ams_events.hpp       # event-group bits
│   │       ├── app_globals.h
│   │       ├── app_init_task.h
│   │       ├── balance_controller.hpp  # pure-logic balancing policy (#74)
│   │       ├── bms_poll_task.h
│   │       ├── bms_service.hpp
│   │       ├── bootloader.hpp
│   │       ├── can_frame.{h,hpp}
│   │       ├── current_service.hpp
│   │       ├── current_task.h
│   │       ├── error_latch.hpp
│   │       ├── fan.hpp
│   │       ├── ltc6811.hpp          # pure-logic LTC6811 wire layer (#67)
│   │       ├── ltc6820.hpp          # SPI/CS isoSPI master wrapper (#68)
│   │       ├── relay_driver.hpp
│   │       ├── safety_predicates.hpp
│   │       ├── safety_task.{h,hpp}      # MainTask body lives here (rename pending)
│   │       ├── scoped_mutex.hpp         # orphaned since refactor/19 phase 1
│   │       ├── state_machine.hpp
│   │       ├── state_task.h             # stub; logic merged into MainTask in #120
│   │       ├── telemetry_task.h         # stub; logic merged into MainTask in #120
│   │       ├── vehicle_service.hpp
│   │       └── watchdog.{h,hpp}
│   ├── Src/
│   │   ├── main.c, freertos.c       # CubeMX-owned; USER CODE blocks
│   │   │                              call ams_*_task_run trampolines
│   │   ├── stm32h7xx_*.c, sysmem.c, syscalls.c
│   │   ├── stm32h7xx_hal_timebase_tim.c
│   │   └── app/                     # HAND-WRITTEN
│   │       ├── acu_can_task.cpp     # also dispatches boot-trigger on FDCAN1
│   │       ├── app_globals.cpp
│   │       ├── app_init_task.cpp
│   │       ├── bms_poll_task.cpp    # LTC6811 isoSPI driver (V + T + balance)
│   │       ├── bms_service.cpp
│   │       ├── bootloader.cpp
│   │       ├── can_isr.cpp           # HAL_FDCAN_RxFifo0Callback (FDCAN1 only)
│   │       ├── current_service.cpp
│   │       ├── current_task.cpp
│   │       ├── error_latch.cpp
│   │       ├── fan.cpp
│   │       ├── firmware_info.cpp
│   │       ├── ltc6811.cpp           # PEC15 + register-group decoders + WRCFGA
│   │       ├── ltc6820.cpp           # HAL_SPI wrapper, wakeup, STCOMM
│   │       ├── relay_driver.cpp
│   │       ├── safety_task.cpp          # MainTask (safety + FSM + telemetry + fan)
│   │       ├── state_task.cpp           # stub: osThreadExit() — retired in #120
│   │       ├── telemetry_task.cpp       # stub: osThreadExit() — retired in #120
│   │       ├── vehicle_service.cpp
│   │       └── watchdog.cpp
│   └── Startup/startup_stm32h733zgtx.s
├── Drivers/                          # STM32H7 HAL + CMSIS
├── Middlewares/Third_Party/FreeRTOS/
├── cmake/
│   ├── gcc-arm-none-eabi.cmake       # toolchain file
│   ├── starm-clang.cmake
│   └── stm32cubemx/CMakeLists.txt    # regenerated by CubeMX
├── docs/
│   ├── ARCHITECTURE.md               # this file
│   ├── BMS_LTC6811.md                # isoSPI BMS wire protocol
│   ├── CAN_MAP.md                    # vehicle / ACU CAN protocol
│   ├── COMMISSIONING.md              # bench / on-vehicle calibration
│   ├── HIL_STUB.md                   # AMS_BMS_HIL_STUB build flag (bench only)
│   └── HIL_TESTS.md                  # bench acceptance plan
├── tests/
│   └── unit/
│       ├── CMakeLists.txt            # host CMake (FetchContent Unity)
│       ├── mocks/                    # cmsis_os2 stub for host
│       ├── test_*.cpp                # 44 tests
│       └── unity_runner.cpp
└── .github/
    ├── roadmap.yaml                  # phase plan
    ├── scripts/render_roadmap.py
    └── workflows/
        ├── branch-issue.yml
        ├── build-tests.yml           # CI: cross-compile + host tests
        ├── close-on-dev-merge.yml
        └── roadmap.yml
```

---

## 13. Where to start reading

Different entry points by goal:

- **Adding a new CAN frame**:
  [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Adding a new CAN frame".
  Declare the ID in `ams_config.hpp`, write encode/decode + unit
  tests, update `CAN_MAP.md`.
- **Adding a new task**:
  [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Adding a new task".
  Touches `AMS.ioc` (`Tasks01` line), one `<task>.h` C-callable
  header, one `<task>.cpp` implementation, one `main.c` USER CODE
  trampoline.
- **Modifying MainTask / predicates / relays**: this is the
  `safety-critical` review bar. Read § 1 and § 5 of this file; then
  follow [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Modifying the
  safety supervisor".
- **Bringing up new hardware**:
  [`COMMISSIONING.md`](COMMISSIONING.md).
- **Working on the LTC6811 / isoSPI path**:
  [`BMS_LTC6811.md`](BMS_LTC6811.md) is the source of truth for the
  wire protocol, cell + temp mappings, PEC15, and balancing.
- **Setting up a bench rig with no real BMS chain**:
  [`HIL_STUB.md`](HIL_STUB.md) explains the `AMS_BMS_HIL_STUB` build
  flag and how to verify a flight image isn't stub-built.

---

## 14. Cross-reference to ECU

For team members coming from
[`isc-fs/IFS08-CE-ECU`](https://github.com/isc-fs/IFS08-CE-ECU):

| ECU pattern | AMS equivalent | Reason for delta |
|---|---|---|
| `g_in` + `g_inMutex` (single shared struct) | 3 services, no mutex (volatile, single-writer) | Distinct domains; lock-free single-writer contract |
| `ControlTask` 10 ms | `MainTask` 10 ms (safety + FSM 20 ms + telemetry 500 ms inline) | Refactor/19 collapsed to one timeline |
| `CanRxTask` 5 ms single bus | `AcuCanTask` (FDCAN1 only) | BMS moved off CAN onto isoSPI in v1.2.0 (#67–#74); `BmsRxTask` retired (#73). |
| `CanTxTask` drains queue | Merged into `AcuCanTask` / `BmsPollTask` | Fewer context switches |
| legacy AMS CAN polling on FDCAN2 | LTC6811-1 isoSPI broadcast (ADCV / RDCV[A-D] / WRCOMM / ADAX / RDAUXA / WRCFGA) via LTC6820 master on SPI1 | Hardware swap to BMS_LITE in v1.2.0; protocol details in [`BMS_LTC6811.md`](BMS_LTC6811.md). |
| (none) | `MainTask` realtime + watchdog discipline | AMS has direct safety output |
| `AppRuntime_*Step` factoring | Pure helpers (`fsm::step`, `safety::evaluate_fault`, `adc_to_mA`, `classify`, `telemetry::encode_*`) | Larger host-testable surface |
| C only | C++ for app, C for CubeMX-owned | Legacy AMS was already C++; classes simplify the service pattern |
