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
   [`Core/Inc/app/safety_predicates.hpp`](../Core/Inc/app/safety_predicates.hpp).
2. **Relays default to open** on any reset, fault, or uninitialised
   state. CubeMX-generated `MX_GPIO_Init` writes `PIN_RESET` to
   PD3 / PD4 / PD5 **before** configuring them as outputs.
3. **No single stuck task can prevent an AIR open.** `SafetyTask`
   runs at `osPriorityRealtime`, has direct GPIO write access via
   `ams::Relays`, and owns the watchdog refresh.
4. **The watchdog is fed only on `SafetyTask`'s clean path.** A
   stuck supervisor → IWDG timeout (≈100 ms) → hardware reset →
   relays default open.
5. **ERROR is latched across resets** via `RTC->BKP0R` (magic
   `0xA115EE51`). Cleared only by full backup-domain power loss.
6. **Shared sensor state has one writer.** Each domain has its own
   mutex; readers `snapshot()` under the lock and work off a copy.

---

## 2. Build model

CubeMX 6.16 generates a **CMake-based** project (not Eclipse-managed-
make). The boundary between generated and hand-written code:

```
CubeMX-owned (regenerated from AMS.ioc):
  ├─ Core/Src/{main.c, freertos.c, stm32h7xx_*.c}
  ├─ Drivers/STM32H7xx_HAL_Driver/
  ├─ Middlewares/Third_Party/FreeRTOS/
  └─ cmake/stm32cubemx/CMakeLists.txt

Hand-written (lives forever):
  ├─ Core/Inc/app/*.hpp + *.h
  ├─ Core/Src/app/*.cpp
  ├─ CMakeLists.txt           (top-level, edited once)
  └─ tests/unit/              (host CMake + Unity)
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

10 tasks total (incl. CMSIS-RTOS-mandated `defaultTask` and the
FreeRTOS timer-service daemon). All declared in
[`AMS.ioc`](../AMS.ioc) so CubeMX regenerates them every time.

| Task | Priority (enum / value) | Period | Stack (words) | Implementation |
|---|---|---|---|---|
| `App_InitTask` | High (40) | once | 512 | [`app_init_task.cpp`](../Core/Src/app/app_init_task.cpp) |
| `SafetyTask` | Realtime (48) | 10 ms | 512 | [`safety_task.cpp`](../Core/Src/app/safety_task.cpp) |
| `StateTask` | High (40) | 20 ms | 1024 | [`state_task.cpp`](../Core/Src/app/state_task.cpp) |
| `BmsRxTask` | AboveNormal (32) | event-driven | 512 | [`bms_rx_task.cpp`](../Core/Src/app/bms_rx_task.cpp) |
| `BmsPollTask` | Normal (24) | timer-driven | 256 | [`bms_poll_task.cpp`](../Core/Src/app/bms_poll_task.cpp) |
| `AcuCanTask` | AboveNormal (32) | mixed (RX event + 250/500 ms TX) | 512 | [`acu_can_task.cpp`](../Core/Src/app/acu_can_task.cpp) |
| `CurrentTask` | AboveNormal (32) | 50 ms | 256 | [`current_task.cpp`](../Core/Src/app/current_task.cpp) |
| `TelemetryTask` | Low (8) | 500 ms | 512 | [`telemetry_task.cpp`](../Core/Src/app/telemetry_task.cpp) |
| `defaultTask` | Low (8) | — | 128 | (placeholder, CMSIS) |
| Timer service | Normal | callback-driven | 256 | FreeRTOS daemon |

`SafetyTask` is the only Realtime-priority task in the system. Its
period of 10 ms is the hard contract: worst-case latency from
"sensor reads out of range" to "AIRs open" is bounded by
`producer_period + 10 ms + GPIO_write < 15 ms`.

---

## 4. Data flow

```mermaid
flowchart TD
  subgraph HW[STM32H733]
    FDCAN1[FDCAN1 RX/TX]
    FDCAN2[FDCAN2 RX/TX]
    ADC1[ADC1 ch2 PF11]
    GPIOD[GPIOD PD3/4/5 relays]
    GPIOE9[GPIOE PE9 SDC]
    TIM17[TIM17_CH1 PB9 fan]
    UART2[USART2 PA2/3]
    IWDG[IWDG1 ~100 ms]
  end

  subgraph ISR[ISRs]
    RX1[FDCAN1 RX-FIFO0 cb]
    RX2[FDCAN2 RX-FIFO0 cb]
  end

  subgraph Queues
    acu_rx[(acu_rx_queue 16)]
    bms_rx[(bms_rx_queue 32)]
    sa_evt[[safety_events]]
    bms_evt[[bms_events]]
  end

  subgraph Services["Services (mutex-protected)"]
    BmsSvc[BmsService<br/>bms_mutex]
    CurSvc[CurrentService<br/>current_mutex]
    VehSvc[VehicleService<br/>vehicle_mutex]
  end

  subgraph Tasks
    AcuT[AcuCanTask]
    BmsRxT[BmsRxTask]
    BmsPollT[BmsPollTask]
    CurT[CurrentTask]
    StateT[StateTask]
    SafeT[SafetyTask]
    TeleT[TelemetryTask]
  end

  FDCAN1 --> RX1 --> acu_rx --> AcuT --> VehSvc
  AcuT -.charger.-> CurSvc
  AcuT -- "0x12C / 0x450" --> FDCAN1

  FDCAN2 --> RX2 --> bms_rx --> BmsRxT --> BmsSvc
  BmsPollT -- "polls" --> FDCAN2

  ADC1 --> CurT --> CurSvc

  BmsSvc & CurSvc & VehSvc --> StateT
  BmsSvc & CurSvc & VehSvc --> SafeT
  BmsSvc & CurSvc & VehSvc --> TeleT

  StateT -- safety_events --> sa_evt --> SafeT
  StateT -- fan duty --> TIM17
  SafeT -- relays --> GPIOD
  SafeT -- refresh --> IWDG
  GPIOE9 --> SafeT
  GPIOE9 --> StateT
  TeleT --> UART2

  bms_evt -- pollV / pollT --> BmsPollT

  classDef hw    fill:#1e293b,stroke:#0f172a,color:#f8fafc
  classDef isr   fill:#fb923c,stroke:#9a3412,color:#1c1917
  classDef queue fill:#fde68a,stroke:#a16207,color:#1c1917
  classDef svc   fill:#60a5fa,stroke:#1e40af,color:#f8fafc
  classDef task  fill:#34d399,stroke:#065f46,color:#052e16
  classDef safe  fill:#ef4444,stroke:#7f1d1d,color:#fef2f2

  class FDCAN1,FDCAN2,ADC1,GPIOD,GPIOE9,TIM17,UART2,IWDG hw
  class RX1,RX2 isr
  class acu_rx,bms_rx,sa_evt,bms_evt queue
  class BmsSvc,CurSvc,VehSvc svc
  class AcuT,BmsRxT,BmsPollT,CurT,StateT,TeleT task
  class SafeT safe
```

**Legend** — hardware (slate) · ISRs (orange) · queues & event groups
(amber) · services (blue) · tasks (green) · `SafetyTask` (red, the
only realtime-priority task in the system).

Arrows are direction of value flow; mutex / queue producers always
go through the labelled primitive. The only direct GPIO writes
happen in `SafetyTask` (relays + watchdog) and `App_InitTask`
(one-shot driver bring-up).

---

## 5. Finite state machine

Six states. Transitions live in pure code at
[`state_machine.hpp`](../Core/Inc/app/state_machine.hpp) so they're
unit-tested in isolation; `StateTask` is the thin wrapper that
calls `fsm::step()` at 20 ms cadence and posts the returned
relay-action flags to `safety_events`.

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

Edge-transition relay actions:

| Transition | `safety_events` bits set |
|---|---|
| Start → Precharge | `kCloseAirN`, `kClosePrecharge` |
| Start → Charge | `kCloseAirN`, `kCloseAirP` |
| Precharge → Transition | `kCloseAirP`, `kOpenPrecharge` |
| any → Error | `kForceError`, `kOpenAirN`, `kOpenAirP`, `kOpenPrecharge` |

**Key properties:**

- **ERROR is sticky within a boot.** Even if the underlying fault
  clears, the FSM stays in `Error` until reset. `ErrorLatch::set()`
  fires on every `Error` entry so the next boot also starts in
  `Error` until backup-domain power is cycled.
- **Predicate-fault evaluation runs first** on every step, so any
  fault from any state preempts the normal transition.
- **AIR / Precharge actions are requests, not direct writes.** The
  FSM posts `kClose*` / `kOpen*` bits on `safety_events`; SafetyTask
  consumes them on the clean path (faulted path ignores them — safety
  wins).

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
  participant safe as SafetyTask
  participant state as StateTask
  participant other as Bms/Acu/Cur/Tele

  HW->>main: Reset_Handler then SystemInit
  main->>main: HAL_Init, SystemClock_Config
  main->>HW: MX_GPIO_Init (relays driven low)
  main->>HW: MX_ADC1/3, MX_FDCAN1/2, MX_TIM17, MX_USART2
  main->>HW: ams_watchdog_init (IWDG1 alive pre-scheduler)
  main->>main: osKernelInitialize
  main->>main: create mutexes, queues, tasks
  main->>main: ams_app_globals_init (event groups)
  main->>main: osKernelStart

  par scheduler running
    init->>init: ErrorLatch init (DBP unlock)
    init->>HW: HAL_FDCAN_ActivateNotification x2
    init->>HW: HAL_FDCAN_Start x2
    init->>init: osThreadExit
  and
    safe->>safe: ErrorLatch is_set
    loop every 10 ms
      safe->>safe: snapshot all services
      alt fault
        safe->>HW: Relays open_all + ErrorLatch set
        Note over safe,HW: NO IWDG refresh, HW reset follows
      else clean
        safe->>HW: service relay-action flags
        safe->>HW: HAL_IWDG_Refresh
      end
    end
  and
    state->>state: Fan init + initial duty
    loop every 20 ms
      state->>state: fsm step returns next + flags
      state->>safe: osEventFlagsSet (safety_events)
    end
  and other
    other-->>other: per-task period or event wait
  end
```

`App_InitTask` self-deletes once peripheral bring-up is done; its
TCB and stack return to the heap.

---

## 7. Shared state

Three domain services. Each is a singleton wrapping a struct
guarded by one mutex. Single-writer per service; many readers.

| Service | Mutex | Writer | Readers |
|---|---|---|---|
| [`BmsService`](../Core/Inc/app/bms_service.hpp) | `bms_mutexHandle` | `BmsRxTask` | Safety, State, AcuCan TX, Telemetry |
| [`CurrentService`](../Core/Inc/app/current_service.hpp) | `current_mutexHandle` | `CurrentTask` (ADC), `AcuCanTask` (charger flag) | Safety, State, AcuCan TX, BmsPoll, Telemetry |
| [`VehicleService`](../Core/Inc/app/vehicle_service.hpp) | `vehicle_mutexHandle` | `AcuCanTask` | Safety, State, Telemetry |

Mutex access goes through
[`ScopedMutex`](../Core/Inc/app/scoped_mutex.hpp) RAII. No raw
`osMutexAcquire` calls anywhere in app code.

---

## 8. Inter-task signalling

Three primitives, nothing else.

**Queues** ([`AMS.ioc`](../AMS.ioc) `FREERTOS.Queues01`):

| Queue | Depth | Item | Producer | Consumer |
|---|---:|---|---|---|
| `bms_rx_queue` | 32 | `CanFrame` | FDCAN2 RX ISR | BmsRxTask |
| `acu_rx_queue` | 16 | `CanFrame` | FDCAN1 RX ISR | AcuCanTask |
| `acu_tx_queue` | 16 | `CanFrame` | (reserved) | (reserved) |

`acu_tx_queue` is declared in the .ioc but currently unused —
`AcuCanTask` is the sole producer on FDCAN1 TX and calls HAL
directly. Kept declared so a future task could publish through
the queue without an .ioc round-trip.

**Event groups** (managed by
[`app_globals.cpp`](../Core/Src/app/app_globals.cpp) because
CubeMX 6.16 doesn't emit them from .ioc):

| Group | Bit | Set by | Cleared by |
|---|---|---|---|
| `safety_events` | `kForceError` | StateTask (Error entry) | SafetyTask (latch + reset only) |
|  | `kCloseAirN/AirP/Precharge` | StateTask | SafetyTask (consume on action) |
|  | `kOpenAirN/AirP/Precharge` | StateTask | SafetyTask (consume on action) |
| `bms_events` | `kPollVDue` | osTimer 250 ms | BmsPollTask |
|  | `kPollTDue` | osTimer 500 ms | BmsPollTask |

---

## 9. Memory budget (as built)

```
FLASH:    74 464 B / 1 MB    ( 7.10 %)
DTCMRAM:  44 408 B / 128 KB  (33.88 %)  -- stacks, TCBs, BSS,
                                          FreeRTOS heap_4
ITCMRAM:       0 B           -- unused
RAM_D1/D2/D3:  0 B           -- unused
```

FreeRTOS heap (`configTOTAL_HEAP_SIZE = 32 768`): ~14.7 KB used,
~18 KB free. Newlib reentrant structs account for ~1 KB. Three
load-bearing tasks (`App_InitTask`, `SafetyTask`, `StateTask`) were
originally planned as `Static` allocation; CubeMX UI workflow makes
that brittle, so they ship `Dynamic` for now. Heap has enough
headroom that this is fine; revisitable post-commissioning.

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
- Task entry points are `extern "C"` trampolines that call
  `Singleton::instance().run()`.
- `enum class` for state types (`fsm::State`, `CanBus`).
- `constexpr` for every threshold, ID, period — collected in
  [`ams_config.hpp`](../Core/Inc/app/ams_config.hpp). Anything
  needing on-vehicle tuning is tagged `COMMISSION` (see
  [`COMMISSIONING.md`](COMMISSIONING.md)).
- `ScopedMutex` always — no raw `osMutexAcquire`.
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
| Unit (single-step) | `test_bms_service`, `test_current_service`, `test_vehicle_service`, `test_safety_predicates`, `test_state_machine` | 39 tests: CAN decode, ADC scaling, mutex-protected snapshot/health, each safety predicate in isolation, each FSM transition |
| SIL (multi-step) | `test_sil_scenarios` | 5 scenarios: nominal startup, precharge timeout, BMS dropout, charger path, SDC sticky |

**44 / 44 PASS on every push** via
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
│   │       ├── ams_config.hpp       # ALL constexpr
│   │       ├── ams_events.hpp       # event-group bits
│   │       ├── app_globals.h
│   │       ├── app_init_task.h
│   │       ├── bms_poll_task.h
│   │       ├── bms_rx_task.h
│   │       ├── bms_service.hpp
│   │       ├── can_frame.{h,hpp}
│   │       ├── current_service.hpp
│   │       ├── current_task.h
│   │       ├── error_latch.hpp
│   │       ├── fan.hpp
│   │       ├── relay_driver.hpp
│   │       ├── safety_predicates.hpp
│   │       ├── safety_task.{h,hpp}
│   │       ├── scoped_mutex.hpp
│   │       ├── state_machine.hpp
│   │       ├── state_task.h
│   │       ├── telemetry_task.h
│   │       ├── vehicle_service.hpp
│   │       └── watchdog.{h,hpp}
│   ├── Src/
│   │   ├── main.c, freertos.c       # CubeMX-owned; USER CODE blocks
│   │   │                              call ams_*_task_run trampolines
│   │   ├── stm32h7xx_*.c, sysmem.c, syscalls.c
│   │   ├── stm32h7xx_hal_timebase_tim.c
│   │   └── app/                     # HAND-WRITTEN
│   │       ├── acu_can_task.cpp
│   │       ├── app_globals.cpp
│   │       ├── app_init_task.cpp
│   │       ├── bms_poll_task.cpp
│   │       ├── bms_rx_task.cpp
│   │       ├── bms_service.cpp
│   │       ├── can_isr.cpp           # HAL_FDCAN_RxFifo0Callback
│   │       ├── current_service.cpp
│   │       ├── current_task.cpp
│   │       ├── error_latch.cpp
│   │       ├── fan.cpp
│   │       ├── relay_driver.cpp
│   │       ├── safety_task.cpp
│   │       ├── state_task.cpp
│   │       ├── telemetry_task.cpp
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
│   ├── CAN_MAP.md                    # wire protocol
│   └── COMMISSIONING.md              # bench / on-vehicle calibration
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
- **Modifying SafetyTask / predicates / relays**: this is the
  `safety-critical` review bar. Read § 1 and § 5 of this file; then
  follow [`CONTRIBUTING.md`](../CONTRIBUTING.md) § "Modifying the
  safety supervisor".
- **Bringing up new hardware**:
  [`COMMISSIONING.md`](COMMISSIONING.md).

---

## 14. Cross-reference to ECU

For team members coming from
[`isc-fs/IFS08-CE-ECU`](https://github.com/isc-fs/IFS08-CE-ECU):

| ECU pattern | AMS equivalent | Reason for delta |
|---|---|---|
| `g_in` + `g_inMutex` (single shared struct) | 3 services, 3 mutexes | Distinct domains; finer locking |
| `ControlTask` 10 ms | `StateTask` 20 ms + `SafetyTask` 10 ms | Safety split out at higher priority |
| `CanRxTask` 5 ms single bus | `BmsRxTask` + `AcuCanTask` | Two buses, different semantics |
| `CanTxTask` drains queue | Merged into `AcuCanTask` / `BmsPollTask` | Fewer context switches |
| (none) | `SafetyTask` realtime + watchdog discipline | AMS has direct safety output |
| `AppRuntime_*Step` factoring | Pure helpers (`fsm::step`, `safety::evaluate_fault`, `adc_to_mA`, `classify`) | Larger host-testable surface |
| C only | C++ for app, C for CubeMX-owned | Legacy AMS was already C++; classes simplify mutex/service patterns |
