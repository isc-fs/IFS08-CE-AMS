# AMS architecture

Target: STM32H733ZGTx, FreeRTOS via CMSIS-RTOS v2, C++ (no exceptions, no
RTTI). Mirrors the structural conventions of `isc-fs/IFS08-CE-ECU` with
AMS-specific adaptations.

This document is the source of truth for the firmware architecture. Code
review rejects PRs that violate the invariants in [§1](#1-safety-invariants)
without a corresponding update here.

---

## 1. Safety invariants

Everything else in this document exists to enforce these:

1. **If the pack is unsafe, AIRs open within one safety period.** "Unsafe"
   means any of: cell V out of range, cell T out of range, pack current out
   of range, precharge timeout, SDC open, BMS comms lost, current-sensor
   stale, watchdog tripped.
2. **Relays default to open on any reset, fault, or uninitialised state.**
   The GPIO init path drives them inactive *before* the scheduler starts.
3. **No single stuck task can prevent an AIR open.** The safety supervisor
   runs independently of the FSM, has direct write access to the relay
   pins, and is the highest-priority task.
4. **The watchdog is fed only on the safety supervisor's clean path.** A
   stuck supervisor → IWDG timeout → hardware reset → relays default open.
5. **The ERROR state is latching across resets** via a backup register flag
   (RTC_BKP_DRx). Cleared only by full power-cycle or explicit operator
   action defined in the FSM spec.
6. **Shared sensor state is never read without a mutex** and never written
   from more than one task. Producers are single-writer; consumers
   snapshot.

---

## 2. Task architecture

8 tasks. All use CMSIS-RTOS v2 (`cmsis_os2.h`).

| # | Task | Priority | Period | Stack (words) | Role |
|---|---|---|---|---|---|
| 1 | `App_InitTask` | `osPriorityHigh` | once | 512 | Construct services, seed NVM state, hand off to FSM, self-delete. |
| 2 | `SafetyTask` | `osPriorityRealtime` | 10 ms | 512 | Read latest snapshots, evaluate safety predicates, drive relays, feed IWDG. Can force FSM → ERROR via event flag. |
| 3 | `StateTask` (FSM) | `osPriorityHigh` | 20 ms | 1024 | Run the 6-state machine. Requests relay transitions via SafetyTask; never writes relay GPIO directly. |
| 4 | `BmsRxTask` | `osPriorityAboveNormal` | event-driven | 512 | Drain BMS FDCAN queue, parse per-module frames, update `BmsState` under mutex. Detect comms timeout. |
| 5 | `BmsPollTask` | `osPriorityNormal` | 250 ms V / 500 ms T | 256 | Emit BMS request frames on schedule. |
| 6 | `AcuCanTask` | `osPriorityAboveNormal` | event-driven + 100 ms heartbeat | 512 | Drain accumulator FDCAN RX queue; publish pack status at 100 ms cadence. |
| 7 | `CurrentTask` | `osPriorityAboveNormal` | 50 ms | 256 | Trigger ADC, filter, update `CurrentState` under mutex, detect sensor-stuck/stale. |
| 8 | `TelemetryTask` | `osPriorityLow` | 500 ms | 512 | UART2 human-readable status. No safety role. |

**Why Realtime for SafetyTask.** Anything lower lets the FSM or a CAN burst
push safety past its deadline. Worst-case latency from "sensor reads
out-of-range" to "AIRs open" is bounded by
`producer_period + safety_period + GPIO_write_time ≈ < 15 ms`.

**Why SafetyTask is not the FSM.** The FSM is large and will change often.
SafetyTask is short (~200 LOC, single file, auditable). Separating them
means FSM bugs cannot jam a relay closed.

**Why BmsPollTask is separate from BmsRxTask.** Request cadence is fixed,
RX is event-driven. Splitting means an RX flood cannot delay a poll and a
delayed poll cannot starve RX.

Stacks are a conservative first pass — refine with
`uxTaskGetStackHighWaterMark` once running.

---

## 3. Shared data model

Three domain objects, each owned by one service, single-writer.

```cpp
struct BmsState {
    uint16_t cell_mV     [5][19];   // 5 modules × 19 cells
    int16_t  cell_tempC  [5][38];   // 5 modules × 38 sensors
    uint16_t pack_voltage_mV;
    uint16_t min_cell_mV, max_cell_mV;
    int16_t  min_tempC,   max_tempC, avg_tempC;
    uint32_t last_rx_tick[5];       // per-module freshness
    uint8_t  module_online_mask;    // bit per module, expect 0x1F
};

struct CurrentState {
    int32_t  current_mA;            // +discharge / −charge
    int32_t  filtered_mA;
    uint32_t last_update_tick;
    bool     charger_detected;
    bool     sensor_fault;
};

struct VehicleState {
    uint16_t dc_bus_V;              // from VCU 0x100
    uint8_t  start_button;          // from VCU 0x600
    uint32_t last_vcu_rx_tick;
};
```

Each wrapped in a service singleton owning its mutex:

```cpp
class BmsService {
public:
    static BmsService& instance();
    void  update_from_frame(const CanFrame&);   // BmsRxTask only
    BmsState snapshot() const;                  // any reader
    bool  is_healthy(uint32_t now_tick) const;
private:
    mutable osMutexId_t mutex_;
    BmsState state_;
};
```

`ScopedMutex` RAII wraps `osMutexAcquire/Release`. No raw mutex calls
anywhere in app code.

**Ownership map:**

| Object | Writer | Readers |
|---|---|---|
| `BmsState` | `BmsRxTask` | SafetyTask, StateTask, AcuCanTask, TelemetryTask |
| `CurrentState` | `CurrentTask` | SafetyTask, StateTask, AcuCanTask, TelemetryTask |
| `VehicleState` | `AcuCanTask` | StateTask, TelemetryTask |

SafetyTask is a pure reader except for the relay GPIO lines and the
`FORCE_ERROR` event flag.

---

## 4. Inter-task communication

Three primitives. Nothing else.

**Queues** (`osMessageQueue`):

| Queue | Depth | Item | Producer | Consumer |
|---|---|---|---|---|
| `bms_rx_queue` | 32 | `CanFrame` | FDCAN1 ISR | BmsRxTask |
| `acu_rx_queue` | 16 | `CanFrame` | FDCAN2 ISR | AcuCanTask |
| `acu_tx_queue` | 16 | `CanFrame` | StateTask, TelemetryTask | AcuCanTask |

(BMS TX bypasses a queue — BmsPollTask is sole producer and calls HAL
directly under a brief lock.)

**Event flags** (`osEventFlags`):

- `safety_events`: `FORCE_ERROR`, `PRECHARGE_START`, `PRECHARGE_OK`,
  `AIR_CLOSE_REQ`, `AIR_OPEN_REQ`. StateTask sets request bits, SafetyTask
  services them and sets result bits.
- `bms_events`: `POLL_V_DUE`, `POLL_T_DUE` set by two `osTimer` instances,
  cleared by BmsPollTask.

**Direct GPIO writes:** only inside `SafetyTask` and `App_InitTask`.

---

## 5. Finite state machine

6 states. Transitions are *requests* to SafetyTask, never direct writes.

```
                   +--------+
 power on -------> | START  |
                   +---+----+
                       | sdc_closed && bms_healthy && current_ok
                       v
                 +-----+------+
                 | PRECHARGE  |   AIR_CLOSE_REQ(AIR-, Precharge)
                 +-----+------+
                       | dc_bus >= 0.95 × pack_V
                       v
                 +-----+------+
                 | TRANSITION |   close AIR+, open Precharge
                 +-----+------+
                       | hold 100 ms, dc_bus steady
                       v
                 +-----+------+                +---------+
                 |    RUN     | <- charger -> | CHARGE  |
                 +-----+------+                +----+----+
                       |                            |
                       +------------ any fault -----+
                                     |
                                     v
                               +-----+-----+
                               |   ERROR   |  (latched, NVM flag set)
                               +-----------+
```

**Precharge timeout.** StateTask arms an `osTimer` on entry (1500 ms).
Timer callback sets `FORCE_ERROR`. SafetyTask sees it and opens all AIRs.

**CHARGE entry.** Either `charger_detected` from CurrentState or charger
CAN frame (`0x18FF50E7`, see `CAN_MAP.md`). Exit when charger removed and
current near zero for 500 ms.

**ERROR entry.** Writes NVM flag, posts telemetry ERROR frame, then spins
reading. Task stays alive only if SafetyTask confirms relays open;
otherwise `NVIC_SystemReset()`.

---

## 6. SafetyTask detail

```cpp
void SafetyTask::step() {
    const auto bms  = BmsService::instance().snapshot();
    const auto cur  = CurrentService::instance().snapshot();
    const uint32_t now = osKernelGetTickCount();
    const uint32_t evt = osEventFlagsWait(safety_events_, kAll,
                                          osFlagsWaitAny, 0);

    bool fault =
        bms.min_cell_mV  < kCellUVmV                ||
        bms.max_cell_mV  > kCellOVmV                ||
        bms.max_tempC    > kCellOTC                 ||
        bms.min_tempC    < kCellUTC                 ||
        std::abs(cur.filtered_mA) > kImaxMa         ||
        (now - cur.last_update_tick) > kIStaleMs    ||
        bms.module_online_mask != kAllModulesMask   ||
        (evt & FORCE_ERROR)                         ||
        !HAL_GPIO_ReadPin(SDC_GPIO_Port, SDC_Pin);

    if (fault) {
        Relays::open_all();
        error_latch_.set();          // backup register write
        return;                      // IWDG not refreshed → HW reset
    }

    if (evt & AIR_CLOSE_REQ)  Relays::service_close_request();
    if (evt & AIR_OPEN_REQ)   Relays::service_open_request();

    HAL_IWDG_Refresh(&hiwdg);        // only on clean path
}
```

Thresholds (`kCellUVmV`, etc.) are compile-time constants in
`ams_config.hpp`. Not runtime-tunable.

---

## 7. ISR policy

| ISR | Action |
|---|---|
| FDCAN1 RX FIFO0 | `HAL_FDCAN_GetRxMessage` → pack `CanFrame` → `osMessageQueuePut(bms_rx_queue, …, 0)` |
| FDCAN2 RX FIFO0 | Same → `acu_rx_queue` |
| ADC EOC | If DMA: none. Else `osThreadFlagsSet(currentTaskHandle, …)`. Preferred: DMA + timer trigger, task polls. |

No parsing, no `printf`, no mutex acquisition in ISRs. Queue-full is
telemetered but never blocks.

SysTick is FreeRTOS-owned. HAL tick must be backed by a separate basic
timer (TIM6 or TIM7) — CubeMX configures this when FreeRTOS is enabled.

---

## 8. Error handling and recovery

| Fault | Detection | Response |
|---|---|---|
| BMS module silent > 1500 ms | `now − last_rx_tick[i]` | FORCE_ERROR |
| Current sensor stale > 200 ms | `now − last_update_tick` | FORCE_ERROR |
| Precharge incomplete > 1500 ms | StateTask timer | FORCE_ERROR |
| CAN TX queue full | `osMessageQueuePut` timeout | Drop + telemeter count |
| Mutex timeout > 10 ms | `ScopedMutex` error | FORCE_ERROR |
| Stack overflow | FreeRTOS hook | `NVIC_SystemReset()` |
| Malloc failed | FreeRTOS hook | `NVIC_SystemReset()` |
| IWDG timeout | HW | HW reset → relays open on boot |

`App_InitTask` reads the backup register `AMS_ERROR_LATCHED` before any
peripheral comes up. If set, FSM starts in ERROR, never START.

---

## 9. Power-up sequence

1. `Reset_Handler` → `SystemInit` → `main()`
2. `HAL_Init`, `SystemClock_Config`
3. `MX_GPIO_Init` — **relays driven inactive here, before anything else**
4. Read backup register; if `AMS_ERROR_LATCHED` is set, mark initial FSM
   state as ERROR
5. `MX_ADC*_Init`, `MX_FDCAN1_Init`, `MX_FDCAN2_Init`, `MX_TIM17_Init`,
   `MX_USART2_UART_Init`, `MX_IWDG_Init` (timeout ≥ 100 ms)
6. Create queues, event flags, mutexes, timers
7. Create tasks, with `App_InitTask` at highest priority for first-run
8. `osKernelStart()`
9. `App_InitTask` constructs services, enables FDCAN RX interrupts, starts
   ADC, posts initial FSM state, self-deletes

No task other than `App_InitTask` runs business logic before init
completes.

---

## 10. Memory and heap

- FreeRTOS heap_4, `configTOTAL_HEAP_SIZE` ≈ 16 KB (refine post-bring-up).
- `configSUPPORT_STATIC_ALLOCATION = 1`. All tasks and queues created with
  statically-allocated control blocks and stacks where the CMSIS-RTOS v2
  API exposes the attrs. Auditable memory layout.
- No runtime allocation after `osKernelStart`.
- No STL containers with dynamic allocation. `std::array`, `std::span`,
  `std::optional`, `<algorithm>` are fine.

---

## 11. C++ rules

- `-fno-exceptions -fno-rtti -fno-threadsafe-statics`
- One service per `.hpp/.cpp` pair, named `*_service` or `*_task`
- Service singletons via `static` local in `instance()`; explicit `init()`
  called from `App_InitTask` to control construction order
- Task entry points are `extern "C"` trampolines calling
  `Service::instance().run()`
- `enum class State : uint8_t` for FSM
- `constexpr` for every threshold, ID, period, gathered in
  `Core/Inc/app/ams_config.hpp`
- `ScopedMutex` RAII
- No virtual methods unless polymorphism is genuinely needed
- No raw `new` / `delete` outside FreeRTOS internals

---

## 12. File layout

```
IFS08-CE-AMS/
├── Core/
│   ├── Inc/
│   │   ├── main.h, FreeRTOSConfig.h, stm32h7xx_hal_conf.h, stm32h7xx_it.h
│   │   └── app/
│   │       ├── ams_config.hpp           # all constexpr thresholds / IDs
│   │       ├── scoped_mutex.hpp
│   │       ├── can_frame.hpp
│   │       ├── bms_service.hpp
│   │       ├── current_service.hpp
│   │       ├── vehicle_service.hpp
│   │       ├── relay_driver.hpp
│   │       ├── safety_task.hpp
│   │       ├── state_task.hpp
│   │       └── telemetry_task.hpp
│   ├── Src/
│   │   ├── main.cpp, freertos.cpp, stm32h7xx_it.c, ...
│   │   └── app/
│   │       ├── bms_service.cpp
│   │       ├── current_service.cpp
│   │       ├── vehicle_service.cpp
│   │       ├── relay_driver.cpp
│   │       ├── safety_task.cpp
│   │       ├── state_task.cpp
│   │       ├── bms_poll_task.cpp
│   │       ├── bms_rx_task.cpp
│   │       ├── acu_can_task.cpp
│   │       ├── current_task.cpp
│   │       └── telemetry_task.cpp
│   └── Startup/startup_stm32h733zgtx.s
├── Drivers/                              # STM32H7 HAL + CMSIS-Device
├── Middlewares/Third_Party/FreeRTOS/
├── tests/{unit,sil}/
├── docs/
│   ├── ARCHITECTURE.md                   # this file
│   └── CAN_MAP.md
├── CMakeLists.txt                        # for tests only
├── AMS.ioc                               # CubeMX with FreeRTOS enabled
├── STM32H733ZGTX_FLASH.ld / _RAM.ld
└── README.md
```

---

## 13. Build

- Production: STM32CubeIDE-generated makefile, `arm-none-eabi-g++` for
  `.cpp`. CXXFLAGS include `-fno-exceptions -fno-rtti
  -fno-threadsafe-statics -Wall -Wextra -Werror`.
- Tests: CMake host-target. Real services linked against mocked HAL +
  mocked `cmsis_os2`, following the ECU SIL pattern.
- CI: build tests on every PR. HIL build wired in once a HIL rig exists.

---

## 14. Open questions

1. Precharge target voltage source — local pack measurement vs VCU-reported
   `DC_BUS` (currently 0x100).
2. Operator ERROR clear path — reset-only, or charge-button long-press, or
   CAN command?
3. Fan PWM policy — stepwise (current behaviour, 40/75%) vs
   temperature-feedback PID. Phase 5 calibration.
4. FreeRTOS heap final sizing — measure after Phase 3.
5. Watchdog window — 100 ms IWDG with safety period 10 ms gives 10× margin.
   Confirm LSI tolerance against the worst-case schedule.

---

## 15. Cross-reference to ECU

| ECU pattern | AMS equivalent | Reason for delta |
|---|---|---|
| `g_in` + `g_inMutex` | 3 services, 3 mutexes | Distinct domains; finer locking |
| `ControlTask` 10 ms | `StateTask` 20 ms | FSM is slower-changing, no torque math |
| `CanRxTask` 5 ms single bus | `BmsRxTask` + `AcuCanTask` | Two buses with different semantics |
| `CanTxTask` drains queue | Merged into `AcuCanTask` / `BmsPollTask` | Fewer context switches |
| (none) | `SafetyTask` Realtime | AMS has direct safety output |
| `AppRuntime_*Step` factoring | `Service::run_step()` per service | Preserve SIL testability |
