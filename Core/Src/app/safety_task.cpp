// SPDX-License-Identifier: proprietary
//
// MainTask — the collapsed SafetyTask + StateTask + TelemetryTask body
// from refactor/19 phase 3. Single 10 ms cadence loop that owns:
//
//   * Service snapshots (BMS / current / vehicle) once per iteration
//   * Safety predicate evaluation every 10 ms (kSafetyPeriodMs)
//   * FSM step every 20 ms (kStatePeriodMs)
//   * Telemetry emit every 500 ms (kTelemetryPeriodMs)
//   * IWDG refresh on every iteration (gated by the fault path)
//   * Fan PWM duty update on state transitions
//   * AMS_OK GPIO drive (in latched state we leave it where the FSM put
//     it; the FSM clears it via Run/Charge/Start logic on relay drive)
//
// The CubeMX-generated thread is still called "SafetyTask" in main.c +
// AMS.ioc. That cosmetic rename ships in a later phase to keep this
// commit free of CubeMX churn. The behavioural collapse is here.
//
// The relay-action ping-pong via osEventFlags between the old
// StateTask (producer) and SafetyTask (consumer) is gone: the FSM's
// output bitmask is consumed inline within this task.

#include "safety_task.hpp"

#include "ams_config.hpp"
#include "ams_events.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "error_latch.hpp"
#include "fan.hpp"
#include "relay_driver.hpp"
#include "safety_predicates.hpp"
#include "state_machine.hpp"
#include "telemetry_encoders.hpp"
#include "vehicle_service.hpp"
#include "watchdog.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;

// CubeMX-generated FreeRTOS task handle for BmsPollTask. NULL if
// osThreadNew silently failed at boot.
extern osThreadId_t BmsPollTaskHandle;

// #123 diagnostics maintained by other TUs.
extern volatile std::uint32_t g_bms_seed_count;     // bms_service.cpp
#if defined(AMS_BMS_HIL_STUB)
extern volatile std::uint8_t  g_app_init_progress;  // app_init_task.cpp (#123 iter 12)
#endif

// FreeRTOS heap diagnostic.
extern std::size_t xPortGetFreeHeapSize(void);
}

// FSM state mirror exposed for BmsPollTask / other read-only consumers.
// Updated on every transition.
extern "C" volatile std::uint8_t g_state_telemetry = 0;

// Telemetry TX failure counter, surfaced via a future diag frame.
extern "C" volatile std::uint32_t g_telemetry_tx_fail = 0;

namespace ams {

SafetyTask& SafetyTask::instance() noexcept {
    static SafetyTask kInstance;
    return kInstance;
}

void SafetyTask::latch_error_() noexcept {
    Relays::open_all();
    ErrorLatch::set();
    error_latched_ = true;
}

namespace {

// Per-state fan duty mirroring the legacy 40 % run / 75 % charge,
// off in transitional / error states.
constexpr std::uint8_t kFanDuty[6] = {
    /*Start*/      0,
    /*Precharge*/  0,
    /*Transition*/ 0,
    /*Run*/       40,
    /*Charge*/    75,
    /*Error*/      0,
};

bool send_telem(std::uint32_t id, const telemetry::Frame& payload) noexcept {
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(
               &hfdcan1, &tx,
               const_cast<std::uint8_t*>(payload.data())) == HAL_OK;
}

// Drive the relay actions encoded in the FSM's safety_flags bitmask.
// Inline replacement for the old osEventFlags(safety_eventsHandle, ...)
// ping-pong that used to hand the bits off to SafetyTask. Now both
// producer and consumer live in the same timeline so we just act.
void apply_relay_actions(std::uint32_t flags) noexcept {
    if (flags & events::safety::kCloseAirN)      Relays::close_air_negative();
    if (flags & events::safety::kCloseAirP)      Relays::close_air_positive();
    if (flags & events::safety::kClosePrecharge) Relays::close_precharge();
    if (flags & events::safety::kOpenAirN)       Relays::open_air_negative();
    if (flags & events::safety::kOpenAirP)       Relays::open_air_positive();
    if (flags & events::safety::kOpenPrecharge)  Relays::open_precharge();
}

}  // namespace

void SafetyTask::run() noexcept {
    // ---------------- One-shot init ----------------
    // ErrorLatch + Fan PWM bring-up. ErrorLatch::init is idempotent;
    // App_InitTask has almost certainly already called it.
    ErrorLatch::init();
    Fan::init();

    // Boot in ERROR if the previous run latched it. App_InitTask
    // clears the latch under -DAMS_BMS_HIL_STUB, so on the bench we
    // come up clean unless the latch was set this session.
    const bool boot_in_error = ErrorLatch::is_set();
    if (boot_in_error) {
        Relays::open_all();
        error_latched_ = true;
    }

    fsm::State state          = boot_in_error ? fsm::State::Error : fsm::State::Start;
    g_state_telemetry         = static_cast<std::uint8_t>(state);
    Fan::set_duty_pct(kFanDuty[static_cast<std::uint8_t>(state)]);

    std::uint32_t last_wake           = osKernelGetTickCount();
    std::uint32_t state_entry_tick    = last_wake;
    std::uint32_t last_state_tick     = last_wake;
    std::uint32_t last_telemetry_tick = last_wake;
    std::uint8_t  heartbeat           = 0;

    for (;;) {
        // ---------------- Wake at fixed 10 ms cadence ----------------
        last_wake += config::kSafetyPeriodMs;
        osDelayUntil(last_wake);

        const std::uint32_t now = osKernelGetTickCount();

        // ---------------- Snapshot inputs once per iteration ----------------
        const auto bms_snap = BmsService::instance().snapshot();
        const auto cur_snap = CurrentService::instance().snapshot();
        const auto veh_snap = VehicleService::instance().snapshot();

        // ---------------- Safety predicate (every 10 ms) ----------------
        const safety::Inputs pred_in = {
            bms_snap, cur_snap, veh_snap,
            /*force_error_set=*/false,   // legacy hook; no live setter
            now,
        };
        const bool fault = error_latched_ || safety::evaluate_fault(pred_in);

        if (fault) {
            if (!error_latched_) {
                latch_error_();
                state            = fsm::State::Error;
                state_entry_tick = now;
                g_state_telemetry = static_cast<std::uint8_t>(state);
                Fan::set_duty_pct(kFanDuty[static_cast<std::uint8_t>(state)]);
            }
            // Stay alive in the latched state: relays already open,
            // ErrorLatch persists across reset, so refreshing the
            // watchdog is safe. Lets the operator read telemetry
            // and explicitly reset via the bootloader path. See PR
            // #107 for the loop-bug this avoids.
            Watchdog::refresh();
        } else {
            // ---------------- FSM step (every 20 ms) ----------------
            if (now - last_state_tick >= config::kStatePeriodMs) {
                last_state_tick = now;

                const fsm::Inputs fsm_in = {
                    state, bms_snap, cur_snap, veh_snap,
                    /*force_error_set=*/false,
                    now, state_entry_tick,
                };
                const auto out = fsm::step(fsm_in);

                apply_relay_actions(out.safety_flags);

                if (out.next != state) {
                    state             = out.next;
                    state_entry_tick  = now;
                    g_state_telemetry = static_cast<std::uint8_t>(state);
                    Fan::set_duty_pct(kFanDuty[static_cast<std::uint8_t>(state)]);

                    // Persist ERROR across resets even when the FSM
                    // got there without a predicate fault (e.g.
                    // precharge timeout).
                    if (state == fsm::State::Error) {
                        ErrorLatch::set();
                        error_latched_ = true;
                    }
                }
            }
            Watchdog::refresh();
        }

        // ---------------- Telemetry (every 500 ms, regardless of state) ----------------
        if (now - last_telemetry_tick >= config::kTelemetryPeriodMs) {
            last_telemetry_tick = now;

            const std::uint8_t ams_ok =
                (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET)
                    ? 1u : 0u;

            // Three telemetry frames. The 0x4A3 diag frame from PR #142
            // was retired in #152 -- it never reached the wire despite
            // five iterations chasing the H7 FDCAN TX scheduler. The
            // diagnostic probes now ride the 0x4A2 encoder's bytes 3..5
            // under -DAMS_BMS_HIL_STUB (dropping dc_bus_V from 0x4A2 in
            // that build only; the bench injects dc_bus_V from the host
            // anyway). Flight builds keep the dc_bus_V layout unchanged.

            const std::uint8_t tx_fail_lo = static_cast<std::uint8_t>(
                g_telemetry_tx_fail & 0xFFu);

            // Diag values are always computed -- they're cheap reads and
            // encode_temps consumes them on the HIL_STUB build path.
            // Flight builds ignore the args via (void) inside the encoder.
            const std::uint8_t bms_task_state_byte = (BmsPollTaskHandle == nullptr)
                ? 0xFFu
                : static_cast<std::uint8_t>(
                    0xA0u | (static_cast<std::uint8_t>(
                                 osThreadGetState(BmsPollTaskHandle)) & 0x0Fu));
            const std::uint8_t  bms_seed_count_lo = static_cast<std::uint8_t>(
                g_bms_seed_count & 0xFFu);
            const std::size_t   free_heap         = xPortGetFreeHeapSize();
            const std::uint8_t  free_heap_kb      = (free_heap >= 0xFF00u)
                ? 0xFFu
                : static_cast<std::uint8_t>(free_heap >> 8);

            const auto frame_status = telemetry::encode_status(
                g_state_telemetry, ams_ok, bms_snap,
#if defined(AMS_BMS_HIL_STUB)
                /*app_init_progress=*/g_app_init_progress
#else
                /*app_init_progress=*/0u
#endif
            );
            const auto frame_pack   = telemetry::encode_pack(bms_snap, cur_snap);
            const auto frame_temps  = telemetry::encode_temps(
                bms_snap, veh_snap, heartbeat, tx_fail_lo,
                bms_task_state_byte, bms_seed_count_lo, free_heap_kb);

            if (!send_telem(config::kAmsTelemStatusId, frame_status)) ++g_telemetry_tx_fail;
            if (!send_telem(config::kAmsTelemPackId,   frame_pack))   ++g_telemetry_tx_fail;
            if (!send_telem(config::kAmsTelemTempsId,  frame_temps))  ++g_telemetry_tx_fail;

            ++heartbeat;  // 8-bit wraparound is intentional
        }
    }
}

}  // namespace ams

extern "C" void ams_safety_task_run(void *argument) {
    (void)argument;
    ams::SafetyTask::instance().run();
}
