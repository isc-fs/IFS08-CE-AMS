// SPDX-License-Identifier: proprietary
//
// MainTask — the collapsed SafetyTask + StateTask + TelemetryTask body
// from refactor/19 phase 3. Single 10 ms cadence loop that owns:
//
//   * Service snapshots (BMS / current / vehicle) once per iteration
//   * Safety predicate evaluation every 10 ms (SafetyPeriodMs)
//   * FSM step every 20 ms (StatePeriodMs)
//   * Telemetry emit every 500 ms (TelemetryPeriodMs)
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
#if defined(AMS_BMS_HIL_STUB)
extern volatile std::uint8_t  g_app_init_progress;  // app_init_task.cpp (#123 iter 12)
// ACU RX dispatch liveness counter. Surfaced in 0x4A2[4] -- ticks on
// any matched ACU frame.
extern volatile std::uint32_t g_acu_rx_total;
#endif
}

#if defined(AMS_BMS_HIL_STUB)
// HIL-only fault-injection hook for Block B safety-predicate tests
// (e.g. B-024 current overlimit, B-025 sensor-fault paths). The bench
// flips this via GDB / SWD / a CAN backdoor; SafetyTask passes it to
// safety::evaluate_fault as `force_error_set`, which short-circuits
// to true on the next 10 ms tick -> latched Error.
//
// Gated under HIL_STUB so flight builds have NO writable backdoor
// into the safety supervisor.
extern "C" volatile bool g_force_error_request = false;
#endif

// FSM state mirror exposed for BmsPollTask / other read-only consumers.
// Updated on every transition.
extern "C" volatile std::uint8_t g_state_telemetry = 0;

// Mode locked at Start->Precharge, mirrored here so the pit-diag stream
// (#247) in AcuCanTask can surface it without taking a snapshot mutex.
// Same 8-bit volatile read pattern as g_state_telemetry. Values match
// the fsm::Mode enum (0=Undecided, 1=Car, 2=Charger).
extern "C" volatile std::uint8_t g_mode_locked_telemetry = 0;

// Telemetry TX failure counter, surfaced via a future diag frame.
extern "C" volatile std::uint32_t g_telemetry_tx_fail = 0;

namespace ams {

SafetyTask& SafetyTask::instance() noexcept {
    static SafetyTask Instance;
    return Instance;
}

void SafetyTask::latch_error_() noexcept {
    Relays::open_all();
    ErrorLatch::set();
    error_latched_ = true;
}

namespace {

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
    if (flags & events::safety::CloseAirN)      Relays::close_air_negative();
    if (flags & events::safety::CloseAirP)      Relays::close_air_positive();
    if (flags & events::safety::ClosePrecharge) Relays::close_precharge();
    if (flags & events::safety::OpenAirN)       Relays::open_air_negative();
    if (flags & events::safety::OpenAirP)       Relays::open_air_positive();
    if (flags & events::safety::OpenPrecharge)  Relays::open_precharge();
}

}  // namespace

void SafetyTask::run() noexcept {
    // ---------------- One-shot init ----------------
    // ErrorLatch bring-up. ErrorLatch::init is idempotent;
    // App_InitTask has almost certainly already called it. Fan PWM
    // retired in fix/48 (fan is wired permanently on; see main.c).
    ErrorLatch::init();

    // Boot in ERROR if the previous run latched it. App_InitTask
    // clears the latch under -DAMS_BMS_HIL_STUB, so on the bench we
    // come up clean unless the latch was set this session.
    const bool boot_in_error = ErrorLatch::is_set();
    if (boot_in_error) {
        Relays::open_all();
        error_latched_ = true;
    }

    fsm::State state          = boot_in_error ? fsm::State::Error : fsm::State::Start;
    fsm::Mode  mode_locked    = fsm::Mode::Undecided;
    g_state_telemetry         = static_cast<std::uint8_t>(state);

    std::uint32_t last_wake           = osKernelGetTickCount();
    std::uint32_t state_entry_tick    = last_wake;
    std::uint32_t last_state_tick     = last_wake;
    std::uint32_t last_telemetry_tick = last_wake;
    std::uint8_t  heartbeat           = 0;

    for (;;) {
        // ---------------- Wake at fixed 10 ms cadence ----------------
        last_wake += config::SafetyPeriodMs;
        osDelayUntil(last_wake);

        const std::uint32_t now = osKernelGetTickCount();

        // ---------------- Snapshot inputs once per iteration ----------------
        const auto bms_snap = BmsService::instance().snapshot();
        const auto cur_snap = CurrentService::instance().snapshot();
        const auto veh_snap = VehicleService::instance().snapshot();

        // Operator-facing GPIO inputs: TSMS (side-of-car external
        // switch, PF9) and DASH_CHG (cockpit dashboard / charger
        // button, PF10). Both active-high, external pull-down on the
        // carrier. Polled every 10 ms; the 20 ms FSM step consumes
        // the latest reading.
        const bool tsms    = HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)       == GPIO_PIN_SET;
        const bool dash_chg = HAL_GPIO_ReadPin(DASH_CHG_GPIO_Port, DASH_CHG_Pin) == GPIO_PIN_SET;

        // ---------------- Safety predicate (every 10 ms) ----------------
#if defined(AMS_BMS_HIL_STUB)
        const bool force_error_set = g_force_error_request;
#else
        constexpr bool force_error_set = false;  // no flight-side setter
#endif
        const safety::Inputs pred_in = {
            bms_snap, cur_snap, veh_snap,
            force_error_set,
            now,
        };
        const bool fault = error_latched_ || safety::evaluate_fault(pred_in);

        if (fault) {
            if (!error_latched_) {
                latch_error_();
                state            = fsm::State::Error;
                state_entry_tick = now;
                g_state_telemetry = static_cast<std::uint8_t>(state);
            }
            // Stay alive in the latched state: relays already open,
            // ErrorLatch persists across reset, so refreshing the
            // watchdog is safe. Lets the operator read telemetry
            // and explicitly reset via the bootloader path. See PR
            // #107 for the loop-bug this avoids.
            Watchdog::refresh();
        } else {
            // ---------------- FSM step (every 20 ms) ----------------
            if (now - last_state_tick >= config::StatePeriodMs) {
                last_state_tick = now;

                // Lock the Car-vs-Charger mode at the EXACT iteration
                // that's about to take us out of Start. Captured here
                // (before fsm::step) so the FSM body sees a
                // self-consistent input snapshot. Read VCU 0x100
                // freshness via the snapshot already taken this tick.
                if (state == fsm::State::Start &&
                    mode_locked == fsm::Mode::Undecided &&
                    tsms && dash_chg) {
                    const bool vcu_fresh =
                        veh_snap.last_dc_bus_tick != 0u &&
                        (now - veh_snap.last_dc_bus_tick) <=
                            config::VcuFreshMs;
                    mode_locked = vcu_fresh ? fsm::Mode::Car
                                            : fsm::Mode::Charger;
                    g_mode_locked_telemetry =
                        static_cast<std::uint8_t>(mode_locked);
                }

                const fsm::Inputs fsm_in = {
                    state, bms_snap, cur_snap, veh_snap,
                    tsms, dash_chg, mode_locked,
                    force_error_set,
                    now, state_entry_tick,
                };
                const auto out = fsm::step(fsm_in);

                apply_relay_actions(out.safety_flags);

                if (out.next != state) {
                    state             = out.next;
                    state_entry_tick  = now;
                    g_state_telemetry = static_cast<std::uint8_t>(state);

                    // Persist ERROR across resets even when the FSM
                    // got there without a predicate fault (e.g.
                    // precharge timeout, or TSMS/DASH_CHG dropped
                    // mid-Run).
                    if (state == fsm::State::Error) {
                        ErrorLatch::set();
                        error_latched_ = true;
                    }
                }
            }
            Watchdog::refresh();
        }

        // ---------------- Telemetry (every 500 ms, regardless of state) ----------------
        if (now - last_telemetry_tick >= config::TelemetryPeriodMs) {
            last_telemetry_tick = now;

            const std::uint8_t ams_ok =
                (HAL_GPIO_ReadPin(AMS_OK_GPIO_Port, AMS_OK_Pin) == GPIO_PIN_SET)
                    ? 1u : 0u;

            // Three telemetry frames. Under AMS_BMS_HIL_STUB the
            // 0x4A2 encoder repurposes bytes 3..5 as diagnostic probes
            // (dropping dc_bus_V on this build only; the bench injects
            // dc_bus_V from the host). Flight builds keep the
            // standard 0x4A2 layout.

            const std::uint8_t tx_fail_lo = static_cast<std::uint8_t>(
                g_telemetry_tx_fail & 0xFFu);

#if defined(AMS_BMS_HIL_STUB)
            // Bench-only diag values riding 0x4A2[3..5]. Skipped in
            // flight so we don't burn an osThreadGetState() call per
            // 500 ms tick.
            //
            // 0x4A2[3] -- BmsPollTask scheduling state (BmsPollTaskHandle)
            // 0x4A2[4] -- g_acu_rx_total low byte  (any ACU RX = ticking)
            // 0x4A2[5] -- TSMS + DASH_CHG pin readback + mode_locked.
            //             High nibble 0x8 is a "this byte is live"
            //             sentinel so 0x00 from older binaries stands
            //             out. Bits: [7]=1, [3..2]=mode_locked
            //             (00=Undecided, 01=Car, 10=Charger),
            //             [1]=TSMS, [0]=DASH_CHG.
            const std::uint8_t bms_task_state_byte = (BmsPollTaskHandle == nullptr)
                ? 0xFFu
                : static_cast<std::uint8_t>(
                    0xA0u | (static_cast<std::uint8_t>(
                                 osThreadGetState(BmsPollTaskHandle)) & 0x0Fu));
            const std::uint8_t acu_rx_total_lo = static_cast<std::uint8_t>(
                g_acu_rx_total & 0xFFu);
            const std::uint8_t tsms_dash_chg_byte = static_cast<std::uint8_t>(
                0x80u |
                (static_cast<std::uint8_t>(mode_locked) << 2) |
                (tsms    ? 0x02u : 0u) |
                (dash_chg ? 0x01u : 0u));

            const auto frame_status = telemetry::encode_status(
                g_state_telemetry, ams_ok, bms_snap,
                /*app_init_progress=*/g_app_init_progress);
            const auto frame_pack   = telemetry::encode_pack(bms_snap, cur_snap);
            const auto frame_temps  = telemetry::encode_temps(
                bms_snap, veh_snap, heartbeat, tx_fail_lo,
                bms_task_state_byte, acu_rx_total_lo, tsms_dash_chg_byte);
#else
            const auto frame_status = telemetry::encode_status(
                g_state_telemetry, ams_ok, bms_snap);
            const auto frame_pack   = telemetry::encode_pack(bms_snap, cur_snap);
            const auto frame_temps  = telemetry::encode_temps(
                bms_snap, veh_snap, heartbeat, tx_fail_lo);
#endif

            if (!send_telem(config::AmsTelemStatusId, frame_status)) ++g_telemetry_tx_fail;
            if (!send_telem(config::AmsTelemPackId,   frame_pack))   ++g_telemetry_tx_fail;
            if (!send_telem(config::AmsTelemTempsId,  frame_temps))  ++g_telemetry_tx_fail;

            ++heartbeat;  // 8-bit wraparound is intentional
        }
    }
}

}  // namespace ams

extern "C" void ams_safety_task_run(void *argument) {
    (void)argument;
    ams::SafetyTask::instance().run();
}
