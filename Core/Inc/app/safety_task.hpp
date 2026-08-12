// SPDX-License-Identifier: proprietary
//
// Realtime-priority safety supervisor. See docs/ARCHITECTURE.md §3 for
// the full contract; in short:
//
//   - Runs at 10 ms cadence.
//   - Reads the latest sensor snapshots (BMS, current, vehicle).
//   - Evaluates all safety predicates.
//   - On fault: opens all relays, drops AMS_OK, and latches ERROR in the
//     backup register so the fault survives a power cycle.
//   - Refreshes the IWDG on BOTH paths, clean and latched. Staying alive in
//     the latched state is deliberate: the relays are already open and the
//     latch survives a reset, so resetting would buy no safety and would cost
//     the operator the telemetry that says why the car stopped.
//
// The predicate set itself lives in safety_predicates.hpp; this file is the
// loop that feeds it, debounces its answer, and acts on it.

#pragma once

#include "app/safety_task.h"
#include "safety_predicates.hpp"

namespace ams {

class SafetyTask {
public:
    static SafetyTask& instance() noexcept;

    // Driven by the C trampoline ams_safety_task_run(). Does not return.
    [[noreturn]] void run() noexcept;

private:
    SafetyTask() = default;

    // True once latch_error_ has fired; once latched the loop continues
    // running (so other tasks see the FORCE_ERROR bit) but the watchdog
    // is never refreshed again, guaranteeing a hardware reset within
    // ~100 ms even if the FSM tries to be clever.
    bool error_latched_ = false;

    // Cell V/T range debounce. Requires a cell-range fault to
    // persist for config::CellFaultConfirmTicks consecutive evaluations
    // before it latches; pure + unit-tested in safety_predicates.hpp.
    safety::CellFaultDebounce cell_debounce_{};
    safety::BmsStaleDebounce  bms_stale_debounce_{};

    // Open all relays, set the backup-register magic, mark this
    // instance as latched. Idempotent.
    void latch_error_() noexcept;
};

}  // namespace ams
