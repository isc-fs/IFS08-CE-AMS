// SPDX-License-Identifier: proprietary
//
// Realtime-priority safety supervisor. See docs/ARCHITECTURE.md §6 for
// the full contract; in short:
//
//   - Runs at 10 ms cadence.
//   - Reads the latest sensor snapshots (BMS, current, vehicle).
//   - Evaluates all safety predicates.
//   - On fault: opens all relays, latches ERROR in the backup register,
//     and DOES NOT refresh the watchdog -> HW reset within ~100 ms.
//   - On clean path: refreshes the IWDG.
//
// This skeleton is the framing of the loop. The full predicate set
// (cell V/T ranges, BMS module-online mask, current freshness, SDC
// state) lands in feat/7-safety-predicates once the corresponding
// services exist.

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

    // Cell V/T range debounce (#279). Requires a cell-range fault to
    // persist for config::CellFaultConfirmTicks consecutive evaluations
    // before it latches; pure + unit-tested in safety_predicates.hpp.
    safety::CellFaultDebounce cell_debounce_{};

    // Open all relays, set the backup-register magic, mark this
    // instance as latched. Idempotent.
    void latch_error_() noexcept;
};

}  // namespace ams
