// SPDX-License-Identifier: proprietary
//
// Realtime-priority safety supervisor. Full contract in
// docs/ARCHITECTURE.md §3; every 10 ms it:
//
//   - snapshots the BMS / current / vehicle services,
//   - evaluates the safety predicates,
//   - on fault: opens all relays, drops AMS_OK, latches ERROR in the
//     RTC backup register,
//   - refreshes the IWDG -- on the fault path too, deliberately; see
//     the refresh site in safety_task.cpp for why.

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

    // True once latch_error_ has fired. Sticky for the rest of the run:
    // the loop keeps stepping and keeps refreshing the watchdog so
    // telemetry stays readable, and AMS_OK is held LOW while it is set.
    bool error_latched_ = false;

    // A fault must persist for N consecutive evaluations before it
    // latches: config::CellFaultConfirmTicks for the cell V/T ranges,
    // config::BmsStaleConfirmTicks for BmsStale. Both are pure and
    // unit-tested in safety_predicates.hpp.
    safety::CellFaultDebounce cell_debounce_{};
    safety::BmsStaleDebounce  bms_stale_debounce_{};

    // Open all relays, drop AMS_OK, set the backup-register magic, mark
    // this instance latched. Idempotent.
    void latch_error_() noexcept;
};

}  // namespace ams
