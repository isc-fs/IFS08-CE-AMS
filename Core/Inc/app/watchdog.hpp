// SPDX-License-Identifier: proprietary
//
// IWDG (independent watchdog) wrapper.
//
// LSI on STM32H733 is nominally 32 Hz with ±47% tolerance. We pick
// prescaler 32 and reload 100 -> ~100 ms timeout at nominal LSI, ~52 ms
// at LSI_min, ~190 ms at LSI_max. SafetyTask period is 10 ms and the
// refresh happens once per period on the clean path, so we have at
// least 5× margin even at the worst LSI corner. See docs/ARCHITECTURE.md
// §1 invariant 4 and §3 (task architecture).
//
// Init runs in CubeMX-generated code (MX_IWDG1_Init() in main.c)
// before osKernelStart, so the watchdog is already alive when the
// scheduler boots and when this wrapper is first touched.

#pragma once

#include "app/watchdog.h"

namespace ams {

class Watchdog {
public:
    // Refresh the down-counter. Must be called at least every ~50 ms
    // (worst-case LSI fast corner). MainTask calls this once per 10 ms
    // cycle on BOTH paths -- clean and latched-fault. Refreshing in the
    // latched state is deliberate: the relays are already open and
    // ErrorLatch survives a reset, so a reset would buy no extra safety
    // and would cost the operator the telemetry that says why the car
    // stopped. Resetting out of a latched fault would also just boot
    // straight back into Error.
    //
    // So the IWDG covers exactly ONE failure: MainTask stopping. It does
    // NOT cover a MainTask that keeps looping while computing wrong
    // answers, and it does not supervise any other task -- nothing else
    // in the system refreshes it. See FMEA WATCHDOG-2.
    static void refresh() noexcept { ams_watchdog_refresh(); }
};

}  // namespace ams
