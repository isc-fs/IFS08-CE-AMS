// SPDX-License-Identifier: proprietary
//
// IWDG (independent watchdog) wrapper.
//
// LSI on STM32H733 is nominally 32 kHz with ±47% tolerance. We pick
// prescaler 32 and reload 100 -> ~100 ms timeout at nominal LSI, ~52 ms
// at LSI_min, ~190 ms at LSI_max. SafetyTask period is 10 ms and the
// refresh happens once per period on the clean path, so we have at
// least 5× margin even at the worst LSI corner. See docs/ARCHITECTURE.md
// §1 invariant 4 and §6 (SafetyTask).
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
    // (worst-case LSI fast corner). SafetyTask calls this once per
    // 10 ms cycle on the clean path; on the fault path it deliberately
    // does not, so the hardware reset fires within one watchdog window
    // and the GPIO init path re-opens the relays.
    static void refresh() noexcept { ams_watchdog_refresh(); }
};

}  // namespace ams
