// SPDX-License-Identifier: proprietary
//
// Latched-ERROR flag that survives any reset short of full power-off
// of the backup domain (i.e. VBAT + VDD both removed).
//
// Backed by RTC backup register RTC_BKP_DR0 (writable when backup-
// domain access is unlocked via HAL_PWR_EnableBkUpAccess). The
// register is preserved across:
//
//   - software resets (NVIC_SystemReset)
//   - watchdog resets (IWDG, WWDG)
//   - brownout resets while VBAT holds the backup domain alive
//   - external debugger-triggered resets
//
// It is cleared by:
//
//   - the very first cold boot (BKP_DR contents are undefined until we
//     write -- App_InitTask treats anything that is NOT the magic value
//     as "not latched")
//   - removing both VDD and VBAT (full power cycle of the backup rail)
//
// See docs/ARCHITECTURE.md §1 invariant 5 and §9 power-up.

#pragma once

namespace ams {

class ErrorLatch {
public:
    // Unlocks backup-domain write access. Must be called once, early
    // in boot (App_InitTask), before any is_set()/set()/clear() call.
    // Idempotent.
    static void init() noexcept;

    [[nodiscard]] static bool is_set()   noexcept;
    static               void set()      noexcept;
    static               void clear()    noexcept;
};

}  // namespace ams
