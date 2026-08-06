// SPDX-License-Identifier: proprietary
//
// Latched-ERROR flag, backed by RTC backup register RTC_BKP_DR1
// (config::BkpErrorReg = 1; writable once backup-domain access is
// unlocked via HAL_PWR_EnableBkUpAccess).
//
// The backup domain is reset only when it loses power, NOT by a CPU
// reset -- so the flag is preserved across every WARM reset:
//
//   - software resets (NVIC_SystemReset, incl. the 0x002 BL reboot)
//   - watchdog resets (IWDG, WWDG)
//   - external reset-pin / debugger-triggered resets
//
// It is cleared by:
//
//   - the first cold boot (BKP_DR contents are undefined until we
//     write -- anything that is NOT the magic value reads as "not
//     latched")
//   - any loss of the backup-domain rail.
//
// IMPORTANT (no VBAT): this carrier has no VBAT source, so the
// backup domain is powered only from VDD. A full LV power-cycle (or a
// brown-out that collapses VDD) therefore CLEARS this flag. That is
// accepted by design: a latch must outlive a warm reset (so a fault
// that watchdog-resets the chip comes back up in Error), but a
// deliberate power-cycle is treated as the manual reset, and a
// persistent fault re-latches on the next post-grace evaluation anyway.
// Do NOT assume this flag survives a power-off. See docs/ARCHITECTURE.md
// §1 invariant 5 and §9 power-up.

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
