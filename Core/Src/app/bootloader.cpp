// SPDX-License-Identifier: proprietary

#include "bootloader.hpp"

#include "ams_config.hpp"
#include "relay_driver.hpp"

#include "cmsis_os2.h"
#include "main.h"

namespace ams {

void Bootloader::request_reboot(config::JumpReason reason) noexcept {
    // 1) Pack-safe state first. The reset itself runs MX_GPIO_Init
    // which will drive PD3/4/5 to PIN_RESET, but that path is non-
    // deterministic (~ms gap). Doing it here closes the gap.
    Relays::open_all();

    // 2) Let any in-flight TX (ACK to whoever sent the trigger, last
    // telemetry frame) clear the controller. 10 ms is plenty at 500
    // kbps -- a full 8-byte frame is < 200 us on the wire.
    osDelay(10);

    // 3) Stamp the jump reason and hand the bootloader its magic in
    // the backup-domain words it checks on every reset. Reason
    // first, magic last -- if a reset interrupts us mid-sequence,
    // we'd rather end up in app-mode with a stale reason than in
    // BL-mode with no reason at all (post-mortem can still proceed).
    HAL_PWR_EnableBkUpAccess();
    (&RTC->BKP0R)[config::BkpJumpReasonReg] =
        static_cast<std::uint32_t>(reason);
    (&RTC->BKP0R)[config::BlBootReqReg]     = config::BlBootReqMagic;

    // 4) Trigger HW reset. NVIC_SystemReset issues a DSB + writes
    // AIRCR, then spins; the CPU resets within a few cycles. The
    // bootloader's reset handler reads BKP0R, sees the magic, clears
    // it (one-shot), and stays in BL mode.
    NVIC_SystemReset();

    // Unreachable -- silences [[noreturn]] warnings on older
    // toolchains that don't recognise NVIC_SystemReset as no-return.
    for (;;) {}
}

}  // namespace ams
