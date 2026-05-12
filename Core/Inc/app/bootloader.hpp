// SPDX-License-Identifier: proprietary
//
// Application-side entry into the stm32-can-bootloader.
//
// The bootloader (sector 0) reads RTC->BKP0R on every reset. If the
// register holds BL_BOOT_REQ_MAGIC (0xB00710AD) the bootloader stays
// in BL mode awaiting CAN traffic; otherwise it validates the app
// metadata FLASHWORD and jumps to the app at 0x08020000.
//
// request_reboot() is the one-way path from a running application
// back into the bootloader. It:
//   1. Opens all relays (BSRR atomic, same as SafetyTask's fault path)
//   2. Drains the FDCAN TX FIFO briefly so any in-flight ACK frame
//      reaches the wire before the reset
//   3. Writes the magic into RTC->BKP0R
//   4. Calls NVIC_SystemReset -- never returns
//
// Trigger surface (today): a single dedicated CAN frame on FDCAN2.
// See docs/CAN_MAP.md and ams_config.hpp::kBlBootReqCanId.

#pragma once

#include "ams_config.hpp"
#include "can_frame.hpp"

#include <cstdint>
#include <cstring>

namespace ams {

class Bootloader {
public:
    // Open all relays, drain TX, write BL_BOOT_REQ_MAGIC to BKP0R,
    // NVIC_SystemReset. Never returns. HAL-dependent; lives in
    // bootloader.cpp and is exercised at HIL, not at unit level.
    [[noreturn]] static void request_reboot() noexcept;

    // True iff a CanFrame matches the boot-request trigger. Pure
    // logic, inlined in the header so the host unit-test build can
    // exercise it without pulling in HAL / FreeRTOS.
    [[nodiscard]] static bool matches_trigger(const CanFrame& f) noexcept {
        if (f.bus != static_cast<std::uint8_t>(CanBus::Bms)) return false;
        if (f.id  != config::kBlBootReqCanId)                return false;
        if (f.dlc != config::kBlBootReqDlc)                  return false;
        return std::memcmp(f.data,
                           config::kBlBootReqPayload,
                           config::kBlBootReqDlc) == 0;
    }
};

}  // namespace ams
