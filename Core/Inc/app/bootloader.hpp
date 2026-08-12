// SPDX-License-Identifier: proprietary
//
// Application-side entry into the stm32-can-bootloader.
//
// The bootloader (sector 0) reads RTC->BKP0R on every reset. If it holds
// BL_BOOT_REQ_MAGIC (0xB00710AD) the bootloader stays in BL mode waiting for
// CAN traffic; otherwise it validates the app metadata FLASHWORD and jumps to
// the app at 0x08020000.
//
// request_reboot() is the one-way path from a running application back into
// the bootloader:
//   1. Open all relays (BSRR atomic, as in SafetyTask's fault path)
//   2. Drain the FDCAN TX FIFO briefly so an in-flight ACK frame reaches the
//      wire before the reset
//   3. Write the magic into RTC->BKP0R
//   4. NVIC_SystemReset -- never returns
//
// Trigger surface: one dedicated CAN frame on FDCAN1, the accumulator bus.
// That is the bus MingoCAN already uses for VCU telemetry, so the flashing
// tool needs a single connection. See docs/CAN_MAP.md and
// ams_config.hpp::BlBootReqCanId.

#pragma once

#include "ams_config.hpp"
#include "can_frame.hpp"
#include "state_machine.hpp"

#include <cstdint>
#include <cstring>

namespace ams {

class Bootloader {
public:
    // Open all relays, drain TX, stamp the jump reason into BKP2R, write
    // BL_BOOT_REQ_MAGIC to BKP0R, NVIC_SystemReset. Never returns. The reason
    // word survives the reset, readable by the BL (or by the next app boot, if
    // the BL passes control on without clearing it). Callers reached from the
    // bus must gate on reboot_allowed_in() first.
    [[noreturn]] static void request_reboot(
        config::JumpReason reason = config::JumpReason::ManualRequest) noexcept;

    // States in which honouring the reboot trigger is safe.
    //
    // request_reboot() opens all relays and resets. In an energised state that
    // means opening AIR+ under whatever the inverter is drawing -- which is
    // how contactors weld -- and dropping the safety supervisor mid-drive, on
    // a 4-byte frame anyone on the accumulator bus can send. In Start and
    // Error the contactors are already open, so the reboot costs nothing that
    // has not already happened.
    //
    // Error is not a grudging exception: ErrorLatch is sticky across resets,
    // so a car that faulted boots back INTO Error. Refusing there would mean
    // clearing the latch before a faulted AMS could be reflashed -- exactly
    // when you most want to reflash it.
    //
    // Same state set and same reasoning as diag_dispatch's logfs_allowed_in;
    // keep the two in step. Both answer "is the tractive system quiet enough
    // to interrupt this node?"
    [[nodiscard]] static bool reboot_allowed_in(fsm::State s) noexcept {
        return s == fsm::State::Start || s == fsm::State::Error;
    }

    // True iff a CanFrame is the boot-request trigger. Pure logic, in the
    // header so the host unit-test build exercises it without HAL / FreeRTOS.
    [[nodiscard]] static bool matches_trigger(const CanFrame& f) noexcept {
        if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;
        if (f.id  != config::BlBootReqCanId)                return false;
        if (f.dlc != config::BlBootReqDlc)                  return false;
        return std::memcmp(f.data,
                           config::BlBootReqPayload,
                           config::BlBootReqDlc) == 0;
    }
};

}  // namespace ams
