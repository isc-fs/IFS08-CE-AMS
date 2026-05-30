// SPDX-License-Identifier: proprietary
//
// Low-level relay driver. Wraps the three GPIO pins controlling the
// accumulator contactors:
//
//   PD3 RELAY_AIR_N         (negative AIR)
//   PD4 RELAY_AIR_P         (positive AIR)
//   PD5 RELAY_PRECHARGE     (precharge contactor)
//
// All pins are active-high (drive HIGH to energise the relay coil).
// CubeMX-generated MX_GPIO_Init drives all three to PIN_RESET (open)
// BEFORE configuring them as outputs, so the pack is electrically
// isolated for the entire window between reset and the first task
// running.  See docs/ARCHITECTURE.md §1 invariant 2 / §9 power-up.
//
// API is intentionally minimal and stateless. The only "policy" lives
// in SafetyTask -- this is just the bit-banging primitive. Per
// ARCHITECTURE.md §6, ONLY SafetyTask (and App_InitTask during boot)
// may call the close_* helpers; everyone else must request transitions
// via the safety_events event group.

#pragma once

namespace ams {

class Relays {
public:
    // Drive all three contactors to the open (de-energised) state.
    // Safe to call from any context, including a fault path -- this
    // is the function the watchdog-driven hardware reset effectively
    // re-asserts on every cold start.
    static void open_all() noexcept;

    static void close_air_negative() noexcept;
    static void open_air_negative()  noexcept;

    static void close_air_positive() noexcept;
    static void open_air_positive()  noexcept;

    static void close_precharge()    noexcept;
    static void open_precharge()     noexcept;

    // AMS_OK (PB4): the AMS's contribution to the shutdown circuit
    // (SDC). Active-high -- HIGH = "AMS healthy, not blocking the SDC";
    // LOW opens the SDC. Driven exclusively by SafetyTask, which asserts
    // it only once the boot grace has passed and no ERROR is latched,
    // and clears it the instant a fault latches (#299). Not an AIR/
    // contactor, but it shares the same SDC-output ownership, so it
    // lives here next to open_all().
    static void set_ams_ok(bool enable) noexcept;

    // Read-back helpers for sanity checks (the GPIO ODR drives the
    // relay coil; an MCU-side read says nothing about the contactor
    // *actually* closing, but it does confirm we didn't get our own
    // state model wrong).
    [[nodiscard]] static bool is_air_negative_closed() noexcept;
    [[nodiscard]] static bool is_air_positive_closed() noexcept;
    [[nodiscard]] static bool is_precharge_closed()    noexcept;
};

}  // namespace ams
