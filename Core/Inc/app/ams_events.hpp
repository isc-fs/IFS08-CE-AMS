// SPDX-License-Identifier: proprietary
//
// Bit positions for the two FreeRTOS event groups declared in AMS.ioc:
//
//   safety_events   <- legacy StateTask -> SafetyTask handshake.
//                       Retired in refactor/19 phase 3: the FSM's
//                       safety_flags bitmask is now consumed inline by
//                       MainTask in the same iteration the FSM produced
//                       it. The bit definitions below survive because
//                       fsm::Output::safety_flags reuses the same bit
//                       layout, but the osEventFlags handle is unused.
//   bms_events      <- osTimer callbacks set PollVDue / PollTDue;
//                       BmsPollTask consumes.
//
// CMSIS-RTOS v2 represents event groups as 24-bit values (bits 24..31
// are reserved for status). Keep all bit allocations under bit 24.

#pragma once

#include <cstdint>

namespace ams::events::safety {

// FSM-driven force-error (precharge-timeout / transition drop / etc.).
inline constexpr std::uint32_t ForceError     = 1u << 0;

// Relay-action bits set by fsm::step's Output::safety_flags. MainTask
// applies them inline on the clean (no-fault) path. If a fault is also
// active in the same tick the relay-action bits are ignored -- safety
// wins (relays already opened by latch_error_).
inline constexpr std::uint32_t CloseAirN      = 1u << 1;
inline constexpr std::uint32_t CloseAirP      = 1u << 2;
inline constexpr std::uint32_t ClosePrecharge = 1u << 3;
inline constexpr std::uint32_t OpenAirN       = 1u << 4;
inline constexpr std::uint32_t OpenAirP       = 1u << 5;
inline constexpr std::uint32_t OpenPrecharge  = 1u << 6;

inline constexpr std::uint32_t AllRequest =
    ForceError    | CloseAirN | CloseAirP | ClosePrecharge |
    OpenAirN      | OpenAirP  | OpenPrecharge;

inline constexpr std::uint32_t AllRelayActions =
    CloseAirN | CloseAirP | ClosePrecharge |
    OpenAirN  | OpenAirP  | OpenPrecharge;

}  // namespace ams::events::safety

namespace ams::events::bms {

inline constexpr std::uint32_t PollVDue = 1u << 0;
inline constexpr std::uint32_t PollTDue = 1u << 1;

inline constexpr std::uint32_t All = PollVDue | PollTDue;

}  // namespace ams::events::bms
