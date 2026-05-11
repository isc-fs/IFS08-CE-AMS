// SPDX-License-Identifier: proprietary
//
// Bit positions for the two FreeRTOS event groups declared in AMS.ioc:
//
//   safety_events   <- StateTask, BMS freshness check, precharge timer
//                       all set bits here; SafetyTask consumes.
//   bms_events      <- osTimer callbacks set kPollVDue / kPollTDue;
//                       BmsPollTask consumes.
//
// CMSIS-RTOS v2 represents event groups as 24-bit values (bits 24..31
// are reserved for status). Keep all bit allocations under bit 24.

#pragma once

#include <cstdint>

namespace ams::events::safety {

// FSM-driven force-error (state-task / precharge-timeout / etc.).
inline constexpr std::uint32_t kForceError     = 1u << 0;

// Relay-action requests from StateTask. SafetyTask services them on
// the clean (no-fault) path. If a fault is also active in the same
// tick the relay-action bits are ignored -- safety wins.
inline constexpr std::uint32_t kCloseAirN      = 1u << 1;
inline constexpr std::uint32_t kCloseAirP      = 1u << 2;
inline constexpr std::uint32_t kClosePrecharge = 1u << 3;
inline constexpr std::uint32_t kOpenAirN       = 1u << 4;
inline constexpr std::uint32_t kOpenAirP       = 1u << 5;
inline constexpr std::uint32_t kOpenPrecharge  = 1u << 6;

inline constexpr std::uint32_t kAllRequest =
    kForceError    | kCloseAirN | kCloseAirP | kClosePrecharge |
    kOpenAirN      | kOpenAirP  | kOpenPrecharge;

inline constexpr std::uint32_t kAllRelayActions =
    kCloseAirN | kCloseAirP | kClosePrecharge |
    kOpenAirN  | kOpenAirP  | kOpenPrecharge;

}  // namespace ams::events::safety

namespace ams::events::bms {

inline constexpr std::uint32_t kPollVDue = 1u << 0;
inline constexpr std::uint32_t kPollTDue = 1u << 1;

inline constexpr std::uint32_t kAll = kPollVDue | kPollTDue;

}  // namespace ams::events::bms
