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

inline constexpr std::uint32_t kForceError     = 1u << 0;
inline constexpr std::uint32_t kAirCloseReq    = 1u << 1;
inline constexpr std::uint32_t kAirOpenReq     = 1u << 2;
inline constexpr std::uint32_t kPrechargeStart = 1u << 3;
inline constexpr std::uint32_t kPrechargeOk    = 1u << 4;

inline constexpr std::uint32_t kAllRequest =
    kForceError | kAirCloseReq | kAirOpenReq |
    kPrechargeStart | kPrechargeOk;

}  // namespace ams::events::safety

namespace ams::events::bms {

inline constexpr std::uint32_t kPollVDue = 1u << 0;
inline constexpr std::uint32_t kPollTDue = 1u << 1;

inline constexpr std::uint32_t kAll = kPollVDue | kPollTDue;

}  // namespace ams::events::bms
