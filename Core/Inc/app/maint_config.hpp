// SPDX-License-Identifier: proprietary
//
// MAINTENANCE-BALANCING BUILD. Not a flight configuration.
//
// Built only with -DAMS_MAINT_BALANCE=ON, for rebalancing the accumulator in the
// pit while part of the BMS is missing or damaged. It changes ONE thing: what
// BmsPollTask writes to the LTC discharge (DCC) bits. Every safety predicate,
// the FSM, the relay driver, AMS_OK and the ErrorLatch are compiled exactly as
// in the flight build -- so with a module missing or the ignored sensors open,
// the AMS latches Error, holds the AIRs open and drops AMS_OK as it always does.
// The maintenance policy is what lets balancing run in that Error state, on the
// data it can still trust (balance::maint::compute_mask).
//
// Flash the release image back when the maintenance is done. The flight build
// (flag OFF) never calls balance::maint and is byte-for-byte unaffected by it.

#pragma once

#include "ams_config.hpp"

#include <cstdint>

namespace ams::maint_config {

#if defined(AMS_MAINT_BALANCE)
inline constexpr bool Enabled = true;
#else
inline constexpr bool Enabled = false;
#endif

// Modules whose cell-temperature sensors are IGNORED by the maintenance
// balancing policy (bit m = module m, 0-based chain order). Module 0's NTC
// harness is known broken, so its readings -- open, or decoding to anything at
// all -- neither feed the thermal lockout nor block balancing as a lost sensor.
//
// CONSEQUENCE: module 0 balances with NO firmware thermal protection on its own
// cells. The BalanceTempMax lockout still reads every other online module, but
// heat in module 0 is invisible to it. Watch module 0 by other means (IR camera,
// a thermocouple on the board) while it balances, or exclude it from the
// operator's 0x104 module-enable mask.
inline constexpr std::uint8_t IgnoredTempModules = 0x01u;

// Discharge limits for the maintenance policy.
//
// Default: UNCAPPED. Every cell more than BalanceDeltaMv above the floor bleeds
// at once, neighbours included -- up to all 19 cells of a module, against the
// flight limits of BalanceMaxActive (8) and no two adjacent.
//
// -DAMS_MAINT_BALANCE_CAPPED=ON keeps the maintenance policy (partial pack,
// module 0 sensors ignored) but restores the flight limits exactly, for when
// board temperatures say uncapped is too much.
//
// Heat: R71 || R72 = 23.5 ohm per cell -> 3.9^2 / 23.5 = 0.65 W per bleeding
// cell, so ~12 W per BMS_LITE board and ~62 W across the pack with all 95 on,
// at roughly 80 % duty (discharge pauses for every measurement). The ~71 C pad
// figure behind the flight cap was ONE pad on an open bench at 8 per module,
// clustered; nothing has been measured at 19 per module, or in a closed
// accumulator. The BalanceTempMax lockout reads CELL temperatures, not the
// board, and reads nothing at all in IgnoredTempModules. Watch board
// temperatures by other means while this runs.
#if defined(AMS_MAINT_BALANCE_CAPPED)
inline constexpr bool         Capped             = true;
inline constexpr std::uint8_t MaxActivePerModule = config::BalanceMaxActive;
inline constexpr bool         SpreadNoAdjacent   = config::BalanceSpreadNoAdjacent;
#else
inline constexpr bool         Capped             = false;
inline constexpr std::uint8_t MaxActivePerModule = config::CellsPerModule;
inline constexpr bool         SpreadNoAdjacent   = false;
#endif

}  // namespace ams::maint_config
