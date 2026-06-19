// SPDX-License-Identifier: proprietary
//
// Pure-logic Bus-Off recovery decision (the rate-limiter). HAL-free so
// the host unit-test build exercises the timing edge cases without an
// M_CAN peripheral. The HAL plumbing -- HAL_FDCAN_GetProtocolStatus to
// read PSR.BO, the HAL_FDCAN_Stop/Start, and the State/ErrorCode unwedge
// -- lives in acu_can_task.cpp; this header owns only the "should I
// attempt a Stop/Start this poll?" policy.
//
// Background: on the STM32H7 M_CAN, sustained TX errors (transmitting
// into a bus with no node ACKing, or a transient bus fault) drive the
// node to Bus_Off, which sets CCCR.INIT and halts BOTH TX and RX -- the
// node stops ACKing, goes silent, and does NOT self-clear; only a
// software Stop->Start re-arms it. AcuCanTask polls for this and recovers.
//
// This mirrors the bootloader's Bootloader_FdcanBusOffRecover rate-limit
// (../stm32-can-bootloader, issues #125 C1 / #174 NG-9): act on the
// transition INTO Bus_Off, then at most once per retry window while it
// persists. Without the rate-limit, a Stop/Start every poll would
// continually restart the M_CAN's automatic recovery (it rejoins after
// 128*11 consecutive recessive bits, ~2.8 ms of idle bus at 500 kbps),
// and the node would never actually finish rejoining.

#pragma once

#include <cstdint>

namespace ams::can_recovery {

// Per-bus latch carried across polls by the caller. A value-initialised
// instance ({}) means "bus healthy, no recovery attempted yet".
struct BusOffState {
    bool          was_busoff      = false;  // were we in Bus_Off on the last poll?
    std::uint32_t last_attempt_ms = 0;      // tick of the last Stop/Start attempt
};

// Decide whether to issue a Stop/Start recovery on this poll.
//
//   st        : per-bus latch, updated in place
//   bus_off   : live PSR.BO read (HAL_FDCAN_GetProtocolStatus -> .BusOff != 0)
//   now_ms    : current tick (osKernelGetTickCount)
//   retry_ms  : minimum spacing between attempts while Bus_Off persists
//
// Returns true exactly on:
//   * the FIRST poll that observes Bus_Off (the healthy->off edge), and
//   * each retry_ms boundary thereafter while it stays off.
//
// A healthy read (bus_off == false) clears the latch, so the next genuine
// entry into Bus_Off is acted on immediately again.
//
// Tick wrap (49.7 days at the 1 kHz FreeRTOS tick) is handled by unsigned
// modular subtraction: (now_ms - last_attempt_ms) wraps consistently, so
// the spacing check stays correct across the rollover.
[[nodiscard]] inline bool should_attempt_recovery(BusOffState&  st,
                                                  bool          bus_off,
                                                  std::uint32_t now_ms,
                                                  std::uint32_t retry_ms) noexcept {
    if (!bus_off) {
        st.was_busoff = false;
        return false;
    }
    // Bus_Off is set. Already attempting and still inside the rate-limit
    // window -> let the M_CAN keep rejoining; don't restart it.
    if (st.was_busoff && (now_ms - st.last_attempt_ms) < retry_ms) {
        return false;
    }
    st.was_busoff      = true;
    st.last_attempt_ms = now_ms;
    return true;
}

}  // namespace ams::can_recovery
