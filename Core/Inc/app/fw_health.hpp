/* SPDX-License-Identifier: proprietary */
#pragma once

// Firmware-health state behind the ungated 0x6CA frame: a task-liveness
// bitfield, the RCC reset cause, and a sticky last-fault sentinel. The pure
// decision logic (map_reset_cause) is host-tested; the hardware glue (RCC
// flags, RTC backup registers, the liveness atomic) lives in fw_health.cpp.

#include <cstdint>

#include "ams_config.hpp"

namespace ams::fw_health {

// Task-liveness bits -> 0x6CA byte [4]. Each critical task pokes its bit each
// pass; the 1 Hz emit samples + clears, so a set bit means "this task stepped
// in the last second". A stalled task leaves its bit 0 and the tool flags it.
enum LivenessBit : std::uint8_t {
    MainStepped  = 0u,  // bit0: SafetyTask (MainTask) 10 ms loop
    CanRx        = 1u,  // bit1: AcuCanTask drained/handled an RX frame
    CanTx        = 2u,  // bit2: AcuCanTask ran its periodic TX scheduler
    Housekeeping = 3u,  // bit3: BmsPollTask isoSPI sweep
};

// --- pure, host-testable ---------------------------------------------------
// Given which RCC reset flags are set, pick the single reported cause. The
// specific causes (watchdogs, software, low-power) win over the generic
// Pin/POR that a reset usually also asserts; POR beats Pin because a cold
// boot raises both.
[[nodiscard]] constexpr ams::config::ResetCause map_reset_cause(
        bool por, bool pin, bool sw, bool iwdg, bool wwdg, bool lowpower) noexcept {
    using RC = ams::config::ResetCause;
    if (iwdg)     return RC::Iwdg;
    if (wwdg)     return RC::Wwdg;
    if (lowpower) return RC::LowPower;
    if (sw)       return RC::Software;
    if (por)      return RC::PowerOn;
    if (pin)      return RC::Pin;
    return RC::Unknown;
}

// --- hardware glue (fw_health.cpp) -----------------------------------------
void          poke(LivenessBit b) noexcept;             // task-side: set a liveness bit
[[nodiscard]] std::uint8_t sample_liveness() noexcept;  // emit-side: read + clear the field

void          capture_reset_cause() noexcept;           // boot: read + clear RCC reset flags
[[nodiscard]] std::uint8_t reset_cause() noexcept;      // the mapped ResetCause byte

void          latch_boot_fault() noexcept;              // boot: BKP3 -> RAM, then clear BKP3
void          record_fault(ams::config::LastFault f) noexcept;  // fault path: stamp BKP3
[[nodiscard]] std::uint8_t last_fault() noexcept;       // the RAM-latched last-fault byte

[[nodiscard]] std::uint8_t uptime_s() noexcept;         // seconds since boot, wraps at 256

[[nodiscard]] std::uint32_t free_heap() noexcept;       // FreeRTOS free heap, bytes
[[nodiscard]] std::uint32_t min_free_heap() noexcept;   // min-ever free heap, bytes

}  // namespace ams::fw_health
