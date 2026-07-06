/* SPDX-License-Identifier: proprietary */
#include "fw_health.hpp"

#include <atomic>

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

namespace ams::fw_health {
namespace {

// Accumulated liveness bits since the last sample. fetch_or/exchange keep the
// multi-task pokes race-free without a mutex (single-byte atomic on the M7).
std::atomic<std::uint8_t> g_liveness{0u};

std::uint8_t g_reset_cause = static_cast<std::uint8_t>(ams::config::ResetCause::Unknown);
std::uint8_t g_last_fault  = static_cast<std::uint8_t>(ams::config::LastFault::None);

// RTC->BKP0R..BKP31R are consecutive uint32_t members; index them flatly.
inline volatile std::uint32_t* bkp(std::uint32_t reg) noexcept {
    return &(&RTC->BKP0R)[reg];
}

}  // namespace

void poke(LivenessBit b) noexcept {
    g_liveness.fetch_or(static_cast<std::uint8_t>(1u << b), std::memory_order_relaxed);
}

std::uint8_t sample_liveness() noexcept {
    return g_liveness.exchange(0u, std::memory_order_relaxed);
}

void capture_reset_cause() noexcept {
    const ams::config::ResetCause c = map_reset_cause(
        __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) || __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST),
        __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST),
        __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST),
        __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST),
        __HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST),
        __HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST));
    g_reset_cause = static_cast<std::uint8_t>(c);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

std::uint8_t reset_cause() noexcept { return g_reset_cause; }

void latch_boot_fault() noexcept {
    HAL_PWR_EnableBkUpAccess();
    g_last_fault = static_cast<std::uint8_t>(*bkp(ams::config::BkpLastFaultReg) & 0xFFu);
    *bkp(ams::config::BkpLastFaultReg) = 0u;   // one-shot: a clean next boot reads None
}

void record_fault(ams::config::LastFault f) noexcept {
    // Called from the HardFault handler. Backup-domain write access is already
    // enabled at boot (ErrorLatch::init / latch_boot_fault), so this is a bare
    // register store -- safe from fault context.
    *bkp(ams::config::BkpLastFaultReg) = static_cast<std::uint32_t>(f);
}

std::uint8_t last_fault() noexcept { return g_last_fault; }

std::uint8_t uptime_s() noexcept {
    return static_cast<std::uint8_t>((osKernelGetTickCount() / 1000u) & 0xFFu);
}

std::uint32_t free_heap() noexcept {
    return static_cast<std::uint32_t>(xPortGetFreeHeapSize());
}

std::uint32_t min_free_heap() noexcept {
    return static_cast<std::uint32_t>(xPortGetMinimumEverFreeHeapSize());
}

}  // namespace ams::fw_health

// C-linkage shim so HardFault_Handler (C, stm32h7xx_it.c) can stamp the
// last-fault sentinel without pulling in the C++ namespace. Called from
// exception context: a single backup-register write, no allocation/locks.
extern "C" void ams_fw_health_record_hardfault(void) {
    ams::fw_health::record_fault(ams::config::LastFault::HardFault);
}
