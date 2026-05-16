// SPDX-License-Identifier: proprietary

#include "error_latch.hpp"
#include "ams_config.hpp"

#include "main.h"  // pulls stm32h7xx_hal.h, RTC peripheral, PWR

namespace ams {

namespace {

// kBkpErrorReg selects which RTC_BKPxR register holds the flag (default
// 0 from ams_config). kBkpErrorMagic is the only value we recognise as
// "latched"; any other value, including the post-power-on garbage, is
// treated as "not latched".
constexpr std::uint32_t kReg   = ams::config::kBkpErrorReg;
constexpr std::uint32_t kMagic = ams::config::kBkpErrorMagic;

inline volatile std::uint32_t* bkp_register() noexcept {
    // The H733 maps BKP0R..BKP31R as consecutive uint32_t members on
    // the RTC peripheral. Indexing past 31 is undefined.
    static_assert(kReg < 32, "kBkpErrorReg out of range");
    return &(&RTC->BKP0R)[kReg];
}

}  // namespace

void ErrorLatch::init() noexcept {
    // STM32H7's PWR controller is always clocked (part of the always-on
    // supervisor); no __HAL_RCC_PWR_CLK_ENABLE() is needed. We just have
    // to lift the DBP write protection on the backup domain.
    HAL_PWR_EnableBkUpAccess();
}

bool ErrorLatch::is_set() noexcept {
    return *bkp_register() == kMagic;
}

void ErrorLatch::set() noexcept {
    *bkp_register() = kMagic;
}

void ErrorLatch::clear() noexcept {
    *bkp_register() = 0;
}

}  // namespace ams

// C-callable wrapper for the freertos.c hooks that need to persist a
// fault across the watchdog reset they're about to trigger. Same
// preconditions as ErrorLatch::set (ErrorLatch::init must have run
// earlier in main / App_InitTask to unlock the backup domain).
extern "C" void ams_error_latch_set_c(void) {
    ams::ErrorLatch::set();
}
