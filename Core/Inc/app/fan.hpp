// SPDX-License-Identifier: proprietary
//
// Cooling-fan PWM wrapper. Drives TIM17_CH1 (PB9). Period is fixed
// by the .ioc (TIM17.Period = 10559) -- we only manipulate CCR1.

#pragma once

#include <cstdint>

namespace ams {

class Fan {
public:
    // Starts PWM output on TIM17_CH1 at 0% duty. Idempotent.
    static void init() noexcept;

    // Set duty cycle 0..100. Anything > 100 clamps to 100.
    static void set_duty_pct(std::uint8_t pct) noexcept;
};

}  // namespace ams
