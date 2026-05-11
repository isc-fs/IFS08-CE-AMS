// SPDX-License-Identifier: proprietary

#include "fan.hpp"

#include "main.h"

extern "C" {
extern TIM_HandleTypeDef htim17;
}

namespace {
constexpr std::uint32_t kPwmPeriod = 10559u;  // matches AMS.ioc TIM17.Period
}  // namespace

namespace ams {

void Fan::init() noexcept {
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0u);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void Fan::set_duty_pct(std::uint8_t pct) noexcept {
    if (pct > 100u) pct = 100u;
    const std::uint32_t ccr = (kPwmPeriod * static_cast<std::uint32_t>(pct)) / 100u;
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, ccr);
}

}  // namespace ams
