// SPDX-License-Identifier: proprietary

#include "relay_driver.hpp"

#include "main.h"   // RELAY_*_Pin / *_GPIO_Port macros, HAL pulled in transitively

namespace ams {

namespace {

inline void write_pin(GPIO_TypeDef *port, uint16_t pin, bool closed) noexcept {
    HAL_GPIO_WritePin(port, pin,
                      closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

inline bool read_pin(GPIO_TypeDef *port, uint16_t pin) noexcept {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

}  // namespace

void Relays::open_all() noexcept {
    // Single-write atomic open of all three contactors. Uses BSRR
    // semantics (HAL_GPIO_WritePin -> BSRR) so this is interrupt-safe
    // without disabling IRQs. Port is GPIOB on the v1.2 daughterboard
    // (PB5 / PB6 / PB7); was GPIOD pre-#117 -- the port literal here
    // wasn't updated in that PR, so on the post-#117 firmware this
    // call wrote to unconfigured PD5/6/7 instead of opening the real
    // relays. Caught in #128.
    HAL_GPIO_WritePin(GPIOB,
                      RELAY_AIR_N_Pin | RELAY_AIR_P_Pin | RELAY_PRECHARGE_Pin,
                      GPIO_PIN_RESET);
}

void Relays::close_air_negative() noexcept {
    write_pin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin, true);
}
void Relays::open_air_negative() noexcept {
    write_pin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin, false);
}

void Relays::close_air_positive() noexcept {
    write_pin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin, true);
}
void Relays::open_air_positive() noexcept {
    write_pin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin, false);
}

void Relays::close_precharge() noexcept {
    write_pin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, true);
}
void Relays::open_precharge() noexcept {
    write_pin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, false);
}

bool Relays::is_air_negative_closed() noexcept {
    return read_pin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin);
}
bool Relays::is_air_positive_closed() noexcept {
    return read_pin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin);
}
bool Relays::is_precharge_closed() noexcept {
    return read_pin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin);
}

}  // namespace ams

// C-callable wrapper so the C-language freertos.c hooks can latch
// safe state without including the C++ header. Resolves to the same
// HAL_GPIO_WritePin -> BSRR transaction; safe from any context
// (including the FreeRTOS hook ISR-ish path).
extern "C" void ams_relays_open_all_c(void) {
    ams::Relays::open_all();
}
