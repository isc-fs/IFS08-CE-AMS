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
    // without disabling IRQs.
    //
    // INVARIANT: all three relay outputs MUST share the same GPIO
    // port for the atomic mask-write to be valid. main.h defines them
    // as PB5/PB6/PB7 per AMS.ioc; using RELAY_AIR_N_GPIO_Port as the
    // canonical port reference means a future .ioc port relocation updates
    // this call transparently. This once hardcoded GPIOB (and before that
    // GPIOD, on the legacy daughterboard); when the relays were re-routed the
    // literal here was not updated, leaving the fault path silently writing to
    // unconfigured PD pins. Referencing the macro makes that class of
    // regression impossible.
    //
    // If a future board variant splits the relays across ports, this
    // function (and the entire BSRR-atomic semantic of "open all in
    // one cycle") must be revisited. Per-pin open via the individual
    // open_air_*/open_precharge() helpers is the safe fallback.
    HAL_GPIO_WritePin(RELAY_AIR_N_GPIO_Port,
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

void Relays::set_ams_ok(bool enable) noexcept {
    // Active-high SDC enable. HAL_GPIO_WritePin -> BSRR, so the write is
    // atomic and interrupt-safe. enable==false drives AMS_OK LOW, which
    // opens the AMS's leg of the shutdown circuit.
    HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
