// SPDX-License-Identifier: proprietary

#include "ltc6820.hpp"

#include "stm32h7xx_hal.h"

// SPI1 isn't yet enabled in AMS.ioc (the CubeMX regen will add
// stm32h7xx_hal_spi.c/.h and #define HAL_SPI_MODULE_ENABLED). Until
// that lands the whole wrapper compiles as an empty TU so the rest
// of the firmware keeps building. The header stays usable in unit
// tests and code review; flipping the .ioc switch activates the body.
#ifdef HAL_SPI_MODULE_ENABLED

#include <cstring>

namespace ams::ltc6820 {

namespace {

// HAL_SPI_TransmitReceive timeout. The longest single transaction we
// send is the read-register-group reply (4 cmd + 8 * kLtcChainLength
// data = 84 bytes at <=1 MHz SCK -> ~700 us on the wire). 10 ms is
// two orders of magnitude of headroom and still leaves the calling
// task responsive.
constexpr std::uint32_t kSpiTimeoutMs = 10;

// LTC6811 wakeup pulse width. Datasheet § "Core LTC6811 State
// Transitions" specifies t_WAKE >= 10 µs. We use a 20 µs pulse + 30
// µs gap per IC so the chain is solidly out of IDLE; the whole
// kLtcChainLength sweep is still under 1 ms.
constexpr std::uint32_t kWakePulseUs = 20;
constexpr std::uint32_t kWakeGapUs   = 30;

// Busy-wait microsecond delay. AMS firmware has no DWT cycle counter
// enabled (yet) so we approximate with a calibrated NOP loop. Coarse
// is fine -- the LTC6811 only cares about the MINIMUM pulse width.
// SYSCLK is 528 MHz so each loop iteration (~4 cycles) is ~7.6 ns;
// for 1 µs we need ~132 iterations. Use 150 to be safe.
__attribute__((always_inline))
inline void delay_us(std::uint32_t us) noexcept {
    for (std::uint32_t i = 0; i < us; ++i) {
        for (volatile std::uint32_t k = 0; k < 150; ++k) {
            __asm__ volatile("nop");
        }
    }
}

}  // namespace

Bus::Bus(SPI_HandleTypeDef* hspi, CsPin cs) noexcept
    : hspi_{hspi}, cs_{cs} {
    cs_high();
}

void Bus::configure(SPI_HandleTypeDef* hspi, CsPin cs) noexcept {
    hspi_ = hspi;
    cs_   = cs;
    cs_high();
}

Bus& Bus::default_instance() noexcept {
    // Function-local static under -fno-threadsafe-statics: zero
    // guards, zero locks, zero overhead at the call site. Default-
    // constructed once on first reach (an early App_InitTask call);
    // configure() then wires it to hspi1 + PA4.
    static Bus s_instance;
    return s_instance;
}

void Bus::cs_low() noexcept {
    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(cs_.port), cs_.pin, GPIO_PIN_RESET);
}

void Bus::cs_high() noexcept {
    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(cs_.port), cs_.pin, GPIO_PIN_SET);
}

void Bus::wakeup() noexcept {
    // One CS pulse per IC in the chain. The pulse train propagates
    // along the isoSPI links: each IC consumes one pulse to wake up,
    // and only when it's awake does it forward the next pulse to the
    // next IC. See LTC6811 datasheet § "Waking Up the Daisy Chain".
    for (std::size_t i = 0; i < config::kLtcChainLength; ++i) {
        cs_low();
        delay_us(kWakePulseUs);
        cs_high();
        delay_us(kWakeGapUs);
    }
}

bool Bus::transfer(const std::uint8_t* tx,
                   std::uint8_t*       rx,
                   std::size_t         len) noexcept {
    if (len == 0) {
        return true;
    }

    cs_low();
    HAL_StatusTypeDef st = HAL_OK;
    if (tx != nullptr && rx != nullptr) {
        st = HAL_SPI_TransmitReceive(hspi_,
                                     const_cast<std::uint8_t*>(tx),
                                     rx,
                                     static_cast<std::uint16_t>(len),
                                     kSpiTimeoutMs);
    } else if (tx != nullptr) {
        st = HAL_SPI_Transmit(hspi_,
                              const_cast<std::uint8_t*>(tx),
                              static_cast<std::uint16_t>(len),
                              kSpiTimeoutMs);
    } else if (rx != nullptr) {
        // Receive-only: HAL_SPI_Receive drives MOSI with whatever the
        // SPI peripheral last latched. The LTC6820 ignores MOSI
        // during read replies, so we don't need to force 0xFF.
        st = HAL_SPI_Receive(hspi_, rx,
                             static_cast<std::uint16_t>(len),
                             kSpiTimeoutMs);
    }
    cs_high();
    return st == HAL_OK;
}

bool Bus::send_command(const std::uint8_t cmd_frame_4[4]) noexcept {
    return transfer(cmd_frame_4, nullptr, 4);
}

bool Bus::read_register_group(const std::uint8_t cmd_frame_4[4],
                              std::uint8_t*      out,
                              std::size_t        out_capacity) noexcept {
    const std::size_t reply_len = 8 * config::kLtcChainLength;
    if (out_capacity < reply_len) {
        return false;
    }

    // Single CS-low transaction: command (4) followed by chain reply
    // (8 * N). We use a small scratch dummy buffer for the TX side
    // during the reply phase since HAL_SPI_TransmitReceive needs a
    // valid pointer for both directions.
    std::uint8_t scratch[8 * config::kLtcChainLength];
    std::memset(scratch, 0xFF, reply_len);

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(
        hspi_, const_cast<std::uint8_t*>(cmd_frame_4), 4, kSpiTimeoutMs);
    if (st == HAL_OK) {
        st = HAL_SPI_TransmitReceive(
            hspi_, scratch, out,
            static_cast<std::uint16_t>(reply_len), kSpiTimeoutMs);
    }
    cs_high();
    return st == HAL_OK;
}

}  // namespace ams::ltc6820

#endif  // HAL_SPI_MODULE_ENABLED
