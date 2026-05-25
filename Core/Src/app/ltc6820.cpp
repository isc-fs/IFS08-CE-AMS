// SPDX-License-Identifier: proprietary

#include "ltc6820.hpp"

#include "ltc6811.hpp"
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
// send is the read-register-group reply (4 cmd + 8 * LtcChainLength
// data = 84 bytes at <=1 MHz SCK -> ~700 us on the wire). 10 ms is
// two orders of magnitude of headroom and still leaves the calling
// task responsive.
constexpr std::uint32_t SpiTimeoutMs = 10;

// LTC6811 wakeup pulse width. Datasheet § "Core LTC6811 State
// Transitions" specifies t_WAKE >= 10 µs. We use a 20 µs pulse + 30
// µs gap per IC so the chain is solidly out of IDLE; the whole
// LtcChainLength sweep is still under 1 ms.
constexpr std::uint32_t WakePulseUs = 20;
constexpr std::uint32_t WakeGapUs   = 30;

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

bool Bus::write_chain_command(std::uint16_t cmd,
                              const std::uint8_t per_ic_data[][6]) noexcept {
    std::uint8_t frame[4 + 8 * config::LtcChainLength];
    ltc6811::build_write_frame(cmd, per_ic_data, frame, sizeof(frame));
    return transfer(frame, nullptr, sizeof(frame));
}

bool Bus::stcomm() noexcept {
    // STCOMM = command frame + 3 bytes (24 SCK pulses) of dummy
    // clocks per IC in the daisy-chain. The LTC uses those clocks to
    // shift the COMM register contents out of its GPIO-SPI port to
    // the attached slave; data on MOSI is ignored, but the bus must
    // keep ticking SCK or the mux receives nothing.
    constexpr std::size_t DummyBytes = 3u * config::LtcChainLength;
    std::uint8_t frame[4 + DummyBytes];
    const auto cmd = ltc6811::pack_command(ltc6811::CmdSTCOMM);
    std::memcpy(frame, cmd.data(), 4);
    std::memset(frame + 4, 0xFFu, DummyBytes);
    return transfer(frame, nullptr, sizeof(frame));
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
    for (std::size_t i = 0; i < config::LtcChainLength; ++i) {
        cs_low();
        delay_us(WakePulseUs);
        cs_high();
        delay_us(WakeGapUs);
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
                                     SpiTimeoutMs);
    } else if (tx != nullptr) {
        st = HAL_SPI_Transmit(hspi_,
                              const_cast<std::uint8_t*>(tx),
                              static_cast<std::uint16_t>(len),
                              SpiTimeoutMs);
    } else if (rx != nullptr) {
        // Receive-only: HAL_SPI_Receive drives MOSI with whatever the
        // SPI peripheral last latched. The LTC6820 ignores MOSI
        // during read replies, so we don't need to force 0xFF.
        st = HAL_SPI_Receive(hspi_, rx,
                             static_cast<std::uint16_t>(len),
                             SpiTimeoutMs);
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
    constexpr std::size_t reply_len = 8 * config::LtcChainLength;
    constexpr std::size_t full_len  = 4 + reply_len;
    if (out_capacity < reply_len) {
        return false;
    }

    // Single CS-low, single HAL call (#213). The previous version
    // issued HAL_SPI_Transmit(4) followed by HAL_SPI_TransmitReceive(80)
    // inside one CS pulse; the few-us gap between the two HAL calls
    // (peripheral mode change TX-only -> full-duplex) breaks bit-sync
    // on slaves that re-sync on every CS edge -- notably the Pi Pico
    // LTC6820/LTC6811 emulator (IFS08_HIL feat/pico-ltc-emulator) which
    // sampled the cmd phase with garbled PECs. A single 84-byte
    // TransmitReceive holds the clock continuous across both phases.
    // MISO from the cmd phase is discarded (LTC ignores it).
    std::uint8_t tx[full_len];
    std::uint8_t rx[full_len];
    std::memcpy(tx, cmd_frame_4, 4);
    std::memset(tx + 4, 0xFFu, reply_len);

    cs_low();
    const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        hspi_, tx, rx, static_cast<std::uint16_t>(full_len), SpiTimeoutMs);
    cs_high();
    if (st != HAL_OK) {
        return false;
    }
    std::memcpy(out, rx + 4, reply_len);
    return true;
}

}  // namespace ams::ltc6820

#endif  // HAL_SPI_MODULE_ENABLED
