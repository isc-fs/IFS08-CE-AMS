// SPDX-License-Identifier: proprietary
//
// LTC6820 isoSPI master wrapper. Thin layer on top of HAL_SPI that
// handles:
//   * the LTC6811 chain wakeup sequence (CSBM low-pulse train)
//   * cs-asserted SPI transfers ("transaction" helpers)
//   * timing budget for chain length (kLtcChainLength from ams_config)
//
// The LTC6820 is a transparent SPI <-> isoSPI bridge: from the MCU
// side it looks like a plain SPI slave, the chip handles isoSPI
// transformer-coupled framing to the LTC6811 daisy-chain.
//
// Wire-format / payload construction is in ltc6811.hpp -- this module
// only owns the bus.
//
// Required CubeMX config (AMS.ioc) -- not yet committed, see #68:
//   IP:        SPI1 master, full-duplex
//   Pins:      PA5 = SPI1_SCK   AF5
//              PA6 = SPI1_MISO  AF5
//              PA7 = SPI1_MOSI  AF5
//              PA4 = GPIO_Output, label LTC6820_CS, default HIGH
//   Mode:      CPOL = HIGH, CPHA = 2-edge   (SPI Mode 3)
//   Format:    MSB first, 8-bit data
//   Baud:      <= 1 MHz  (LTC6820 datasheet figure 10 caps SCK at 1 MHz
//              for full daisy-chain operation; we run at ~500 kHz to
//              give headroom for cable runs to the BMS modules).
//   NSS:       software-managed (we drive PA4 manually so we can hold
//              CS low across multi-byte chain transactions).
//
// Pin numbers are inferred from the AMS schematic: the MAIN_LITE
// module's symbol pins 30/31/32 correspond to SPI1_MOSI/MISO/SCK and
// the LTC6820_CS net. PA4-PA7 are the conventional STM32H7 SPI1 AF5
// mapping, and currently all four are unassigned in AMS.ioc.
// VERIFY against the MAIN_LITE module schematic before flashing.

#pragma once

#include "ams_config.hpp"

#include <cstddef>
#include <cstdint>

// Forward-declare the HAL types so this header can be included from
// host unit tests without dragging in the STM32 HAL.
struct __SPI_HandleTypeDef;
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;

namespace ams::ltc6820 {

// GPIO descriptor for the LTC6820 CS line. Stored by Bus so it can be
// banged from any task without a global handle.
struct CsPin {
    void*         port;  // GPIO_TypeDef* -- erased to keep header HAL-free
    std::uint16_t pin;   // GPIO_PIN_x bitmask
};

// One Bus instance per MCU SPI peripheral feeding an LTC6820.
class Bus {
   public:
    // Default ctor leaves the bus inert (no HAL touched). The
    // singleton ams::ltc6820::Bus::default_instance() relies on this
    // so it can have static-storage duration without pulling HAL into
    // the C-runtime init sequence. Call configure() after HAL is up.
    Bus() noexcept = default;

    Bus(SPI_HandleTypeDef* hspi, CsPin cs) noexcept;

    // Late-binding init. Safe to call from App_InitTask once HAL_SPI
    // and the GPIO peripheral are alive. Calling twice rewires the
    // handles; the previously-asserted CS line is released.
    void configure(SPI_HandleTypeDef* hspi, CsPin cs) noexcept;

    // Process-wide default Bus. Initialised by App_InitTask via
    // configure() and read from BmsPollTask (and future balancing /
    // temp tasks). One LTC6820 master per AMS board, so a singleton
    // is the natural fit.
    static Bus& default_instance() noexcept;

    // Wake the chain. Per LTC6811 datasheet § "Core LTC6811 State
    // Transitions": each IC needs >=  10 µs of CS-low to leave IDLE,
    // and a fresh pulse for every IC in the chain (the wakeup signal
    // does not propagate transparently). We send kLtcChainLength
    // pulses with a generous margin so a freshly-powered chain is
    // ready to talk before the first command goes out.
    void wakeup() noexcept;

    // Blocking SPI exchange with CS held low for the full duration.
    // Pass tx == nullptr to receive-only (we still drive 0xFF on
    // MOSI); rx == nullptr to transmit-only. Returns true on HAL OK.
    bool transfer(const std::uint8_t* tx,
                  std::uint8_t*       rx,
                  std::size_t         len) noexcept;

    // Convenience: drive a 4-byte command frame (already PEC-packed
    // by ams::ltc6811::pack_command) and discard the response. Used
    // for ADCV / ADAX / STCOMM where the slave doesn't reply.
    bool send_command(const std::uint8_t cmd_frame_4[4]) noexcept;

    // Convenience: send command frame then clock in
    //   kLtcChainLength * 8 bytes
    // of slave reply (cell-voltage groups, AUX groups, status). The
    // caller decodes per IC with ams::ltc6811::decode_*_group.
    bool read_register_group(const std::uint8_t cmd_frame_4[4],
                             std::uint8_t*      out,
                             std::size_t        out_capacity) noexcept;

    // Drive a broadcast WRITE command: cmd frame + per-IC 6-byte
    // payload + per-IC PEC, packed by ams::ltc6811::build_write_frame.
    // Used by WRCOMM (mux address load), WRCFGA (balancing config),
    // etc. per_ic_data must point to kLtcChainLength rows of 6 bytes.
    bool write_chain_command(std::uint16_t        cmd,
                             const std::uint8_t   per_ic_data[][6]) noexcept;

    // STCOMM: shift the COMM register out of every LTC's GPIO SPI
    // port to the attached slave (ADG731 mux). The LTC needs the
    // master to keep clocking SCK for 24 cycles per IC after the
    // command, which we provide by sending 3 * kLtcChainLength
    // dummy bytes after the 4-byte command frame.
    bool stcomm() noexcept;

   private:
    SPI_HandleTypeDef* hspi_ = nullptr;
    CsPin              cs_   = {};

    void cs_low() noexcept;
    void cs_high() noexcept;
};

}  // namespace ams::ltc6820
