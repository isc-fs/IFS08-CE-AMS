// SPDX-License-Identifier: proprietary
//
// LTC6820 isoSPI master. Thin layer over HAL_SPI that owns the bus:
//   * the LTC6811 chain wakeup sequence (CSBM low-pulse train)
//   * CS-asserted SPI transfers
//   * the timing budget for LtcChainLength ICs (ams_config)
// Wire formats and payloads are ltc6811.hpp.
//
// The LTC6820 is a transparent SPI <-> isoSPI bridge: to the MCU it is
// a plain SPI slave, and the chip does the transformer-coupled isoSPI
// framing to the LTC6811 daisy-chain.
//
// SPI1 as configured in AMS.ioc:
//   PA5 = SPI1_SCK, PA6 = SPI1_MISO, PA7 = SPI1_MOSI -- AF5, full-duplex
//         master, MSB first, 8-bit data
//   PB9 = GPIO_Output, label LTC6820_CS, PinState = SET (idle HIGH);
//         app_init_task passes the generated LTC6820_CS_GPIO_Port/_Pin
//   Baud: prescaler 256 on the 132 MHz SPI123 clock = 515.625 kHz, under
//         the 1 MHz SCK cap for full daisy-chain operation (LTC6820
//         datasheet figure 10), with headroom for the cable runs to the
//         BMS modules.
//   NSS:  software-managed -- this module drives LTC6820_CS itself so CS
//         stays low across a whole multi-byte chain transaction.
//
// UNVERIFIED WIRING -- CHECK THE MAIN_LITE SCHEMATIC BEFORE FLASHING:
//   * The SPI pins are INFERRED, not confirmed: MAIN_LITE symbol pins
//     30/31/32 read as MOSI/MISO/SCK plus the LTC6820_CS net, and
//     PA5-PA7 are the conventional STM32H7 SPI1 AF5 mapping. Nobody has
//     compared this against the schematic.
//   * AMS.ioc sets CPOL = LOW, CPHA = 1-edge (SPI mode 0). The LTC6820
//     clocks in whatever mode its POL/PHA strap pins select, and those
//     straps are unchecked. A mismatch corrupts every transaction on
//     this bus.

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
    // Leaves the bus inert -- no HAL touched. default_instance() relies
    // on that to have static-storage duration without pulling HAL into
    // the C-runtime init sequence. Call configure() once HAL is up.
    Bus() noexcept = default;

    Bus(SPI_HandleTypeDef* hspi, CsPin cs) noexcept;

    // Late-binding init, safe from App_InitTask once HAL_SPI and the
    // GPIO peripheral are alive. Calling it again rewires the handles
    // and releases the previously-asserted CS line.
    void configure(SPI_HandleTypeDef* hspi, CsPin cs) noexcept;

    // Process-wide Bus -- one LTC6820 master per AMS board. App_InitTask
    // wires it up via configure(); BmsPollTask (and the balancing / temp
    // paths) read it.
    static Bus& default_instance() noexcept;

    // Wake the chain: one CS-low pulse per IC. Each IC needs >= 10 µs of
    // CS-low to leave IDLE and does not forward the pulse until it is
    // itself awake, so the chain takes LtcChainLength pulses (LTC6811
    // datasheet § "Core LTC6811 State Transitions"). Pulse widths carry
    // generous margin so a freshly-powered chain is ready before the
    // first command goes out.
    void wakeup() noexcept;

    // Blocking SPI exchange with CS held low for the whole transfer.
    // tx == nullptr receives only (MOSI still drives 0xFF); rx ==
    // nullptr transmits only. Returns true on HAL OK.
    bool transfer(const std::uint8_t* tx,
                  std::uint8_t*       rx,
                  std::size_t         len) noexcept;

    // Send a 4-byte command frame (PEC-packed by
    // ams::ltc6811::pack_command) and discard the response. For ADCV /
    // ADAX / STCOMM, where the slave does not reply.
    bool send_command(const std::uint8_t cmd_frame_4[4]) noexcept;

    // Send a command frame, then clock in LtcChainLength * 8 bytes of
    // slave reply (cell-voltage groups, AUX groups, status). The caller
    // decodes per IC with ams::ltc6811::decode_*_group.
    bool read_register_group(const std::uint8_t cmd_frame_4[4],
                             std::uint8_t*      out,
                             std::size_t        out_capacity) noexcept;

    // Broadcast WRITE: command frame + per-IC 6-byte payload + per-IC
    // PEC, packed by ams::ltc6811::build_write_frame. Used by WRCOMM
    // (mux address load) and WRCFGA (balancing config). per_ic_data must
    // point to LtcChainLength rows of 6 bytes.
    bool write_chain_command(std::uint16_t        cmd,
                             const std::uint8_t   per_ic_data[][6]) noexcept;

    // STCOMM: shift each LTC's COMM register out of its GPIO SPI port
    // to the attached slave (ADG731 mux). The LTC needs the master to
    // keep clocking SCK for 24 cycles per IC after the command, which
    // we supply as 3 * LtcChainLength dummy bytes following the 4-byte
    // command frame.
    bool stcomm() noexcept;

   private:
    SPI_HandleTypeDef* hspi_ = nullptr;
    CsPin              cs_   = {};

    void cs_low() noexcept;
    void cs_high() noexcept;
};

}  // namespace ams::ltc6820
