// SPDX-License-Identifier: proprietary
//
// LTC6811-1 wire-format layer. Pure logic, no HAL / SPI dependency.
// Lives here so the host unit-test build can exercise PEC15 + every
// register-group decoder without dragging in FreeRTOS / STM32 HAL.
//
// The SPI-HAL wrapper (LTC6820 master + actual transactions) belongs
// to a separate module (lands on a later branch).
//
// References
// ----------
// - LTC6811-1 datasheet (pcbs/BMS_LITE/Datasheets/LTC6811HG-1.pdf)
//   § "Bus Protocol", § "Packet Error Code (PEC)",
//   § "Memory Map", § "Cell Voltage Register Group",
//   § "Auxiliary Register Group"
// - ADG731 datasheet (pcbs/BMS_LITE/Datasheets/ADG731.pdf) for the
//   3-wire SPI byte format that the LTC6811's GPIO/COMM bit-bang
//   shifts out to the mux.

#pragma once

#include "ams_config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ams::ltc6811 {

// ---------------------------------------------------------------------------
// PEC15 -- 15-bit CRC the LTC6811 appends to every transaction.
//
//   polynomial  0x4599  (x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
//   seed        0x0010
//
// The remainder is shifted left by 1 before transmission so the lowest
// bit of the 16-bit on-wire PEC is always 0. pec15() returns the
// already-shifted 16-bit value ready to drop into a 2-byte slot.
// ---------------------------------------------------------------------------
[[nodiscard]] std::uint16_t pec15(const std::uint8_t* data,
                                  std::size_t       len) noexcept;

// ---------------------------------------------------------------------------
// Command codes (datasheet table 38). 11 bits each; the on-wire
// command frame is 4 bytes: [cmd_hi, cmd_lo, pec_hi, pec_lo].
// ---------------------------------------------------------------------------
inline constexpr std::uint16_t CmdWRCFGA  = 0x0001;
inline constexpr std::uint16_t CmdRDCFGA  = 0x0002;
inline constexpr std::uint16_t CmdRDCVA   = 0x0004;  // cells 1..3
inline constexpr std::uint16_t CmdRDCVB   = 0x0006;  // cells 4..6
inline constexpr std::uint16_t CmdRDCVC   = 0x0008;  // cells 7..9
inline constexpr std::uint16_t CmdRDCVD   = 0x000A;  // cells 10..12
inline constexpr std::uint16_t CmdRDAUXA  = 0x000C;  // GPIO 1..3 + REF
inline constexpr std::uint16_t CmdRDAUXB  = 0x000E;  // GPIO 4..5 + spare
inline constexpr std::uint16_t CmdRDSTATA = 0x0010;
inline constexpr std::uint16_t CmdRDSTATB = 0x0012;
inline constexpr std::uint16_t CmdWRCOMM  = 0x0721;
inline constexpr std::uint16_t CmdRDCOMM  = 0x0722;
inline constexpr std::uint16_t CmdSTCOMM  = 0x0723;

// ADCV / ADAX are not single 11-bit constants -- they're composed of
// the base plus mode (MD), discharge-permit (DCP), and channel (CH/CHG)
// bits. Helpers below construct the actual code at call time.

// Mode encoding shared by ADCV and ADAX.
//   00 = 422 Hz  (very slow, only the most-filtered)
//   01 =   27 Hz (fast)
//   10 =    7 Hz (normal -- the bring-up default)
//   11 =   26 Hz  (filtered, most accurate)
enum class AdcMode : std::uint8_t { Slow422Hz = 0, Fast27kHz = 1, Norm7kHz = 2, Filt26Hz = 3 };

// Cell channel selection for ADCV.
//   000 = all cells, 001 = cell 1+7, 010 = cell 2+8,...
enum class CellSel  : std::uint8_t { All = 0 };

// AUX channel selection for ADAX.
//   000 = all GPIOs + REF2, 001 = GPIO1, 010 = GPIO2,..., 110 = REF2
enum class AuxSel   : std::uint8_t { All = 0, Gpio1 = 1, Gpio2 = 2, Gpio3 = 3,
                                     Gpio4 = 4, Gpio5 = 5, Ref2  = 6 };

[[nodiscard]] inline std::uint16_t adcv_cmd(AdcMode mode, bool discharge_permit,
                                            CellSel cell = CellSel::All) noexcept {
    // Per the LTC6811-1 datasheet:
    //   ADCV = 0x0260 | (MD << 7) | (DCP << 4) | CH
    //   bits 9,6,5 are the fixed ADCV prefix (= 0x260)
    //   bits 8..7  carry MD[1:0]
    //   bit 4      carries DCP
    //   bits 2..0  carry the channel select
    // Worked example from the datasheet: MD=10 (7kHz), DCP=1, CH=000
    // -> 0x0260 | 0x100 | 0x010 | 0x000 = 0x0370.
    const std::uint16_t md_v  = static_cast<std::uint16_t>(mode);
    const std::uint16_t dcp_v = discharge_permit ? 1u : 0u;
    const std::uint16_t ch_v  = static_cast<std::uint16_t>(cell);
    return static_cast<std::uint16_t>(0x0260u | (md_v << 7) | (dcp_v << 4) | ch_v);
}

[[nodiscard]] inline std::uint16_t adax_cmd(AdcMode mode,
                                            AuxSel  aux = AuxSel::All) noexcept {
    // Per the LTC6811-1 datasheet:
    //   ADAX = 0x0460 | (MD << 7) | CHG
    //   bits 10,6,5 are the fixed ADAX prefix (= 0x460)
    //   bits 8..7  carry MD[1:0]
    //   bits 2..0  carry the channel-group select
    // Worked example: MD=10 (7kHz), CHG=000 (all GPIOs + REF2)
    // -> 0x0460 | 0x100 | 0x000 = 0x0560.
    const std::uint16_t md_v  = static_cast<std::uint16_t>(mode);
    const std::uint16_t chg_v = static_cast<std::uint16_t>(aux);
    return static_cast<std::uint16_t>(0x0460u | (md_v << 7) | chg_v);
}

// ADOW -- Start Open-Wire ADC Conversion (datasheet table 38 / "Open Wire
// Check"). Same 11-bit family as ADCV; PUP (pull-up/down current select) rides
// bit6, bit5 stays a fixed 1, and bit3 is set:
//   ADCV = 0 1 MD1 MD0 1   1 DCP 0 CH2 CH1 CH0   -> base 0x0260
//   ADOW = 0 1 MD1 MD0 PUP 1 DCP 1 CH2 CH1 CH0   -> base 0x0228
// so ADOW = 0x0228 | (MD << 7) | (PUP << 6) | (DCP << 4) | CH.
// Worked check: MD=10 (7 kHz), DCP=0, CH=000 -> PUP=1 gives 0x0228|0x100|0x040
// = 0x0368, PUP=0 gives 0x0328. Matches the Linduino LTC681x_adow reference
// (cmd = 0x0228 + (PUP<<6) + (DCP<<4) + CH, MD split across the two bytes).
// Two conversions (PUP=1 then PUP=0), each read back with the normal RDCV*
// groups, feed ams::open_wire::detect_open_conductors().
//
// HARDWARE-CONFIRMED (bench, feat/adow-raw-diagnostic): the previous encoding
// used base 0x0248 with PUP<<5, i.e. PUP and the fixed bit5 were SWAPPED. That
// left the PUP=1 pass correct by coincidence (0x0368) but emitted 0x0348 for
// PUP=0 -- b6 set (still pull-UP) and the fixed b5 clear -- which the LTC does
// not accept, so no second conversion ran and RDCV re-returned the pull-up
// result. The raw diag dump showed PU == PD bit-for-bit on all 95 cells, making
// the PU-PD delta identically 0 and CellOpenWire impossible to trigger.
[[nodiscard]] inline std::uint16_t adow_cmd(AdcMode mode, bool pull_up,
                                            bool discharge_permit,
                                            CellSel cell = CellSel::All) noexcept {
    const std::uint16_t md_v  = static_cast<std::uint16_t>(mode);
    const std::uint16_t pup_v = pull_up ? 1u : 0u;
    const std::uint16_t dcp_v = discharge_permit ? 1u : 0u;
    const std::uint16_t ch_v  = static_cast<std::uint16_t>(cell);
    return static_cast<std::uint16_t>(
        0x0228u | (md_v << 7) | (pup_v << 6) | (dcp_v << 4) | ch_v);
}

// ---------------------------------------------------------------------------
// Frame builders
// ---------------------------------------------------------------------------

// Pack an 11-bit command into the 4-byte command frame:
//   [cmd_hi, cmd_lo, pec_hi, pec_lo]
[[nodiscard]] std::array<std::uint8_t, 4> pack_command(std::uint16_t cmd) noexcept;

// For broadcast WRITE commands (WRCFGA, WRCOMM): build a frame of
//   [cmd(2) | pec(2) | (data(6) | pec(2)) * N]
// where N is LtcChainLength. The bottom of the chain is the first
// 8-byte group after the command frame (the LTC6811 shifts addresses
// "backwards" through the chain, so chain order is reversed on the
// wire relative to logical "module 0..4 / LTC 0..1" ordering -- see
// datasheet § "Daisy Chain"). For tests we don't care about reversal;
// the caller passes per-IC payloads in the order they want them on
// the wire.
void build_write_frame(std::uint16_t                       cmd,
                       const std::uint8_t                  per_ic_data[][6],
                       std::uint8_t*                       out,
                       std::size_t                         out_capacity) noexcept;

// For READ commands (RDCVx, RDAUXx, etc.): only the 4-byte command
// frame is built by the master. The slave chain shifts back
//   [(data(6) | pec(2)) * N]
// which the caller decodes with the per-register-group helpers below.

// ---------------------------------------------------------------------------
// Register-group decoders
//
// Each helper takes a pointer to the 8-byte chain segment (6 data + 2
// PEC), validates the PEC, and unpacks 3 cell voltages (in mV, units
// of 100 µV on the wire) or AUX voltages. Returns false on PEC fail;
// the out array is then untouched.
// ---------------------------------------------------------------------------
[[nodiscard]] bool decode_cell_voltage_group(const std::uint8_t*       bytes_8,
                                             std::array<std::uint16_t, 3>& out_mV) noexcept;

[[nodiscard]] bool decode_aux_voltage_group(const std::uint8_t*       bytes_8,
                                            std::array<std::uint16_t, 3>& out_mV) noexcept;

// ---------------------------------------------------------------------------
// ADG731 mux address packing for the LTC6811 WRCOMM register.
//
// The 32:1 mux on each BMS_LITE board is wired to the LTC's GPIO/COMM
// port (DIN / SCLK / SYNC bit-banged through WRCOMM + STCOMM). To
// select channel N (0..31) we transmit one 8-bit word (datasheet Rev.B
// Fig.3 + Table II), MSB..LSB:
//
//   DB7  EN       ACTIVE-LOW: 0 = enable addressed switch, 1 = ALL OFF
//   DB6  CS       0 = accept this write (1 = retain previous switch)
//   DB5  X        don't-care
//   DB4..DB0  A4..A0   channel 0..31
//
// i.e. the byte is simply the 5-bit address (EN/CS/X = 0).
//
// The LTC6811's COMM register is 6 bytes packed as 3 transmit "slots".
// Each slot has the form
//   pb[2k]   = ICOM[3:0] << 4 | data_hi_nibble
//   pb[2k+1] = data_lo_nibble << 4 | FCOM[3:0]
// We use slot 0 for the actual byte and mark slots 1 + 2 as "no
// transmission" (ICOM = 0xF).
//
// ICOM[3:0] = 0x8 -> CSBM driven LOW for this transmission
// ICOM[3:0] = 0xF -> no transmission for this slot
// FCOM[3:0] = 0x9 -> CSBM released HIGH after transmission
// FCOM[3:0] = 0xF -> no action at end of slot
// ---------------------------------------------------------------------------
[[nodiscard]] std::array<std::uint8_t, 6> pack_adg731_select(std::uint8_t channel) noexcept;

// ---------------------------------------------------------------------------
// Chain length discovery.
//
// The LTC6811-1 daisy-chain has no built-in addressing -- a single read
// command propagates through every IC and the slaves shift back N * 8
// bytes (6 data + 2 PEC per IC) in chain order. The only way to
// confirm "N ICs are alive and PEC-clean" is to issue a known low-
// impact read (RDCFGA is the usual choice -- it never triggers an ADC)
// and walk the reply counting consecutive PEC-valid 8-byte segments.
//
// count_pec_valid_segments stops on the first PEC-bad segment (or when
// it hits max_chain) and returns the count of valid ones in front of
// the failure. n_bytes is the size of the reply buffer (must be a
// multiple of 8 in well-formed callers, but the helper rounds down
// safely so a partial last segment is just ignored).
//
// This is pure logic, fully testable on host.
// ---------------------------------------------------------------------------
[[nodiscard]] std::uint8_t count_pec_valid_segments(const std::uint8_t* bytes,
                                                    std::size_t         n_bytes,
                                                    std::uint8_t        max_chain) noexcept;

// ---------------------------------------------------------------------------
// WRCFGA payload builder.
//
// CFGAR register layout (LTC6811 datasheet Table 39):
//   byte 0  GPIO5..GPIO1 pull-down disables | REFON | SWTRD | ADCOPT
//   byte 1  VUV[7:0]
//   byte 2  VOV[3:0] : VUV[11:8]
//   byte 3  VOV[11:4]
//   byte 4  DCC8..DCC1               (bit i = DCC{i+1})
//   byte 5  DCTO[3:0] : DCC12..DCC9  (low nibble = DCC{i+9})
//
// We disable the LTC's own UV/OV detection (kept in software via the
// safety predicates), enable REFON (faster ADC startup), and leave
// GPIO1..5 as inputs so the ADAX path keeps working. The DCTO
// timer is left at 0 -- each cycle we re-send WRCFGA explicitly so
// discharge can't latch beyond one balancing window.
//
// dcc_bits is a 12-bit mask: bit i = "discharge cell channel i+1".
// Bits above bit 11 are silently ignored.
[[nodiscard]] std::array<std::uint8_t, 6>
pack_cfga_payload(std::uint16_t dcc_bits) noexcept;

}  // namespace ams::ltc6811
