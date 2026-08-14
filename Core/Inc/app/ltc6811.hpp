// SPDX-License-Identifier: proprietary
//
// LTC6811-1 wire format: PEC15, command encodings, frame builders and
// register-group decoders. Pure logic, no HAL or SPI, so the host unit
// tests exercise all of it without FreeRTOS or hardware. The bus itself
// (LTC6820 master, real transactions) is ltc6820.hpp.
//
// References
// ----------
// - LTC6811-1 datasheet (pcbs/BMS_LITE/Datasheets/LTC6811HG-1.pdf)
//   § "Bus Protocol", § "Packet Error Code (PEC)",
//   § "Memory Map", § "Cell Voltage Register Group",
//   § "Auxiliary Register Group"
// - ADG731 datasheet (pcbs/BMS_LITE/Datasheets/ADG731.pdf) for the
//   3-wire SPI byte the LTC6811's GPIO/COMM bit-bang shifts out to
//   the mux.

#pragma once

#include "ams_config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ams::ltc6811 {

// ---------------------------------------------------------------------------
// PEC15 -- the 15-bit CRC the LTC6811 appends to every transaction.
//
//   polynomial  0x4599  (x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
//   seed        0x0010
//
// pec15() returns the remainder already shifted left by 1, ready to drop
// into the 2-byte on-wire slot, so its lowest bit is always 0.
// ---------------------------------------------------------------------------
[[nodiscard]] std::uint16_t pec15(const std::uint8_t* data,
                                  std::size_t       len) noexcept;

// ---------------------------------------------------------------------------
// Command codes (datasheet table 38). 11 bits each; on the wire a
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

// ADCV / ADAX are not fixed 11-bit constants: the base is OR-ed with
// mode (MD), discharge-permit (DCP) and channel (CH/CHG) bits. The
// helpers below build the code at call time.

// Mode encoding shared by ADCV and ADAX.
//   00 = 422 Hz  (slowest, most filtered)
//   01 =  27 kHz (fast)
//   10 =   7 kHz (normal -- what config::AdcMode selects)
//   11 =  26 Hz  (filtered, most accurate)
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
    // ADCV = 0x0260 | (MD << 7) | (DCP << 4) | CH   (LTC6811-1 datasheet)
    //   bits 9,6,5  fixed ADCV prefix (= 0x260)
    //   bits 8..7   MD[1:0]
    //   bit 4       DCP
    //   bits 2..0   channel select
    // Check: MD=10 (7 kHz), DCP=1, CH=000
    // -> 0x0260 | 0x100 | 0x010 | 0x000 = 0x0370.
    const std::uint16_t md_v  = static_cast<std::uint16_t>(mode);
    const std::uint16_t dcp_v = discharge_permit ? 1u : 0u;
    const std::uint16_t ch_v  = static_cast<std::uint16_t>(cell);
    return static_cast<std::uint16_t>(0x0260u | (md_v << 7) | (dcp_v << 4) | ch_v);
}

[[nodiscard]] inline std::uint16_t adax_cmd(AdcMode mode,
                                            AuxSel  aux = AuxSel::All) noexcept {
    // ADAX = 0x0460 | (MD << 7) | CHG   (LTC6811-1 datasheet)
    //   bits 10,6,5  fixed ADAX prefix (= 0x460)
    //   bits 8..7    MD[1:0]
    //   bits 2..0    channel-group select
    // Check: MD=10 (7 kHz), CHG=000 (all GPIOs + REF2)
    // -> 0x0460 | 0x100 | 0x000 = 0x0560.
    const std::uint16_t md_v  = static_cast<std::uint16_t>(mode);
    const std::uint16_t chg_v = static_cast<std::uint16_t>(aux);
    return static_cast<std::uint16_t>(0x0460u | (md_v << 7) | chg_v);
}

// ADOW -- Start Open-Wire ADC Conversion (datasheet table 38 / "Open Wire
// Check"). Same 11-bit family as ADCV, with PUP (pull-up/down current select)
// on bit 6, bit 5 a fixed 1, and bit 3 set:
//   ADCV = 0 1 MD1 MD0 1   1 DCP 0 CH2 CH1 CH0   -> base 0x0260
//   ADOW = 0 1 MD1 MD0 PUP 1 DCP 1 CH2 CH1 CH0   -> base 0x0228
// so ADOW = 0x0228 | (MD << 7) | (PUP << 6) | (DCP << 4) | CH.
// Check: MD=10 (7 kHz), DCP=0, CH=000 -> PUP=1 gives 0x0228|0x100|0x040 =
// 0x0368, PUP=0 gives 0x0328. Matches the Linduino LTC681x_adow reference
// (cmd = 0x0228 + (PUP<<6) + (DCP<<4) + CH, MD split across the two bytes).
// Two conversions (PUP=1 then PUP=0), each read back with the normal RDCV*
// groups, feed ams::open_wire::detect_open_conductors(). This encoding is
// bench-confirmed on a real chain.
//
// TRAP -- do NOT swap PUP with the fixed bit 5 (base 0x0248, PUP<<5). PUP=1
// still comes out 0x0368, so the pull-up pass looks correct, but PUP=0 emits
// 0x0348: bit 6 set (still pull-UP) and the fixed bit 5 clear, which the LTC
// rejects. No second conversion runs, RDCV re-returns the pull-up result, PU
// equals PD bit-for-bit on all 95 cells, the PU-PD delta is identically 0 and
// CellOpenWire can never trigger.
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

// Broadcast WRITE commands (WRCFGA, WRCOMM): build a frame of
//   [cmd(2) | pec(2) | (data(6) | pec(2)) * N],  N = LtcChainLength.
// The first 8-byte group after the command frame goes to the BOTTOM of
// the chain: the LTC6811 shifts addresses "backwards", so wire order is
// the reverse of logical "module 0..4 / LTC 0..1" order (datasheet
// § "Daisy Chain"). This builder does not reverse anything -- the caller
// passes per-IC payloads in the order it wants them on the wire.
void build_write_frame(std::uint16_t                       cmd,
                       const std::uint8_t                  per_ic_data[][6],
                       std::uint8_t*                       out,
                       std::size_t                         out_capacity) noexcept;

// READ commands (RDCVx, RDAUXx, etc.): the master builds only the
// 4-byte command frame; the slave chain shifts back
//   [(data(6) | pec(2)) * N]
// which the caller decodes with the register-group helpers below.

// ---------------------------------------------------------------------------
// Register-group decoders
//
// Each takes the 8-byte chain segment for one IC (6 data + 2 PEC),
// validates the PEC and unpacks 3 cell or AUX voltages into out_mV
// (the wire carries 100 µV units). Returns false on PEC fail, leaving
// out_mV untouched.
// ---------------------------------------------------------------------------
[[nodiscard]] bool decode_cell_voltage_group(const std::uint8_t*       bytes_8,
                                             std::array<std::uint16_t, 3>& out_mV) noexcept;

[[nodiscard]] bool decode_aux_voltage_group(const std::uint8_t*       bytes_8,
                                            std::array<std::uint16_t, 3>& out_mV) noexcept;

// ---------------------------------------------------------------------------
// ADG731 mux address packing for the LTC6811 WRCOMM register.
//
// The 32:1 mux on each BMS_LITE board hangs off the LTC's GPIO/COMM
// port (DIN / SCLK / SYNC bit-banged through WRCOMM + STCOMM).
// Selecting channel N (0..31) takes one 8-bit word (ADG731 datasheet
// Rev.B Fig.3 + Table II), MSB..LSB:
//
//   DB7       EN      ACTIVE-LOW: 0 = enable addressed switch, 1 = ALL OFF
//   DB6       CS      0 = accept this write (1 = retain previous switch)
//   DB5       X       don't-care
//   DB4..DB0  A4..A0  channel 0..31
//
// With EN/CS/X = 0 the byte is just the 5-bit address.
//
// The LTC6811's COMM register is 6 bytes = 3 transmit "slots":
//   pb[2k]   = ICOM[3:0] << 4 | data_hi_nibble
//   pb[2k+1] = data_lo_nibble << 4 | FCOM[3:0]
// Slot 0 carries the byte; slots 1 and 2 are marked "no transmission".
//
//   ICOM 0x8 -> drive CSBM LOW for this transmission
//   ICOM 0xF -> no transmission for this slot
//   FCOM 0x9 -> release CSBM HIGH after transmission
//   FCOM 0xF -> no action at end of slot
// ---------------------------------------------------------------------------
[[nodiscard]] std::array<std::uint8_t, 6> pack_adg731_select(std::uint8_t channel) noexcept;

// ---------------------------------------------------------------------------
// Chain length discovery.
//
// The LTC6811-1 daisy-chain has no addressing: one read command
// propagates through every IC and the slaves shift back N * 8 bytes
// (6 data + 2 PEC per IC) in chain order. So the only way to confirm
// "N ICs are alive and PEC-clean" is to issue a harmless read -- RDCFGA,
// which never triggers an ADC -- and count consecutive PEC-valid 8-byte
// segments in the reply.
//
// count_pec_valid_segments returns the number of valid segments before
// the first PEC-bad one, stopping there or at max_chain. n_bytes is the
// reply-buffer size; a trailing partial segment is ignored. Pure logic,
// fully host-testable.
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
// We disable the LTC's own UV/OV detection (the software safety
// predicates own it), set REFON for faster ADC startup, and leave
// GPIO1..5 as inputs so ADAX keeps working. DCTO stays 0: every cycle
// re-sends WRCFGA explicitly, so discharge cannot latch beyond one
// balancing window.
//
// dcc_bits is a 12-bit mask: bit i = "discharge cell channel i+1".
// Bits above bit 11 are silently ignored.
[[nodiscard]] std::array<std::uint8_t, 6>
pack_cfga_payload(std::uint16_t dcc_bits) noexcept;

}  // namespace ams::ltc6811
