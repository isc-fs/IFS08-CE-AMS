// SPDX-License-Identifier: proprietary

#include "ltc6811.hpp"

#include <cstring>

namespace ams::ltc6811 {

// ---------------------------------------------------------------------------
// PEC15 lookup table -- generated at compile time so no runtime init
// needed and so we get a single read-only blob in flash.
//
// The polynomial 0x4599 represents x^15 + x^14 + x^10 + x^8 + x^7 +
// x^4 + x^3 + 1 (the +1 / x^0 term is implicit in CRC bookkeeping).
// ---------------------------------------------------------------------------
namespace {

constexpr std::uint16_t Crc15Poly = 0x4599u;

constexpr std::array<std::uint16_t, 256> make_pec15_table() {
    std::array<std::uint16_t, 256> t = {};
    for (int i = 0; i < 256; ++i) {
        std::uint16_t rem = static_cast<std::uint16_t>(i << 7);
        for (int bit = 0; bit < 8; ++bit) {
            if (rem & 0x4000u) {
                rem = static_cast<std::uint16_t>((rem << 1) ^ Crc15Poly);
            } else {
                rem = static_cast<std::uint16_t>(rem << 1);
            }
        }
        t[i] = static_cast<std::uint16_t>(rem & 0xFFFFu);
    }
    return t;
}

constexpr auto Pec15Table = make_pec15_table();

// Convert the high byte of one of the COMM-register packed slots from
// (ICOM[3:0], data_hi[3:0]). Reverse helper for tests.
[[nodiscard]] constexpr std::uint8_t comm_byte_hi(std::uint8_t icom, std::uint8_t data_hi) {
    return static_cast<std::uint8_t>((icom << 4) | (data_hi & 0x0Fu));
}
[[nodiscard]] constexpr std::uint8_t comm_byte_lo(std::uint8_t data_lo, std::uint8_t fcom) {
    return static_cast<std::uint8_t>(((data_lo & 0x0Fu) << 4) | (fcom & 0x0Fu));
}

}  // namespace

std::uint16_t pec15(const std::uint8_t* data, std::size_t len) noexcept {
    std::uint16_t rem = 0x0010u;
    for (std::size_t i = 0; i < len; ++i) {
        const std::uint8_t addr =
            static_cast<std::uint8_t>(((rem >> 7) ^ data[i]) & 0xFFu);
        rem = static_cast<std::uint16_t>((rem << 8) ^ Pec15Table[addr]);
    }
    // The LTC6811 transmits the 15-bit remainder shifted up by 1 so
    // the LSB on the wire is always 0.
    return static_cast<std::uint16_t>(rem << 1);
}

std::array<std::uint8_t, 4> pack_command(std::uint16_t cmd) noexcept {
    std::array<std::uint8_t, 4> frame = {};
    frame[0] = static_cast<std::uint8_t>((cmd >> 8) & 0xFFu);
    frame[1] = static_cast<std::uint8_t>(cmd & 0xFFu);
    const std::uint16_t crc = pec15(frame.data(), 2);
    frame[2] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    frame[3] = static_cast<std::uint8_t>(crc & 0xFFu);
    return frame;
}

void build_write_frame(std::uint16_t        cmd,
                       const std::uint8_t   per_ic_data[][6],
                       std::uint8_t*        out,
                       std::size_t          out_capacity) noexcept {
    const std::size_t needed = 4 + 8 * config::LtcChainLength;
    if (out_capacity < needed) {
        // Caller passed too small a buffer; refuse to write anything.
        // (Asserting would be nicer but this is a no-exception build.)
        return;
    }
    const auto cmd_frame = pack_command(cmd);
    std::memcpy(out, cmd_frame.data(), 4);
    for (std::size_t i = 0; i < config::LtcChainLength; ++i) {
        std::uint8_t* segment = out + 4 + 8 * i;
        std::memcpy(segment, per_ic_data[i], 6);
        const std::uint16_t crc = pec15(segment, 6);
        segment[6] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
        segment[7] = static_cast<std::uint8_t>(crc & 0xFFu);
    }
}

namespace {

// Shared helper for the cell + AUX voltage decoders. Both register
// groups are laid out the same way: 3 × 2-byte little-endian samples
// in units of 100 µV, followed by 2 bytes of PEC.
[[nodiscard]] bool decode_voltage_group(const std::uint8_t*           bytes_8,
                                        std::array<std::uint16_t, 3>& out_mV) noexcept {
    const std::uint16_t expected =
        static_cast<std::uint16_t>((bytes_8[6] << 8) | bytes_8[7]);
    if (pec15(bytes_8, 6) != expected) {
        return false;
    }
    for (std::size_t k = 0; k < 3; ++k) {
        const std::uint16_t raw_100uV =
            static_cast<std::uint16_t>(bytes_8[2 * k] |
                                       (bytes_8[2 * k + 1] << 8));
        // 100 µV -> mV  ==>  divide by 10. Rounding doesn't matter at
        // this scale (the LTC6811 LSB is 0.1 mV, ten times below cell
        // measurement noise).
        out_mV[k] = static_cast<std::uint16_t>((raw_100uV + 5) / 10);
    }
    return true;
}

}  // namespace

bool decode_cell_voltage_group(const std::uint8_t*           bytes_8,
                               std::array<std::uint16_t, 3>& out_mV) noexcept {
    return decode_voltage_group(bytes_8, out_mV);
}

bool decode_aux_voltage_group(const std::uint8_t*           bytes_8,
                              std::array<std::uint16_t, 3>& out_mV) noexcept {
    return decode_voltage_group(bytes_8, out_mV);
}

std::array<std::uint8_t, 6> pack_adg731_select(std::uint8_t channel) noexcept {
    // ADG731 selection byte: EN | x | A4..A0 | x
    const std::uint8_t data = static_cast<std::uint8_t>(
        0x80u | ((channel & 0x1Fu) << 1));

    std::array<std::uint8_t, 6> p = {};
    // Slot 0 -- send the byte, drive CSBM LOW during transmission and
    // release HIGH after.
    p[0] = comm_byte_hi(/*icom=*/0x8u, static_cast<std::uint8_t>(data >> 4));
    p[1] = comm_byte_lo(static_cast<std::uint8_t>(data & 0x0Fu), /*fcom=*/0x9u);
    // Slots 1 + 2 -- no transmission.
    p[2] = comm_byte_hi(/*icom=*/0xFu, 0x0u);
    p[3] = comm_byte_lo(0x0u, /*fcom=*/0xFu);
    p[4] = comm_byte_hi(/*icom=*/0xFu, 0x0u);
    p[5] = comm_byte_lo(0x0u, /*fcom=*/0xFu);
    return p;
}

std::array<std::uint8_t, 6> pack_cfga_payload(std::uint16_t dcc_bits) noexcept {
    std::array<std::uint8_t, 6> cfg = {};
    // byte 0: GPIO5..GPIO1 pull-downs disabled (bits 7..3 = 1) so the
    // ADAX path keeps reading the buffered mux output; REFON = 1 for
    // fast ADC startup; SWTRD = 0, ADCOPT = 0.
    cfg[0] = 0xFCu;
    // VUV = 0, VOV = 0xFFF -> LTC's onboard UV/OV detection effectively
    // disabled (safety predicates do this in software).
    cfg[1] = 0x00u;                       // VUV[7:0]
    cfg[2] = 0xF0u;                       // VOV[3:0]=F | VUV[11:8]=0
    cfg[3] = 0xFFu;                       // VOV[11:4]=FF
    cfg[4] = static_cast<std::uint8_t>(dcc_bits & 0xFFu);            // DCC1..DCC8
    cfg[5] = static_cast<std::uint8_t>((dcc_bits >> 8) & 0x0Fu);     // DCC9..DCC12 (DCTO=0)
    return cfg;
}

std::uint8_t count_pec_valid_segments(const std::uint8_t* bytes,
                                      std::size_t         n_bytes,
                                      std::uint8_t        max_chain) noexcept {
    if (bytes == nullptr) {
        return 0;
    }
    const std::size_t segments = n_bytes / 8;
    const std::size_t cap      = (segments < max_chain) ? segments
                                                        : static_cast<std::size_t>(max_chain);
    std::uint8_t valid = 0;
    for (std::size_t i = 0; i < cap; ++i) {
        const std::uint8_t* seg = bytes + 8 * i;
        const std::uint16_t expected =
            static_cast<std::uint16_t>((seg[6] << 8) | seg[7]);
        if (pec15(seg, 6) != expected) {
            break;
        }
        ++valid;
    }
    return valid;
}

}  // namespace ams::ltc6811
