// SPDX-License-Identifier: proprietary
//
// CRC-32 (ISO-HDLC, a.k.a. zlib/PKZIP) for log-file integrity.
//
// WHY SOFTWARE, on a chip that has a CRC peripheral
// -------------------------------------------------
// The STM32H7 CRC unit defaults to CRC-32/MPEG-2: same polynomial, but NOT
// reflected and no final XOR. That is a different checksum, and the host tools
// verify with Python's zlib.crc32. Matching the clients matters more than
// saving cycles here, so this is computed in software:
//
//   poly 0x04C11DB7 reflected -> 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF
//
// (The peripheral CAN be configured to match, but it is a shared resource and
// the log CRC is accumulated incrementally across seconds of writes -- holding
// peripheral state across that would couple the logger to anything else that
// ever wants a CRC.)
//
// A 16-entry nibble table: 64 bytes of flash and 2 lookups per byte. The
// 256-entry version costs 1 KiB to roughly double the speed, which this does
// not need -- the CRC is folded in as rows are written, never in a burst.
//
// Pure logic: no HAL, no RTOS, host-testable.

#pragma once

#include <cstddef>
#include <cstdint>

namespace ams::crc {

inline constexpr std::uint32_t Crc32Init  = 0xFFFFFFFFu;
inline constexpr std::uint32_t Crc32Xor   = 0xFFFFFFFFu;

// Reflected-poly nibble table (0xEDB88320).
inline constexpr std::uint32_t kNibbleTable[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

// Fold `len` bytes into a running CRC. Pass Crc32Init to start; feed the
// result back in to continue across calls (this is how the logger accumulates
// a file's CRC as rows are written, without re-reading it at seal).
//
// The value carried between calls is the RAW running state -- Crc32Xor has
// NOT been applied. Call finalize() once, at the end, to get the wire value.
[[nodiscard]] inline std::uint32_t update(std::uint32_t     crc,
                                          const void*       data,
                                          std::size_t       len) noexcept {
    const auto* p = static_cast<const std::uint8_t*>(data);
    if (p == nullptr) return crc;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        crc = kNibbleTable[crc & 0x0Fu] ^ (crc >> 4);
        crc = kNibbleTable[crc & 0x0Fu] ^ (crc >> 4);
    }
    return crc;
}

// Turn a running state into the published value.
[[nodiscard]] inline std::uint32_t finalize(std::uint32_t crc) noexcept {
    return crc ^ Crc32Xor;
}

// One-shot: equivalent to Python zlib.crc32(data).
[[nodiscard]] inline std::uint32_t compute(const void* data, std::size_t len) noexcept {
    return finalize(update(Crc32Init, data, len));
}

}  // namespace ams::crc
