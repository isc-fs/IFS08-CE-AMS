// SPDX-License-Identifier: proprietary
//
// Bit-level pack/unpack helpers + runtime descriptor types used by the
// code-first CAN DSL. This header is shared between the firmware (which
// uses encode/decode) and the host-side dbc_dump tool (which iterates
// the runtime FieldDesc tables).
//
// Nothing here is message-specific. All message layouts live in
// Core/Inc/can/messages/*.def and are #included by Core/Inc/can/can_codecs.hpp.

#ifndef AMS_CAN_DSL_HPP_
#define AMS_CAN_DSL_HPP_

#include <cstdint>

namespace can_dsl {

// ---- Bit traversals --------------------------------------------------------
//
// LE (Intel): value bit i lives at frame bit (start + i). Simple linear.
//
// BE (Motorola Forward MSB, DBC sawtooth): MSB of the value lives at
// frame bit `start` (= 8*byte + 7 for byte-aligned fields). Bits descend
// within a byte (towards bit 0), then jump to bit 7 of the next byte.
// This matches the convention cantools / Vector CANdb++ use.

inline uint64_t get_le(const uint8_t* d, unsigned start, unsigned len) noexcept {
    uint64_t v = 0;
    for (unsigned i = 0; i < len; ++i) {
        const unsigned bit = start + i;
        v |= static_cast<uint64_t>((d[bit >> 3] >> (bit & 7)) & 1u) << i;
    }
    return v;
}

inline void set_le(uint8_t* d, unsigned start, unsigned len, uint64_t v) noexcept {
    for (unsigned i = 0; i < len; ++i) {
        const unsigned bit = start + i;
        const uint8_t b = static_cast<uint8_t>((v >> i) & 1u);
        d[bit >> 3] = static_cast<uint8_t>(
            (d[bit >> 3] & ~(1u << (bit & 7))) | (b << (bit & 7)));
    }
}

inline uint64_t get_be(const uint8_t* d, unsigned start, unsigned len) noexcept {
    uint64_t v = 0;
    unsigned bit = start;
    for (unsigned i = 0; i < len; ++i) {
        v = (v << 1) | ((d[bit >> 3] >> (bit & 7)) & 1u);
        if ((bit & 7) == 0) bit += 15;
        else                bit -= 1;
    }
    return v;
}

inline void set_be(uint8_t* d, unsigned start, unsigned len, uint64_t v) noexcept {
    unsigned bit = start;
    for (unsigned i = 0; i < len; ++i) {
        const uint8_t b = static_cast<uint8_t>((v >> (len - 1 - i)) & 1u);
        d[bit >> 3] = static_cast<uint8_t>(
            (d[bit >> 3] & ~(1u << (bit & 7))) | (b << (bit & 7)));
        if ((bit & 7) == 0) bit += 15;
        else                bit -= 1;
    }
}

inline int64_t sign_extend(uint64_t v, unsigned len) noexcept {
    const uint64_t m = 1ull << (len - 1);
    return static_cast<int64_t>((v ^ m) - m);
}

// ---- Compile-time bit-claim masks (for the per-message overlap guard) ------
//
// Return the set of FRAME bit positions a field occupies, as a 64-bit
// mask (bit = 8*byte + intra-byte position). LE is linear from `start`;
// BE walks the DBC sawtooth identically to set_be/get_be. constexpr so
// can_codecs.hpp's pass-5 static_assert can fold it at compile time.

constexpr uint64_t bitmask_le(unsigned start, unsigned len) noexcept {
    uint64_t m = 0;
    for (unsigned i = 0; i < len; ++i) m |= uint64_t{1} << (start + i);
    return m;
}

constexpr uint64_t bitmask_be(unsigned start, unsigned len) noexcept {
    uint64_t m = 0;
    unsigned bit = start;
    for (unsigned i = 0; i < len; ++i) {
        m |= uint64_t{1} << bit;
        if ((bit & 7) == 0) bit += 15;
        else                bit -= 1;
    }
    return m;
}

// ---- Array-of-frames (multi-ID window) support -----------------------------
//
// Some pit-diag streams are an ARRAY windowed across consecutive frame
// IDs: frame N (id = base_id + N) carries per_frame elements
// [per_frame*N .. per_frame*N + per_frame). The byte layout of one
// window is uniform, so it lives ONCE here -- encode_array_window for the
// firmware, ArrayMsgDesc for the host dbc_dump expansion (which emits one
// DBC message per frame). The element SOURCE mapping (which [m][c] a flat
// index is) and any value transform stay in the firmware wrapper; only
// the wire layout is shared.

// Encode the `frame_idx` window into d[8]. `accessor(flat)` returns the
// already-transformed raw wire integer for a flat element index; slots
// past `total` get `sentinel`. byte stride per slot = elem_bits/8.
template <typename Acc>
inline void encode_array_window(unsigned frame_idx, unsigned per_frame,
                                unsigned elem_bits, bool big_endian,
                                unsigned total, uint64_t sentinel,
                                Acc accessor, uint8_t* d) noexcept {
    for (unsigned i = 0; i < 8; ++i) d[i] = 0;
    const unsigned elem_bytes = elem_bits / 8u;
    for (unsigned slot = 0; slot < per_frame; ++slot) {
        const unsigned flat = per_frame * frame_idx + slot;
        const uint64_t v = (flat < total) ? accessor(flat) : sentinel;
        const unsigned byte = slot * elem_bytes;
        if (big_endian) set_be(d, 8u * byte + 7u, elem_bits, v);
        else            set_le(d, 8u * byte,      elem_bits, v);
    }
}

struct ArrayMsgDesc {
    const char* sender;
    uint32_t    base_id;
    const char* msg_name_fmt;       // printf(frame_idx)        e.g. "PitDiag_cells_%02u"
    unsigned    frame_count;
    unsigned    per_frame;
    unsigned    elem_bits;
    bool        big_endian;
    bool        is_signed;
    unsigned    inner_dim;          // module = flat/inner_dim, elem = flat%inner_dim
    unsigned    total_elems;
    const char* sig_name_fmt;       // printf(module, elem)     e.g. "cell_m%u_c%02u_mV"
    const char* sentinel_name_fmt;  // printf(slot) or nullptr  e.g. "sentinel_slot%u"
    const char* unit;
    long        dbc_min;
    long        dbc_max;
    unsigned    period_ms;
};

// ---- Runtime descriptors (read by the host dbc_dump tool) ------------------

struct FieldDesc {
    const char* name;
    unsigned    start_bit;   // already in DBC convention (LE: 8*byte; BE: 8*byte+7)
    unsigned    len;
    bool        big_endian;
    bool        is_signed;
    double      factor;
    double      offset;
    const char* unit;
};

struct MsgDesc {
    const char*      name;
    uint32_t         id;
    unsigned         dlc;
    const char*      sender;
    unsigned         period_ms;
    const FieldDesc* fields;
    unsigned         n_fields;
};

}  // namespace can_dsl

#endif  // AMS_CAN_DSL_HPP_
