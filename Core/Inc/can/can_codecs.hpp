// SPDX-License-Identifier: proprietary
//
// Code-first CAN codec layer. The message registry
// (Core/Inc/can/messages/all_messages.inc) is #included once per
// expansion pass below, under different macro definitions. The passes
// produce, for every message: (1) a typed struct, (2) the firmware
// encoder, (3) the firmware decoder, (4) a runtime FieldDesc[] table the
// host-side dbc_dump tool walks to emit a .dbc, and (5) a compile-time
// bit-overlap guard. There is exactly ONE place each layout is written
// down -- its .def -- and all of these derive from it mechanically, so a
// field add / width change / endian flip moves the struct, encoder,
// decoder and DBC row together. No second source of truth, no C++<->DBC
// drift (which gen_dbc.py's separate Python re-declaration allowed).
//
// Per-message byte-for-byte parity with the hand-rolled encoders is
// asserted in tests/unit/test_dsl_parity.cpp; the descriptors are
// checked against the committed DBC in test_dsl_dbc_consistency.cpp.
//
// Field macros (byte-aligned position given as a BYTE index):
//   FIELD_LE      little-endian, unsigned
//   FIELD_LE_S    little-endian, signed (sign-extended on decode)
//   FIELD_BE      big-endian (DBC/Motorola sawtooth), unsigned
//   FIELD_BE_S    big-endian, signed
// Sub-byte / unaligned fields (position given as an absolute START BIT,
// DBC convention -- LE: 8*byte+bit; BE: the MSB bit = 8*byte+7-...):
//   FIELD_LE_BITS little-endian, unsigned, arbitrary start_bit/len
//   FIELD_BE_BITS big-endian,    unsigned, arbitrary start_bit/len
//
// CONVENTION -- the struct holds RAW WIRE INTEGERS. The (factor, offset)
// args are DBC-display metadata ONLY; they are emitted into the FieldDesc
// for the DBC and are NEVER applied by the C++ encoder/decoder. Any
// physical<->raw scaling (mA->deciamps, int8 temp clip, etc.) happens in
// the adapter (telemetry_encoders.hpp / acu_tx_encoders.hpp / the
// pit-diag adapter) BEFORE the struct is populated.

#ifndef AMS_CAN_CODECS_HPP_
#define AMS_CAN_CODECS_HPP_

#include "can_dsl.hpp"

namespace ifs08 {

// Mask helper: low `len` bits, guarding the len==64 UB of (1<<64).
#define AMS_DSL_MASK(len) (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1))

// ---- pass 1: typed structs (+ per-field width static_assert) ---------------
// static_assert catches FIELD_LE(x, uint8_t, 0, 16, ...) typos at compile time.
#define CAN_MSG(Name, Id, Dlc, Sender, Period)      struct Name##_t {
#define CAN_MSG_END(Name)                           };
#define AMS_DSL_W(name, ctype, len) ctype name {}; \
    static_assert((len) <= 8u*sizeof(ctype), "DSL " #name ": bit length exceeds " #ctype " width");
#define FIELD_LE(name, ctype, byte, len, f, o, u)        AMS_DSL_W(name, ctype, len)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)      AMS_DSL_W(name, ctype, len)
#define FIELD_BE(name, ctype, byte, len, f, o, u)        AMS_DSL_W(name, ctype, len)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)      AMS_DSL_W(name, ctype, len)
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)  AMS_DSL_W(name, ctype, len)
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)  AMS_DSL_W(name, ctype, len)
#include "messages/all_messages.inc"
#undef AMS_DSL_W
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

// ---- pass 2: encode (struct -> bytes) --------------------------------------
// The encoder takes a sized array reference so the DLC is enforced at the
// callsite -- passing a 4-byte buffer to a Dlc=6 encoder fails to compile.
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void encode_##Name(const Name##_t& in, uint8_t (&d)[Dlc]) noexcept { \
        for (auto& b : d) b = 0;
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    can_dsl::set_le(d, 8u*(byte), len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    can_dsl::set_le(d, 8u*(byte), len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    can_dsl::set_be(d, 8u*(byte)+7u, len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    can_dsl::set_be(d, 8u*(byte)+7u, len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    can_dsl::set_le(d, (start), len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    can_dsl::set_be(d, (start), len, static_cast<uint64_t>(in.name) & AMS_DSL_MASK(len));
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

// ---- pass 3: decode (bytes -> struct) --------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void decode_##Name(const uint8_t (&d)[Dlc], Name##_t& out) noexcept {
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_le(d, 8u*(byte), len));
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>( \
        can_dsl::sign_extend(can_dsl::get_le(d, 8u*(byte), len), len));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_be(d, 8u*(byte)+7u, len));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>( \
        can_dsl::sign_extend(can_dsl::get_be(d, 8u*(byte)+7u, len), len));
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_le(d, (start), len));
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_be(d, (start), len));
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

// ---- pass 4: runtime descriptors (host-side dbc_dump iterates these) -------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    static const can_dsl::FieldDesc Name##_fields[] = {
#define CAN_MSG_END(Name) };
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte),    len, false, false, static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte),    len, false, true,  static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  false, static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  true,  static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    { #name, (start),      len, false, false, static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    { #name, (start),      len, true,  false, static_cast<double>(f), static_cast<double>(o), u },
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    { #Name, (Id), (Dlc), (Sender), (Period), Name##_fields, \
      sizeof(Name##_fields)/sizeof(can_dsl::FieldDesc) },
#define CAN_MSG_END(Name)
#define FIELD_LE(name, ctype, byte, len, f, o, u)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)
#define FIELD_BE(name, ctype, byte, len, f, o, u)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)
static const can_dsl::MsgDesc ALL_MSGS[] = {
#include "messages/all_messages.inc"
};
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

static const unsigned ALL_MSGS_COUNT = sizeof(ALL_MSGS) / sizeof(can_dsl::MsgDesc);

// ---- pass 5: per-message bit-overlap guard (compile-time) ------------------
// Each field ORs its claimed frame-bit mask; a static_assert fires if two
// fields claim the same bit (e.g. a copy-paste byte index, or a BITS field
// colliding with a byte-aligned one). Gaps are allowed -- reserved bytes
// are legitimate.
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    constexpr bool Name##_dsl_no_overlap() noexcept { \
        uint64_t claimed = 0; bool ok = true;
#define CAN_MSG_END(Name) \
        return ok; \
    } \
    static_assert(Name##_dsl_no_overlap(), "DSL field bit-overlap in " #Name);
#define AMS_DSL_CLAIM(m) { const uint64_t _m = (m); if (claimed & _m) ok = false; claimed |= _m; }
#define FIELD_LE(name, ctype, byte, len, f, o, u)        AMS_DSL_CLAIM(can_dsl::bitmask_le(8u*(byte),     len))
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)      AMS_DSL_CLAIM(can_dsl::bitmask_le(8u*(byte),     len))
#define FIELD_BE(name, ctype, byte, len, f, o, u)        AMS_DSL_CLAIM(can_dsl::bitmask_be(8u*(byte)+7u,  len))
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)      AMS_DSL_CLAIM(can_dsl::bitmask_be(8u*(byte)+7u,  len))
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)  AMS_DSL_CLAIM(can_dsl::bitmask_le((start),       len))
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)  AMS_DSL_CLAIM(can_dsl::bitmask_be((start),       len))
#include "messages/all_messages.inc"
#undef AMS_DSL_CLAIM
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

#undef AMS_DSL_MASK

}  // namespace ifs08

#endif  // AMS_CAN_CODECS_HPP_
