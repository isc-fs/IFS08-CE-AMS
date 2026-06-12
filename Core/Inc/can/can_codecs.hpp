// SPDX-License-Identifier: proprietary
//
// Code-first CAN codec layer (Phase 2a: the three AMS telemetry frames
// 0x4A0 / 0x4A1 / 0x4A2).
//
// The message registry (Core/Inc/can/messages/all_messages.inc) is
// #included once per expansion pass below, under different macro
// definitions. The passes produce, for every message: (1) a typed
// struct, (2) the firmware encoder, (3) the firmware decoder, (4) a
// runtime FieldDesc[] table the host-side dbc_dump tool walks to emit a
// .dbc. There is exactly ONE place each layout is written down -- its
// .def -- and all four artefacts derive from it mechanically, so a
// field add / width change / endian flip moves the struct, encoder,
// decoder and DBC row together. No second source of truth, no C++<->DBC
// drift (which gen_dbc.py's separate Python re-declaration allows today).
//
// Per-message byte-for-byte parity with the hand-rolled encoders is
// asserted in tests/unit/test_dsl_parity.cpp.
//
// Field macros:
//   FIELD_LE   - little-endian, unsigned
//   FIELD_LE_S - little-endian, signed (sign-extended on decode)
//   FIELD_BE   - big-endian (DBC/Motorola sawtooth), unsigned
//   FIELD_BE_S - big-endian, signed

#ifndef AMS_CAN_CODECS_HPP_
#define AMS_CAN_CODECS_HPP_

#include "can_dsl.hpp"

namespace ifs08 {

// Mask helper: low `len` bits, guarding the len==64 UB of (1<<64).
#define AMS_DSL_MASK(len) (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1))

// ---- pass 1: typed structs -------------------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period)      struct Name##_t {
#define CAN_MSG_END(Name)                           };
#define FIELD_LE(name, ctype, byte, len, f, o, u)   ctype name {};
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) ctype name {};
#define FIELD_BE(name, ctype, byte, len, f, o, u)   ctype name {};
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) ctype name {};
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S

// ---- pass 2: encode (struct -> bytes) --------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void encode_##Name(const Name##_t& in, uint8_t (&d)[8]) noexcept { \
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
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S

// ---- pass 3: decode (bytes -> struct) --------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void decode_##Name(const uint8_t (&d)[8], Name##_t& out) noexcept {
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
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S

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
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S

#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    { #Name, (Id), (Dlc), (Sender), (Period), Name##_fields, \
      sizeof(Name##_fields)/sizeof(can_dsl::FieldDesc) },
#define CAN_MSG_END(Name)
#define FIELD_LE(name, ctype, byte, len, f, o, u)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)
#define FIELD_BE(name, ctype, byte, len, f, o, u)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)
static const can_dsl::MsgDesc ALL_MSGS[] = {
#include "messages/all_messages.inc"
};
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S

static const unsigned ALL_MSGS_COUNT = sizeof(ALL_MSGS) / sizeof(can_dsl::MsgDesc);

#undef AMS_DSL_MASK

}  // namespace ifs08

#endif  // AMS_CAN_CODECS_HPP_
