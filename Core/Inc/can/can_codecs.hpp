// SPDX-License-Identifier: proprietary
//
// Code-first CAN codec layer (Phase 1 prototype, scope = 0x4A0 only).
//
// Each message .def is included four times here under different macro
// definitions. The four expansions produce: (1) a typed struct, (2) the
// firmware encoder, (3) the firmware decoder, (4) a runtime FieldDesc[]
// table that the host-side dbc_dump tool walks to emit a .dbc.
//
// Adding a field to a .def therefore makes a typed struct field, the
// matching encoder bit-pack, the matching decoder bit-unpack, and the
// matching DBC signal row all appear together. There is exactly one
// place the layout is written down -- this file's expansions are
// purely mechanical over it.
//
// The host-side parity test in tests/unit/test_dsl_parity_ams_status.cpp
// asserts byte-for-byte equality between encode_AMS_status() here and
// ams::telemetry::encode_status() in Core/Inc/app/telemetry_encoders.hpp.

#ifndef AMS_CAN_CODECS_HPP_
#define AMS_CAN_CODECS_HPP_

#include "can_dsl.hpp"

namespace ifs08 {

// ---- pass 1: typed structs -------------------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period)     struct Name##_t {
#define CAN_MSG_END(Name)                          };
#define FIELD_LE(name, ctype, byte, len, f, o, u)   ctype name {};
#define FIELD_BE(name, ctype, byte, len, f, o, u)   ctype name {};
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) ctype name {};
#include "messages/ams_status.def"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_BE
#undef FIELD_BE_S

// ---- pass 2: encode (struct -> bytes) --------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void encode_##Name(const Name##_t& in, uint8_t (&d)[8]) noexcept { \
        for (auto& b : d) b = 0;
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    can_dsl::set_le(d, 8u*(byte),    len, \
        static_cast<uint64_t>(in.name) & (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1)));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    can_dsl::set_be(d, 8u*(byte)+7u, len, \
        static_cast<uint64_t>(in.name) & (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1)));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    can_dsl::set_be(d, 8u*(byte)+7u, len, \
        static_cast<uint64_t>(in.name) & (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1)));
#include "messages/ams_status.def"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_BE
#undef FIELD_BE_S

// ---- pass 3: decode (bytes -> struct) --------------------------------------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    inline void decode_##Name(const uint8_t (&d)[8], Name##_t& out) noexcept {
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_le(d, 8u*(byte),    len));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>(can_dsl::get_be(d, 8u*(byte)+7u, len));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    out.name = static_cast<ctype>( \
        can_dsl::sign_extend(can_dsl::get_be(d, 8u*(byte)+7u, len), len));
#include "messages/ams_status.def"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_BE
#undef FIELD_BE_S

// ---- pass 4: runtime descriptors (host-side dbc_dump iterates these) -------
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    static const can_dsl::FieldDesc Name##_fields[] = {
#define CAN_MSG_END(Name) };
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte),    len, false, false, static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  false, static_cast<double>(f), static_cast<double>(o), u },
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  true,  static_cast<double>(f), static_cast<double>(o), u },
#include "messages/ams_status.def"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_BE
#undef FIELD_BE_S

#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    { #Name, (Id), (Dlc), (Sender), (Period), Name##_fields, \
      sizeof(Name##_fields)/sizeof(can_dsl::FieldDesc) },
#define CAN_MSG_END(Name)
#define FIELD_LE(name, ctype, byte, len, f, o, u)
#define FIELD_BE(name, ctype, byte, len, f, o, u)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)
static const can_dsl::MsgDesc ALL_MSGS[] = {
#include "messages/ams_status.def"
};
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_BE
#undef FIELD_BE_S

static const unsigned ALL_MSGS_COUNT = sizeof(ALL_MSGS) / sizeof(can_dsl::MsgDesc);

}  // namespace ifs08

#endif  // AMS_CAN_CODECS_HPP_
