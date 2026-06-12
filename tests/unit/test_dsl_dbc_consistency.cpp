// SPDX-License-Identifier: proprietary
//
// The C++ <-> DBC drift guard (Phase 2a, slice 3/3).
//
// For every message the code-first DSL owns (ifs08::ALL_MSGS), this
// asserts that each generated FieldDesc matches the signal in the
// committed docs/dbc/ams.dbc -- same bit position, length, byte order
// and sign. Signals are matched by START BIT, not name, because the DBC
// occasionally uses a more descriptive name than the .def (e.g.
// reserved_b3 vs app_init_progress_or_reserved).
//
// Today gen_dbc.py still OWNS the DBC (it re-declares every message in
// Python), and only the three telemetry frames are on the DSL -- so this
// can't yet replace the "DBC matches generator" check. What it adds is
// the guard that NEVER existed: that the firmware's actual encoder layout
// (the .def the C++ encoder is generated from) agrees with the DBC the
// ECU/dash consume. Once all messages are ported, dbc_dump replaces
// gen_dbc.py and this becomes the whole check.

#include "can/can_codecs.hpp"

#include "unity.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

struct DbcSig { unsigned start, len; char order, sign; bool found; };

// Find, in the committed DBC, the signal at `start_bit` inside message
// `id`. Returns found=false if the BO_ or a signal at that start is
// absent. Hand-parsed (no <regex>: the host test build is -fno-exceptions
// and std::regex throws).
DbcSig find_dbc_signal(std::uint32_t id, unsigned start_bit) {
    DbcSig out{0, 0, 0, 0, false};
    std::FILE* fp = std::fopen(AMS_DBC_PATH, "r");
    if (!fp) return out;

    char line[512];
    bool in_msg = false;
    while (std::fgets(line, sizeof(line), fp)) {
        if (std::strncmp(line, "BO_ ", 4) == 0) {
            unsigned long mid = 0;
            std::sscanf(line, "BO_ %lu", &mid);
            in_msg = (static_cast<std::uint32_t>(mid) == id);
            continue;
        }
        if (in_msg && std::strncmp(line, " SG_ ", 5) == 0) {
            char name[96] = {0};
            unsigned s = 0, l = 0; char o = 0, sg = 0;
            // " SG_ <name> : <start>|<len>@<order><sign> (...)"
            if (std::sscanf(line, " SG_ %95s : %u|%u@%c%c", name, &s, &l, &o, &sg) == 5) {
                if (s == start_bit) { out = {s, l, o, sg, true}; break; }
            }
        }
    }
    std::fclose(fp);
    return out;
}

}  // namespace

extern "C" void test_dsl_descriptors_match_committed_dbc(void) {
    TEST_ASSERT_GREATER_THAN_UINT(0u, ifs08::ALL_MSGS_COUNT);

    for (unsigned mi = 0; mi < ifs08::ALL_MSGS_COUNT; ++mi) {
        const auto& m = ifs08::ALL_MSGS[mi];
        for (unsigned fi = 0; fi < m.n_fields; ++fi) {
            const auto& fd = m.fields[fi];
            const DbcSig sig = find_dbc_signal(m.id, fd.start_bit);

            char msg[160];
            std::snprintf(msg, sizeof(msg),
                "msg 0x%lX field '%s' @bit %u not found in committed DBC",
                static_cast<unsigned long>(m.id), fd.name, fd.start_bit);
            TEST_ASSERT_TRUE_MESSAGE(sig.found, msg);

            // Length matches.
            std::snprintf(msg, sizeof(msg),
                "msg 0x%lX field '%s' length DBC=%u DSL=%u",
                static_cast<unsigned long>(m.id), fd.name, sig.len, fd.len);
            TEST_ASSERT_EQUAL_UINT_MESSAGE(fd.len, sig.len, msg);

            // Byte order: DBC '1' = little-endian, '0' = big-endian.
            const char want_order = fd.big_endian ? '0' : '1';
            std::snprintf(msg, sizeof(msg),
                "msg 0x%lX field '%s' byte-order DBC='%c' want='%c'",
                static_cast<unsigned long>(m.id), fd.name, sig.order, want_order);
            TEST_ASSERT_EQUAL_INT_MESSAGE(want_order, sig.order, msg);

            // Sign: DBC '-' = signed, '+' = unsigned.
            const char want_sign = fd.is_signed ? '-' : '+';
            std::snprintf(msg, sizeof(msg),
                "msg 0x%lX field '%s' sign DBC='%c' want='%c'",
                static_cast<unsigned long>(m.id), fd.name, sig.sign, want_sign);
            TEST_ASSERT_EQUAL_INT_MESSAGE(want_sign, sig.sign, msg);
        }
    }
}

// The array-of-frames families (pit-diag cell/temp grids) expand to one
// DBC message per frame. Validate every slot of every frame against the
// committed DBC by bit position -- this is what lets dbc_dump's array
// expansion eventually replace gen_dbc.py's pit_cells()/pit_temps().
extern "C" void test_dsl_array_descriptors_match_committed_dbc(void) {
    TEST_ASSERT_GREATER_THAN_UINT(0u, ifs08::ALL_ARRAYS_COUNT);
    for (unsigned ai = 0; ai < ifs08::ALL_ARRAYS_COUNT; ++ai) {
        const auto& A = ifs08::ALL_ARRAYS[ai];
        const unsigned elem_bytes = A.elem_bits / 8u;
        for (unsigned frame = 0; frame < A.frame_count; ++frame) {
            const std::uint32_t id = A.base_id + frame;
            for (unsigned slot = 0; slot < A.per_frame; ++slot) {
                const unsigned byte  = slot * elem_bytes;
                const unsigned start = A.big_endian ? (8u * byte + 7u) : (8u * byte);
                const DbcSig sig = find_dbc_signal(id, start);

                char msg[160];
                std::snprintf(msg, sizeof(msg),
                    "array 0x%lX frame %u slot %u (@bit %u) missing in DBC",
                    static_cast<unsigned long>(A.base_id), frame, slot, start);
                TEST_ASSERT_TRUE_MESSAGE(sig.found, msg);
                TEST_ASSERT_EQUAL_UINT_MESSAGE(A.elem_bits, sig.len, msg);
                TEST_ASSERT_EQUAL_INT_MESSAGE(A.big_endian ? '0' : '1', sig.order, msg);
                TEST_ASSERT_EQUAL_INT_MESSAGE(A.is_signed ? '-' : '+', sig.sign, msg);
            }
        }
    }
}
