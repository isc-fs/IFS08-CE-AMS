// SPDX-License-Identifier: proprietary
//
// dbc_dump -- emit DBC BO_/SG_ rows straight from the code-first CAN DSL
// descriptors (ifs08::ALL_MSGS). This is the host-side counterpart that
// will REPLACE tools/gen_dbc.py once every message is ported onto the
// DSL: instead of a second, hand-maintained Python re-declaration of the
// layouts (which can silently drift from the firmware encoders), the DBC
// is generated from the exact same .def the C++ encoders are generated
// from. One source of truth, no C++<->DBC drift.
//
// Phase 2a scope: only the three telemetry frames are on the DSL, so this
// emits just those. gen_dbc.py still owns the full DBC for now; the unit
// test test_dsl_dbc_consistency.cpp guards that these three agree with the
// committed ams.dbc in the meantime.
//
// Build + run (host):
//   c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump

#include "can/can_codecs.hpp"

#include <cstdio>

int main() {
    std::printf("VERSION \"\"\n\n");
    std::printf("BU_: AMS VCU ECU\n\n");

    // Fixed-layout messages (one BO_ each).
    for (unsigned mi = 0; mi < ifs08::ALL_MSGS_COUNT; ++mi) {
        const auto& m = ifs08::ALL_MSGS[mi];
        std::printf("BO_ %u %s: %u %s\n", m.id, m.name, m.dlc, m.sender);
        for (unsigned fi = 0; fi < m.n_fields; ++fi) {
            const auto& f = m.fields[fi];
            std::printf(" SG_ %s : %u|%u@%c%c (%g,%g) \"%s\" Vector__XXX\n",
                        f.name, f.start_bit, f.len,
                        f.big_endian ? '0' : '1', f.is_signed ? '-' : '+',
                        f.factor, f.offset, f.unit);
        }
        std::printf("\n");
    }

    // Array-of-frames families: each expands to `frame_count` messages
    // (id = base_id + frame), `per_frame` signals each. Reproduces
    // gen_dbc.py's pit_cells()/pit_temps() naming + positions.
    for (unsigned ai = 0; ai < ifs08::ALL_ARRAYS_COUNT; ++ai) {
        const auto& A = ifs08::ALL_ARRAYS[ai];
        const unsigned elem_bytes = A.elem_bits / 8u;
        for (unsigned frame = 0; frame < A.frame_count; ++frame) {
            char mname[64];
            std::snprintf(mname, sizeof(mname), A.msg_name_fmt, frame);
            std::printf("BO_ %u %s: 8 %s\n", A.base_id + frame, mname, A.sender);
            for (unsigned slot = 0; slot < A.per_frame; ++slot) {
                const unsigned flat  = A.per_frame * frame + slot;
                const unsigned byte  = slot * elem_bytes;
                const unsigned start = A.big_endian ? (8u * byte + 7u) : (8u * byte);
                char sname[64];
                if (flat < A.total_elems) {
                    std::snprintf(sname, sizeof(sname), A.sig_name_fmt,
                                  flat / A.inner_dim, flat % A.inner_dim);
                } else if (A.sentinel_name_fmt != nullptr) {
                    std::snprintf(sname, sizeof(sname), A.sentinel_name_fmt, slot);
                } else {
                    continue;
                }
                std::printf(" SG_ %s : %u|%u@%c%c (1,0) [%ld|%ld] \"%s\" Vector__XXX\n",
                            sname, start, A.elem_bits,
                            A.big_endian ? '0' : '1', A.is_signed ? '-' : '+',
                            A.dbc_min, A.dbc_max, A.unit);
            }
            std::printf("\n");
        }
    }
    return 0;
}
