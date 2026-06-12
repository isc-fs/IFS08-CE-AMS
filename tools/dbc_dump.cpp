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
    // DBC header boilerplate (cantools-parseable). The committed
    // docs/dbc/ams.dbc IS this tool's output -- the CI check regenerates
    // and diffs. Lean by design (no CM_ comments / VAL_ enum tables);
    // DBCinator re-enriches downstream.
    std::printf("VERSION \"\"\n\n");
    std::printf("NS_ :\n");
    static const char* ns[] = {
        "NS_DESC_","CM_","BA_DEF_","BA_","VAL_","CAT_DEF_","CAT_","FILTER",
        "BA_DEF_DEF_","EV_DATA_","ENVVAR_DATA_","SGTYPE_","SGTYPE_VAL_",
        "BA_DEF_SGTYPE_","BA_SGTYPE_","SIG_TYPE_REF_","VAL_TABLE_","SIG_GROUP_",
        "SIG_VALTYPE_","SIGTYPE_VALTYPE_","BO_TX_BU_","BA_DEF_REL_","BA_REL_",
        "BA_DEF_DEF_REL_","BU_SG_REL_","BU_EV_REL_","BU_BO_REL_","SG_MUL_VAL_",
    };
    for (const char* s : ns) std::printf("\t%s\n", s);
    std::printf("\nBS_:\n\n");
    std::printf("BU_: AMS VCU ECU UDV Pit_Tool\n\n");

    // GenMsgCycleTime attribute declaration. cantools exposes this as
    // Message.cycle_time; DBCinator's bus_load.py reads it to size the
    // fleet bus budget. Default 0 means "no declared cadence".
    std::printf("BA_DEF_ BO_  \"GenMsgCycleTime\" INT 0 65535;\n");
    std::printf("BA_DEF_DEF_  \"GenMsgCycleTime\" 0;\n\n");

    // Fixed-layout messages (one BO_ each).
    for (unsigned mi = 0; mi < ifs08::ALL_MSGS_COUNT; ++mi) {
        const auto& m = ifs08::ALL_MSGS[mi];
        std::printf("BO_ %u %s: %u %s\n", m.id, m.name, m.dlc, m.sender);
        for (unsigned fi = 0; fi < m.n_fields; ++fi) {
            const auto& f = m.fields[fi];
            std::printf(" SG_ %s : %u|%u@%c%c (%g,%g) [0|0] \"%s\" Vector__XXX\n",
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

    // Per-message GenMsgCycleTime attributes, emitted at the end so
    // they come after every BO_ they refer to. Period 0 means the
    // frame is one-shot / on-demand (BL trigger, ACK echoes), and
    // emitting "BA_ ... BO_ id 0;" is redundant with BA_DEF_DEF_,
    // so we skip those rows for a leaner DBC.
    for (unsigned mi = 0; mi < ifs08::ALL_MSGS_COUNT; ++mi) {
        const auto& m = ifs08::ALL_MSGS[mi];
        if (m.period_ms > 0) {
            std::printf("BA_ \"GenMsgCycleTime\" BO_ %u %u;\n", m.id, m.period_ms);
        }
    }
    for (unsigned ai = 0; ai < ifs08::ALL_ARRAYS_COUNT; ++ai) {
        const auto& A = ifs08::ALL_ARRAYS[ai];
        if (A.period_ms > 0) {
            for (unsigned frame = 0; frame < A.frame_count; ++frame) {
                std::printf("BA_ \"GenMsgCycleTime\" BO_ %u %u;\n",
                            A.base_id + frame, A.period_ms);
            }
        }
    }
    return 0;
}
