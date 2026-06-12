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
    for (unsigned mi = 0; mi < ifs08::ALL_MSGS_COUNT; ++mi) {
        const auto& m = ifs08::ALL_MSGS[mi];
        std::printf("BO_ %u %s: %u %s\n",
                    m.id, m.name, m.dlc, m.sender);
        for (unsigned fi = 0; fi < m.n_fields; ++fi) {
            const auto& f = m.fields[fi];
            // Standard DBC SG_ row: name : start|len@order+/-(factor,offset)[min|max] "unit" receiver
            // The [min|max] bracket is mandatory -- cantools / Vector
            // CANdb++ reject the row without it. We emit [0|0] (the
            // "unspecified" convention) since the DSL doesn't carry
            // physical range info today; gen_dbc.py does the same.
            std::printf(" SG_ %s : %u|%u@%c%c (%g,%g) [0|0] \"%s\" Vector__XXX\n",
                        f.name, f.start_bit, f.len,
                        f.big_endian ? '0' : '1',
                        f.is_signed  ? '-' : '+',
                        f.factor, f.offset, f.unit);
        }
        std::printf("\n");
    }
    return 0;
}
