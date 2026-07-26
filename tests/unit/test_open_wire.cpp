// SPDX-License-Identifier: proprietary
//
// Cell open-wire (LTC6811 ADOW) tests: the command encoder and the pure
// detection algorithm. Hardware timing / the two RDCV passes are HIL-gated
// (config::CellOpenWireCheck); the logic here is what must be provably correct.

#include "ltc6811.hpp"
#include "open_wire.hpp"
#include "ams_config.hpp"

#include "unity.h"

#include <cstdint>

namespace {
constexpr std::uint16_t DELTA = ams::config::CellOpenWireDeltaMv;  // 400 mV
}

// --- ADOW command encoding (datasheet-derived; see ltc6811.hpp adow_cmd) -----
extern "C" void test_adow_cmd_encoding(void) {
    using ams::ltc6811::adow_cmd;
    using ams::ltc6811::AdcMode;
    using ams::ltc6811::CellSel;
    // MD=Norm7kHz(10), PUP=1, DCP=0, CH=All -> 0x0248|0x100|0x020 = 0x0368
    TEST_ASSERT_EQUAL_HEX16(0x0368,
        adow_cmd(AdcMode::Norm7kHz, /*pull_up=*/true,  /*dcp=*/false, CellSel::All));
    // Same but PUP=0 -> 0x0348
    TEST_ASSERT_EQUAL_HEX16(0x0348,
        adow_cmd(AdcMode::Norm7kHz, /*pull_up=*/false, /*dcp=*/false, CellSel::All));
    // DCP=1 sets bit4 (+0x10)
    TEST_ASSERT_EQUAL_HEX16(0x0378,
        adow_cmd(AdcMode::Norm7kHz, /*pull_up=*/true,  /*dcp=*/true,  CellSel::All));
}

// --- detect_open_conductors: no open on a healthy IC -------------------------
extern "C" void test_open_wire_none_when_healthy(void) {
    // Pull-up and pull-down readings track each other on a connected cell.
    std::uint16_t pu[10], pd[10];
    for (int i = 0; i < 10; ++i) { pu[i] = 3700; pd[i] = 3700; }
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(pu, pd, 10, DELTA));
    TEST_ASSERT_FALSE(ams::open_wire::has_open(pu, pd, 10, DELTA));
}

// --- interior conductor open: CELL_PU(n+1) - CELL_PD(n+1) < -delta -----------
extern "C" void test_open_wire_interior_conductor(void) {
    std::uint16_t pu[10], pd[10];
    for (int i = 0; i < 10; ++i) { pu[i] = 3700; pd[i] = 3700; }
    // Conductor 4 open: cell index 4's pull-up reading collapses below pull-down.
    pu[4] = 3000; pd[4] = 3700;   // diff = -700 < -400
    const std::uint16_t mask = ams::open_wire::detect_open_conductors(pu, pd, 10, DELTA);
    TEST_ASSERT_BITS_HIGH(1u << 4, mask);
    TEST_ASSERT_TRUE(ams::open_wire::has_open(pu, pd, 10, DELTA));
    // A delta smaller than the threshold does NOT flag.
    pu[4] = 3400; pd[4] = 3700;   // diff = -300 > -400
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(pu, pd, 10, DELTA));
}

// --- endpoint conductors: C(0) via CELL_PU(1)==0, C(N) via CELL_PD(N)==0 -----
extern "C" void test_open_wire_endpoints(void) {
    std::uint16_t pu[10], pd[10];
    for (int i = 0; i < 10; ++i) { pu[i] = 3700; pd[i] = 3700; }
    pu[0] = 0;   // bottom conductor C(0) open
    TEST_ASSERT_BITS_HIGH(1u << 0, ams::open_wire::detect_open_conductors(pu, pd, 10, DELTA));

    for (int i = 0; i < 10; ++i) { pu[i] = 3700; pd[i] = 3700; }
    pd[9] = 0;   // top conductor C(N) open (N = n_cells = 10)
    TEST_ASSERT_BITS_HIGH(1u << 10, ams::open_wire::detect_open_conductors(pu, pd, 10, DELTA));
}

// --- guards: null / zero / oversized never report a (false) open -------------
extern "C" void test_open_wire_input_guards(void) {
    std::uint16_t pu[10] = {0}, pd[10] = {0};
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(nullptr, pd, 10, DELTA));
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(pu, nullptr, 10, DELTA));
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(pu, pd, 0, DELTA));
    TEST_ASSERT_EQUAL_UINT16(0u, ams::open_wire::detect_open_conductors(pu, pd, 99, DELTA));
}
