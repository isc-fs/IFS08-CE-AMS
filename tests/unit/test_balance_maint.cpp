// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for ams::balance::maint -- the maintenance-balancing policy
// (maint_config.hpp) that bleeds cells while the AMS is latched in Error with a
// BMS module unplugged and module 0's temperature sensors ignored.

#include "ams_config.hpp"
#include "balance_controller.hpp"
#include "bms_service.hpp"
#include "ltc6811.hpp"
#include "maint_config.hpp"
#include "state_machine.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

using namespace ams;

constexpr std::uint8_t kModules0to3 = 0x0Fu;   // module 4 (last two LTCs) unplugged
constexpr std::uint8_t kIgnoreM0    = 0x01u;

// Every cell at base_mV, every NTC at base_C. Modules in `online` answer and
// have reported; the rest keep the BmsService boot seed (3700 mV, no temps).
BmsState make_state(std::uint16_t base_mV, std::int16_t base_C, std::uint8_t online) {
    BmsState s{};
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        const bool on = (online & (1u << m)) != 0u;
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            s.cell_mV[m][c] = on ? base_mV : 3700;
        }
        for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
            s.cell_tempC[m][t] = on ? base_C : config::NtcNoReading;
        }
        s.last_rx_tick[m] = on ? 5000u : 0u;
    }
    s.module_online_mask  = online;
    // Summary fields only the flight policy reads (the maintenance policy
    // derives its own from the matrices above).
    std::uint16_t n_on = 0;
    for (std::uint8_t m = 0; m < config::BmsModuleCount; ++m) {
        if ((online & (1u << m)) != 0u) n_on = static_cast<std::uint16_t>(n_on + config::TempsPerModule);
    }
    s.valid_temp_channels = n_on;
    s.max_tempC           = base_C;
    return s;
}

balance::Mask run(const BmsState& s,
                  fsm::State fsm_state = fsm::State::Error,
                  config::BalanceCmd cmd = config::BalanceCmd::On,
                  safety::FaultReason fault = safety::FaultReason::BmsModuleOffline,
                  std::uint8_t enable = config::AllModulesMask,
                  std::uint8_t ignored = kIgnoreM0) {
    return balance::maint::compute_mask(s, fsm_state, /*temps_trusted=*/true, cmd,
                                        fault, enable, ignored, nullptr);
}

std::uint8_t count_set(const balance::Mask& m) {
    std::uint8_t n = 0;
    for (std::uint8_t mm = 0; mm < config::BmsModuleCount; ++mm) {
        for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
            if (m.cell[mm][c]) ++n;
        }
    }
    return n;
}

// One RDCFGA reply segment: 6 data bytes + their PEC, as an LTC6811 sends it.
void put_segment(std::uint8_t* seg, const std::array<std::uint8_t, 6>& data) {
    std::memcpy(seg, data.data(), 6);
    const std::uint16_t pec = ltc6811::pec15(seg, 6);
    seg[6] = static_cast<std::uint8_t>(pec >> 8);
    seg[7] = static_cast<std::uint8_t>(pec & 0xFFu);
}

}  // namespace

// --- the build switch itself ------------------------------------------------

extern "C" void test_maint_build_flag_off_in_host_tests(void) {
    // The host suite is the flight configuration: the maintenance policy is
    // compiled and tested here, but nothing in a flag-OFF build selects it.
    TEST_ASSERT_FALSE(maint_config::Enabled);
    TEST_ASSERT_EQUAL_HEX8(0x01u, maint_config::IgnoredTempModules);
    // Default limits are uncapped; AMS_MAINT_BALANCE_CAPPED is the way back.
    TEST_ASSERT_FALSE(maint_config::Capped);
    TEST_ASSERT_EQUAL_UINT8(config::CellsPerModule, maint_config::MaxActivePerModule);
    TEST_ASSERT_FALSE(maint_config::SpreadNoAdjacent);
}

// --- partial pack -----------------------------------------------------------

extern "C" void test_maint_balance_missing_module_does_not_drag_floor(void) {
    // Modules 0..3 sit at 3900 mV with one high cell; module 4 is unplugged and
    // still holds the 3700 mV boot seed.
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;

    const auto out = run(s);
    TEST_ASSERT_TRUE(out.cell[1][3]);
    TEST_ASSERT_EQUAL_UINT8(1, count_set(out));   // nothing else is above the real floor

    // The flight policy, fed the same snapshot, takes the seed as the floor and
    // bleeds the whole connected pack -- the failure this policy exists to avoid.
    const auto flight = balance::compute_mask(s, fsm::State::Error, true,
                                              config::BalanceCmd::On,
                                              safety::FaultReason::BmsModuleOffline);
    TEST_ASSERT_TRUE(count_set(flight) > 1);
}

extern "C" void test_maint_balance_never_selects_a_missing_module(void) {
    auto s = make_state(3900, 25, kModules0to3);
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) s.cell_mV[4][c] = 4100;
    const auto out = run(s);
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
        TEST_ASSERT_FALSE(out.cell[4][c]);
    }
}

extern "C" void test_maint_balance_never_reported_module_is_excluded(void) {
    // In the first BmsStaleMs of uptime a module that has never answered still
    // counts as online. It must not count as data until it has reported.
    auto s = make_state(3900, 25, kModules0to3);
    s.module_online_mask = config::AllModulesMask;   // module 4 "online"...
    s.last_rx_tick[4]    = 0u;                       // ...but never reported
    s.cell_mV[1][3]      = 3990;
    const auto out = run(s);
    TEST_ASSERT_TRUE(out.cell[1][3]);
    TEST_ASSERT_EQUAL_UINT8(1, count_set(out));
}

extern "C" void test_maint_balance_full_pack_balances_too(void) {
    auto s = make_state(3900, 25, config::AllModulesMask);
    s.cell_mV[4][5] = 3990;
    const auto out = run(s, fsm::State::Error, config::BalanceCmd::On,
                         safety::FaultReason::TempSensorDisconnected);
    TEST_ASSERT_TRUE(out.cell[4][5]);
}

extern "C" void test_maint_balance_is_uncapped(void) {
    // Every cell of module 1 well above the floor set by modules 0, 2 and 3: the
    // maintenance policy bleeds all 19 at once, neighbours included.
    auto s = make_state(3900, 25, kModules0to3);
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) s.cell_mV[1][c] = 4000;

    const auto out = run(s);
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
        TEST_ASSERT_TRUE(out.cell[1][c]);
    }
    TEST_ASSERT_EQUAL_UINT8(config::CellsPerModule, count_set(out));

    // The flight policy on the same snapshot stays within its limits.
    const auto flight = balance::compute_mask(s, fsm::State::Error, true,
                                              config::BalanceCmd::On,
                                              safety::FaultReason::BmsModuleOffline);
    std::uint8_t n_flight = 0;
    for (std::uint8_t c = 0; c < config::CellsPerModule; ++c) {
        if (flight.cell[1][c]) ++n_flight;
        if (c + 1u < config::CellsPerModule && flight.cell[1][c] && flight.cell[1][c + 1u]) {
            TEST_ASSERT_FALSE(balance::physically_adjacent(c, static_cast<std::uint8_t>(c + 1u)));
        }
    }
    TEST_ASSERT_TRUE(n_flight <= config::BalanceMaxActive);
}

extern "C" void test_maint_balance_uncapped_still_skips_matched_cells(void) {
    // Uncapped does not mean "every cell": cells within BalanceDeltaMv of the
    // floor have nothing to bleed and stay off.
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][4] = 3990;
    s.cell_mV[1][5] = 3990;
    s.cell_mV[1][6] = static_cast<std::uint16_t>(3900 + config::BalanceDeltaMv);
    const auto out = run(s);
    TEST_ASSERT_TRUE(out.cell[1][4]);
    TEST_ASSERT_TRUE(out.cell[1][5]);
    TEST_ASSERT_FALSE(out.cell[1][6]);
    TEST_ASSERT_EQUAL_UINT8(2, count_set(out));
}

// --- operator command and FSM state -----------------------------------------

extern "C" void test_maint_balance_requires_operator_on(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s, fsm::State::Error, config::BalanceCmd::Off)));
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s, fsm::State::Error, config::BalanceCmd::Auto)));
}

extern "C" void test_maint_balance_only_with_airs_open(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;
    TEST_ASSERT_TRUE(run(s, fsm::State::Start).cell[1][3]);
    TEST_ASSERT_TRUE(run(s, fsm::State::Error).cell[1][3]);
    for (auto st : {fsm::State::Precharge, fsm::State::Transition,
                    fsm::State::Run, fsm::State::Charge}) {
        TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s, st)));
    }
}

extern "C" void test_maint_balance_respects_module_enable(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;
    s.cell_mV[2][3] = 3990;
    const auto out = run(s, fsm::State::Error, config::BalanceCmd::On,
                         safety::FaultReason::BmsModuleOffline,
                         static_cast<std::uint8_t>(config::AllModulesMask & ~(1u << 1)));
    TEST_ASSERT_FALSE(out.cell[1][3]);
    TEST_ASSERT_TRUE(out.cell[2][3]);
}

// --- cell data, re-derived from the live snapshot ---------------------------

extern "C" void test_maint_balance_cell_data_rederived_while_latched(void) {
    // The latched reason is BmsModuleOffline -- the first fault -- so a later
    // open wire, displaced tap or out-of-range cell never reaches it.
    auto base = make_state(3900, 25, kModules0to3);
    base.cell_mV[1][3] = 3990;
    TEST_ASSERT_EQUAL_UINT8(1, count_set(run(base)));

    auto s = base; s.cell_open_mask = 0x04u;
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));

    s = base; s.tap_fault_mask = 0x02u;
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));

    s = base; s.cell_mV[3][7] = static_cast<std::uint16_t>(config::CellUnderVoltageMv - 1);
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));

    s = base; s.cell_mV[0][0] = static_cast<std::uint16_t>(config::CellOverVoltageMv + 1);
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));

    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(base, fsm::State::Error, config::BalanceCmd::On,
                                             safety::FaultReason::CellOpenWire)));
}

extern "C" void test_maint_balance_cell_data_of_missing_module_ignored(void) {
    // Stale values in the unplugged module say nothing about the pack.
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;
    s.cell_mV[4][0] = 0;
    s.cell_open_mask = 0x10u;
    TEST_ASSERT_TRUE(run(s).cell[1][3]);
}

extern "C" void test_maint_balance_no_module_answering(void) {
    auto s = make_state(3900, 25, 0x00u);
    TEST_ASSERT_EQUAL_HEX16(balance::maint::InhibitNoModules,
        balance::maint::gate_reasons(s, fsm::State::Error, true, config::BalanceCmd::On,
                                     safety::FaultReason::BmsModuleOffline, kIgnoreM0)
        & balance::maint::InhibitNoModules);
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));
}

// --- temperatures -----------------------------------------------------------

extern "C" void test_maint_balance_ignores_module0_sensors(void) {
    // Module 0's harness: some channels open (and flagged lost), one decoding to
    // a nonsense high value. None of it may block balancing -- module 0 included.
    auto s = make_state(3900, 25, kModules0to3);
    for (std::uint8_t t = 0; t < config::TempsPerModule; ++t) {
        s.cell_tempC[0][t] = config::NtcNoReading;
    }
    s.cell_tempC[0][7]     = 140;
    s.temp_disconnect_mask = 0x01u;
    s.cell_mV[0][2]        = 3990;
    s.cell_mV[2][2]        = 3990;
    const auto out = run(s);
    TEST_ASSERT_TRUE(out.cell[0][2]);
    TEST_ASSERT_TRUE(out.cell[2][2]);
}

extern "C" void test_maint_balance_thermal_lockout_on_trusted_modules(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3]     = 3990;
    s.cell_tempC[2][10] = static_cast<std::int16_t>(config::BalanceTempMax + 1);
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));
    s.cell_tempC[2][10] = config::BalanceTempMax;   // at the limit is still allowed
    TEST_ASSERT_TRUE(run(s).cell[1][3]);
}

extern "C" void test_maint_balance_lost_sensor_on_trusted_module_blocks(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3]        = 3990;
    s.temp_disconnect_mask = 0x04u;   // module 2
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));
}

extern "C" void test_maint_balance_needs_thermal_data_outside_module0(void) {
    // Only module 0 answering: its sensors are ignored, so there is no thermal
    // data at all and nothing balances -- not even module 0.
    auto s = make_state(3900, 25, 0x01u);
    s.cell_mV[0][2] = 3990;
    TEST_ASSERT_EQUAL_UINT8(0, count_set(run(s)));

    // Without the ignore, the same snapshot would balance.
    TEST_ASSERT_TRUE(run(s, fsm::State::Error, config::BalanceCmd::On,
                         safety::FaultReason::BmsModuleOffline,
                         config::AllModulesMask, /*ignored=*/0x00u).cell[0][2]);
}

extern "C" void test_maint_balance_refuses_untrusted_temps(void) {
    auto s = make_state(3900, 25, kModules0to3);
    s.cell_mV[1][3] = 3990;
    const auto out = balance::maint::compute_mask(
        s, fsm::State::Error, /*temps_trusted=*/false, config::BalanceCmd::On,
        safety::FaultReason::BmsModuleOffline, config::AllModulesMask, kIgnoreM0, nullptr);
    TEST_ASSERT_EQUAL_UINT8(0, count_set(out));
}

// --- configuration readback -------------------------------------------------

extern "C" void test_maint_readback_ok_when_every_ic_matches(void) {
    std::uint8_t written[config::LtcChainLength][6];
    std::uint8_t reply[8 * config::LtcChainLength];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const auto p = ltc6811::pack_cfga_payload(ic == 3 ? 0x0105u : 0u);
        std::memcpy(written[ic], p.data(), 6);
        auto back = p;
        back[0] = 0x3Fu;   // GPIO levels read back differently; not compared
        put_segment(reply + 8 * ic, back);
    }
    TEST_ASSERT_TRUE(balance::maint::check_cfga_readback(written, reply, sizeof reply)
                     == balance::maint::Readback::Ok);
}

extern "C" void test_maint_readback_catches_bits_on_the_wrong_ic(void) {
    // Written for IC 3, landed on IC 6: the short-chain mapping is wrong.
    std::uint8_t written[config::LtcChainLength][6];
    std::uint8_t reply[8 * config::LtcChainLength];
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const auto w = ltc6811::pack_cfga_payload(ic == 3 ? 0x0021u : 0u);
        const auto r = ltc6811::pack_cfga_payload(ic == 6 ? 0x0021u : 0u);
        std::memcpy(written[ic], w.data(), 6);
        put_segment(reply + 8 * ic, r);
    }
    TEST_ASSERT_TRUE(balance::maint::check_cfga_readback(written, reply, sizeof reply)
                     == balance::maint::Readback::Mismatch);
}

extern "C" void test_maint_readback_absent_module_is_fine_when_idle(void) {
    // ICs 8 and 9 unplugged: the reply carries the bus idle level, which fails
    // PEC. They were written no discharge, so that is not a problem.
    std::uint8_t written[config::LtcChainLength][6];
    std::uint8_t reply[8 * config::LtcChainLength];
    std::memset(reply, 0xFF, sizeof reply);
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const auto p = ltc6811::pack_cfga_payload(ic == 2 ? 0x0004u : 0u);
        std::memcpy(written[ic], p.data(), 6);
        if (ic < 8) put_segment(reply + 8 * ic, p);
    }
    TEST_ASSERT_TRUE(balance::maint::check_cfga_readback(written, reply, sizeof reply)
                     == balance::maint::Readback::Ok);
}

extern "C" void test_maint_readback_unproven_when_a_discharging_ic_is_silent(void) {
    std::uint8_t written[config::LtcChainLength][6];
    std::uint8_t reply[8 * config::LtcChainLength];
    std::memset(reply, 0xFF, sizeof reply);
    for (std::uint8_t ic = 0; ic < config::LtcChainLength; ++ic) {
        const auto p = ltc6811::pack_cfga_payload(ic == 5 ? 0x0004u : 0u);
        std::memcpy(written[ic], p.data(), 6);
        if (ic != 5) put_segment(reply + 8 * ic, p);
    }
    TEST_ASSERT_TRUE(balance::maint::check_cfga_readback(written, reply, sizeof reply)
                     == balance::maint::Readback::Unproven);
}

extern "C" void test_maint_readback_short_reply_is_unproven(void) {
    std::uint8_t written[config::LtcChainLength][6] = {};
    std::uint8_t reply[8] = {};
    TEST_ASSERT_TRUE(balance::maint::check_cfga_readback(written, reply, sizeof reply)
                     == balance::maint::Readback::Unproven);
}
