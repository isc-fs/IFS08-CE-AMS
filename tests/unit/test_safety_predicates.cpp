// SPDX-License-Identifier: proprietary
//
// Pure-logic tests for safety::evaluate_fault. Constructs Inputs with
// known service-snapshot values and asserts the predicate boolean.

#include "ams_config.hpp"
#include "safety_predicates.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

// Build a fully-healthy input set keyed off a single now_tick.
ams::safety::Inputs make_nominal(ams::BmsState& bms,
                                 ams::CurrentState& cur,
                                 ams::VehicleState& veh,
                                 std::uint32_t now) {
    std::memset(&bms, 0, sizeof(bms));
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&veh, 0, sizeof(veh));

    bms.module_online_mask = ams::config::AllModulesMask;
    for (std::uint8_t m = 0; m < ams::config::BmsModuleCount; ++m) {
        bms.last_rx_tick[m] = now - 100;
        // Per-module aggregates consistent with a healthy pack so the
        // fault-detail localisation (#279) has something to scan.
        bms.vmin_module[m] = 3700;
        bms.vmax_module[m] = 3800;
        bms.tmax_module[m] = 35;
    }
    bms.min_cell_mV = 3700;
    bms.max_cell_mV = 3800;
    bms.min_tempC   =  20;
    bms.max_tempC   =  35;
    // A nominal pack has been fully polled at least once, so the cell
    // V/T range predicates are armed (#279 gate).
    bms.first_full_poll_done = true;

    cur.last_update_tick = now - 50;
    cur.filtered_mA      = 5000;       // 5 A discharge, well under limit
    cur.sensor_fault     = false;

    veh.last_dc_bus_tick = now - 50;

    return { bms, cur, veh,
             /*force_error_set=*/false,
             /*vcu_required=*/true,   // Car-mode nominal (VCU required)
             now };
}

}  // namespace

extern "C" void test_predicates_nominal_no_fault(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);

    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_force_error(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    in.force_error_set = true;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_undervoltage(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.min_cell_mV = ams::config::CellUnderVoltageMv - 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_overvoltage(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.max_cell_mV = ams::config::CellOverVoltageMv + 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_cell_overtemp(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.max_tempC = ams::config::CellOverTempC + 1;
    // Cell-temp faults are gated behind config::TempFaultsTrusted. While temps
    // are not trusted (mux path unvalidated) an over-temp must NOT fault; when
    // the flag flips true it must. This asserts whichever the flag selects.
    if (ams::config::TempFaultsTrusted) {
        TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
    } else {
        TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
    }
}

extern "C" void test_predicates_bms_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.last_rx_tick[2] = 10000 - ams::config::BmsStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_current_overlimit(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    cur.filtered_mA = ams::config::CurrentMaxMa + 1;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_current_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    cur.last_update_tick = 10000 - ams::config::IStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_vcu_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    veh.last_dc_bus_tick = 10000 - ams::config::VcuStaleMs - 10;
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

// ---------------------------------------------------------------------------
// Boot grace (regression guard for the bench-discovered IWDG reset loop
// caused by all-zero last_*_ticks at first SafetyTask iteration).
// ---------------------------------------------------------------------------

// All ticks zero + within the grace window: every freshness check
// would otherwise fault. Boot grace must suppress them.
extern "C" void test_predicates_boot_grace_suppresses_data_predicates(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    // Build manually -- can't reuse make_nominal because it pre-sets
    // last_*_ticks, and the bug is exactly about ticks at zero.
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*force_error_set=*/false,
        /*vcu_required=*/true,
        /*now_tick=*/500u,  // half a second in -- well inside grace
    };
    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

// Immediate-safety predicates must keep firing during the grace.
extern "C" void test_predicates_boot_grace_does_not_suppress_force_error(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*force_error_set=*/true,
        /*vcu_required=*/true,
        /*now_tick=*/100u,
    };
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

// One tick past the grace window with no service activity -> fault.
// The unsigned `now - 0 = now` age naturally exceeds every staleness
// window for now_tick > grace > max(BmsStaleMs, IStaleMs, VcuStaleMs).
extern "C" void test_predicates_after_grace_zero_ticks_faults(void) {
    ams::BmsState     bms{};
    ams::CurrentState cur{};
    ams::VehicleState veh{};
    const ams::safety::Inputs in = {
        bms, cur, veh,
        /*force_error_set=*/false,
        /*vcu_required=*/true,
        /*now_tick=*/ams::config::SafetyBootGraceMs + 1u,
    };
    TEST_ASSERT_TRUE(ams::safety::evaluate_fault(in));
}

// ---------------------------------------------------------------------------
// #276 regression: a producer that updated its last_*_tick to a value
// slightly AHEAD of the SafetyTask `now` sample (it reported between the
// now-read and the snapshot read) must NOT trip staleness. The naive
// `now - last` underflows to ~4e9; tick_age() clamps it to 0 (fresh).
// ---------------------------------------------------------------------------
extern "C" void test_predicates_future_tick_not_stale(void) {
    ams::BmsState     bms;
    ams::CurrentState cur;
    ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    for (std::uint8_t m = 0; m < ams::config::BmsModuleCount; ++m) {
        bms.last_rx_tick[m] = 10005;   // 5 ticks "in the future"
    }
    cur.last_update_tick = 10005;
    veh.last_dc_bus_tick = 10005;
    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

extern "C" void test_predicates_tick_age_clamps_future(void) {
    TEST_ASSERT_EQUAL_UINT32(0u,    ams::safety::tick_age(1000u, 1005u));
    TEST_ASSERT_EQUAL_UINT32(5u,    ams::safety::tick_age(1005u, 1000u));
    TEST_ASSERT_EQUAL_UINT32(1000u, ams::safety::tick_age(1000u, 0u));
}

// ---------------------------------------------------------------------------
// Fault-reason mapping (#276). evaluate_fault_detail surfaces the exact
// branch that latched ERROR so the HIL bench can localise it on 0x6C0[6].
// ---------------------------------------------------------------------------
extern "C" void test_predicates_reason_none_when_nominal(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::None),
        static_cast<std::uint8_t>(ams::safety::evaluate_fault_detail(in).reason));
}

extern "C" void test_predicates_reason_force_error(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    in.force_error_set = true;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::ForceError),
        static_cast<std::uint8_t>(ams::safety::evaluate_fault_detail(in).reason));
}

extern "C" void test_predicates_reason_bms_stale_reports_module(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.last_rx_tick[3] = 10000 - ams::config::BmsStaleMs - 10;
    const auto res = ams::safety::evaluate_fault_detail(in);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::BmsStale),
        static_cast<std::uint8_t>(res.reason));
    TEST_ASSERT_EQUAL_UINT8(3u, res.detail);  // module index
}

extern "C" void test_predicates_reason_current_stale(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    cur.last_update_tick = 10000 - ams::config::IStaleMs - 10;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::CurrentStale),
        static_cast<std::uint8_t>(ams::safety::evaluate_fault_detail(in).reason));
}

extern "C" void test_predicates_reason_vcu_stale(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    veh.last_dc_bus_tick = 10000 - ams::config::VcuStaleMs - 10;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::VcuStale),
        static_cast<std::uint8_t>(ams::safety::evaluate_fault_detail(in).reason));
}

// ---------------------------------------------------------------------------
// #279: the cell V/T range predicates are gated on first_full_poll_done.
// A partially-populated BmsState at the boot-grace edge (a module that
// hasn't reported, so min_cell reads a sentinel/zero) must NOT trip
// CellUnderVoltage until the whole pack has been polled once.
// ---------------------------------------------------------------------------
extern "C" void test_predicates_undervoltage_suppressed_before_first_poll(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.first_full_poll_done = false;          // pack not fully polled yet
    bms.min_cell_mV    = 0;                     // partial / zero-init cells
    bms.vmin_module[2] = 0;
    // Freshness + mask are still fine (boot free-pass), so no other branch
    // fires either -- the chip must stay out of ERROR.
    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
}

// Once armed, a genuine under-voltage still trips -- and the detail byte
// names the offending module so the bench can localise it on 0x6C0[7].
extern "C" void test_predicates_undervoltage_armed_reports_module(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.first_full_poll_done = true;
    bms.min_cell_mV    = ams::config::CellUnderVoltageMv - 1;
    bms.vmin_module[3] = ams::config::CellUnderVoltageMv - 1;  // module 3 is low
    const auto res = ams::safety::evaluate_fault_detail(in);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::CellUnderVoltage),
        static_cast<std::uint8_t>(res.reason));
    TEST_ASSERT_EQUAL_UINT8(3u, res.detail);
}

// A genuinely-offline module still latches ERROR before the gate is even
// consulted -- the gate only relaxes the *value* checks, never the
// data-presence checks.
extern "C" void test_predicates_offline_module_trips_regardless_of_gate(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    bms.first_full_poll_done = false;
    bms.module_online_mask   = ams::config::AllModulesMask & ~0x04u;  // module 2 off
    const auto res = ams::safety::evaluate_fault_detail(in);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::BmsModuleOffline),
        static_cast<std::uint8_t>(res.reason));
}

// ---------------------------------------------------------------------------
// #279 follow-up: torn-snapshot fingerprint + cell-fault debounce.
// ---------------------------------------------------------------------------

// module_below returns the NoOffendingModule sentinel (0xFF) when the
// summary crossed the threshold but no per-module aggregate did -- the
// signature of a torn lock-free snapshot read (min_cell_mV and
// vmin_module copied from different poll cycles). Distinct from a real
// module index so the bench can tell them apart on 0x6C0[7].
extern "C" void test_module_below_sentinel_when_none(void) {
    std::uint16_t per_module[ams::config::BmsModuleCount];
    for (std::uint8_t m = 0; m < ams::config::BmsModuleCount; ++m) per_module[m] = 3700;
    TEST_ASSERT_EQUAL_UINT8(ams::safety::NoOffendingModule,
                            ams::safety::module_below(per_module, 2800));
    per_module[2] = 2000;  // now module 2 is genuinely low
    TEST_ASSERT_EQUAL_UINT8(2u, ams::safety::module_below(per_module, 2800));
}

// is_cell_range_reason classifies only the four V/T range reasons.
extern "C" void test_is_cell_range_reason(void) {
    using R = ams::safety::FaultReason;
    TEST_ASSERT_TRUE(ams::safety::is_cell_range_reason(R::CellUnderVoltage));
    TEST_ASSERT_TRUE(ams::safety::is_cell_range_reason(R::CellOverVoltage));
    TEST_ASSERT_TRUE(ams::safety::is_cell_range_reason(R::CellUnderTemp));
    TEST_ASSERT_TRUE(ams::safety::is_cell_range_reason(R::CellOverTemp));
    TEST_ASSERT_FALSE(ams::safety::is_cell_range_reason(R::None));
    TEST_ASSERT_FALSE(ams::safety::is_cell_range_reason(R::ForceError));
    TEST_ASSERT_FALSE(ams::safety::is_cell_range_reason(R::BmsStale));
    TEST_ASSERT_FALSE(ams::safety::is_cell_range_reason(R::CurrentStale));
    TEST_ASSERT_FALSE(ams::safety::is_cell_range_reason(R::VcuStale));
}

// A cell-range fault must persist for `confirm` consecutive updates
// before the debounce confirms it. A transient (single tick) never does.
extern "C" void test_cell_debounce_confirms_after_n(void) {
    ams::safety::CellFaultDebounce db;
    const std::uint16_t N = 30;
    // 29 consecutive under-voltage ticks: not yet confirmed.
    for (std::uint16_t i = 0; i < N - 1; ++i) {
        TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));
    }
    // The Nth consecutive tick confirms.
    TEST_ASSERT_TRUE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));
    // Stays confirmed while it persists.
    TEST_ASSERT_TRUE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));
}

extern "C" void test_cell_debounce_transient_never_confirms(void) {
    ams::safety::CellFaultDebounce db;
    const std::uint16_t N = 30;
    // A single sub-threshold tick surrounded by clean ticks: a glitch.
    for (int cycle = 0; cycle < 50; ++cycle) {
        TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));  // 1 bad
        TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::None, N));              // clean -> reset
    }
}

// A changed offending reason restarts the streak (no cross-reason carry).
extern "C" void test_cell_debounce_reason_change_resets(void) {
    ams::safety::CellFaultDebounce db;
    const std::uint16_t N = 3;
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));  // 1
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellUnderVoltage, N));  // 2
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellOverTemp, N));      // reset -> 1
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CellOverTemp, N));      // 2
    TEST_ASSERT_TRUE (db.update(ams::safety::FaultReason::CellOverTemp, N));      // 3 -> confirm
}

// A non-cell reason is never debounced -- update() returns false and the
// caller applies its own immediate-fault decision.
extern "C" void test_cell_debounce_ignores_non_cell(void) {
    ams::safety::CellFaultDebounce db;
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::CurrentOverLimit, 3));
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::VcuStale, 3));
    TEST_ASSERT_FALSE(db.update(ams::safety::FaultReason::ForceError, 3));
}

// ---------------------------------------------------------------------------
// #299: AMS_OK (SDC enable) drive decision. HIGH only past boot grace
// with no ERROR latched; LOW during grace and whenever latched.
// ---------------------------------------------------------------------------
extern "C" void test_ams_ok_low_during_grace(void) {
    // Mid-grace, no latch: still LOW (predicates suppressed, don't enable SDC).
    TEST_ASSERT_FALSE(ams::safety::ams_ok_asserted(
        ams::config::SafetyBootGraceMs - 1u, /*error_latched=*/false));
    TEST_ASSERT_FALSE(ams::safety::ams_ok_asserted(0u, false));
}

extern "C" void test_ams_ok_high_when_healthy_post_grace(void) {
    TEST_ASSERT_TRUE(ams::safety::ams_ok_asserted(
        ams::config::SafetyBootGraceMs, /*error_latched=*/false));
    TEST_ASSERT_TRUE(ams::safety::ams_ok_asserted(
        ams::config::SafetyBootGraceMs + 100000u, false));
}

extern "C" void test_ams_ok_low_when_latched(void) {
    // Latched ERROR forces LOW regardless of time (before or after grace).
    TEST_ASSERT_FALSE(ams::safety::ams_ok_asserted(
        ams::config::SafetyBootGraceMs + 100000u, /*error_latched=*/true));
    TEST_ASSERT_FALSE(ams::safety::ams_ok_asserted(0u, true));
}

// ---------------------------------------------------------------------------
// #302: VcuStale must be exempt when not committed to Car mode, or the
// 200 ms VcuStaleMs predicate pre-empts the 1000 ms charger-mode lock and
// Charger mode is unreachable. Car-mode (vcu_required=true) still faults.
// ---------------------------------------------------------------------------
extern "C" void test_predicates_vcu_stale_exempt_when_not_car(void) {
    ams::BmsState bms; ams::CurrentState cur; ams::VehicleState veh;
    auto in = make_nominal(bms, cur, veh, 10000);
    veh.last_dc_bus_tick = 10000 - ams::config::VcuStaleMs - 10;  // VCU stale
    in.vcu_required = false;  // Charger / pre-lock Undecided
    // Everything else healthy -> no fault despite the stale VCU.
    TEST_ASSERT_FALSE(ams::safety::evaluate_fault(in));
    // And confirm the same input WOULD fault in Car mode (regression
    // guard that we only relaxed the charger path).
    in.vcu_required = true;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(ams::safety::FaultReason::VcuStale),
        static_cast<std::uint8_t>(ams::safety::evaluate_fault_detail(in).reason));
}
