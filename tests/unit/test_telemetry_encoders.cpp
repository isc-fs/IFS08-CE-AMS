// SPDX-License-Identifier: proprietary
//
// Tests for the three pure-logic telemetry encoders. Each frame is
// verified byte-by-byte against the layout documented in
// docs/CAN_MAP.md.

#include "ams_config.hpp"
#include "bms_service.hpp"
#include "current_service.hpp"
#include "telemetry_encoders.hpp"
#include "vehicle_service.hpp"

#include "unity.h"

#include <cstdint>
#include <cstring>

namespace {

ams::BmsState     g_bms;
ams::CurrentState g_cur;
ams::VehicleState g_veh;

void reset_all() {
    std::memset(&g_bms, 0, sizeof(g_bms));
    std::memset(&g_cur, 0, sizeof(g_cur));
    std::memset(&g_veh, 0, sizeof(g_veh));
}

}  // namespace

// ---------------------------------------------------------------------------
// 0x4A0 status frame
// ---------------------------------------------------------------------------

extern "C" void test_telem_status_layout(void) {
    reset_all();
    g_bms.module_online_mask = 0x1F;
    g_bms.min_cell_mV        = 0x1234;
    g_bms.max_cell_mV        = 0xABCD;

    const auto f = ams::telemetry::encode_status(
        /*state=*/3u, /*ams_ok=*/1u, g_bms);

    TEST_ASSERT_EQUAL_UINT8(0x03, f[0]);  // state Run
    TEST_ASSERT_EQUAL_UINT8(0x01, f[1]);  // ams_ok asserted
    TEST_ASSERT_EQUAL_UINT8(0x1F, f[2]);  // all 5 modules online
    TEST_ASSERT_EQUAL_UINT8(0x00, f[3]);  // reserved
    TEST_ASSERT_EQUAL_UINT8(0x12, f[4]);  // min_cell_mV BE high
    TEST_ASSERT_EQUAL_UINT8(0x34, f[5]);  // min_cell_mV BE low
    TEST_ASSERT_EQUAL_UINT8(0xAB, f[6]);  // max_cell_mV BE high
    TEST_ASSERT_EQUAL_UINT8(0xCD, f[7]);  // max_cell_mV BE low
}

extern "C" void test_telem_status_ams_ok_normalised(void) {
    reset_all();
    // Any non-zero value must normalise to 1 in byte 1.
    const auto f = ams::telemetry::encode_status(0u, 0xFEu, g_bms);
    TEST_ASSERT_EQUAL_UINT8(1u, f[1]);
}

// ---------------------------------------------------------------------------
// 0x4A1 pack frame
// ---------------------------------------------------------------------------

extern "C" void test_telem_pack_layout_positive_current(void) {
    reset_all();
    g_bms.pack_voltage_mV = 0x12345678u;
    g_cur.filtered_mA     = 0x0001ABCD;     // +109,517 mA

    const auto f = ams::telemetry::encode_pack(g_bms, g_cur);

    // pack mV little-endian
    TEST_ASSERT_EQUAL_UINT8(0x78, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x56, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x34, f[2]);
    TEST_ASSERT_EQUAL_UINT8(0x12, f[3]);
    // current mA little-endian
    TEST_ASSERT_EQUAL_UINT8(0xCD, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xAB, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0x01, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[7]);
}

extern "C" void test_telem_pack_layout_negative_current(void) {
    reset_all();
    g_bms.pack_voltage_mV = 0;
    g_cur.filtered_mA     = -1000;          // 1 A charge

    const auto f = ams::telemetry::encode_pack(g_bms, g_cur);

    // -1000 in two's complement int32 = 0xFFFFFC18
    TEST_ASSERT_EQUAL_UINT8(0x18, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFC, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, f[7]);
}

// ---------------------------------------------------------------------------
// 0x4A2 temps + heartbeat frame
// ---------------------------------------------------------------------------

extern "C" void test_telem_temps_layout(void) {
    reset_all();
    g_bms.min_tempC = 18;
    g_bms.max_tempC = 47;
    g_bms.avg_tempC = 30;
    g_veh.dc_bus_V  = 350;

    const auto f = ams::telemetry::encode_temps(
        g_bms, g_veh,
        /*heartbeat=*/0x42u,
        /*tx_fail_count_lo=*/0x99u,
        /*bms_task_state_byte=*/0xA3u,
        /*bms_seed_count_lo=*/0x55u,
        /*free_heap_kb=*/0x21u);

    TEST_ASSERT_EQUAL_INT8 (18,   static_cast<std::int8_t>(f[0]));
    TEST_ASSERT_EQUAL_INT8 (47,   static_cast<std::int8_t>(f[1]));
    TEST_ASSERT_EQUAL_INT8 (30,   static_cast<std::int8_t>(f[2]));
#if defined(AMS_BMS_HIL_STUB)
    // Bench layout: bytes 3..5 carry #123 diag probes; dc_bus_V dropped.
    TEST_ASSERT_EQUAL_UINT8(0xA3, f[3]);   // bms_task_state_byte
    TEST_ASSERT_EQUAL_UINT8(0x55, f[4]);   // bms_seed_count_lo
    TEST_ASSERT_EQUAL_UINT8(0x21, f[5]);   // free_heap_kb
#else
    // Flight layout: dc_bus_V LE in 3..4, byte 5 reserved.
    TEST_ASSERT_EQUAL_UINT8(0x5E, f[3]);   // 350 = 0x015E, LE
    TEST_ASSERT_EQUAL_UINT8(0x01, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00, f[5]);
#endif
    TEST_ASSERT_EQUAL_UINT8(0x99, f[6]);   // tx_fail_count_lo (#123)
    TEST_ASSERT_EQUAL_UINT8(0x42, f[7]);   // heartbeat
}

extern "C" void test_telem_temps_clip_to_int8_range(void) {
    reset_all();
    g_bms.min_tempC = -300;      // below int8 min
    g_bms.max_tempC =  300;      // above int8 max
    g_bms.avg_tempC =    5;

    const auto f = ams::telemetry::encode_temps(g_bms, g_veh, 0u, /*tx_fail=*/0u);

    TEST_ASSERT_EQUAL_INT8(-128, static_cast<std::int8_t>(f[0]));
    TEST_ASSERT_EQUAL_INT8( 127, static_cast<std::int8_t>(f[1]));
    TEST_ASSERT_EQUAL_INT8(   5, static_cast<std::int8_t>(f[2]));
}

extern "C" void test_telem_temps_heartbeat_passthrough(void) {
    reset_all();
    for (std::uint8_t hb : { 0u, 1u, 127u, 200u, 255u }) {
        const auto f = ams::telemetry::encode_temps(g_bms, g_veh, hb, /*tx_fail=*/0u);
        TEST_ASSERT_EQUAL_UINT8(hb, f[7]);
    }
}

// ---------------------------------------------------------------------------
// Field independence: changing one BmsState field must not perturb
// unrelated bytes in any frame.
// ---------------------------------------------------------------------------

extern "C" void test_telem_field_independence(void) {
    reset_all();
    const auto base_status = ams::telemetry::encode_status(0, 0, g_bms);

    // Flip min_cell_mV. Bytes 4..5 should change, others must not.
    g_bms.min_cell_mV = 0xABCD;
    const auto perturbed = ams::telemetry::encode_status(0, 0, g_bms);
    TEST_ASSERT_EQUAL_UINT8(base_status[0], perturbed[0]);
    TEST_ASSERT_EQUAL_UINT8(base_status[1], perturbed[1]);
    TEST_ASSERT_EQUAL_UINT8(base_status[2], perturbed[2]);
    TEST_ASSERT_EQUAL_UINT8(base_status[3], perturbed[3]);
    TEST_ASSERT_NOT_EQUAL  (base_status[4], perturbed[4]);
    TEST_ASSERT_NOT_EQUAL  (base_status[5], perturbed[5]);
    TEST_ASSERT_EQUAL_UINT8(base_status[6], perturbed[6]);
    TEST_ASSERT_EQUAL_UINT8(base_status[7], perturbed[7]);
}

// 0x4A3 diag frame (#123). Bench-only; remove when seeder issue closes.
extern "C" void test_telem_diag_layout(void) {
    const auto f = ams::telemetry::encode_diag(
        /*bms_poll_task_state=*/0xA3u,        // Blocked, healthy
        /*bms_seed_count_lo=*/  0x42u,
        /*free_heap_bytes=*/    0x1234u);
    TEST_ASSERT_EQUAL_UINT8(0xA3u, f[0]);
    TEST_ASSERT_EQUAL_UINT8(0x42u, f[1]);
    TEST_ASSERT_EQUAL_UINT8(0x12u, f[2]);   // BE high byte
    TEST_ASSERT_EQUAL_UINT8(0x34u, f[3]);   // BE low byte
    TEST_ASSERT_EQUAL_UINT8(0x00u, f[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, f[5]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, f[6]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, f[7]);
}

extern "C" void test_telem_diag_free_heap_saturates_at_64k(void) {
    const auto f1 = ams::telemetry::encode_diag(0, 0, 0xFFFFu);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f1[2]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f1[3]);

    const auto f2 = ams::telemetry::encode_diag(0, 0, 0xFFFFFFFFu);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f2[2]);   // saturated, not truncated
    TEST_ASSERT_EQUAL_UINT8(0xFFu, f2[3]);

    const auto f3 = ams::telemetry::encode_diag(0, 0, 0u);
    TEST_ASSERT_EQUAL_UINT8(0x00u, f3[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00u, f3[3]);
}
