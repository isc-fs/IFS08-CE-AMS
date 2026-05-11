// SPDX-License-Identifier: proprietary
//
// Unity entry point. Add new test functions here as the suite grows.

#include "unity.h"

extern "C" {

// test_bms_service.cpp
void test_voltage_frame_module0_decodes_cells_0_3(void);
void test_voltage_frame_module4_decodes(void);
void test_voltage_frame_idx4_handles_three_cells(void);
void test_temperature_frame_decodes_signed_int8(void);
void test_frame_on_wrong_bus_is_rejected(void);
void test_unknown_id_is_rejected(void);
void test_is_healthy_requires_all_modules(void);
void test_is_healthy_false_after_staleness(void);

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_voltage_frame_module0_decodes_cells_0_3);
    RUN_TEST(test_voltage_frame_module4_decodes);
    RUN_TEST(test_voltage_frame_idx4_handles_three_cells);
    RUN_TEST(test_temperature_frame_decodes_signed_int8);
    RUN_TEST(test_frame_on_wrong_bus_is_rejected);
    RUN_TEST(test_unknown_id_is_rejected);
    RUN_TEST(test_is_healthy_requires_all_modules);
    RUN_TEST(test_is_healthy_false_after_staleness);

    return UNITY_END();
}

}  // extern "C"
