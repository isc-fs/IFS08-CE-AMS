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

// test_current_service.cpp
void test_current_adc_zero_point_reads_near_zero(void);
void test_current_adc_discharge_positive(void);
void test_current_adc_charge_negative(void);
void test_current_adc_symmetric_around_zero(void);

// test_vehicle_service.cpp
void test_decode_dc_bus_little_endian(void);
void test_decode_start_button(void);
void test_update_dc_bus_frame(void);
void test_update_start_button_frame(void);
void test_acu_frame_on_wrong_bus_rejected(void);
void test_acu_unknown_id_rejected(void);

// test_safety_predicates.cpp
void test_predicates_nominal_no_fault(void);
void test_predicates_force_error(void);
void test_predicates_sdc_open(void);
void test_predicates_cell_undervoltage(void);
void test_predicates_cell_overvoltage(void);
void test_predicates_cell_overtemp(void);
void test_predicates_bms_stale(void);
void test_predicates_current_overlimit(void);
void test_predicates_current_stale(void);
void test_predicates_vcu_stale(void);

// test_state_machine.cpp
void test_fsm_start_waits_without_button(void);
void test_fsm_start_transitions_to_precharge_on_button(void);
void test_fsm_start_transitions_to_charge_on_charger(void);
void test_fsm_precharge_reaches_target(void);
void test_fsm_precharge_stays_below_target(void);
void test_fsm_transition_holds_then_runs(void);
void test_fsm_run_to_charge_and_back(void);
void test_fsm_any_state_to_error_on_fault(void);
void test_fsm_error_is_sticky(void);

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

    RUN_TEST(test_current_adc_zero_point_reads_near_zero);
    RUN_TEST(test_current_adc_discharge_positive);
    RUN_TEST(test_current_adc_charge_negative);
    RUN_TEST(test_current_adc_symmetric_around_zero);

    RUN_TEST(test_decode_dc_bus_little_endian);
    RUN_TEST(test_decode_start_button);
    RUN_TEST(test_update_dc_bus_frame);
    RUN_TEST(test_update_start_button_frame);
    RUN_TEST(test_acu_frame_on_wrong_bus_rejected);
    RUN_TEST(test_acu_unknown_id_rejected);

    RUN_TEST(test_predicates_nominal_no_fault);
    RUN_TEST(test_predicates_force_error);
    RUN_TEST(test_predicates_sdc_open);
    RUN_TEST(test_predicates_cell_undervoltage);
    RUN_TEST(test_predicates_cell_overvoltage);
    RUN_TEST(test_predicates_cell_overtemp);
    RUN_TEST(test_predicates_bms_stale);
    RUN_TEST(test_predicates_current_overlimit);
    RUN_TEST(test_predicates_current_stale);
    RUN_TEST(test_predicates_vcu_stale);

    RUN_TEST(test_fsm_start_waits_without_button);
    RUN_TEST(test_fsm_start_transitions_to_precharge_on_button);
    RUN_TEST(test_fsm_start_transitions_to_charge_on_charger);
    RUN_TEST(test_fsm_precharge_reaches_target);
    RUN_TEST(test_fsm_precharge_stays_below_target);
    RUN_TEST(test_fsm_transition_holds_then_runs);
    RUN_TEST(test_fsm_run_to_charge_and_back);
    RUN_TEST(test_fsm_any_state_to_error_on_fault);
    RUN_TEST(test_fsm_error_is_sticky);

    return UNITY_END();
}

}  // extern "C"
