// SPDX-License-Identifier: proprietary
//
// Unity entry point. Add new test functions here as the suite grows.

#include "unity.h"

extern "C" {

// test_bms_service.cpp
void test_bms_ltc_clean_response_decodes_all_cells(void);
void test_bms_ltc_pec_fail_on_one_ic_marks_module_stale(void);
void test_bms_ltc_pec_fail_increments_error_counter(void);
void test_bms_ltc_short_buffer_rejected(void);
void test_bms_ltc_null_buffer_rejected(void);
void test_bms_ltc_is_healthy_false_after_staleness(void);
void test_bms_temp_sweep_room_temp_on_one_channel(void);
void test_bms_temp_hotter_voltage_gives_hotter_reading(void);
void test_bms_temp_rail_reading_skips_slot(void);
void test_bms_temp_pec_fail_skips_slot(void);
void test_bms_temp_bad_channel_idx_rejected(void);
void test_bms_temp_short_buffer_rejected(void);

// test_current_service.cpp
void test_current_adc_zero_point_reads_near_zero(void);
void test_current_adc_discharge_positive(void);
void test_current_adc_charge_negative(void);
void test_current_adc_symmetric_around_zero(void);
void test_current_adc_upper_rail_near_82500_mA(void);
void test_current_adc_lower_rail_near_minus_82500_mA(void);

// test_vehicle_service.cpp
void test_decode_dc_bus_little_endian(void);
void test_update_dc_bus_frame(void);
void test_acu_frame_on_wrong_bus_rejected(void);
void test_acu_unknown_id_rejected(void);

// test_bootloader.cpp
void test_bootloader_trigger_exact_match(void);
void test_bootloader_trigger_wrong_bus(void);
void test_bootloader_trigger_wrong_id(void);
void test_bootloader_trigger_wrong_dlc(void);
void test_bootloader_trigger_each_magic_byte_flipped(void);
void test_bootloader_trigger_trailing_bytes_ignored(void);

// test_safety_predicates.cpp
void test_predicates_nominal_no_fault(void);
void test_predicates_force_error(void);
void test_predicates_cell_undervoltage(void);
void test_predicates_cell_overvoltage(void);
void test_predicates_cell_overtemp(void);
void test_predicates_bms_stale(void);
void test_predicates_current_overlimit(void);
void test_predicates_current_stale(void);
void test_predicates_vcu_stale(void);
void test_predicates_boot_grace_suppresses_data_predicates(void);
void test_predicates_boot_grace_does_not_suppress_force_error(void);
void test_predicates_after_grace_zero_ticks_faults(void);

// test_state_machine.cpp
void test_fsm_start_waits_without_tsms_or_dash_chg(void);
void test_fsm_start_waits_with_tsms_only(void);
void test_fsm_start_waits_with_dash_chg_only(void);
void test_fsm_start_to_precharge_on_both_inputs(void);
void test_fsm_precharge_reaches_target(void);
void test_fsm_precharge_stays_below_target(void);
void test_fsm_precharge_timeout_forces_error(void);
void test_fsm_transition_holds_then_runs_in_car_mode(void);
void test_fsm_transition_holds_then_charges_in_charger_mode(void);
void test_fsm_transition_undecided_mode_forces_error(void);
void test_fsm_transition_drops_voltage_to_error(void);
void test_fsm_run_stays_while_tsms_and_dash_chg_high(void);
void test_fsm_run_to_error_on_tsms_drop(void);
void test_fsm_run_to_error_on_dash_chg_drop(void);
void test_fsm_charge_to_error_on_tsms_or_dash_chg_drop(void);
void test_fsm_any_state_to_error_on_fault(void);
void test_fsm_error_is_sticky(void);

// test_ltc6811_decode.cpp
void test_ltc6811_pec_datasheet_wrcfga(void);
void test_ltc6811_pec_empty(void);
void test_ltc6811_pec_deterministic(void);
void test_ltc6811_pec_sensitivity(void);
void test_ltc6811_pec_lsb_is_zero(void);
void test_ltc6811_pack_command_wrcfga(void);
void test_ltc6811_pack_command_rdcva(void);
void test_ltc6811_adcv_norm_dcp1_all_cells(void);
void test_ltc6811_adcv_norm_dcp0_all_cells(void);
void test_ltc6811_adax_norm_all_aux(void);
void test_ltc6811_decode_cell_voltage_roundtrip(void);
void test_ltc6811_decode_cell_voltage_bad_pec(void);
void test_ltc6811_decode_aux_voltage_roundtrip(void);
void test_ltc6811_build_write_frame_chain10(void);
void test_ltc6811_build_write_frame_short_buffer_safe(void);
void test_ltc6811_pack_adg731_channel_5(void);
void test_ltc6811_pack_adg731_channel_0(void);
void test_ltc6811_pack_adg731_channel_31(void);
void test_ltc6811_pack_adg731_channel_masked(void);
void test_ltc6811_chain_discovery_all_ten_valid(void);
void test_ltc6811_chain_discovery_nine_then_bad(void);
void test_ltc6811_chain_discovery_ten_plus_trailing_garbage(void);
void test_ltc6811_chain_discovery_first_bad(void);
void test_ltc6811_chain_discovery_short_buffer(void);
void test_ltc6811_chain_discovery_null_safe(void);

// test_telemetry_encoders.cpp
void test_telem_status_layout(void);
void test_telem_status_ams_ok_normalised(void);
void test_telem_pack_layout_positive_current(void);
void test_telem_pack_layout_negative_current(void);
void test_telem_temps_layout(void);
void test_telem_temps_clip_to_int8_range(void);
void test_telem_temps_heartbeat_passthrough(void);
void test_telem_field_independence(void);
void test_telem_diag_layout(void);
void test_telem_diag_free_heap_saturates_at_64k(void);

// test_balance_controller.cpp
void test_balance_uniform_pack_no_discharge(void);
void test_balance_single_hot_cell_in_charge(void);
void test_balance_caps_at_max_active_per_module(void);
void test_balance_disabled_outside_charge(void);
void test_balance_thermal_lockout(void);
void test_balance_threshold_strict_inequality(void);

// test_sil_scenarios.cpp
void test_sil_nominal_startup_to_run(void);
void test_sil_precharge_timeout_to_error(void);
void test_sil_bms_dropout_in_run(void);
void test_sil_charger_path(void);
void test_sil_tsms_drop_in_run_latches_error(void);

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_bms_ltc_clean_response_decodes_all_cells);
    RUN_TEST(test_bms_ltc_pec_fail_on_one_ic_marks_module_stale);
    RUN_TEST(test_bms_ltc_pec_fail_increments_error_counter);
    RUN_TEST(test_bms_ltc_short_buffer_rejected);
    RUN_TEST(test_bms_ltc_null_buffer_rejected);
    RUN_TEST(test_bms_ltc_is_healthy_false_after_staleness);
    RUN_TEST(test_bms_temp_sweep_room_temp_on_one_channel);
    RUN_TEST(test_bms_temp_hotter_voltage_gives_hotter_reading);
    RUN_TEST(test_bms_temp_rail_reading_skips_slot);
    RUN_TEST(test_bms_temp_pec_fail_skips_slot);
    RUN_TEST(test_bms_temp_bad_channel_idx_rejected);
    RUN_TEST(test_bms_temp_short_buffer_rejected);

    RUN_TEST(test_current_adc_zero_point_reads_near_zero);
    RUN_TEST(test_current_adc_discharge_positive);
    RUN_TEST(test_current_adc_charge_negative);
    RUN_TEST(test_current_adc_symmetric_around_zero);
    RUN_TEST(test_current_adc_upper_rail_near_82500_mA);
    RUN_TEST(test_current_adc_lower_rail_near_minus_82500_mA);

    RUN_TEST(test_decode_dc_bus_little_endian);
    RUN_TEST(test_update_dc_bus_frame);
    RUN_TEST(test_acu_frame_on_wrong_bus_rejected);
    RUN_TEST(test_acu_unknown_id_rejected);

    RUN_TEST(test_bootloader_trigger_exact_match);
    RUN_TEST(test_bootloader_trigger_wrong_bus);
    RUN_TEST(test_bootloader_trigger_wrong_id);
    RUN_TEST(test_bootloader_trigger_wrong_dlc);
    RUN_TEST(test_bootloader_trigger_each_magic_byte_flipped);
    RUN_TEST(test_bootloader_trigger_trailing_bytes_ignored);

    RUN_TEST(test_predicates_nominal_no_fault);
    RUN_TEST(test_predicates_force_error);
    RUN_TEST(test_predicates_cell_undervoltage);
    RUN_TEST(test_predicates_cell_overvoltage);
    RUN_TEST(test_predicates_cell_overtemp);
    RUN_TEST(test_predicates_bms_stale);
    RUN_TEST(test_predicates_current_overlimit);
    RUN_TEST(test_predicates_current_stale);
    RUN_TEST(test_predicates_vcu_stale);
    RUN_TEST(test_predicates_boot_grace_suppresses_data_predicates);
    RUN_TEST(test_predicates_boot_grace_does_not_suppress_force_error);
    RUN_TEST(test_predicates_after_grace_zero_ticks_faults);

    RUN_TEST(test_fsm_start_waits_without_tsms_or_dash_chg);
    RUN_TEST(test_fsm_start_waits_with_tsms_only);
    RUN_TEST(test_fsm_start_waits_with_dash_chg_only);
    RUN_TEST(test_fsm_start_to_precharge_on_both_inputs);
    RUN_TEST(test_fsm_precharge_reaches_target);
    RUN_TEST(test_fsm_precharge_stays_below_target);
    RUN_TEST(test_fsm_precharge_timeout_forces_error);
    RUN_TEST(test_fsm_transition_holds_then_runs_in_car_mode);
    RUN_TEST(test_fsm_transition_holds_then_charges_in_charger_mode);
    RUN_TEST(test_fsm_transition_undecided_mode_forces_error);
    RUN_TEST(test_fsm_transition_drops_voltage_to_error);
    RUN_TEST(test_fsm_run_stays_while_tsms_and_dash_chg_high);
    RUN_TEST(test_fsm_run_to_error_on_tsms_drop);
    RUN_TEST(test_fsm_run_to_error_on_dash_chg_drop);
    RUN_TEST(test_fsm_charge_to_error_on_tsms_or_dash_chg_drop);
    RUN_TEST(test_fsm_any_state_to_error_on_fault);
    RUN_TEST(test_fsm_error_is_sticky);

    RUN_TEST(test_ltc6811_pec_datasheet_wrcfga);
    RUN_TEST(test_ltc6811_pec_empty);
    RUN_TEST(test_ltc6811_pec_deterministic);
    RUN_TEST(test_ltc6811_pec_sensitivity);
    RUN_TEST(test_ltc6811_pec_lsb_is_zero);
    RUN_TEST(test_ltc6811_pack_command_wrcfga);
    RUN_TEST(test_ltc6811_pack_command_rdcva);
    RUN_TEST(test_ltc6811_adcv_norm_dcp1_all_cells);
    RUN_TEST(test_ltc6811_adcv_norm_dcp0_all_cells);
    RUN_TEST(test_ltc6811_adax_norm_all_aux);
    RUN_TEST(test_ltc6811_decode_cell_voltage_roundtrip);
    RUN_TEST(test_ltc6811_decode_cell_voltage_bad_pec);
    RUN_TEST(test_ltc6811_decode_aux_voltage_roundtrip);
    RUN_TEST(test_ltc6811_build_write_frame_chain10);
    RUN_TEST(test_ltc6811_build_write_frame_short_buffer_safe);
    RUN_TEST(test_ltc6811_pack_adg731_channel_5);
    RUN_TEST(test_ltc6811_pack_adg731_channel_0);
    RUN_TEST(test_ltc6811_pack_adg731_channel_31);
    RUN_TEST(test_ltc6811_pack_adg731_channel_masked);
    RUN_TEST(test_ltc6811_chain_discovery_all_ten_valid);
    RUN_TEST(test_ltc6811_chain_discovery_nine_then_bad);
    RUN_TEST(test_ltc6811_chain_discovery_ten_plus_trailing_garbage);
    RUN_TEST(test_ltc6811_chain_discovery_first_bad);
    RUN_TEST(test_ltc6811_chain_discovery_short_buffer);
    RUN_TEST(test_ltc6811_chain_discovery_null_safe);

    RUN_TEST(test_telem_status_layout);
    RUN_TEST(test_telem_status_ams_ok_normalised);
    RUN_TEST(test_telem_pack_layout_positive_current);
    RUN_TEST(test_telem_pack_layout_negative_current);
    RUN_TEST(test_telem_temps_layout);
    RUN_TEST(test_telem_temps_clip_to_int8_range);
    RUN_TEST(test_telem_temps_heartbeat_passthrough);
    RUN_TEST(test_telem_field_independence);
    RUN_TEST(test_telem_diag_layout);
    RUN_TEST(test_telem_diag_free_heap_saturates_at_64k);

    RUN_TEST(test_balance_uniform_pack_no_discharge);
    RUN_TEST(test_balance_single_hot_cell_in_charge);
    RUN_TEST(test_balance_caps_at_max_active_per_module);
    RUN_TEST(test_balance_disabled_outside_charge);
    RUN_TEST(test_balance_thermal_lockout);
    RUN_TEST(test_balance_threshold_strict_inequality);

    RUN_TEST(test_sil_nominal_startup_to_run);
    RUN_TEST(test_sil_precharge_timeout_to_error);
    RUN_TEST(test_sil_bms_dropout_in_run);
    RUN_TEST(test_sil_charger_path);
    RUN_TEST(test_sil_tsms_drop_in_run_latches_error);

    return UNITY_END();
}

}  // extern "C"
