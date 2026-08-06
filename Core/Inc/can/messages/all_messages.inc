/* SPDX-License-Identifier: proprietary */
/*
 * The message registry. can_codecs.hpp #includes THIS file once per
 * expansion pass (struct / encode / decode / descriptor), so adding a
 * message to the DSL is a single line here plus its .def. Each .def is
 * expanded under whatever CAN_MSG / FIELD_* macros the including pass
 * has defined.
 *
 * Phase 2a scope: the three AMS telemetry frames (0x4A0/4A1/4A2).
 * Phase 2b adds the ECU TX matrix (0x020/0x12C/0x131..0x137).
 * Phase 2c adds the FIXED-layout pit-diag frames (0x6C0..0x6C8). The
 * parameterised cell/temp grids (0x680../0x6A0..) are NOT here yet --
 * they are windowed array views selected at runtime and need a
 * multiplexed-message representation (tracked in #365).
 */

#include "ams_status.def"
#include "ams_pack.def"
#include "ams_temps.def"
#include "relay_status.def"
#include "ams_fw_health.def"

#include "acu_ok_precharge.def"
#include "acu_discharge_interlock.def"
#include "acu_v_cell_min.def"
#include "acu_vmin_module_a.def"
#include "acu_vmin_module_b.def"
#include "acu_vmax_module_a.def"
#include "acu_vmax_module_b.def"
#include "acu_currents.def"
#include "acu_tmax_module_a.def"
#include "acu_tmax_module_b.def"
#include "acu_soc.def"

#include "pit_fsm_status.def"
#include "pit_timing.def"
#include "pit_balance_mask_a.def"
#include "pit_balance_mask_b.def"
#include "pit_boot_diag.def"
#include "pit_post_mortem.def"
#include "pit_fw_id.def"
#include "pit_pec_per_ic_a.def"
#include "pit_pec_per_ic_b.def"
#include "pit_comms_health.def"
#include "pit_balance_health.def"
#include "pit_ack.def"

/* RX frames AMS consumes (sender = VCU / Pit_Tool). Carried here so
 * dbc_dump covers the full wire contract; ownership moves to the
 * sender's repo once DBCinator centralises (Phase 3 / #365). */
#include "rx_vcu_dc_bus.def"
#include "rx_operator_charge_request.def"
#include "rx_operator_balance_override.def"
#include "rx_operator_balance_modules.def"
#include "rx_pitdiag_cmd.def"
#include "rx_bl_boot_trigger.def"
