#!/usr/bin/env python3
"""
Generator for docs/dbc/ams.dbc -- single source of truth for the CAN
database MingoCAN (and any external decoder) needs.

Why generated:
  - 95 cell signals + 200 NTC signals + per-module aggregates = ~310
    signals. Hand-writing every entry is error-prone and the offset
    arithmetic (DBC big-endian bit numbering, especially) is exactly
    the kind of thing that wants to come out of a loop.
  - When the BMS module count or cell layout changes, regenerate
    rather than patch by hand.

DBC format reference: Vector CAN database. Key conventions:
  - Message ID is decimal (so 0x680 -> 1664).
  - Big-endian (Motorola) signals use start_bit = bit position of the
    MSB across the whole message, where bit 7 of byte 0 == 7,
    bit 0 of byte 0 == 0, bit 7 of byte 1 == 15, etc. For a BE u16
    in bytes [N, N+1], start_bit = 8*N + 7.
  - Byte order char: '0' = Motorola/BE, '1' = Intel/LE.
  - Sign char: '+' unsigned, '-' signed (two's complement).

Run:
  python3 tools/gen_dbc.py > docs/dbc/ams.dbc

Or:
  python3 tools/gen_dbc.py --check
to verify the committed file matches what the generator would produce
(useful as a CI guardrail; not wired yet).
"""

import sys
from dataclasses import dataclass, field
from typing import List, Tuple

# --- module / cell layout constants ----------------------------------------

MODULES        = 5
CELLS_PER_MOD  = 19
NTCS_PER_MOD   = 40
TOTAL_CELLS    = MODULES * CELLS_PER_MOD     # 95
TOTAL_NTCS     = MODULES * NTCS_PER_MOD      # 200

# --- helpers ----------------------------------------------------------------

def be_start_bit_for_byte(byte_idx: int) -> int:
    """Motorola/BE start bit for a multi-byte field whose MSB is at
    byte `byte_idx`. DBC convention: bit 7 of byte 0 is bit 7,
    bit 0 of byte 0 is bit 0, bit 7 of byte 1 is bit 15, etc."""
    return 8 * byte_idx + 7


def le_start_bit_for_byte(byte_idx: int) -> int:
    """Intel/LE start bit -- LSB of byte byte_idx."""
    return 8 * byte_idx


@dataclass
class Signal:
    name: str
    start_bit: int
    length: int
    byte_order: str   # '0' BE, '1' LE
    sign: str         # '+' '-'
    factor: float = 1.0
    offset: float = 0.0
    minimum: float = 0.0
    maximum: float = 0.0
    unit: str = ""
    receivers: List[str] = field(default_factory=lambda: ["Vector__XXX"])
    comment: str = ""
    # Optional value->name enum table, emitted as a DBC `VAL_` line so a
    # DBC-consuming tool can decode the enum without re-hardcoding the
    # mapping (#291). Order preserved; keep the human-readable `comment`
    # too for anyone reading the raw file.
    values: List[Tuple[int, str]] = field(default_factory=list)


@dataclass
class Message:
    can_id: int
    name: str
    dlc: int
    sender: str
    signals: List[Signal] = field(default_factory=list)
    comment: str = ""


# --- shared enum tables (emitted as DBC VAL_; see #291) ---------------------
# Single source of truth for the value->name mappings that also appear in
# the human-readable signal comments. Keep the two in sync.

FSM_STATE_VALUES = [
    (0, "Start"), (1, "Precharge"), (2, "Transition"),
    (3, "Run"), (4, "Charge"), (5, "Error"),
]
MODE_LOCKED_VALUES = [
    (0, "Undecided"), (1, "Car"), (2, "Charger"),
]
FAULT_REASON_VALUES = [
    (0, "None"), (1, "ForceError"), (2, "BmsModuleOffline"), (3, "BmsStale"),
    (4, "CellUnderVoltage"), (5, "CellOverVoltage"), (6, "CellUnderTemp"),
    (7, "CellOverTemp"), (8, "CurrentSensorFault"), (9, "CurrentStale"),
    (10, "CurrentOverLimit"), (11, "VcuStale"), (12, "FsmError"),
]


# --- message builders -------------------------------------------------------

def status_4a0() -> Message:
    m = Message(0x4A0, "AMS_status", 8, "AMS",
                comment="500 ms cadence. Top-level supervisor snapshot.")
    m.signals = [
        Signal("fsm_state", le_start_bit_for_byte(0), 8, "1", "+",
               unit="enum", values=FSM_STATE_VALUES,
               comment="0=Start, 1=Precharge, 2=Transition, 3=Run, 4=Charge, 5=Error"),
        Signal("ams_ok", le_start_bit_for_byte(1), 8, "1", "+",
               unit="bool", comment="AMS_OK GPIO readback"),
        Signal("module_online_mask", le_start_bit_for_byte(2), 8, "1", "+",
               minimum=0, maximum=31, unit="bitmask",
               comment="Bit N set iff module N's last PEC-clean response is within BmsStaleMs"),
        Signal("app_init_progress_or_reserved", le_start_bit_for_byte(3), 8, "1", "+",
               unit="enum",
               comment="Reserved (=0)."),
        Signal("min_cell_mV", be_start_bit_for_byte(4), 16, "0", "+",
               minimum=0, maximum=5000, unit="mV"),
        Signal("max_cell_mV", be_start_bit_for_byte(6), 16, "0", "+",
               minimum=0, maximum=5000, unit="mV"),
    ]
    return m


def pack_4a1() -> Message:
    m = Message(0x4A1, "AMS_pack", 8, "AMS",
                comment="500 ms cadence. Pack energy-state.")
    m.signals = [
        Signal("pack_voltage_mV", le_start_bit_for_byte(0), 32, "1", "+",
               minimum=0, maximum=600_000, unit="mV"),
        Signal("filtered_mA", le_start_bit_for_byte(4), 32, "1", "-",
               unit="mA", comment="+ discharge, − charge"),
    ]
    return m


def temps_4a2() -> Message:
    m = Message(0x4A2, "AMS_temps_diag", 8, "AMS",
                comment=("500 ms cadence. Bytes 3..4 carry dc_bus_V. "
                         "Byte 5 is the always-on cockpit byte (#251)."))
    m.signals = [
        Signal("min_tempC",   le_start_bit_for_byte(0), 8, "1", "-",
               unit="degC"),
        Signal("max_tempC",   le_start_bit_for_byte(1), 8, "1", "-",
               unit="degC"),
        Signal("avg_tempC",   le_start_bit_for_byte(2), 8, "1", "-",
               unit="degC"),
        # bytes 3..4: dc_bus_V LE u16
        Signal("dc_bus_V",    le_start_bit_for_byte(3), 16, "1", "+",
               minimum=0, maximum=600, unit="V",
               comment="DC bus voltage mirrored from VehicleService."),
        Signal("tsms_dash_chg_byte", le_start_bit_for_byte(5), 8, "1", "+",
               unit="packed",
               comment=("Cockpit snapshot (#251, always-on): bit7=sentinel, "
                        "bits3:2=mode_locked (0=Undecided/1=Car/2=Charger), "
                        "bit1=TSMS, bit0=DASH_CHG.")),
        Signal("tx_fail_count_lo", le_start_bit_for_byte(6), 8, "1", "+",
               unit="count", comment="Low byte of g_telemetry_tx_fail"),
        Signal("heartbeat",   le_start_bit_for_byte(7), 8, "1", "+",
               unit="count", comment="500 ms wraparound counter"),
    ]
    return m


def ok_precharge_020() -> Message:
    m = Message(0x020, "ECU_ok_precharge", 1, "AMS",
                comment="100 ms. 1 iff FSM in {Run, Charge}.")
    m.signals = [
        Signal("ok_precharge", le_start_bit_for_byte(0), 8, "1", "+",
               unit="bool"),
    ]
    return m


def vmin_pack_12c() -> Message:
    m = Message(0x12C, "ECU_v_cell_min", 2, "AMS",
                comment="100 ms. Pack-wide min cell voltage.")
    m.signals = [
        Signal("v_cell_min_mV", be_start_bit_for_byte(0), 16, "0", "+",
               minimum=0, maximum=5000, unit="mV"),
    ]
    return m


def vmod(can_id: int, mods: List[int], kind: str, suffix: str) -> Message:
    """kind = 'min' or 'max'; mods = list of module indices; suffix = 'A'/'B'."""
    dlc = 2 * len(mods)
    m = Message(can_id, f"ECU_v{kind}_module_{suffix}", dlc, "AMS",
                comment=f"100 ms. Per-module v{kind} for modules {mods}.")
    for i, mod in enumerate(mods):
        m.signals.append(
            Signal(f"v{kind}_module_{mod}", be_start_bit_for_byte(2 * i),
                   16, "0", "+",
                   minimum=0, maximum=5000, unit="mV",
                   comment=("Sentinel 0xFFFF if module offline"
                            if kind == "min" else
                            "Sentinel 0x0000 if module offline")))
    return m


def currents_135() -> Message:
    m = Message(0x135, "ECU_currents", 4, "AMS",
                comment="50 ms (the fast lane). Pack + DCDC currents.")
    m.signals = [
        Signal("pack_current_dA", be_start_bit_for_byte(0), 16, "0", "-",
               factor=0.1, minimum=-825, maximum=825, unit="A",
               comment="Signed deciamps (1 LSB = 0.1 A). + discharge."),
        Signal("dcdc_current_dA", be_start_bit_for_byte(2), 16, "0", "-",
               factor=0.1, unit="A"),
    ]
    return m


def tmax_mod(can_id: int, mods: List[int], suffix: str,
             include_dcdc: bool=False) -> Message:
    dlc = 2 * (len(mods) + (1 if include_dcdc else 0))
    m = Message(can_id, f"ECU_tmax_module_{suffix}", dlc, "AMS",
                comment=f"250 ms. Per-module max temperature for modules {mods}.")
    for i, mod in enumerate(mods):
        m.signals.append(
            Signal(f"tmax_module_{mod}", be_start_bit_for_byte(2 * i),
                   16, "0", "-", unit="degC",
                   comment="Sentinel INT16_MIN if offline"))
    if include_dcdc:
        m.signals.append(
            Signal("temp_dcdc", be_start_bit_for_byte(2 * len(mods)),
                   16, "0", "-", unit="degC",
                   comment="DCDC temperature stub (currently 0)"))
    return m


def vcu_100() -> Message:
    m = Message(0x100, "VCU_dc_bus_heartbeat", 2, "VCU",
                comment="20 Hz from VCU. AMS consumes for mode-lock + DC bus.")
    m.signals = [
        Signal("dc_bus_V", le_start_bit_for_byte(0), 16, "1", "+",
               minimum=0, maximum=600, unit="V"),
    ]
    return m


def charge_req_101() -> Message:
    m = Message(0x101, "Operator_charge_request", 4, "Pit_Tool",
                comment=("Operator declares 'on the charger' (#305). AMS "
                         "enters Charger mode only if fresh (<=1000 ms) AND "
                         "VCU absent at the Start->Precharge lock. Magic-gated "
                         "so noise can't flip HV mode."))
    m.signals = [
        Signal("magic", be_start_bit_for_byte(0), 32, "0", "+",
               unit="magic", comment="Must equal 0x43485247 (ascii CHRG)"),
    ]
    return m


def bl_trigger_002() -> Message:
    m = Message(0x002, "BL_boot_trigger", 4, "Pit_Tool",
                comment=("Reboot AMS into bootloader. Payload must be the "
                         "magic B007AD11. Handled by AcuCanTask before VCU "
                         "dispatch."))
    m.signals = [
        Signal("magic", be_start_bit_for_byte(0), 32, "0", "+",
               unit="magic", comment="Must equal 0xB007AD11"),
    ]
    return m


# --- pit-diag frames --------------------------------------------------------

def pit_cmd_7f0() -> Message:
    m = Message(0x7F0, "PitDiag_cmd", 4, "Pit_Tool",
                comment=("Enable/disable the pit-diag stream (#247). 4-byte "
                         "payload = DEADBEEF to enable, 00000000 to disable."))
    m.signals = [
        Signal("cmd_magic", be_start_bit_for_byte(0), 32, "0", "+",
               unit="magic",
               comment="0xDEADBEEF enable, 0x00000000 disable"),
    ]
    return m


def pit_ack_7f1() -> Message:
    m = Message(0x7F1, "PitDiag_ack", 1, "AMS",
                comment="One-shot ACK after a 0x7F0 state transition.")
    m.signals = [
        Signal("enabled", le_start_bit_for_byte(0), 8, "1", "+",
               unit="bool", comment="1 if stream just enabled, 0 if disabled"),
    ]
    return m


def pit_cells() -> List[Message]:
    """24 frames: 0x680..0x697. 4 cells/frame BE u16 mV. Row-major over
    cell_mV[5][19]; last frame has 3 real cells + 0xFFFF sentinel."""
    out = []
    for frame_idx in range(24):
        m = Message(0x680 + frame_idx, f"PitDiag_cells_{frame_idx:02d}",
                    8, "AMS",
                    comment=("Pit-diag cell-V frame. Decode: "
                             "cell_index = 4*frame_idx + slot; "
                             "module = cell_index // 19; "
                             "cell = cell_index % 19. "
                             "0xFFFF in any slot = no cell at that index."))
        for slot in range(4):
            cell_index = 4 * frame_idx + slot
            if cell_index < TOTAL_CELLS:
                mod = cell_index // CELLS_PER_MOD
                cell = cell_index % CELLS_PER_MOD
                name = f"cell_m{mod}_c{cell:02d}_mV"
                cmt = f"Module {mod}, cell {cell}"
            else:
                name = f"sentinel_slot{slot}"
                cmt = "Beyond cell 94: always 0xFFFF"
            m.signals.append(
                Signal(name, be_start_bit_for_byte(2 * slot), 16, "0", "+",
                       minimum=0, maximum=5000, unit="mV",
                       comment=cmt))
        out.append(m)
    return out


def pit_temps() -> List[Message]:
    """25 frames: 0x6A0..0x6B8. 8 NTCs/frame i8 degC."""
    out = []
    for frame_idx in range(25):
        m = Message(0x6A0 + frame_idx, f"PitDiag_temps_{frame_idx:02d}",
                    8, "AMS",
                    comment=("Pit-diag NTC-temp frame. Decode: "
                             "temp_index = 8*frame_idx + slot; "
                             "module = temp_index // 40; "
                             "temp = temp_index % 40."))
        for slot in range(8):
            temp_index = 8 * frame_idx + slot
            mod = temp_index // NTCS_PER_MOD
            temp = temp_index % NTCS_PER_MOD
            name = f"temp_m{mod}_t{temp:02d}_C"
            m.signals.append(
                Signal(name, le_start_bit_for_byte(slot), 8, "1", "-",
                       unit="degC",
                       comment=f"Module {mod}, NTC {temp}"))
        out.append(m)
    return out


def pit_fsm_status_6c0() -> Message:
    m = Message(0x6C0, "PitDiag_fsm_status", 8, "AMS",
                comment="FSM extended status frame. 1 Hz when pit-diag enabled.")
    m.signals = [
        Signal("fsm_state",     le_start_bit_for_byte(0), 8, "1", "+",
               unit="enum", values=FSM_STATE_VALUES,
               comment="0=Start, 1=Precharge, 2=Transition, 3=Run, 4=Charge, 5=Error"),
        Signal("mode_locked",   le_start_bit_for_byte(1), 8, "1", "+",
               unit="enum", values=MODE_LOCKED_VALUES,
               comment="0=Undecided, 1=Car, 2=Charger"),
        Signal("tsms_readback", le_start_bit_for_byte(2), 1, "1", "+",
               unit="bool"),
        Signal("dash_chg_readback", 8*2+1, 1, "1", "+",
               unit="bool"),
        Signal("ams_ok",        le_start_bit_for_byte(3), 8, "1", "+",
               unit="bool"),
        Signal("pec_err_total", be_start_bit_for_byte(4), 16, "0", "+",
               unit="count",
               comment="Sum of g_ltc_pec_err_count[10]; saturates at 0xFFFF"),
        Signal("fault_reason", le_start_bit_for_byte(6), 8, "1", "+",
               unit="enum", values=FAULT_REASON_VALUES,
               comment=("Predicate branch that latched ERROR (#276). "
                        "0=None, 1=ForceError, 2=BmsModuleOffline, "
                        "3=BmsStale, 4=CellUnderVoltage, 5=CellOverVoltage, "
                        "6=CellUnderTemp, 7=CellOverTemp, "
                        "8=CurrentSensorFault, 9=CurrentStale, "
                        "10=CurrentOverLimit, 11=VcuStale, "
                        "12=FsmError (FSM-driven Error path).")),
        Signal("fault_detail", le_start_bit_for_byte(7), 8, "1", "+",
               unit="raw",
               comment=("Per-branch detail. BmsStale / CellUnderVoltage / "
                        "CellOverVoltage / CellOverTemp: offending module "
                        "index 0..4, or 0xFF = none matched (inconsistent / "
                        "torn snapshot). BmsModuleOffline: live "
                        "module_online_mask. Otherwise 0.")),
    ]
    return m


def pit_timing_6c1() -> Message:
    m = Message(0x6C1, "PitDiag_timing", 8, "AMS",
                comment="V-poll cadence + last temp-sweep failure mask.")
    m.signals = [
        Signal("bms_volt_poll_ms",     be_start_bit_for_byte(0), 16, "0", "+",
               unit="ms"),
        Signal("bms_volt_poll_max_ms", be_start_bit_for_byte(2), 16, "0", "+",
               unit="ms"),
        Signal("temp_sweep_last_mask", le_start_bit_for_byte(4), 32, "1", "+",
               unit="bitmask",
               comment="1 bit per NTC channel that failed the last sweep"),
    ]
    return m


def pit_balance_a_6c2() -> Message:
    m = Message(0x6C2, "PitDiag_balance_mask_a", 8, "AMS",
                comment=("Balance DCC bits for cell indices 0..63. Bit b of "
                         "byte i represents cell (8*i + b). Cell -> module/cell: "
                         "module = idx // 19, cell = idx % 19."))
    # Just dump the 8 bytes as an opaque uint64 — MingoCAN unpacks bits.
    m.signals = [
        Signal("dcc_bits_lo64", le_start_bit_for_byte(0), 64, "1", "+",
               unit="bitmask"),
    ]
    return m


def pit_balance_b_6c3() -> Message:
    m = Message(0x6C3, "PitDiag_balance_mask_b", 8, "AMS",
                comment=("Balance DCC bits for cell indices 64..94 + per-cycle "
                         "counters. Bits 64..94 occupy bytes 0..3 (low 31 bits)."))
    m.signals = [
        Signal("dcc_bits_hi32",     le_start_bit_for_byte(0), 32, "1", "+",
               unit="bitmask",
               comment="Low 31 bits = cells 64..94; bit 31 reserved 0"),
        Signal("balance_cycles_total",  le_start_bit_for_byte(4), 16, "1", "+",
               unit="count",
               comment="Mod 65536"),
        Signal("balance_cycles_active", le_start_bit_for_byte(6), 16, "1", "+",
               unit="count",
               comment="Cycles where at least one DCC bit was set"),
    ]
    return m


def pit_boot_diag_6c4() -> Message:
    m = Message(0x6C4, "PitDiag_boot_diag", 8, "AMS",
                comment="Reset reason + App_InitTask milestone + FDCAN1 start result.")
    m.signals = [
        Signal("jump_reason", le_start_bit_for_byte(0), 32, "1", "+",
               unit="enum",
               comment=("0=POR cold boot; 0x4A554D50='JUMP' CAN trigger; "
                        "0x4D414E55='MANU' manual request")),
        Signal("app_init_progress", le_start_bit_for_byte(4), 8, "1", "+",
               unit="enum",
               comment="0..7 milestone counter, 7=clean self-exit"),
        Signal("fdcan1_start_result", le_start_bit_for_byte(5), 24, "1", "+",
               unit="HAL_StatusTypeDef",
               comment="0=HAL_OK; low 24 bits of HAL_FDCAN_Start return"),
    ]
    return m


def pit_post_mortem_6c5() -> Message:
    m = Message(0x6C5, "PitDiag_post_mortem", 8, "AMS",
                comment=("FreeRTOS stack-overflow + malloc-failed hooks "
                         "(captured to .bss; survives soft reset)."))
    m.signals = [
        Signal("stack_overflow_seen", le_start_bit_for_byte(0), 8, "1", "+",
               unit="bool",
               comment="1 if g_stack_overflow_task_addr != 0"),
        Signal("watermark_low_byte",  le_start_bit_for_byte(1), 8, "1", "+",
               unit="words", comment="Saturates at 0xFF"),
        Signal("task_addr_lo",        le_start_bit_for_byte(2), 32, "1", "+",
               unit="address",
               comment="Low 32 bits of failing task's xTaskHandle"),
        Signal("malloc_failed_count", le_start_bit_for_byte(6), 16, "1", "+",
               unit="count", comment="Saturates at 0xFFFF"),
    ]
    return m


def pit_pec_per_ic_a_6c7() -> Message:
    m = Message(0x6C7, "PitDiag_pec_per_ic_a", 8, "AMS",
                comment=("Per-IC PEC error count for chain indices 0..7 "
                         "(#258). Saturating uint8 per IC. Chain index 2m = "
                         "upper LTC of module m (cells 0..9); 2m+1 = lower "
                         "(cells 10..18). 0xFF means '>= 255 failures' -- "
                         "for diagnostic localisation the chip-identity "
                         "matters, not the precise count past saturation."))
    for ic in range(8):
        mod = ic // 2
        half = "upper" if ic % 2 == 0 else "lower"
        m.signals.append(
            Signal(f"pec_err_ic_{ic}_count", le_start_bit_for_byte(ic),
                   8, "1", "+",
                   unit="count",
                   comment=f"IC {ic} = module {mod} {half} LTC6811. Saturates at 0xFF."))
    return m


def pit_pec_per_ic_b_6c8() -> Message:
    m = Message(0x6C8, "PitDiag_pec_per_ic_b", 8, "AMS",
                comment="Per-IC PEC count for ICs 8..9 + reserved bytes (#258).")
    m.signals = [
        Signal("pec_err_ic_8_count", le_start_bit_for_byte(0), 8, "1", "+",
               unit="count",
               comment="IC 8 = module 4 upper LTC6811. Saturates at 0xFF."),
        Signal("pec_err_ic_9_count", le_start_bit_for_byte(1), 8, "1", "+",
               unit="count",
               comment="IC 9 = module 4 lower LTC6811. Saturates at 0xFF."),
    ]
    return m


def pit_fw_id_6c6() -> Message:
    m = Message(0x6C6, "PitDiag_fw_id", 8, "AMS",
                comment=("Firmware identification (post-#252). Populated from "
                         "VERSION file + git rev-parse at configure time."))
    m.signals = [
        Signal("fw_version_major", le_start_bit_for_byte(0), 8, "1", "+",
               unit="semver"),
        Signal("fw_version_minor", le_start_bit_for_byte(1), 8, "1", "+",
               unit="semver"),
        Signal("fw_version_patch", le_start_bit_for_byte(2), 8, "1", "+",
               unit="semver"),
        Signal("git_hash_0", le_start_bit_for_byte(3), 8, "1", "+",
               unit="byte"),
        Signal("git_hash_1", le_start_bit_for_byte(4), 8, "1", "+",
               unit="byte"),
        Signal("git_hash_2", le_start_bit_for_byte(5), 8, "1", "+",
               unit="byte"),
        Signal("git_hash_3", le_start_bit_for_byte(6), 8, "1", "+",
               unit="byte"),
        Signal("bl_node_id", le_start_bit_for_byte(7), 8, "1", "+",
               unit="id",
               comment="From firmware_info.reserved[0]; MingoCAN checks against the BL"),
    ]
    return m


# --- DBC emission -----------------------------------------------------------

NODES = ["AMS", "VCU", "ECU", "Pit_Tool"]

def _repo_version() -> str:
    """Semver from the repo VERSION file, stamped into the DBC VERSION
    line as a freshness signal (#291). Changes only on a version bump --
    not per-commit -- so it doesn't churn the generated file. The host
    can compare this against the running firmware's 0x6C6 fw-id frame to
    warn when its cached DBC may be behind the board on the bus."""
    import pathlib
    try:
        v = (pathlib.Path(__file__).resolve().parent.parent
             / "VERSION").read_text().strip()
        return f"AMS {v}"
    except OSError:
        return "AMS"


def emit_dbc(messages: List[Message]) -> str:
    lines = []
    lines.append(f'VERSION "{_repo_version()}"')
    lines.append("")
    lines.append("NS_ :")
    # Minimal but valid NS_ block.
    ns_items = [
        "NS_DESC_", "CM_", "BA_DEF_", "BA_", "VAL_", "CAT_DEF_", "CAT_",
        "FILTER", "BA_DEF_DEF_", "EV_DATA_", "ENVVAR_DATA_", "SGTYPE_",
        "SGTYPE_VAL_", "BA_DEF_SGTYPE_", "BA_SGTYPE_", "SIG_TYPE_REF_",
        "VAL_TABLE_", "SIG_GROUP_", "SIG_VALTYPE_", "SIGTYPE_VALTYPE_",
        "BO_TX_BU_", "BA_DEF_REL_", "BA_REL_", "BA_DEF_DEF_REL_",
        "BU_SG_REL_", "BU_EV_REL_", "BU_BO_REL_", "SG_MUL_VAL_",
    ]
    for it in ns_items:
        lines.append(f"\t{it}")
    lines.append("")
    lines.append("BS_:")
    lines.append("")
    lines.append("BU_: " + " ".join(NODES))
    lines.append("")

    for m in messages:
        lines.append(f"BO_ {m.can_id} {m.name}: {m.dlc} {m.sender}")
        for s in m.signals:
            recv = ",".join(s.receivers)
            # Default the range to the full physical extent of the type
            # when the caller didn't set one; strict parsers (cantools)
            # reject values outside [min|max], and `[0|0]` is *not* the
            # universal "unrestricted" sentinel.
            if s.maximum == s.minimum == 0:
                if s.sign == '+':
                    raw_max = (1 << s.length) - 1
                    smin, smax = 0, raw_max * s.factor + s.offset
                else:
                    raw_max = (1 << (s.length - 1)) - 1
                    smin = -(1 << (s.length - 1)) * s.factor + s.offset
                    smax = raw_max * s.factor + s.offset
            else:
                smin, smax = s.minimum, s.maximum
            # DBC range fields should be plain ints when the type fits.
            def _fmt(v):
                return f"{int(v)}" if float(v).is_integer() else f"{v}"
            lines.append(
                f" SG_ {s.name} : {s.start_bit}|{s.length}@{s.byte_order}{s.sign}"
                f" ({s.factor},{s.offset}) [{_fmt(smin)}|{_fmt(smax)}]"
                f' "{s.unit}" {recv}'
            )
        lines.append("")

    # Comments (CM_).
    for m in messages:
        if m.comment:
            lines.append(f'CM_ BO_ {m.can_id} "{m.comment}";')
        for s in m.signals:
            if s.comment:
                lines.append(
                    f'CM_ SG_ {m.can_id} {s.name} "{s.comment}";'
                )

    # Value tables (VAL_). Machine-readable enum decoding for DBC
    # consumers (#291). Format: VAL_ <id> <signal> <v> "name" ... ;
    for m in messages:
        for s in m.signals:
            if s.values:
                pairs = " ".join(f'{v} "{name}"' for v, name in s.values)
                lines.append(f"VAL_ {m.can_id} {s.name} {pairs} ;")

    lines.append("")
    return "\n".join(lines) + "\n"


def build_all() -> List[Message]:
    msgs: List[Message] = []
    # Flight telemetry (SafetyTask).
    msgs += [status_4a0(), pack_4a1(), temps_4a2()]
    # ECU TX matrix (AcuCanTask).
    msgs += [ok_precharge_020(), vmin_pack_12c()]
    msgs += [vmod(0x131, [0, 1, 2], "min", "A"),
             vmod(0x132, [3, 4],    "min", "B"),
             vmod(0x133, [0, 1, 2], "max", "A"),
             vmod(0x134, [3, 4],    "max", "B"),
             currents_135(),
             tmax_mod(0x136, [0, 1, 2], "A"),
             tmax_mod(0x137, [3, 4],    "B", include_dcdc=True)]
    # VCU + BL trigger.
    msgs += [vcu_100(), charge_req_101(), bl_trigger_002()]
    # Pit-diag.
    msgs += [pit_cmd_7f0(), pit_ack_7f1()]
    msgs += pit_cells()
    msgs += pit_temps()
    msgs += [pit_fsm_status_6c0(),
             pit_timing_6c1(),
             pit_balance_a_6c2(),
             pit_balance_b_6c3(),
             pit_boot_diag_6c4(),
             pit_post_mortem_6c5(),
             pit_fw_id_6c6(),
             pit_pec_per_ic_a_6c7(),
             pit_pec_per_ic_b_6c8()]
    return msgs


CHECK_TARGET = "docs/dbc/ams.dbc"

def main():
    import argparse, pathlib, sys
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--check", action="store_true",
        help=("Compare the generator output to the committed "
              f"{CHECK_TARGET} instead of printing to stdout. Exits "
              "non-zero if they drift -- meant for CI to enforce "
              "'whoever edited the generator also regenerated the dbc'."))
    args = ap.parse_args()

    fresh = emit_dbc(build_all())

    if not args.check:
        sys.stdout.write(fresh)
        return 0

    target = pathlib.Path(CHECK_TARGET)
    if not target.exists():
        sys.stderr.write(f"error: {target} does not exist\n")
        return 2
    committed = target.read_text()
    if committed == fresh:
        print(f"OK -- {target} matches generator output "
              f"({len(fresh.splitlines())} lines).")
        return 0
    # Compute a small diff so the PR author can see what they missed.
    import difflib
    diff = "".join(difflib.unified_diff(
        committed.splitlines(keepends=True),
        fresh.splitlines(keepends=True),
        fromfile=f"{target} (committed)",
        tofile=f"{target} (generator output)",
        n=3))
    sys.stderr.write(
        f"error: {target} is out of sync with the generator.\n"
        f"       Regenerate with:  python3 tools/gen_dbc.py > {target}\n\n")
    sys.stderr.write(diff)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
