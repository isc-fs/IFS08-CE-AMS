# AMS DBC files

CAN database describing every frame the AMS firmware emits or consumes on FDCAN1. Loaded by MingoCAN ([`isc-fs/can-flasher`](https://github.com/isc-fs/can-flasher)) to decode the live stream — both the flight telemetry and the runtime-toggleable pit-diag firehose.

## Files

| File | Generated? | Source of truth |
|---|---|---|
| `ams.dbc` | yes | the **code-first CAN DSL** — `Core/Inc/can/messages/*.def` → [`tools/dbc_dump.cpp`](../../tools/dbc_dump.cpp) |

> Generated straight from the firmware's own encoder layouts (the same
> `.def` files the C++ encoders are built from), so the DBC can't drift
> from the firmware. This is lean by design — no `CM_` comments or `VAL_`
> enum tables; those are re-added downstream by the DBCinator bot
> (`isc-fs/IFS08-DBCinator`).

## What's in it

78 messages, 384 signals covering:

| Group | IDs | What |
|---|---|---|
| **AMS flight telemetry** | `0x4A0` / `0x4A1` / `0x4A2` @ 500 ms + `0x4A4` @ 100 ms | status / pack / temps+diag; `0x4A4` = always-on relay-status snapshot (contactor + AMS_OK read-backs) |
| **ECU TX matrix** | `0x020`, `0x12C`, `0x131..0x137` | feeds the ECU's wireless telemetry uplink |
| **External RX** | `0x100` (VCU), `0x002` (BL trigger) | what the AMS consumes |
| **Pit-diag enable / ACK** | `0x7F0` / `0x7F1` | runtime toggle |
| **Pit-diag stream** | `0x680..0x697` (24 cell-V), `0x6A0..0x6B8` (25 temps), `0x6C0..0x6C9` (FSM+fault-reason / timing / balance / boot diag / post-mortem / firmware ID / per-IC PEC / comms-health = FDCAN1 Bus-Off recovery count) | 59 frames @ 1 Hz when enabled |

## Decoding from the wire

```python
import cantools, can
db = cantools.database.load_file('docs/dbc/ams.dbc')
bus = can.interface.Bus(channel='can0', interface='socketcan')
for msg in bus:
    try:
        decoded = db.decode_message(msg.arbitration_id, msg.data)
        m = db.get_message_by_frame_id(msg.arbitration_id)
        print(f"{m.name}: {decoded}")
    except KeyError:
        pass   # frame ID not in DBC
```

## Big-endian bit numbering

Most AMS-side values are big-endian (the firmware uses `>> 8` / `& 0xFF` to pack). The DBC follows Vector's Motorola convention: for a BE u16 in bytes `[N, N+1]`, the `start_bit` is `8*N + 7` (the MSB of byte N). The DSL emits this automatically — see `FIELD_BE` in `Core/Inc/can/can_codecs.hpp` (the descriptor uses `8*byte+7`).

## Regenerating

```sh
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump
/tmp/dbc_dump > docs/dbc/ams.dbc
```

Validate with cantools:

```sh
pip3 install --user cantools
python3 -c "import cantools; db = cantools.database.load_file('docs/dbc/ams.dbc'); print(f'{len(db.messages)} messages OK')"
```

To change a frame's layout, edit its `Core/Inc/can/messages/*.def` (or the
array families in `can_codecs.hpp` for the cell/temp grids) and regenerate.
Don't hand-edit `ams.dbc` — CI's "DBC matches code" check (and the next
regen) will overwrite it.

## Versioning

The DBC file's identity (frame layouts, signal names, scaling factors) is part of the AMS wire contract. Any change here is a coordination point with:

- MingoCAN (`isc-fs/can-flasher#252`)
- The VCU team (consumes `0x4A0..0x4A2` + the ECU TX matrix)
- Bench tools / loggers that depend on signal names

When the firmware's wire format changes (a new signal, a renamed field, a different scaling), regenerate and PR alongside the firmware change.
