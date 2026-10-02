#!/usr/bin/env python3
# SPDX-License-Identifier: proprietary
"""Decode the AMS binary log files (IMUnnnn.BIN, CELnnnn.BIN) to CSV.

Each .BIN file starts with a 512-byte header that carries its own schema
(Core/Inc/app/bin_log.hpp), so this decoder needs no per-stream code: it reads
the schema, unpacks the fixed-size little-endian records and applies the
declared scales.

    tools/log_decode.py CEL0003.BIN                 # -> CEL0003.csv beside it
    tools/log_decode.py *.BIN -o decoded/           # several files into a directory
    tools/log_decode.py --info IMU0003.BIN          # print the header only
    tools/log_decode.py --raw CEL0003.BIN           # raw integers, no scaling

Column names: a field with count 1 keeps its name; a field with count N becomes
name0..nameN-1; a field with count AxB becomes nameA_B (so the CEL cells are
c<module>_<cell>, the same names LOGnnnn.CSV uses). When a scale other than 1
is applied, the unit is appended (a0_g, i_A). A field flagged "z" decodes a raw
0 to an empty cell ("not measured on this record").

A file cut short by a power loss ends in a partial record; it is dropped with
a warning. Every row is decoded independently, so nothing else is lost.

Joining with LOGnnnn.CSV: all files of one index share the tick_ms clock, so
e.g. pandas.merge_asof(cel, log, left_on="t_adcv_ms", right_on="tick_ms")
attaches the state row in force at each cell frame.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

HEADER_BYTES = 512
MAGIC = b"AMSBIN1\0"
SCHEMA_OFFSET = 64

TYPES = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i"}


@dataclass
class Field:
    name: str
    type: str
    dims: tuple[int, ...]   # () for a scalar, (n,) or (a, b)
    scale: float
    unit: str
    zero_empty: bool

    @property
    def count(self) -> int:
        n = 1
        for d in self.dims:
            n *= d
        return n

    def columns(self, raw: bool) -> list[str]:
        if not self.dims:
            names = [self.name]
        elif len(self.dims) == 1:
            names = [f"{self.name}{i}" for i in range(self.dims[0])]
        else:
            a, b = self.dims
            names = [f"{self.name}{i}_{j}" for i in range(a) for j in range(b)]
        if not raw and self.scale != 1 and self.unit != "-":
            suffix = self.unit.replace("/", "_")
            names = [f"{n}_{suffix}" for n in names]
        return names


@dataclass
class Header:
    version: int
    record_size: int
    file_index: int
    open_tick_ms: int
    stream: str
    fw_version: str
    git_hash: str
    schema: list[Field]


def parse_scale(text: str) -> float:
    num, _, den = text.partition("/")
    return float(num) / float(den) if den else float(num)


def parse_schema(text: str) -> list[Field]:
    fields = []
    for lineno, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        tok = line.split()
        if len(tok) < 5 or tok[1] not in TYPES:
            raise ValueError(f"schema line {lineno} malformed: {line!r}")
        name, typ, count, scale, unit = tok[:5]
        flags = tok[5] if len(tok) > 5 else ""
        if "x" in count:
            dims = tuple(int(x) for x in count.split("x"))
        else:
            n = int(count)
            dims = () if n == 1 else (n,)
        fields.append(Field(name, typ, dims, parse_scale(scale), unit, "z" in flags))
    return fields


def read_header(blob: bytes) -> Header:
    if len(blob) < HEADER_BYTES or blob[:8] != MAGIC:
        raise ValueError("not an AMS binary log (bad magic)")
    version, record_size, file_index, open_tick = struct.unpack_from("<HHII", blob, 8)
    stream = blob[20:28].split(b"\0", 1)[0].decode("ascii", "replace")
    fw = "{}.{}.{}".format(*blob[28:31])
    git = blob[32:36].hex()
    schema_text = blob[SCHEMA_OFFSET:HEADER_BYTES].split(b"\0", 1)[0].decode("ascii")
    hdr = Header(version, record_size, file_index, open_tick, stream, fw, git,
                 parse_schema(schema_text))
    declared = struct.calcsize("<" + "".join(TYPES[f.type] * f.count for f in hdr.schema))
    if declared != record_size:
        raise ValueError(f"schema describes {declared} B but header says {record_size} B")
    return hdr


def fmt(v: float) -> str:
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def decode(path: Path, out: Path, raw: bool) -> int:
    blob = path.read_bytes()
    hdr = read_header(blob)
    rec = struct.Struct("<" + "".join(TYPES[f.type] * f.count for f in hdr.schema))
    body = blob[HEADER_BYTES:]
    n, tail = divmod(len(body), rec.size)
    if tail:
        print(f"{path.name}: dropped a partial last record ({tail} of {rec.size} B)",
              file=sys.stderr)

    columns = [c for f in hdr.schema for c in f.columns(raw)]
    with out.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(columns)
        for k in range(n):
            vals = rec.unpack_from(body, k * rec.size)
            row, i = [], 0
            for f in hdr.schema:
                for v in vals[i:i + f.count]:
                    if f.zero_empty and v == 0:
                        row.append("")
                    elif raw or f.scale == 1:
                        row.append(str(v))
                    else:
                        row.append(fmt(v * f.scale))
                i += f.count
            w.writerow(row)
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("-o", "--out", type=Path,
                    help="output file (one input) or directory (several)")
    ap.add_argument("--raw", action="store_true", help="write raw integers, no scaling")
    ap.add_argument("--info", action="store_true", help="print the header and exit")
    args = ap.parse_args()

    status = 0
    for path in args.files:
        try:
            if args.info:
                h = read_header(path.read_bytes()[:HEADER_BYTES])
                size = path.stat().st_size
                print(f"{path.name}: stream {h.stream}, index {h.file_index}, "
                      f"opened at tick {h.open_tick_ms} ms, fw {h.fw_version} ({h.git_hash}), "
                      f"{h.record_size} B/record, "
                      f"{(size - HEADER_BYTES) // h.record_size} records")
                for f in h.schema:
                    dims = "x".join(map(str, f.dims)) or "1"
                    print(f"  {f.name:<10} {f.type:<4} {dims:<6} x{f.scale:<12g} {f.unit}"
                          f"{'  (0 = empty)' if f.zero_empty else ''}")
                continue
            if args.out and (len(args.files) > 1 or args.out.is_dir()):
                args.out.mkdir(parents=True, exist_ok=True)
                out = args.out / (path.stem + ".csv")
            else:
                out = args.out or path.with_suffix(".csv")
            n = decode(path, out, args.raw)
            print(f"{path.name}: {n} records -> {out}")
        except (OSError, ValueError, struct.error) as e:
            print(f"{path.name}: {e}", file=sys.stderr)
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
