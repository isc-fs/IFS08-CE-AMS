#!/usr/bin/env python3
"""Verify the AMS firmware is bootloader-compatible.

Constraints (matches isc-fs/stm32-can-bootloader Core/Inc/bl_memmap.h):

  - sector 0  (0x08000000..0x0801FFFF) reserved for the bootloader
  - sector 7  (0x080E0000..0x080FFFFF) reserved for BL NVM + app metadata
  - app region: sectors 1..6 (0x08020000..0x080DFFFF), 768 KB

The check is:
  1. No section's LMA range falls in sector 0.
  2. No section's LMA range overflows sector 7.
  3. The .isr_vector section starts EXACTLY at 0x08020000 -- the
     bootloader jumps to *(BL_APP_BASE+4) which is the reset vector
     in this table.

Usage:
    python3 scripts/check_flash_layout.py build/AMS.elf
    OBJDUMP=arm-none-eabi-objdump python3 scripts/check_flash_layout.py

Exits 0 on pass, 1 on any violation.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys


BL_APP_BASE = 0x0802_0000
BL_APP_END  = 0x080D_FFFF   # last byte of sector 6
FLASH_BASE  = 0x0800_0000
FLASH_END   = 0x080F_FFFF


def parse_objdump(elf: str, objdump_bin: str) -> list[tuple[str, int, int]]:
    """Return (name, lma, size) for every section that actually occupies
    flash (i.e. has the LOAD flag). BSS-like sections show a flash LMA
    in objdump output but consume no flash bytes; they're filtered out
    here based on the flag line objdump prints immediately after each
    section header."""
    out = subprocess.run(
        [objdump_bin, "-h", elf],
        check=True, capture_output=True, text=True,
    ).stdout

    rows: list[tuple[str, int, int]] = []
    # Each section is two lines in objdump -h output:
    #   "  0 .isr_vector   000002cc  08020000  08020000  00001000  2**0"
    #   "                  CONTENTS, ALLOC, LOAD, READONLY, DATA"
    row_re = re.compile(
        r"^\s*\d+\s+"
        r"(\S+)\s+"            # name
        r"([0-9a-fA-F]+)\s+"   # size
        r"[0-9a-fA-F]+\s+"     # vma  (ignored)
        r"([0-9a-fA-F]+)\s+"   # lma
        r"[0-9a-fA-F]+\s+"     # file_off (ignored)
        r"2\*\*\d+"
    )
    lines = out.splitlines()
    for i, line in enumerate(lines):
        m = row_re.match(line)
        if not m:
            continue
        flags_line = lines[i + 1] if i + 1 < len(lines) else ""
        if "LOAD" not in flags_line:
            continue  # .bss, NOLOAD, etc. -- no flash bytes consumed

        name = m.group(1)
        size = int(m.group(2), 16)
        lma  = int(m.group(3), 16)
        if size == 0:
            continue
        if not (FLASH_BASE <= lma <= FLASH_END):
            continue
        rows.append((name, lma, size))
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("elf", nargs="?", default="build/AMS.elf",
                   help="path to the linked firmware ELF")
    p.add_argument("--objdump",
                   default=os.environ.get("OBJDUMP", "arm-none-eabi-objdump"),
                   help="objdump binary (default: arm-none-eabi-objdump)")
    args = p.parse_args()

    if not os.path.isfile(args.elf):
        print(f"FAIL: ELF not found at {args.elf}", file=sys.stderr)
        return 1

    sections = parse_objdump(args.elf, args.objdump)
    if not sections:
        print("FAIL: no flash sections found in ELF (corrupt? wrong objdump?)",
              file=sys.stderr)
        return 1

    fail = False

    for name, lma, size in sections:
        end = lma + size - 1
        rng = f"0x{lma:08x}..0x{end:08x} ({size:>6} B)"

        if lma < BL_APP_BASE:
            print(f"FAIL: section {name:<14} {rng} overlaps bootloader sector 0",
                  file=sys.stderr)
            fail = True
        elif end > BL_APP_END:
            print(f"FAIL: section {name:<14} {rng} overflows into sector 7 "
                  f"(reserved for BL NVM)", file=sys.stderr)
            fail = True
        else:
            print(f"ok:   section {name:<14} {rng}")

    # .isr_vector must land exactly at BL_APP_BASE.
    isr = next((s for s in sections if s[0] == ".isr_vector"), None)
    if isr is None:
        print("FAIL: no .isr_vector section in ELF", file=sys.stderr)
        fail = True
    elif isr[1] != BL_APP_BASE:
        print(f"FAIL: .isr_vector at 0x{isr[1]:08x}, expected 0x{BL_APP_BASE:08x}",
              file=sys.stderr)
        fail = True
    else:
        print(f"ok:   .isr_vector at 0x{isr[1]:08x} "
              f"(bootloader entry point)")

    if fail:
        print("\nFAIL: flash layout incompatible with stm32-can-bootloader",
              file=sys.stderr)
        return 1

    print()
    print("PASS: flash layout compatible with stm32-can-bootloader")
    print(f"      sector 0 (0x{FLASH_BASE:08x}..0x0801FFFF) free for bootloader")
    print(f"      sector 7 (0x080E0000..0x{FLASH_END:08x}) free for BL NVM")
    return 0


if __name__ == "__main__":
    sys.exit(main())
