#!/usr/bin/env python3
"""
BMS isoSPI live monitor for the AMS bare-LTC harness (feat/ltc-bare-comms).

Reads the harness CAN broadcast over PCAN and shows a live dashboard of the
PEC-clean IC count + per-cell voltages — i.e. exactly the `can-flasher replay
record` + decode loop, but continuous and self-serve.

Usage:
    python3 tools/bms_monitor.py                  # curses dashboard (default)
    python3 tools/bms_monitor.py --plain          # plain auto-refreshing text
    python3 tools/bms_monitor.py --plain --once   # one snapshot, then exit
    python3 tools/bms_monitor.py --channel PCAN_USBBUS1 --bitrate 500000

NOTE: this is the SOLE reader of the bus while it runs — quit it (q / Ctrl-C)
before running can-flasher, and vice versa; PCAN allows one handle per channel.
"""
import argparse
import logging
import sys
import threading
import time

logging.getLogger("can").setLevel(logging.ERROR)  # hush the pcan "uptime" notice
import can  # noqa: E402

# --- bare-harness CAN map (Core/Src/app/ltc_bare_task.cpp) ---
ID_STATUS = 0x7E0           # [0]loop [1]expected [2]pec_clean [3]spi_ok [4..7]IC0 CFG
# Per-IC cell-frame IDs, chain pos 0..3 = IC0..IC3. Each row: [A(1-3), B(4-6),
# C(7-9), D(10-12)]. Mirrors ltc_bare_task.cpp read_send_group().
IC_CELL_IDS = [
    [0x7E1, 0x7E4, 0x7E5, 0x7E6],   # IC0
    [0x7B1, 0x7B4, 0x7B5, 0x7B6],   # IC1
    [0x7C1, 0x7C4, 0x7C5, 0x7C6],   # IC2
    [0x7D1, 0x7D4, 0x7D5, 0x7D6],   # IC3
]
IC_LABELS = ["IC0 (bottom)", "IC1 (second)", "IC2 (third)", "IC3 (fourth)"]
ID_IC1_RAW = 0x7BF          # IC1 RDCFGA raw (6 data + 2 PEC) — all 0xFF = no return on first hop
# Per-IC temperature block: TEMP_BASE + ic*8 + frame(0..6); each frame
# [ctr, base_channel, 3x mV LE] = the muxed NTC-divider voltage per channel.
TEMP_BASE = 0x740
TEMPS_PER_IC = 20           # ADG731 channels swept per LTC
TEMP_FRAMES = (TEMPS_PER_IC + 2) // 3   # 7 frames of 3
FRESH_S = 2.0               # frames older than this are treated as stale


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}    # arbitration_id -> (bytes, monotonic_ts)
        self.count = 0
        self.last = 0.0
        self.err = None


def reader(bus, shared, stop):
    while not stop.is_set():
        try:
            m = bus.recv(timeout=0.2)
        except Exception as e:  # noqa: BLE001
            with shared.lock:
                shared.err = str(e)
            time.sleep(0.2)
            continue
        if m is None:
            continue
        now = time.monotonic()
        with shared.lock:
            shared.frames[m.arbitration_id] = (bytes(m.data), now)
            shared.count += 1
            shared.last = now
            shared.err = None


def decode(frames, now):
    def fresh(cid):
        e = frames.get(cid)
        return e if (e and now - e[1] < FRESH_S) else None

    snap = {"have_status": False}
    st = fresh(ID_STATUS)
    if st and len(st[0]) >= 8:
        d = st[0]
        snap.update(have_status=True, loop=d[0], expected=d[1],
                    pec_clean=d[2], spi_ok=d[3], ic0_cfg=d[4:8])

    def cells(ids):
        ok, vals = None, []
        for cid in ids:
            e = fresh(cid)
            if e and len(e[0]) >= 8:
                d = e[0]
                if ok is None:
                    ok = d[1]
                vals += [int.from_bytes(d[2 + i * 2:4 + i * 2], "little") for i in range(3)]
            else:
                vals += [None, None, None]
        return ok, vals

    def temps(pos):
        tvals = [None] * TEMPS_PER_IC
        for f in range(TEMP_FRAMES):
            e = fresh(TEMP_BASE + pos * 8 + f)
            if e and len(e[0]) >= 8:
                d = e[0]
                base = d[1]
                for j in range(3):
                    idx = base + j
                    if idx < TEMPS_PER_IC:
                        tvals[idx] = int.from_bytes(d[2 + j * 2:4 + j * 2], "little")
        return tvals

    ics = []
    for pos, ids in enumerate(IC_CELL_IDS):
        ok, vals = cells(ids)
        ics.append({"ok": ok, "cells": vals, "noreturn": False, "temps": temps(pos)})
    raw = fresh(ID_IC1_RAW)
    ics[1]["noreturn"] = bool(raw and all(b == 0xFF for b in raw[0]))
    snap["ics"] = ics
    return snap


def _stats(cells):
    real = [v for v in cells if v not in (None, 0)]
    if not real:
        return None
    return min(real), max(real), max(real) - min(real), len(real)


# ----------------------------- curses UI -----------------------------
def draw(scr, snap, count, stale, err):
    import curses
    scr.erase()
    h, w = scr.getmaxyx()
    OK, BAD, WARN, HDR = (curses.color_pair(i) for i in (1, 2, 3, 5))
    DIM = curses.A_DIM

    def put(y, x, s, attr=0):
        if 0 <= y < h and 0 <= x < w:
            scr.addnstr(y, x, s, max(0, w - x - 1), attr)

    put(0, 2, "IFS08 AMS — BMS isoSPI Live Monitor", HDR | curses.A_BOLD)
    put(0, max(38, w - 16), f"frames {count}", DIM)
    if stale:
        put(1, 2, "  NO RECENT FRAMES — bus quiet / board rebooting", BAD | curses.A_BOLD)

    y = 2
    if snap["have_status"]:
        pec, exp = snap["pec_clean"], snap["expected"]
        col = OK if pec == exp else (WARN if pec > 0 else BAD)
        put(y, 2, f"PEC-clean: {pec} / {exp}", col | curses.A_BOLD)
        put(y, 24, f"spi_ok: {'YES' if snap['spi_ok'] else 'NO'}",
            OK if snap["spi_ok"] else BAD)
        put(y, 40, f"loop: {snap['loop']}", DIM)
        put(y + 1, 2, "IC0 CFG: " + " ".join(f"{b:02X}" for b in snap["ic0_cfg"]), DIM)
    else:
        put(y, 2, "PEC-clean: -- / --   (no status frame)", DIM)
    y += 3

    n_show = snap["expected"] if snap.get("have_status") and snap["expected"] else len(snap["ics"])
    n_show = min(n_show, len(snap["ics"]))
    for pos in range(n_show):
        ic = snap["ics"][pos]
        ok = ic["ok"]
        okstr = "YES" if ok else ("NO" if ok is not None else "--")
        put(y, 2, f"{IC_LABELS[pos]}   decode_ok: {okstr}", (OK if ok else BAD) | curses.A_BOLD)
        if ic["noreturn"]:
            put(y, 40, "NO RETURN (all 0xFF)", BAD | curses.A_BOLD)
        y += 1
        cells = ic["cells"]
        for i, v in enumerate(cells):
            row, col = y + i // 4, 2 + (i % 4) * 14
            txt = f"c{i + 1:<2} {v:>5}" if v is not None else f"c{i + 1:<2}   ---"
            put(row, col, txt, OK if (v not in (None, 0)) else DIM)
        y += 3
        s = _stats(cells)
        if s:
            put(y, 2, f"min {s[0]}   max {s[1]}   spread {s[2]} mV   ({s[3]} cells)", DIM)
        y += 1
        t = ic["temps"]
        nt = sum(1 for v in t if v not in (None, 0))
        put(y, 2, f"NTC divider mV (ch 1-20)  [{nt}/20]", WARN)
        y += 1
        for k, v in enumerate(t):
            row, col = y + k // 10, 2 + (k % 10) * 6
            put(row, col, (f"{v:>5}" if v is not None else "    ."),
                OK if (v not in (None, 0)) else DIM)
        y += 3

    put(h - 1, 2, "q to quit", DIM)
    if err:
        put(h - 1, 16, f"err: {err}", BAD)
    scr.refresh()


def curses_main(scr, shared, stop):
    import curses
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    for i, c in ((1, curses.COLOR_GREEN), (2, curses.COLOR_RED),
                 (3, curses.COLOR_YELLOW), (5, curses.COLOR_CYAN)):
        curses.init_pair(i, c, -1)
    scr.nodelay(True)
    while not stop.is_set():
        try:
            if scr.getch() in (ord("q"), ord("Q")):
                break
        except curses.error:
            pass
        with shared.lock:
            frames = dict(shared.frames)
            count, last, err = shared.count, shared.last, shared.err
        now = time.monotonic()
        draw(scr, decode(frames, now), count, (now - last > FRESH_S) if last else True, err)
        time.sleep(0.15)


# ----------------------------- plain UI ------------------------------
def render_plain(snap, count, stale, color):
    g, r, yel, dim, rst = (("\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m")
                           if color else ("", "", "", "", ""))
    L = []
    L.append(f"{dim}IFS08 AMS — BMS isoSPI Live Monitor   frames {count}{rst}")
    if stale:
        L.append(f"{r}NO RECENT FRAMES — bus quiet / board rebooting{rst}")
    if snap["have_status"]:
        pec, exp = snap["pec_clean"], snap["expected"]
        c = g if pec == exp else (yel if pec > 0 else r)
        L.append(f"{c}PEC-clean: {pec}/{exp}{rst}   "
                 f"spi_ok: {'YES' if snap['spi_ok'] else 'NO'}   loop: {snap['loop']}   "
                 f"IC0 CFG: {' '.join(f'{b:02X}' for b in snap['ic0_cfg'])}")
    else:
        L.append("PEC-clean: --/--  (no status frame)")
    n_show = snap["expected"] if snap.get("have_status") and snap["expected"] else len(snap["ics"])
    n_show = min(n_show, len(snap["ics"]))
    for pos in range(n_show):
        ic = snap["ics"][pos]
        ok = ic["ok"]
        tag = "YES" if ok else ("NO" if ok is not None else "--")
        extra = "  NO RETURN (all 0xFF)" if ic["noreturn"] else ""
        L.append(f"{(g if ok else r)}{IC_LABELS[pos]}  decode_ok:{tag}{rst}{extra}")
        cells = ic["cells"]
        row = "  " + "  ".join(f"c{i+1:>2}{'.' if v is None else ''}{v if v is not None else '---':>5}"
                               for i, v in enumerate(cells))
        L.append(row)
        s = _stats(cells)
        if s:
            L.append(f"  min {s[0]}  max {s[1]}  spread {s[2]} mV  ({s[3]} cells)")
        t = ic["temps"]
        nt = sum(1 for v in t if v not in (None, 0))
        L.append(f"  {yel}NTC divider mV (ch 1-20)  [{nt}/20 present]{rst}")
        for r0 in (0, 10):
            L.append("    " + " ".join(
                (f"{t[k]:>4}" if t[k] is not None else "   .")
                for k in range(r0, min(r0 + 10, TEMPS_PER_IC))))
    return "\n".join(L)


def plain_loop(shared, stop, once):
    color = sys.stdout.isatty()
    deadline = time.monotonic() + 5.0  # --once: wait up to this long for first data
    while not stop.is_set():
        time.sleep(0.5 if once else 1.0)
        with shared.lock:
            frames = dict(shared.frames)
            count, last = shared.count, shared.last
        now = time.monotonic()
        snap = decode(frames, now)
        if once and not snap["have_status"] and time.monotonic() < deadline:
            continue  # still warming up — keep waiting for the first status frame
        text = render_plain(snap, count, (now - last > FRESH_S) if last else True, color)
        if color and not once:
            sys.stdout.write("\033[2J\033[H")  # clear + home
        sys.stdout.write(text + "\n")
        sys.stdout.flush()
        if once:
            break


# ------------------------------- main --------------------------------
def main():
    ap = argparse.ArgumentParser(description="Live BMS isoSPI monitor (bare-LTC harness)")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--plain", action="store_true", help="plain text instead of curses")
    ap.add_argument("--once", action="store_true", help="one snapshot then exit (implies --plain)")
    a = ap.parse_args()

    try:
        bus = can.Bus(interface="pcan", channel=a.channel, bitrate=a.bitrate)
    except Exception as e:  # noqa: BLE001
        print(f"Failed to open PCAN ({a.channel} @ {a.bitrate}): {e}")
        print("Is another process holding the bus (can-flasher)? Only one handle per channel.")
        return 1

    shared, stop = Shared(), threading.Event()
    t = threading.Thread(target=reader, args=(bus, shared, stop), daemon=True)
    t.start()
    try:
        if a.plain or a.once:
            plain_loop(shared, stop, a.once)
        else:
            import curses
            curses.wrapper(curses_main, shared, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(timeout=1.0)
        try:
            bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
