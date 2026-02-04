#!/usr/bin/env python3
import argparse
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def find_ucsim():
    for name in ("ucsim_51", "s51"):
        p = shutil.which(name)
        if p:
            return p
    return None


def run(cmd, **kwargs):
    print("+", " ".join(str(c) for c in cmd))
    return subprocess.run(cmd, check=True, **kwargs)


def gen_rom(out_hex, out_sym):
    script = ROOT / "tests/isa/gen_isa_basic.py"
    run(["python3", str(script), "--out", str(out_hex), "--sym", str(out_sym)])


def parse_dump_file(path):
    mem = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            addr = int(parts[0], 16)
            val = int(parts[1], 16)
            mem[addr] = val
    return mem


def parse_ucsim_output(text):
    mem = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("0x"):
            continue
        parts = line.split()
        try:
            addr = int(parts[0], 16)
        except ValueError:
            continue
        for tok in parts[1:]:
            if re.fullmatch(r"[0-9A-Fa-f]{2}", tok):
                mem[addr] = int(tok, 16)
                addr += 1
    return mem


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rom", default="")
    parser.add_argument("--sym", default="")
    parser.add_argument("--dump-begin", type=lambda x: int(x, 0), default=0)
    parser.add_argument("--dump-end", type=lambda x: int(x, 0), default=0x0FFF)
    parser.add_argument("--max-cycles", type=int, default=500000)
    args = parser.parse_args()

    build_dir = ROOT / "tests/isa/build"
    build_dir.mkdir(parents=True, exist_ok=True)

    rom = Path(args.rom) if args.rom else build_dir / "isa_basic.hex"
    sym = Path(args.sym) if args.sym else build_dir / "isa_basic.sym.json"

    if not rom.exists() or not sym.exists():
        gen_rom(rom, sym)

    ucsim = find_ucsim()
    if not ucsim:
        raise SystemExit("ucsim_51 or s51 not found in PATH")

    with open(sym, "r") as f:
        labels = json.load(f)
    if "done" not in labels:
        raise SystemExit("done label not found in sym file")
    done_addr = labels["done"]

    # Run ucsim and dump XRAM
    ucsim_cmds = "\n".join([
        f"break 0x{done_addr:04X}",
        "run",
        f"dump xram 0x{args.dump_begin:04X} 0x{args.dump_end:04X}",
        "quit",
        "",
    ])

    print(f"Running ucsim at breakpoint 0x{done_addr:04X}")
    proc = subprocess.run(
        [ucsim, "-t", "8051", str(rom)],
        input=ucsim_cmds,
        text=True,
        capture_output=True,
        check=True,
    )
    ucsim_mem = parse_ucsim_output(proc.stdout)

    # Run RTL via Verilator
    dump_rtl = build_dir / "isa_dump_rtl.hex"
    rtl_cmd = [
        "make",
        "isa",
        f"ROM={rom}",
        f"DUMP={dump_rtl}",
        f"MAX={args.max_cycles}",
        f"DUMP_BEGIN={args.dump_begin}",
        f"DUMP_END={args.dump_end}",
    ]
    run(rtl_cmd, cwd=str(ROOT))

    rtl_mem = parse_dump_file(dump_rtl)

    # Compare
    mismatches = []
    for addr in range(args.dump_begin, args.dump_end + 1):
        rv = rtl_mem.get(addr, 0)
        uv = ucsim_mem.get(addr, 0)
        if rv != uv:
            mismatches.append((addr, rv, uv))

    if mismatches:
        print(f"MISMATCHES: {len(mismatches)}")
        for addr, rv, uv in mismatches[:50]:
            print(f"0x{addr:04X}: rtl={rv:02X} ucsim={uv:02X}")
        raise SystemExit(1)

    print("ISA validation PASS")


if __name__ == "__main__":
    main()
