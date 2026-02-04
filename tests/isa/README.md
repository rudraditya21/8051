# ISA validation (differential testing)

This harness runs the same ROM on two models:
- RTL simulation (`tb/tb_isa.sv` via Verilator)
- A golden software emulator (ucsim 8051)

It dumps XDATA from both and compares the results.

## Dependencies
- SDCC assembler: `sdas8051`
- ucsim for 8051: `ucsim_51` (or `s51` depending on install)
- Verilator

## Usage
```sh
python3 tests/isa/run_isa_validation.py
```

Options:
- `--rom` path to a prebuilt hex (skip assembly step)
- `--max-cycles` simulation timeout
- `--dump-begin` / `--dump-end` XDATA range to compare

The script generates a ROM under `tests/isa/build/` by default.
