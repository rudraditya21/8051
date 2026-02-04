# 8051 (MCS-51) MCU in Verilog/SystemVerilog

A full MCS‑51/8051 microcontroller implementation in SystemVerilog, with CPU core, internal RAM/SFRs, timers, serial port, interrupt controller, and a Verilator testbench.

## Features
- Full MCS‑51 instruction set
- Internal data RAM (IRAM) with bit addressing
- SFRs with standard 8051 map
- Timers: T0/T1 modes 0/1/2/3
- Serial port modes 0/1/2/3
- External interrupt logic and priority
- XDATA interface and on‑chip XDATA RAM model
- Verilator testbench with ROM loader

## Directory layout
- `rtl/` RTL sources
- `tb/` Testbench and ROM images
- `sim/` (reserved) simulation assets

## Build and run (Verilator)
```sh
make sim
```

## Top level
The top‑level MCU wrapper is `rtl/mcs51_mcu.sv` and exposes port pins plus internal ROM/XDATA memories for simulation.

## Parameters
- `IRAM_SIZE` (default 128 bytes)
- `CODE_SIZE` (default 4096 bytes)
- `XDATA_SIZE` (default 65536 bytes)
- `CODE_INIT_FILE` (hex file for ROM initialization)

## Notes
- Clock is treated as the machine‑cycle clock.
- Port outputs are modeled as quasi‑bidirectional (`p*_oe` drives low when latch=0).

## Testbench
`tb/tb_mcs51.sv` loads `tb/rom_basic.hex` and checks XDATA results after execution.

## Status
Core, peripherals, and testbench are implemented and pass the basic ROM test.
