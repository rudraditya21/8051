VERILATOR ?= verilator

RTL = rtl/mcs51_pkg.sv rtl/mcs51_core.sv rtl/mcs51_mcu.sv
TB  = tb/tb_mcs51.sv
ISA_TB = tb/tb_isa.sv

ROM ?= tb/rom_basic.hex
DUMP ?= tb/isa_dump_rtl.hex
MAX ?= 200000
DUMP_BEGIN ?= 0
DUMP_END ?= 4095

all: sim

.PHONY: all sim isa clean

sim:
	$(VERILATOR) -Wall -sv --timing --binary $(RTL) $(TB) --top-module tb_mcs51
	./obj_dir/Vtb_mcs51

isa:
	$(VERILATOR) -Wall -sv --timing --binary $(RTL) $(ISA_TB) --top-module tb_isa
	./obj_dir/Vtb_isa +rom=$(ROM) +dump=$(DUMP) +max=$(MAX) +dump_begin=$(DUMP_BEGIN) +dump_end=$(DUMP_END)

clean:
	rm -rf obj_dir
