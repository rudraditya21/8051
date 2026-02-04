VERILATOR ?= verilator

RTL = rtl/mcs51_pkg.sv rtl/mcs51_core.sv rtl/mcs51_mcu.sv
TB  = tb/tb_mcs51.sv

all: sim

.PHONY: all sim clean

sim:
	$(VERILATOR) -Wall -sv --timing --binary $(RTL) $(TB) --top-module tb_mcs51
	./obj_dir/Vtb_mcs51

clean:
	rm -rf obj_dir
