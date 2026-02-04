`timescale 1ns/1ps
`default_nettype none

module tb_isa;
  logic clk;
  logic reset_n;
  logic [7:0] p0_in, p1_in, p2_in, p3_in;
  logic [7:0] p0_out, p1_out, p2_out, p3_out;
  logic [7:0] p0_oe, p1_oe, p2_oe, p3_oe;
  logic       unused_tb;

  string rom_path;
  string dump_path;
  integer max_cycles;
  integer dump_begin;
  integer dump_end;

  mcs51_mcu #(
    .IRAM_SIZE(128),
    .CODE_SIZE(65536),
    .XDATA_SIZE(65536),
    .CODE_INIT_FILE("")
  ) dut (
    .clk(clk),
    .reset_n(reset_n),
    .p0_in(p0_in),
    .p1_in(p1_in),
    .p2_in(p2_in),
    .p3_in(p3_in),
    .p0_out(p0_out),
    .p1_out(p1_out),
    .p2_out(p2_out),
    .p3_out(p3_out),
    .p0_oe(p0_oe),
    .p1_oe(p1_oe),
    .p2_oe(p2_oe),
    .p3_oe(p3_oe)
  );

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    if (!$value$plusargs("rom=%s", rom_path)) rom_path = "tb/rom_basic.hex";
    if (!$value$plusargs("dump=%s", dump_path)) dump_path = "tb/isa_dump_rtl.hex";
    if (!$value$plusargs("max=%d", max_cycles)) max_cycles = 200000;
    if (!$value$plusargs("dump_begin=%d", dump_begin)) dump_begin = 0;
    if (!$value$plusargs("dump_end=%d", dump_end)) dump_end = 4095;

    reset_n = 1'b0;
    p0_in = 8'hFF;
    p1_in = 8'hFF;
    p2_in = 8'hFF;
    p3_in = 8'hFF;

    $readmemh(rom_path, dut.code_mem);

    #50;
    reset_n = 1'b1;

    integer i;
    run_loop: for (i = 0; i < max_cycles; i = i + 1) begin
      @(posedge clk);
      if (dut.xdata_mem[16'hFFFE] == 8'hA5) begin
        disable run_loop;
      end
    end

    // Dump XDATA region
    integer fd;
    fd = $fopen(dump_path, "w");
    if (fd == 0) $fatal(1, "Failed to open dump file %s", dump_path);
    for (i = dump_begin; i <= dump_end; i = i + 1) begin
      $fwrite(fd, "%04x %02x\n", i[15:0], dut.xdata_mem[i[15:0]]);
    end
    $fclose(fd);

    $display("ISA dump written to %s", dump_path);
    $finish;
  end

  assign unused_tb = ^{p0_out, p1_out, p2_out, p3_out, p0_oe, p1_oe, p2_oe, p3_oe};

endmodule

`default_nettype wire
