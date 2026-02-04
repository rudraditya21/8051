`timescale 1ns/1ps
`default_nettype none

/* verilator lint_off IMPORTSTAR */
import mcs51_pkg::*;
/* verilator lint_on IMPORTSTAR */

module tb_mcs51;
  logic clk;
  logic reset_n;
  logic [7:0] p0_in, p1_in, p2_in, p3_in;
  logic [7:0] p0_out, p1_out, p2_out, p3_out;
  logic [7:0] p0_oe, p1_oe, p2_oe, p3_oe;
  logic       unused_tb;

  mcs51_mcu #(
    .IRAM_SIZE(128),
    .CODE_SIZE(4096),
    .XDATA_SIZE(65536),
    .CODE_INIT_FILE("tb/rom_basic.hex")
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

  assign unused_tb = ^{p0_out, p1_out, p2_out, p3_out, p0_oe, p1_oe, p2_oe, p3_oe};

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    reset_n = 1'b0;
    p0_in = 8'hFF;
    p1_in = 8'hFF;
    p2_in = 8'hFF;
    p3_in = 8'hFF;
    #50;
    reset_n = 1'b1;

    repeat (2000) @(posedge clk);

    if (dut.xdata_mem[16'h0100] !== 8'h1A) $fatal(1, "XDATA[0100] expected 1A, got %02x", dut.xdata_mem[16'h0100]);
    if (dut.xdata_mem[16'h0101] !== 8'h10) $fatal(1, "XDATA[0101] expected 10, got %02x", dut.xdata_mem[16'h0101]);
    if (dut.xdata_mem[16'h0102] !== 8'h77) $fatal(1, "XDATA[0102] expected 77, got %02x", dut.xdata_mem[16'h0102]);

    $display("PASS: basic core/peripheral test");
    $finish;
  end

endmodule

`default_nettype wire
