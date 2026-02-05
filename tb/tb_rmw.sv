`timescale 1ns/1ps
`default_nettype none

module tb_rmw;
  logic clk;
  logic reset_n;
  logic [7:0] p0_in, p1_in, p2_in, p3_in;
  logic [7:0] p0_out, p1_out, p2_out, p3_out;
  logic [7:0] p0_oe, p1_oe, p2_oe, p3_oe;
  logic       unused_tb;

  mcs51_mcu #(
    .IRAM_SIZE(128),
    .CODE_SIZE(256),
    .XDATA_SIZE(65536),
    .CODE_INIT_FILE("tb/rom_rmw.hex")
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
    p1_in = 8'h55; // distinguish pin vs latch for RMW
    p2_in = 8'hFF;
    p3_in = 8'hFF;
    #50;
    reset_n = 1'b1;

    repeat (200) @(posedge clk);

    if (p1_out !== 8'h8A) $fatal(1, "RMW latch expected 8A, got %02x", p1_out);

    $display("PASS: port RMW uses latch");
    $finish;
  end

endmodule

`default_nettype wire
