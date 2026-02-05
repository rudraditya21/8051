// MCS-51 package: constants, SFR addresses, interrupt vectors
`timescale 1ns/1ps

package mcs51_pkg;
  // SFR addresses
  localparam logic [7:0] SFR_P0   = 8'h80;
  localparam logic [7:0] SFR_SP   = 8'h81;
  localparam logic [7:0] SFR_DPL  = 8'h82;
  localparam logic [7:0] SFR_DPH  = 8'h83;
  localparam logic [7:0] SFR_PCON = 8'h87;
  localparam logic [7:0] SFR_TCON = 8'h88;
  localparam logic [7:0] SFR_TMOD = 8'h89;
  localparam logic [7:0] SFR_TL0  = 8'h8A;
  localparam logic [7:0] SFR_TL1  = 8'h8B;
  localparam logic [7:0] SFR_TH0  = 8'h8C;
  localparam logic [7:0] SFR_TH1  = 8'h8D;
  localparam logic [7:0] SFR_P1   = 8'h90;
  localparam logic [7:0] SFR_SCON = 8'h98;
  localparam logic [7:0] SFR_SBUF = 8'h99;
  localparam logic [7:0] SFR_P2   = 8'hA0;
  localparam logic [7:0] SFR_IE   = 8'hA8;
  localparam logic [7:0] SFR_P3   = 8'hB0;
  localparam logic [7:0] SFR_IP   = 8'hB8;
  localparam logic [7:0] SFR_PSW  = 8'hD0;
  localparam logic [7:0] SFR_ACC  = 8'hE0;
  localparam logic [7:0] SFR_B    = 8'hF0;

  // Reset values (Intel MCS-51 Programmer's Guide and Instruction Set)
  localparam logic [7:0] RESET_ACC  = 8'h00;
  localparam logic [7:0] RESET_B    = 8'h00;
  localparam logic [7:0] RESET_PSW  = 8'h00;
  localparam logic [7:0] RESET_SP   = 8'h07;
  localparam logic [7:0] RESET_DPL  = 8'h00;
  localparam logic [7:0] RESET_DPH  = 8'h00;
  localparam logic [7:0] RESET_PCON = 8'h00;
  localparam logic [7:0] RESET_P0   = 8'hFF;
  localparam logic [7:0] RESET_P1   = 8'hFF;
  localparam logic [7:0] RESET_P2   = 8'hFF;
  localparam logic [7:0] RESET_P3   = 8'hFF;
  localparam logic [7:0] RESET_TCON = 8'h00;
  localparam logic [7:0] RESET_TMOD = 8'h00;
  localparam logic [7:0] RESET_TL0  = 8'h00;
  localparam logic [7:0] RESET_TH0  = 8'h00;
  localparam logic [7:0] RESET_TL1  = 8'h00;
  localparam logic [7:0] RESET_TH1  = 8'h00;
  localparam logic [7:0] RESET_SCON = 8'h00;
  localparam logic [7:0] RESET_IE   = 8'h00;
  localparam logic [7:0] RESET_IP   = 8'h00;

  // Interrupt vectors
  localparam logic [15:0] VEC_INT0  = 16'h0003;
  localparam logic [15:0] VEC_T0    = 16'h000B;
  localparam logic [15:0] VEC_INT1  = 16'h0013;
  localparam logic [15:0] VEC_T1    = 16'h001B;
  localparam logic [15:0] VEC_SERIAL= 16'h0023;

  // IE bits
  localparam int IE_EX0 = 0;
  localparam int IE_ET0 = 1;
  localparam int IE_EX1 = 2;
  localparam int IE_ET1 = 3;
  localparam int IE_ES  = 4;
  localparam int IE_EA  = 7;

  // IP bits
  localparam int IP_PX0 = 0;
  localparam int IP_PT0 = 1;
  localparam int IP_PX1 = 2;
  localparam int IP_PT1 = 3;
  localparam int IP_PS  = 4;

  // TCON bits
  localparam int TCON_IT0 = 0;
  localparam int TCON_IE0 = 1;
  localparam int TCON_IT1 = 2;
  localparam int TCON_IE1 = 3;
  localparam int TCON_TR0 = 4;
  localparam int TCON_TF0 = 5;
  localparam int TCON_TR1 = 6;
  localparam int TCON_TF1 = 7;

  // PSW bits
  localparam int PSW_P  = 0;
  localparam int PSW_OV = 2;
  localparam int PSW_RS0= 3;
  localparam int PSW_RS1= 4;
  localparam int PSW_F0 = 5;
  /* verilator lint_off UNUSEDPARAM */
  localparam int _UNUSED_PSW_F0 = PSW_F0;
  /* verilator lint_on UNUSEDPARAM */
  localparam int PSW_AC = 6;
  localparam int PSW_CY = 7;

  function automatic logic parity_odd(input logic [7:0] v);
    parity_odd = ^v;
  endfunction

endpackage
