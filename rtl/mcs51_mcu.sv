`timescale 1ns/1ps
`default_nettype none

/* verilator lint_off IMPORTSTAR */
import mcs51_pkg::*;
/* verilator lint_on IMPORTSTAR */

module mcs51_mcu #(
  parameter int IRAM_SIZE = 128,
  parameter int CODE_SIZE = 4096,
  parameter int XDATA_SIZE = 65536,
  parameter string CODE_INIT_FILE = ""
) (
  input  logic        clk,
  input  logic        reset_n,

  // Port pins
  input  logic [7:0]  p0_in,
  input  logic [7:0]  p1_in,
  input  logic [7:0]  p2_in,
  input  logic [7:0]  p3_in,
  output logic [7:0]  p0_out,
  output logic [7:0]  p1_out,
  output logic [7:0]  p2_out,
  output logic [7:0]  p3_out,
  output logic [7:0]  p0_oe,
  output logic [7:0]  p1_oe,
  output logic [7:0]  p2_oe,
  output logic [7:0]  p3_oe
);

  // Core connections
  logic [15:0] code_addr;
  logic [7:0]  code_rdata;

  logic [15:0] xdata_addr;
  logic [7:0]  xdata_wdata;
  logic        xdata_we;
  logic        xdata_re;
  logic [7:0]  xdata_rdata;

  logic [7:0]  sfr_raddr;
  logic [7:0]  sfr_rdata;
  logic [7:0]  sfr_waddr;
  logic [7:0]  sfr_wdata;
  logic        sfr_we;

  logic        int_req;
  logic [15:0] int_vector;
  logic        int_prio;
  logic        int_ack;
  logic        reti_pulse;
  logic [7:0]  pcon_out;

  // Program memory
  logic [7:0] code_mem [0:CODE_SIZE-1];

  // XDATA memory
  logic [7:0] xdata_mem [0:XDATA_SIZE-1];

  localparam int CODE_ADDR_W = (CODE_SIZE <= 1) ? 1 : $clog2(CODE_SIZE);
  localparam int XDATA_ADDR_W = (XDATA_SIZE <= 1) ? 1 : $clog2(XDATA_SIZE);
  localparam int CODE_SIZE_I = CODE_SIZE;
  localparam int XDATA_SIZE_I = XDATA_SIZE;

  // SFRs (external to core)
  logic [7:0] p0_latch, p1_latch, p2_latch, p3_latch;
  logic [7:0] tcon, tmod, tl0, tl1, th0, th1;
  logic [7:0] scon, sbuf_tx, sbuf_rx;
  logic [7:0] ie, ip;
  logic       unused_mcu;

  // Serial internal
  logic        tx_busy, rx_busy;
  logic [3:0]  tx_cnt, rx_cnt;
  logic [10:0] tx_shift;
  logic [9:0]  rx_shift;
  logic        txd;
  logic [7:0]  baud_div_cnt;

  // External interrupt edge detect
  logic int0_prev, int1_prev;
  logic t0_prev, t1_prev;

  // Interrupt source selection
  logic [2:0] int_src;

  // Code memory init
  initial begin
    if (CODE_INIT_FILE != "") begin
      $readmemh(CODE_INIT_FILE, code_mem);
    end
  end

  // Code memory read (combinational)
  always_comb begin
    if (int'(code_addr) < CODE_SIZE_I) code_rdata = code_mem[code_addr[CODE_ADDR_W-1:0]];
    else code_rdata = 8'hFF;
  end

  // XDATA memory read/write
  always_comb begin
    if (int'(xdata_addr) < XDATA_SIZE_I) xdata_rdata = xdata_mem[xdata_addr[XDATA_ADDR_W-1:0]];
    else xdata_rdata = 8'h00;
  end

  always_ff @(posedge clk) begin
    if (xdata_we) begin
      if (int'(xdata_addr) < XDATA_SIZE_I) xdata_mem[xdata_addr[XDATA_ADDR_W-1:0]] <= xdata_wdata;
    end
  end

  // Instantiate core
  mcs51_core #(
    .IRAM_SIZE(IRAM_SIZE)
  ) u_core (
    .clk(clk),
    .reset_n(reset_n),
    .code_addr(code_addr),
    .code_rdata(code_rdata),
    .xdata_addr(xdata_addr),
    .xdata_wdata(xdata_wdata),
    .xdata_we(xdata_we),
    .xdata_re(xdata_re),
    .xdata_rdata(xdata_rdata),
    .sfr_raddr(sfr_raddr),
    .sfr_rdata(sfr_rdata),
    .sfr_waddr(sfr_waddr),
    .sfr_wdata(sfr_wdata),
    .sfr_we(sfr_we),
    .int_req(int_req),
    .int_vector(int_vector),
    .int_prio(int_prio),
    .int_ack(int_ack),
    .reti_pulse(reti_pulse),
    .pcon_out(pcon_out)
  );

  // Port outputs (quasi-bidirectional)
  assign p0_out = p0_latch;
  assign p1_out = p1_latch;
  assign p2_out = p2_latch;
  assign p3_out = {p3_latch[7:2], txd, p3_latch[0]};

  assign p0_oe = ~p0_latch; // drive low when latch=0
  assign p1_oe = ~p1_latch;
  assign p2_oe = ~p2_latch;
  assign p3_oe = {~p3_latch[7:2], 1'b1, ~p3_latch[0]};

  assign unused_mcu = xdata_re ^ reti_pulse ^ ^pcon_out[6:0] ^ p3_latch[1] ^ ^sbuf_tx;

  // SFR read mux
  always_comb begin
    unique case (sfr_raddr)
      SFR_P0:   sfr_rdata = p0_in;
      SFR_P1:   sfr_rdata = p1_in;
      SFR_P2:   sfr_rdata = p2_in;
      SFR_P3:   sfr_rdata = p3_in;
      SFR_TCON: sfr_rdata = tcon;
      SFR_TMOD: sfr_rdata = tmod;
      SFR_TL0:  sfr_rdata = tl0;
      SFR_TL1:  sfr_rdata = tl1;
      SFR_TH0:  sfr_rdata = th0;
      SFR_TH1:  sfr_rdata = th1;
      SFR_SCON: sfr_rdata = scon;
      SFR_SBUF: sfr_rdata = sbuf_rx;
      SFR_IE:   sfr_rdata = ie;
      SFR_IP:   sfr_rdata = ip;
      default:  sfr_rdata = 8'hFF;
    endcase
  end

  // Interrupt selection (combinational)
  always_comb begin
    int_req = 1'b0;
    int_vector = 16'h0000;
    int_prio = 1'b0;
    int_src = 3'd0;

    if (ie[IE_EA]) begin
      // High priority first
      if (ip[IP_PX0] && ie[IE_EX0] && tcon[TCON_IE0]) begin
        int_req = 1'b1; int_vector = VEC_INT0; int_prio = 1'b1; int_src = 3'd0;
      end else if (ip[IP_PT0] && ie[IE_ET0] && tcon[TCON_TF0]) begin
        int_req = 1'b1; int_vector = VEC_T0; int_prio = 1'b1; int_src = 3'd1;
      end else if (ip[IP_PX1] && ie[IE_EX1] && tcon[TCON_IE1]) begin
        int_req = 1'b1; int_vector = VEC_INT1; int_prio = 1'b1; int_src = 3'd2;
      end else if (ip[IP_PT1] && ie[IE_ET1] && tcon[TCON_TF1]) begin
        int_req = 1'b1; int_vector = VEC_T1; int_prio = 1'b1; int_src = 3'd3;
      end else if (ip[IP_PS] && ie[IE_ES] && (scon[0] || scon[1])) begin
        int_req = 1'b1; int_vector = VEC_SERIAL; int_prio = 1'b1; int_src = 3'd4;
      end else begin
        // Low priority
        if (ie[IE_EX0] && tcon[TCON_IE0]) begin
          int_req = 1'b1; int_vector = VEC_INT0; int_prio = 1'b0; int_src = 3'd0;
        end else if (ie[IE_ET0] && tcon[TCON_TF0]) begin
          int_req = 1'b1; int_vector = VEC_T0; int_prio = 1'b0; int_src = 3'd1;
        end else if (ie[IE_EX1] && tcon[TCON_IE1]) begin
          int_req = 1'b1; int_vector = VEC_INT1; int_prio = 1'b0; int_src = 3'd2;
        end else if (ie[IE_ET1] && tcon[TCON_TF1]) begin
          int_req = 1'b1; int_vector = VEC_T1; int_prio = 1'b0; int_src = 3'd3;
        end else if (ie[IE_ES] && (scon[0] || scon[1])) begin
          int_req = 1'b1; int_vector = VEC_SERIAL; int_prio = 1'b0; int_src = 3'd4;
        end
      end
    end
  end

  // Peripheral/SFR update
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      p0_latch <= 8'hFF;
      p1_latch <= 8'hFF;
      p2_latch <= 8'hFF;
      p3_latch <= 8'hFF;

      tcon <= 8'h00;
      tmod <= 8'h00;
      tl0 <= 8'h00;
      th0 <= 8'h00;
      tl1 <= 8'h00;
      th1 <= 8'h00;

      scon <= 8'h00;
      sbuf_tx <= 8'h00;
      sbuf_rx <= 8'h00;

      ie <= 8'h00;
      ip <= 8'h00;

      tx_busy <= 1'b0;
      rx_busy <= 1'b0;
      tx_cnt <= 4'd0;
      rx_cnt <= 4'd0;
      tx_shift <= 11'd0;
      rx_shift <= 10'd0;
      txd <= 1'b1;
      baud_div_cnt <= 8'd0;

      int0_prev <= 1'b1;
      int1_prev <= 1'b1;
      t0_prev <= 1'b1;
      t1_prev <= 1'b1;
    end else begin
      // Local next-state variables
      logic [7:0] tl0_next, th0_next, tl1_next, th1_next;
      logic [7:0] tcon_next;
      logic [7:0] scon_next;
      logic [1:0] mode0, mode1;
      logic ct0, ct1, t0_gate, t1_gate;
      logic t0_fall, t1_fall;
      logic t0_tick, t1_tick;
      logic t1_overflow_pulse;
      logic baud_tick_now;
      logic [7:0] baud_div_next;
      logic [1:0] sm;

      tl0_next = tl0;
      th0_next = th0;
      tl1_next = tl1;
      th1_next = th1;
      tcon_next = tcon;
      scon_next = scon;
      t1_overflow_pulse = 1'b0;
      baud_tick_now = 1'b0;
      baud_div_next = baud_div_cnt;

      mode0 = tmod[1:0];
      mode1 = tmod[5:4];
      ct0 = tmod[2];
      ct1 = tmod[6];
      t0_gate = tmod[3];
      t1_gate = tmod[7];

      t0_fall = t0_prev & ~p3_in[4];
      t1_fall = t1_prev & ~p3_in[5];

      t0_tick = tcon[TCON_TR0] && (!t0_gate || p3_in[2]) && (ct0 ? t0_fall : 1'b1);
      t1_tick = tcon[TCON_TR1] && (!t1_gate || p3_in[3]) && (ct1 ? t1_fall : 1'b1);

      // Timer0 update
      if (mode0 != 2'b11) begin
        if (t0_tick) begin
          case (mode0)
            2'b00: begin // 13-bit
              logic [12:0] t0_13;
              t0_13 = {th0_next, tl0_next[4:0]} + 13'd1;
              if (t0_13 == 13'd0) tcon_next[TCON_TF0] = 1'b1;
              th0_next = t0_13[12:5];
              tl0_next = {tl0_next[7:5], t0_13[4:0]};
            end
            2'b01: begin // 16-bit
              logic [15:0] t0_16;
              t0_16 = {th0_next, tl0_next} + 16'd1;
              if (t0_16 == 16'd0) tcon_next[TCON_TF0] = 1'b1;
              {th0_next, tl0_next} = t0_16;
            end
            2'b10: begin // 8-bit auto-reload
              logic [7:0] tl;
              tl = tl0_next + 8'd1;
              if (tl == 8'd0) begin
                tl0_next = th0_next;
                tcon_next[TCON_TF0] = 1'b1;
              end else begin
                tl0_next = tl;
              end
            end
            default: begin end
          endcase
        end
      end else begin
        // Mode 3: split timer0
        if (t0_tick) begin
          tl0_next = tl0_next + 8'd1;
          if (tl0_next == 8'd0) tcon_next[TCON_TF0] = 1'b1;
        end
        if (tcon[TCON_TR1] && (!t1_gate || p3_in[3]) && (ct1 ? t1_fall : 1'b1)) begin
          th0_next = th0_next + 8'd1;
          if (th0_next == 8'd0) tcon_next[TCON_TF1] = 1'b1;
        end
      end

      // Timer1 update (if not halted by mode3)
      if (mode0 != 2'b11) begin
        if (t1_tick) begin
          case (mode1)
            2'b00: begin // 13-bit
              logic [12:0] t1_13;
              t1_13 = {th1_next, tl1_next[4:0]} + 13'd1;
              if (t1_13 == 13'd0) begin
                tcon_next[TCON_TF1] = 1'b1;
                t1_overflow_pulse = 1'b1;
              end
              th1_next = t1_13[12:5];
              tl1_next = {tl1_next[7:5], t1_13[4:0]};
            end
            2'b01: begin // 16-bit
              logic [15:0] t1_16;
              t1_16 = {th1_next, tl1_next} + 16'd1;
              if (t1_16 == 16'd0) begin
                tcon_next[TCON_TF1] = 1'b1;
                t1_overflow_pulse = 1'b1;
              end
              {th1_next, tl1_next} = t1_16;
            end
            2'b10: begin // 8-bit auto-reload
              logic [7:0] tl;
              tl = tl1_next + 8'd1;
              if (tl == 8'd0) begin
                tl1_next = th1_next;
                tcon_next[TCON_TF1] = 1'b1;
                t1_overflow_pulse = 1'b1;
              end else begin
                tl1_next = tl;
              end
            end
            default: begin end
          endcase
        end
      end

      // External interrupt latch update
      if (tcon_next[TCON_IT0]) begin
        if (int0_prev && !p3_in[2]) tcon_next[TCON_IE0] = 1'b1;
      end else begin
        tcon_next[TCON_IE0] = ~p3_in[2];
      end

      if (tcon_next[TCON_IT1]) begin
        if (int1_prev && !p3_in[3]) tcon_next[TCON_IE1] = 1'b1;
      end else begin
        tcon_next[TCON_IE1] = ~p3_in[3];
      end

      // CPU writes
      if (sfr_we) begin
        unique case (sfr_waddr)
          SFR_P0:   p0_latch <= sfr_wdata;
          SFR_P1:   p1_latch <= sfr_wdata;
          SFR_P2:   p2_latch <= sfr_wdata;
          SFR_P3:   p3_latch <= sfr_wdata;
          SFR_TCON: tcon_next <= sfr_wdata;
          SFR_TMOD: tmod <= sfr_wdata;
          SFR_TL0:  tl0_next <= sfr_wdata;
          SFR_TL1:  tl1_next <= sfr_wdata;
          SFR_TH0:  th0_next <= sfr_wdata;
          SFR_TH1:  th1_next <= sfr_wdata;
          SFR_SCON: scon_next <= sfr_wdata;
          SFR_SBUF: begin
            sbuf_tx <= sfr_wdata;
            scon_next[1] <= 1'b0; // TI cleared on write
          end
          SFR_IE:   ie <= sfr_wdata;
          SFR_IP:   ip <= sfr_wdata;
          default: begin end
        endcase
      end

      // Interrupt ack clears flags (except serial)
      if (int_ack) begin
        case (int_src)
          3'd0: if (tcon_next[TCON_IT0]) tcon_next[TCON_IE0] = 1'b0;
          3'd1: tcon_next[TCON_TF0] = 1'b0;
          3'd2: if (tcon_next[TCON_IT1]) tcon_next[TCON_IE1] = 1'b0;
          3'd3: tcon_next[TCON_TF1] = 1'b0;
          default: begin end
        endcase
      end

      // Update timer regs
      tcon <= tcon_next;
      tl0 <= tl0_next;
      th0 <= th0_next;
      tl1 <= tl1_next;
      th1 <= th1_next;

      // -------- Serial baud generator --------
      sm = scon_next[7:6];
      if ((sm == 2'b10) || (sm == 2'b11)) begin
        if (baud_div_cnt == 8'd0) begin
          baud_div_next = (pcon_out[7] ? 8'd15 : 8'd31);
          baud_tick_now = 1'b1;
        end else begin
          baud_div_next = baud_div_cnt - 8'd1;
        end
      end else if (sm == 2'b00) begin
        baud_tick_now = 1'b1;
        baud_div_next = 8'd0;
      end else begin
        baud_tick_now = t1_overflow_pulse;
        baud_div_next = 8'd0;
      end
      baud_div_cnt <= baud_div_next;

      // -------- Serial transmitter --------
      if (!tx_busy && sfr_we && (sfr_waddr == SFR_SBUF)) begin
        tx_busy <= 1'b1;
        case (sm)
          2'b00: begin
            tx_cnt <= 4'd8;
            tx_shift <= {3'b000, sfr_wdata};
          end
          2'b01: begin
            tx_cnt <= 4'd10;
            tx_shift <= {1'b1, 1'b1, sfr_wdata, 1'b0};
          end
          default: begin
            tx_cnt <= 4'd11;
            tx_shift <= {1'b1, scon_next[3], sfr_wdata, 1'b0};
          end
        endcase
        txd <= 1'b0;
      end else if (tx_busy && baud_tick_now) begin
        txd <= tx_shift[0];
        tx_shift <= {1'b1, tx_shift[10:1]};
        if (tx_cnt != 0) tx_cnt <= tx_cnt - 4'd1;
        if (tx_cnt == 4'd1) begin
          tx_busy <= 1'b0;
          scon_next[1] <= 1'b1; // TI
          txd <= 1'b1;
        end
      end

      // -------- Serial receiver --------
      if (!rx_busy && scon_next[4] && (sm != 2'b00) && (p3_in[0] == 1'b0)) begin
        rx_busy <= 1'b1;
        rx_cnt <= (sm == 2'b01) ? 4'd9 : 4'd10;
        rx_shift <= 10'd0;
      end else if (rx_busy && baud_tick_now) begin
        rx_shift <= {p3_in[0], rx_shift[9:1]};
        if (rx_cnt != 0) rx_cnt <= rx_cnt - 4'd1;
        if (rx_cnt == 4'd1) begin
          rx_busy <= 1'b0;
          sbuf_rx <= rx_shift[7:0];
          if (sm != 2'b01) scon_next[2] <= rx_shift[8];
          scon_next[0] <= 1'b1; // RI
        end
      end

      // Mode0 synchronous receive
      if (sm == 2'b00 && scon_next[4]) begin
        if (!rx_busy) begin
          rx_busy <= 1'b1;
          rx_cnt <= 4'd8;
          rx_shift <= 10'd0;
        end else if (baud_tick_now) begin
          rx_shift <= {p3_in[0], rx_shift[9:1]};
          if (rx_cnt != 0) rx_cnt <= rx_cnt - 4'd1;
          if (rx_cnt == 4'd1) begin
            rx_busy <= 1'b0;
            sbuf_rx <= rx_shift[7:0];
            scon_next[0] <= 1'b1;
          end
        end
      end

      scon <= scon_next;

      // External interrupt edge history
      int0_prev <= p3_in[2];
      int1_prev <= p3_in[3];
      t0_prev <= p3_in[4];
      t1_prev <= p3_in[5];
    end
  end

endmodule

`default_nettype wire
