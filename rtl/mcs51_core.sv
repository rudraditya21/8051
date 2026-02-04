`timescale 1ns/1ps
`default_nettype none

/* verilator lint_off IMPORTSTAR */
import mcs51_pkg::*;
/* verilator lint_on IMPORTSTAR */

module mcs51_core #(
  parameter int IRAM_SIZE = 128
) (
  input  logic        clk,
  input  logic        reset_n,

  // Code memory interface (combinational read)
  output logic [15:0] code_addr,
  input  logic [7:0]  code_rdata,

  // XDATA interface (combinational read)
  output logic [15:0] xdata_addr,
  output logic [7:0]  xdata_wdata,
  output logic        xdata_we,
  output logic        xdata_re,
  input  logic [7:0]  xdata_rdata,

  // External SFR interface
  output logic [7:0]  sfr_raddr,
  input  logic [7:0]  sfr_rdata,
  output logic [7:0]  sfr_waddr,
  output logic [7:0]  sfr_wdata,
  output logic        sfr_we,

  // Interrupt request interface
  input  logic        int_req,
  input  logic [15:0] int_vector,
  input  logic        int_prio, // 1=high, 0=low
  output logic        int_ack,
  output logic        reti_pulse,
  output logic [7:0]  pcon_out
);

  typedef enum logic [1:0] {S_FETCH, S_FETCH1, S_FETCH2, S_EXEC} state_t;
  state_t state;

  logic [15:0] pc;
  logic [7:0] opcode, op1, op2;
  logic [1:0] instr_len;

  // Internal SFRs
  logic [7:0] acc, b, psw, sp, dpl, dph, pcon;
  logic [7:0] sp_next;

  // Internal RAM
  logic [7:0] iram [0:IRAM_SIZE-1];

  localparam int IRAM_ADDR_W = (IRAM_SIZE <= 1) ? 1 : $clog2(IRAM_SIZE);
  localparam logic [7:0] IRAM_SIZE_U8 = IRAM_SIZE[7:0];

  // Interrupt level stack (0=none,1=low,2=high)
  logic [1:0] int_level;
  logic [1:0] int_stack [0:3];
  logic [1:0] int_sp;

  // Combinational: instruction length
  function automatic logic [1:0] instr_length(input logic [7:0] op);
    begin
      case (op)
        8'h02, 8'h12, 8'h85, 8'h90, 8'hD5: instr_length = 2'd3;
        8'h10, 8'h20, 8'h30, 8'h43, 8'h53, 8'h63,
        8'h75, 8'hB4, 8'hB5, 8'hB6, 8'hB7,
        8'hB8, 8'hB9, 8'hBA, 8'hBB, 8'hBC, 8'hBD, 8'hBE, 8'hBF: instr_length = 2'd3;
        8'h01, 8'h11, 8'h21, 8'h31, 8'h41, 8'h51, 8'h61, 8'h71,
        8'h81, 8'h91, 8'hA1, 8'hB1, 8'hC1, 8'hD1, 8'hE1, 8'hF1: instr_length = 2'd2;
        8'h05, 8'h15, 8'h25, 8'h35, 8'h40, 8'h42, 8'h44, 8'h45, 8'h50,
        8'h52, 8'h54, 8'h55, 8'h60, 8'h62, 8'h64, 8'h65, 8'h70, 8'h72,
        8'h74, 8'h76, 8'h77, 8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F,
        8'h80, 8'h82, 8'h86, 8'h87, 8'h88, 8'h89, 8'h8A, 8'h8B, 8'h8C, 8'h8D, 8'h8E, 8'h8F,
        8'h92, 8'h94, 8'h95, 8'hA0, 8'hA2, 8'hA6, 8'hA7, 8'hA8, 8'hA9, 8'hAA, 8'hAB, 8'hAC, 8'hAD, 8'hAE, 8'hAF,
        8'hB0, 8'hB2, 8'hC0, 8'hC2, 8'hC5, 8'hD0, 8'hD2, 8'hD8, 8'hD9, 8'hDA, 8'hDB, 8'hDC, 8'hDD, 8'hDE, 8'hDF, 8'hE5, 8'hF5: instr_length = 2'd2;
        default: instr_length = 2'd1;
      endcase
    end
  endfunction

  // Helpers
  function automatic logic [7:0] iram_read(input logic [7:0] addr);
    if (addr < IRAM_SIZE_U8) iram_read = iram[addr[IRAM_ADDR_W-1:0]];
    else iram_read = 8'h00;
  endfunction

  task automatic iram_write(input logic [7:0] addr, input logic [7:0] val);
    if (addr < IRAM_SIZE_U8) iram[addr[IRAM_ADDR_W-1:0]] <= val;
  endtask

  function automatic logic [7:0] reg_bank_sel();
    reg_bank_sel = {3'b000, psw[PSW_RS1], psw[PSW_RS0], 3'b000};
  endfunction

  function automatic logic [7:0] get_rn(input logic [2:0] n);
    get_rn = iram_read(reg_bank_sel() + {5'd0, n});
  endfunction

  task automatic set_rn(input logic [2:0] n, input logic [7:0] val);
    iram_write(reg_bank_sel() + {5'd0, n}, val);
  endtask

  function automatic logic [15:0] get_dptr();
    get_dptr = {dph, dpl};
  endfunction

  task automatic set_dptr(input logic [15:0] val);
    begin
      dpl <= val[7:0];
      dph <= val[15:8];
    end
  endtask

  task automatic set_parity(input logic [7:0] acc_val);
    psw[PSW_P] <= parity_odd(acc_val);
  endtask

  function automatic logic [15:0] rel8(input logic [7:0] v);
    rel8 = {{8{v[7]}}, v};
  endfunction

  function automatic logic [7:0] read_direct(input logic [7:0] addr);
    begin
      if (addr < 8'h80) begin
        read_direct = iram_read(addr);
      end else begin
        case (addr)
          SFR_ACC:  read_direct = acc;
          SFR_B:    read_direct = b;
          SFR_PSW:  read_direct = psw;
          SFR_SP:   read_direct = sp;
          SFR_DPL:  read_direct = dpl;
          SFR_DPH:  read_direct = dph;
          SFR_PCON: read_direct = pcon;
          default:  read_direct = sfr_rdata;
        endcase
      end
    end
  endfunction

  task automatic write_direct(input logic [7:0] addr, input logic [7:0] val);
    begin
      if (addr < 8'h80) begin
        iram_write(addr, val);
      end else begin
        case (addr)
          SFR_ACC:  begin acc <= val; set_parity(val); end
          SFR_B:    b   <= val;
          SFR_PSW:  psw <= val;
          SFR_SP:   sp_next = val;
          SFR_DPL:  dpl <= val;
          SFR_DPH:  dph <= val;
          SFR_PCON: pcon<= val;
          default:  begin
            sfr_waddr <= addr;
            sfr_wdata <= val;
            sfr_we    <= 1'b1;
          end
        endcase
      end
    end
  endtask

  function automatic logic bit_read(input logic [7:0] bit_addr);
    logic [7:0] base;
    logic [2:0] bitn;
    begin
      if (bit_addr < 8'h80) begin
        base = 8'h20 + {bit_addr[7:3], 3'b000};
        bitn = bit_addr[2:0];
        bit_read = iram_read(base)[bitn];
      end else begin
        base = {bit_addr[7:3], 3'b000};
        bitn = bit_addr[2:0];
        case (base)
          SFR_ACC: bit_read = acc[bitn];
          SFR_B:   bit_read = b[bitn];
          SFR_PSW: bit_read = psw[bitn];
          default: bit_read = sfr_rdata[bitn];
        endcase
      end
    end
  endfunction

  task automatic bit_write(input logic [7:0] bit_addr, input logic val);
    logic [7:0] base;
    logic [2:0] bitn;
    logic [7:0] tmp;
    begin
      if (bit_addr < 8'h80) begin
        base = 8'h20 + {bit_addr[7:3], 3'b000};
        bitn = bit_addr[2:0];
        tmp = iram_read(base);
        tmp[bitn] = val;
        iram_write(base, tmp);
      end else begin
        base = {bit_addr[7:3], 3'b000};
        bitn = bit_addr[2:0];
        case (base)
          SFR_ACC: begin
            tmp = acc;
            tmp[bitn] = val;
            acc <= tmp;
            set_parity(tmp);
          end
          SFR_B:   b[bitn] <= val;
          SFR_PSW: psw[bitn] <= val;
          default: begin
            tmp = sfr_rdata;
            tmp[bitn] = val;
            sfr_waddr <= base;
            sfr_wdata <= tmp;
            sfr_we    <= 1'b1;
          end
        endcase
      end
    end
  endtask

  // Arithmetic helpers
  task automatic add_to_acc(input logic [7:0] val, input logic carry_in);
    logic [8:0] sum;
    begin
      sum = {1'b0, acc} + {1'b0, val} + {8'd0, carry_in};
      acc <= sum[7:0];
      psw[PSW_CY] <= sum[8];
      psw[PSW_AC] <= ({1'b0, acc[3:0]} + {1'b0, val[3:0]} + {4'd0, carry_in}) >= 5'h10;
      psw[PSW_OV] <= (acc[7] & val[7] & ~sum[7]) | (~acc[7] & ~val[7] & sum[7]);
      set_parity(sum[7:0]);
    end
  endtask

  task automatic sub_from_acc(input logic [7:0] val, input logic carry_in);
    logic [8:0] diff;
    begin
      diff = {1'b0, acc} - {1'b0, val} - {8'd0, carry_in};
      acc <= diff[7:0];
      psw[PSW_CY] <= diff[8];
      psw[PSW_AC] <= ({1'b0, acc[3:0]}) < ({1'b0, val[3:0]} + {4'd0, carry_in});
      psw[PSW_OV] <= (acc[7] ^ val[7]) & (acc[7] ^ diff[7]);
      set_parity(diff[7:0]);
    end
  endtask

  // SFR read address for external SFRs (combinational)
  always_comb begin
    sfr_raddr = 8'h00;
    unique case (opcode)
      // Bit-addressable SFR reads (bit ops)
      8'h10, 8'h20, 8'h30, 8'h72, 8'h82, 8'hA0, 8'hB0, 8'hA2, 8'h92, 8'hB2, 8'hC2, 8'hD2:
        sfr_raddr = {op1[7:3], 3'b000};
      // Direct read source in op1
      8'h05, 8'h15, 8'h25, 8'h35, 8'h45, 8'h55, 8'h65,
      8'h42, 8'h43, 8'h52, 8'h53, 8'h62, 8'h63,
      8'h95, 8'hB5, 8'hC0, 8'hC5, 8'hD5, 8'hE5,
      8'hA6, 8'hA7, 8'hA8, 8'hA9, 8'hAA, 8'hAB, 8'hAC, 8'hAD, 8'hAE, 8'hAF:
        sfr_raddr = op1;
      // Direct read source in op2 (MOV direct,direct)
      8'h85: sfr_raddr = op2;
      default: sfr_raddr = 8'h00;
    endcase
  end

  // Code address multiplexer for MOVC
  always_comb begin
    code_addr = pc;
    if (state == S_EXEC) begin
      if (opcode == 8'h83) begin
        code_addr = pc + {8'h00, acc}; // MOVC A,@A+PC (PC already points to next instr)
      end else if (opcode == 8'h93) begin
        code_addr = get_dptr() + {8'h00, acc}; // MOVC A,@A+DPTR
      end
    end
  end

  assign pcon_out = pcon;

  // XDATA address multiplexer for MOVX
  always_comb begin
    xdata_addr = 16'h0000;
    if (state == S_EXEC) begin
      unique case (opcode)
        8'hE0, 8'hF0: xdata_addr = get_dptr();
        8'hE2, 8'hE3, 8'hF2, 8'hF3: xdata_addr = {8'h00, get_rn({2'b00, opcode[0]})};
        default: xdata_addr = 16'h0000;
      endcase
    end
  end

  // XDATA control (combinational)
  always_comb begin
    xdata_we = 1'b0;
    xdata_re = 1'b0;
    xdata_wdata = acc;
    if (state == S_EXEC) begin
      if (opcode == 8'hF0 || opcode == 8'hF2 || opcode == 8'hF3) xdata_we = 1'b1;
      if (opcode == 8'hE0 || opcode == 8'hE2 || opcode == 8'hE3) xdata_re = 1'b1;
    end
  end

  // Defaults for outputs
  /* verilator lint_off BLKSEQ */
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state <= S_FETCH;
      pc <= 16'h0000;
      opcode <= 8'h00;
      op1 <= 8'h00;
      op2 <= 8'h00;
      instr_len <= 2'd1;

      acc <= 8'h00;
      b <= 8'h00;
      psw <= 8'h00;
      sp <= 8'h07;
      dpl <= 8'h00;
      dph <= 8'h00;
      pcon <= 8'h00;

      sfr_waddr <= 8'h00;
      sfr_wdata <= 8'h00;
      sfr_we <= 1'b0;

      int_ack <= 1'b0;
      reti_pulse <= 1'b0;
      int_level <= 2'd0;
      int_sp <= 2'd0;
    end else begin
      // Defaults each cycle
      sfr_we <= 1'b0;
      int_ack <= 1'b0;
      reti_pulse <= 1'b0;

      case (state)
        S_FETCH: begin
          opcode <= code_rdata;
          instr_len <= instr_length(code_rdata);
          pc <= pc + 16'd1;
          if (instr_length(code_rdata) == 2'd1) begin
            state <= S_EXEC;
          end else begin
            state <= S_FETCH1;
          end
        end
        S_FETCH1: begin
          op1 <= code_rdata;
          pc <= pc + 16'd1;
          if (instr_len == 2'd2) begin
            state <= S_EXEC;
          end else begin
            state <= S_FETCH2;
          end
        end
        S_FETCH2: begin
          op2 <= code_rdata;
          pc <= pc + 16'd1;
          state <= S_EXEC;
        end
        S_EXEC: begin
          logic [15:0] pc_next;

          pc_next = pc;
          sp_next = sp;

          // Execute instruction
          unique case (opcode)
            8'h00: begin end // NOP

            // AJMP/ACALL (11-bit)
            8'h01, 8'h21, 8'h41, 8'h61, 8'h81, 8'hA1, 8'hC1, 8'hE1: begin
              // AJMP
              pc_next = {pc_next[15:11], opcode[7:5], op1};
            end
            8'h11, 8'h31, 8'h51, 8'h71, 8'h91, 8'hB1, 8'hD1, 8'hF1: begin
              // ACALL
              sp_next = sp_next + 8'd1;
              iram_write(sp_next, pc_next[7:0]);
              sp_next = sp_next + 8'd1;
              iram_write(sp_next, pc_next[15:8]);
              pc_next = {pc_next[15:11], opcode[7:5], op1};
            end

            8'h02: begin // LJMP
              pc_next = {op1, op2};
            end

            8'h12: begin // LCALL
              sp_next = sp_next + 8'd1;
              iram_write(sp_next, pc_next[7:0]);
              sp_next = sp_next + 8'd1;
              iram_write(sp_next, pc_next[15:8]);
              pc_next = {op1, op2};
            end

            8'h22: begin // RET
              pc_next = {iram_read(sp_next), iram_read(sp_next - 8'd1)};
              sp_next = sp_next - 8'd2;
            end

            8'h32: begin // RETI
              pc_next = {iram_read(sp_next), iram_read(sp_next - 8'd1)};
              sp_next = sp_next - 8'd2;
              reti_pulse <= 1'b1;
              if (int_sp != 0) begin
                int_sp <= int_sp - 2'd1;
                int_level <= int_stack[int_sp - 2'd1];
              end else begin
                int_level <= 2'd0;
              end
            end

            8'h03: begin // RR A
              acc <= {acc[0], acc[7:1]};
              set_parity({acc[0], acc[7:1]});
            end
            8'h13: begin // RRC A
              acc <= {psw[PSW_CY], acc[7:1]};
              psw[PSW_CY] <= acc[0];
              set_parity({psw[PSW_CY], acc[7:1]});
            end
            8'h23: begin // RL A
              acc <= {acc[6:0], acc[7]};
              set_parity({acc[6:0], acc[7]});
            end
            8'h33: begin // RLC A
              acc <= {acc[6:0], psw[PSW_CY]};
              psw[PSW_CY] <= acc[7];
              set_parity({acc[6:0], psw[PSW_CY]});
            end

            8'h04: begin // INC A
              acc <= acc + 8'd1;
              set_parity(acc + 8'd1);
            end
            8'h05: begin // INC direct
              write_direct(op1, read_direct(op1) + 8'd1);
            end
            8'h06, 8'h07: begin // INC @Ri
              logic [7:0] addr;
              addr = get_rn({2'b00, opcode[0]});
              iram_write(addr, iram_read(addr) + 8'd1);
            end
            8'h08, 8'h09, 8'h0A, 8'h0B, 8'h0C, 8'h0D, 8'h0E, 8'h0F: begin // INC Rn
              set_rn(opcode[2:0], get_rn(opcode[2:0]) + 8'd1);
            end

            8'h14: begin // DEC A
              acc <= acc - 8'd1;
              set_parity(acc - 8'd1);
            end
            8'h15: begin // DEC direct
              write_direct(op1, read_direct(op1) - 8'd1);
            end
            8'h16, 8'h17: begin // DEC @Ri
              logic [7:0] addr;
              addr = get_rn({2'b00, opcode[0]});
              iram_write(addr, iram_read(addr) - 8'd1);
            end
            8'h18, 8'h19, 8'h1A, 8'h1B, 8'h1C, 8'h1D, 8'h1E, 8'h1F: begin // DEC Rn
              set_rn(opcode[2:0], get_rn(opcode[2:0]) - 8'd1);
            end

            8'h24: add_to_acc(op1, 1'b0); // ADD A,#imm
            8'h25: add_to_acc(read_direct(op1), 1'b0); // ADD A,direct
            8'h26, 8'h27: begin
              add_to_acc(iram_read(get_rn({2'b00, opcode[0]})), 1'b0); // ADD A,@Ri
            end
            8'h28, 8'h29, 8'h2A, 8'h2B, 8'h2C, 8'h2D, 8'h2E, 8'h2F: begin
              add_to_acc(get_rn(opcode[2:0]), 1'b0); // ADD A,Rn
            end

            8'h34: add_to_acc(op1, psw[PSW_CY]); // ADDC A,#imm
            8'h35: add_to_acc(read_direct(op1), psw[PSW_CY]); // ADDC A,direct
            8'h36, 8'h37: begin
              add_to_acc(iram_read(get_rn({2'b00, opcode[0]})), psw[PSW_CY]); // ADDC A,@Ri
            end
            8'h38, 8'h39, 8'h3A, 8'h3B, 8'h3C, 8'h3D, 8'h3E, 8'h3F: begin
              add_to_acc(get_rn(opcode[2:0]), psw[PSW_CY]); // ADDC A,Rn
            end

            8'h94: sub_from_acc(op1, psw[PSW_CY]); // SUBB A,#imm
            8'h95: sub_from_acc(read_direct(op1), psw[PSW_CY]); // SUBB A,direct
            8'h96, 8'h97: begin
              sub_from_acc(iram_read(get_rn({2'b00, opcode[0]})), psw[PSW_CY]); // SUBB A,@Ri
            end
            8'h98, 8'h99, 8'h9A, 8'h9B, 8'h9C, 8'h9D, 8'h9E, 8'h9F: begin
              sub_from_acc(get_rn(opcode[2:0]), psw[PSW_CY]); // SUBB A,Rn
            end

            8'h84: begin // DIV AB
              if (b == 8'h00) begin
                psw[PSW_OV] <= 1'b1;
                psw[PSW_CY] <= 1'b0;
                acc <= 8'h00;
                b <= 8'h00;
                set_parity(8'h00);
              end else begin
                logic [7:0] quo;
                logic [7:0] rem;
                quo = acc / b;
                rem = acc % b;
                acc <= quo;
                b <= rem;
                psw[PSW_OV] <= 1'b0;
                psw[PSW_CY] <= 1'b0;
                set_parity(quo);
              end
            end

            8'hA4: begin // MUL AB
              logic [15:0] prod;
              prod = acc * b;
              acc <= prod[7:0];
              b <= prod[15:8];
              psw[PSW_OV] <= (prod[15:8] != 8'h00);
              psw[PSW_CY] <= 1'b0;
              set_parity(prod[7:0]);
            end

            8'hD4: begin // DA A
              logic [8:0] tmp;
              tmp = {1'b0, acc};
              if ((acc[3:0] > 4'd9) || psw[PSW_AC]) tmp = tmp + 9'h006;
              if ((tmp[7:4] > 4'd9) || psw[PSW_CY]) tmp = tmp + 9'h060;
              acc <= tmp[7:0];
              psw[PSW_CY] <= tmp[8];
              set_parity(tmp[7:0]);
            end

            8'h44: begin // ORL A,#imm
              acc <= acc | op1;
              set_parity(acc | op1);
            end
            8'h45: begin // ORL A,direct
              acc <= acc | read_direct(op1);
              set_parity(acc | read_direct(op1));
            end
            8'h46, 8'h47: begin // ORL A,@Ri
              acc <= acc | iram_read(get_rn({2'b00, opcode[0]}));
              set_parity(acc | iram_read(get_rn({2'b00, opcode[0]})));
            end
            8'h48, 8'h49, 8'h4A, 8'h4B, 8'h4C, 8'h4D, 8'h4E, 8'h4F: begin // ORL A,Rn
              acc <= acc | get_rn(opcode[2:0]);
              set_parity(acc | get_rn(opcode[2:0]));
            end
            8'h42: begin // ORL direct,A
              write_direct(op1, read_direct(op1) | acc);
            end
            8'h43: begin // ORL direct,#imm
              write_direct(op1, read_direct(op1) | op2);
            end
            8'h72: begin // ORL C,bit
              psw[PSW_CY] <= psw[PSW_CY] | bit_read(op1);
            end
            8'hA0: begin // ORL C,/bit
              psw[PSW_CY] <= psw[PSW_CY] | ~bit_read(op1);
            end

            8'h54: begin // ANL A,#imm
              acc <= acc & op1;
              set_parity(acc & op1);
            end
            8'h55: begin // ANL A,direct
              acc <= acc & read_direct(op1);
              set_parity(acc & read_direct(op1));
            end
            8'h56, 8'h57: begin // ANL A,@Ri
              acc <= acc & iram_read(get_rn({2'b00, opcode[0]}));
              set_parity(acc & iram_read(get_rn({2'b00, opcode[0]})));
            end
            8'h58, 8'h59, 8'h5A, 8'h5B, 8'h5C, 8'h5D, 8'h5E, 8'h5F: begin // ANL A,Rn
              acc <= acc & get_rn(opcode[2:0]);
              set_parity(acc & get_rn(opcode[2:0]));
            end
            8'h52: begin // ANL direct,A
              write_direct(op1, read_direct(op1) & acc);
            end
            8'h53: begin // ANL direct,#imm
              write_direct(op1, read_direct(op1) & op2);
            end
            8'h82: begin // ANL C,bit
              psw[PSW_CY] <= psw[PSW_CY] & bit_read(op1);
            end
            8'hB0: begin // ANL C,/bit
              psw[PSW_CY] <= psw[PSW_CY] & ~bit_read(op1);
            end

            8'h64: begin // XRL A,#imm
              acc <= acc ^ op1;
              set_parity(acc ^ op1);
            end
            8'h65: begin // XRL A,direct
              acc <= acc ^ read_direct(op1);
              set_parity(acc ^ read_direct(op1));
            end
            8'h66, 8'h67: begin // XRL A,@Ri
              acc <= acc ^ iram_read(get_rn({2'b00, opcode[0]}));
              set_parity(acc ^ iram_read(get_rn({2'b00, opcode[0]})));
            end
            8'h68, 8'h69, 8'h6A, 8'h6B, 8'h6C, 8'h6D, 8'h6E, 8'h6F: begin // XRL A,Rn
              acc <= acc ^ get_rn(opcode[2:0]);
              set_parity(acc ^ get_rn(opcode[2:0]));
            end
            8'h62: begin // XRL direct,A
              write_direct(op1, read_direct(op1) ^ acc);
            end
            8'h63: begin // XRL direct,#imm
              write_direct(op1, read_direct(op1) ^ op2);
            end

            8'h74: begin // MOV A,#imm
              acc <= op1;
              set_parity(op1);
            end
            8'h75: begin // MOV direct,#imm
              write_direct(op1, op2);
            end
            8'h76, 8'h77: begin // MOV @Ri,#imm
              iram_write(get_rn({2'b00, opcode[0]}), op1);
            end
            8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F: begin // MOV Rn,#imm
              set_rn(opcode[2:0], op1);
            end
            8'hE5: begin // MOV A,direct
              acc <= read_direct(op1);
              set_parity(read_direct(op1));
            end
            8'hE6, 8'hE7: begin // MOV A,@Ri
              acc <= iram_read(get_rn({2'b00, opcode[0]}));
              set_parity(iram_read(get_rn({2'b00, opcode[0]})));
            end
            8'hE8, 8'hE9, 8'hEA, 8'hEB, 8'hEC, 8'hED, 8'hEE, 8'hEF: begin // MOV A,Rn
              acc <= get_rn(opcode[2:0]);
              set_parity(get_rn(opcode[2:0]));
            end
            8'hF5: begin // MOV direct,A
              write_direct(op1, acc);
            end
            8'hF6, 8'hF7: begin // MOV @Ri,A
              iram_write(get_rn({2'b00, opcode[0]}), acc);
            end
            8'hF8, 8'hF9, 8'hFA, 8'hFB, 8'hFC, 8'hFD, 8'hFE, 8'hFF: begin // MOV Rn,A
              set_rn(opcode[2:0], acc);
            end
            8'h85: begin // MOV direct,direct
              write_direct(op1, read_direct(op2));
            end
            8'h86, 8'h87: begin // MOV direct,@Ri
              write_direct(op1, iram_read(get_rn({2'b00, opcode[0]})));
            end
            8'h88, 8'h89, 8'h8A, 8'h8B, 8'h8C, 8'h8D, 8'h8E, 8'h8F: begin // MOV direct,Rn
              write_direct(op1, get_rn(opcode[2:0]));
            end
            8'hA6, 8'hA7: begin // MOV @Ri,direct
              iram_write(get_rn({2'b00, opcode[0]}), read_direct(op1));
            end
            8'hA8, 8'hA9, 8'hAA, 8'hAB, 8'hAC, 8'hAD, 8'hAE, 8'hAF: begin // MOV Rn,direct
              set_rn(opcode[2:0], read_direct(op1));
            end
            8'h90: begin // MOV DPTR,#imm16
              set_dptr({op1, op2});
            end

            8'h83: begin // MOVC A,@A+PC
              acc <= code_rdata;
              set_parity(code_rdata);
            end
            8'h93: begin // MOVC A,@A+DPTR
              acc <= code_rdata;
              set_parity(code_rdata);
            end

            8'hE0: begin // MOVX A,@DPTR
              acc <= xdata_rdata;
              set_parity(xdata_rdata);
            end
            8'hE2, 8'hE3: begin // MOVX A,@Ri
              acc <= xdata_rdata;
              set_parity(xdata_rdata);
            end
            8'hF0: begin // MOVX @DPTR,A
            end
            8'hF2, 8'hF3: begin // MOVX @Ri,A
            end

            8'hA3: begin // INC DPTR
              set_dptr(get_dptr() + 16'd1);
            end

            8'hC0: begin // PUSH direct
              sp_next = sp_next + 8'd1;
              iram_write(sp_next, read_direct(op1));
            end
            8'hD0: begin // POP direct
              write_direct(op1, iram_read(sp_next));
              sp_next = sp_next - 8'd1;
            end

            8'hC4: begin // SWAP A
              acc <= {acc[3:0], acc[7:4]};
              set_parity({acc[3:0], acc[7:4]});
            end
            8'hC5: begin // XCH A,direct
              logic [7:0] tmp;
              tmp = read_direct(op1);
              write_direct(op1, acc);
              acc <= tmp;
              set_parity(tmp);
            end
            8'hC6, 8'hC7: begin // XCH A,@Ri
              logic [7:0] tmp;
              tmp = iram_read(get_rn({2'b00, opcode[0]}));
              iram_write(get_rn({2'b00, opcode[0]}), acc);
              acc <= tmp;
              set_parity(tmp);
            end
            8'hC8, 8'hC9, 8'hCA, 8'hCB, 8'hCC, 8'hCD, 8'hCE, 8'hCF: begin // XCH A,Rn
              logic [7:0] tmp;
              tmp = get_rn(opcode[2:0]);
              set_rn(opcode[2:0], acc);
              acc <= tmp;
              set_parity(tmp);
            end
            8'hD6, 8'hD7: begin // XCHD A,@Ri
              logic [7:0] tmp;
              tmp = iram_read(get_rn({2'b00, opcode[0]}));
              iram_write(get_rn({2'b00, opcode[0]}), {tmp[7:4], acc[3:0]});
              acc <= {acc[7:4], tmp[3:0]};
              set_parity({acc[7:4], tmp[3:0]});
            end

            8'hE4: begin // CLR A
              acc <= 8'h00;
              set_parity(8'h00);
            end
            8'hF4: begin // CPL A
              acc <= ~acc;
              set_parity(~acc);
            end
            8'hC3: begin // CLR C
              psw[PSW_CY] <= 1'b0;
            end
            8'hD3: begin // SETB C
              psw[PSW_CY] <= 1'b1;
            end
            8'hB3: begin // CPL C
              psw[PSW_CY] <= ~psw[PSW_CY];
            end

            8'hC2: begin // CLR bit
              bit_write(op1, 1'b0);
            end
            8'hD2: begin // SETB bit
              bit_write(op1, 1'b1);
            end
            8'hB2: begin // CPL bit
              bit_write(op1, ~bit_read(op1));
            end
            8'hA2: begin // MOV C,bit
              psw[PSW_CY] <= bit_read(op1);
            end
            8'h92: begin // MOV bit,C
              bit_write(op1, psw[PSW_CY]);
            end

            8'h40: begin // JC rel
              if (psw[PSW_CY]) pc_next = pc_next + rel8(op1);
            end
            8'h50: begin // JNC rel
              if (!psw[PSW_CY]) pc_next = pc_next + rel8(op1);
            end
            8'h60: begin // JZ rel
              if (acc == 8'h00) pc_next = pc_next + rel8(op1);
            end
            8'h70: begin // JNZ rel
              if (acc != 8'h00) pc_next = pc_next + rel8(op1);
            end
            8'h80: begin // SJMP rel
              pc_next = pc_next + rel8(op1);
            end
            8'h10: begin // JBC bit,rel
              if (bit_read(op1)) begin
                bit_write(op1, 1'b0);
                pc_next = pc_next + rel8(op2);
              end
            end
            8'h20: begin // JB bit,rel
              if (bit_read(op1)) pc_next = pc_next + rel8(op2);
            end
            8'h30: begin // JNB bit,rel
              if (!bit_read(op1)) pc_next = pc_next + rel8(op2);
            end
            8'hD5: begin // DJNZ direct,rel
              logic [7:0] tmp;
              tmp = read_direct(op1) - 8'd1;
              write_direct(op1, tmp);
              if (tmp != 8'h00) pc_next = pc_next + rel8(op2);
            end
            8'hD8, 8'hD9, 8'hDA, 8'hDB, 8'hDC, 8'hDD, 8'hDE, 8'hDF: begin // DJNZ Rn,rel
              logic [7:0] tmp;
              tmp = get_rn(opcode[2:0]) - 8'd1;
              set_rn(opcode[2:0], tmp);
              if (tmp != 8'h00) pc_next = pc_next + rel8(op1);
            end

            8'hB4: begin // CJNE A,#imm,rel
              if (acc != op1) pc_next = pc_next + rel8(op2);
              psw[PSW_CY] <= (acc < op1);
            end
            8'hB5: begin // CJNE A,direct,rel
              logic [7:0] tmp;
              tmp = read_direct(op1);
              if (acc != tmp) pc_next = pc_next + rel8(op2);
              psw[PSW_CY] <= (acc < tmp);
            end
            8'hB6, 8'hB7: begin // CJNE @Ri,#imm,rel
              logic [7:0] tmp;
              tmp = iram_read(get_rn({2'b00, opcode[0]}));
              if (tmp != op1) pc_next = pc_next + rel8(op2);
              psw[PSW_CY] <= (tmp < op1);
            end
            8'hB8, 8'hB9, 8'hBA, 8'hBB, 8'hBC, 8'hBD, 8'hBE, 8'hBF: begin // CJNE Rn,#imm,rel
              logic [7:0] tmp;
              tmp = get_rn(opcode[2:0]);
              if (tmp != op1) pc_next = pc_next + rel8(op2);
              psw[PSW_CY] <= (tmp < op1);
            end

            8'h73: begin // JMP @A+DPTR
              pc_next = get_dptr() + {8'h00, acc};
            end

            default: begin
              // Unused/illegal opcodes treated as NOP
            end
          endcase

          // Interrupt handling at instruction boundary
          if (int_req && ((int_prio == 1'b1 && int_level < 2'd2) || (int_prio == 1'b0 && int_level == 2'd0))) begin
            // Push PC
            sp_next = sp_next + 8'd1;
            iram_write(sp_next, pc_next[7:0]);
            sp_next = sp_next + 8'd1;
            iram_write(sp_next, pc_next[15:8]);
            // Update interrupt level stack
            int_stack[int_sp] <= int_level;
            int_sp <= int_sp + 2'd1;
            int_level <= (int_prio ? 2'd2 : 2'd1);
            // Vector
            pc_next = int_vector;
            int_ack <= 1'b1;
          end

          pc <= pc_next;
          sp <= sp_next;
          state <= S_FETCH;
        end
        default: state <= S_FETCH;
      endcase
    end
  end
  /* verilator lint_on BLKSEQ */

endmodule

`default_nettype wire
