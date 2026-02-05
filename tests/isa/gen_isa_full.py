#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


class Asm:
    def __init__(self, origin=0):
        self.pc = origin
        self.mem = {}
        self.labels = {}
        self.fixups = []  # (pos, label, kind)

    def label(self, name):
        self.labels[name] = self.pc

    def emit(self, *bytes_):
        for b in bytes_:
            self.mem[self.pc] = b & 0xFF
            self.pc += 1

    def emit_rel(self, label):
        pos = self.pc
        self.emit(0x00)
        self.fixups.append((pos, label, "rel8"))

    def emit_addr16(self, label):
        pos = self.pc
        self.emit(0x00, 0x00)
        self.fixups.append((pos, label, "abs16"))

    def resolve(self):
        for pos, label, kind in self.fixups:
            if label not in self.labels:
                raise KeyError(f"Unknown label {label}")
            addr = self.labels[label]
            if kind == "rel8":
                rel = addr - (pos + 1)
                if rel < -128 or rel > 127:
                    raise ValueError(f"rel8 out of range: {label} rel={rel}")
                self.mem[pos] = rel & 0xFF
            elif kind == "abs16":
                self.mem[pos] = (addr >> 8) & 0xFF
                self.mem[pos + 1] = addr & 0xFF
            elif kind == "ajmp":
                # Opcode already emitted; operand at pos is low 8 bits
                self.mem[pos] = addr & 0xFF
            elif kind == "acall":
                self.mem[pos] = addr & 0xFF
            elif kind == "ajmp_full":
                page_mask = 0xF800
                if (pos + 2) & page_mask != addr & page_mask:
                    raise ValueError(f"AJMP target not in same 2K page: {label}")
                opcode = 0x01 | (((addr >> 8) & 0x07) << 5)
                self.mem[pos] = opcode & 0xFF
                self.mem[pos + 1] = addr & 0xFF
            elif kind == "acall_full":
                page_mask = 0xF800
                if (pos + 2) & page_mask != addr & page_mask:
                    raise ValueError(f"ACALL target not in same 2K page: {label}")
                opcode = 0x11 | (((addr >> 8) & 0x07) << 5)
                self.mem[pos] = opcode & 0xFF
                self.mem[pos + 1] = addr & 0xFF
            else:
                raise ValueError(f"Unknown fixup kind {kind}")



# Opcodes helpers

def mov_a_imm(a, val):
    a.emit(0x74, val)


def mov_direct_imm(a, addr, val):
    a.emit(0x75, addr, val)


def mov_rn_imm(a, n, val):
    a.emit(0x78 + n, val)


def mov_dptr_imm(a, val16):
    a.emit(0x90, (val16 >> 8) & 0xFF, val16 & 0xFF)


def mov_dptr_label(a, label):
    a.emit(0x90)
    a.emit_addr16(label)


def setb_c(a):
    a.emit(0xD3)


def clr_c(a):
    a.emit(0xC3)


def reset_state(a):
    mov_direct_imm(a, 0x81, 0x30)  # SP
    mov_direct_imm(a, 0xD0, 0x00)  # PSW
    mov_a_imm(a, 0x12)
    mov_direct_imm(a, 0xF0, 0x34)  # B
    mov_direct_imm(a, 0x82, 0x00)  # DPL
    mov_direct_imm(a, 0x83, 0x02)  # DPH
    for i in range(8):
        mov_rn_imm(a, i, 0x40 + i)
    # IRAM 0x20..0x2F
    for i in range(0x20, 0x30):
        mov_direct_imm(a, i, i)
    mov_direct_imm(a, 0x20, 0x01)  # bit0 set for bit tests
    mov_direct_imm(a, 0x40, 0x22)
    mov_direct_imm(a, 0x41, 0x33)
    mov_direct_imm(a, 0x30, 0x5A)
    mov_direct_imm(a, 0x31, 0xA5)
    # Ports
    mov_direct_imm(a, 0x80, 0xF0)
    mov_direct_imm(a, 0x90, 0x0F)
    mov_direct_imm(a, 0xA0, 0xAA)
    mov_direct_imm(a, 0xB0, 0x55)
    # Timers/serial
    mov_direct_imm(a, 0x88, 0x00)  # TCON
    mov_direct_imm(a, 0x89, 0x00)  # TMOD
    mov_direct_imm(a, 0x8A, 0x12)  # TL0
    mov_direct_imm(a, 0x8C, 0x34)  # TH0
    mov_direct_imm(a, 0x8B, 0x56)  # TL1
    mov_direct_imm(a, 0x8D, 0x78)  # TH1
    mov_direct_imm(a, 0x98, 0x50)  # SCON
    mov_direct_imm(a, 0xA8, 0x00)  # IE
    mov_direct_imm(a, 0xB8, 0x00)  # IP
    mov_direct_imm(a, 0x87, 0x00)  # PCON


def dump_state(a, out_addr):
    count = 0

    def dump_a():
        nonlocal count
        a.emit(0xF0)  # MOVX @DPTR,A
        a.emit(0xA3)  # INC DPTR
        count += 1

    # Preserve DPTR in scratch IRAM (0x7E/0x7F)
    a.emit(0xE5, 0x82)  # MOV A,DPL
    a.emit(0xF5, 0x7E)  # MOV 7Eh,A
    a.emit(0xE5, 0x83)  # MOV A,DPH
    a.emit(0xF5, 0x7F)  # MOV 7Fh,A

    mov_dptr_imm(a, out_addr)

    # ACC
    dump_a()

    # B
    a.emit(0xE5, 0xF0)
    dump_a()

    # PSW, SP, DPL, DPH
    for addr in (0xD0, 0x81, 0x7E, 0x7F):
        a.emit(0xE5, addr)
        dump_a()

    # PCON
    a.emit(0xE5, 0x87)
    dump_a()

    # P0..P3
    for addr in (0x80, 0x90, 0xA0, 0xB0):
        a.emit(0xE5, addr)
        dump_a()

    # TCON, TMOD, TL0, TH0, TL1, TH1
    for addr in (0x88, 0x89, 0x8A, 0x8C, 0x8B, 0x8D):
        a.emit(0xE5, addr)
        dump_a()

    # SCON, SBUF, IE, IP
    for addr in (0x98, 0x99, 0xA8, 0xB8):
        a.emit(0xE5, addr)
        dump_a()

    # R0..R7
    for i in range(8):
        a.emit(0xE8 + i)
        dump_a()

    # IRAM 0x20..0x2F
    for addr in range(0x20, 0x30):
        a.emit(0xE5, addr)
        dump_a()

    return count


def add_case(cases, name, body_fn):
    cases.append((name, body_fn))


def build_program():
    a = Asm(origin=0x0000)
    # Reset vector
    a.emit(0x02)  # LJMP
    a.emit_addr16("main")

    # Main tests
    a.label("main")

    cases = []

    # NOP
    def case_nop(a, idx):
        reset_state(a)
        a.emit(0x00)
    add_case(cases, "NOP", case_nop)

    # Rotates
    def case_rr(a, idx):
        reset_state(a)
        a.emit(0x03)
    add_case(cases, "RR A", case_rr)

    def case_rl(a, idx):
        reset_state(a)
        a.emit(0x23)
    add_case(cases, "RL A", case_rl)

    def case_rrc(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x13)
    add_case(cases, "RRC A", case_rrc)

    def case_rlc(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x33)
    add_case(cases, "RLC A", case_rlc)

    # INC/DEC
    def case_inc_a(a, idx):
        reset_state(a)
        a.emit(0x04)
    add_case(cases, "INC A", case_inc_a)

    def case_inc_direct(a, idx):
        reset_state(a)
        a.emit(0x05, 0x30)
    add_case(cases, "INC direct", case_inc_direct)

    def case_inc_ind(a, idx):
        reset_state(a)
        a.emit(0x06)
    add_case(cases, "INC @R0", case_inc_ind)

    def case_inc_rn(a, idx):
        reset_state(a)
        a.emit(0x08)
    add_case(cases, "INC R0", case_inc_rn)

    def case_inc_dptr(a, idx):
        reset_state(a)
        a.emit(0xA3)
    add_case(cases, "INC DPTR", case_inc_dptr)

    def case_dec_a(a, idx):
        reset_state(a)
        a.emit(0x14)
    add_case(cases, "DEC A", case_dec_a)

    def case_dec_direct(a, idx):
        reset_state(a)
        a.emit(0x15, 0x30)
    add_case(cases, "DEC direct", case_dec_direct)

    def case_dec_ind(a, idx):
        reset_state(a)
        a.emit(0x16)
    add_case(cases, "DEC @R0", case_dec_ind)

    def case_dec_rn(a, idx):
        reset_state(a)
        a.emit(0x18)
    add_case(cases, "DEC R0", case_dec_rn)

    # ADD/ADDC/SUBB flag vectors
    def case_add_ac(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x0F)
        a.emit(0x24, 0x01)
    add_case(cases, "ADD A,#imm (AC)", case_add_ac)

    def case_add_ov(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x7F)
        a.emit(0x24, 0x01)
    add_case(cases, "ADD A,#imm (OV)", case_add_ov)

    def case_add_cy(a, idx):
        reset_state(a)
        mov_a_imm(a, 0xFF)
        a.emit(0x24, 0x01)
    add_case(cases, "ADD A,#imm (CY)", case_add_cy)

    def case_add_direct(a, idx):
        reset_state(a)
        a.emit(0x25, 0x30)
    add_case(cases, "ADD A,direct", case_add_direct)

    def case_add_ind(a, idx):
        reset_state(a)
        a.emit(0x26)
    add_case(cases, "ADD A,@R0", case_add_ind)

    def case_add_rn(a, idx):
        reset_state(a)
        a.emit(0x28)
    add_case(cases, "ADD A,R0", case_add_rn)

    def case_addc(a, idx):
        reset_state(a)
        setb_c(a)
        mov_a_imm(a, 0x10)
        a.emit(0x34, 0x01)
    add_case(cases, "ADDC A,#imm", case_addc)

    def case_addc_direct(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x35, 0x30)
    add_case(cases, "ADDC A,direct", case_addc_direct)

    def case_addc_ind(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x36)
    add_case(cases, "ADDC A,@R0", case_addc_ind)

    def case_addc_rn(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x38)
    add_case(cases, "ADDC A,R0", case_addc_rn)

    def case_subb(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x00)
        a.emit(0x94, 0x01)
    add_case(cases, "SUBB A,#imm", case_subb)

    def case_subb_direct(a, idx):
        reset_state(a)
        a.emit(0x95, 0x30)
    add_case(cases, "SUBB A,direct", case_subb_direct)

    def case_subb_ind(a, idx):
        reset_state(a)
        a.emit(0x96)
    add_case(cases, "SUBB A,@R0", case_subb_ind)

    def case_subb_rn(a, idx):
        reset_state(a)
        a.emit(0x98)
    add_case(cases, "SUBB A,R0", case_subb_rn)

    # MUL/DIV/DA
    def case_mul(a, idx):
        reset_state(a)
        a.emit(0xA4)
    add_case(cases, "MUL AB", case_mul)

    def case_div(a, idx):
        reset_state(a)
        mov_direct_imm(a, 0xF0, 0x03)
        a.emit(0x84)
    add_case(cases, "DIV AB", case_div)

    def case_daa(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x9A)
        a.emit(0xD4)
    add_case(cases, "DA A", case_daa)

    # Logical A with src
    def case_orl_imm(a, idx):
        reset_state(a)
        a.emit(0x44, 0x0F)
    add_case(cases, "ORL A,#imm", case_orl_imm)

    def case_orl_direct(a, idx):
        reset_state(a)
        a.emit(0x45, 0x30)
    add_case(cases, "ORL A,direct", case_orl_direct)

    def case_orl_ind(a, idx):
        reset_state(a)
        a.emit(0x46)
    add_case(cases, "ORL A,@R0", case_orl_ind)

    def case_orl_rn(a, idx):
        reset_state(a)
        a.emit(0x48)
    add_case(cases, "ORL A,R0", case_orl_rn)

    def case_anl_imm(a, idx):
        reset_state(a)
        a.emit(0x54, 0x0F)
    add_case(cases, "ANL A,#imm", case_anl_imm)

    def case_anl_direct(a, idx):
        reset_state(a)
        a.emit(0x55, 0x30)
    add_case(cases, "ANL A,direct", case_anl_direct)

    def case_anl_ind(a, idx):
        reset_state(a)
        a.emit(0x56)
    add_case(cases, "ANL A,@R0", case_anl_ind)

    def case_anl_rn(a, idx):
        reset_state(a)
        a.emit(0x58)
    add_case(cases, "ANL A,R0", case_anl_rn)

    def case_xrl_imm(a, idx):
        reset_state(a)
        a.emit(0x64, 0x0F)
    add_case(cases, "XRL A,#imm", case_xrl_imm)

    def case_xrl_direct(a, idx):
        reset_state(a)
        a.emit(0x65, 0x30)
    add_case(cases, "XRL A,direct", case_xrl_direct)

    def case_xrl_ind(a, idx):
        reset_state(a)
        a.emit(0x66)
    add_case(cases, "XRL A,@R0", case_xrl_ind)

    def case_xrl_rn(a, idx):
        reset_state(a)
        a.emit(0x68)
    add_case(cases, "XRL A,R0", case_xrl_rn)

    # Logical direct
    def case_orl_direct_a(a, idx):
        reset_state(a)
        a.emit(0x42, 0x30)
    add_case(cases, "ORL direct,A", case_orl_direct_a)

    def case_orl_direct_imm(a, idx):
        reset_state(a)
        a.emit(0x43, 0x30, 0x0F)
    add_case(cases, "ORL direct,#imm", case_orl_direct_imm)

    def case_anl_direct_a(a, idx):
        reset_state(a)
        a.emit(0x52, 0x30)
    add_case(cases, "ANL direct,A", case_anl_direct_a)

    def case_anl_direct_imm(a, idx):
        reset_state(a)
        a.emit(0x53, 0x30, 0x0F)
    add_case(cases, "ANL direct,#imm", case_anl_direct_imm)

    def case_xrl_direct_a(a, idx):
        reset_state(a)
        a.emit(0x62, 0x30)
    add_case(cases, "XRL direct,A", case_xrl_direct_a)

    def case_xrl_direct_imm(a, idx):
        reset_state(a)
        a.emit(0x63, 0x30, 0x0F)
    add_case(cases, "XRL direct,#imm", case_xrl_direct_imm)

    # MOV variants
    def case_mov_a_imm(a, idx):
        reset_state(a)
        a.emit(0x74, 0xAA)
    add_case(cases, "MOV A,#imm", case_mov_a_imm)

    def case_mov_a_direct(a, idx):
        reset_state(a)
        a.emit(0xE5, 0x30)
    add_case(cases, "MOV A,direct", case_mov_a_direct)

    def case_mov_a_ind(a, idx):
        reset_state(a)
        a.emit(0xE6)
    add_case(cases, "MOV A,@R0", case_mov_a_ind)

    def case_mov_a_rn(a, idx):
        reset_state(a)
        a.emit(0xE8)
    add_case(cases, "MOV A,R0", case_mov_a_rn)

    def case_mov_direct_a(a, idx):
        reset_state(a)
        a.emit(0xF5, 0x30)
    add_case(cases, "MOV direct,A", case_mov_direct_a)

    def case_mov_direct_imm(a, idx):
        reset_state(a)
        a.emit(0x75, 0x30, 0x11)
    add_case(cases, "MOV direct,#imm", case_mov_direct_imm)

    def case_mov_direct_direct(a, idx):
        reset_state(a)
        a.emit(0x85, 0x30, 0x31)
    add_case(cases, "MOV direct,direct", case_mov_direct_direct)

    def case_mov_direct_ind(a, idx):
        reset_state(a)
        a.emit(0x86, 0x30)
    add_case(cases, "MOV direct,@R0", case_mov_direct_ind)

    def case_mov_direct_rn(a, idx):
        reset_state(a)
        a.emit(0x88, 0x30)
    add_case(cases, "MOV direct,R0", case_mov_direct_rn)

    def case_mov_ind_imm(a, idx):
        reset_state(a)
        a.emit(0x76, 0x11)
    add_case(cases, "MOV @R0,#imm", case_mov_ind_imm)

    def case_mov_ind_a(a, idx):
        reset_state(a)
        a.emit(0xF6)
    add_case(cases, "MOV @R0,A", case_mov_ind_a)

    def case_mov_ind_direct(a, idx):
        reset_state(a)
        a.emit(0xA6, 0x30)
    add_case(cases, "MOV @R0,direct", case_mov_ind_direct)

    def case_mov_rn_imm(a, idx):
        reset_state(a)
        a.emit(0x78, 0x11)
    add_case(cases, "MOV R0,#imm", case_mov_rn_imm)

    def case_mov_rn_a(a, idx):
        reset_state(a)
        a.emit(0xF8)
    add_case(cases, "MOV R0,A", case_mov_rn_a)

    def case_mov_rn_direct(a, idx):
        reset_state(a)
        a.emit(0xA8, 0x30)
    add_case(cases, "MOV R0,direct", case_mov_rn_direct)

    def case_mov_dptr(a, idx):
        reset_state(a)
        a.emit(0x90, 0x12, 0x34)
    add_case(cases, "MOV DPTR,#imm16", case_mov_dptr)

    # MOV C,bit / MOV bit,C
    def case_mov_c_bit(a, idx):
        reset_state(a)
        a.emit(0xA2, 0x00)
    add_case(cases, "MOV C,bit", case_mov_c_bit)

    def case_mov_bit_c(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x92, 0x00)
    add_case(cases, "MOV bit,C", case_mov_bit_c)

    # XCH / XCHD
    def case_xch_direct(a, idx):
        reset_state(a)
        a.emit(0xC5, 0x30)
    add_case(cases, "XCH A,direct", case_xch_direct)

    def case_xch_ind(a, idx):
        reset_state(a)
        a.emit(0xC6)
    add_case(cases, "XCH A,@R0", case_xch_ind)

    def case_xch_rn(a, idx):
        reset_state(a)
        a.emit(0xC8)
    add_case(cases, "XCH A,R0", case_xch_rn)

    def case_xchd(a, idx):
        reset_state(a)
        a.emit(0xD6)
    add_case(cases, "XCHD A,@R0", case_xchd)

    # MOVX
    def case_movx_dptr(a, idx):
        reset_state(a)
        mov_dptr_imm(a, 0x1234)
        mov_a_imm(a, 0x5A)
        a.emit(0xF0)  # MOVX @DPTR,A
        mov_a_imm(a, 0x00)
        a.emit(0xE0)  # MOVX A,@DPTR
    add_case(cases, "MOVX @DPTR/A", case_movx_dptr)

    def case_movx_ind(a, idx):
        reset_state(a)
        mov_rn_imm(a, 0, 0x40)
        mov_a_imm(a, 0xA5)
        a.emit(0xF2)  # MOVX @R0,A
        mov_a_imm(a, 0x00)
        a.emit(0xE2)  # MOVX A,@R0
    add_case(cases, "MOVX @Ri/A", case_movx_ind)

    # MOVC
    def case_movc_pc(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x03)
        a.emit(0x83)  # MOVC A,@A+PC
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_movc_pc_done")
        a.label(f"case_{idx}_movc_pc_data")
        a.emit(0x11, 0x22, 0x33, 0x44)
        a.label(f"case_{idx}_movc_pc_done")
    add_case(cases, "MOVC A,@A+PC", case_movc_pc)

    def case_movc_dptr(a, idx):
        reset_state(a)
        mov_dptr_label(a, "code_data_dptr")
        mov_a_imm(a, 0x02)
        a.emit(0x93)
    add_case(cases, "MOVC A,@A+DPTR", case_movc_dptr)

    # CLR/SETB/CPL A and C
    def case_clr_a(a, idx):
        reset_state(a)
        a.emit(0xE4)
    add_case(cases, "CLR A", case_clr_a)

    def case_cpl_a(a, idx):
        reset_state(a)
        a.emit(0xF4)
    add_case(cases, "CPL A", case_cpl_a)

    def case_swap_a(a, idx):
        reset_state(a)
        a.emit(0xC4)
    add_case(cases, "SWAP A", case_swap_a)

    def case_clr_c(a, idx):
        reset_state(a)
        a.emit(0xC3)
    add_case(cases, "CLR C", case_clr_c)

    def case_setb_c(a, idx):
        reset_state(a)
        a.emit(0xD3)
    add_case(cases, "SETB C", case_setb_c)

    def case_cpl_c(a, idx):
        reset_state(a)
        a.emit(0xB3)
    add_case(cases, "CPL C", case_cpl_c)

    # Bit ops
    def case_setb_bit(a, idx):
        reset_state(a)
        a.emit(0xD2, 0x00)
    add_case(cases, "SETB bit", case_setb_bit)

    def case_clr_bit(a, idx):
        reset_state(a)
        a.emit(0xC2, 0x00)
    add_case(cases, "CLR bit", case_clr_bit)

    def case_cpl_bit(a, idx):
        reset_state(a)
        a.emit(0xB2, 0x00)
    add_case(cases, "CPL bit", case_cpl_bit)

    def case_anl_c_bit(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x82, 0x00)
    add_case(cases, "ANL C,bit", case_anl_c_bit)

    def case_anl_c_nbit(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0xB0, 0x00)
    add_case(cases, "ANL C,/bit", case_anl_c_nbit)

    def case_orl_c_bit(a, idx):
        reset_state(a)
        clr_c(a)
        a.emit(0x72, 0x00)
    add_case(cases, "ORL C,bit", case_orl_c_bit)

    def case_orl_c_nbit(a, idx):
        reset_state(a)
        clr_c(a)
        a.emit(0xA0, 0x00)
    add_case(cases, "ORL C,/bit", case_orl_c_nbit)

    # PUSH/POP
    def case_push_pop(a, idx):
        reset_state(a)
        a.emit(0xC0, 0x30)
        a.emit(0xD0, 0x31)
    add_case(cases, "PUSH/POP", case_push_pop)

    # Jumps/branches
    def case_sjmp(a, idx):
        reset_state(a)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_sjmp_t")
        mov_a_imm(a, 0x11)
        a.label(f"case_{idx}_sjmp_t")
        mov_a_imm(a, 0x22)
    add_case(cases, "SJMP", case_sjmp)

    def case_jz(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x00)
        a.emit(0x60)
        a.emit_rel(f"case_{idx}_jz_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jz_done")
        a.label(f"case_{idx}_jz_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jz_done")
    add_case(cases, "JZ", case_jz)

    def case_jnz(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x01)
        a.emit(0x70)
        a.emit_rel(f"case_{idx}_jnz_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jnz_done")
        a.label(f"case_{idx}_jnz_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jnz_done")
    add_case(cases, "JNZ", case_jnz)

    def case_jc(a, idx):
        reset_state(a)
        setb_c(a)
        a.emit(0x40)
        a.emit_rel(f"case_{idx}_jc_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jc_done")
        a.label(f"case_{idx}_jc_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jc_done")
    add_case(cases, "JC", case_jc)

    def case_jnc(a, idx):
        reset_state(a)
        clr_c(a)
        a.emit(0x50)
        a.emit_rel(f"case_{idx}_jnc_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jnc_done")
        a.label(f"case_{idx}_jnc_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jnc_done")
    add_case(cases, "JNC", case_jnc)

    def case_jb(a, idx):
        reset_state(a)
        a.emit(0x20, 0x00)
        a.emit_rel(f"case_{idx}_jb_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jb_done")
        a.label(f"case_{idx}_jb_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jb_done")
    add_case(cases, "JB", case_jb)

    def case_jnb(a, idx):
        reset_state(a)
        a.emit(0x30, 0x07)
        a.emit_rel(f"case_{idx}_jnb_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jnb_done")
        a.label(f"case_{idx}_jnb_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jnb_done")
    add_case(cases, "JNB", case_jnb)

    def case_jbc(a, idx):
        reset_state(a)
        a.emit(0x10, 0x00)
        a.emit_rel(f"case_{idx}_jbc_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jbc_done")
        a.label(f"case_{idx}_jbc_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jbc_done")
    add_case(cases, "JBC", case_jbc)

    def case_cjne_imm(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x00)
        a.emit(0xB4, 0x01)
        a.emit_rel(f"case_{idx}_cjne_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_cjne_done")
        a.label(f"case_{idx}_cjne_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_cjne_done")
    add_case(cases, "CJNE A,#imm", case_cjne_imm)

    def case_cjne_direct(a, idx):
        reset_state(a)
        a.emit(0xB5, 0x30)
        a.emit_rel(f"case_{idx}_cjned_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_cjned_done")
        a.label(f"case_{idx}_cjned_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_cjned_done")
    add_case(cases, "CJNE A,direct", case_cjne_direct)

    def case_cjne_ind(a, idx):
        reset_state(a)
        a.emit(0xB6, 0x11)
        a.emit_rel(f"case_{idx}_cjneri_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_cjneri_done")
        a.label(f"case_{idx}_cjneri_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_cjneri_done")
    add_case(cases, "CJNE @R0,#imm", case_cjne_ind)

    def case_cjne_rn(a, idx):
        reset_state(a)
        a.emit(0xB8, 0x11)
        a.emit_rel(f"case_{idx}_cjnerr_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_cjnerr_done")
        a.label(f"case_{idx}_cjnerr_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_cjnerr_done")
    add_case(cases, "CJNE R0,#imm", case_cjne_rn)

    def case_djnz_direct(a, idx):
        reset_state(a)
        mov_direct_imm(a, 0x30, 0x02)
        a.emit(0xD5, 0x30)
        a.emit_rel(f"case_{idx}_djnz_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_djnz_done")
        a.label(f"case_{idx}_djnz_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_djnz_done")
    add_case(cases, "DJNZ direct", case_djnz_direct)

    def case_djnz_rn(a, idx):
        reset_state(a)
        mov_rn_imm(a, 0, 0x02)
        a.emit(0xD8)
        a.emit_rel(f"case_{idx}_djnzr_t")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_djnzr_done")
        a.label(f"case_{idx}_djnzr_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_djnzr_done")
    add_case(cases, "DJNZ R0", case_djnz_rn)

    def case_ljmp(a, idx):
        reset_state(a)
        a.emit(0x02)
        a.emit_addr16(f"case_{idx}_ljmp_t")
        mov_a_imm(a, 0x11)
        a.label(f"case_{idx}_ljmp_t")
        mov_a_imm(a, 0x22)
    add_case(cases, "LJMP", case_ljmp)

    def case_lcall_ret(a, idx):
        reset_state(a)
        a.emit(0x12)
        a.emit_addr16(f"case_{idx}_lcall_sub")
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_lcall_done")
        a.label(f"case_{idx}_lcall_sub")
        mov_a_imm(a, 0x22)
        a.emit(0x22)
        a.label(f"case_{idx}_lcall_done")
    add_case(cases, "LCALL/RET", case_lcall_ret)

    def case_reti(a, idx):
        reset_state(a)
        a.emit(0x12)
        a.emit_addr16(f"case_{idx}_reti_sub")
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_reti_done")
        a.label(f"case_{idx}_reti_sub")
        a.emit(0x32)
        a.label(f"case_{idx}_reti_done")
    add_case(cases, "RETI", case_reti)

    def case_ajmp(a, idx):
        reset_state(a)
        pos = a.pc
        a.emit(0x00, 0x00)
        a.fixups.append((pos, f"case_{idx}_ajmp_t", "ajmp_full"))
        mov_a_imm(a, 0x11)
        a.label(f"case_{idx}_ajmp_t")
        mov_a_imm(a, 0x22)
    add_case(cases, "AJMP", case_ajmp)

    def case_acall(a, idx):
        reset_state(a)
        pos = a.pc
        a.emit(0x00, 0x00)
        a.fixups.append((pos, f"case_{idx}_acall_sub", "acall_full"))
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_acall_done")
        a.label(f"case_{idx}_acall_sub")
        mov_a_imm(a, 0x22)
        a.emit(0x22)
        a.label(f"case_{idx}_acall_done")
    add_case(cases, "ACALL", case_acall)

    def case_jmp_a_dptr(a, idx):
        reset_state(a)
        mov_dptr_label(a, f"case_{idx}_jmp_table")
        mov_a_imm(a, 0x03)
        a.emit(0x73)  # JMP @A+DPTR
        mov_a_imm(a, 0x11)
        a.emit(0x80)
        a.emit_rel(f"case_{idx}_jmp_done")
        a.label(f"case_{idx}_jmp_table")
        a.emit(0x00, 0x00, 0x00)
        a.label(f"case_{idx}_jmp_t")
        mov_a_imm(a, 0x22)
        a.label(f"case_{idx}_jmp_done")
    add_case(cases, "JMP @A+DPTR", case_jmp_a_dptr)

    # Illegal opcode treated as NOP
    def case_illegal(a, idx):
        reset_state(a)
        mov_a_imm(a, 0x11)
        a.emit(0xA5)  # illegal on 8051
        a.emit(0x04)  # INC A
    add_case(cases, "Illegal opcode", case_illegal)

    # Emit all cases
    out_addr = 0x0000
    for idx, (name, fn) in enumerate(cases):
        fn(a, idx)
        out_addr += dump_state(a, out_addr)

    # Code data for MOVC A,@A+DPTR
    a.label("code_data_dptr")
    a.emit(0x11, 0x22, 0x33, 0x44)

    # Done signature and halt
    a.label("done")
    mov_dptr_imm(a, 0xFFFE)
    mov_a_imm(a, 0xA5)
    a.emit(0xF0)
    a.emit(0x80)
    a.emit_rel("done")

    # Resolve fixups
    a.resolve()
    return a, out_addr


def write_hex(mem, path):
    # Intel HEX, 16-byte records
    max_addr = max(mem.keys()) if mem else 0
    addr = 0
    with open(path, "w") as f:
        while addr <= max_addr:
            chunk = [mem.get(addr + i, 0x00) for i in range(16)]
            if not any(chunk):
                addr += 16
                continue
            record = [0x10, (addr >> 8) & 0xFF, addr & 0xFF, 0x00] + chunk
            csum = ((-sum(record)) & 0xFF)
            f.write(":" + "".join(f"{b:02X}" for b in record) + f"{csum:02X}\n")
            addr += 16
        f.write(":00000001FF\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="tests/isa/build/isa_full.hex")
    parser.add_argument("--sym", default="tests/isa/build/isa_full.sym.json")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    a, dump_end = build_program()
    write_hex(a.mem, out_path)

    sym_path = Path(args.sym)
    labels = dict(a.labels)
    labels["_dump_begin"] = 0
    labels["_dump_end"] = dump_end - 1 if dump_end > 0 else 0
    with open(sym_path, "w") as f:
        json.dump(labels, f, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
