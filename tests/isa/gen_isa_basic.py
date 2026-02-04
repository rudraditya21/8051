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
    mov_dptr_imm(a, out_addr)
    # ACC
    a.emit(0xF0)  # MOVX @DPTR,A
    a.emit(0xA3)  # INC DPTR
    # B
    a.emit(0xE5, 0xF0)
    a.emit(0xF0)
    a.emit(0xA3)
    # PSW, SP, DPL, DPH
    for addr in (0xD0, 0x81, 0x82, 0x83):
        a.emit(0xE5, addr)
        a.emit(0xF0)
        a.emit(0xA3)
    # P0..P3
    for addr in (0x80, 0x90, 0xA0, 0xB0):
        a.emit(0xE5, addr)
        a.emit(0xF0)
        a.emit(0xA3)
    # TCON, TMOD, TL0, TH0, TL1, TH1
    for addr in (0x88, 0x89, 0x8A, 0x8C, 0x8B, 0x8D):
        a.emit(0xE5, addr)
        a.emit(0xF0)
        a.emit(0xA3)
    # SCON, SBUF, IE, IP, PCON
    for addr in (0x98, 0x99, 0xA8, 0xB8, 0x87):
        a.emit(0xE5, addr)
        a.emit(0xF0)
        a.emit(0xA3)
    # R0..R7
    for i in range(8):
        a.emit(0xE8 + i)
        a.emit(0xF0)
        a.emit(0xA3)
    # IRAM 0x20..0x2F
    for addr in range(0x20, 0x30):
        a.emit(0xE5, addr)
        a.emit(0xF0)
        a.emit(0xA3)
    # Pad to 48 bytes total (3 bytes pad)
    for _ in range(3):
        a.emit(0xE4)  # CLR A
        a.emit(0xF0)
        a.emit(0xA3)


def add_case(cases, name, body_fn):
    cases.append((name, body_fn))


def build_program():
    a = Asm(origin=0x0000)
    # Reset vector
    a.emit(0x02)  # LJMP
    a.emit_addr16("main")

    # Place a small AJMP/ACALL test block early
    a.label("pre")
    reset_state(a)
    # AJMP to ajmp_target
    a.emit(0x01)  # AJMP page 0
    a.fixups.append((a.pc, "ajmp_target", "ajmp"))
    a.emit(0x00)
    # Should not execute
    a.emit(0x00)
    a.label("ajmp_target")
    dump_state(a, 0x0000)

    # ACALL test
    reset_state(a)
    a.emit(0x11)  # ACALL page 0
    a.fixups.append((a.pc, "acall_target", "acall"))
    a.emit(0x00)
    a.label("acall_target")
    a.emit(0x22)  # RET
    dump_state(a, 0x0030)

    # Main tests
    a.label("main")

    out_addr = 0x0060
    cases = []

    # NOP
    def case_nop(a):
        reset_state(a)
        a.emit(0x00)
    add_case(cases, "NOP", case_nop)

    # Rotates
    def case_rr(a):
        reset_state(a)
        a.emit(0x03)
    add_case(cases, "RR A", case_rr)

    def case_rl(a):
        reset_state(a)
        a.emit(0x23)
    add_case(cases, "RL A", case_rl)

    def case_rrc(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x13)
    add_case(cases, "RRC A", case_rrc)

    def case_rlc(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x33)
    add_case(cases, "RLC A", case_rlc)

    # INC/DEC
    def case_inc_a(a):
        reset_state(a)
        a.emit(0x04)
    add_case(cases, "INC A", case_inc_a)

    def case_dec_a(a):
        reset_state(a)
        a.emit(0x14)
    add_case(cases, "DEC A", case_dec_a)

    def case_inc_direct(a):
        reset_state(a)
        a.emit(0x05, 0x30)
    add_case(cases, "INC direct", case_inc_direct)

    def case_dec_direct(a):
        reset_state(a)
        a.emit(0x15, 0x31)
    add_case(cases, "DEC direct", case_dec_direct)

    # INC/DEC @Ri
    def case_inc_at_r0(a):
        reset_state(a)
        a.emit(0x06)
    add_case(cases, "INC @R0", case_inc_at_r0)

    def case_inc_at_r1(a):
        reset_state(a)
        a.emit(0x07)
    add_case(cases, "INC @R1", case_inc_at_r1)

    def case_dec_at_r0(a):
        reset_state(a)
        a.emit(0x16)
    add_case(cases, "DEC @R0", case_dec_at_r0)

    def case_dec_at_r1(a):
        reset_state(a)
        a.emit(0x17)
    add_case(cases, "DEC @R1", case_dec_at_r1)

    # INC/DEC Rn
    for n in range(8):
        def make_inc_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x08 + n)
            return _case
        add_case(cases, f"INC R{n}", make_inc_rn(n))

        def make_dec_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x18 + n)
            return _case
        add_case(cases, f"DEC R{n}", make_dec_rn(n))

    # ADD/ADDC/SUBB A, #imm
    def case_add_imm(a):
        reset_state(a)
        a.emit(0x24, 0x22)
    add_case(cases, "ADD A,#imm", case_add_imm)

    def case_addc_imm(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x34, 0x22)
    add_case(cases, "ADDC A,#imm", case_addc_imm)

    def case_subb_imm(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x94, 0x10)
    add_case(cases, "SUBB A,#imm", case_subb_imm)

    # ADD/ADDC/SUBB A, direct
    def case_add_direct(a):
        reset_state(a)
        a.emit(0x25, 0x30)
    add_case(cases, "ADD A,direct", case_add_direct)

    def case_addc_direct(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x35, 0x30)
    add_case(cases, "ADDC A,direct", case_addc_direct)

    def case_subb_direct(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x95, 0x31)
    add_case(cases, "SUBB A,direct", case_subb_direct)

    # ADD/ADDC/SUBB A, @Ri
    def case_add_at_r0(a):
        reset_state(a)
        a.emit(0x26)
    add_case(cases, "ADD A,@R0", case_add_at_r0)

    def case_add_at_r1(a):
        reset_state(a)
        a.emit(0x27)
    add_case(cases, "ADD A,@R1", case_add_at_r1)

    def case_addc_at_r0(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x36)
    add_case(cases, "ADDC A,@R0", case_addc_at_r0)

    def case_addc_at_r1(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x37)
    add_case(cases, "ADDC A,@R1", case_addc_at_r1)

    def case_subb_at_r0(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x96)
    add_case(cases, "SUBB A,@R0", case_subb_at_r0)

    def case_subb_at_r1(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x97)
    add_case(cases, "SUBB A,@R1", case_subb_at_r1)

    # ADD/ADDC/SUBB A, Rn
    for n in range(8):
        def make_add_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x28 + n)
            return _case
        add_case(cases, f"ADD A,R{n}", make_add_rn(n))

        def make_addc_rn(n):
            def _case(a):
                reset_state(a)
                setb_c(a)
                a.emit(0x38 + n)
            return _case
        add_case(cases, f"ADDC A,R{n}", make_addc_rn(n))

        def make_subb_rn(n):
            def _case(a):
                reset_state(a)
                setb_c(a)
                a.emit(0x98 + n)
            return _case
        add_case(cases, f"SUBB A,R{n}", make_subb_rn(n))

    # Logical ops A, #imm
    def case_orl_imm(a):
        reset_state(a)
        a.emit(0x44, 0x0F)
    add_case(cases, "ORL A,#imm", case_orl_imm)

    def case_anl_imm(a):
        reset_state(a)
        a.emit(0x54, 0xF0)
    add_case(cases, "ANL A,#imm", case_anl_imm)

    def case_xrl_imm(a):
        reset_state(a)
        a.emit(0x64, 0xFF)
    add_case(cases, "XRL A,#imm", case_xrl_imm)

    # Logical ops A, direct
    def case_orl_direct(a):
        reset_state(a)
        a.emit(0x45, 0x30)
    add_case(cases, "ORL A,direct", case_orl_direct)

    def case_anl_direct(a):
        reset_state(a)
        a.emit(0x55, 0x31)
    add_case(cases, "ANL A,direct", case_anl_direct)

    def case_xrl_direct(a):
        reset_state(a)
        a.emit(0x65, 0x30)
    add_case(cases, "XRL A,direct", case_xrl_direct)

    # Logical ops A, @Ri
    def case_orl_at_r0(a):
        reset_state(a)
        a.emit(0x46)
    add_case(cases, "ORL A,@R0", case_orl_at_r0)

    def case_orl_at_r1(a):
        reset_state(a)
        a.emit(0x47)
    add_case(cases, "ORL A,@R1", case_orl_at_r1)

    def case_anl_at_r0(a):
        reset_state(a)
        a.emit(0x56)
    add_case(cases, "ANL A,@R0", case_anl_at_r0)

    def case_anl_at_r1(a):
        reset_state(a)
        a.emit(0x57)
    add_case(cases, "ANL A,@R1", case_anl_at_r1)

    def case_xrl_at_r0(a):
        reset_state(a)
        a.emit(0x66)
    add_case(cases, "XRL A,@R0", case_xrl_at_r0)

    def case_xrl_at_r1(a):
        reset_state(a)
        a.emit(0x67)
    add_case(cases, "XRL A,@R1", case_xrl_at_r1)

    # Logical ops A, Rn
    for n in range(8):
        def make_orl_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x48 + n)
            return _case
        add_case(cases, f"ORL A,R{n}", make_orl_rn(n))

        def make_anl_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x58 + n)
            return _case
        add_case(cases, f"ANL A,R{n}", make_anl_rn(n))

        def make_xrl_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x68 + n)
            return _case
        add_case(cases, f"XRL A,R{n}", make_xrl_rn(n))

    # Logical ops direct,A / direct,#imm
    def case_orl_direct_a(a):
        reset_state(a)
        a.emit(0x42, 0x30)
    add_case(cases, "ORL direct,A", case_orl_direct_a)

    def case_orl_direct_imm(a):
        reset_state(a)
        a.emit(0x43, 0x30, 0x0F)
    add_case(cases, "ORL direct,#imm", case_orl_direct_imm)

    def case_anl_direct_a(a):
        reset_state(a)
        a.emit(0x52, 0x31)
    add_case(cases, "ANL direct,A", case_anl_direct_a)

    def case_anl_direct_imm(a):
        reset_state(a)
        a.emit(0x53, 0x31, 0xF0)
    add_case(cases, "ANL direct,#imm", case_anl_direct_imm)

    def case_xrl_direct_a(a):
        reset_state(a)
        a.emit(0x62, 0x30)
    add_case(cases, "XRL direct,A", case_xrl_direct_a)

    def case_xrl_direct_imm(a):
        reset_state(a)
        a.emit(0x63, 0x30, 0xFF)
    add_case(cases, "XRL direct,#imm", case_xrl_direct_imm)

    # MOV variants
    def case_mov_a_imm(a):
        reset_state(a)
        mov_a_imm(a, 0x77)
    add_case(cases, "MOV A,#imm", case_mov_a_imm)

    def case_mov_direct_imm(a):
        reset_state(a)
        mov_direct_imm(a, 0x30, 0x66)
    add_case(cases, "MOV direct,#imm", case_mov_direct_imm)

    def case_mov_at_r0_imm(a):
        reset_state(a)
        a.emit(0x76, 0x55)
    add_case(cases, "MOV @R0,#imm", case_mov_at_r0_imm)

    def case_mov_at_r1_imm(a):
        reset_state(a)
        a.emit(0x77, 0x55)
    add_case(cases, "MOV @R1,#imm", case_mov_at_r1_imm)

    for n in range(8):
        def make_mov_rn_imm(n):
            def _case(a):
                reset_state(a)
                mov_rn_imm(a, n, 0x55)
            return _case
        add_case(cases, f"MOV R{n},#imm", make_mov_rn_imm(n))

    def case_mov_a_direct(a):
        reset_state(a)
        a.emit(0xE5, 0x30)
    add_case(cases, "MOV A,direct", case_mov_a_direct)

    def case_mov_a_at_r0(a):
        reset_state(a)
        a.emit(0xE6)
    add_case(cases, "MOV A,@R0", case_mov_a_at_r0)

    def case_mov_a_at_r1(a):
        reset_state(a)
        a.emit(0xE7)
    add_case(cases, "MOV A,@R1", case_mov_a_at_r1)

    for n in range(8):
        def make_mov_a_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0xE8 + n)
            return _case
        add_case(cases, f"MOV A,R{n}", make_mov_a_rn(n))

    def case_mov_direct_a(a):
        reset_state(a)
        a.emit(0xF5, 0x30)
    add_case(cases, "MOV direct,A", case_mov_direct_a)

    def case_mov_at_r0_a(a):
        reset_state(a)
        a.emit(0xF6)
    add_case(cases, "MOV @R0,A", case_mov_at_r0_a)

    def case_mov_at_r1_a(a):
        reset_state(a)
        a.emit(0xF7)
    add_case(cases, "MOV @R1,A", case_mov_at_r1_a)

    for n in range(8):
        def make_mov_rn_a(n):
            def _case(a):
                reset_state(a)
                a.emit(0xF8 + n)
            return _case
        add_case(cases, f"MOV R{n},A", make_mov_rn_a(n))

    def case_mov_direct_direct(a):
        reset_state(a)
        a.emit(0x85, 0x30, 0x31)
    add_case(cases, "MOV direct,direct", case_mov_direct_direct)

    def case_mov_direct_at_r0(a):
        reset_state(a)
        a.emit(0x86, 0x30)
    add_case(cases, "MOV direct,@R0", case_mov_direct_at_r0)

    def case_mov_direct_at_r1(a):
        reset_state(a)
        a.emit(0x87, 0x30)
    add_case(cases, "MOV direct,@R1", case_mov_direct_at_r1)

    for n in range(8):
        def make_mov_direct_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0x88 + n, 0x30)
            return _case
        add_case(cases, f"MOV direct,R{n}", make_mov_direct_rn(n))

    def case_mov_at_r0_direct(a):
        reset_state(a)
        a.emit(0xA6, 0x30)
    add_case(cases, "MOV @R0,direct", case_mov_at_r0_direct)

    def case_mov_at_r1_direct(a):
        reset_state(a)
        a.emit(0xA7, 0x31)
    add_case(cases, "MOV @R1,direct", case_mov_at_r1_direct)

    for n in range(8):
        def make_mov_rn_direct(n):
            def _case(a):
                reset_state(a)
                a.emit(0xA8 + n, 0x30)
            return _case
        add_case(cases, f"MOV R{n},direct", make_mov_rn_direct(n))

    def case_mov_dptr_imm(a):
        reset_state(a)
        mov_dptr_imm(a, 0x1234)
    add_case(cases, "MOV DPTR,#imm16", case_mov_dptr_imm)

    # MOVC
    def case_movc_a_dptr(a):
        reset_state(a)
        a.emit(0x90)
        a.emit_addr16("code_data")
        mov_a_imm(a, 0x01)
        a.emit(0x93)
    add_case(cases, "MOVC A,@A+DPTR", case_movc_a_dptr)

    def case_movc_a_pc(a):
        reset_state(a)
        mov_a_imm(a, 0x02)  # skip over SJMP
        a.emit(0x83)         # MOVC A,@A+PC
        a.emit(0x80)         # SJMP after_data
        a.emit_rel("movc_pc_after")
        a.label("movc_pc_data")
        a.emit(0x42, 0x43, 0x44)
        a.label("movc_pc_after")
    add_case(cases, "MOVC A,@A+PC", case_movc_a_pc)

    # MOVX
    def case_movx_dptr_write(a):
        reset_state(a)
        mov_dptr_imm(a, 0x0200)
        mov_a_imm(a, 0x5A)
        a.emit(0xF0)
    add_case(cases, "MOVX @DPTR,A", case_movx_dptr_write)

    def case_movx_dptr_read(a):
        reset_state(a)
        mov_dptr_imm(a, 0x0200)
        mov_a_imm(a, 0xA5)
        a.emit(0xF0)
        mov_a_imm(a, 0x00)
        a.emit(0xE0)
    add_case(cases, "MOVX A,@DPTR", case_movx_dptr_read)

    def case_movx_r0_write(a):
        reset_state(a)
        mov_rn_imm(a, 0, 0x80)
        mov_a_imm(a, 0x33)
        a.emit(0xF2)
    add_case(cases, "MOVX @R0,A", case_movx_r0_write)

    def case_movx_r1_read(a):
        reset_state(a)
        mov_rn_imm(a, 1, 0x81)
        mov_a_imm(a, 0x99)
        a.emit(0xF3)
        mov_a_imm(a, 0x00)
        a.emit(0xE3)
    add_case(cases, "MOVX A,@R1", case_movx_r1_read)

    # Bit ops
    def case_clr_a(a):
        reset_state(a)
        a.emit(0xE4)
    add_case(cases, "CLR A", case_clr_a)

    def case_cpl_a(a):
        reset_state(a)
        a.emit(0xF4)
    add_case(cases, "CPL A", case_cpl_a)

    def case_clr_c(a):
        reset_state(a)
        a.emit(0xC3)
    add_case(cases, "CLR C", case_clr_c)

    def case_setb_c(a):
        reset_state(a)
        a.emit(0xD3)
    add_case(cases, "SETB C", case_setb_c)

    def case_cpl_c(a):
        reset_state(a)
        a.emit(0xB3)
    add_case(cases, "CPL C", case_cpl_c)

    def case_clr_bit(a):
        reset_state(a)
        a.emit(0xC2, 0x00)
    add_case(cases, "CLR bit", case_clr_bit)

    def case_setb_bit(a):
        reset_state(a)
        a.emit(0xD2, 0x00)
    add_case(cases, "SETB bit", case_setb_bit)

    def case_cpl_bit(a):
        reset_state(a)
        a.emit(0xB2, 0x00)
    add_case(cases, "CPL bit", case_cpl_bit)

    def case_mov_c_bit(a):
        reset_state(a)
        a.emit(0xA2, 0x00)
    add_case(cases, "MOV C,bit", case_mov_c_bit)

    def case_mov_bit_c(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x92, 0x00)
    add_case(cases, "MOV bit,C", case_mov_bit_c)

    # Logical ops with C
    def case_anl_c_bit(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x82, 0x00)
    add_case(cases, "ANL C,bit", case_anl_c_bit)

    def case_anl_c_nbit(a):
        reset_state(a)
        setb_c(a)
        a.emit(0xB0, 0x00)
    add_case(cases, "ANL C,/bit", case_anl_c_nbit)

    def case_orl_c_bit(a):
        reset_state(a)
        clr_c(a)
        a.emit(0x72, 0x00)
    add_case(cases, "ORL C,bit", case_orl_c_bit)

    def case_orl_c_nbit(a):
        reset_state(a)
        clr_c(a)
        a.emit(0xA0, 0x00)
    add_case(cases, "ORL C,/bit", case_orl_c_nbit)

    # Misc
    def case_swap(a):
        reset_state(a)
        a.emit(0xC4)
    add_case(cases, "SWAP A", case_swap)

    def case_xch_direct(a):
        reset_state(a)
        a.emit(0xC5, 0x30)
    add_case(cases, "XCH A,direct", case_xch_direct)

    def case_xch_at_r0(a):
        reset_state(a)
        a.emit(0xC6)
    add_case(cases, "XCH A,@R0", case_xch_at_r0)

    def case_xch_at_r1(a):
        reset_state(a)
        a.emit(0xC7)
    add_case(cases, "XCH A,@R1", case_xch_at_r1)

    for n in range(8):
        def make_xch_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0xC8 + n)
            return _case
        add_case(cases, f"XCH A,R{n}", make_xch_rn(n))

    def case_xchd_r0(a):
        reset_state(a)
        a.emit(0xD6)
    add_case(cases, "XCHD A,@R0", case_xchd_r0)

    def case_xchd_r1(a):
        reset_state(a)
        a.emit(0xD7)
    add_case(cases, "XCHD A,@R1", case_xchd_r1)

    def case_push(a):
        reset_state(a)
        a.emit(0xC0, 0x30)
    add_case(cases, "PUSH direct", case_push)

    def case_pop(a):
        reset_state(a)
        a.emit(0xC0, 0x30)
        a.emit(0xD0, 0x31)
    add_case(cases, "POP direct", case_pop)

    # INC DPTR
    def case_inc_dptr(a):
        reset_state(a)
        a.emit(0xA3)
    add_case(cases, "INC DPTR", case_inc_dptr)

    # MUL/DIV/DAA
    def case_mul(a):
        reset_state(a)
        a.emit(0xA4)
    add_case(cases, "MUL AB", case_mul)

    def case_div(a):
        reset_state(a)
        a.emit(0x84)
    add_case(cases, "DIV AB", case_div)

    def case_da(a):
        reset_state(a)
        mov_a_imm(a, 0x9A)
        a.emit(0xD4)
    add_case(cases, "DA A", case_da)

    # Jumps and branches
    def case_sjmp(a):
        reset_state(a)
        a.emit(0x80)
        a.emit_rel("sjmp_target")
        a.label("sjmp_skip")
        a.emit(0x00)
        a.label("sjmp_target")
    add_case(cases, "SJMP rel", case_sjmp)

    def case_jc(a):
        reset_state(a)
        setb_c(a)
        a.emit(0x40)
        a.emit_rel("jc_target")
        a.emit(0x00)
        a.label("jc_target")
    add_case(cases, "JC", case_jc)

    def case_jnc(a):
        reset_state(a)
        clr_c(a)
        a.emit(0x50)
        a.emit_rel("jnc_target")
        a.emit(0x00)
        a.label("jnc_target")
    add_case(cases, "JNC", case_jnc)

    def case_jz(a):
        reset_state(a)
        mov_a_imm(a, 0x00)
        a.emit(0x60)
        a.emit_rel("jz_target")
        a.emit(0x00)
        a.label("jz_target")
    add_case(cases, "JZ", case_jz)

    def case_jnz(a):
        reset_state(a)
        mov_a_imm(a, 0x01)
        a.emit(0x70)
        a.emit_rel("jnz_target")
        a.emit(0x00)
        a.label("jnz_target")
    add_case(cases, "JNZ", case_jnz)

    def case_jb(a):
        reset_state(a)
        a.emit(0x20, 0x00)
        a.emit_rel("jb_target")
        a.emit(0x00)
        a.label("jb_target")
    add_case(cases, "JB", case_jb)

    def case_jnb(a):
        reset_state(a)
        a.emit(0x30, 0x00)
        a.emit_rel("jnb_target")
        a.emit(0x00)
        a.label("jnb_target")
    add_case(cases, "JNB", case_jnb)

    def case_jbc(a):
        reset_state(a)
        a.emit(0x10, 0x00)
        a.emit_rel("jbc_target")
        a.emit(0x00)
        a.label("jbc_target")
    add_case(cases, "JBC", case_jbc)

    def case_cjne_a_imm(a):
        reset_state(a)
        a.emit(0xB4, 0x11)
        a.emit_rel("cjne_ai_target")
        a.emit(0x00)
        a.label("cjne_ai_target")
    add_case(cases, "CJNE A,#imm", case_cjne_a_imm)

    def case_cjne_a_direct(a):
        reset_state(a)
        a.emit(0xB5, 0x30)
        a.emit_rel("cjne_ad_target")
        a.emit(0x00)
        a.label("cjne_ad_target")
    add_case(cases, "CJNE A,direct", case_cjne_a_direct)

    def case_cjne_at_r0(a):
        reset_state(a)
        a.emit(0xB6, 0x11)
        a.emit_rel("cjne_at_r0_target")
        a.emit(0x00)
        a.label("cjne_at_r0_target")
    add_case(cases, "CJNE @R0,#imm", case_cjne_at_r0)

    def case_cjne_at_r1(a):
        reset_state(a)
        a.emit(0xB7, 0x11)
        a.emit_rel("cjne_at_r1_target")
        a.emit(0x00)
        a.label("cjne_at_r1_target")
    add_case(cases, "CJNE @R1,#imm", case_cjne_at_r1)

    for n in range(8):
        def make_cjne_rn(n):
            def _case(a):
                reset_state(a)
                a.emit(0xB8 + n, 0x11)
                a.emit_rel(f"cjne_r{n}_target")
                a.emit(0x00)
                a.label(f"cjne_r{n}_target")
            return _case
        add_case(cases, f"CJNE R{n},#imm", make_cjne_rn(n))

    def case_djnz_direct(a):
        reset_state(a)
        mov_direct_imm(a, 0x32, 0x02)
        a.emit(0xD5, 0x32)
        a.emit_rel("djnz_direct_target")
        a.emit(0x00)
        a.label("djnz_direct_target")
    add_case(cases, "DJNZ direct,rel", case_djnz_direct)

    for n in range(8):
        def make_djnz_rn(n):
            def _case(a):
                reset_state(a)
                mov_rn_imm(a, n, 0x02)
                a.emit(0xD8 + n)
                a.emit_rel(f"djnz_r{n}_target")
                a.emit(0x00)
                a.label(f"djnz_r{n}_target")
            return _case
        add_case(cases, f"DJNZ R{n},rel", make_djnz_rn(n))

    # JMP @A+DPTR
    def case_jmp_a_dptr(a):
        reset_state(a)
        a.emit(0x90)
        a.emit_addr16("jmp_a_dptr_target")
        mov_a_imm(a, 0x00)
        a.emit(0x73)
        a.label("jmp_a_dptr_target")
    add_case(cases, "JMP @A+DPTR", case_jmp_a_dptr)

    # CALL/RET
    def case_lcall_ret(a):
        reset_state(a)
        a.emit(0x12)
        a.emit_addr16("sub_ret")
    add_case(cases, "LCALL/RET", case_lcall_ret)

    def case_reti(a):
        reset_state(a)
        a.emit(0x12)
        a.emit_addr16("sub_reti")
    add_case(cases, "RETI", case_reti)

    # LJMP
    def case_ljmp(a):
        reset_state(a)
        a.emit(0x02)
        a.emit_addr16("ljmp_target")
        a.emit(0x00)
        a.label("ljmp_target")
    add_case(cases, "LJMP", case_ljmp)

    # Emit all cases
    for name, fn in cases:
        fn(a)
        dump_state(a, out_addr)
        out_addr += 0x30

    # Subroutines and data
    a.label("sub_ret")
    a.emit(0x22)

    a.label("sub_reti")
    a.emit(0x32)

    a.label("code_data")
    a.emit(0x11, 0x22, 0x33, 0x44, 0x55)

    # Done signature and halt
    a.label("done")
    mov_dptr_imm(a, 0xFFFE)
    mov_a_imm(a, 0xA5)
    a.emit(0xF0)
    a.emit(0x80)
    a.emit_rel("done")

    # Resolve fixups
    a.resolve()
    return a


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
    parser.add_argument("--out", default="tests/isa/build/isa_basic.hex")
    parser.add_argument("--sym", default="tests/isa/build/isa_basic.sym.json")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    a = build_program()
    write_hex(a.mem, out_path)

    with open(args.sym, "w") as f:
        json.dump(a.labels, f, indent=2, sort_keys=True)

    print(f"Wrote {out_path} and {args.sym}")


if __name__ == "__main__":
    main()
