#!/usr/bin/env python3
"""Phase 7: generate the FPU cosim test-vector battery.

Emits tests/fpu_vectors.txt, one line per test:
    <ext_2hex> <rx_high_4hex> <rx_low_16hex> <ry_high_4hex> <ry_low_16hex> <round_1hex>

rx is always the SOURCE operand; ry is always the pre-existing DESTINATION
operand (matching rtl/m68882_proto.sv's own apu_a_sel=cmd_rx_r/apu_b_sel=
cmd_ry_r convention, and Table 4-13's own real "opclass 000: RX=source
FPm, RY=destination FPn" rule). Both tools/musashi_fpu_ref (the Musashi
golden-reference harness) and tb/m68882_musashi_cosim_tb.sv (the RTL
cosim testbench) consume this exact same file, so both sides run the
IDENTICAL vector set -- no separate transcription to drift out of sync.

Known-good extended-precision bit patterns below are the same
independently-verified constants tb/m68882_apu_tb.sv already uses
(Table 3-3: sign(1)/exp(15,bias 16383)/reserved(16)/mantissa(64), explicit
integer bit) -- reused here rather than re-derived, since they're already
established as correct.
"""

VALUES = {
    "1.0":  (0x3fff, 0x8000000000000000),
    "2.0":  (0x4000, 0x8000000000000000),
    "3.0":  (0x4000, 0xc000000000000000),
    "0.5":  (0x3ffe, 0x8000000000000000),
    "1.5":  (0x3fff, 0xc000000000000000),
    "4.0":  (0x4001, 0x8000000000000000),
    "6.0":  (0x4001, 0xc000000000000000),
    "9.0":  (0x4002, 0x9000000000000000),
    "0.25": (0x3ffd, 0x8000000000000000),
    "-1.0": (0xbfff, 0x8000000000000000),
    "-3.0": (0xc000, 0xc000000000000000),
    "-4.0": (0xc001, 0x8000000000000000),
    "0.0":  (0x0000, 0x0000000000000000),
    "+inf": (0x7fff, 0x8000000000000000),
    "qnan": (0x7fff, 0xc000000000000001),
}

# (ext, name, rx, ry, round) -- round: 0=Nearest,1=Zero,2=-Inf,3=+Inf
CASES = [
    (0x22, "FADD",  "1.0",  "2.0",  0),
    (0x22, "FADD",  "0.5",  "0.5",  0),
    (0x22, "FADD",  "1.0",  "-1.0", 0),
    (0x22, "FADD",  "1.0",  "+inf", 0),
    (0x22, "FADD",  "1.0",  "qnan", 0),
    (0x28, "FSUB",  "1.0",  "3.0",  0),
    (0x28, "FSUB",  "2.0",  "1.0",  0),
    (0x23, "FMUL",  "2.0",  "2.0",  0),
    (0x23, "FMUL",  "0.5",  "0.5",  0),
    (0x23, "FMUL",  "-1.0", "3.0",  0),
    (0x20, "FDIV",  "2.0",  "4.0",  0),
    (0x20, "FDIV",  "0.0",  "1.0",  0),
    (0x20, "FDIV",  "2.0",  "3.0",  0),
    (0x04, "FSQRT", "4.0",  "0.0",  0),
    (0x04, "FSQRT", "-4.0", "0.0",  0),
    (0x04, "FSQRT", "9.0",  "0.0",  0),
    (0x18, "FABS",  "-3.0", "0.0",  0),
    (0x18, "FABS",  "3.0",  "0.0",  0),
    (0x1A, "FNEG",  "2.0",  "0.0",  0),
    (0x1A, "FNEG",  "-1.0", "0.0",  0),
    (0x38, "FCMP",  "2.0",  "3.0",  0),
    (0x38, "FCMP",  "2.0",  "2.0",  0),
    (0x38, "FCMP",  "3.0",  "2.0",  0),
    (0x3A, "FTST",  "0.0",  "0.0",  0),
    (0x3A, "FTST",  "-1.0", "0.0",  0),
    # Same arithmetic set again under each of the other 3 rounding modes --
    # these 5 core ops are the ones round_mantissa actually branches on.
    (0x22, "FADD",  "1.0",  "2.0",  1),
    (0x22, "FADD",  "1.0",  "2.0",  2),
    (0x22, "FADD",  "1.0",  "2.0",  3),
    (0x20, "FDIV",  "2.0",  "3.0",  1),
    (0x20, "FDIV",  "2.0",  "3.0",  2),
    (0x20, "FDIV",  "2.0",  "3.0",  3),
]


def main():
    with open("tests/fpu_vectors.txt", "w") as f:
        for ext, name, rx, ry, rnd in CASES:
            rx_hi, rx_lo = VALUES[rx]
            ry_hi, ry_lo = VALUES[ry]
            f.write(f"{ext:02x} {rx_hi:04x} {rx_lo:016x} {ry_hi:04x} {ry_lo:016x} {rnd:x}"
                     f"  # {name} {rx},{ry} round={rnd}\n")
    print(f"wrote {len(CASES)} vectors to tests/fpu_vectors.txt")


if __name__ == "__main__":
    main()
