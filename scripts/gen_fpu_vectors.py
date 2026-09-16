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
    # Phase 9b additions
    "2.5":   (0x4000, 0xa000000000000000),
    "-2.5":  (0xc000, 0xa000000000000000),
    "0.75":  (0x3ffe, 0xc000000000000000),
    "-0.75": (0xbffe, 0xc000000000000000),
    "8.0":   (0x4002, 0x8000000000000000),
    "-8.0":  (0xc002, 0x8000000000000000),
    "7.0":   (0x4001, 0xe000000000000000),
    "-7.0":  (0xc001, 0xe000000000000000),
    "10.0":  (0x4002, 0xa000000000000000),
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
    # Phase 9b: exact auxiliary ops (FINT/FINTRZ/FGETEXP). ry is unused
    # (all 3 are monadic) but still required by the vector format.
    (0x01, "FINT",   "2.5",   "0.0", 0), # nearest, exact tie -> even (2.0)
    (0x01, "FINT",   "2.5",   "0.0", 1), # toward zero -> 2.0
    (0x01, "FINT",   "2.5",   "0.0", 2), # toward -inf -> 2.0
    (0x01, "FINT",   "2.5",   "0.0", 3), # toward +inf -> 3.0
    (0x01, "FINT",   "-2.5",  "0.0", 0), # nearest, exact tie -> even (-2.0)
    (0x01, "FINT",   "-2.5",  "0.0", 2), # toward -inf -> -3.0 (away from zero)
    (0x01, "FINT",   "-2.5",  "0.0", 3), # toward +inf -> -2.0 (toward zero)
    (0x01, "FINT",   "0.75",  "0.0", 0), # nearest, not a tie -> 1.0
    (0x01, "FINT",   "-0.75", "0.0", 0), # nearest, not a tie -> -1.0
    (0x01, "FINT",   "8.0",   "0.0", 0), # already integral -> unchanged
    (0x01, "FINT",   "0.0",   "0.0", 0),
    (0x03, "FINTRZ", "2.5",   "0.0", 0), # always truncates regardless of round field -> 2.0
    (0x03, "FINTRZ", "-2.5",  "0.0", 0), # -> -2.0
    (0x03, "FINTRZ", "0.75",  "0.0", 0), # -> 0.0
    # FINTRZ(-0.75) deliberately excluded: Musashi's own FINT/FINTRZ
    # (m68kfpu.c) round-trip through a plain sint32 intermediate
    # (floatx80_to_int32[_round_to_zero] then int32_to_floatx80) --
    # confirmed by direct inspection, not assumed -- and a plain 32-bit
    # integer has no negative-zero representation at all, so ANY
    # negative source that truncates to zero necessarily loses its sign
    # there and comes back +0.0, regardless of what real hardware does.
    # This project's own RTL never goes through an integer intermediate
    # (direct bit manipulation throughout) and preserves the sign, per
    # the standard round-toward-zero convention (IEEE 754-2008's own
    # roundToIntegralTowardZero explicitly preserves the sign of a
    # zero result). Tested directly in tb/m68882_apu_tb.sv instead.
    (0x03, "FINTRZ", "8.0",   "0.0", 0), # already integral -> unchanged
    # FGETEXP: POSITIVE, nonzero sources only -- a real, confirmed Musashi
    # bug (tools/musashi/m68kfpu.c's own FGETEXP case, `temp =
    # source.high` reads the sign bit into the exponent value UNMASKED,
    # corrupting the result for any negative or zero source; confirmed by
    # direct inspection of that one line, not assumed) means this vector
    # battery can only cross-check the cases where that bug doesn't
    # trigger. Negative/zero-source FGETEXP is instead tested directly in
    # tb/m68882_apu_tb.sv with a hand-derived expected value -- see that
    # file's own comment for the full writeup.
    (0x1E, "FGETEXP", "8.0",  "0.0", 0), # 8.0 = 1.0*2^3 -> 3.0
    (0x1E, "FGETEXP", "0.75", "0.0", 0), # 0.75 = 1.5*2^-1 -> -1.0
    (0x1E, "FGETEXP", "1.0",  "0.0", 0), # 1.0 = 1.0*2^0 -> 0.0
    # Phase 9c: FSGLDIV/FSGLMUL (single-precision-rounded, round=0/nearest
    # only -- Musashi's own C-cast-based rounding for these two doesn't
    # respect FPCR's rounding mode at all, so it's only a valid reference
    # at round=0 where both sides agree regardless) and FMOD/FREM (result
    # only -- the FPSR quotient byte isn't captured by this vector format
    # at all, checked directly in tb/m68882_apu_tb.sv instead).
    (0x24, "FSGLDIV", "2.0", "3.0", 0),  # 3.0 / 2.0 == 1.5, exact in single precision
    (0x27, "FSGLMUL", "2.0", "3.0", 0),  # 2.0 * 3.0 == 6.0
    # FMOD(2.0, 7.0) deliberately excluded: a THIRD real, confirmed
    # Musashi bug (m68kfpu.c's own `case 0x21: FMOD` calls `floatx80_rem`
    # -- the IDENTICAL softfloat function FREM's own `case 0x25` calls --
    # so Musashi's FMOD never actually truncates the quotient at all; it
    # is silently just an alias for FREM. Confirmed empirically: Musashi
    # returns -1.0 for BOTH FMOD(2.0,7.0) and FREM(2.0,7.0), when the
    # manual's own documented distinction (FMOD truncates the quotient
    # toward zero, FREM rounds it to nearest) requires them to differ
    # here (N=trunc(3.5)=3 for FMOD vs N=round(3.5)=4 for FREM, giving
    # +1.0 vs -1.0 respectively) -- this project's own RTL is tested
    # directly in tb/m68882_apu_tb.sv instead, including the real FPSR
    # quotient-byte values Musashi's own results don't expose at all
    # through this vector format anyway.
    (0x25, "FREM",    "2.0", "7.0", 0),  # N=round(7/2)=round(3.5)=4 (ties to even) -> 7-2*4=-1.0
    (0x21, "FMOD",    "3.0", "10.0", 0), # N=trunc(10/3)=3 -> 10-3*3=1.0 (agrees with FREM here, doesn't distinguish the bug)
    (0x25, "FREM",    "3.0", "10.0", 0), # N=round(10/3)=round(3.33)=3 -> 10-3*3=1.0
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
