# GitHub issue (ready to file at https://github.com/mattuna15/68881-fpga/issues)

## Title

Denormal (subnormal) results round twice — `ST_NORM_ROUND` rounds to
normal precision unconditionally, then truncates again with no sticky
bit when the result underflows

## Body

Found while doing a comparative review against an independent,
from-scratch cycle-accurate MC68882 RTL implementation
(https://github.com/harrowm/mh882), specifically checking how each
project handles gradual underflow (denormal/subnormal results) — a
spot both projects' own commit history shows is easy to get wrong.

### The bug

`src/mc68881_fp80_addsub_unit.vhd`, state `ST_NORM_ROUND`:

1. **Lines 353–407** compute the guard/round/sticky bits at the
   **normal**-precision bit position (fixed offsets selected by
   `rp_reg`: single/double/extended) and round `mant_main` to normal
   precision unconditionally — this runs regardless of whether the
   true, unrounded result is actually in the normal range or the
   denormal range.
2. **Only afterward**, lines 424–432 check `exp_var <= 0` (the result
   underflows into denormal range) and, if so, right-shift the
   **already-rounded** `mant_main` by `denorm_shift` bits:

   ```vhdl
   elsif exp_var <= 0 then
     denorm_shift := 1 - exp_var;
     if denorm_shift >= FP_MANT_WIDTH or mant_main = 0 then
       result_reg <= (others => '0');
     else
       result_reg(FP_WIDTH-1) <= res_sign_reg;
       result_reg(FP_WIDTH-2 downto FP_WIDTH-1-FP_EXP_WIDTH) <= (others => '0');
       result_reg(FP_MANT_WIDTH-1 downto 0) <= std_logic_vector(shift_right(mant_main, denorm_shift));
     end if;
   ```

   This final shift uses plain `shift_right` — not
   `shift_right_with_sticky`, which this same file already uses
   elsewhere for every other shift that needs to preserve rounding
   information (e.g. the mantissa-alignment shifts at lines 261, 265,
   and 292). No new guard/round/sticky decision is made at the
   denormal bit position; whatever bits fall off in this second shift
   are simply discarded.

This is textbook double rounding: rounding once to normal precision,
then truncating again to denormal precision, can produce a different
(wrong) result than rounding directly from the full-precision
intermediate value straight to the final denormal precision in one
step. The fix is well-known and even already implemented once in this
same codebase's pattern for the *first* rounding stage (accumulate a
combined sticky bit across everything being discarded, then round
once, at the position of the *final* stored precision) — it just needs
to happen once, at the denormal position, not twice.

### Repro (bit-exact, hand-verifiable)

The real 64-bit mantissa / 4-GRS-bit case is tedious to trace by hand,
so here's the identical algorithm — same guard/round/sticky extraction,
same round-to-nearest-even rule (`increment if guard AND (round OR
sticky OR kept_LSB)`, exactly matching lines 372–381's `FP_RND_NEAREST`
case), same two-stage shift — reduced to a 4-bit kept mantissa + 4 GRS
bits (1 guard + 1 round + 2 sticky-source bits), with `denorm_shift=1`.
Verified in Python against both the buggy two-stage algorithm and a
correct single-shift-then-round algorithm; **60 of the 256 possible
8-bit inputs diverge** — this is not a rare corner case.

Concrete instance: `mant_ext = 0001_0001` (kept=`0001`, guard=`0`,
round=`0`, sticky-source=`01`→sticky=`1`).

- **Stage 1 (normal-precision round):** guard=`0`, so no increment
  regardless of round/sticky (matches the code's own `if guard = '1'
  and (...)` gate) → `mant_main` stays `0001`.
- **Buggy stage 2:** `shift_right(0001, 1)` = `0000`. **Result: flushed
  to zero.**
- **Correct (shift the original value once, re-decide rounding at the
  true final position):** shifting the full 8-bit value right by 1
  gives kept=`0000`, guard=`1`, round=`0`, and — critically — the bit
  shifted off the bottom (the original sticky-source bit that was
  `1`) folds into the new sticky, giving sticky=`1`. Increment
  condition (guard=1 AND sticky=1) is now true → result = `0000+1` =
  `0001`. **Correct result: the smallest representable denormal, not
  zero.**

So for this input, the current code returns an incorrectly
flushed-to-zero result where IEEE 754 round-to-nearest-even requires
the smallest denormal. This can matter beyond the numeric value itself
— e.g. it can affect whether the correct "inexact"/"underflow"
exception behavior is signaled versus a spurious exact-zero result,
and any guest code that branches on "is this operation's result
exactly zero" gets a wrong answer.

### Suggested fix

Don't round twice. Two ways to do it, in order of how much they
disturb the existing pipeline stage boundaries:

1. **Minimal**: at line 431, instead of a plain `shift_right`, compute
   a combined sticky bit from every bit that gets shifted out of
   `mant_main` by `denorm_shift` (OR them together, same idea as
   `shift_right_with_sticky` already used elsewhere in this file), and
   redo the round-to-nearest-even increment decision at the new,
   post-shift position using that combined sticky — instead of trusting
   the stage-1 rounding decision, which was made at the wrong
   (normal-precision) bit position for an underflowing result.
2. **More thorough**: determine `exp_var <= 0` (denormal vs. normal)
   *before* the single guard/round/sticky extraction at lines 353–368,
   and extract guard/round/sticky at the *correct* final bit position
   (normal or `denorm_shift`-adjusted) in one pass, so there's only
   ever one rounding decision, made at the position the result will
   actually be stored at — mirroring how MH882's own
   `m68882_apu.sv:360-402` handles this (pre-normalize/determine the
   target position first, then round once).

### Version

Checked against the `main` branch as of 2026-09-19.

---

*Found during a comparative review that also independently
cross-validated several other parts of this project against MH882 with
no discrepancies found: denormal input classification (both projects
agree on the `exp=0`-with-explicit-integer-bit-set edge case), all 32
FP condition predicates, and FDIV operand order. Also worth a look: this
project's own packed-decimal unit
(`src/mc68881_packed_decimal_unit.vhd`) implements F-format (k≤0)
significant-digit selection, which MH882 currently documents as a
deliberately deferred gap — nice piece of independent reference
material, not a bug report item.*
