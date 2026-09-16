# GitHub issue (ready to file at https://github.com/kstenerud/Musashi/issues)

## Title

FMOD (opcode $21) is silently an alias for FREM — the quotient is never truncated

## Body

Found while cross-checking an independent, from-scratch cycle-accurate
MC68882 RTL implementation (https://github.com/harrowm/mh882) against
Musashi's own FPU emulation, via a harness that hand-assembles a real
F-line MC68881/2 opcode and runs it through `m68k_execute()` directly
(not a hand-picked internal function call).

### The bug

`fpgen_rm_reg()` in `m68kfpu.c`:

```c
case 0x21:		// FMOD
{
    REG_FP[dst] = floatx80_rem(REG_FP[dst], source);
    SET_CONDITION_CODES(REG_FP[dst]);
    USE_CYCLES(43);
    break;
}
...
case 0x25:		// FREM
{
    REG_FP[dst] = floatx80_rem(REG_FP[dst], source);
    SET_CONDITION_CODES(REG_FP[dst]);
    break;
}
```

Both opcodes call the **identical** `floatx80_rem()` softfloat function
with identical arguments — `FMOD` has no implementation of its own at
all.

Per the MC68881/MC68882 User's Manual (Section 4, the FMOD and FREM
instruction descriptions), these are two genuinely different operations
that only share the general shape `result = FPn - (Source × N)`:

- **FMOD**: `N = INT(FPn ÷ Source)` in the **round-to-zero** mode
  (truncated quotient).
- **FREM**: `N = INT(FPn ÷ Source)` in the **round-to-nearest** mode
  (the IEEE 754 remainder).

The manual explicitly calls out this distinction as the whole reason
both instructions exist ("this function is not the same as the FREM
instruction, which uses the round-to-nearest mode"). Since Musashi's own
`floatx80_rem` always rounds to nearest internally, `FMOD` never
actually truncates — it always behaves exactly like `FREM`.

### Repro

`FMOD(2.0, 7.0)` and `FREM(2.0, 7.0)` (source = 2.0, destination = 7.0,
i.e. "7.0 mod 2.0"): the true quotient is 3.5, exactly on a rounding
boundary, so truncation (FMOD) and round-to-nearest-even (FREM)
genuinely diverge — `N=3` for FMOD, `N=4` for FREM.

| Instruction | Expected `N` | Expected result | Musashi's actual result |
|---|---|---|---|
| FMOD | 3 (truncated) | `7.0 − 2.0×3 = +1.0` | `−1.0` (used `N=4`, i.e. ran FREM's own logic) |
| FREM | 4 (round-nearest-even) | `7.0 − 2.0×4 = −1.0` | `−1.0` (correct) |

Musashi's own `FMOD` result is identical to its own `FREM` result for
*every* input — it's only "correct" by coincidence on operands where
truncation and round-to-nearest happen to agree (e.g. `10.0 mod 3.0`,
where both give `N=3`), which is why this is easy to miss with a single
spot-check test.

### Suggested fix

Give `FMOD` its own round-to-zero remainder computation instead of
reusing `floatx80_rem` wholesale — e.g. compute the quotient via
`floatx80_div`, truncate it toward zero (note: `floatx80_to_int32_
round_to_zero` followed by `int32_to_floatx80` would reintroduce the
separate negative-zero bug described in
https://github.com/harrowm/mh882/blob/main/docs/musashi_bugs_found.md,
bug 2 — the truncated quotient here should stay a `floatx80` throughout,
never collapse through a plain `int32`), and subtract `Source × N` from
`FPn` directly — mirroring whatever `floatx80_rem`'s own internals
already do for the round-to-nearest case, just with the rounding mode
on the quotient itself changed to round-to-zero.

### Version

Vendored copy's own printed version "4.10" per `readme.txt`, though the
file's own license-change note dated 2013 suggests this is a later
snapshot than that literal version string (no separate version constant
found in the source to cite more precisely).

---

*Related: this is the third of three FPU-emulation bugs found during the
same cross-check — see
https://github.com/harrowm/mh882/blob/main/docs/musashi_bugs_found.md
for the other two (FGETEXP's unmasked sign bit, FINT/FINTRZ losing the
sign of a zero result).*
