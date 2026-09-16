# Three bugs found in Musashi's FPU emulation (`m68kfpu.c`)

Found while building an independent MC68881/2 FPU verification harness
(`tools/musashi_fpu_ref.c` in this repo, https://github.com/harrowm/mh882)
that cross-checks a from-scratch cycle-accurate MC68882 RTL implementation
against Musashi's own softfloat-based FPU emulation. All three bugs were
confirmed by direct inspection of the source (not just inferred from a
mismatching test result), and all three are still present in the copy of
Musashi vendored into this repo at `tools/musashi/m68kfpu.c` (printed
version "4.10" per `tools/musashi/readme.txt`, though the file's own
license-change note dated 2013 suggests this snapshot is newer than that
literal version string — no separate version constant was found in the
source to cite more precisely).

Upstream repository: https://github.com/kstenerud/Musashi

---

## Bug 1: `FGETEXP` reads the sign bit into the exponent value unmasked

**File:** `m68kfpu.c`, function `fpgen_rm_reg()`, `case 0x1e` (the FGETEXP
opcode)

```c
case 0x1e:		// FGETEXP
{
    sint16 temp;
    temp = source.high;	// get the exponent
    temp -= 0x3fff;	// take off the bias
    REG_FP[dst] = double_to_fx80((double)temp);
    SET_CONDITION_CODES(REG_FP[dst]);
    USE_CYCLES(6);
    break;
}
```

`floatx80.high` (the softfloat `floatx80` struct's own 16-bit field) packs
**both** the sign bit (bit 15) and the 15-bit biased exponent (bits 14-0)
together. This code reads the whole 16-bit field into a signed `sint16`
without masking the sign bit off first. For any operand with the sign bit
set — i.e. any **negative** source, or a **zero** source with the sign bit
happening to be part of that pattern — `temp` is corrupted by the sign bit
before the bias is even subtracted, producing a garbage result instead of
the correct exponent.

FGETEXP is specified to return "the exponent of the source operand" as a
floating-point value, independent of the source operand's own sign — the
exponent's sign reflects whether the *exponent itself* is negative or
positive, not the sign of the mantissa. A negative source with a positive
exponent (e.g. -4.0 = -1.0 × 2³) must return a *positive* result (+3.0 in
that example — sign of the result tracks the exponent's own sign only).

### Repro

Using this repo's own harness (`tools/musashi_fpu_ref`, which drives
Musashi through a real assembled `FGETEXP` F-line opcode, not a
hand-picked internal function call):

| Source | Expected result | Musashi's actual result |
|---|---|---|
| `-4.0` (`high=0xc001`) | `+2.0` (`4000 c000...`) | `c00d fffc0000...` (garbage) |
| `+0.0` (`high=0x0000`) | `+0.0` | `c00c fffc0000...` (garbage) |
| `+8.0` (`high=0x4002`, no sign bit) | `+3.0` | `+3.0` (correct — no sign bit set, bug doesn't trigger) |

The bug only manifests when `source.high`'s own bit 15 (the sign bit) is
set, which is exactly why it went unnoticed for positive-source testing.

### Suggested fix

Mask the sign bit off before computing the exponent, e.g.:

```c
sint16 temp;
temp = source.high & 0x7fff;	// exponent field only, sign bit masked off
temp -= 0x3fff;			// take off the bias
```

---

## Bug 2: `FINT`/`FINTRZ` lose the sign of a zero result

**File:** `m68kfpu.c`, function `fpgen_rm_reg()`, `case 0x01` (FINT) and
`case 0x03` (FINTRZ)

```c
case 0x01:		// Fsint
{
    sint32 temp;
    temp = floatx80_to_int32(source);
    REG_FP[dst] = int32_to_floatx80(temp);
    SET_CONDITION_CODES(REG_FP[dst]);  // JFF needs update condition codes
    break;
}
case 0x03:		// FsintRZ
{
    sint32 temp;
    temp = floatx80_to_int32_round_to_zero(source);
    REG_FP[dst] = int32_to_floatx80(temp);
    SET_CONDITION_CODES(REG_FP[dst]);
    break;
}
```

Both implementations round-trip the source through a plain **`sint32`**
intermediate value. A 32-bit two's-complement integer has exactly one
representation of zero — there is no way to distinguish "positive zero"
from "negative zero" once the value has passed through `temp`. Any
negative source operand whose rounded/truncated integer value is exactly
0 therefore always comes back as **+0.0** (`int32_to_floatx80(0)` has no
way to know the original sign), regardless of what a real MC68881/68882
would produce.

Per IEEE 754-2008 §5.9 (`roundToIntegralTowardZero` and friends), rounding
a small negative value toward zero is specified to produce a result whose
sign matches the operand's sign — i.e. a negative operand in `(-1, 0)`
truncated to an integer must yield **-0.0**, not +0.0. This is standard,
widely-implemented FPU behavior (x87, ARM, etc. all preserve the sign in
this case), not an edge case unique to the 68881/2.

### Repro

| Instruction | Source | Expected result | Musashi's actual result |
|---|---|---|---|
| `FINTRZ` | `-0.75` (`high=0xbffe`) | `-0.0` (`high=0x8000`) | `+0.0` (`high=0x0000`) — sign lost |

Musashi's own condition-code computation (`SET_CONDITION_CODES`) still
correctly reports the Z flag either way, since +0.0 and -0.0 are both
"zero" for comparison purposes — only the *sign bit of the stored result*
is wrong.

### Suggested fix

Special-case a zero result and restore the original operand's sign
directly on the `floatx80`, rather than relying on `int32_to_floatx80` to
preserve information a plain `int32` cannot carry, e.g.:

```c
case 0x01:		// Fsint
{
    sint32 temp;
    temp = floatx80_to_int32(source);
    REG_FP[dst] = int32_to_floatx80(temp);
    if (temp == 0) {
        REG_FP[dst].high = source.high & 0x8000; // restore sign, exponent/mantissa already 0
    }
    SET_CONDITION_CODES(REG_FP[dst]);
    break;
}
```

(and the same pattern for `FsintRZ`, `case 0x03`).

---

## Bug 3: `FMOD` is silently an alias for `FREM` — the quotient is never truncated

**File:** `m68kfpu.c`, function `fpgen_rm_reg()`, `case 0x21` (FMOD) and
`case 0x25` (FREM)

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
with identical arguments. Per the MC68881/MC68882 User's Manual (Section
4, the FMOD and FREM instruction descriptions), these are two genuinely
different operations that only share their general shape
(`result = FPn - (Source × N)`):

- **FMOD**: `N = INT(FPn ÷ Source)` in the **round-to-zero** mode
  (truncated quotient).
- **FREM**: `N = INT(FPn ÷ Source)` in the **round-to-nearest** mode (the
  IEEE 754 remainder).

The manual explicitly calls this out as the whole point of having both
instructions ("this function is not the same as the FREM instruction,
which uses the round-to-nearest mode"). Since Musashi's own `floatx80_rem`
always uses round-to-nearest internally, `FMOD` never actually truncates
— it always behaves exactly like `FREM`.

### Repro

`FMOD(2.0, 7.0)` and `FREM(2.0, 7.0)` (source=2.0, destination=7.0 — i.e.
7.0 mod 2.0): the true quotient is 3.5, exactly on a rounding boundary,
so truncation (FMOD) and round-to-nearest-even (FREM) genuinely diverge:
`N=3` for FMOD, `N=4` for FREM.

| Instruction | Expected `N` | Expected result | Musashi's actual result |
|---|---|---|---|
| `FMOD` | 3 (truncated) | `7.0 − 2.0×3 = +1.0` | `−1.0` (used `N=4`, i.e. ran FREM's own logic) |
| `FREM` | 4 (round-nearest-even) | `7.0 − 2.0×4 = −1.0` | `−1.0` (correct) |

Musashi's own `FMOD` result is identical to its own `FREM` result for
every input, which is only "correct" by accident on operands where
truncation and round-to-nearest happen to agree (e.g. `10.0 mod 3.0`,
where both give `N=3`) — this project's own vector battery originally
included exactly such a coincidentally-agreeing case, which is why this
bug wasn't obvious from a single spot-check.

### Suggested fix

Give `FMOD` its own round-to-zero remainder computation instead of
reusing `floatx80_rem`, e.g. compute the quotient via `floatx80_div`,
truncate it (`floatx80_to_int32_round_to_zero`-style truncation, but
without collapsing to a 32-bit intermediate — see Bug 2 above for why
that loses information), and subtract `Source × N` from `FPn` directly,
mirroring what `floatx80_rem`'s own internals almost certainly already do
for the round-to-nearest case.

---

## How these were found

All three were found via `tools/musashi_fpu_ref.c` in this repo — a
harness that hand-assembles a real F-line MC68881/2 general-instruction-
format opcode (the same command-word encoding the real chip uses) and
runs it through Musashi's own `m68k_execute()` instruction-decode path,
then compares the result against an independently-built cycle-accurate
MC68882 RTL implementation (`https://github.com/harrowm/mh882`). All
three bugs were confirmed by reading the relevant `m68kfpu.c` source
directly, not just inferred from a test mismatch, before being ruled out
as bugs in the RTL side instead.
