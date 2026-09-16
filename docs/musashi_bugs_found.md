# Two bugs found in Musashi's FPU emulation (`m68kfpu.c`)

Found while building an independent MC68881/2 FPU verification harness
(`tools/musashi_fpu_ref.c` in this repo, https://github.com/harrowm/mh882)
that cross-checks a from-scratch cycle-accurate MC68882 RTL implementation
against Musashi's own softfloat-based FPU emulation. Both bugs were
confirmed by direct inspection of the source (not just inferred from a
mismatching test result), and both are still present in the copy of
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

## How these were found

Both were found via `tools/musashi_fpu_ref.c` in this repo — a harness
that hand-assembles a real F-line MC68881/2 general-instruction-format
opcode (the same command-word encoding the real chip uses) and runs it
through Musashi's own `m68k_execute()` instruction-decode path, then
compares the result against an independently-built cycle-accurate MC68882
RTL implementation (`https://github.com/harrowm/mh882`). Both bugs were
confirmed by reading the relevant `m68kfpu.c` source directly, not just
inferred from a test mismatch, before being ruled out as bugs in the RTL
side instead.
