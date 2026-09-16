# GitHub issue (ready to file at https://github.com/kstenerud/Musashi/issues)

## Title

FSIN/FCOS/FSINCOS compute via IEEE double (53-bit) precision, not extended
(64-bit) precision — mantissa's low 11+ bits are always zero

## Body

Found while cross-checking an independent, from-scratch cycle-accurate
MC68882 RTL implementation (https://github.com/harrowm/mh882) against
Musashi's own FPU emulation, via a harness that hand-assembles a real
F-line MC68881/2 opcode and runs it through `m68k_execute()` directly
(not a hand-picked internal function call).

This is a different *kind* of finding than the other three FPU bugs
already filed against this codebase
(https://github.com/harrowm/mh882/blob/main/docs/musashi_bugs_found.md
and
https://github.com/harrowm/mh882/blob/main/docs/musashi_issue_fmod_frem.md)
— those are outright logic bugs producing a definitely-wrong answer.
This one is a genuine precision/fidelity shortfall: the *computation
path itself* discards information no real MC68881/2 discards, so no
amount of algorithm tuning downstream can recover it. Filing it
separately, and describing it plainly as a precision issue rather than
a "bug," so it's your call whether/how to treat it.

### The issue

`fpgen_rm_reg()` in `m68kfpu.c`:

```c
case 0xe:		// SIN
    REG_FP[dst] = double_to_fx80(sin(fx80_to_double(source)));
    SET_CONDITION_CODES(REG_FP[dst]); // JFF
    USE_CYCLES(400);
    break;
case 0x1d:		// COS
    REG_FP[dst] = double_to_fx80(cos(fx80_to_double(source)));
    SET_CONDITION_CODES(REG_FP[dst]); // JFF
    USE_CYCLES(400);
    break;
case 0x30: ... case 0x37:  // SINCOS
{
    double ds = fx80_to_double(source);
    REG_FP[dst] = double_to_fx80(sin(ds));
    REG_FP[opmode&7] = double_to_fx80(cos(ds));
    SET_CONDITION_CODES(REG_FP[dst]); // JFF
    USE_CYCLES(400);
    break;
}
```

All three convert the extended-precision (80-bit, 64 significant
mantissa bits) source operand down to a plain C `double` (64-bit IEEE,
53 significant mantissa bits), call libc's `sin()`/`cos()`, and convert
the `double` result back up to `floatx80`. `double_to_fx80` is a
value-preserving widen — it introduces no *additional* rounding of its
own — but the `double` it's widening from has already lost the low
64-53 = 11 bits of precision the source operand actually carried, and
libc's own `sin`/`cos` are only ever accurate to something on the order
of the `double` ULP to begin with. The result: every `floatx80` this
code produces has its low ≥11 mantissa bits forced to zero (confirmed
directly, see below), and is only ever as accurate as a `double`
computation, never as accurate as the real chip's own internal
extended-precision evaluation.

The MC68881/MC68882 User's Manual, Section 4.3 "Computational
Accuracy," documents real silicon's own transcendental accuracy as
"the worst-case accuracy is 4096 units in the last place... the
typical error bound... is approximately 64 units in the last place" —
i.e. real hardware is expected to get within roughly 64 ULP of the
true value in the *last place of a 64-bit mantissa*, not a 53-bit one.
A `double`-precision computation cannot make that promise: its own
result is already coarse by up to ~2^10 extended-precision ULP purely
from the missing bits, before any algorithmic error on top.

### Repro

Using this repo's own harness (`tools/musashi_fpu_ref`) against 8
representative `FSIN`/`FCOS` inputs, compared against independently
Python-computed reference values (60-significant-digit `Decimal`
Taylor series — not `math.sin`/`math.cos`, which is itself only
`double`-precision and too coarse a reference here):

| Input | Musashi mantissa (low 16 bits) | Trailing zero bits | ULP error vs. true value |
|---|---|---|---|
| FSIN(0.5) | `...582f8000` | 15 | 188 |
| FCOS(0.5) | `...dbea8000` | 15 | 786 |
| FSIN(1.0) | `...48677000` | 12 | 33 |
| FCOS(1.0) | `...a8346000` | 13 | 878 |
| FSIN(2.0) | `...8da23000` | 12 | 259 |
| FCOS(2.0) | `...9b902800` | 11 | 734 |
| FSIN(10.0)| `...9a7a9000` | 12 | 718 |
| FCOS(10.0)| `...358f800`  | 11 | 261 |

Every one of the 8 samples has **at least 11 trailing zero bits** — the
exact signature of a 64-bit mantissa that only ever had 53 real bits of
information to begin with (64 − 53 = 11). And **7 of the 8 samples
exceed the manual's own "~64 ULP typical" accuracy bound** — some by
more than 10×, none of the 8 comes close to `double`-precision's own
theoretical ~0.5-ULP-at-double-scale (~1024-ULP-at-extended-scale)
ceiling, but several sit close enough to it (734, 786, 878 ULP) that
the shortfall is clearly the double round-trip, not just the specific
angles chosen.

This doesn't show up as a functional failure the way the other three
bugs do — there's no single input where the result is *qualitatively*
wrong (wrong sign, wrong magnitude, aliased to the wrong instruction).
It only shows up as reduced numerical fidelity, which is why it's
filed separately and described as a precision issue rather than
grouped with the other three.

### Suggested fix

Compute `sin`/`cos` directly against the `floatx80` (or an internal
extended-precision-equivalent, e.g. `long double` on platforms where
it's genuinely 80-bit extended) rather than round-tripping through
`double`. If matching this specific codebase's own existing genuinely
extended-precision building blocks (`floatx80_add`/`floatx80_mul`/etc.
in `softfloat.c`) is preferred over pulling in a new dependency, a
Taylor-series-after-argument-reduction approach (reduce to `[-π/4,
π/4]`, evaluate a fixed-degree polynomial via Horner's method,
reconstruct by quadrant) is straightforward to implement entirely in
`floatx80` arithmetic and converges to well within the manual's own
64-ULP-typical bound in under 10 terms — this project's own RTL
implementation (`rtl/m68882_apu.sv`, `fp_sincos` task, in the repo
linked above) takes exactly this approach if a worked reference is
useful.

### Version

Vendored copy's own printed version "4.10" per `readme.txt`, though the
file's own license-change note dated 2013 suggests this is a later
snapshot than that literal version string (no separate version
constant found in the source to cite more precisely).

---

*Related: this is a separate, precision-focused finding from the three
outright logic bugs already filed —
https://github.com/harrowm/mh882/blob/main/docs/musashi_bugs_found.md
(FGETEXP's unmasked sign bit, FINT/FINTRZ losing the sign of a zero
result, FMOD/FREM aliasing) — filed on its own since it's a different
kind of issue (a real precision ceiling from the computation path
itself, not a logic defect with a single clearly-wrong answer).*
