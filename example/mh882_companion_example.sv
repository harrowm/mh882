`timescale 1ns/1ps
`default_nettype none

// Phase 8: companion integration example -- demonstrates this project's
// own explicit scoping requirement ("standalone, pin-compatible... but
// architecturally independent from" the sibling MH030 68030 project)
// concretely, at the RTL level, rather than leaving it as prose.
//
// This module is NOT part of the FPU's own required scope (plan.md's
// own Phase 8 note: "not part of this project's own required scope").
// It instantiates m68882_top + the one piece of external glue any real
// 68020+ host needs (rtl/glue_cs_decode.sv -- neither this project nor
// the sibling MH030 68030 project has a dedicated coprocessor-select
// pin of its own), and exposes an external port list that IS exactly
// what a real 68030's own CPU-space coprocessor bus cycle presents
// (MC68030UM.pdf, also documented in MH030's own CLAUDE.md under "BIU
// Cycle Types"/"Coprocessor interface (FPU)"). Deliberately does NOT
// instantiate MH030's own m68030_top -- this project stays its own
// independent repository with no code dependency on that one (a real
// integrator connects this module's own port list directly to their
// own 68030, whether that's MH030's checkout, a real chip, or any other
// 68020+ implementation); a sibling checkout or git submodule remains
// the documented option for whoever pairs the two, never a hard
// dependency baked into this repo's own build.
//
// clk_4x stays this chip's OWN independent clock domain throughout, per
// this project's own standing requirement (m68882_top.sv's own header
// comment) -- never derived from or synchronized to the host's clock.
//
// Deliberately NOT attempted here (real, out-of-scope work for a
// DIFFERENT repository, not silently dropped): MH030's own CLAUDE.md
// documents cpBcc/cpDBcc/cpScc/cpTRAPcc and Coprocessor Protocol
// Violation (vector 13) as unimplemented specifically because testing
// them needs a real attached coprocessor model, which this repo now IS
// -- but actually closing that gap means modifying MH030's own RTL and
// test suite, a separate undertaking against a different repository
// this session was not asked to touch. This module is the missing
// piece such a future undertaking would attach to, not that
// undertaking itself.

module mh882_companion_example #(
    parameter logic [2:0] CPID = 3'b001
) (
    input  logic        clk_4x,     // THIS CHIP'S OWN clock -- see header comment
    input  logic         rst_n,      // RESET# -- typically shared with the host's own reset

    // ── Host-side bus, exactly the shape a real 68030 CPU Space
    // coprocessor cycle presents (FC=111, A[19:16]=0010, A[15:13]=CpID,
    // A[4:0]=CIR select) -- connect these directly to a real 68030's
    // own external pins. ─────────────────────────────────────────────
    input  logic [2:0]  host_fc,      // FC2-FC0
    input  logic [19:0] host_a,       // A19-A0 (only [19:13] used by the glue decode;
                                       // [4:0] passed straight through to the FPU)
    inout  wire  [31:0] host_d,       // D31-D0
    input  logic         host_size_n, // SIZE#
    input  logic         host_as_n,   // AS#
    input  logic         host_rw,     // R/W
    input  logic         host_ds_n,   // DS#
    output logic         host_dsack0_n,
    output logic         host_dsack1_n
);

    wire cs_n;

    glue_cs_decode #(.CPID(CPID)) u_glue (
        .fc   (host_fc),
        .a    (host_a[19:13]),
        .as_n (host_as_n),
        .cs_n (cs_n)
    );

    m68882_top u_fpu (
        .clk_4x   (clk_4x),
        .rst_n    (rst_n),
        .a        (host_a[4:0]),
        .d        (host_d),
        .size_n   (host_size_n),
        .as_n     (host_as_n),
        .cs_n     (cs_n),
        .rw       (host_rw),
        .ds_n     (host_ds_n),
        .dsack0_n (host_dsack0_n),
        .dsack1_n (host_dsack1_n),
        .sense_n  ()
    );

endmodule
