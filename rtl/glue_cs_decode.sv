`timescale 1ns/1ps
`default_nettype none

// Phase 8: companion CS# decode glue -- the one piece of external logic
// a real 68030 (or any 68020+ host) needs to attach this chip, since
// neither this project nor the sibling MH030 68030 project has a
// dedicated coprocessor-select output pin of its own (documented as a
// standing gap in both projects' own CLAUDE.md files).
//
// Confirmed directly against MC68030UM.pdf (also independently
// documented in MH030's own CLAUDE.md, "Coprocessor interface (FPU)"
// under BIU Cycle Types): a coprocessor communication bus cycle is a
// CPU Space (FC=111) access with A[19:16]=0010 (distinguishing it from
// Interrupt Acknowledge's own A[19:16]=1111 CPU-space sub-type), and
// A[15:13] selects the CpID -- which of up to 7 coprocessors, matching
// the F-line operation word's own bits[11:9]. A[4:0] (not decoded here)
// then selects a specific Coprocessor Interface Register within that
// coprocessor's own register block -- this chip's own m68882_top.sv
// already takes that directly as its `a` port.
//
// CPID is a compile-time parameter (not a runtime strap) since a real
// system wires each coprocessor's own glue instance to its own fixed
// CpID -- this project's own m68882_top has no CpID input of its own to
// match against (the F-line operation word's CpID field is entirely a
// main-processor/decode-side concept, never visible to the coprocessor
// itself beyond "was I selected").
//
// CS_n only asserts while AS_n is genuinely asserted (a real, active bus
// cycle) -- a real PAL/GAL-equivalent glue never asserts CS# from
// address/FC lines alone, since those can be driven to arbitrary
// intermediate values between bus cycles.

module glue_cs_decode #(
    parameter logic [2:0] CPID = 3'b001
) (
    input  logic [2:0]  fc,      // FC2-FC0 (function code)
    input  logic [19:13] a,      // A19-A13 (only the bits this decode needs)
    input  logic         as_n,   // AS# -- qualifies the decode to an active bus cycle
    output logic         cs_n    // to this chip's own CS# pin (m68882_top.sv's `cs_n`)
);

    wire match = (fc == 3'b111) && (a[19:16] == 4'b0010) && (a[15:13] == CPID);

    assign cs_n = !(match && !as_n);

endmodule
