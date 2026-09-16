`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// MC68882 CIR register storage -- Control/Operand-Address only.
// Response, Command, Condition, Operand, and Register Select moved to
// m68882_proto.sv (Phase 3); Save/Restore moved there too (Phase 5).
// Instruction Address moved there too (Phase 6): mandatory-dispatch
// dialog logic needs to genuinely CONSUME the write (capture the
// per-instruction address, clear the "still waiting for it" dialog
// state) rather than treat it as dead storage nothing ever reads --
// this module keeps only the registers with no real dialog behind them.
//
// Correct per-register width and the real D16-D31 data-lane placement
// (Section 9.8's own explicit note: every 16-bit CIR always lives on
// D16-D31, regardless of port size or the register's own odd/even
// address). 32-bit CIRs (Operand-Address) span the full D31-D0.
//
// Scope boundary, matching m68882_cir_pkg.sv's own documented boundary:
// 8-bit-port operation and a 16-bit port accessing a 32-bit CIR both need
// a genuine multi-cycle byte-sequencing state machine, not implemented
// here -- this project's real use cases (the companion MH030 68030
// project, or any reasonable third-party 68030/68020 host) are all
// 32-bit-bus systems.
//
// Control CIR (Section 7.5.4/plan.md's own Phase 0 research): the 68881
// treats ANY Control CIR write as an unconditional total abort. The
// 68882 genuinely distinguishes two separate operations packed into the
// same 16-bit register:
//   - AB (Abort): terminates whatever dialog is currently in an "abort
//     window" (i.e. genuinely in progress) -- a no-op if the FPCP is
//     already idle, since there is nothing to abort. (The "only within
//     an abort window" gating is enforced by m68882_proto.sv itself,
//     which already only ever has something abortable while state_r !=
//     ST_IDLE -- this module just forwards the raw pulse.)
//   - XA (eXception Acknowledge): the main processor's exception handler
//     acknowledging a reported FP exception. Section 7.5.4.2, confirmed
//     directly and already documented in plan.md: "the write-exception-
//     acknowledge operation does NOT itself cause a null primitive --
//     only the exception handler's own FSAVE changes the primitive back
//     to null." XA is therefore modeled here as a pure pass-through
//     pulse to m68882_proto.sv with NO effect on dialog/primitive state
//     of its own -- it exists for a real host to assert (matching real
//     protocol shape) and for m68882_proto.sv's own FSAVE-clears-the-
//     primitive logic to be tested against, not to itself clear
//     anything.
//
// Bit assignment within the 16-bit Control CIR field (d_in[17:16] of the
// 32-bit bus write, since every 16-bit CIR lives on D16-D31): AB=bit0,
// XA=bit1. THIS PROJECT'S OWN internal convention (like the Response
// Primitive payload encoding in m68882_cir_pkg.sv) -- the manual's own
// real bit-level Control CIR encoding was not located in this project's
// extraction; only the AB-vs-XA semantic DISTINCTION itself (Section
// 7.5.4) is confirmed, not its bit position.

module m68882_cir (
    input  logic      clk_4x,
    input  logic      rst_n,

    input  logic      cyc_ack,     // this cycle's DSACK has asserted
    input  logic      cyc_write,
    input  cir_sel_t  cyc_sel,

    input  logic [31:0] d_in,
    output logic [31:0] d_out,
    output logic         d_oe,       // 1 while this module should drive d_out
    output logic         abort,      // one-tick pulse on a Control CIR write with AB set
    output logic         xa_pulse    // one-tick pulse on a Control CIR write with XA set
);

    logic [31:0] opndaddr_r;

    wire write_strobe = cyc_ack && cyc_write;
    wire read_strobe  = cyc_ack && !cyc_write;

    // Phase 2's own write-pulse finding (see git history / CLAUDE.md):
    // write_strobe is a LEVEL that stays asserted a few extra clk_4x
    // ticks past when a real host releases the data bus (synchronizer
    // latency) -- edge-detect it so every write latches exactly once.
    logic write_strobe_prev_r;
    always_ff @(posedge clk_4x or negedge rst_n)
        if (!rst_n) write_strobe_prev_r <= 1'b0;
        else        write_strobe_prev_r <= write_strobe;

    wire write_pulse = write_strobe && !write_strobe_prev_r;

    assign abort    = write_pulse && (cyc_sel == CIR_CONTROL) && d_in[16];
    assign xa_pulse = write_pulse && (cyc_sel == CIR_CONTROL) && d_in[17];

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            opndaddr_r  <= '0;
        end else if (write_pulse) begin
            case (cyc_sel)
                CIR_OPNDADDR:  opndaddr_r  <= d_in[31:0];
                default: ; // CIR_CONTROL handled above (AB/XA pulses only,
                           // no storage needed); CIR_INSTRADDR moved to
                           // m68882_proto.sv
            endcase
        end
    end

    always_comb begin
        d_out = 32'h0000_0000;
        d_oe  = 1'b0;
        case (cyc_sel)
            CIR_OPNDADDR:  begin d_oe = read_strobe; d_out = opndaddr_r; end // R/W but
                                                    // dead/unimplemented on real
                                                    // silicon -- modeled as plain
                                                    // storage anyway, never actually
                                                    // referenced by the real protocol
            default: ; // Control (write-only), Instruction-Address/Save/Restore
                       // (moved to m68882_proto.sv), and reserved offsets: no
                       // read path here, d_oe stays 0
        endcase
    end

endmodule
