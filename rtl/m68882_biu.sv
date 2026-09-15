`timescale 1ns/1ps
`default_nettype none

// MC68882 Bus Interface Unit -- Phase 1 skeleton.
//
// Scope (plan.md Phase 1): recognize a CS#+AS#+DS# bus-cycle boundary and
// source/sink DSACK0#/DSACK1# with correct polarity. No CIR register
// content yet (Phase 2) -- this module cannot yet tell a synchronous-read
// CIR (Response/Save, address $00/$04) apart from every other, genuinely
// asynchronous CIR, so every cycle is handled with the asynchronous model
// for now.
//
// This is NOT a placeholder shortcut for the asynchronous case itself --
// confirmed directly against the manual (Section 10.4.2/10.4.3): "the FPCP
// responds by ... asserting the appropriate data transfer and size
// acknowledge encoding" essentially as soon as CS#+AS#+DS# is detected;
// unlike the synchronous-read case, "this bus cycle timing does NOT depend
// on the clock frequency of the FPCP" -- the surrounding 3-clock/S0-S5
// duration shown in Figures 10-7/10-8 is a property of the HOST's own bus
// sequencing (address setup, then AS, then DS), not of this chip inserting
// deliberate wait states. Both figures also state DSACKx "is recognized by
// the MPU on the falling edge of S2" for READ AND WRITE alike -- unlike the
// 68030 CPU side (CLAUDE.md there: read and write S-state tables are NOT
// symmetric), this FPU's own async read/write timing genuinely is, so one
// shared detect-and-ack path below covers both directions.
//
// Phase 2 adds the genuinely different synchronous-read model (a real
// counted delay: DSACK 1.5 internal clocks after CS#+AS#+DS# is sampled,
// Section 10.4.1, Figure 10-6) once the CIR register file can tell
// Response/Save apart from every other register by address, plus the real
// per-CIR DSACK width encoding (see the placeholder note below).

module m68882_biu (
    input  logic clk_4x,
    input  logic rst_n,

    // Already-synchronized host bus inputs (m68882_top owns the sync stage)
    input  logic as_n_s,
    input  logic ds_n_s,
    input  logic cs_n_s,
    input  logic rw_s,

    // Pins
    output logic dsack0_n,
    output logic dsack1_n,

    // Internal status, consumed by later phases (CIR register file, etc.)
    output logic cyc_active,   // a recognized CS#+AS#+DS# bus cycle is in progress
    output logic cyc_write     // 1 = write cycle (rw_s low), 0 = read cycle
);

    // Registered "cycle detected" -- one clk_4x tick of response latency
    // past the already-synchronized inputs themselves. This models the
    // manual's own allowance for chip-select/strobe propagation delay
    // (Section 10.4.2/10.4.3's own closing paragraph: "This assumes that
    // the chip select logic causes the assertion of CS to precede AS and
    // DS so that the ... delay is not lengthened by the chip select logic
    // propagation time" -- i.e. some finite response latency is expected
    // and budgeted for, not a zero-delay combinational requirement).
    logic cyc_seen;

    wire cyc_request = !cs_n_s && !as_n_s && !ds_n_s;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            cyc_seen <= 1'b0;
        end else begin
            cyc_seen <= cyc_request;
        end
    end

    assign cyc_active = cyc_seen;
    assign cyc_write  = !rw_s;

    // DSACKx stays asserted from the tick after cycle detection until the
    // host negates AS# or DS# (Section 10.4.2: "...until the first of the
    // two signals, AS or DS, is negated..."; Section 10.4.3: "...until AS
    // is negated..." -- using AS-or-DS for both directions here is the
    // conservative, manual-consistent superset). Because cyc_seen is a
    // plain one-tick-delayed register of cyc_request, it naturally drops
    // the same way it rises -- one tick after the host negates either
    // strobe -- so no separate negate-edge logic is needed.
    assign dsack0_n = !cyc_seen;
    assign dsack1_n = 1'b1; // 16-bit-port placeholder -- see module header;
                            // Phase 2 replaces this with the real per-CIR
                            // width (Response/Control/Save/Restore/
                            // Operation-Word/Command/Condition/Register-
                            // Select are 16-bit; Operand/Instruction-
                            // Address/Operand-Address are 32-bit).

endmodule
