`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// MC68882 Bus Interface Unit -- Phase 2.
//
// Implements the 3 real bus-cycle types (Section 10.4) and the Section
// 10.5 inter-cycle timing restriction, replacing Phase 1's own single
// generic async-only model and placeholder DSACK polarity (see
// m68882_cir_pkg.sv's own header for the correction this found).
//
// Cycle-type dispatch:
//  - SYNCHRONOUS READ (Response $00 / Save $04 only, Section 10.4.1):
//    DSACK asserts a fixed 1.5 "real" clocks (SYNC_DSACK_TICKS) after
//    CS#+AS#+DS# is sampled asserted -- a genuine counted delay, unlike
//    the async case. This RTL always lands on the clean 5-clock/2-wait-
//    state baseline (Figure 10-6): the manual's own "can extend to 6-7
//    clocks" variability is a real-silicon metastability/clock-edge-
//    relationship artifact this project's own fixed 2-stage synchronizer
//    does not reproduce -- a deliberate, documented simplification, not
//    an oversight (there is no principled distribution to draw an
//    artificial extension from).
//  - ASYNCHRONOUS READ/WRITE (everything else, Section 10.4.2/10.4.3):
//    Phase 1's existing fast-detect-and-ack model (one clk_4x tick past
//    the already-synchronized inputs), now correctly polarized via the
//    port-size/A4-driven dsack_encode() table.
//
// Section 10.5 inter-cycle timing restriction: a write to the Control or
// Restore CIR, or any Operand CIR access, arms a BUSY_TICKS-long internal
// delay on DS#'s negated edge; a subsequent asynchronous cycle started
// while busy has its own DSACK held off until the delay clears.
//
// Simplification, documented here and in plan.md: modeled at WORD
// granularity -- any complete write to Control/Restore triggers the
// delay, not just a byte write to the register's own low byte
// specifically (the manual's literal wording, Section 10.5 items 1-2).
// This is a safe superset, never a safe subset: a full 16-bit write
// always includes writing the low byte, so this can only trigger the
// delay in cases real silicon also would, never fewer. Narrowing it to
// the exact byte-level trigger needs real byte-lane-aware CIR writes,
// which don't exist until Phase 3's own register file lands. Likewise,
// "any Operand CIR access" approximates items 3-4 (specifically the
// LAST Operand write of a busy-frame restore / FIRST Operand read of an
// idle-or-busy-frame save) -- genuine first/last-of-frame tracking needs
// Phase 5's own FSAVE/FRESTORE state-frame logic; this is a safe
// superset for the same reason (every real trigger case is also an
// Operand CIR access).

module m68882_biu #(
    parameter int TICKS_PER_CLOCK  = 4,                     // clk_4x ticks per this
                                                             // chip's own real external
                                                             // CLK period (mirrors
                                                             // MH030's own 4x convention)
    parameter int SYNC_DSACK_TICKS = (TICKS_PER_CLOCK * 3) / 2, // 1.5 clocks
    parameter int BUSY_TICKS       = TICKS_PER_CLOCK * 4        // 4 clocks
) (
    input  logic       clk_4x,
    input  logic       rst_n,

    // Already-synchronized host bus inputs (m68882_top owns the sync stage)
    input  logic       as_n_s,
    input  logic       ds_n_s,
    input  logic       cs_n_s,
    input  logic       rw_s,
    input  logic       size_n_s,
    input  logic [4:0] a_s,

    // Pins
    output logic       dsack0_n,
    output logic       dsack1_n,

    // Internal status, consumed by later phases (CIR register file, etc.)
    output logic       cyc_active,  // a recognized CS#+AS#+DS# bus cycle is in progress
    output logic       cyc_ack,     // this cycle's DSACK has asserted (data may be latched/read)
    output logic       cyc_write,   // 1 = write cycle (rw_s low), 0 = read cycle
    output cir_sel_t   cyc_sel      // decoded A4-A1 register select, valid while cyc_active
);

    localparam int SYNC_CNT_W = (SYNC_DSACK_TICKS < 1) ? 1 : $clog2(SYNC_DSACK_TICKS + 1);
    localparam int BUSY_CNT_W = (BUSY_TICKS < 1) ? 1 : $clog2(BUSY_TICKS + 1);

    wire cir_sel_t   sel       = a_s[4:1];
    wire             a4        = a_s[4];
    wire             sync_read = is_sync_read(sel) && rw_s;
    wire port_size_t psize     = port_size(size_n_s, a_s[0]);

    wire cyc_request = !cs_n_s && !as_n_s && !ds_n_s;

    assign cyc_active = cyc_request;
    assign cyc_write  = !rw_s;
    assign cyc_sel    = sel;

    // ── Latch cycle identity at the start of each new cycle (needed by
    // the inter-cycle busy check below, which fires later at DS#'s
    // negate edge -- by then cyc_request has already gone low). ────────
    logic     cyc_seen_r;
    cir_sel_t sel_latched_r;
    logic     write_latched_r;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            cyc_seen_r      <= 1'b0;
            sel_latched_r   <= CIR_RESPONSE;
            write_latched_r <= 1'b0;
        end else if (cyc_request && !cyc_seen_r) begin
            cyc_seen_r      <= 1'b1;
            sel_latched_r   <= sel;
            write_latched_r <= !rw_s;
        end else if (!cyc_request) begin
            cyc_seen_r <= 1'b0;
        end
    end

    // ── Inter-cycle busy timer (Section 10.5) ──────────────────────────
    logic ds_n_s_prev_r;
    always_ff @(posedge clk_4x or negedge rst_n)
        if (!rst_n) ds_n_s_prev_r <= 1'b1;
        else        ds_n_s_prev_r <= ds_n_s;

    wire ds_negate_edge = ds_n_s && !ds_n_s_prev_r;

    wire trigger_busy = ds_negate_edge && cyc_seen_r &&
                        ((write_latched_r && (sel_latched_r == CIR_CONTROL)) ||
                         (write_latched_r && (sel_latched_r == CIR_RESTORE)) ||
                         (sel_latched_r == CIR_OPERAND));

    logic [BUSY_CNT_W-1:0] busy_cnt_r;
    logic                  busy_r;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            busy_cnt_r <= '0;
            busy_r     <= 1'b0;
        end else if (trigger_busy) begin
            busy_cnt_r <= BUSY_TICKS[BUSY_CNT_W-1:0];
            busy_r     <= 1'b1;
        end else if (busy_r) begin
            if (busy_cnt_r <= 1) begin
                busy_r     <= 1'b0;
                busy_cnt_r <= '0;
            end else begin
                busy_cnt_r <= busy_cnt_r - 1'b1;
            end
        end
    end

    // ── Synchronous-read counted delay (Section 10.4.1) ────────────────
    logic [SYNC_CNT_W-1:0] sync_cnt_r;
    logic                  sync_ack_r;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            sync_cnt_r <= '0;
            sync_ack_r <= 1'b0;
        end else if (!cyc_request) begin
            sync_cnt_r <= '0;
            sync_ack_r <= 1'b0;
        end else if (sync_read) begin
            if (sync_cnt_r >= SYNC_DSACK_TICKS[SYNC_CNT_W-1:0]) begin
                sync_ack_r <= 1'b1;
            end else begin
                sync_cnt_r <= sync_cnt_r + 1'b1;
            end
        end
    end

    // ── Asynchronous fast-ack (Section 10.4.2/10.4.3), gated by the
    // inter-cycle busy timer ────────────────────────────────────────────
    logic async_ack_r;
    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            async_ack_r <= 1'b0;
        end else begin
            async_ack_r <= cyc_request && !sync_read && !busy_r;
        end
    end

    wire ack = cyc_request && (sync_read ? sync_ack_r : async_ack_r);
    assign cyc_ack = ack;

    wire [1:0] dsack_n = ack ? dsack_encode(psize, a4) : 2'b11;
    assign dsack1_n = dsack_n[1];
    assign dsack0_n = dsack_n[0];

endmodule
