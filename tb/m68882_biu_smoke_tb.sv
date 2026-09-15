`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// Phase 2 smoke test: drive m68882_top's own pins directly (no host CPU
// involved -- mirrors MH030's own tb/biu_tb.sv convention for a first-cut
// BIU test). Confirms:
//   1. Synchronous read (Response CIR, $00) takes genuinely longer than
//      an asynchronous read/write (Section 10.4.1 vs 10.4.2/10.4.3) --
//      the core cycle-type distinction Phase 2 exists to get right.
//   2. DSACK0#/DSACK1# polarity matches Table 9-3 across all 3 port
//      sizes and both CIR-width halves (A4=0/1) -- this is the table
//      whose correct reading found and fixed Phase 1's own placeholder
//      bug (see m68882_cir_pkg.sv's own header comment).
//   3. The Section 10.5 inter-cycle busy timer genuinely delays a
//      subsequent asynchronous access issued right after a Control CIR
//      write, instead of acking at the normal fast async latency.
//   4. The CIR register file (m68882_cir.sv) actually stores and returns
//      correct values, on the correct D16-D31/D31-D0 data lanes.

module m68882_biu_smoke_tb;

    logic clk_4x = 1'b0;
    logic rst_n  = 1'b0;

    logic [4:0]  a      = 5'h00;
    logic        size_n = 1'b1;
    logic        as_n   = 1'b1;
    logic        cs_n   = 1'b1;
    logic        rw     = 1'b1;
    logic        ds_n   = 1'b1;
    wire         dsack0_n;
    wire         dsack1_n;
    wire         sense_n;

    // Host-side data-bus drive: only driven during a write, released
    // (high-Z) during a read so the DUT itself can drive it.
    logic [31:0] d_drv    = 32'h0;
    logic        d_drv_en = 1'b0;
    wire  [31:0] d = d_drv_en ? d_drv : 32'bz;

    // 100 MHz internal clk_4x (mirrors MH030's own "4x the external bus
    // frequency" convention) -- this chip's own independent clock domain.
    always #5 clk_4x = ~clk_4x;

    m68882_top u_top (
        .clk_4x   (clk_4x),
        .rst_n    (rst_n),
        .a        (a),
        .d        (d),
        .size_n   (size_n),
        .as_n     (as_n),
        .cs_n     (cs_n),
        .rw       (rw),
        .ds_n     (ds_n),
        .dsack0_n (dsack0_n),
        .dsack1_n (dsack1_n),
        .sense_n  (sense_n)
    );

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input logic cond, input string msg);
        if (cond) begin
            pass_count++;
            $display("PASS: %s", msg);
        end else begin
            fail_count++;
            $display("FAIL: %s", msg);
        end
    endtask

    // Drive one bus cycle to CIR offset `sel_a4_a1` (the A4-A1 field),
    // with the given port-size straps, direction, and (for writes) data.
    // Returns the tick latency to DSACK0# assertion and the DSACK
    // polarity observed, and (for reads) the data sampled at ack.
    task automatic run_cycle(
        input  logic [3:0] sel_a4_a1,
        input  logic       is_write,
        input  logic       size_n_val,
        input  logic       a0_val,
        input  logic [31:0] wdata,
        output int          latency_ticks,
        output logic [1:0]  dsack_seen,
        output logic [31:0] rdata
    );
        @(posedge clk_4x);
        a      = {sel_a4_a1, a0_val};
        size_n = size_n_val;
        rw     = !is_write;
        if (is_write) begin
            d_drv_en = 1'b1;
            d_drv    = wdata;
        end else begin
            d_drv_en = 1'b0;
        end
        cs_n = 1'b0;
        as_n = 1'b0;
        ds_n = 1'b0;

        latency_ticks = 0;
        while (dsack0_n && dsack1_n && latency_ticks < 60) begin
            @(posedge clk_4x);
            latency_ticks++;
        end

        dsack_seen = {dsack1_n, dsack0_n};
        rdata = d;

        @(posedge clk_4x);
        cs_n     = 1'b1;
        as_n     = 1'b1;
        ds_n     = 1'b1;
        d_drv_en = 1'b0;
        repeat (2) @(posedge clk_4x);
    endtask

    int          lat;
    logic [1:0]  dsack;
    logic [31:0] rdata;

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        // ── 1/2: synchronous vs asynchronous read latency, 16-bit port ──
        run_cycle(CIR_RESPONSE, 1'b0, 1'b1, 1'b0, 32'h0, lat, dsack, rdata);
        $display("INFO: sync read (Response, 16-bit port) latency=%0d dsack=%b", lat, dsack);
        check(lat >= 7, "sync read (Response CIR) takes the genuine counted delay, not the fast async path");
        check(dsack == 2'b01, "sync read (Response CIR, 16-bit port): DSACK matches Table 9-3 (DSACK1#=0,DSACK0#=1)");

        run_cycle(CIR_RESTORE, 1'b0, 1'b1, 1'b0, 32'h0, lat, dsack, rdata);
        $display("INFO: async read (Restore, 16-bit port) latency=%0d dsack=%b", lat, dsack);
        check(lat <= 5, "async read (Restore CIR) uses the fast path, latency well under the sync-read delay");
        check(dsack == 2'b01, "async read (Restore CIR, 16-bit port): DSACK matches Table 9-3 (DSACK1#=0,DSACK0#=1)");

        // ── 3: Table 9-3 DSACK polarity across port sizes / A4 halves ──
        run_cycle(CIR_OPERAND, 1'b0, 1'b1, 1'b1, 32'h0, lat, dsack, rdata);
        check(dsack == 2'b00, "32-bit port, A4=1 (Operand CIR): DSACK1#=0,DSACK0#=0 (D31-D00)");

        run_cycle(CIR_RESTORE, 1'b0, 1'b1, 1'b1, 32'h0, lat, dsack, rdata);
        check(dsack == 2'b01, "32-bit port, A4=0 (Restore CIR): DSACK1#=0,DSACK0#=1 (16-bit CIR still on D16-D31)");

        run_cycle(CIR_RESTORE, 1'b0, 1'b0, 1'b0, 32'h0, lat, dsack, rdata);
        check(dsack == 2'b10, "8-bit port: DSACK1#=1,DSACK0#=0");

        // ── 4: inter-cycle busy timer (Section 10.5) ────────────────────
        run_cycle(CIR_CONTROL, 1'b1, 1'b1, 1'b0, 32'hABCD_0000, lat, dsack, rdata);
        $display("INFO: Control CIR write latency=%0d", lat);
        run_cycle(CIR_RESTORE, 1'b0, 1'b1, 1'b0, 32'h0, lat, dsack, rdata);
        $display("INFO: async read immediately after Control CIR write, latency=%0d", lat);
        check(lat >= 10, "async access right after a Control CIR write is delayed by the inter-cycle busy timer");

        repeat (20) @(posedge clk_4x); // let busy timer fully clear before the next test

        // ── 5: register storage round-trip ──────────────────────────────
        run_cycle(CIR_RESTORE, 1'b1, 1'b1, 1'b0, 32'h1234_0000, lat, dsack, rdata);
        repeat (20) @(posedge clk_4x); // let the write's own busy timer clear first
        run_cycle(CIR_RESTORE, 1'b0, 1'b1, 1'b0, 32'h0, lat, dsack, rdata);
        check(rdata[31:16] == 16'h1234, "Restore CIR write/read-back round-trips on the D16-D31 lane");

        repeat (20) @(posedge clk_4x);
        run_cycle(CIR_OPERAND, 1'b1, 1'b1, 1'b1, 32'hDEAD_BEEF, lat, dsack, rdata);
        $display("INFO: Operand write latency=%0d", lat);
        repeat (20) @(posedge clk_4x);
        run_cycle(CIR_OPERAND, 1'b0, 1'b1, 1'b1, 32'h0, lat, dsack, rdata);
        $display("INFO: Operand read-back latency=%0d rdata=%h dsack=%b", lat, rdata, dsack);
        check(rdata == 32'hDEAD_BEEF, "Operand CIR (32-bit) write/read-back round-trips on D31-D00");

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("SMOKE TEST FAILED");
            $finish;
        end
        $display("SMOKE TEST PASSED");
        $finish;
    end

endmodule
