`timescale 1ns/1ps
`default_nettype none

// Phase 1 smoke test: drive m68882_top's own pins directly (no host CPU
// involved -- MH030's own convention for a first-cut BIU test, e.g.
// tb/biu_tb.sv) and confirm:
//   1. A read cycle (CS#+AS#+DS# asserted, R/W=1) is recognized and
//      DSACK0#/DSACK1# assert with the correct (Phase 1 placeholder)
//      16-bit-port polarity, then negate once AS#/DS#/CS# are released.
//   2. A write cycle (R/W=0) is recognized the same way.
//   3. Latency from driving the pins to DSACK0# asserting is bounded and
//      consistent with a 2-stage synchronizer plus one detect register
//      (m68882_sync + m68882_biu's own cyc_seen), not immediate and not
//      unbounded.
//
// This is deliberately a raw pin-level drive, not a real host-CPU bus
// sequence (Figures 10-7/10-8's own S0-S5 progression is a HOST-side
// property -- see m68882_biu.sv's own header comment) -- Phase 1's own
// scope is "this chip responds correctly to a recognized cycle," not
// "a real host produces the full manual-shaped S-state waveform," which
// is Phase 2's job once timing_diagrams/ exists for this project too.

module m68882_biu_smoke_tb;

    logic clk_4x = 1'b0;
    logic rst_n  = 1'b0;

    logic [4:0]  a      = 5'h00;
    wire  [31:0] d;
    logic        size_n = 1'b1;
    logic        as_n   = 1'b1;
    logic        cs_n   = 1'b1;
    logic        rw     = 1'b1;
    logic        ds_n   = 1'b1;
    wire         dsack0_n;
    wire         dsack1_n;
    wire         sense_n;

    // 100 MHz internal clk_4x (mirrors MH030's own "4x the external bus
    // frequency" convention, e.g. 100 MHz internal for a 25 MHz bus) --
    // this chip's own independent clock domain, unrelated to any host.
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

    // Drive a cycle, wait for DSACK0# to assert, check it asserts within
    // a small bounded latency window (not immediately, not never), then
    // release the cycle and check DSACK0# negates.
    task automatic run_cycle(input logic is_write, input string label);
        int wait_ticks;

        @(posedge clk_4x);
        rw   = !is_write;
        a    = 5'h00;
        cs_n = 1'b0;
        as_n = 1'b0;
        ds_n = 1'b0;

        wait_ticks = 0;
        while (dsack0_n && wait_ticks < 20) begin
            @(posedge clk_4x);
            wait_ticks++;
        end

        check(!dsack0_n, {label, ": DSACK0# asserted"});
        check(dsack1_n,  {label, ": DSACK1# stayed negated (16-bit placeholder polarity)"});
        check(wait_ticks >= 2 && wait_ticks <= 6,
              {label, ": DSACK0# assert latency bounded (2-stage sync + detect register)"});

        @(posedge clk_4x);
        cs_n = 1'b1;
        as_n = 1'b1;
        ds_n = 1'b1;

        wait_ticks = 0;
        while (!dsack0_n && wait_ticks < 20) begin
            @(posedge clk_4x);
            wait_ticks++;
        end

        check(dsack0_n, {label, ": DSACK0# negated after AS#/DS#/CS# released"});
    endtask

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        run_cycle(1'b0, "READ");
        repeat (4) @(posedge clk_4x);
        run_cycle(1'b1, "WRITE");

        repeat (2) @(posedge clk_4x);
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
