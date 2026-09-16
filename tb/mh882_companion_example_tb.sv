`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// Phase 8 test: example/mh882_companion_example.sv end to end -- drives
// its own host-side port list (the exact shape a real 68030 CPU Space
// coprocessor cycle presents) directly, confirming the assembled glue+
// FPU pair actually responds to a matching CpID address and, just as
// importantly, stays completely silent (no DSACK, no CS#) for a
// non-matching one -- the real point of the glue module existing at
// all.

module mh882_companion_example_tb;

    logic clk_4x = 1'b0;
    logic rst_n  = 1'b0;

    logic [2:0]  host_fc     = 3'b111;
    logic [19:0] host_a      = 20'h0;
    logic        host_size_n = 1'b1;
    logic        host_as_n   = 1'b1;
    logic        host_rw     = 1'b1;
    logic        host_ds_n   = 1'b1;
    wire         host_dsack0_n, host_dsack1_n;

    logic [31:0] d_drv    = 32'h0;
    logic        d_drv_en = 1'b0;
    wire  [31:0] host_d = d_drv_en ? d_drv : 32'bz;

    always #5 clk_4x = ~clk_4x;

    mh882_companion_example #(.CPID(3'b001)) u_dut (
        .clk_4x, .rst_n,
        .host_fc, .host_a, .host_d, .host_size_n, .host_as_n, .host_rw, .host_ds_n,
        .host_dsack0_n, .host_dsack1_n
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

    // Drives a Response CIR (offset 0x00) read cycle at the given CpID
    // address. Returns whether DSACK ever asserted within a bounded wait.
    task automatic try_response_read(input logic [2:0] cpid, output logic acked, output logic [31:0] rdata);
        int wait_ticks;
        @(posedge clk_4x);
        host_fc     = 3'b111;
        host_a      = {4'b0010, cpid, 13'h0}; // A[19:16]=0010, A[15:13]=cpid, A[4:0]=Response CIR (0x00)
        host_rw     = 1'b1; // read
        d_drv_en    = 1'b0;
        host_as_n   = 1'b0;
        host_ds_n   = 1'b0;

        wait_ticks = 0;
        while (host_dsack0_n && host_dsack1_n && wait_ticks < 60) begin
            @(posedge clk_4x);
            wait_ticks++;
        end
        acked = !(host_dsack0_n && host_dsack1_n);
        rdata = host_d;

        @(posedge clk_4x);
        host_as_n = 1'b1;
        host_ds_n = 1'b1;
        repeat (2) @(posedge clk_4x);
    endtask

    logic acked;
    logic [31:0] rdata;

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        try_response_read(3'b001, acked, rdata); // matches this instance's own CPID=1
        check(acked, "matching CpID (1): the FPU responds with a real DSACK");

        repeat (4) @(posedge clk_4x);

        try_response_read(3'b010, acked, rdata); // a DIFFERENT coprocessor's own slot
        check(!acked, "non-matching CpID (2): the glue never asserts CS#, so the FPU never responds at all");

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("COMPANION EXAMPLE TEST FAILED");
            $finish;
        end
        $display("COMPANION EXAMPLE TEST PASSED");
        $finish;
    end

endmodule
