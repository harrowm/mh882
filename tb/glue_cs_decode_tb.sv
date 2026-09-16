`timescale 1ns/1ps
`default_nettype none

// Phase 8 test: rtl/glue_cs_decode.sv's own CS# decode logic -- purely
// combinational, so this is a direct truth-table check, not a clocked
// dialog test like every other testbench in this project.

module glue_cs_decode_tb;

    logic [2:0]  fc;
    logic [19:13] a;
    logic         as_n;
    wire          cs_n;

    glue_cs_decode #(.CPID(3'b001)) u_glue (.fc, .a, .as_n, .cs_n);

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

    initial begin
        // Matching cycle: FC=111, A[19:16]=0010, A[15:13]=CPID(001), AS# asserted
        fc = 3'b111; a = {4'b0010, 3'b001}; as_n = 1'b0; #1;
        check(cs_n == 1'b0, "matching FC/address/CpID with AS# asserted -> CS# asserted");

        // Same address match, but AS# NOT asserted -- a real glue must
        // never assert CS# outside an active bus cycle.
        as_n = 1'b1; #1;
        check(cs_n == 1'b1, "matching address but AS# deasserted -> CS# stays deasserted");

        // Wrong FC (not CPU space)
        fc = 3'b101; a = {4'b0010, 3'b001}; as_n = 1'b0; #1;
        check(cs_n == 1'b1, "FC != 111 (not CPU space) -> CS# deasserted");

        // Right FC, wrong A[19:16] (e.g. 1111 = Interrupt Acknowledge's own CPU-space sub-type)
        fc = 3'b111; a = {4'b1111, 3'b001}; as_n = 1'b0; #1;
        check(cs_n == 1'b1, "A[19:16]=1111 (IACK, not coprocessor) -> CS# deasserted");

        // Right FC/type, wrong CpID (a different coprocessor's own slot)
        fc = 3'b111; a = {4'b0010, 3'b010}; as_n = 1'b0; #1;
        check(cs_n == 1'b1, "A[15:13] selects a DIFFERENT CpID -> CS# deasserted");

        // Every other CpID slot (0,2-7) must also stay deselected for
        // this instance's own CPID=1.
        for (int i = 0; i < 8; i++) begin
            if (i == 1) continue;
            fc = 3'b111; a = {4'b0010, i[2:0]}; as_n = 1'b0; #1;
            check(cs_n == 1'b1, $sformatf("CpID=%0d (not this instance's CPID=1) -> CS# deasserted", i));
        end

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("GLUE CS DECODE TEST FAILED");
            $finish;
        end
        $display("GLUE CS DECODE TEST PASSED");
        $finish;
    end

endmodule
