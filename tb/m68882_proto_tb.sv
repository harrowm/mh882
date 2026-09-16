`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// Phase 3 dialog test: drives m68882_top's own pins directly (no host
// CPU) through real Command/Response/Operand/Register-Select/Condition
// CIR dialogs, confirming m68882_proto.sv's own instruction-class decode
// and transfer sequencing (Section 4.7.1/Table 4-11) actually works end
// to end, not just that the bus timing is correct (that's
// tb/m68882_biu_smoke_tb.sv's own job, Phase 2).
//
// Covers: opclass 010 (external operand -> FPn), opclass 011 (FPn ->
// external), opclass 100 (move to FPCR), a Condition CIR EQ evaluation,
// and opclass 110 (move multiple to FP data registers, 2 registers).
// Opclass 000/001/101/111 are not independently retested -- 000 is a
// trivial no-transfer case, 101/111 exercise the identical code paths as
// 100/110 in the opposite direction (SUPPLY instead of RECEIVE), and
// 001 is unused/reserved (Table 4-11).

module m68882_proto_tb;

    logic clk_4x = 1'b0;
    logic rst_n  = 1'b0;

    logic [4:0]  a      = 5'h00;
    logic        size_n = 1'b1; // 32-bit port throughout (size_n=1, a0=1)
    logic        as_n   = 1'b1;
    logic        cs_n   = 1'b1;
    logic        rw     = 1'b1;
    logic        ds_n   = 1'b1;
    wire         dsack0_n;
    wire         dsack1_n;
    wire         sense_n;

    logic [31:0] d_drv    = 32'h0;
    logic        d_drv_en = 1'b0;
    wire  [31:0] d = d_drv_en ? d_drv : 32'bz;

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

    // Drive one bus cycle. a0 is always 1 (32-bit port, Table 9-2).
    task automatic run_cycle(
        input  logic [3:0]  sel_a4_a1,
        input  logic        is_write,
        input  logic [31:0] wdata,
        output logic [31:0] rdata
    );
        int wait_ticks;
        @(posedge clk_4x);
        a      = {sel_a4_a1, 1'b1};
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

        wait_ticks = 0;
        while (dsack0_n && dsack1_n && wait_ticks < 60) begin
            @(posedge clk_4x);
            wait_ticks++;
        end
        rdata = d;

        @(posedge clk_4x);
        cs_n     = 1'b1;
        as_n     = 1'b1;
        ds_n     = 1'b1;
        d_drv_en = 1'b0;
        repeat (2) @(posedge clk_4x);
    endtask

    logic [31:0] rd;

    // Command-word builder (Section 4.7.1): opclass(3)/rx(3)/ry(3)/ext(7),
    // placed on d[31:16] (the D16-D31 lane, matching every 16-bit CIR).
    function automatic logic [31:0] cmd_word(
        logic [2:0] opclass, logic [2:0] rx, logic [2:0] ry, logic [6:0] ext
    );
        return {opclass, rx, ry, ext, 16'h0};
    endfunction

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        // ── opclass 010: external operand (Long, 1 chunk) -> FP3 ────────
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b010, FMT_L, 3'd3, 7'd0), rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_EVAL_EA)},
              "opclass 010: Response = CA=1,DR=0,PRIM_EVAL_EA");
        run_cycle(CIR_OPERAND, 1'b1, 32'hCAFE_BABE, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000, "opclass 010: Response reverts to Null after the transfer");
        check(u_top.u_proto.u_regfile.fp_r[3][95:64] == 32'hCAFE_BABE,
              "opclass 010: the transferred value landed in FP3's own storage");

        repeat (4) @(posedge clk_4x);

        // ── opclass 011: FP3 -> external operand (Long) ─────────────────
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b011, FMT_L, 3'd3, 7'd0), rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b1, 13'(PRIM_EVAL_EA)},
              "opclass 011: Response = CA=1,DR=1,PRIM_EVAL_EA");
        run_cycle(CIR_OPERAND, 1'b0, 32'h0, rd);
        check(rd == 32'hCAFE_BABE, "opclass 011: FP3's own stored value is supplied back out");
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000, "opclass 011: Response reverts to Null after the transfer");

        repeat (4) @(posedge clk_4x);

        // ── opclass 100: move to FPCR ────────────────────────────────────
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b100, 3'b100, 3'b000, 7'd0), rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_XFER_SINGLE)},
              "opclass 100: Response = CA=1,DR=0,PRIM_XFER_SINGLE");
        run_cycle(CIR_OPERAND, 1'b1, 32'h1234_5678, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000, "opclass 100: Response reverts to Null after the transfer");
        check(u_top.u_proto.u_regfile.fpcr_r == 32'h1234_5678, "opclass 100: FPCR received the transferred value");

        repeat (4) @(posedge clk_4x);

        // ── opclass 101: FPSR -> memory (round-trip via opclass 100 first) ──
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b100, 3'b010, 3'b000, 7'd0), rd); // move to FPSR
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        run_cycle(CIR_OPERAND, 1'b1, 32'h0400_0000, rd); // Z bit (bit26) set
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);

        repeat (4) @(posedge clk_4x);

        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b101, 3'b010, 3'b000, 7'd0), rd); // FPSR to memory
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b1, 13'(PRIM_XFER_SINGLE)},
              "opclass 101: Response = CA=1,DR=1,PRIM_XFER_SINGLE");
        run_cycle(CIR_OPERAND, 1'b0, 32'h0, rd);
        check(rd == 32'h0400_0000, "opclass 101: FPSR's own stored value is supplied back out");

        repeat (4) @(posedge clk_4x);

        // ── Condition CIR: EQ against FPSR's Z bit (still set from above) ──
        run_cycle(CIR_CONDITION, 1'b1, {10'b0, 6'b000001, 16'h0}, rd); // EQ predicate
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[16] == 1'b1, "Condition CIR: EQ evaluates true when FPSR's Z bit is set");

        repeat (4) @(posedge clk_4x);

        // ── opclass 110: move multiple to FP0/FP1 ───────────────────────
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b110, 3'b000, 3'b000, 7'd3), rd); // mask=8'b0000_0011
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_XFER_MULTI)},
              "opclass 110: Response = CA=1,DR=0,PRIM_XFER_MULTI");
        run_cycle(CIR_REGSELECT, 1'b0, 32'h0, rd);
        check(rd == 32'h0300_0000, "opclass 110: Register Select CIR reports mask 0x03 on its own MSB 8 bits");

        for (int i = 0; i < 6; i++) begin
            run_cycle(CIR_OPERAND, 1'b1, 32'hA000_0000 + i, rd);
        end
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000, "opclass 110: Response reverts to Null after all 6 chunks");
        check(u_top.u_proto.u_regfile.fp_r[0] == {32'hA000_0000, 32'hA000_0001, 32'hA000_0002},
              "opclass 110: FP0 received its own 3 chunks in order");
        check(u_top.u_proto.u_regfile.fp_r[1] == {32'hA000_0003, 32'hA000_0004, 32'hA000_0005},
              "opclass 110: FP1 received its own 3 chunks in order");

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("PROTO TEST FAILED");
            $finish;
        end
        $display("PROTO TEST PASSED");
        $finish;
    end

endmodule
