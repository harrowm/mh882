`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// Phase 5 test: FSAVE/FRESTORE state-frame dialog (Section 7.2.3/7.2.4).
// Drives m68882_top's own pins directly, mirroring this project's own
// established raw-pin-drive convention for a first-cut protocol test.
//
// Covers: Save CIR classifying Null vs Idle correctly (based on whether
// the programmer's model is genuinely all-zero); the Idle frame's own
// 13-longword placeholder payload transfer; Restore CIR validating the
// Null/Idle format words and rejecting an invalid one; a Null-frame
// restore genuinely resetting the WHOLE register file (not just
// internal state, per Section 6.4.2.1); and that the dialog machinery
// returns to a fully usable ST_IDLE afterward (a real Command CIR
// dialog works normally right after a save/restore sequence completes).

module m68882_frame_tb;

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

    task automatic run_cycle(
        input  logic [3:0]  sel_a4_a1,
        input  logic        is_write,
        input  logic [31:0] wdata,
        output logic [31:0] rd
    );
        int wait_ticks;
        @(posedge clk_4x);
        a      = {sel_a4_a1, 1'b1}; // 32-bit port throughout
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
        rd = d;

        @(posedge clk_4x);
        cs_n     = 1'b1;
        as_n     = 1'b1;
        ds_n     = 1'b1;
        d_drv_en = 1'b0;
        repeat (2) @(posedge clk_4x);
    endtask

    function automatic logic [31:0] cmd_word(
        logic [2:0] opclass, logic [2:0] rx, logic [2:0] ry, logic [6:0] ext
    );
        return {opclass, rx, ry, ext, 16'h0};
    endfunction

    localparam logic [15:0] FRAME_NULL_FMT = 16'h0000;
    localparam logic [15:0] FRAME_IDLE_FMT = {8'h1F, 8'd52};

    logic [31:0] rd;

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        // ── FSAVE: Null frame (programmer's model genuinely all-zero) ────
        run_cycle(CIR_SAVE, 1'b0, 32'h0, rd);
        check(rd[31:16] == FRAME_NULL_FMT, "FSAVE: an all-zero programmer's model reports the Null format word");
        check(u_top.u_proto.state_r == 0, "FSAVE: Null frame needs no Operand CIR transfer -- back to IDLE immediately");

        repeat (4) @(posedge clk_4x);

        // ── Populate the programmer's model (write FPCR via opclass 100) ──
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b100, 3'b100, 3'b000, 7'd0), rd); // move to FPCR
        run_cycle(CIR_INSTRADDR, 1'b1, 32'h0000_3000, rd); // Phase 6: mandatory Instruction Address CIR
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        run_cycle(CIR_OPERAND, 1'b1, 32'h0000_1000, rd); // any nonzero value
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);

        repeat (4) @(posedge clk_4x);

        // ── FSAVE: Idle frame (populated model, no dialog in progress) ────
        run_cycle(CIR_SAVE, 1'b0, 32'h0, rd);
        check(rd[31:16] == FRAME_IDLE_FMT, "FSAVE: a populated programmer's model reports the Idle format word");
        check(u_top.u_proto.state_r == 5, "FSAVE: Idle frame transitions to ST_WAIT_SAVE_XFER (state 5)"); // Phase 6: ST_WAIT_IADDR inserted at 1, shifting this from 4

        // Idle frame payload: 13 placeholder longwords, all zero.
        for (int i = 0; i < 13; i++) begin
            run_cycle(CIR_OPERAND, 1'b0, 32'h0, rd);
            check(rd == 32'h0, $sformatf("FSAVE: Idle frame payload longword %0d reads as the documented zero placeholder", i));
        end
        check(u_top.u_proto.state_r == 0, "FSAVE: after all 13 payload longwords, back to ST_IDLE");

        repeat (4) @(posedge clk_4x);

        // ── FRESTORE: an invalid format word is rejected ─────────────────
        run_cycle(CIR_RESTORE, 1'b1, 32'h1234_0000, rd); // not Null/Idle/Busy
        run_cycle(CIR_RESTORE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'hFFFF, "FRESTORE: an invalid format word reads back as the invalid-format marker");
        check(u_top.u_proto.state_r == 0, "FRESTORE: an invalid format word leaves the dialog at ST_IDLE, no transfer");

        repeat (4) @(posedge clk_4x);

        // ── FRESTORE: Idle frame validates and transfers its own payload ──
        run_cycle(CIR_RESTORE, 1'b1, {FRAME_IDLE_FMT, 16'h0}, rd);
        run_cycle(CIR_RESTORE, 1'b0, 32'h0, rd);
        check(rd[31:16] == FRAME_IDLE_FMT, "FRESTORE: a valid Idle format word echoes back unchanged");
        check(u_top.u_proto.state_r == 6, "FRESTORE: valid Idle format word transitions to ST_WAIT_RESTORE_XFER (state 6)"); // Phase 6: ST_WAIT_IADDR inserted at 1, shifting this from 5
        for (int i = 0; i < 13; i++) begin
            run_cycle(CIR_OPERAND, 1'b1, 32'hDEAD_0000 + i, rd); // discarded placeholder payload
        end
        check(u_top.u_proto.state_r == 0, "FRESTORE: after all 13 payload longwords, back to ST_IDLE");

        // Confirm the dialog machinery is genuinely usable again afterward.
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b000, 3'd0, 3'd1, 7'd0), rd); // FPm to FPn, no-op ext
        run_cycle(CIR_INSTRADDR, 1'b1, 32'h0000_3004, rd); // Phase 6: mandatory Instruction Address CIR
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000, "FRESTORE: a real Command CIR dialog works normally right afterward");

        repeat (4) @(posedge clk_4x);

        // ── FRESTORE: Null frame resets the WHOLE register file ──────────
        // FPCR was set nonzero earlier -- confirm it's genuinely nonzero
        // right before the Null restore, then confirm it's zero after.
        check(u_top.u_proto.u_regfile.fpcr_r != 32'h0, "FRESTORE setup: FPCR is genuinely nonzero before the Null-frame restore");
        run_cycle(CIR_RESTORE, 1'b1, {FRAME_NULL_FMT, 16'h0}, rd);
        run_cycle(CIR_RESTORE, 1'b0, 32'h0, rd);
        check(rd[31:16] == FRAME_NULL_FMT, "FRESTORE: the Null format word echoes back unchanged");
        repeat (2) @(posedge clk_4x);
        check(u_top.u_proto.u_regfile.fpcr_r == 32'h0,
              "FRESTORE: a Null-frame restore resets FPCR (Section 6.4.2.1: whole programmer's model, not just internal state)");
        check(u_top.u_proto.state_r == 0, "FRESTORE: Null frame needs no Operand CIR transfer -- back to IDLE immediately");

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("FRAME TEST FAILED");
            $finish;
        end
        $display("FRAME TEST PASSED");
        $finish;
    end

endmodule
