`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;
import m68882_apu_pkg::*;

// Phase 6 test: the genuine BIU->CU->APU pipeline overlap (Section
// 5.1.1) -- a real 2-deep pipeline (slot A executing in the APU, slot B
// staged and waiting), the busy-reject on a 3rd arithmetic dispatch
// while both slots are full (Section 7.2.6), the mandatory Instruction
// Address CIR requirement and its protocol-violation detection, the real
// AB-vs-XA Control CIR distinction (Section 7.5.4), and the exception-
// primitive-persists-until-FSAVE rule (Section 7.5.4.2). Drives
// m68882_top's own pins directly, mirroring this project's own
// established raw-pin-drive convention.

module m68882_pipeline_tb;

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
        output logic [31:0] rdata
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
        rdata = d;

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

    task automatic load_fp(input int idx, input logic [95:0] val);
        u_top.u_proto.u_regfile.fp_r[idx] = val;
    endtask

    // Dispatch WITHOUT waiting for the pipeline to drain -- used
    // deliberately here (unlike apu_tb.sv's own dispatch() helper) since
    // this whole test's own point is to observe mid-flight pipeline
    // state between dispatches, not just the final committed result.
    task automatic dispatch_noblock(input logic [31:0] cmdw, input logic [31:0] iaddr, output logic [31:0] rd2);
        run_cycle(CIR_COMMAND, 1'b1, cmdw, rd2);
        run_cycle(CIR_INSTRADDR, 1'b1, iaddr, rd2);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd2);
    endtask

    // Both tasks below settle 2 extra ticks after their own wait loop
    // exits, mirroring apu_tb.sv's own dispatch() task and for the same
    // reason: apu_wr_en (proto.sv's own registered output) is only
    // CONSUMED by m68882_regfile.sv's own separate always_ff one clk_4x
    // edge later than slotA_op_r/slotA_valid_r themselves update -- the
    // register-file WRITE genuinely lags the pipeline-state transition by
    // one more edge, so checking fp_r[]/fpsr_r/etc immediately after the
    // wait loop exits (before that edge) would race a real, structural
    // one-cycle latency, not a testbench bug.
    task automatic wait_slotA_op(input logic [6:0] want_op, input int bound);
        int wait_ticks;
        wait_ticks = 0;
        while (!(u_top.u_proto.slotA_valid_r && u_top.u_proto.slotA_op_r == want_op) && wait_ticks < bound) begin
            @(posedge clk_4x);
            wait_ticks++;
        end
        repeat (2) @(posedge clk_4x);
    endtask

    task automatic wait_pipeline_idle(input int bound);
        int wait_ticks;
        wait_ticks = 0;
        while ((u_top.u_proto.slotA_valid_r || u_top.u_proto.slotB_valid_r) && wait_ticks < bound) begin
            @(posedge clk_4x);
            wait_ticks++;
        end
        repeat (2) @(posedge clk_4x);
    endtask

    // Known-good extended-precision constants (sign|exp15|reserved16|mantissa64)
    localparam logic [95:0] EXT_1_0  = 96'h3fff_0000_8000_0000_0000_0000;
    localparam logic [95:0] EXT_2_0  = 96'h4000_0000_8000_0000_0000_0000;
    localparam logic [95:0] EXT_3_0  = 96'h4000_0000_c000_0000_0000_0000;
    localparam logic [95:0] EXT_4_0  = 96'h4001_0000_8000_0000_0000_0000;
    localparam logic [95:0] EXT_6_0  = 96'h4001_0000_c000_0000_0000_0000;

    // Phase 9: slotA_op_r/slotB_op_r now hold the RAW Table 4-13
    // extension-field code directly (rtl/m68882_proto.sv's own header
    // comment) -- these mirror that directly rather than a separate enum.
    localparam logic [6:0] APU_OP_ADD  = 7'h22;
    localparam logic [6:0] APU_OP_MUL  = 7'h23;
    localparam logic [6:0] APU_OP_DIV  = 7'h20;

    logic [31:0] rd;

    // proto_violation_r is a genuine one-clk_4x-tick pulse (same
    // established convention as `abort`/write_pulse elsewhere in this
    // project) -- latch it into a sticky flag so a check well after the
    // offending access can still see it fired.
    logic violation_seen = 1'b0;
    always @(posedge clk_4x)
        if (u_top.u_proto.proto_violation_r) violation_seen <= 1'b1;

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        // ═══ Genuine 2-deep pipeline overlap ═══════════════════════════
        load_fp(0, EXT_2_0);  load_fp(1, EXT_4_0);  // FDIV: FP1 = 4.0/2.0
        load_fp(2, EXT_1_0);  load_fp(3, EXT_2_0);  // FADD: FP3 = FP2+FP3 (dest sentinel != 3.0)
        load_fp(4, EXT_2_0);  load_fp(5, EXT_3_0);  // FMUL: FP5 = FP4*FP5 (dest sentinel != 6.0)

        // Dispatch a slow FDIV -- occupies slot A for apu_latency(DIV) real
        // Table 8-3 cycles (108 external cycles x4 = 432 clk_4x ticks).
        dispatch_noblock(cmd_word(3'b000, 3'd0, 3'd1, 7'h20), 32'h0000_5000, rd);
        check(rd[31:16] == 16'h0000, "FDIV dispatch: Response is Null/CA=0 immediately (CU's own part is instant)");
        check(u_top.u_proto.slotA_valid_r && u_top.u_proto.slotA_op_r == APU_OP_DIV,
              "FDIV dispatch: slot A now holds the DIV, occupying the APU pipeline");
        check(u_top.u_proto.u_regfile.fpiar_r == 32'h0000_5000,
              "FDIV dispatch: FPIAR (the real APU-stage register) auto-loads immediately on slot-A entry");

        // While slot A is still busy, dispatch a genuinely CU-only
        // operation (move to FPCR, opclass 100). Phase 9's own Table 8-3
        // investigation found NOTHING in opclass 000 (register-to-
        // register arithmetic, including FABS/FNEG/FCMP/FTST) is
        // actually zero-latency on real silicon -- Table 5-1's own
        // "Minimum-Concurrency" framing means concurrent WITH other
        // pipeline activity, not zero-latency in isolation. The real
        // "never touches the APU pipeline at all" concurrency Section
        // 5.1.1 describes belongs to opclass 100/101/110/111 instead.
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b100, 3'b100, 3'b000, 7'd0), rd); // move to FPCR
        run_cycle(CIR_INSTRADDR, 1'b1, 32'h0000_5010, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_XFER_SINGLE)},
              "move-to-FPCR dispatch (while DIV busy): a genuinely CU-only dialog, never touches slot A/B at all");
        run_cycle(CIR_OPERAND, 1'b1, 32'h0000_0010, rd); // rounding mode = toward-zero, FPCR bits[5:4]
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(u_top.u_proto.u_regfile.fpcr_r == 32'h0000_0010,
              "move-to-FPCR: completed immediately despite the APU pipeline being busy with the DIV");
        check(u_top.u_proto.slotA_valid_r && u_top.u_proto.slotA_op_r == APU_OP_DIV,
              "move-to-FPCR dispatch left slot A completely undisturbed -- still the same in-flight DIV");
        check(u_top.u_proto.u_regfile.fpiar_r == 32'h0000_5000,
              "move-to-FPCR dispatch: FPIAR NOT touched -- a real CU-only dialog never touches the APU-stage register");

        // While slot A is STILL busy, dispatch a SECOND real arithmetic
        // op (FADD) -- the genuine 2-deep pipeline: it must be accepted
        // into slot B (Response still Null/CA=0 -- the main processor is
        // never blocked here either), not executed yet.
        dispatch_noblock(cmd_word(3'b000, 3'd2, 3'd3, 7'h22), 32'h0000_5020, rd);
        check(rd[31:16] == 16'h0000, "FADD dispatch (while DIV busy): Response is Null/CA=0 -- genuine 2-deep pipelining, main processor proceeds");
        check(u_top.u_proto.slotB_valid_r && u_top.u_proto.slotB_op_r == APU_OP_ADD,
              "FADD dispatch: staged into slot B, waiting for slot A (the DIV) to free");
        check(u_top.u_proto.u_regfile.fp_r[3] != EXT_3_0,
              "FADD dispatch: destination register NOT yet written -- slot B hasn't executed");
        check(u_top.u_proto.u_regfile.fpiar_r == 32'h0000_5000,
              "FADD dispatch: FPIAR NOT yet updated (still DIV's address) -- only the genuinely-executing APU-stage instruction loads it, not a merely-staged one");

        // A THIRD arithmetic dispatch while BOTH slots are full must be
        // rejected (Section 7.2.6) -- Response reports CA=1 (busy), and
        // neither slot's own content changes.
        dispatch_noblock(cmd_word(3'b000, 3'd4, 3'd5, 7'h23), 32'h0000_5030, rd);
        check(rd[31:16] == 16'h8000, "FMUL dispatch (both slots full): Response reports CA=1 (busy) -- Section 7.2.6 defer-command-word");
        check(u_top.u_proto.slotB_valid_r && u_top.u_proto.slotB_op_r == APU_OP_ADD,
              "FMUL rejection: slot B still holds the earlier FADD, untouched by the rejected dispatch");
        check(u_top.u_proto.u_regfile.fp_r[5] != EXT_6_0,
              "FMUL rejection: destination register NOT written -- the dispatch never happened");

        // Wait for the DIV to commit and the FADD to promote from slot B
        // into slot A.
        wait_slotA_op(APU_OP_ADD, 1000);
        check(u_top.u_proto.u_regfile.fp_r[1] == EXT_2_0, "FDIV eventually commits: FP1 = 4.0/2.0 == 2.0");
        check(!u_top.u_proto.slotB_valid_r, "Slot B promotion: now empty after handing off to slot A");
        check(u_top.u_proto.slotA_valid_r && u_top.u_proto.slotA_op_r == APU_OP_ADD,
              "Slot B promotion: the staged FADD is now genuinely executing in slot A");
        check(u_top.u_proto.u_regfile.fpiar_r == 32'h0000_5020,
              "Slot B promotion: FPIAR (APU-stage) auto-loads the PROMOTED instruction's own address, not the DIV's stale one");

        // Retry the previously-rejected FMUL -- slot A is busy (the
        // promoted FADD) but slot B is free again, so this time it's
        // accepted.
        dispatch_noblock(cmd_word(3'b000, 3'd4, 3'd5, 7'h23), 32'h0000_5040, rd);
        check(rd[31:16] == 16'h0000, "FMUL retry (slot B free again): Response is Null/CA=0 -- accepted this time");
        check(u_top.u_proto.slotB_valid_r && u_top.u_proto.slotB_op_r == APU_OP_MUL,
              "FMUL retry: staged into slot B");

        wait_slotA_op(APU_OP_MUL, 1000);
        check(u_top.u_proto.u_regfile.fp_r[3] == EXT_3_0, "Slot A (FADD) eventually commits: FP3 = 1.0+2.0 == 3.0");
        check(u_top.u_proto.u_regfile.fpiar_r == 32'h0000_5040,
              "Second promotion: FPIAR now reflects the FMUL's own address");

        wait_pipeline_idle(1000);
        check(!u_top.u_proto.slotA_valid_r && !u_top.u_proto.slotB_valid_r,
              "Pipeline fully drains: both slots empty once the FMUL itself commits");
        check(u_top.u_proto.u_regfile.fp_r[5] == EXT_6_0, "Slot A (FMUL) eventually commits: FP5 = 2.0*3.0 == 6.0");

        repeat (4) @(posedge clk_4x);

        // ═══ Mandatory Instruction Address CIR / protocol violation ═════
        violation_seen = 1'b0;
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b000, 3'd6, 3'd7, 7'h18), rd); // FABS, dispatched...
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd); // ...but IA CIR is skipped: a real protocol violation
        check(violation_seen, "Protocol violation: proto_violation_r pulses when Response CIR is read before the mandatory Instruction Address CIR write");
        check(u_top.u_proto.state_r == 1 /* ST_WAIT_IADDR */,
              "Protocol violation: skipping the mandatory Instruction Address CIR leaves the dialog stuck in ST_WAIT_IADDR");

        // The dialog is recoverable via abort (AB); confirm a real
        // Command CIR dispatch works normally afterward.
        run_cycle(CIR_CONTROL, 1'b1, 32'h0001_0000, rd); // AB bit (bit16 of the 32-bit write == bit0 of the 16-bit CIR)
        repeat (2) @(posedge clk_4x);
        check(u_top.u_proto.state_r == 0, "AB recovers the stuck dialog back to ST_IDLE");

        repeat (4) @(posedge clk_4x);

        // ═══ AB abort-window semantics: a genuine in-progress dialog ════
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b010, FMT_L, 3'd0, 7'd0), rd); // external L operand -> FP0
        run_cycle(CIR_INSTRADDR, 1'b1, 32'h0000_6000, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(u_top.u_proto.state_r == 2 /* ST_WAIT_XFER */,
              "AB test setup: a genuine external-operand dialog is in progress (a real abort window)");
        run_cycle(CIR_CONTROL, 1'b1, 32'h0001_0000, rd); // AB
        repeat (2) @(posedge clk_4x);
        check(u_top.u_proto.state_r == 0, "AB: aborts a genuinely in-progress dialog back to ST_IDLE");

        repeat (4) @(posedge clk_4x);

        // ═══ XA vs FSAVE: exception primitive persists until FSAVE ══════
        // Enable the OVFL trap (FPCR ENABLE byte bit 12), then force a
        // real exponent overflow via FADD (same pattern as apu_tb.sv's
        // own Phase 4d OVFL check).
        run_cycle(CIR_COMMAND, 1'b1, cmd_word(3'b100, 3'b100, 3'b000, 7'd0), rd); // move to FPCR
        run_cycle(CIR_INSTRADDR, 1'b1, 32'h0000_7000, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        run_cycle(CIR_OPERAND, 1'b1, 32'h0000_1000, rd); // bit12 == OVFL enable
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);

        load_fp(0, {1'b0, 15'd32766, 16'h0, 64'h8000_0000_0000_0000});
        load_fp(1, {1'b0, 15'd32766, 16'h0, 64'h8000_0000_0000_0000});
        dispatch_noblock(cmd_word(3'b000, 3'd0, 3'd1, 7'h22), 32'h0000_7010, rd); // FADD -> overflow
        wait_pipeline_idle(1000);

        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_TAKE_MID)},
              "XA/FSAVE: a trapped OVFL exception reports Take-Mid-Instruction-Exception (CA=1)");

        // Section 7.5.4.2: XA alone does NOT clear it.
        run_cycle(CIR_CONTROL, 1'b1, 32'h0002_0000, rd); // XA bit (bit17 of the 32-bit write == bit1 of the 16-bit CIR)
        repeat (2) @(posedge clk_4x);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_TAKE_MID)},
              "XA/FSAVE: writing XA alone leaves the primitive exactly as Take-Mid-Instruction-Exception -- unchanged");

        // Only a COMPLETING FSAVE clears it.
        run_cycle(CIR_SAVE, 1'b0, 32'h0, rd); // Busy frame (a pending exception primitive counts as busy... actually
                                                // FPCR/FPSR are nonzero here regardless, so Idle/Busy either way --
                                                // what matters is the transfer itself completing below.
        if (u_top.u_proto.state_r != 0) begin
            for (int i = 0; i < 52; i++) begin
                run_cycle(CIR_OPERAND, 1'b0, 32'h0, rd);
                if (u_top.u_proto.state_r == 0) break;
            end
        end
        repeat (2) @(posedge clk_4x);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        check(rd[31:16] == 16'h0000,
              "XA/FSAVE: a COMPLETING FSAVE clears the primitive back to Null (Section 7.5.4.2)");

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("PIPELINE TEST FAILED");
            $finish;
        end
        $display("PIPELINE TEST PASSED");
        $finish;
    end

endmodule
