`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;

// Phase 10 test: the full 32-entry Conditional Predicate Field (Table
// 4-20/4.4) plus BSUN + Take-Pre-Instruction-Exception. Drives
// m68882_top's own pins directly (no host CPU), matching
// tb/m68882_proto_tb.sv's own established Condition CIR dialog pattern
// (that file's own single EQ check is the only prior coverage this
// project had).
//
// FPSR's own N/Z/NAN condition-code bits and FPCR's own BSUN-enable bit
// are set via direct hierarchical backdoor writes (same established
// convention as tb/m68882_apu_tb.sv's own load_fp task) rather than
// driving a real arithmetic op to that state each time -- this test is
// about the Condition CIR's own predicate-evaluation logic in isolation,
// not re-proving the arithmetic core.
//
// Expected values below were derived independently from
// tools/musashi/m68kfpu.c's own TEST_CONDITION() (the same source
// rtl/m68882_proto.sv's own cond_eval function cites as its primary
// reference, since the manual's own printed Boolean equations have lost
// negation-bar formatting in several places) -- transcribed here as a
// flat per-state array rather than copied from the RTL's own case
// statement, so a bug in one wouldn't trivially also appear in the
// other.

module m68882_cond_tb;

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

    task automatic set_fpsr_ccode(input logic n, input logic z, input logic i, input logic nan);
        u_top.u_proto.u_regfile.fpsr_r[27] = n;
        u_top.u_proto.u_regfile.fpsr_r[26] = z;
        u_top.u_proto.u_regfile.fpsr_r[25] = i;
        u_top.u_proto.u_regfile.fpsr_r[24] = nan;
    endtask

    task automatic clear_fpsr_exc(); // clears BSUN et al so each state starts clean
        u_top.u_proto.u_regfile.fpsr_r[15:8] = 8'h0;
    endtask

    task automatic set_fpcr_bsun_enable(input logic en);
        u_top.u_proto.u_regfile.fpcr_r[15] = en;
    endtask

    // Evaluate one predicate (6-bit field) and return the true/false bit.
    task automatic eval_cond(input logic [5:0] pred, output logic tf);
        run_cycle(CIR_CONDITION, 1'b1, {10'b0, pred, 16'h0}, rd);
        run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
        tf = rd[16];
    endtask

    // Base-16 predicate mnemonics, code 0x0-0xF, for check messages.
    string mnem [0:15] = '{"F","EQ","GT","GE","LT","LE","GL","GLE",
                            "NGLE","NGL","NLE","NLT","NGE","NGT","NE","T"};

    initial begin
        repeat (4) @(posedge clk_4x);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_4x);

        // ── The 16 base predicates (Table 4-20's own "ordered" group,
        // 0x00-0x0F -- never sets BSUN), across 5 representative FPSR
        // condition-code states. Expected arrays derived independently
        // from Musashi's own TEST_CONDITION() (see this file's own
        // header comment). ──────────────────────────────────────────
        begin
            logic tf;
            int   k;

            // index k = predicate code 0x0..0xF
            logic gt_exp [0:15];
            logic lt_exp [0:15];
            logic eq_exp [0:15];
            logic un_exp [0:15];
            logic un2_exp[0:15];
            gt_exp  = '{0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1}; // n0,z0,nan0
            lt_exp  = '{0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1}; // n1,z0,nan0
            eq_exp  = '{0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}; // n0,z1,nan0
            un_exp  = '{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}; // n0,z0,nan1
            un2_exp = '{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}; // n1,z0,nan1 (N must not leak through NAN)

            set_fpsr_ccode(1'b0, 1'b0, 1'b0, 1'b0); // GT-shaped: n0,z0,nan0
            for (k = 0; k < 16; k++) begin
                eval_cond(6'(k), tf);
                check(tf == gt_exp[k], $sformatf("Phase 10: predicate %s (0x%02h) on GT state (n=0,z=0,nan=0) == %0d", mnem[k], k, gt_exp[k]));
            end

            set_fpsr_ccode(1'b1, 1'b0, 1'b0, 1'b0); // LT-shaped: n1,z0,nan0
            for (k = 0; k < 16; k++) begin
                eval_cond(6'(k), tf);
                check(tf == lt_exp[k], $sformatf("Phase 10: predicate %s (0x%02h) on LT state (n=1,z=0,nan=0) == %0d", mnem[k], k, lt_exp[k]));
            end

            set_fpsr_ccode(1'b0, 1'b1, 1'b0, 1'b0); // EQ-shaped: n0,z1,nan0
            for (k = 0; k < 16; k++) begin
                eval_cond(6'(k), tf);
                check(tf == eq_exp[k], $sformatf("Phase 10: predicate %s (0x%02h) on EQ state (n=0,z=1,nan=0) == %0d", mnem[k], k, eq_exp[k]));
            end

            set_fpsr_ccode(1'b0, 1'b0, 1'b0, 1'b1); // UN-shaped: n0,z0,nan1
            for (k = 0; k < 16; k++) begin
                eval_cond(6'(k), tf);
                check(tf == un_exp[k], $sformatf("Phase 10: predicate %s (0x%02h) on UN state (n=0,z=0,nan=1) == %0d", mnem[k], k, un_exp[k]));
            end

            set_fpsr_ccode(1'b1, 1'b0, 1'b0, 1'b1); // UN2-shaped: n1,z0,nan1 -- N must not leak through
            for (k = 0; k < 16; k++) begin
                eval_cond(6'(k), tf);
                check(tf == un2_exp[k], $sformatf("Phase 10: predicate %s (0x%02h) on UN2 state (n=1,z=0,nan=1, N must not leak through NAN) == %0d", mnem[k], k, un2_exp[k]));
            end
        end

        repeat (4) @(posedge clk_4x);

        // ── The "signaling" group (0x10-0x1F) reuses the IDENTICAL 16
        // formulas -- spot-check a representative sample against the
        // same expected values already proven above for the 0x0X group. ──
        begin
            logic tf;
            set_fpsr_ccode(1'b0, 1'b0, 1'b0, 1'b0); // GT-shaped, ordered (no BSUN risk here)
            eval_cond(6'h12, tf); // GT, signaling
            check(tf == 1'b1, "Phase 10: signaling-group GT (0x12) on GT state matches ordered-group GT (0x02)");
            eval_cond(6'h11, tf); // SEQ
            check(tf == 1'b0, "Phase 10: signaling-group SEQ (0x11) on GT state matches ordered-group EQ (0x01)");
            eval_cond(6'h1F, tf); // ST
            check(tf == 1'b1, "Phase 10: signaling-group ST (0x1F) == always true");
            eval_cond(6'h10, tf); // SF
            check(tf == 1'b0, "Phase 10: signaling-group SF (0x10) == always false");
        end

        repeat (4) @(posedge clk_4x);

        // ── BSUN: fires ONLY for the signaling group (0x10-0x1F) with
        // NAN set; the ordered group (0x00-0x0F) NEVER sets BSUN "under
        // any circumstances" (Table 4-20 Note 1, confirmed directly). ──
        begin
            logic tf;
            set_fpsr_ccode(1'b0, 1'b0, 1'b0, 1'b1); // NAN set
            set_fpcr_bsun_enable(1'b0);
            clear_fpsr_exc();

            eval_cond(6'h02, tf); // OGT (ordered group) with NaN present
            check(u_top.u_proto.u_regfile.fpsr_r[15] == 1'b0,
                  "Phase 10: ordered-group predicate (OGT) with NaN present does NOT set BSUN");

            clear_fpsr_exc();
            eval_cond(6'h12, tf); // GT (signaling group) with NaN present, BSUN trap disabled
            check(u_top.u_proto.u_regfile.fpsr_r[15] == 1'b1,
                  "Phase 10: signaling-group predicate (GT) with NaN present DOES set BSUN");
            check(tf == 1'b0,
                  "Phase 10: signaling-group GT with NaN present still returns the ordinary (false) true/false result when the BSUN trap is disabled");
        end

        repeat (4) @(posedge clk_4x);

        // ── Take-Pre-Instruction-Exception: BSUN trap ENABLED + a
        // signaling predicate + NaN present must return PRIM_TAKE_PRE
        // instead of the ordinary null true/false response (Table 4-20
        // Note 2, confirmed directly). ──────────────────────────────
        begin
            set_fpsr_ccode(1'b0, 1'b0, 1'b0, 1'b1); // NAN set
            set_fpcr_bsun_enable(1'b1);
            clear_fpsr_exc();

            run_cycle(CIR_CONDITION, 1'b1, {10'b0, 6'h14, 16'h0}, rd); // LT (signaling), with BSUN trap enabled
            run_cycle(CIR_RESPONSE, 1'b0, 32'h0, rd);
            check(rd[31:16] == {1'b1, 1'b0, 1'b0, 13'(PRIM_TAKE_PRE)},
                  "Phase 10: signaling predicate + NaN + BSUN trap enabled -> Response = CA=1,DR=0,PRIM_TAKE_PRE");
            check(u_top.u_proto.u_regfile.fpsr_r[15] == 1'b1,
                  "Phase 10: BSUN bit is set alongside the Take-Pre-Instruction-Exception primitive");

            set_fpcr_bsun_enable(1'b0);
        end

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count != 0) begin
            $display("COND TEST FAILED");
            $finish;
        end
        $display("COND TEST PASSED");
        $finish;
    end

endmodule
