`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;
import m68882_apu_pkg::*;

// MC68882 instruction-dialog protocol -- Phase 3 (dialog shape) + Phase 4a
// (register-to-register FADD/FSUB, wired into the opclass-000 branch
// below).
//
// Owns the Response ($00), Command ($0A), Condition ($0E), Operand
// ($10), and Register Select ($14) CIRs, and drives m68882_regfile.
// Control/Restore/Save/Instruction-Address/Operand-Address stay in
// m68882_cir.sv (Phase 2/5/6 territory) -- this module only takes a
// single `abort` pulse from there (Control CIR write) to reset its own
// dialog state.
//
// Scope, explicitly bounded to match plan.md's own Phase 3 description
// ("no real math yet"):
//   - Command word decode (Section 4.7.1/Table 4-11) picks the REAL
//     instruction class and REAL primitive/transfer-count shape for all
//     6 opclasses -- this is genuine protocol logic, not a synthetic
//     testbed, because the command word IS the real, confirmed
//     instruction encoding.
//   - What actually gets STORED/SUPPLIED through the Operand CIR is a
//     raw, unconverted copy of whatever bytes cross the bus -- no real
//     B/W/L/S/D/X/P format conversion or arithmetic happens (that is
//     the CU/APU's own job, Phase 4/6). This exercises the real dialog
//     SHAPE (right primitive, right number of Operand CIR accesses per
//     format, right CIR sequencing) without pretending to compute a
//     real numeric result.
//   - The "busy APU, defer command word" path (Section 7.2.6) is not
//     modeled -- every Command CIR write is treated as arriving to an
//     idle FPCP, because nothing in this phase ever takes more than one
//     cycle to "complete." Revisit once Phase 4 gives an instruction a
//     genuine multi-cycle execution time.
//   - Response CIR's own primitive-identifying payload bits[12:0] are
//     this project's OWN internal encoding (m68882_cir_pkg.sv's own
//     header comment) -- CA(15)/PC(14)/DR(13) are confirmed real bit
//     POSITIONS, but not cross-checked against a real numeric primitive
//     code table.
//   - Condition CIR (Section 4.4): only EQ/NE are evaluated against a
//     real formula (both unambiguous -- Z and !Z, confirmed identically
//     in both the manual's own IEEE-aware and non-aware predicate
//     tables). The other 30 of the 32 predicates have real Boolean
//     formulas in the manual's own Table 4-8, but the extracted OCR text
//     has visibly corrupted logical operators -- rather than transcribe
//     a formula this project cannot actually verify, they are stubbed to
//     always evaluate false. Cross-check against Musashi's own 68881
//     predicate table (MH030's tools/musashi/) before relying on this.
//   - FPSR condition-code byte bit positions (N=27,Z=26,I=25,NAN=24)
//     follow the widely-documented standard convention, not independently
//     re-derived here.
//   - The FPcr-select field's own bit-to-register assignment is INFERRED
//     from the field's 3-bit width and the confirmed FPCR-first/FPSR-
//     second/FPIAR-third transfer order -- not from an explicit
//     bit-position statement in this extraction.
//   - Move-multiple's dynamic register-list-in-Dn form is NOT
//     implemented -- only the static list-in-the-command-word form.

module m68882_proto (
    input  logic clk_4x,
    input  logic rst_n,

    input  logic      cyc_ack,
    input  logic      cyc_write,
    input  cir_sel_t  cyc_sel,
    input  logic      abort,        // pulse from m68882_cir.sv's Control CIR write

    input  logic [31:0] d_in,
    output logic [31:0] d_out,
    output logic         d_oe
);

    // ── One-shot access pulse (Phase 2's own write_pulse finding,
    // generalized to both directions). ─────────────────────────────────
    logic cyc_ack_prev_r;
    always_ff @(posedge clk_4x or negedge rst_n)
        if (!rst_n) cyc_ack_prev_r <= 1'b0;
        else        cyc_ack_prev_r <= cyc_ack;

    wire ack_pulse = cyc_ack && !cyc_ack_prev_r;

    // ── Dialog state (declared before the register-file wiring below,
    // since reg_idx_r/chunk_idx_r are referenced there) ────────────────
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_WAIT_XFER,        // single EA / FPcr transfer(s) in progress
        ST_WAIT_REGSEL,      // multi-register move: mask ready to be read
        ST_WAIT_MULTI_XFER,  // multi-register move: transferring longwords
        ST_WAIT_SAVE_XFER,   // Phase 5: FSAVE, transferring the state-frame payload out
        ST_WAIT_RESTORE_XFER // Phase 5: FRESTORE, transferring the state-frame payload in
    } state_t;

    state_t      state_r;
    logic        ca_r, dr_r;          // current Response primitive's CA/DR bits
    prim_id_t    prim_r;              // current Response primitive id
    logic        cond_tf_r;           // last Condition CIR evaluation result
    logic [7:0]  mask_r;              // multi-register move mask (Register Select CIR content)
    logic [2:0]  reg_idx_r;           // FP0-7 index, or ctrl_sel value, currently being transferred
    logic [1:0]  chunk_idx_r;         // 0=MSB..2=LSB for FP regs; unused (0) for ctrl regs
    logic [1:0]  chunks_left_r;       // remaining Operand CIR accesses for the CURRENT register
    logic        is_ctrl_reg_r;       // 1 = transferring an FPCR/FPSR/FPIAR register, 0 = an FPn
    logic        multi_ctrl_r;        // 1 = this WAIT_XFER dialog is a multi-control-register move

    // Phase 4c: opclass 010/011 (external operand <-> FPn) own state.
    // is_ctrl_reg_r==0 && state_r==ST_WAIT_XFER already uniquely
    // identifies this dialog (no other WAIT_XFER user has is_ctrl_reg_r
    // clear), so no separate "is this an EA transfer" flag is needed.
    logic [2:0] xfer_fmt_r;    // captured c_rx (data-format code) at command dispatch
    logic [95:0] xfer_stage_r; // RECEIVE: raw bytes staged as they arrive, converted on
                                // the last chunk. SUPPLY: pre-converted bytes, staged once
                                // at command dispatch, read out chunk by chunk.

    // Phase 5: FSAVE/FRESTORE state-frame dialog own state.
    logic [5:0] frame_words_left_r; // remaining Operand CIR longwords in the
                                     // current state-frame payload transfer
                                     // (up to 52, the 68882 Busy frame)
    logic [15:0] restore_fmt_r;     // format word latched from the last Restore CIR write
    logic        restore_valid_r;   // did that format word validate?
    logic        restore_is_null_r; // was it specifically the Null format word?

    // ── Register file ──────────────────────────────────────────────
    logic [2:0]  fp_sel;
    logic [1:0]  fp_chunk_idx;
    logic [31:0] fp_rd_data;
    logic [2:0]  fp_wr_sel_r;
    logic [1:0]  fp_wr_chunk_r;
    logic        fp_wr_en;
    logic [31:0] fp_wr_data;
    logic [1:0]  ctrl_sel;
    logic [31:0] ctrl_rd_data;
    logic [1:0]  ctrl_wr_sel_r;
    logic        ctrl_wr_en;
    logic [31:0] ctrl_wr_data;
    logic [31:0] fpsr_o;
    logic [31:0] fpcr_o;
    logic [31:0] fpiar_o;
    logic        all_zero_o;
    logic        null_reset_en;
    logic [2:0]  apu_a_sel, apu_b_sel;
    logic [95:0] apu_a_rd, apu_b_rd;
    logic        apu_wr_en;
    logic [2:0]  apu_wr_sel;
    logic [95:0] apu_wr_data;

    m68882_regfile u_regfile (
        .clk_4x, .rst_n,
        .fp_sel, .fp_chunk_idx, .fp_rd_data,
        .fp_wr_sel(fp_wr_sel_r), .fp_wr_chunk(fp_wr_chunk_r), .fp_wr_en, .fp_wr_data,
        .ctrl_sel, .ctrl_rd_data,
        .ctrl_wr_sel(ctrl_wr_sel_r), .ctrl_wr_en, .ctrl_wr_data,
        .fpsr_o, .fpcr_o, .fpiar_o, .all_zero_o, .null_reset_en,
        .apu_a_sel, .apu_b_sel, .apu_a_rd, .apu_b_rd,
        .apu_wr_en, .apu_wr_sel, .apu_wr_data
    );

    // fp_sel/fp_chunk_idx/ctrl_sel (READ side) are pure combinational
    // aliases of the dialog's own current register/chunk pointers --
    // always valid for SUPPLY-direction reads. fp_wr_sel_r/fp_wr_chunk_r/
    // ctrl_wr_sel_r (WRITE side) are instead captured via nonblocking
    // assignment in the same always_ff tick as fp_wr_en/ctrl_wr_en
    // themselves (see the CIR_OPERAND branch below) -- see
    // m68882_regfile.sv's own header comment for why these must be
    // separate ports, not the same live pointer.
    assign fp_sel       = reg_idx_r;
    assign fp_chunk_idx = chunk_idx_r;
    assign ctrl_sel     = reg_idx_r[1:0];

    // Command word field decode (combinational; only meaningful the tick
    // Command CIR is written)
    wire [2:0] c_opclass = cmd_opclass(d_in[31:16]);
    wire [2:0] c_rx      = cmd_rx(d_in[31:16]);
    wire [2:0] c_ry      = cmd_ry(d_in[31:16]);
    wire [6:0] c_ext     = cmd_ext(d_in[31:16]);
    wire [7:0] c_multi_mask = {c_ry[0], c_ext};

    // Phase 4a: register-to-register FADD/FSUB (opclass 000). Both
    // source operands are read combinationally every cycle (harmless --
    // the result is only ever LATCHED on the Command CIR's own ack_pulse,
    // gated by c_opclass/c_ext below) and the ADD/SUB core itself is pure
    // combinational logic (m68882_apu_pkg::fp_add_sub).
    assign apu_a_sel = c_rx; // opclass 000: RX = source FPm
    assign apu_b_sel = c_ry; // opclass 000: RY = destination FPn

    wire is_fadd = (c_ext == 7'h22);
    wire is_fsub = (c_ext == 7'h28);
    wire is_fmul = (c_ext == 7'h23);
    wire is_fdiv = (c_ext == 7'h20);
    wire is_fcmp = (c_ext == 7'h38);
    wire is_ftst = (c_ext == 7'h3A);
    wire is_fabs = (c_ext == 7'h18);
    wire is_fneg = (c_ext == 7'h1A);
    wire is_fsqrt = (c_ext == 7'h04);

    // Phase 5: state-frame format words (Section 6.4.2). The Null format
    // word (16'h0000) is the one value the M68000 coprocessor interface
    // itself defines and every coprocessor must recognize identically
    // (confirmed directly). The Idle/Busy version byte (0x1F) and the
    // size bytes (52/208 -- Idle=56 and Busy=212 TOTAL bytes each, minus
    // the 4-byte format-word+reserved-word header the manual's own "size
    // does not include the format word or reserved word" rule excludes)
    // are THIS PROJECT'S OWN choice, not confirmed against real 68882
    // silicon's own version-number value -- there is no internal
    // microarchitectural state in this project yet for the payload
    // itself to hold (Section 6.4.2's own Idle/Busy fields -- exceptional
    // operand latch, BIU flags, internal command/condition register
    // copies -- are all genuinely internal state, confirmed NOT to
    // include the visible FP0-7/FPCR/FPSR/FPIAR registers at all), so the
    // payload is a documented, zero-filled placeholder; only the
    // protocol SHAPE (format word, sizes, Null-vs-Idle-vs-Busy
    // classification, round-trip validation) is real.
    localparam logic [15:0] FRAME_NULL_FMT = 16'h0000;
    localparam logic [15:0] FRAME_IDLE_FMT = {8'h1F, 8'd52};
    localparam logic [15:0] FRAME_BUSY_FMT = {8'h1F, 8'd208};
    localparam logic [5:0]  FRAME_IDLE_WORDS = 6'd13; // 52 bytes / 4
    localparam logic [5:0]  FRAME_BUSY_WORDS = 6'd52; // 208 bytes / 4

    // FCMP (Table 4-13 $38): condition codes as if FPn-source were
    // computed (real fp_add_sub subtraction), but the result is
    // discarded -- FPn itself is never written. FTST ($3A): condition
    // codes from the SOURCE operand alone, no arithmetic at all.
    fpx_t ftst_x;
    logic ftst_z, ftst_n, ftst_i, ftst_nan;
    assign ftst_x   = unpack_fpx(apu_a_rd);
    assign ftst_z   = is_zero_fpx(ftst_x);
    assign ftst_n   = ftst_x.sign && !ftst_z;
    assign ftst_i   = is_inf_fpx(ftst_x);
    assign ftst_nan = is_nan_fpx(ftst_x);

    // FABS/FNEG (Table 4-13 $18/$1A): a plain sign-bit operation on the
    // SOURCE operand, no rounding/normalization needed at all.
    logic [95:0] absneg_result;
    assign absneg_result = is_fabs ? {1'b0, apu_a_rd[94:0]}
                                    : {!apu_a_rd[95], apu_a_rd[94:0]};

    logic [95:0] addsub_result, mul_result, div_result, sqrt_result;
    logic        addsub_z, addsub_n, addsub_i, addsub_nan, addsub_operr, addsub_ovfl, addsub_unfl, addsub_inex2;
    logic        mul_z, mul_n, mul_i, mul_nan, mul_operr, mul_ovfl, mul_unfl, mul_inex2;
    logic        div_z, div_n, div_i, div_nan, div_operr, div_dz, div_ovfl, div_unfl, div_inex2;
    logic        sqrt_z, sqrt_n, sqrt_i, sqrt_nan, sqrt_operr, sqrt_ovfl, sqrt_unfl, sqrt_inex2;
    logic [95:0] apu_result;
    logic        apu_flag_z, apu_flag_n, apu_flag_i, apu_flag_nan;
    logic        apu_flag_operr, apu_flag_dz, apu_flag_ovfl, apu_flag_unfl, apu_flag_inex2;

    // Phase 4d: SNAN is detected directly from the RAW input operands
    // (not a task output) -- Section 4.5.4.2: "If either operand to an
    // operation is a signaling NAN, then the SNAN bit is set." Applies
    // uniformly regardless of which arithmetic op is selected.
    wire apu_flag_snan = is_snan_fpx(unpack_fpx(apu_a_rd)) || is_snan_fpx(unpack_fpx(apu_b_rd));

    always_comb begin
        // FCMP reuses the identical subtraction fp_add_sub already
        // performs for FSUB (Section 4.5.5.1: FCMP sets condition codes
        // "as if" FPn-source were computed) -- only the register write
        // is skipped for FCMP, further down.
        fp_add_sub(apu_a_rd, apu_b_rd, (is_fsub || is_fcmp), round_mode_t'(fpcr_o[5:4]),
                   addsub_result, addsub_z, addsub_n, addsub_i, addsub_nan, addsub_operr,
                   addsub_ovfl, addsub_unfl, addsub_inex2);
        fp_mul(apu_a_rd, apu_b_rd, round_mode_t'(fpcr_o[5:4]),
               mul_result, mul_z, mul_n, mul_i, mul_nan, mul_operr, mul_ovfl, mul_unfl, mul_inex2);
        fp_div(apu_a_rd, apu_b_rd, round_mode_t'(fpcr_o[5:4]),
               div_result, div_z, div_n, div_i, div_nan, div_operr, div_dz, div_ovfl, div_unfl, div_inex2);
        fp_sqrt(apu_a_rd, round_mode_t'(fpcr_o[5:4]),
                sqrt_result, sqrt_z, sqrt_n, sqrt_i, sqrt_nan, sqrt_operr, sqrt_ovfl, sqrt_unfl, sqrt_inex2);

        apu_flag_dz = 1'b0;
        if (is_fmul) begin
            apu_result     = mul_result;
            apu_flag_z     = mul_z;
            apu_flag_n     = mul_n;
            apu_flag_i     = mul_i;
            apu_flag_nan   = mul_nan;
            apu_flag_operr = mul_operr;
            apu_flag_ovfl  = mul_ovfl;
            apu_flag_unfl  = mul_unfl;
            apu_flag_inex2 = mul_inex2;
        end else if (is_fdiv) begin
            apu_result     = div_result;
            apu_flag_z     = div_z;
            apu_flag_n     = div_n;
            apu_flag_i     = div_i;
            apu_flag_nan   = div_nan;
            apu_flag_operr = div_operr;
            apu_flag_dz    = div_dz;
            apu_flag_ovfl  = div_ovfl;
            apu_flag_unfl  = div_unfl;
            apu_flag_inex2 = div_inex2;
        end else if (is_fabs || is_fneg) begin
            apu_result     = absneg_result;
            apu_flag_z     = is_zero_fpx(unpack_fpx(absneg_result));
            apu_flag_n     = absneg_result[95] && !apu_flag_z;
            apu_flag_i     = is_inf_fpx(unpack_fpx(absneg_result));
            apu_flag_nan   = is_nan_fpx(unpack_fpx(absneg_result));
            apu_flag_operr = 1'b0;
            apu_flag_ovfl  = 1'b0;
            apu_flag_unfl  = 1'b0;
            apu_flag_inex2 = 1'b0;
        end else if (is_ftst) begin
            apu_result     = apu_a_rd; // unused (FTST never writes a register)
            apu_flag_z     = ftst_z;
            apu_flag_n     = ftst_n;
            apu_flag_i     = ftst_i;
            apu_flag_nan   = ftst_nan;
            apu_flag_operr = 1'b0;
            apu_flag_ovfl  = 1'b0;
            apu_flag_unfl  = 1'b0;
            apu_flag_inex2 = 1'b0;
        end else if (is_fsqrt) begin
            apu_result     = sqrt_result;
            apu_flag_z     = sqrt_z;
            apu_flag_n     = sqrt_n;
            apu_flag_i     = sqrt_i;
            apu_flag_nan   = sqrt_nan;
            apu_flag_operr = sqrt_operr;
            apu_flag_ovfl  = sqrt_ovfl;
            apu_flag_unfl  = sqrt_unfl;
            apu_flag_inex2 = sqrt_inex2;
        end else begin
            // FADD, FSUB, and FCMP (condition codes only, see above)
            apu_result     = addsub_result;
            apu_flag_z     = addsub_z;
            apu_flag_n     = addsub_n;
            apu_flag_i     = addsub_i;
            apu_flag_nan   = addsub_nan;
            apu_flag_operr = addsub_operr;
            apu_flag_ovfl  = addsub_ovfl;
            apu_flag_unfl  = addsub_unfl;
            apu_flag_inex2 = addsub_inex2;
        end
    end

    // ── Phase 4c: external-operand format conversion ────────────────────
    //
    // SUPPLY (opclass 011, FPn -> external): the source register
    // (apu_b_rd, since apu_b_sel==c_ry, and reg_idx_r is ALSO set to
    // c_ry for this opclass -- the same register) is converted to the
    // target format the moment the Command CIR is written, staged into
    // xfer_stage_r, and then just read out chunk by chunk -- exactly
    // like the pre-Phase-4c raw-passthrough path did, except the value
    // staged is now a real conversion instead of the raw register bits.
    logic [31:0] supply_int32;
    logic [31:0] supply_single;
    logic [63:0] supply_double;
    logic        supply_int32_operr, supply_single_operr, supply_double_operr;
    logic [95:0] supply_staged;

    always_comb begin
        ext_to_int32(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_int32, supply_int32_operr);
        ext_to_single(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_single, supply_single_operr);
        ext_to_double(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_double, supply_double_operr);

        unique case (c_rx)
            FMT_L:   supply_staged = {supply_int32, 64'h0};
            FMT_S:   supply_staged = {supply_single, 64'h0};
            FMT_D:   supply_staged = {supply_double, 32'h0};
            FMT_X:   supply_staged = apu_b_rd; // native format, pure passthrough
            default: supply_staged = 96'h0; // W/B/P: not yet implemented (Phase 4c scope)
        endcase
    end

    // RECEIVE (opclass 010, external -> FPn): raw chunks are staged as
    // they arrive; receive_assembled is what xfer_stage_r WOULD hold
    // once the CURRENT chunk (d_in) is folded in, computed combinationally
    // so the conversion on the FINAL chunk doesn't have to wait a tick
    // for xfer_stage_r's own registered update to catch up.
    logic [95:0] receive_assembled;
    always_comb begin
        receive_assembled = xfer_stage_r;
        unique case (chunk_idx_r)
            2'd0: receive_assembled[95:64] = d_in;
            2'd1: receive_assembled[63:32] = d_in;
            2'd2: receive_assembled[31:0]  = d_in;
            default: ;
        endcase
    end

    logic [95:0] receive_int32_ext, receive_single_ext, receive_double_ext;
    always_comb begin
        int32_to_ext(receive_assembled[95:64], receive_int32_ext);
        single_to_ext(receive_assembled[95:64], receive_single_ext);
        double_to_ext(receive_assembled[95:32], receive_double_ext);
    end

    logic [95:0] receive_converted;
    always_comb begin
        unique case (xfer_fmt_r)
            FMT_L:   receive_converted = receive_int32_ext;
            FMT_S:   receive_converted = receive_single_ext;
            FMT_D:   receive_converted = receive_double_ext;
            FMT_X:   receive_converted = receive_assembled; // native format, pure passthrough
            default: receive_converted = 96'h0; // W/B/P: not yet implemented
        endcase
    end

    // Condition CIR predicate field + FPSR Z bit (combinational)
    wire [5:0] cond_pred = d_in[21:16];
    wire       fpsr_z    = fpsr_o[26];
    logic      cond_tf_next;
    always_comb begin
        unique case (cond_pred)
            6'b000001: cond_tf_next = fpsr_z;   // EQ
            6'b001110: cond_tf_next = !fpsr_z;  // NE
            default:   cond_tf_next = 1'b0;     // other 30 predicates: not yet implemented
        endcase
    end

    function automatic logic [1:0] chunks_for_bytes(int unsigned nbytes);
        if (nbytes <= 4)      return 2'd1;
        else if (nbytes <= 8) return 2'd2;
        else                  return 2'd3;
    endfunction

    // Priority search: lowest bit index >= start that is set in mask,
    // or 8 if none. Avoids a `break` inside always_ff -- pure function.
    function automatic logic [3:0] first_set_from(logic [7:0] mask, logic [3:0] start);
        logic [3:0] result;
        result = 4'd8;
        for (int i = 7; i >= 0; i--)
            if (i >= int'(start) && mask[i])
                result = i[3:0];
        return result;
    endfunction

    logic [3:0] next_fp_idx;
    logic [3:0] next_ctrl_idx;
    logic [3:0] first_ctrl_idx_from_rx;

    // The command word's own RX field is (inferred) bit2=FPCR/bit1=FPSR/
    // bit0=FPIAR, matching the confirmed FPCR-first transfer order
    // MSB-to-LSB. m68882_regfile.sv's own ctrl_sel indexing is the
    // OPPOSITE direction (0=FPCR/1=FPSR/2=FPIAR, matching that same
    // order but counted from 0 upward). mask_r/reg_idx_r use the
    // ctrl_sel convention throughout this module, so the RX field's own
    // bit order must be reversed once, here, at the point of decode --
    // found via a real simulation mismatch (an FPCR-only select
    // resolved to ctrl_sel index 2, i.e. FPIAR, instead of 0).
    wire [7:0] c_ctrl_mask = {5'b0, c_rx[0], c_rx[1], c_rx[2]};
    assign next_fp_idx   = first_set_from(mask_r, {1'b0, reg_idx_r} + 4'd1);
    assign next_ctrl_idx = first_set_from({5'b0, mask_r[2:0]}, {1'b0, reg_idx_r} + 4'd1);
    assign first_ctrl_idx_from_rx = first_set_from(c_ctrl_mask, 4'd0);

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            state_r       <= ST_IDLE;
            ca_r          <= 1'b0;
            dr_r          <= 1'b0;
            prim_r        <= PRIM_NULL;
            cond_tf_r     <= 1'b0;
            mask_r        <= 8'h00;
            reg_idx_r     <= 3'h0;
            chunk_idx_r   <= 2'h0;
            chunks_left_r <= 2'h0;
            is_ctrl_reg_r <= 1'b0;
            multi_ctrl_r  <= 1'b0;
            xfer_fmt_r    <= 3'h0;
            xfer_stage_r  <= 96'h0;
            frame_words_left_r <= 6'h0;
            restore_fmt_r      <= 16'h0;
            restore_valid_r    <= 1'b0;
            restore_is_null_r  <= 1'b0;
            null_reset_en <= 1'b0;
            fp_wr_en      <= 1'b0;
            fp_wr_sel_r   <= 3'h0;
            fp_wr_chunk_r <= 2'h0;
            ctrl_wr_en    <= 1'b0;
            ctrl_wr_sel_r <= 2'h0;
            apu_wr_en     <= 1'b0;
            apu_wr_sel    <= 3'h0;
            apu_wr_data   <= '0;
        end else begin
            fp_wr_en   <= 1'b0;
            ctrl_wr_en <= 1'b0;
            apu_wr_en  <= 1'b0;
            null_reset_en <= 1'b0;

            if (abort) begin
                state_r <= ST_IDLE;
                ca_r    <= 1'b0;
                prim_r  <= PRIM_NULL;
            end else if (ack_pulse) begin
                unique case (cyc_sel)

                    // A Condition CIR write folds its TF result into the
                    // NEXT Response CIR read's own Null payload
                    // (cond_tf_r, Section 7.2.7: "the null (CA=0,TF=x)
                    // primitive"). Reading it once consumes it -- without
                    // this, cond_tf_r would keep bleeding into every
                    // LATER, unrelated Null response until the next
                    // Condition CIR write overwrites it.
                    CIR_RESPONSE: cond_tf_r <= 1'b0;

                    CIR_COMMAND: if (state_r == ST_IDLE) begin
                        unique case (c_opclass)
                            3'b000: begin // FPm to FPn, register-to-register: no external
                                           // transfer needed (Table 4-13 Note 1: the first
                                           // primitive issued is Null even for a genuine
                                           // arithmetic op, since both operands are already
                                           // on-chip). Phase 4a: FADD ($22) and FSUB ($28)
                                           // are computed for real here; every other
                                           // extension-field opcode is still a no-op stub
                                           // (documented in m68882_apu.sv/plan.md).
                                ca_r    <= 1'b0;
                                prim_r  <= PRIM_NULL;
                                state_r <= ST_IDLE;
                                if (is_fadd || is_fsub || is_fmul || is_fdiv ||
                                    is_fabs || is_fneg || is_fcmp || is_ftst || is_fsqrt) begin
                                    // FCMP/FTST only ever update condition codes --
                                    // the destination register is never written
                                    // (Section 4.5.5.1: FCMP compares "as if"
                                    // FPn-source were computed, but FPn itself is
                                    // unaffected; FTST likewise never writes).
                                    if (!is_fcmp && !is_ftst) begin
                                        apu_wr_en   <= 1'b1;
                                        apu_wr_sel  <= c_ry;
                                        apu_wr_data <= apu_result;
                                    end
                                    ctrl_wr_en    <= 1'b1;
                                    ctrl_wr_sel_r <= 2'd1; // FPSR
                                    // Phase 4d: CC byte and the EXC (exception-status)
                                    // byte are both overwritten FRESH each op (Section
                                    // 4.5.5.1/2.3.3 -- EXC is not sticky). The AEXC
                                    // (accrued-exception) byte instead uses the manual's
                                    // own confirmed OR-accumulation formulas (Section
                                    // 2.3.4, Figure 2-7):
                                    //   AEXC(IOP)  |= EXC(BSUN|SNAN|OPERR)
                                    //   AEXC(OVFL) |= EXC(OVFL)
                                    //   AEXC(UNFL) |= EXC(UNFL & INEX2)
                                    //   AEXC(DZ)   |= EXC(DZ)
                                    //   AEXC(INEX) |= EXC(INEX1|INEX2|OVFL)
                                    // BSUN and INEX1 are not yet implemented (BSUN needs
                                    // the still-stubbed conditional-predicate set;
                                    // INEX1 only applies to Packed-Decimal input, not yet
                                    // implemented -- Phase 4c) -- both wired as a
                                    // constant 0 rather than silently omitted from the
                                    // formulas above.
                                    ctrl_wr_data  <= {4'b0, apu_flag_n, apu_flag_z, apu_flag_i, apu_flag_nan,
                                                       fpsr_o[23:16],
                                                       1'b0, apu_flag_snan, apu_flag_operr, apu_flag_ovfl,
                                                       apu_flag_unfl, apu_flag_dz, apu_flag_inex2, 1'b0,
                                                       (fpsr_o[7] | apu_flag_snan | apu_flag_operr),
                                                       (fpsr_o[6] | apu_flag_ovfl),
                                                       (fpsr_o[5] | (apu_flag_unfl && apu_flag_inex2)),
                                                       (fpsr_o[4] | apu_flag_dz),
                                                       (fpsr_o[3] | apu_flag_inex2 | apu_flag_ovfl),
                                                       fpsr_o[2:0]};
                                end
                            end
                            3'b010: if (c_rx == 3'b111) begin // move constant to FPn
                                ca_r    <= 1'b0;
                                prim_r  <= PRIM_NULL;
                                state_r <= ST_IDLE;
                            end else begin // external operand to FPn
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b0; // host supplies data (RECEIVE)
                                prim_r        <= PRIM_EVAL_EA;
                                is_ctrl_reg_r <= 1'b0;
                                multi_ctrl_r  <= 1'b0;
                                reg_idx_r     <= c_ry;
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= chunks_for_bytes(fmt_bytes(c_rx));
                                xfer_fmt_r    <= c_rx;
                                xfer_stage_r  <= 96'h0; // Phase 4c: format-conversion staging buffer
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b011: begin // FPm to external destination
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b1; // FPCP supplies data (SUPPLY)
                                prim_r        <= PRIM_EVAL_EA;
                                is_ctrl_reg_r <= 1'b0;
                                multi_ctrl_r  <= 1'b0;
                                reg_idx_r     <= c_ry; // source FPm
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= chunks_for_bytes(fmt_bytes(c_rx));
                                xfer_fmt_r    <= c_rx;
                                xfer_stage_r  <= supply_staged; // Phase 4c: pre-converted at dispatch
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b100: begin // move to system control register(s)
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b0; // RECEIVE
                                prim_r        <= PRIM_XFER_SINGLE;
                                is_ctrl_reg_r <= 1'b1;
                                multi_ctrl_r  <= 1'b1;
                                mask_r        <= c_ctrl_mask;
                                reg_idx_r     <= first_ctrl_idx_from_rx[2:0];
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= 2'd1;
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b101: begin // move system control register(s) to memory
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b1; // SUPPLY
                                prim_r        <= PRIM_XFER_SINGLE;
                                is_ctrl_reg_r <= 1'b1;
                                multi_ctrl_r  <= 1'b1;
                                mask_r        <= c_ctrl_mask;
                                reg_idx_r     <= first_ctrl_idx_from_rx[2:0];
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= 2'd1;
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b110: begin // move multiple to FP data registers
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b0; // RECEIVE
                                prim_r        <= PRIM_XFER_MULTI;
                                is_ctrl_reg_r <= 1'b0;
                                mask_r        <= c_multi_mask;
                                state_r       <= ST_WAIT_REGSEL;
                            end
                            3'b111: begin // move multiple from FP data registers
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b1; // SUPPLY
                                prim_r        <= PRIM_XFER_MULTI;
                                is_ctrl_reg_r <= 1'b0;
                                mask_r        <= c_multi_mask;
                                state_r       <= ST_WAIT_REGSEL;
                            end
                            default: ;
                        endcase
                    end

                    CIR_CONDITION: if (state_r == ST_IDLE) begin
                        ca_r      <= 1'b0;
                        prim_r    <= PRIM_NULL;
                        cond_tf_r <= cond_tf_next;
                        state_r   <= ST_IDLE;
                    end

                    CIR_REGSELECT: if (state_r == ST_WAIT_REGSEL && !cyc_write) begin
                        logic [3:0] first_idx;
                        first_idx = first_set_from(mask_r, 4'd0);
                        if (first_idx == 4'd8) begin
                            ca_r    <= 1'b0;
                            prim_r  <= PRIM_NULL;
                            state_r <= ST_IDLE;
                        end else begin
                            reg_idx_r     <= first_idx[2:0];
                            chunk_idx_r   <= 2'h0;
                            chunks_left_r <= 2'd3;
                            state_r       <= ST_WAIT_MULTI_XFER;
                        end
                    end

                    // Phase 5: FSAVE dialog (Section 7.2.3). A read is
                    // legal at any time EXCEPT during an already-active
                    // state-frame transfer; unlike every other CIR this
                    // module owns, it deliberately does NOT gate on
                    // state_r==ST_IDLE, since capturing a genuinely BUSY
                    // dialog is exactly what the Busy frame is for.
                    CIR_SAVE: if (!cyc_write &&
                                  state_r != ST_WAIT_SAVE_XFER && state_r != ST_WAIT_RESTORE_XFER) begin
                        if (all_zero_o && state_r == ST_IDLE) begin
                            // Null: "the FPCP is in the reset state, and
                            // the next expected access is to the command
                            // or condition CIR" -- no Operand CIR transfer.
                            state_r <= ST_IDLE;
                        end else if (state_r == ST_IDLE) begin
                            frame_words_left_r <= FRAME_IDLE_WORDS;
                            state_r            <= ST_WAIT_SAVE_XFER;
                        end else begin
                            // a genuine dialog was in progress -- Busy frame
                            frame_words_left_r <= FRAME_BUSY_WORDS;
                            state_r            <= ST_WAIT_SAVE_XFER;
                        end
                    end

                    // Phase 5: FRESTORE dialog (Section 7.2.4). The host
                    // writes the format word first; the FOLLOWING read
                    // reports validation success/failure and (if valid)
                    // advances the dialog.
                    CIR_RESTORE: begin
                        if (cyc_write && state_r == ST_IDLE) begin
                            restore_fmt_r     <= d_in[31:16];
                            restore_is_null_r <= (d_in[31:16] == FRAME_NULL_FMT);
                            restore_valid_r   <= (d_in[31:16] == FRAME_NULL_FMT) ||
                                                  (d_in[31:16] == FRAME_IDLE_FMT) ||
                                                  (d_in[31:16] == FRAME_BUSY_FMT);
                        end else if (!cyc_write && state_r == ST_IDLE) begin
                            if (restore_valid_r) begin
                                if (restore_is_null_r) begin
                                    // Section 6.4.2.1: restoring the Null
                                    // frame resets the WHOLE programmer's
                                    // model, not just the internal state.
                                    null_reset_en <= 1'b1;
                                    state_r       <= ST_IDLE;
                                end else begin
                                    frame_words_left_r <= (restore_fmt_r == FRAME_BUSY_FMT)
                                                           ? FRAME_BUSY_WORDS : FRAME_IDLE_WORDS;
                                    state_r            <= ST_WAIT_RESTORE_XFER;
                                end
                            end else begin
                                // invalid format word -- stay idle; a real
                                // host is expected to write an abort to the
                                // Control CIR next (Section 7.2.4), which
                                // this project's own Control CIR (Phase 3's
                                // unconditional-abort model) already handles
                                // unconditionally regardless of dialog state.
                                state_r <= ST_IDLE;
                            end
                        end
                    end

                    CIR_OPERAND: if (state_r == ST_WAIT_SAVE_XFER || state_r == ST_WAIT_RESTORE_XFER) begin
                        // Phase 5: FSAVE/FRESTORE payload transfer. Fully
                        // self-contained (its own state-advance logic, not
                        // shared with the chunks_left_r-based block below)
                        // since frame_words_left_r is a differently-sized
                        // counter with no relationship to chunks_left_r's
                        // own stale value from whatever dialog last used
                        // it. Placeholder payload both directions (see
                        // FRAME_IDLE_FMT's own header comment) -- reads
                        // return zero (driven combinationally below),
                        // writes are accepted and discarded; either way
                        // there is no real internal state yet to latch.
                        if (frame_words_left_r <= 6'd1) state_r <= ST_IDLE;
                        else frame_words_left_r <= frame_words_left_r - 6'd1;
                    end else if (state_r == ST_WAIT_XFER || state_r == ST_WAIT_MULTI_XFER) begin
                        if (!dr_r) begin
                            // RECEIVE: latch host-supplied data into the register file
                            if (is_ctrl_reg_r) begin
                                ctrl_wr_en    <= 1'b1;
                                ctrl_wr_sel_r <= reg_idx_r[1:0]; // pre-advance value
                                ctrl_wr_data  <= d_in;
                            end else if (state_r == ST_WAIT_XFER) begin
                                // Phase 4c: opclass 010 external-operand-to-FPn --
                                // stage the raw chunk every access; on the LAST
                                // chunk, also issue the real converted value (a
                                // whole-register write, reusing the APU's own
                                // write port rather than the chunked fp_wr_en
                                // path FMOVEM/ctrl-register moves use below).
                                xfer_stage_r <= receive_assembled;
                                if (chunks_left_r <= 2'd1) begin
                                    apu_wr_en   <= 1'b1;
                                    apu_wr_sel  <= reg_idx_r; // pre-advance value
                                    apu_wr_data <= receive_converted;
                                end
                            end else begin
                                // opclass 110 move-multiple-to-FPn: each chunk
                                // is already a genuine native 32-bit slice of
                                // the internal 96-bit format -- no conversion.
                                fp_wr_en      <= 1'b1;
                                fp_wr_sel_r   <= reg_idx_r;      // pre-advance value
                                fp_wr_chunk_r <= chunk_idx_r;    // pre-advance value
                                fp_wr_data    <= d_in;
                            end
                        end

                        if (chunks_left_r > 2'd1) begin
                            chunks_left_r <= chunks_left_r - 2'd1;
                            chunk_idx_r   <= chunk_idx_r + 2'd1;
                        end else if (state_r == ST_WAIT_MULTI_XFER) begin
                            // this FPn's own transfer is done -- advance to
                            // the next selected FPn, if any (ascending order)
                            if (next_fp_idx == 4'd8) begin
                                ca_r    <= 1'b0;
                                prim_r  <= PRIM_NULL;
                                state_r <= ST_IDLE;
                            end else begin
                                reg_idx_r     <= next_fp_idx[2:0];
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= 2'd3;
                            end
                        end else if (multi_ctrl_r) begin
                            // this control register's own transfer is done --
                            // advance to the next selected one, if any
                            if (next_ctrl_idx == 4'd8) begin
                                ca_r    <= 1'b0;
                                prim_r  <= PRIM_NULL;
                                state_r <= ST_IDLE;
                            end else begin
                                reg_idx_r     <= next_ctrl_idx[2:0];
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= 2'd1;
                            end
                        end else begin
                            // single EA transfer complete
                            ca_r    <= 1'b0;
                            prim_r  <= PRIM_NULL;
                            state_r <= ST_IDLE;
                        end
                    end

                    default: ;
                endcase
            end
        end
    end

    // ── Combinational read/data-bus drive ──────────────────────────────
    logic [12:0] response_payload;
    assign response_payload = (prim_r == PRIM_NULL) ? {12'b0, cond_tf_r} : prim_r;

    // dr_r is only meaningful while a real (non-Null) primitive is
    // pending -- it's set once at dialog start and never explicitly
    // cleared at every one of the several "revert to Null" transitions
    // above, so it can go stale (still reflecting the JUST-FINISHED
    // dialog's own direction) by the time the NEXT Response CIR read
    // happens. Gate it here, in one place, rather than adding a
    // `dr_r<=0` to every transition site.
    logic dr_eff;
    assign dr_eff = (prim_r == PRIM_NULL) ? 1'b0 : dr_r;

    always_comb begin
        d_out = 32'h0;
        d_oe  = 1'b0;

        unique case (cyc_sel)
            CIR_RESPONSE: begin
                d_oe  = 1'b1;
                d_out = {response_word(ca_r, 1'b0, dr_eff, response_payload), 16'h0};
            end
            CIR_REGSELECT: begin
                d_oe  = 1'b1;
                d_out = {mask_r, 24'h0}; // MS 8 bits = mask, rest zero (Section 7.2.9)
            end
            CIR_SAVE: begin
                // Phase 5: the format word for whichever frame type
                // applies right now (Null/Idle/Busy) -- combinational,
                // since the actual dialog state transition happens in
                // the always_ff block above; this just needs to present
                // the correct value AT the ack tick.
                d_oe = 1'b1;
                if (state_r != ST_IDLE) begin
                    d_out = {FRAME_BUSY_FMT, 16'h0};
                end else if (all_zero_o) begin
                    d_out = {FRAME_NULL_FMT, 16'h0};
                end else begin
                    d_out = {FRAME_IDLE_FMT, 16'h0};
                end
            end
            CIR_RESTORE: if (!cyc_write) begin
                // Phase 5: echo the validated format word back (success),
                // or an "invalid format" marker (Section 7.2.4) -- this
                // project's own placeholder for that marker, 16'hFFFF,
                // is NOT confirmed against a real numeric code from the
                // manual (not located in this project's own extraction).
                // BUG FOUND VIA SIMULATION: this branch originally
                // asserted d_oe unconditionally, including during a
                // WRITE cycle -- genuine bus contention against the
                // host's own driven write data (the host's data showed
                // as X on the D16-D31 half specifically, exactly where
                // this readback value collided with it).
                d_oe  = 1'b1;
                d_out = {(restore_valid_r ? restore_fmt_r : 16'hFFFF), 16'h0};
            end
            CIR_OPERAND: if (state_r == ST_WAIT_SAVE_XFER) begin
                // Phase 5: placeholder payload (see FRAME_IDLE_FMT's own
                // header comment) -- always zero.
                d_oe  = 1'b1;
                d_out = 32'h0;
            end else if (dr_r && (state_r == ST_WAIT_XFER || state_r == ST_WAIT_MULTI_XFER)) begin
                d_oe  = 1'b1;
                if (is_ctrl_reg_r) begin
                    d_out = ctrl_rd_data;
                end else if (state_r == ST_WAIT_XFER) begin
                    // Phase 4c: opclass 011 FPn-to-external -- read out the
                    // pre-converted staging buffer (supply_staged, latched
                    // at command dispatch) chunk by chunk, not the raw
                    // register value fp_rd_data would present.
                    unique case (chunk_idx_r)
                        2'd0: d_out = xfer_stage_r[95:64];
                        2'd1: d_out = xfer_stage_r[63:32];
                        2'd2: d_out = xfer_stage_r[31:0];
                        default: d_out = 32'h0;
                    endcase
                end else begin
                    // opclass 111 move-multiple-from-FPn: native format,
                    // no conversion.
                    d_out = fp_rd_data;
                end
            end
            default: ;
        endcase
    end

endmodule
