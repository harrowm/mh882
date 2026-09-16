`timescale 1ns/1ps
`default_nettype none

import m68882_cir_pkg::*;
import m68882_apu_pkg::*;

// MC68882 instruction-dialog protocol.
//
// Owns the Response ($00), Command ($0A), Condition ($0E), Operand
// ($10), Register Select ($14), and (Phase 6) Instruction Address ($18)
// CIRs, and drives m68882_regfile. Control/Operand-Address stay in
// m68882_cir.sv -- this module takes an `abort` pulse (Control CIR AB
// bit) and an `xa_pulse` (Control CIR XA bit) from there.
//
// ── Phase 6: genuine BIU->CU->APU pipeline overlap ──────────────────
//
// This is the real 68882 differentiator over the 68881 (Section 5.1.1,
// confirmed directly, already documented in plan.md): the CU can accept
// and fully process a SECOND instruction (an FMOVE-class register move,
// or the register-to-register EA/operand-capture half of a second
// arithmetic instruction) while the APU is still busy computing a FIRST
// instruction's real arithmetic result. This module now implements that
// as a genuine 2-deep pipeline, not a documented deferral:
//
//   - slotA_*/apu_busy_cnt_r: the instruction currently EXECUTING in the
//     APU (FADD/FSUB/FMUL/FDIV/FSQRT only -- the only opcodes needing
//     real multi-cycle APU work; Table 5-1's own "Minimum-Concurrency
//     Instructions" list, FMOVE/FMOVECR/FMOVEM/FTST, plus this project's
//     own FABS/FNEG/FCMP -- trivial sign-bit ops and a shared-adder
//     compare needing no pipeline stage of their own -- complete
//     INSTANTLY regardless of slot A/B occupancy, exactly matching that
//     table's own concurrency guarantee).
//   - slotB_*: a SECOND arithmetic instruction staged by the CU while
//     slot A is still busy -- captured operands/opcode/destination/
//     rounding mode, waiting for slot A to free. This is the literal
//     "genuine 2-deep pipelining, not just faster single-instruction
//     dispatch" plan.md's own Phase 0 research already flagged.
//   - A THIRD arithmetic dispatch arriving while BOTH slots are already
//     full is rejected (Section 7.2.6, "busy APU, defer command word"):
//     the Response primitive reports CA=1 (busy) instead of accepting
//     the command word, and a real host is expected to retry the SAME
//     Command+Instruction-Address write sequence later.
//   - apu_latency(): per-opcode cycle counts are THIS PROJECT'S OWN
//     placeholder convention (ADD/SUB/CMP=50, MUL=100, DIV=200, SQRT=250,
//     deliberately spread wide -- generous enough that the genuine
//     2-deep pipeline overlap window is comfortably observable in
//     simulation against this project's own real CIR bus-dialog
//     overhead, not just nominally present) -- not
//     confirmed against a real 68882 timing table (none was located in
//     this project's own manual extraction, unlike MH030's own
//     extensively cross-checked S-state timing tables) -- but the
//     PIPELINE MECHANISM itself (2 real, independently-tracked
//     instructions in flight, the exact CU-free/APU-busy overlap window
//     a real host program can exploit, the busy-reject on a 3rd) is
//     real, not a stub.
//
// ── Phase 6: mandatory Instruction Address CIR ──────────────────────
//
// Section 5.1.1/plan.md: unlike the 68881 (optional PC transfer), the
// 68882 REQUIRES the main processor to write the Instruction Address CIR
// as the very next access after every Command CIR write, before doing
// anything else (including reading Response CIR) -- because the 68882
// has 3 real per-pipeline-stage instruction-address registers and needs
// to know which instruction it's even dispatching before it can decode
// it. Modeled via a new ST_WAIT_IADDR dialog state: Command CIR write
// only captures the raw command-word fields (cmd_opclass_r/cmd_rx_r/
// cmd_ry_r/cmd_ext_r/cmd_multi_mask_r/cmd_round_r) and transitions here;
// the REAL opclass dispatch decode (everything the old Phase 3-5 code
// did directly on the Command CIR write) now fires on the FOLLOWING
// Instruction Address CIR write instead. Any OTHER CIR access while
// state_r==ST_WAIT_IADDR pulses proto_violation_r -- a real, testable
// stand-in for Coprocessor Protocol Violation (vector 13), which is
// architecturally raised by the MAIN PROCESSOR, not this chip (MH030's
// own CLAUDE.md documents that side as out of scope there for the same
// "needs a real attached coprocessor" reason this project exists to
// eventually close -- see plan.md's own companion-integration section).
// This mandatory-IA requirement applies to Command CIR dispatch only,
// not Condition CIR (a materially different, main-processor-side
// cpBcc/cpScc dialog this project models only minimally -- see that
// CIR's own header comment below).
//
// CLAUDE.md's own Response Primitive Protocol section documents the PC
// bit (bit14) as "requests the host write the Instruction Address CIR"
// -- i.e. a CONDITIONAL, per-primitive request. This project instead
// enforces the write as UNCONDITIONALLY mandatory after every Command
// CIR write (matching plan.md's own Phase 0 research text precisely),
// which structurally can't be discovered by first reading Response CIR
// for the PC bit (that read is itself gated behind ST_WAIT_IADDR). PC is
// therefore left permanently driven 0 here -- an explicit, documented
// simplification of a real tension between two believable readings of
// the source material, not an oversight.
//
// cu_instr_addr_r is the real "CU-stage" instruction-address register;
// FPIAR (m68882_regfile.sv) is the real "APU-stage" one, confirmed by
// plan.md's own Phase 0 research to be THE one of the 68882's 3
// per-pipeline-stage instruction-address registers that's actually
// programmer-visible -- it auto-loads (via the regfile's own dedicated
// fpiar_auto_wr_en port, not the shared ctrl_wr_en path) the moment an
// instruction genuinely enters the APU (immediate dispatch into a free
// slot A, or slot B's own later promotion into slot A), NOT at mere
// slot-B staging time. FMOVE-class (opclass 010/011/100/101/110/111)
// dispatches never touch FPIAR at all -- correct, since they never touch
// the APU (Section 5.1.1's own "CU relieves the APU of a significant
// work load" framing). The 3rd, main-processor-side "BIU stage" address
// is outside this coprocessor's own boundary (there is no BIU-stage
// register here -- the main processor's own PC is never visible to this
// chip beyond whatever address it once chooses to write to Instruction
// Address CIR).
//
// ── Phase 6: exception-primitive persistence until FSAVE ────────────
//
// Section 7.5.4.2, confirmed directly and already documented in
// plan.md: when a completing APU instruction (slot A commit) trips an
// exception FPCR's own ENABLE byte has asked to be trapped, the Response
// primitive becomes Take-Mid-Instruction-Exception (PRIM_TAKE_MID) and
// STAYS that way across every subsequent Response CIR read -- "the
// write-exception-acknowledge (XA) operation does NOT itself cause a
// null primitive -- only the exception handler's own FSAVE changes the
// primitive back to null." xa_pulse (from m68882_cir.sv's Control CIR
// XA bit) is therefore accepted but deliberately has NO effect on
// prim_r/ca_r here -- only a completing FSAVE dialog (CIR_SAVE/CIR_OPERAND
// below) clears a pending TAKE_MID/TAKE_PRE primitive. BSUN/pre-
// instruction exceptions (PRIM_TAKE_PRE) are not yet raised anywhere
// (BSUN itself needs the still-stubbed conditional-predicate set) --
// only the clearing side of PRIM_TAKE_PRE is wired, for whenever a
// future phase starts raising it.
//
// Everything else retains its original phase-by-phase scope boundary
// (see plan.md): W/B/P operand-format conversion still unimplemented;
// the 30 stubbed Condition CIR predicates; arithmetic-with-a-memory-
// operand (opclass 010 with a non-FMOVE extension field) still performs
// a plain FMOVE regardless of the requested op -- a pre-existing gap
// noticed, not introduced, while building this phase, documented in
// plan.md rather than silently left unmentioned.

module m68882_proto (
    input  logic clk_4x,
    input  logic rst_n,

    input  logic      cyc_ack,
    input  logic      cyc_write,
    input  cir_sel_t  cyc_sel,
    input  logic      abort,        // pulse from m68882_cir.sv's Control CIR AB bit
    input  logic      xa_pulse,     // pulse from m68882_cir.sv's Control CIR XA bit

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

    // ── Dialog state ─────────────────────────────────────────────────
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_WAIT_IADDR,        // Phase 6: mandatory Instruction Address CIR before dispatch decode
        ST_WAIT_XFER,         // single EA / FPcr transfer(s) in progress
        ST_WAIT_REGSEL,       // multi-register move: mask ready to be read
        ST_WAIT_MULTI_XFER,   // multi-register move: transferring longwords
        ST_WAIT_SAVE_XFER,    // FSAVE: transferring the state-frame payload out
        ST_WAIT_RESTORE_XFER  // FRESTORE: transferring the state-frame payload in
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

    // opclass 010/011 (external operand <-> FPn) own state.
    logic [2:0] xfer_fmt_r;    // captured data-format code at command dispatch
    logic [95:0] xfer_stage_r; // RECEIVE: raw bytes staged as they arrive, converted on
                                // the last chunk. SUPPLY: pre-converted bytes, staged once
                                // at command dispatch, read out chunk by chunk.

    // FSAVE/FRESTORE state-frame dialog own state.
    logic [5:0] frame_words_left_r; // remaining Operand CIR longwords in the
                                     // current state-frame payload transfer
                                     // (up to 52, the 68882 Busy frame)
    logic [15:0] restore_fmt_r;     // format word latched from the last Restore CIR write
    logic        restore_valid_r;   // did that format word validate?
    logic        restore_is_null_r; // was it specifically the Null format word?

    // ── Phase 6: mandatory Instruction Address CIR + registered
    // command-word capture (dispatch decode now fires on the IA write,
    // not the Command write itself -- see header comment) ──────────────
    logic [2:0] cmd_opclass_r;
    logic [2:0] cmd_rx_r;
    logic [2:0] cmd_ry_r;
    logic [6:0] cmd_ext_r;
    logic [7:0] cmd_multi_mask_r;
    logic [1:0] cmd_round_r;
    logic [31:0] cu_instr_addr_r;   // CU-stage instruction address
    logic        proto_violation_r; // one-tick pulse: mandatory IA write was skipped

    // ── Phase 6/9: 2-deep APU pipeline (slot A = executing, slot B =
    // staged, waiting for slot A to free) ───────────────────────────────
    // FCMP ($38) is folded in here too, NOT kept as a separate instant/
    // CU-only op -- two independent reasons: (1) it's the identical adder
    // hardware FADD/FSUB already use (Section 4.5.5.1: FCMP computes "as
    // if" FPn-source were subtracted), so real contention with an
    // in-flight FADD/FSUB for that shared resource is correct modeling,
    // not a simplification; (2) a confirmed Icarus tool limitation (not a
    // logic bug): two SEPARATE always_comb processes each independently
    // calling the SAME automatic task (fp_add_sub) produces a genuine
    // zero-time delta-cycle livelock in this simulator specifically
    // (bisected directly -- removing either call site individually fixes
    // it; the task's own logic is unaffected, confirmed by testing it in
    // complete isolation too) -- rather than work around a tool bug with
    // an awkward duplicate-logic-avoidance hack, routing FCMP through the
    // exact same single slot-A call site both fixes the tool issue AND is
    // the more realistic hardware choice.
    //
    // Phase 9: FABS/FNEG/FTST/FMOVE(reg-reg) also moved INTO this same
    // real pipeline slot, no longer "instant" -- Table 8-3 (below) proved
    // that assumption wrong. slotA_op_r/slotB_op_r hold the RAW 7-bit
    // extension-field code directly (Table 4-13's own real opcode value,
    // e.g. 7'h22 for FADD) rather than a separate translated enum -- one
    // real number, traceable straight back to the manual, that scales to
    // the full ~40-opcode extension-field space without a growing
    // translation layer to keep in sync.
    //
    // apu_latency(): confirmed directly against Table 8-3, "MC68882
    // Overall Execution Times" (MC68881/MC68882 User's Manual p.8-13,
    // both text-extracted AND visually confirmed against the actual page
    // image, not OCR-trusted blindly) -- the "FPn to FPm" (register-to-
    // register) column's own Total figure, in REAL external clock
    // cycles, x4 to convert to this chip's own clk_4x ticks (matching
    // the same 4x-multiplied-clock convention MH030 uses for its own
    // S-state timing). This REPLACES Phase 6's own placeholder
    // convention, which badly understated the real relative spread
    // (e.g. modeled FSQRT as barely 5x FADD; real silicon's FSQRT is
    // ~2x FADD, while a real transcendental like FACOS is ~11x FADD) and
    // wrongly modeled FABS/FNEG/FCMP/FTST as literally zero-cycle
    // (Table 5-1's "Minimum-Concurrency" framing means CONCURRENT with
    // other pipeline activity, not zero-latency in isolation -- a real,
    // if fast, ~36-38-cycle op on real silicon). Entries for opcodes not
    // yet implemented by this RTL (the still-open transcendental set)
    // are included here anyway, since it's pure real data with no
    // implementation cost -- only the dispatch gate (cmd_is_* below)
    // decides which ones this RTL actually routes into the pipeline yet;
    // everything else still falls through to the documented no-op stub.
    function automatic logic [11:0] apu_latency(logic [6:0] ext);
        unique case (ext)
            7'h00:   return 12'd84;   // FMOVE (reg-reg),  21 cyc
            7'h01:   return 12'd232;  // FINT,             58 cyc
            7'h02:   return 12'd2760; // FSINH,           690 cyc
            7'h03:   return 12'd232;  // FINTRZ,           58 cyc
            7'h04:   return 12'd440;  // FSQRT,           110 cyc
            7'h06:   return 12'd2296; // FLOGNP1,         574 cyc
            7'h08:   return 12'd2192; // FETOXM1,         548 cyc
            7'h09:   return 12'd2656; // FTANH,           664 cyc
            7'h0A:   return 12'd1624; // FATAN,           406 cyc
            7'h0C:   return 12'd2336; // FASIN,           584 cyc
            7'h0D:   return 12'd2784; // FATANH,          696 cyc
            7'h0E:   return 12'd1576; // FSIN,            394 cyc
            7'h0F:   return 12'd1904; // FTAN,            476 cyc
            7'h10:   return 12'd2000; // FETOX,           500 cyc
            7'h11:   return 12'd2280; // FTWOTOX,         570 cyc
            7'h12:   return 12'd2280; // FTENTOX,         570 cyc
            7'h14:   return 12'd2112; // FLOGN,           528 cyc
            7'h15:   return 12'd2336; // FLOG10,          584 cyc
            7'h16:   return 12'd2336; // FLOG2,           584 cyc
            7'h18:   return 12'd152;  // FABS,             38 cyc
            7'h19:   return 12'd2440; // FCOSH,           610 cyc
            7'h1A:   return 12'd152;  // FNEG,             38 cyc
            7'h1C:   return 12'd2512; // FACOS,           628 cyc
            7'h1D:   return 12'd1576; // FCOS,            394 cyc
            7'h1E:   return 12'd192;  // FGETEXP,          48 cyc
            7'h1F:   return 12'd136;  // FGETMAN,          34 cyc
            7'h20:   return 12'd432;  // FDIV,            108 cyc
            7'h21:   return 12'd300;  // FMOD,             75 cyc
            7'h22:   return 12'd224;  // FADD,             56 cyc
            7'h23:   return 12'd304;  // FMUL,             76 cyc
            7'h24:   return 12'd296;  // FSGLDIV,          74 cyc
            7'h25:   return 12'd420;  // FREM,            105 cyc
            7'h26:   return 12'd184;  // FSCALE,           46 cyc
            7'h27:   return 12'd256;  // FSGLMUL,          64 cyc
            7'h28:   return 12'd224;  // FSUB,             56 cyc
            7'h38:   return 12'd152;  // FCMP,             38 cyc
            7'h3A:   return 12'd144;  // FTST,             36 cyc
            default: return 12'd4;    // FSINCOS ($30-$37) + anything else
                                       // not yet dispatched into the real
                                       // pipeline (see cmd_is_* gating --
                                       // never actually reached today)
        endcase
    endfunction

    logic        slotA_valid_r;
    logic [6:0]  slotA_op_r;
    logic [95:0] slotA_a_r, slotA_b_r;
    logic [2:0]  slotA_dest_r;
    logic [1:0]  slotA_round_r;
    logic [11:0] apu_busy_cnt_r;

    logic        slotB_valid_r;
    logic [6:0]  slotB_op_r;
    logic [95:0] slotB_a_r, slotB_b_r;
    logic [2:0]  slotB_dest_r;
    logic [1:0]  slotB_round_r;
    logic [31:0] slotB_iaddr_r;

    wire apu_pipeline_busy = slotA_valid_r || slotB_valid_r;

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
    logic        fpiar_auto_wr_en;
    logic [31:0] fpiar_auto_wr_data;

    m68882_regfile u_regfile (
        .clk_4x, .rst_n,
        .fp_sel, .fp_chunk_idx, .fp_rd_data,
        .fp_wr_sel(fp_wr_sel_r), .fp_wr_chunk(fp_wr_chunk_r), .fp_wr_en, .fp_wr_data,
        .ctrl_sel, .ctrl_rd_data,
        .ctrl_wr_sel(ctrl_wr_sel_r), .ctrl_wr_en, .ctrl_wr_data,
        .fpsr_o, .fpcr_o, .fpiar_o, .all_zero_o, .null_reset_en,
        .fpiar_auto_wr_en, .fpiar_auto_wr_data,
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

    // apu_a_sel/apu_b_sel source from the REGISTERED command-word fields
    // (Phase 6: dispatch decode now fires a tick after the Command CIR
    // write, on the mandatory Instruction Address CIR write -- these
    // must stay stable across that gap, unlike the old live d_in-derived
    // wires). opclass 000: RX = source FPm, RY = destination FPn.
    // opclass 011's own supply_staged computation below reuses apu_b_rd
    // for exactly the same reason the original Phase 4c comment
    // documented: RY is ALSO the source FPm register for that opclass.
    assign apu_a_sel = cmd_rx_r;
    assign apu_b_sel = cmd_ry_r;

    // Command word field decode (combinational; only meaningful the tick
    // Command CIR is written -- captured into cmd_*_r registers below,
    // consumed one tick later).
    wire [2:0] c_opclass = cmd_opclass(d_in[31:16]);
    wire [2:0] c_rx      = cmd_rx(d_in[31:16]);
    wire [2:0] c_ry      = cmd_ry(d_in[31:16]);
    wire [6:0] c_ext     = cmd_ext(d_in[31:16]);
    wire [7:0] c_multi_mask = {c_ry[0], c_ext};

    // Registered opclass-000 op classification (stable from Command
    // write through the IA-write dispatch tick and beyond).
    wire cmd_is_fadd  = (cmd_ext_r == 7'h22);
    wire cmd_is_fsub  = (cmd_ext_r == 7'h28);
    wire cmd_is_fmul  = (cmd_ext_r == 7'h23);
    wire cmd_is_fdiv  = (cmd_ext_r == 7'h20);
    wire cmd_is_fcmp  = (cmd_ext_r == 7'h38);
    wire cmd_is_ftst  = (cmd_ext_r == 7'h3A);
    wire cmd_is_fabs  = (cmd_ext_r == 7'h18);
    wire cmd_is_fneg  = (cmd_ext_r == 7'h1A);
    wire cmd_is_fsqrt = (cmd_ext_r == 7'h04);
    wire cmd_is_fmove = (cmd_ext_r == 7'h00);

    // State-frame format words (Section 6.4.2) -- see plan.md/CLAUDE.md
    // for the full derivation; unchanged from Phase 5.
    localparam logic [15:0] FRAME_NULL_FMT = 16'h0000;
    localparam logic [15:0] FRAME_IDLE_FMT = {8'h1F, 8'd52};
    localparam logic [15:0] FRAME_BUSY_FMT = {8'h1F, 8'd208};
    localparam logic [5:0]  FRAME_IDLE_WORDS = 6'd13; // 52 bytes / 4
    localparam logic [5:0]  FRAME_BUSY_WORDS = 6'd52; // 208 bytes / 4

    // Phase 9: FABS/FNEG/FTST/FMOVE(reg-reg) all moved INTO the real
    // slot-A pipeline (see its own header comment) -- their computation
    // now lives alongside FADD/FSUB/FMUL/FDIV/FSQRT/FCMP's own, below,
    // operating on the CAPTURED slotA_a_r/slotA_b_r rather than live
    // apu_a_rd/apu_b_rd (which by commit time may belong to a completely
    // different, later dispatch).

    // Shared FPSR-update formula (Phase 4d's own confirmed OR-accumulation
    // formulas, Section 2.3.4/Figure 2-7) -- factored into a function so
    // both the instant-op path and the slot-A commit path (below) use the
    // identical, single-sourced expression.
    function automatic logic [31:0] fpsr_next(
        logic [31:0] fpsr_cur,
        logic n, logic z, logic i, logic nan,
        logic snan, logic operr, logic dz, logic ovfl, logic unfl, logic inex2
    );
        logic [31:0] r;
        r = {4'b0, n, z, i, nan,
             fpsr_cur[23:16],
             1'b0, snan, operr, ovfl, unfl, dz, inex2, 1'b0,
             (fpsr_cur[7] | snan | operr),
             (fpsr_cur[6] | ovfl),
             (fpsr_cur[5] | (unfl && inex2)),
             (fpsr_cur[4] | dz),
             (fpsr_cur[3] | inex2 | ovfl),
             fpsr_cur[2:0]};
        return r;
    endfunction

    // ── Phase 6: slot A's own combinational arithmetic core. Operates
    // on the CAPTURED slot-A operands/round-mode (not live register-file
    // reads, which by commit time may belong to a completely different,
    // later dispatch) -- pure combinational, re-evaluated every cycle,
    // only ACTED ON (write-back) at the commit tick (apu_busy_cnt_r==1)
    // in the always_ff block below. ─────────────────────────────────────
    logic [95:0] slotA_addsub_result, slotA_mul_result, slotA_div_result, slotA_sqrt_result;
    logic        slotA_addsub_z, slotA_addsub_n, slotA_addsub_i, slotA_addsub_nan, slotA_addsub_operr,
                 slotA_addsub_ovfl, slotA_addsub_unfl, slotA_addsub_inex2;
    logic        slotA_mul_z, slotA_mul_n, slotA_mul_i, slotA_mul_nan, slotA_mul_operr,
                 slotA_mul_ovfl, slotA_mul_unfl, slotA_mul_inex2;
    logic        slotA_div_z, slotA_div_n, slotA_div_i, slotA_div_nan, slotA_div_operr, slotA_div_dz,
                 slotA_div_ovfl, slotA_div_unfl, slotA_div_inex2;
    logic        slotA_sqrt_z, slotA_sqrt_n, slotA_sqrt_i, slotA_sqrt_nan, slotA_sqrt_operr,
                 slotA_sqrt_ovfl, slotA_sqrt_unfl, slotA_sqrt_inex2;

    always_comb begin
        fp_add_sub(slotA_a_r, slotA_b_r, (slotA_op_r == 7'h28 || slotA_op_r == 7'h38),
                   round_mode_t'(slotA_round_r),
                   slotA_addsub_result, slotA_addsub_z, slotA_addsub_n, slotA_addsub_i, slotA_addsub_nan,
                   slotA_addsub_operr, slotA_addsub_ovfl, slotA_addsub_unfl, slotA_addsub_inex2);
        fp_mul(slotA_a_r, slotA_b_r, round_mode_t'(slotA_round_r),
               slotA_mul_result, slotA_mul_z, slotA_mul_n, slotA_mul_i, slotA_mul_nan,
               slotA_mul_operr, slotA_mul_ovfl, slotA_mul_unfl, slotA_mul_inex2);
        fp_div(slotA_a_r, slotA_b_r, round_mode_t'(slotA_round_r),
               slotA_div_result, slotA_div_z, slotA_div_n, slotA_div_i, slotA_div_nan,
               slotA_div_operr, slotA_div_dz, slotA_div_ovfl, slotA_div_unfl, slotA_div_inex2);
        fp_sqrt(slotA_a_r, round_mode_t'(slotA_round_r),
                slotA_sqrt_result, slotA_sqrt_z, slotA_sqrt_n, slotA_sqrt_i, slotA_sqrt_nan,
                slotA_sqrt_operr, slotA_sqrt_ovfl, slotA_sqrt_unfl, slotA_sqrt_inex2);
    end

    // FABS/FNEG (trivial sign-bit ops) and FTST/FMOVE (no real ALU work
    // at all) need no genuine arithmetic core -- computed directly here,
    // off the CAPTURED slotA_a_r, same as every other slot-A op.
    wire [95:0] slotA_absneg_result = (slotA_op_r == 7'h18) ? {1'b0, slotA_a_r[94:0]}
                                                              : {!slotA_a_r[95], slotA_a_r[94:0]};
    fpx_t slotA_a_unpacked;
    assign slotA_a_unpacked = unpack_fpx(slotA_a_r);
    wire slotA_a_z   = is_zero_fpx(slotA_a_unpacked);
    wire slotA_a_n   = slotA_a_unpacked.sign && !slotA_a_z;
    wire slotA_a_i   = is_inf_fpx(slotA_a_unpacked);
    wire slotA_a_nan = is_nan_fpx(slotA_a_unpacked);

    logic [95:0] slotA_result;
    logic        slotA_flag_z, slotA_flag_n, slotA_flag_i, slotA_flag_nan, slotA_flag_operr,
                 slotA_flag_dz, slotA_flag_ovfl, slotA_flag_unfl, slotA_flag_inex2;
    always_comb begin
        unique case (slotA_op_r)
            7'h23: begin // FMUL
                slotA_result = slotA_mul_result; slotA_flag_z = slotA_mul_z; slotA_flag_n = slotA_mul_n;
                slotA_flag_i = slotA_mul_i; slotA_flag_nan = slotA_mul_nan; slotA_flag_operr = slotA_mul_operr;
                slotA_flag_dz = 1'b0; slotA_flag_ovfl = slotA_mul_ovfl; slotA_flag_unfl = slotA_mul_unfl;
                slotA_flag_inex2 = slotA_mul_inex2;
            end
            7'h20: begin // FDIV
                slotA_result = slotA_div_result; slotA_flag_z = slotA_div_z; slotA_flag_n = slotA_div_n;
                slotA_flag_i = slotA_div_i; slotA_flag_nan = slotA_div_nan; slotA_flag_operr = slotA_div_operr;
                slotA_flag_dz = slotA_div_dz; slotA_flag_ovfl = slotA_div_ovfl; slotA_flag_unfl = slotA_div_unfl;
                slotA_flag_inex2 = slotA_div_inex2;
            end
            7'h04: begin // FSQRT
                slotA_result = slotA_sqrt_result; slotA_flag_z = slotA_sqrt_z; slotA_flag_n = slotA_sqrt_n;
                slotA_flag_i = slotA_sqrt_i; slotA_flag_nan = slotA_sqrt_nan; slotA_flag_operr = slotA_sqrt_operr;
                slotA_flag_dz = 1'b0; slotA_flag_ovfl = slotA_sqrt_ovfl; slotA_flag_unfl = slotA_sqrt_unfl;
                slotA_flag_inex2 = slotA_sqrt_inex2;
            end
            7'h18, 7'h1A: begin // FABS / FNEG
                slotA_result = slotA_absneg_result;
                slotA_flag_z = is_zero_fpx(unpack_fpx(slotA_absneg_result));
                slotA_flag_n = slotA_absneg_result[95] && !slotA_flag_z;
                slotA_flag_i = is_inf_fpx(unpack_fpx(slotA_absneg_result));
                slotA_flag_nan = is_nan_fpx(unpack_fpx(slotA_absneg_result));
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h3A: begin // FTST -- source-only, never writes a register (see commit gating below)
                slotA_result = slotA_a_r; // unused
                slotA_flag_z = slotA_a_z; slotA_flag_n = slotA_a_n;
                slotA_flag_i = slotA_a_i; slotA_flag_nan = slotA_a_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h00: begin // FMOVE (register-to-register copy)
                slotA_result = slotA_a_r;
                slotA_flag_z = slotA_a_z; slotA_flag_n = slotA_a_n;
                slotA_flag_i = slotA_a_i; slotA_flag_nan = slotA_a_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            default: begin // FADD / FSUB / FCMP (identical adder)
                slotA_result = slotA_addsub_result; slotA_flag_z = slotA_addsub_z; slotA_flag_n = slotA_addsub_n;
                slotA_flag_i = slotA_addsub_i; slotA_flag_nan = slotA_addsub_nan; slotA_flag_operr = slotA_addsub_operr;
                slotA_flag_dz = 1'b0; slotA_flag_ovfl = slotA_addsub_ovfl; slotA_flag_unfl = slotA_addsub_unfl;
                slotA_flag_inex2 = slotA_addsub_inex2;
            end
        endcase
    end

    wire slotA_flag_snan = is_snan_fpx(unpack_fpx(slotA_a_r)) || is_snan_fpx(unpack_fpx(slotA_b_r));

    // Phase 6: FPCR ENABLE byte (bits[15:8], same bit positions as the
    // EXC byte -- Section 2.3/Figure 2-6) requesting a trap for whichever
    // exception slot A's own commit is about to report. When set, the
    // Response primitive becomes Take-Mid-Instruction-Exception instead
    // of reverting to Null (Section 7.5.4.1/4.5.5.1's general "arithmetic
    // exceptions are always reported mid-instruction, after the op
    // itself completes" framing) and PERSISTS until a completing FSAVE
    // (see the CIR_SAVE/CIR_OPERAND handling below). BSUN/INEX1 excluded
    // -- not yet raised anywhere, same Phase 4d scope boundary as before.
    wire slotA_exc_trap = (fpcr_o[14] && slotA_flag_snan)  || (fpcr_o[13] && slotA_flag_operr) ||
                           (fpcr_o[12] && slotA_flag_ovfl)  || (fpcr_o[11] && slotA_flag_unfl)  ||
                           (fpcr_o[10] && slotA_flag_dz)    || (fpcr_o[9]  && slotA_flag_inex2);

    // ── Phase 4c: external-operand format conversion (opclass 010/011)
    // -- unchanged in substance from Phase 4c/5, just retargeted onto the
    // registered cmd_rx_r/cmd_ry_r fields (dispatch now fires a tick
    // later than the Command CIR write itself). ─────────────────────────
    logic [31:0] supply_int32;
    logic [31:0] supply_single;
    logic [63:0] supply_double;
    logic        supply_int32_operr, supply_single_operr, supply_double_operr;
    logic [95:0] supply_staged;

    always_comb begin
        ext_to_int32(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_int32, supply_int32_operr);
        ext_to_single(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_single, supply_single_operr);
        ext_to_double(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_double, supply_double_operr);

        unique case (cmd_rx_r)
            FMT_L:   supply_staged = {supply_int32, 64'h0};
            FMT_S:   supply_staged = {supply_single, 64'h0};
            FMT_D:   supply_staged = {supply_double, 32'h0};
            FMT_X:   supply_staged = apu_b_rd; // native format, pure passthrough
            default: supply_staged = 96'h0; // W/B/P: not yet implemented (Phase 4c scope)
        endcase
    end

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

    // Condition CIR predicate field + FPSR Z bit (combinational). This
    // dialog is NOT gated by the mandatory-Instruction-Address rule --
    // see the header comment's own scoping note.
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

    // RX field bit-order reversal -- see the original Phase 3 finding
    // (an FPCR-only select resolved to ctrl_sel index 2/FPIAR instead of
    // 0) -- now derived from the REGISTERED cmd_rx_r.
    wire [7:0] cmd_ctrl_mask = {5'b0, cmd_rx_r[0], cmd_rx_r[1], cmd_rx_r[2]};
    assign next_fp_idx   = first_set_from(mask_r, {1'b0, reg_idx_r} + 4'd1);
    assign next_ctrl_idx = first_set_from({5'b0, mask_r[2:0]}, {1'b0, reg_idx_r} + 4'd1);
    assign first_ctrl_idx_from_rx = first_set_from(cmd_ctrl_mask, 4'd0);

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
            cmd_opclass_r <= 3'h0;
            cmd_rx_r      <= 3'h0;
            cmd_ry_r      <= 3'h0;
            cmd_ext_r     <= 7'h0;
            cmd_multi_mask_r <= 8'h0;
            cmd_round_r   <= 2'h0;
            cu_instr_addr_r   <= 32'h0;
            proto_violation_r <= 1'b0;
            slotA_valid_r <= 1'b0;
            slotA_op_r    <= 3'h0;
            slotA_a_r     <= 96'h0;
            slotA_b_r     <= 96'h0;
            slotA_dest_r  <= 3'h0;
            slotA_round_r <= 2'h0;
            apu_busy_cnt_r <= 12'h0;
            slotB_valid_r <= 1'b0;
            slotB_op_r    <= 3'h0;
            slotB_a_r     <= 96'h0;
            slotB_b_r     <= 96'h0;
            slotB_dest_r  <= 3'h0;
            slotB_round_r <= 2'h0;
            slotB_iaddr_r <= 32'h0;
            fpiar_auto_wr_en   <= 1'b0;
            fpiar_auto_wr_data <= 32'h0;
        end else begin
            fp_wr_en      <= 1'b0;
            ctrl_wr_en    <= 1'b0;
            apu_wr_en     <= 1'b0;
            null_reset_en <= 1'b0;
            proto_violation_r  <= 1'b0;
            fpiar_auto_wr_en   <= 1'b0;

            if (abort) begin
                state_r <= ST_IDLE;
                ca_r    <= 1'b0;
                prim_r  <= PRIM_NULL;
            end else if (ack_pulse) begin
                if (state_r == ST_WAIT_IADDR && cyc_sel != CIR_INSTRADDR) begin
                    // Phase 6: mandatory Instruction Address CIR write was
                    // skipped -- a real protocol violation. This access is
                    // otherwise dropped (no CIR here acts while waiting on
                    // the mandatory write); the dialog stays parked in
                    // ST_WAIT_IADDR until the host does write it (or aborts).
                    proto_violation_r <= 1'b1;
                end else begin
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
                        // Phase 6: only capture the command word here --
                        // real dispatch decode is deferred to the
                        // mandatory Instruction Address CIR write below.
                        cmd_opclass_r    <= c_opclass;
                        cmd_rx_r         <= c_rx;
                        cmd_ry_r         <= c_ry;
                        cmd_ext_r        <= c_ext;
                        cmd_multi_mask_r <= c_multi_mask;
                        cmd_round_r      <= fpcr_o[5:4];
                        state_r          <= ST_WAIT_IADDR;
                    end

                    CIR_INSTRADDR: if (state_r == ST_WAIT_IADDR) begin
                        cu_instr_addr_r <= d_in;

                        unique case (cmd_opclass_r)
                            3'b000: begin // FPm to FPn, register-to-register: no external
                                           // transfer needed (Table 4-13 Note 1), so the
                                           // CU's own part is ALWAYS instant (Null/CA=0
                                           // immediately) regardless of whether the APU
                                           // has even started the real computation --
                                           // every recognized op here (including FABS/
                                           // FNEG/FCMP/FTST/FMOVE -- Table 8-3 confirms
                                           // NONE of them are actually zero-cycle on real
                                           // silicon, just fast relative to the
                                           // transcendentals) goes through the real
                                           // 2-deep APU pipeline with its own genuine
                                           // Table 8-3 latency.
                                if (cmd_is_fadd || cmd_is_fsub || cmd_is_fmul ||
                                    cmd_is_fdiv || cmd_is_fsqrt || cmd_is_fcmp ||
                                    cmd_is_fabs || cmd_is_fneg || cmd_is_ftst || cmd_is_fmove) begin
                                    state_r <= ST_IDLE;
                                    if (!slotA_valid_r) begin
                                        ca_r    <= 1'b0;
                                        prim_r  <= PRIM_NULL;
                                        slotA_valid_r  <= 1'b1;
                                        slotA_op_r     <= cmd_ext_r;
                                        slotA_a_r      <= apu_a_rd;
                                        slotA_b_r      <= apu_b_rd;
                                        slotA_dest_r   <= cmd_ry_r;
                                        slotA_round_r  <= cmd_round_r;
                                        apu_busy_cnt_r <= apu_latency(cmd_ext_r);
                                        fpiar_auto_wr_en   <= 1'b1;
                                        fpiar_auto_wr_data <= d_in;
                                    end else if (!slotB_valid_r) begin
                                        // Genuine 2-deep pipeline: slot A
                                        // still busy with a PRIOR
                                        // instruction -- stage this one
                                        // into slot B and wait. No FPIAR
                                        // load yet (only the APU-stage
                                        // register updates, at promotion).
                                        ca_r    <= 1'b0;
                                        prim_r  <= PRIM_NULL;
                                        slotB_valid_r <= 1'b1;
                                        slotB_op_r    <= cmd_ext_r;
                                        slotB_a_r     <= apu_a_rd;
                                        slotB_b_r     <= apu_b_rd;
                                        slotB_dest_r  <= cmd_ry_r;
                                        slotB_round_r <= cmd_round_r;
                                        slotB_iaddr_r <= d_in;
                                    end else begin
                                        // Both stages full -- Section 7.2.6
                                        // "busy APU, defer command word":
                                        // reject. CA=1 signals "still busy";
                                        // a real host retries the SAME
                                        // Command+Instruction-Address
                                        // sequence later. Nothing about
                                        // slot A/B changes.
                                        ca_r   <= 1'b1;
                                        prim_r <= PRIM_NULL;
                                    end
                                end else begin
                                    // Every other extension-field opcode
                                    // (the full transcendental set, etc.)
                                    // is still a no-op stub -- matches
                                    // every prior phase's own documented
                                    // scope boundary (plan.md/m68882_apu.sv).
                                    // Still needs a real state_r<=ST_IDLE,
                                    // same as every recognized op, or the
                                    // dialog would get stuck in
                                    // ST_WAIT_IADDR forever.
                                    ca_r    <= 1'b0;
                                    prim_r  <= PRIM_NULL;
                                    state_r <= ST_IDLE;
                                end
                            end
                            3'b010: if (cmd_rx_r == 3'b111) begin // move constant to FPn
                                ca_r    <= 1'b0;
                                prim_r  <= PRIM_NULL;
                                state_r <= ST_IDLE;
                            end else begin // external operand to FPn
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b0; // host supplies data (RECEIVE)
                                prim_r        <= PRIM_EVAL_EA;
                                is_ctrl_reg_r <= 1'b0;
                                multi_ctrl_r  <= 1'b0;
                                reg_idx_r     <= cmd_ry_r;
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= chunks_for_bytes(fmt_bytes(cmd_rx_r));
                                xfer_fmt_r    <= cmd_rx_r;
                                xfer_stage_r  <= 96'h0;
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b011: begin // FPm to external destination
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b1; // FPCP supplies data (SUPPLY)
                                prim_r        <= PRIM_EVAL_EA;
                                is_ctrl_reg_r <= 1'b0;
                                multi_ctrl_r  <= 1'b0;
                                reg_idx_r     <= cmd_ry_r; // source FPm
                                chunk_idx_r   <= 2'h0;
                                chunks_left_r <= chunks_for_bytes(fmt_bytes(cmd_rx_r));
                                xfer_fmt_r    <= cmd_rx_r;
                                xfer_stage_r  <= supply_staged; // pre-converted at dispatch
                                state_r       <= ST_WAIT_XFER;
                            end
                            3'b100: begin // move to system control register(s)
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b0; // RECEIVE
                                prim_r        <= PRIM_XFER_SINGLE;
                                is_ctrl_reg_r <= 1'b1;
                                multi_ctrl_r  <= 1'b1;
                                mask_r        <= cmd_ctrl_mask;
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
                                mask_r        <= cmd_ctrl_mask;
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
                                mask_r        <= cmd_multi_mask_r;
                                state_r       <= ST_WAIT_REGSEL;
                            end
                            3'b111: begin // move multiple from FP data registers
                                ca_r          <= 1'b1;
                                dr_r          <= 1'b1; // SUPPLY
                                prim_r        <= PRIM_XFER_MULTI;
                                is_ctrl_reg_r <= 1'b0;
                                mask_r        <= cmd_multi_mask_r;
                                state_r       <= ST_WAIT_REGSEL;
                            end
                            default: state_r <= ST_IDLE;
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

                    // FSAVE dialog (Section 7.2.3). A read is legal at any
                    // time EXCEPT during an already-active state-frame
                    // transfer; deliberately does NOT gate on
                    // state_r==ST_IDLE, since capturing a genuinely BUSY
                    // dialog is exactly what the Busy frame is for.
                    // Phase 6: apu_pipeline_busy (a real in-flight
                    // arithmetic instruction, even with state_r itself
                    // idle) now ALSO forces the Busy classification.
                    CIR_SAVE: if (!cyc_write &&
                                  state_r != ST_WAIT_SAVE_XFER && state_r != ST_WAIT_RESTORE_XFER) begin
                        if (all_zero_o && state_r == ST_IDLE && !apu_pipeline_busy) begin
                            // Null: "the FPCP is in the reset state, and
                            // the next expected access is to the command
                            // or condition CIR" -- no Operand CIR transfer.
                            state_r <= ST_IDLE;
                        end else if (state_r == ST_IDLE && !apu_pipeline_busy) begin
                            frame_words_left_r <= FRAME_IDLE_WORDS;
                            state_r            <= ST_WAIT_SAVE_XFER;
                        end else begin
                            // a genuine dialog (CU OR APU-pipeline) was in
                            // progress -- Busy frame
                            frame_words_left_r <= FRAME_BUSY_WORDS;
                            state_r            <= ST_WAIT_SAVE_XFER;
                        end
                    end

                    // FRESTORE dialog (Section 7.2.4). The host writes the
                    // format word first; the FOLLOWING read reports
                    // validation success/failure and (if valid) advances
                    // the dialog.
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
                                    // model, not just the internal state --
                                    // Phase 6: that now includes the live
                                    // APU pipeline and any pending
                                    // exception primitive too.
                                    null_reset_en <= 1'b1;
                                    slotA_valid_r <= 1'b0;
                                    slotB_valid_r <= 1'b0;
                                    apu_busy_cnt_r <= 12'h0;
                                    prim_r  <= PRIM_NULL;
                                    ca_r    <= 1'b0;
                                    state_r <= ST_IDLE;
                                end else begin
                                    frame_words_left_r <= (restore_fmt_r == FRAME_BUSY_FMT)
                                                           ? FRAME_BUSY_WORDS : FRAME_IDLE_WORDS;
                                    state_r            <= ST_WAIT_RESTORE_XFER;
                                end
                            end else begin
                                // invalid format word -- stay idle; a real
                                // host is expected to write an abort (AB
                                // bit) to the Control CIR next.
                                state_r <= ST_IDLE;
                            end
                        end
                    end

                    CIR_OPERAND: if (state_r == ST_WAIT_SAVE_XFER || state_r == ST_WAIT_RESTORE_XFER) begin
                        // FSAVE/FRESTORE payload transfer. Placeholder
                        // payload both directions (see FRAME_IDLE_FMT's
                        // own header comment).
                        if (frame_words_left_r <= 6'd1) begin
                            // Section 7.5.4.2: only a COMPLETING FSAVE
                            // clears a pending mid/pre-instruction
                            // exception primitive back to null.
                            if (state_r == ST_WAIT_SAVE_XFER &&
                                (prim_r == PRIM_TAKE_MID || prim_r == PRIM_TAKE_PRE)) begin
                                prim_r <= PRIM_NULL;
                                ca_r   <= 1'b0;
                            end
                            state_r <= ST_IDLE;
                        end else begin
                            frame_words_left_r <= frame_words_left_r - 6'd1;
                        end
                    end else if (state_r == ST_WAIT_XFER || state_r == ST_WAIT_MULTI_XFER) begin
                        if (!dr_r) begin
                            // RECEIVE: latch host-supplied data into the register file
                            if (is_ctrl_reg_r) begin
                                ctrl_wr_en    <= 1'b1;
                                ctrl_wr_sel_r <= reg_idx_r[1:0]; // pre-advance value
                                ctrl_wr_data  <= d_in;
                            end else if (state_r == ST_WAIT_XFER) begin
                                // opclass 010 external-operand-to-FPn --
                                // stage the raw chunk every access; on the
                                // LAST chunk, also issue the real converted
                                // value.
                                xfer_stage_r <= receive_assembled;
                                if (chunks_left_r <= 2'd1) begin
                                    apu_wr_en   <= 1'b1;
                                    apu_wr_sel  <= reg_idx_r; // pre-advance value
                                    apu_wr_data <= receive_converted;
                                end
                            end else begin
                                // opclass 110 move-multiple-to-FPn: each
                                // chunk is already a genuine native 32-bit
                                // slice -- no conversion.
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

            // ── Phase 6: APU pipeline countdown + slot-B promotion.
            // Runs every cycle, independent of any CIR bus access --
            // this is what actually lets the APU keep computing while
            // the CU handles unrelated bus traffic. Placed textually
            // AFTER the ack_pulse-gated dispatch switch above so that,
            // on the rare cycle a slot-A commit and a brand-new dispatch
            // land together, the commit's own ca_r/prim_r write (only
            // ever touched here when a trapped exception is being
            // reported) deterministically wins -- exception reporting
            // takes priority over an ordinary same-cycle dispatch ack. ──
            if (slotA_valid_r) begin
                if (apu_busy_cnt_r <= 12'd1) begin
                    // Section 4.5.5.1: FCMP compares "as if" FPn-source
                    // were computed, but FPn itself is never written.
                    // FTST likewise never writes (source-only, condition
                    // codes only).
                    if (slotA_op_r != 7'h38 && slotA_op_r != 7'h3A) begin
                        apu_wr_en   <= 1'b1;
                        apu_wr_sel  <= slotA_dest_r;
                        apu_wr_data <= slotA_result;
                    end
                    ctrl_wr_en    <= 1'b1;
                    ctrl_wr_sel_r <= 2'd1; // FPSR
                    ctrl_wr_data  <= fpsr_next(fpsr_o, slotA_flag_n, slotA_flag_z, slotA_flag_i,
                                                slotA_flag_nan, slotA_flag_snan, slotA_flag_operr,
                                                slotA_flag_dz, slotA_flag_ovfl, slotA_flag_unfl,
                                                slotA_flag_inex2);
                    if (slotA_exc_trap) begin
                        prim_r <= PRIM_TAKE_MID;
                        ca_r   <= 1'b1;
                        // dr_r is otherwise only ever explicitly set by the
                        // opclass 010/011 dialogs -- Take-Mid-Instruction-
                        // Exception is a genuinely NEW non-null primitive
                        // this always_ff block can raise, and dr_eff's own
                        // staleness guard (see its header comment) only
                        // protects reverting TO null, not a fresh non-null
                        // primitive that never sets dr_r itself. Pin it to
                        // a defined value rather than leak whatever the
                        // last EA-transfer dialog happened to leave here.
                        dr_r   <= 1'b0;
                    end
                    if (slotB_valid_r) begin
                        slotA_op_r     <= slotB_op_r;
                        slotA_a_r      <= slotB_a_r;
                        slotA_b_r      <= slotB_b_r;
                        slotA_dest_r   <= slotB_dest_r;
                        slotA_round_r  <= slotB_round_r;
                        apu_busy_cnt_r <= apu_latency(slotB_op_r);
                        slotB_valid_r  <= 1'b0;
                        fpiar_auto_wr_en   <= 1'b1;
                        fpiar_auto_wr_data <= slotB_iaddr_r;
                    end else begin
                        slotA_valid_r <= 1'b0;
                    end
                end else begin
                    apu_busy_cnt_r <= apu_busy_cnt_r - 12'd1;
                end
            end else if (slotB_valid_r) begin
                // Safety net for a same-cycle race: a dispatch's own
                // Instruction-Address-write ack_pulse can land on the
                // exact tick slot A's countdown independently reaches its
                // last cycle. Both the dispatch decode above and this
                // commit branch read the SAME pre-tick slotB_valid_r==0,
                // so dispatch stages the new instruction into slot B
                // while this branch (seeing slot A about to go empty with
                // nothing to promote) simply clears slotA_valid_r -- left
                // alone, slot B would then sit orphaned forever with slot
                // A empty and no promotion trigger pending. Reachable only
                // via that exact race; harmless/never-taken in ordinary
                // operation (slot B is normally only ever populated while
                // slot A is ALREADY valid).
                slotA_valid_r  <= 1'b1;
                slotA_op_r     <= slotB_op_r;
                slotA_a_r      <= slotB_a_r;
                slotA_b_r      <= slotB_b_r;
                slotA_dest_r   <= slotB_dest_r;
                slotA_round_r  <= slotB_round_r;
                apu_busy_cnt_r <= apu_latency(slotB_op_r);
                slotB_valid_r  <= 1'b0;
                fpiar_auto_wr_en   <= 1'b1;
                fpiar_auto_wr_data <= slotB_iaddr_r;
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
                // The format word for whichever frame type applies right
                // now (Null/Idle/Busy) -- combinational, since the actual
                // dialog state transition happens in the always_ff block
                // above; this just needs to present the correct value AT
                // the ack tick. Phase 6: apu_pipeline_busy also forces Busy.
                d_oe = 1'b1;
                if (state_r != ST_IDLE || apu_pipeline_busy) begin
                    d_out = {FRAME_BUSY_FMT, 16'h0};
                end else if (all_zero_o) begin
                    d_out = {FRAME_NULL_FMT, 16'h0};
                end else begin
                    d_out = {FRAME_IDLE_FMT, 16'h0};
                end
            end
            CIR_RESTORE: if (!cyc_write) begin
                // Echo the validated format word back (success), or an
                // "invalid format" marker (Section 7.2.4) -- this
                // project's own placeholder for that marker, 16'hFFFF,
                // is NOT confirmed against a real numeric code from the
                // manual.
                d_oe  = 1'b1;
                d_out = {(restore_valid_r ? restore_fmt_r : 16'hFFFF), 16'h0};
            end
            CIR_OPERAND: if (state_r == ST_WAIT_SAVE_XFER) begin
                // Placeholder payload (see FRAME_IDLE_FMT's own header
                // comment) -- always zero.
                d_oe  = 1'b1;
                d_out = 32'h0;
            end else if (dr_r && (state_r == ST_WAIT_XFER || state_r == ST_WAIT_MULTI_XFER)) begin
                d_oe  = 1'b1;
                if (is_ctrl_reg_r) begin
                    d_out = ctrl_rd_data;
                end else if (state_r == ST_WAIT_XFER) begin
                    // opclass 011 FPn-to-external -- read out the
                    // pre-converted staging buffer chunk by chunk, not the
                    // raw register value fp_rd_data would present.
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
