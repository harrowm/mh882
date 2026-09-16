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
            // FSINCOS ($30-$37): real Table 8-3 total is 454 cyc (×4 =
            // 1816 ticks, "FPn to FPm" column, confirmed directly),
            // same for all 8 values -- the low 3 bits only select the
            // cos-destination register, never affecting timing. Returns
            // 1815, ONE tick short of that real total: the commit
            // logic's own two-tick write sequence (sin then cos, since
            // the register file has only one write port -- see
            // slotA_sincos_pending_r's own header comment) spends an
            // extra tick beyond the single-tick commit every other op
            // uses, and that extra tick is what brings the TOTAL
            // dispatch-to-completion latency back up to the real 1816 --
            // not an extra, undocumented cycle on top of it.
            7'h30, 7'h31, 7'h32, 7'h33, 7'h34, 7'h35, 7'h36, 7'h37:
                     return 12'd1815; // FSINCOS,   454 cyc total - 1
            default: return 12'd4;    // anything else not yet dispatched
                                       // into the real pipeline (see
                                       // cmd_is_* gating -- never actually
                                       // reached today)
        endcase
    endfunction

    logic        slotA_valid_r;
    logic [6:0]  slotA_op_r;
    logic [95:0] slotA_a_r, slotA_b_r;
    logic [2:0]  slotA_dest_r;
    logic [1:0]  slotA_round_r;
    logic [11:0] apu_busy_cnt_r;
    // Phase 9e: FSINCOS ($30-$37) is the one op whose real semantics
    // need TWO register writes (sin to the usual Ry destination, cos to
    // a SECOND register the extension word's own low 3 bits select --
    // slotA_op_r[2:0], since slotA_op_r already holds the raw 7-bit
    // Table 4-13 extension code and FSINCOS's own encoding packs its
    // cos-destination directly into those bits, confirmed against
    // Musashi's own `REG_FP[opmode&7]`). This architecture's commit
    // logic otherwise always finishes a slot in exactly one tick; this
    // flag extends FSINCOS's own commit to two, reusing the SAME single
    // apu_wr_en/apu_wr_sel/apu_wr_data port sequentially rather than
    // adding a second write port to the register file.
    logic        slotA_sincos_pending_r;

    logic        slotB_valid_r;
    logic [6:0]  slotB_op_r;
    logic [95:0] slotB_a_r, slotB_b_r;
    logic [2:0]  slotB_dest_r;
    logic [1:0]  slotB_round_r;
    logic [31:0] slotB_iaddr_r;

    wire apu_pipeline_busy = slotA_valid_r || slotB_valid_r;
    wire slotA_is_fsincos = (slotA_op_r[6:3] == 4'b0110);

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
    logic        bsun_set_en;

    m68882_regfile u_regfile (
        .clk_4x, .rst_n,
        .fp_sel, .fp_chunk_idx, .fp_rd_data,
        .fp_wr_sel(fp_wr_sel_r), .fp_wr_chunk(fp_wr_chunk_r), .fp_wr_en, .fp_wr_data,
        .ctrl_sel, .ctrl_rd_data,
        .ctrl_wr_sel(ctrl_wr_sel_r), .ctrl_wr_en, .ctrl_wr_data,
        .fpsr_o, .fpcr_o, .fpiar_o, .all_zero_o, .null_reset_en,
        .fpiar_auto_wr_en, .fpiar_auto_wr_data,
        .bsun_set_en,
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
    wire cmd_is_fmove   = (cmd_ext_r == 7'h00);
    wire cmd_is_fint    = (cmd_ext_r == 7'h01);
    wire cmd_is_fintrz  = (cmd_ext_r == 7'h03);
    wire cmd_is_fgetexp = (cmd_ext_r == 7'h1E);
    wire cmd_is_fgetman = (cmd_ext_r == 7'h1F);
    wire cmd_is_fscale  = (cmd_ext_r == 7'h26);
    wire cmd_is_fsgldiv = (cmd_ext_r == 7'h24);
    wire cmd_is_fsglmul = (cmd_ext_r == 7'h27);
    wire cmd_is_fmod    = (cmd_ext_r == 7'h21);
    wire cmd_is_frem    = (cmd_ext_r == 7'h25);
    wire cmd_is_fsin    = (cmd_ext_r == 7'h0E);
    wire cmd_is_fcos    = (cmd_ext_r == 7'h1D);
    // FSINCOS ($30-$37): top 4 bits fixed at 0110, low 3 bits select the
    // cos-destination register (Musashi's own `opmode&7`; confirmed
    // against Table 4-13's own general-instruction-format extension-word
    // breakdown).
    wire cmd_is_fsincos = (cmd_ext_r[6:3] == 4'b0110);
    // Phase 9f: the exponential family, all built on the shared
    // fp_exp_core task.
    wire cmd_is_fetox   = (cmd_ext_r == 7'h10);
    wire cmd_is_fetoxm1 = (cmd_ext_r == 7'h08);
    wire cmd_is_ftwotox = (cmd_ext_r == 7'h11);
    wire cmd_is_ftentox = (cmd_ext_r == 7'h12);
    // Phase 9g: hyperbolic family + FTAN.
    wire cmd_is_fsinh = (cmd_ext_r == 7'h02);
    wire cmd_is_fcosh = (cmd_ext_r == 7'h19);
    wire cmd_is_ftanh = (cmd_ext_r == 7'h09);
    wire cmd_is_ftan  = (cmd_ext_r == 7'h0F);
    // Phase 9h: the logarithm family.
    wire cmd_is_flogn   = (cmd_ext_r == 7'h14);
    wire cmd_is_flognp1 = (cmd_ext_r == 7'h06);
    wire cmd_is_flog10  = (cmd_ext_r == 7'h15);
    wire cmd_is_flog2   = (cmd_ext_r == 7'h16);
    // Phase 9i: the last 4 functions of the original transcendental set.
    wire cmd_is_fatan  = (cmd_ext_r == 7'h0A);
    wire cmd_is_fasin  = (cmd_ext_r == 7'h0C);
    wire cmd_is_facos  = (cmd_ext_r == 7'h1C);
    wire cmd_is_fatanh = (cmd_ext_r == 7'h0D);

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

    // Phase 9b: exact auxiliary ops. FINT/FINTRZ share ONE fp_int() call
    // site (a second, independent call site of the same task is exactly
    // the class of confirmed Icarus livelock Phase 6 already hit once --
    // APU_OP_CMP's own header comment -- so FINTRZ's "always round
    // toward zero regardless of FPCR" requirement is expressed by muxing
    // the EFFECTIVE rounding mode into the single shared call instead of
    // adding a second call).
    logic [1:0] slotA_int_round_bits;
    assign slotA_int_round_bits = (slotA_op_r == 7'h03) ? 2'b01 /* RND_ZERO */ : slotA_round_r;

    logic [95:0] slotA_int_result, slotA_getexp_result, slotA_getman_result, slotA_scale_result;
    logic        slotA_int_z, slotA_int_n, slotA_int_i, slotA_int_nan, slotA_int_inex2;
    logic        slotA_getexp_z, slotA_getexp_n, slotA_getexp_i, slotA_getexp_nan, slotA_getexp_operr;
    logic        slotA_getman_z, slotA_getman_n, slotA_getman_i, slotA_getman_nan, slotA_getman_operr;
    logic        slotA_scale_z, slotA_scale_n, slotA_scale_i, slotA_scale_nan, slotA_scale_ovfl, slotA_scale_unfl;

    // Phase 9c: FSGLDIV/FSGLMUL/FMOD/FREM -- one call site each, same
    // constraint as everything else in this block.
    logic [95:0] slotA_sgldiv_result, slotA_sglmul_result, slotA_modrem_result;
    logic        slotA_sgldiv_z, slotA_sgldiv_n, slotA_sgldiv_i, slotA_sgldiv_nan, slotA_sgldiv_operr,
                 slotA_sgldiv_dz, slotA_sgldiv_ovfl, slotA_sgldiv_unfl, slotA_sgldiv_inex2;
    logic        slotA_sglmul_z, slotA_sglmul_n, slotA_sglmul_i, slotA_sglmul_nan, slotA_sglmul_operr,
                 slotA_sglmul_ovfl, slotA_sglmul_unfl, slotA_sglmul_inex2;
    logic        slotA_modrem_z, slotA_modrem_n, slotA_modrem_i, slotA_modrem_nan, slotA_modrem_operr;
    logic [7:0]  slotA_modrem_quot_byte;

    // Phase 9d: FSIN/FCOS -- shared fp_sincos task, one call site, same
    // constraint as everything else in this block. FSINCOS itself
    // (dual-register write) is deliberately NOT implemented here -- this
    // architecture's slot pipeline only ever commits a single result to
    // a single destination register per dispatched op, so FSINCOS's own
    // dual-write would need new commit-path plumbing beyond this task's
    // own scope. FSIN and FCOS individually cover the two Musashi-
    // verifiable results FSINCOS itself would also produce, so nothing
    // numerically new is left unverified by deferring it.
    logic [95:0] slotA_sin_result, slotA_cos_result;
    logic        slotA_sincos_operr;
    fpx_t        slotA_sincos_sel_unpacked;
    wire  [95:0] slotA_sincos_result = (slotA_op_r == 7'h0E) ? slotA_sin_result : slotA_cos_result;
    assign slotA_sincos_sel_unpacked = unpack_fpx(slotA_sincos_result);
    wire         slotA_sincos_z = is_zero_fpx(slotA_sincos_sel_unpacked);
    wire         slotA_sincos_n = slotA_sincos_sel_unpacked.sign && !slotA_sincos_z;
    wire         slotA_sincos_i = is_inf_fpx(slotA_sincos_sel_unpacked);
    wire         slotA_sincos_nan = is_nan_fpx(slotA_sincos_sel_unpacked);

    // Phase 9f: FETOX/FETOXM1/FTWOTOX/FTENTOX -- each its own call site
    // (fp_etox/fp_etoxm1/fp_twotox/fp_tentox are thin wrappers around
    // the shared fp_exp_core, each with their own single call site here
    // -- same one-site-per-task discipline as every other op in this
    // block; fp_exp_core's own internal call sites are private to
    // whichever wrapper invokes it and don't multiply out here).
    logic [95:0] slotA_etox_result, slotA_etoxm1_result, slotA_twotox_result, slotA_tentox_result;
    logic        slotA_etox_z, slotA_etox_n, slotA_etox_i, slotA_etox_nan, slotA_etox_ovfl, slotA_etox_unfl;
    logic        slotA_etoxm1_z, slotA_etoxm1_n, slotA_etoxm1_i, slotA_etoxm1_nan, slotA_etoxm1_ovfl, slotA_etoxm1_unfl;
    logic        slotA_twotox_z, slotA_twotox_n, slotA_twotox_i, slotA_twotox_nan, slotA_twotox_ovfl, slotA_twotox_unfl;
    logic        slotA_tentox_z, slotA_tentox_n, slotA_tentox_i, slotA_tentox_nan, slotA_tentox_ovfl, slotA_tentox_unfl;

    // Phase 9g: hyperbolic family + FTAN -- one call site each.
    logic [95:0] slotA_sinh_result, slotA_cosh_result, slotA_tanh_result, slotA_tan_result;
    logic        slotA_sinh_z, slotA_sinh_n, slotA_sinh_i, slotA_sinh_nan;
    logic        slotA_cosh_z, slotA_cosh_n, slotA_cosh_i, slotA_cosh_nan;
    logic        slotA_tanh_z, slotA_tanh_n, slotA_tanh_i, slotA_tanh_nan;
    logic        slotA_tan_z, slotA_tan_n, slotA_tan_i, slotA_tan_nan, slotA_tan_operr;

    // Phase 9h: the logarithm family -- one call site each.
    logic [95:0] slotA_logn_result, slotA_lognp1_result, slotA_log10_result, slotA_log2_result;
    logic        slotA_logn_z, slotA_logn_n, slotA_logn_i, slotA_logn_nan, slotA_logn_operr, slotA_logn_dz;
    logic        slotA_lognp1_z, slotA_lognp1_n, slotA_lognp1_i, slotA_lognp1_nan, slotA_lognp1_operr, slotA_lognp1_dz;
    logic        slotA_log10_z, slotA_log10_n, slotA_log10_i, slotA_log10_nan, slotA_log10_operr, slotA_log10_dz;
    logic        slotA_log2_z, slotA_log2_n, slotA_log2_i, slotA_log2_nan, slotA_log2_operr, slotA_log2_dz;

    // Phase 9i: the last 4 functions -- one call site each.
    logic [95:0] slotA_atan_result, slotA_asin_result, slotA_acos_result, slotA_atanh_result;
    logic        slotA_atan_z, slotA_atan_n, slotA_atan_i, slotA_atan_nan;
    logic        slotA_asin_z, slotA_asin_n, slotA_asin_i, slotA_asin_nan, slotA_asin_operr;
    logic        slotA_acos_z, slotA_acos_n, slotA_acos_i, slotA_acos_nan, slotA_acos_operr;
    logic        slotA_atanh_z, slotA_atanh_n, slotA_atanh_i, slotA_atanh_nan, slotA_atanh_operr, slotA_atanh_dz;

    always_comb begin
        fp_int(slotA_a_r, round_mode_t'(slotA_int_round_bits),
               slotA_int_result, slotA_int_z, slotA_int_n, slotA_int_i, slotA_int_nan, slotA_int_inex2);
        fp_getexp(slotA_a_r, slotA_getexp_result, slotA_getexp_z, slotA_getexp_n, slotA_getexp_i,
                  slotA_getexp_nan, slotA_getexp_operr);
        fp_getman(slotA_a_r, slotA_getman_result, slotA_getman_z, slotA_getman_n, slotA_getman_i,
                  slotA_getman_nan, slotA_getman_operr);
        fp_scale(slotA_a_r, slotA_b_r, slotA_scale_result, slotA_scale_z, slotA_scale_n, slotA_scale_i,
                 slotA_scale_nan, slotA_scale_ovfl, slotA_scale_unfl);
        fp_sgldiv(slotA_a_r, slotA_b_r, round_mode_t'(slotA_round_r),
                  slotA_sgldiv_result, slotA_sgldiv_z, slotA_sgldiv_n, slotA_sgldiv_i, slotA_sgldiv_nan,
                  slotA_sgldiv_operr, slotA_sgldiv_dz, slotA_sgldiv_ovfl, slotA_sgldiv_unfl, slotA_sgldiv_inex2);
        fp_sglmul(slotA_a_r, slotA_b_r, round_mode_t'(slotA_round_r),
                  slotA_sglmul_result, slotA_sglmul_z, slotA_sglmul_n, slotA_sglmul_i, slotA_sglmul_nan,
                  slotA_sglmul_operr, slotA_sglmul_ovfl, slotA_sglmul_unfl, slotA_sglmul_inex2);
        fp_mod_rem(slotA_a_r, slotA_b_r, (slotA_op_r == 7'h25) /* 1=FREM, 0=FMOD */,
                   slotA_modrem_result, slotA_modrem_z, slotA_modrem_n, slotA_modrem_i, slotA_modrem_nan,
                   slotA_modrem_operr, slotA_modrem_quot_byte);
        fp_sincos(slotA_a_r, slotA_sin_result, slotA_cos_result, slotA_sincos_operr);
        fp_etox(slotA_a_r, slotA_etox_result, slotA_etox_z, slotA_etox_n, slotA_etox_i,
                slotA_etox_nan, slotA_etox_ovfl, slotA_etox_unfl);
        fp_etoxm1(slotA_a_r, slotA_etoxm1_result, slotA_etoxm1_z, slotA_etoxm1_n, slotA_etoxm1_i,
                  slotA_etoxm1_nan, slotA_etoxm1_ovfl, slotA_etoxm1_unfl);
        fp_twotox(slotA_a_r, slotA_twotox_result, slotA_twotox_z, slotA_twotox_n, slotA_twotox_i,
                  slotA_twotox_nan, slotA_twotox_ovfl, slotA_twotox_unfl);
        fp_tentox(slotA_a_r, slotA_tentox_result, slotA_tentox_z, slotA_tentox_n, slotA_tentox_i,
                  slotA_tentox_nan, slotA_tentox_ovfl, slotA_tentox_unfl);
        fp_sinh(slotA_a_r, slotA_sinh_result, slotA_sinh_z, slotA_sinh_n, slotA_sinh_i, slotA_sinh_nan);
        fp_cosh(slotA_a_r, slotA_cosh_result, slotA_cosh_z, slotA_cosh_n, slotA_cosh_i, slotA_cosh_nan);
        fp_tanh(slotA_a_r, slotA_tanh_result, slotA_tanh_z, slotA_tanh_n, slotA_tanh_i, slotA_tanh_nan);
        fp_tan(slotA_a_r, slotA_tan_result, slotA_tan_z, slotA_tan_n, slotA_tan_i, slotA_tan_nan, slotA_tan_operr);
        fp_logn(slotA_a_r, slotA_logn_result, slotA_logn_z, slotA_logn_n, slotA_logn_i, slotA_logn_nan,
                slotA_logn_operr, slotA_logn_dz);
        fp_lognp1(slotA_a_r, slotA_lognp1_result, slotA_lognp1_z, slotA_lognp1_n, slotA_lognp1_i, slotA_lognp1_nan,
                  slotA_lognp1_operr, slotA_lognp1_dz);
        fp_log10(slotA_a_r, slotA_log10_result, slotA_log10_z, slotA_log10_n, slotA_log10_i, slotA_log10_nan,
                 slotA_log10_operr, slotA_log10_dz);
        fp_log2(slotA_a_r, slotA_log2_result, slotA_log2_z, slotA_log2_n, slotA_log2_i, slotA_log2_nan,
                slotA_log2_operr, slotA_log2_dz);
        fp_atan(slotA_a_r, slotA_atan_result, slotA_atan_z, slotA_atan_n, slotA_atan_i, slotA_atan_nan);
        fp_asin(slotA_a_r, slotA_asin_result, slotA_asin_z, slotA_asin_n, slotA_asin_i, slotA_asin_nan, slotA_asin_operr);
        fp_acos(slotA_a_r, slotA_acos_result, slotA_acos_z, slotA_acos_n, slotA_acos_i, slotA_acos_nan, slotA_acos_operr);
        fp_atanh(slotA_a_r, slotA_atanh_result, slotA_atanh_z, slotA_atanh_n, slotA_atanh_i, slotA_atanh_nan,
                 slotA_atanh_operr, slotA_atanh_dz);
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
            7'h01, 7'h03: begin // FINT / FINTRZ
                slotA_result = slotA_int_result;
                slotA_flag_z = slotA_int_z; slotA_flag_n = slotA_int_n;
                slotA_flag_i = slotA_int_i; slotA_flag_nan = slotA_int_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = slotA_int_inex2;
            end
            7'h1E: begin // FGETEXP
                slotA_result = slotA_getexp_result;
                slotA_flag_z = slotA_getexp_z; slotA_flag_n = slotA_getexp_n;
                slotA_flag_i = slotA_getexp_i; slotA_flag_nan = slotA_getexp_nan;
                slotA_flag_operr = slotA_getexp_operr; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h1F: begin // FGETMAN
                slotA_result = slotA_getman_result;
                slotA_flag_z = slotA_getman_z; slotA_flag_n = slotA_getman_n;
                slotA_flag_i = slotA_getman_i; slotA_flag_nan = slotA_getman_nan;
                slotA_flag_operr = slotA_getman_operr; slotA_flag_dz = 1'b0; slotA_flag_ovfl = 1'b0;
                slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h26: begin // FSCALE (dyadic: slotA_a_r=RX=scale factor, slotA_b_r=RY=value+dest)
                slotA_result = slotA_scale_result;
                slotA_flag_z = slotA_scale_z; slotA_flag_n = slotA_scale_n;
                slotA_flag_i = slotA_scale_i; slotA_flag_nan = slotA_scale_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0; slotA_flag_ovfl = slotA_scale_ovfl;
                slotA_flag_unfl = slotA_scale_unfl; slotA_flag_inex2 = 1'b0;
            end
            7'h24: begin // FSGLDIV
                slotA_result = slotA_sgldiv_result;
                slotA_flag_z = slotA_sgldiv_z; slotA_flag_n = slotA_sgldiv_n;
                slotA_flag_i = slotA_sgldiv_i; slotA_flag_nan = slotA_sgldiv_nan;
                slotA_flag_operr = slotA_sgldiv_operr; slotA_flag_dz = slotA_sgldiv_dz;
                slotA_flag_ovfl = slotA_sgldiv_ovfl; slotA_flag_unfl = slotA_sgldiv_unfl;
                slotA_flag_inex2 = slotA_sgldiv_inex2;
            end
            7'h27: begin // FSGLMUL
                slotA_result = slotA_sglmul_result;
                slotA_flag_z = slotA_sglmul_z; slotA_flag_n = slotA_sglmul_n;
                slotA_flag_i = slotA_sglmul_i; slotA_flag_nan = slotA_sglmul_nan;
                slotA_flag_operr = slotA_sglmul_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = slotA_sglmul_ovfl; slotA_flag_unfl = slotA_sglmul_unfl;
                slotA_flag_inex2 = slotA_sglmul_inex2;
            end
            7'h21, 7'h25: begin // FMOD / FREM (quotient byte handled separately at commit)
                slotA_result = slotA_modrem_result;
                slotA_flag_z = slotA_modrem_z; slotA_flag_n = slotA_modrem_n;
                slotA_flag_i = slotA_modrem_i; slotA_flag_nan = slotA_modrem_nan;
                slotA_flag_operr = slotA_modrem_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h0E, 7'h1D: begin // FSIN / FCOS (approximated, see fp_sincos's own header comment)
                slotA_result = slotA_sincos_result;
                slotA_flag_z = slotA_sincos_z; slotA_flag_n = slotA_sincos_n;
                slotA_flag_i = slotA_sincos_i; slotA_flag_nan = slotA_sincos_nan;
                slotA_flag_operr = slotA_sincos_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h30, 7'h31, 7'h32, 7'h33, 7'h34, 7'h35, 7'h36, 7'h37: begin
                // FSINCOS: the PRIMARY committed result (into the usual
                // Ry destination, slotA_dest_r) is the SIN value --
                // condition codes/exception flags are likewise derived
                // from sin alone, matching Musashi's own
                // `SET_CONDITION_CODES(REG_FP[dst])` (dst == the sin
                // register). The cos value (slotA_cos_result) is written
                // to a SECOND register on commit's own extra tick below;
                // it never flows through this flag set at all.
                slotA_result = slotA_sin_result;
                slotA_flag_z = is_zero_fpx(unpack_fpx(slotA_sin_result));
                slotA_flag_n = slotA_sin_result[95] && !slotA_flag_z;
                slotA_flag_i = is_inf_fpx(unpack_fpx(slotA_sin_result));
                slotA_flag_nan = is_nan_fpx(unpack_fpx(slotA_sin_result));
                slotA_flag_operr = slotA_sincos_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h10: begin // FETOX
                slotA_result = slotA_etox_result;
                slotA_flag_z = slotA_etox_z; slotA_flag_n = slotA_etox_n;
                slotA_flag_i = slotA_etox_i; slotA_flag_nan = slotA_etox_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = slotA_etox_ovfl; slotA_flag_unfl = slotA_etox_unfl; slotA_flag_inex2 = 1'b0;
            end
            7'h08: begin // FETOXM1
                slotA_result = slotA_etoxm1_result;
                slotA_flag_z = slotA_etoxm1_z; slotA_flag_n = slotA_etoxm1_n;
                slotA_flag_i = slotA_etoxm1_i; slotA_flag_nan = slotA_etoxm1_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = slotA_etoxm1_ovfl; slotA_flag_unfl = slotA_etoxm1_unfl; slotA_flag_inex2 = 1'b0;
            end
            7'h11: begin // FTWOTOX
                slotA_result = slotA_twotox_result;
                slotA_flag_z = slotA_twotox_z; slotA_flag_n = slotA_twotox_n;
                slotA_flag_i = slotA_twotox_i; slotA_flag_nan = slotA_twotox_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = slotA_twotox_ovfl; slotA_flag_unfl = slotA_twotox_unfl; slotA_flag_inex2 = 1'b0;
            end
            7'h12: begin // FTENTOX
                slotA_result = slotA_tentox_result;
                slotA_flag_z = slotA_tentox_z; slotA_flag_n = slotA_tentox_n;
                slotA_flag_i = slotA_tentox_i; slotA_flag_nan = slotA_tentox_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = slotA_tentox_ovfl; slotA_flag_unfl = slotA_tentox_unfl; slotA_flag_inex2 = 1'b0;
            end
            7'h02: begin // FSINH
                slotA_result = slotA_sinh_result;
                slotA_flag_z = slotA_sinh_z; slotA_flag_n = slotA_sinh_n;
                slotA_flag_i = slotA_sinh_i; slotA_flag_nan = slotA_sinh_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h19: begin // FCOSH
                slotA_result = slotA_cosh_result;
                slotA_flag_z = slotA_cosh_z; slotA_flag_n = slotA_cosh_n;
                slotA_flag_i = slotA_cosh_i; slotA_flag_nan = slotA_cosh_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h09: begin // FTANH
                slotA_result = slotA_tanh_result;
                slotA_flag_z = slotA_tanh_z; slotA_flag_n = slotA_tanh_n;
                slotA_flag_i = slotA_tanh_i; slotA_flag_nan = slotA_tanh_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h0F: begin // FTAN
                slotA_result = slotA_tan_result;
                slotA_flag_z = slotA_tan_z; slotA_flag_n = slotA_tan_n;
                slotA_flag_i = slotA_tan_i; slotA_flag_nan = slotA_tan_nan;
                slotA_flag_operr = slotA_tan_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h14: begin // FLOGN
                slotA_result = slotA_logn_result;
                slotA_flag_z = slotA_logn_z; slotA_flag_n = slotA_logn_n;
                slotA_flag_i = slotA_logn_i; slotA_flag_nan = slotA_logn_nan;
                slotA_flag_operr = slotA_logn_operr; slotA_flag_dz = slotA_logn_dz;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h06: begin // FLOGNP1
                slotA_result = slotA_lognp1_result;
                slotA_flag_z = slotA_lognp1_z; slotA_flag_n = slotA_lognp1_n;
                slotA_flag_i = slotA_lognp1_i; slotA_flag_nan = slotA_lognp1_nan;
                slotA_flag_operr = slotA_lognp1_operr; slotA_flag_dz = slotA_lognp1_dz;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h15: begin // FLOG10
                slotA_result = slotA_log10_result;
                slotA_flag_z = slotA_log10_z; slotA_flag_n = slotA_log10_n;
                slotA_flag_i = slotA_log10_i; slotA_flag_nan = slotA_log10_nan;
                slotA_flag_operr = slotA_log10_operr; slotA_flag_dz = slotA_log10_dz;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h16: begin // FLOG2
                slotA_result = slotA_log2_result;
                slotA_flag_z = slotA_log2_z; slotA_flag_n = slotA_log2_n;
                slotA_flag_i = slotA_log2_i; slotA_flag_nan = slotA_log2_nan;
                slotA_flag_operr = slotA_log2_operr; slotA_flag_dz = slotA_log2_dz;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h0A: begin // FATAN
                slotA_result = slotA_atan_result;
                slotA_flag_z = slotA_atan_z; slotA_flag_n = slotA_atan_n;
                slotA_flag_i = slotA_atan_i; slotA_flag_nan = slotA_atan_nan;
                slotA_flag_operr = 1'b0; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h0C: begin // FASIN
                slotA_result = slotA_asin_result;
                slotA_flag_z = slotA_asin_z; slotA_flag_n = slotA_asin_n;
                slotA_flag_i = slotA_asin_i; slotA_flag_nan = slotA_asin_nan;
                slotA_flag_operr = slotA_asin_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h1C: begin // FACOS
                slotA_result = slotA_acos_result;
                slotA_flag_z = slotA_acos_z; slotA_flag_n = slotA_acos_n;
                slotA_flag_i = slotA_acos_i; slotA_flag_nan = slotA_acos_nan;
                slotA_flag_operr = slotA_acos_operr; slotA_flag_dz = 1'b0;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
            end
            7'h0D: begin // FATANH
                slotA_result = slotA_atanh_result;
                slotA_flag_z = slotA_atanh_z; slotA_flag_n = slotA_atanh_n;
                slotA_flag_i = slotA_atanh_i; slotA_flag_nan = slotA_atanh_nan;
                slotA_flag_operr = slotA_atanh_operr; slotA_flag_dz = slotA_atanh_dz;
                slotA_flag_ovfl = 1'b0; slotA_flag_unfl = 1'b0; slotA_flag_inex2 = 1'b0;
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

    // Single call site for fpsr_next (a function, but kept to one call
    // site anyway as a matter of consistent style with every task in
    // this file) -- the commit block below either uses this value
    // directly, or splices FMOD/FREM's own quotient byte into it.
    wire [31:0] slotA_fpsr_next_val = fpsr_next(fpsr_o, slotA_flag_n, slotA_flag_z, slotA_flag_i,
                                                 slotA_flag_nan, slotA_flag_snan, slotA_flag_operr,
                                                 slotA_flag_dz, slotA_flag_ovfl, slotA_flag_unfl,
                                                 slotA_flag_inex2);

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

    // Phase 12: Section 6.1.2 (SNAN) / 6.1.3 (Operand Error) / 6.1.6
    // (Divide-by-Zero) each say, for a floating-point-register
    // destination specifically, "the register is not modified" /
    // "the destination floating-point data register is not modified"
    // when THAT exception's own trap is enabled -- confirmed directly,
    // the opposite of OVFL/UNFL's own text (Section 6.1.4/6.1.5), which
    // says the result IS stored, same as trap-disabled. Only these 3
    // exceptions suppress the write; OVFL/UNFL/INEX2 keep writing
    // exactly as they always have (their own Trap-Enabled text already
    // matches current behavior).
    wire slotA_exc_trap_suppress = (fpcr_o[14] && slotA_flag_snan) ||
                                    (fpcr_o[13] && slotA_flag_operr) ||
                                    (fpcr_o[10] && slotA_flag_dz);

    // ── Phase 4c: external-operand format conversion (opclass 010/011)
    // -- unchanged in substance from Phase 4c/5, just retargeted onto the
    // registered cmd_rx_r/cmd_ry_r fields (dispatch now fires a tick
    // later than the Command CIR write itself). ─────────────────────────
    logic [31:0] supply_int32;
    logic [31:0] supply_single;
    logic [63:0] supply_double;
    logic [15:0] supply_int16;
    logic [7:0]  supply_int8;
    logic        supply_int32_operr, supply_single_operr, supply_double_operr;
    logic        supply_int16_operr, supply_int8_operr;
    logic [95:0] supply_staged;

    always_comb begin
        ext_to_int32(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_int32, supply_int32_operr);
        ext_to_single(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_single, supply_single_operr);
        ext_to_double(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_double, supply_double_operr);
        ext_to_int16(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_int16, supply_int16_operr);
        ext_to_int8(apu_b_rd, round_mode_t'(fpcr_o[5:4]), supply_int8, supply_int8_operr);

        // Phase 11: Word/Byte, like every format under 4 bytes (Figure
        // 7-4, "Operand CIR Data Alignment," confirmed directly), are
        // aligned with the MOST SIGNIFICANT byte of the (single) 32-bit
        // Operand CIR access -- the value sits in the TOP 16/8 bits of
        // the chunk, not the bottom.
        unique case (cmd_rx_r)
            FMT_L:   supply_staged = {supply_int32, 64'h0};
            FMT_S:   supply_staged = {supply_single, 64'h0};
            FMT_D:   supply_staged = {supply_double, 32'h0};
            FMT_X:   supply_staged = apu_b_rd; // native format, pure passthrough
            FMT_W:   supply_staged = {supply_int16, 16'h0, 64'h0};
            FMT_B:   supply_staged = {supply_int8, 24'h0, 64'h0};
            default: supply_staged = 96'h0; // P: not yet implemented (Phase 4c/14 scope)
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
    logic [95:0] receive_int16_ext, receive_int8_ext;
    always_comb begin
        int32_to_ext(receive_assembled[95:64], receive_int32_ext);
        single_to_ext(receive_assembled[95:64], receive_single_ext);
        double_to_ext(receive_assembled[95:32], receive_double_ext);
        // Phase 11: same MSB-alignment convention as supply_staged above
        // -- the Word/Byte value lives in the TOP 16/8 bits of the
        // single 32-bit chunk this format ever transfers.
        int16_to_ext(receive_assembled[95:80], receive_int16_ext);
        int8_to_ext(receive_assembled[95:88], receive_int8_ext);
    end

    logic [95:0] receive_converted;
    always_comb begin
        unique case (xfer_fmt_r)
            FMT_L:   receive_converted = receive_int32_ext;
            FMT_S:   receive_converted = receive_single_ext;
            FMT_D:   receive_converted = receive_double_ext;
            FMT_X:   receive_converted = receive_assembled; // native format, pure passthrough
            FMT_W:   receive_converted = receive_int16_ext;
            FMT_B:   receive_converted = receive_int8_ext;
            default: receive_converted = 96'h0; // P: not yet implemented
        endcase
    end

    // ── Phase 10: the full 32-condition Conditional Predicate Field
    // (Table 4-20/4.4). The manual's own printed Boolean equations have
    // lost negation-bar (overline) formatting in several places (directly
    // observed while transcribing them -- e.g. GE's own printed equation
    // is literally ambiguous without knowing which sub-term the lost bar
    // covered). tools/musashi/m68kfpu.c's own TEST_CONDITION() (already
    // vendored into this repo, already trusted elsewhere as a golden
    // reference) implements the identical 32-condition set unambiguously
    // in C, and confirms the encoding structure: predicate bit 4 (0x10)
    // selects the "signaling"/BSUN-checking group (Table 4-20 Note 2) vs.
    // the "ordered" group (Note 1, never sets BSUN); bits[3:0] select one
    // of 16 underlying Boolean tests, reused IDENTICALLY by both groups
    // (Musashi's own switch literally falls through `case 0x1X: case
    // 0x0X: return <formula>` for every one of the 16). cond_eval below
    // is a direct, line-for-line port of that switch -- the primary
    // source for the equations themselves, not the OCR'd manual text.
    function automatic logic cond_eval(logic [3:0] base, logic n, logic z, logic nan);
        // Plain case (not `unique case`) -- base is arithmetically
        // exhaustive over 4'h0..4'hF for any real predicate field, but at
        // simulation time 0 (before reset/first Condition CIR write)
        // cond_pred is still X, which `unique case` would flag as a
        // spurious violation every run (the same X-propagation shape
        // fp_sincos's own quadrant case already hit this session).
        case (base)
            4'h0: cond_eval = 1'b0;                // F / SF
            4'h1: cond_eval = z;                   // EQ / SEQ
            4'h2: cond_eval = !(nan || z || n);     // OGT / GT
            4'h3: cond_eval = z || !(nan || n);     // OGE / GE
            4'h4: cond_eval = n && !(nan || z);     // OLT / LT
            4'h5: cond_eval = z || (n && !nan);     // OLE / LE
            4'h6: cond_eval = !nan && !z;           // OGL / GL
            4'h7: cond_eval = !nan;                 // OR / GLE
            4'h8: cond_eval = nan;                  // UN / NGLE
            4'h9: cond_eval = nan || z;              // UEQ / NGL
            4'hA: cond_eval = nan || !(n || z);       // UGT / NLE
            4'hB: cond_eval = nan || z || !n;         // UGE / NLT
            4'hC: cond_eval = nan || (n && !z);       // ULT / NGE
            4'hD: cond_eval = nan || z || n;          // ULE / NGT
            4'hE: cond_eval = !z;                     // NE / SNE
            4'hF: cond_eval = 1'b1;                   // T / ST
        endcase
    endfunction

    // Condition CIR predicate field + FPSR N/Z/NAN condition-code bits
    // (combinational). This dialog is NOT gated by the mandatory-
    // Instruction-Address rule -- see the header comment's own scoping
    // note.
    wire [5:0] cond_pred     = d_in[21:16];
    wire       fpsr_n        = fpsr_o[27];
    wire       fpsr_z        = fpsr_o[26];
    wire       fpsr_nan      = fpsr_o[24];
    // Table 4-20 Note 3: predicate bit 5 set (0x20-0x3F) is "undefined,
    // reserved... redundant encodings with 0XXXXX" -- masked off here so
    // those 32 reserved codes evaluate identically to their own
    // low-5-bit twin rather than reading past cond_pred's own real range.
    wire       cond_bsun_group = cond_pred[4];
    wire       cond_tf_base    = cond_eval(cond_pred[3:0], fpsr_n, fpsr_z, fpsr_nan);
    // Table 4-20 Note 2: "If the NAN condition code bit is set, then set
    // the BSUN bit in the FPSR. If the BSUN trap is enabled, then return
    // the take pre-instruction exception primitive... otherwise, indicate
    // the condition true/false result" -- confirmed directly.
    wire       cond_bsun_fires = cond_bsun_group && fpsr_nan;
    wire       fpcr_bsun_enable = fpcr_o[15];
    logic      cond_tf_next;
    always_comb begin
        cond_tf_next = cond_tf_base;
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
            slotA_sincos_pending_r <= 1'b0;
            slotB_valid_r <= 1'b0;
            slotB_op_r    <= 3'h0;
            slotB_a_r     <= 96'h0;
            slotB_b_r     <= 96'h0;
            slotB_dest_r  <= 3'h0;
            slotB_round_r <= 2'h0;
            slotB_iaddr_r <= 32'h0;
            fpiar_auto_wr_en   <= 1'b0;
            fpiar_auto_wr_data <= 32'h0;
            bsun_set_en        <= 1'b0;
        end else begin
            fp_wr_en      <= 1'b0;
            ctrl_wr_en    <= 1'b0;
            apu_wr_en     <= 1'b0;
            null_reset_en <= 1'b0;
            proto_violation_r  <= 1'b0;
            fpiar_auto_wr_en   <= 1'b0;
            bsun_set_en        <= 1'b0;

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
                                    cmd_is_fabs || cmd_is_fneg || cmd_is_ftst || cmd_is_fmove ||
                                    cmd_is_fint || cmd_is_fintrz || cmd_is_fgetexp ||
                                    cmd_is_fgetman || cmd_is_fscale ||
                                    cmd_is_fsgldiv || cmd_is_fsglmul || cmd_is_fmod || cmd_is_frem ||
                                    cmd_is_fsin || cmd_is_fcos || cmd_is_fsincos ||
                                    cmd_is_fetox || cmd_is_fetoxm1 || cmd_is_ftwotox || cmd_is_ftentox ||
                                    cmd_is_fsinh || cmd_is_fcosh || cmd_is_ftanh || cmd_is_ftan ||
                                    cmd_is_flogn || cmd_is_flognp1 || cmd_is_flog10 || cmd_is_flog2 ||
                                    cmd_is_fatan || cmd_is_fasin || cmd_is_facos || cmd_is_fatanh) begin
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
                        // Table 4-20 Note 2: a "signaling" predicate
                        // (cond_pred[4]=1) evaluated while the NAN
                        // condition-code bit is set always sets BSUN;
                        // if FPCR's own BSUN trap-enable bit is ALSO
                        // set, the response becomes Take-Pre-Instruction-
                        // Exception instead of the ordinary null
                        // true/false result -- confirmed directly, not
                        // assumed.
                        if (cond_bsun_fires) bsun_set_en <= 1'b1;
                        if (cond_bsun_fires && fpcr_bsun_enable) begin
                            prim_r <= PRIM_TAKE_PRE;
                            ca_r   <= 1'b1;
                            // dr_r pinned for the same reason
                            // PRIM_TAKE_MID's own commit-path comment
                            // gives below -- a genuinely new non-null
                            // primitive that never otherwise sets dr_r.
                            dr_r   <= 1'b0;
                        end else begin
                            ca_r      <= 1'b0;
                            prim_r    <= PRIM_NULL;
                            cond_tf_r <= cond_tf_next;
                        end
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
                  if (slotA_is_fsincos && !slotA_sincos_pending_r) begin
                    // FSINCOS tick 1 of 2: write SIN to the usual Ry
                    // destination and set FPSR from sin's own condition
                    // codes now (matching Musashi's own single
                    // SET_CONDITION_CODES(REG_FP[dst]) call, dst==sin) --
                    // everything else (exception-trap reporting, slot-B
                    // promotion, FPIAR auto-load) deliberately waits for
                    // tick 2 below, since the instruction isn't "done"
                    // from the host's own point of view until BOTH
                    // registers have landed. apu_busy_cnt_r is re-armed
                    // to 1 rather than decremented, buying exactly one
                    // more tick through this same branch. Phase 12: SNAN/
                    // OPERR (the only two of the 3 suppressing exceptions
                    // FSINCOS can ever raise -- it never divides) also
                    // suppress this write when trap-enabled, same as
                    // every other op's own commit path.
                    if (!slotA_exc_trap_suppress) begin
                        apu_wr_en   <= 1'b1;
                        apu_wr_sel  <= slotA_dest_r;
                        apu_wr_data <= slotA_result; // == slotA_sin_result, via the mux above
                    end
                    ctrl_wr_en    <= 1'b1;
                    ctrl_wr_sel_r <= 2'd1; // FPSR
                    ctrl_wr_data  <= slotA_fpsr_next_val;
                    slotA_sincos_pending_r <= 1'b1;
                    apu_busy_cnt_r <= 12'd1;
                  end else begin
                    if (slotA_sincos_pending_r) begin
                        // FSINCOS tick 2 of 2: write COS to the SECOND,
                        // opmode-encoded register (slotA_op_r[2:0] -- the
                        // extension word's own low 3 bits, confirmed
                        // against Musashi's own `REG_FP[opmode&7]`). FPSR
                        // was already finalized on tick 1; don't re-touch
                        // it here.
                        // Phase 12: same trap-enabled write suppression
                        // as tick 1 -- if the whole instruction is being
                        // suppressed, neither register lands, not just
                        // the first one.
                        if (!slotA_exc_trap_suppress) begin
                            apu_wr_en   <= 1'b1;
                            apu_wr_sel  <= slotA_op_r[2:0];
                            apu_wr_data <= slotA_cos_result;
                        end
                        slotA_sincos_pending_r <= 1'b0;
                    end else begin
                        // Section 4.5.5.1: FCMP compares "as if" FPn-source
                        // were computed, but FPn itself is never written.
                        // FTST likewise never writes (source-only, condition
                        // codes only). Phase 12: SNAN/OPERR/DZ additionally
                        // suppress the write whenever THAT exception's own
                        // trap is enabled (slotA_exc_trap_suppress's own
                        // header comment).
                        if (slotA_op_r != 7'h38 && slotA_op_r != 7'h3A && !slotA_exc_trap_suppress) begin
                            apu_wr_en   <= 1'b1;
                            apu_wr_sel  <= slotA_dest_r;
                            apu_wr_data <= slotA_result;
                        end
                        ctrl_wr_en    <= 1'b1;
                        ctrl_wr_sel_r <= 2'd1; // FPSR
                        // Section 2.3.2/Figure 2-5: FMOD/FREM ALONE also load
                        // the FPSR quotient byte (bits[23:16]) -- every other
                        // instruction's own Status Register table entry reads
                        // "Quotient Byte: Not affected," confirmed directly.
                        if (slotA_op_r == 7'h21 || slotA_op_r == 7'h25) begin
                            ctrl_wr_data <= {slotA_fpsr_next_val[31:24], slotA_modrem_quot_byte,
                                              slotA_fpsr_next_val[15:0]};
                        end else begin
                            ctrl_wr_data <= slotA_fpsr_next_val;
                        end
                    end
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
