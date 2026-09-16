`timescale 1ns/1ps
`default_nettype none

// MC68882 programming-model register file -- Phase 3.
//
// FP0-FP7: each 12 bytes (96 bits) of internal storage -- confirmed
// directly (Section 7.2.9: "Each FPCP floating-point data register is
// 12 bytes long, and thus requires three long word transfers"), even
// though IEEE extended precision is nominally 80 bits; the extra 16
// bits are real on-chip storage, not modeled padding.
//
// FPCR/FPSR/FPIAR: 32-bit each (Section 1.2/2.3).
//
// This is plain storage with simple word-addressed read/write ports.
//
// Phase 6: FPIAR gained a SECOND, dedicated write port
// (fpiar_auto_wr_en/fpiar_auto_wr_data), separate from the general
// ctrl_wr_en/ctrl_wr_sel/ctrl_wr_data port every other control register
// still shares. Reason: the APU pipeline (m68882_proto.sv) can need to
// commit a just-finished instruction's FPSR update AND auto-load FPIAR
// with the address of the instruction now ENTERING the APU stage (slotB
// promoted to slotA) on the very same cycle -- two independent control-
// register writes in one tick, which the old single shared port could
// never express. FPIAR itself remains the single, real "APU-stage
// instruction address" register (plan.md's own Phase 0 research: of the
// 3 per-pipeline-stage instruction-address registers the 68882 genuinely
// has, FPIAR is specifically the APU one -- the only one visible to the
// programmer). The CU-stage register (m68882_proto.sv's own
// cu_instr_addr_r) is internal, not part of this file's programmer-
// visible register file.

module m68882_regfile (
    input  logic clk_4x,
    input  logic rst_n,

    // FP0-7 READ access: one 32-bit chunk (chunk_idx: 0=bits[95:64],
    // 1=bits[63:32], 2=bits[31:0] -- MSB-first, matching the Operand CIR's
    // own MSB-aligned transfer convention, Figure 7-4) per cycle.
    input  logic [2:0] fp_sel,
    input  logic [1:0] fp_chunk_idx,
    output logic [31:0] fp_rd_data,

    // FP0-7 WRITE access -- a SEPARATE selector from the read side above,
    // deliberately: the caller (m68882_proto.sv) advances its own "next
    // register/chunk" pointer in the SAME cycle it asserts fp_wr_en (for
    // the register/chunk THIS write belongs to), so a write selector
    // sharing the live read-side pointer would race against that same-
    // cycle advance and target the wrong register one tick later. The
    // caller captures fp_wr_sel/fp_wr_chunk via nonblocking assignment
    // alongside fp_wr_en itself, which naturally freezes the pre-advance
    // value.
    input  logic [2:0] fp_wr_sel,
    input  logic [1:0] fp_wr_chunk,
    input  logic       fp_wr_en,
    input  logic [31:0] fp_wr_data,

    // FPCR/FPSR/FPIAR: selected by a 2-bit index (0=FPCR,1=FPSR,2=FPIAR).
    // Same read/write selector split as FP0-7, for the same reason.
    input  logic [1:0] ctrl_sel,
    output logic [31:0] ctrl_rd_data,

    input  logic [1:0] ctrl_wr_sel,
    input  logic       ctrl_wr_en,
    input  logic [31:0] ctrl_wr_data,

    // Phase 6: dedicated FPIAR auto-load port (see header comment) --
    // takes priority over ctrl_wr_en/ctrl_wr_sel==FPIAR on any cycle both
    // happen to be asserted together (the automatic pipeline load is the
    // architecturally "later" event within the same tick: slotB only
    // promotes to slotA AFTER slotA's own commit has already claimed the
    // shared ctrl_wr_en port for the FPSR write).
    input  logic       fpiar_auto_wr_en,
    input  logic [31:0] fpiar_auto_wr_data,

    // Phase 10: dedicated BSUN-bit-set port (Table 4-20 Note 2: "set the
    // BSUN bit in the FPSR" -- a single-bit OR, not a full FPSR
    // overwrite, since a Condition CIR evaluation's own EXC-byte
    // contribution is scoped to BSUN alone, unlike an arithmetic op's
    // own commit-time fpsr_next() which legitimately refreshes all 8 EXC
    // bits together). Composes safely with a same-cycle ctrl_wr_en/FPSR
    // write below (spliced into that write's own data) rather than
    // racing it with a second, independent bit-select assignment to the
    // same register.
    input  logic       bsun_set_en,

    // Direct debug/condition-evaluation read ports (Phase 3's own
    // Condition CIR logic needs FPSR's condition-code byte without going
    // through the chunked ctrl_sel port)
    output logic [31:0] fpsr_o,
    output logic [31:0] fpcr_o,
    output logic [31:0] fpiar_o,

    // Phase 5: does an FRESTORE-of-a-null-frame need to reset the WHOLE
    // programmer's model? -- Section 6.4.2.1: "the programmer's model is
    // set to the reset state" (all zero). all_zero_o lets the Save CIR
    // dialog classify Null vs Idle without its own separate read port
    // for all 8 FP registers; null_reset_en is a one-tick pulse that
    // clears every register the same way rst_n does.
    output logic         all_zero_o,
    input  logic          null_reset_en,

    // Whole-register FP0-7 ports for the APU (Phase 4) -- arithmetic
    // needs both source operands available combinationally in the SAME
    // cycle (unlike the chunked Operand CIR path above, which only ever
    // moves one 32-bit slice at a time), and writes a complete 96-bit
    // result in one cycle rather than three chunked writes.
    input  logic [2:0]  apu_a_sel,
    input  logic [2:0]  apu_b_sel,
    output logic [95:0] apu_a_rd,
    output logic [95:0] apu_b_rd,
    input  logic         apu_wr_en,
    input  logic [2:0]  apu_wr_sel,
    input  logic [95:0] apu_wr_data
);

    logic [95:0] fp_r [0:7];
    logic [31:0] fpcr_r;
    logic [31:0] fpsr_r;
    logic [31:0] fpiar_r;

    assign fpsr_o   = fpsr_r;
    assign fpcr_o   = fpcr_r;
    assign fpiar_o  = fpiar_r;
    assign apu_a_rd = fp_r[apu_a_sel];
    assign apu_b_rd = fp_r[apu_b_sel];
    assign all_zero_o = (fpcr_r == 32'h0) && (fpsr_r == 32'h0) && (fpiar_r == 32'h0) &&
                         (fp_r[0] == 96'h0) && (fp_r[1] == 96'h0) && (fp_r[2] == 96'h0) &&
                         (fp_r[3] == 96'h0) && (fp_r[4] == 96'h0) && (fp_r[5] == 96'h0) &&
                         (fp_r[6] == 96'h0) && (fp_r[7] == 96'h0);

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n || null_reset_en) begin
            for (int i = 0; i < 8; i++) fp_r[i] <= '0;
            fpcr_r  <= '0;
            fpsr_r  <= '0;
            fpiar_r <= '0;
        end else begin
            if (apu_wr_en) begin
                fp_r[apu_wr_sel] <= apu_wr_data;
            end else if (fp_wr_en) begin
                unique case (fp_wr_chunk)
                    2'd0: fp_r[fp_wr_sel][95:64] <= fp_wr_data;
                    2'd1: fp_r[fp_wr_sel][63:32] <= fp_wr_data;
                    2'd2: fp_r[fp_wr_sel][31:0]  <= fp_wr_data;
                    default: ;
                endcase
            end
            if (ctrl_wr_en) begin
                unique case (ctrl_wr_sel)
                    2'd0: fpcr_r  <= ctrl_wr_data;
                    2'd1: fpsr_r  <= ctrl_wr_data | (bsun_set_en ? 32'h0000_8000 : 32'h0);
                    2'd2: if (!fpiar_auto_wr_en) fpiar_r <= ctrl_wr_data;
                    default: ;
                endcase
            end else if (bsun_set_en) begin
                fpsr_r[15] <= 1'b1;
            end
            // Phase 6: independent of the block above, so an FPSR commit
            // write (ctrl_wr_sel==FPSR) and an FPIAR auto-load can both
            // land in the same cycle.
            if (fpiar_auto_wr_en) fpiar_r <= fpiar_auto_wr_data;
        end
    end

    always_comb begin
        unique case (fp_chunk_idx)
            2'd0: fp_rd_data = fp_r[fp_sel][95:64];
            2'd1: fp_rd_data = fp_r[fp_sel][63:32];
            2'd2: fp_rd_data = fp_r[fp_sel][31:0];
            default: fp_rd_data = 32'h0;
        endcase
    end

    always_comb begin
        unique case (ctrl_sel)
            2'd0: ctrl_rd_data = fpcr_r;
            2'd1: ctrl_rd_data = fpsr_r;
            2'd2: ctrl_rd_data = fpiar_r;
            default: ctrl_rd_data = 32'h0;
        endcase
    end

endmodule
