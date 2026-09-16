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
// This is plain storage with simple word-addressed read/write ports --
// no arithmetic (Phase 4) and no pipeline-stage instruction-address
// tracking (Phase 6, which needs FPIAR to become 3 separate per-stage
// registers internally). A single flat FPIAR is sufficient for Phase 3's
// own protocol-only scope.

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

    // Direct debug/condition-evaluation read ports (Phase 3's own
    // Condition CIR logic needs FPSR's condition-code byte without going
    // through the chunked ctrl_sel port)
    output logic [31:0] fpsr_o
);

    logic [95:0] fp_r [0:7];
    logic [31:0] fpcr_r;
    logic [31:0] fpsr_r;
    logic [31:0] fpiar_r;

    assign fpsr_o = fpsr_r;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 8; i++) fp_r[i] <= '0;
            fpcr_r  <= '0;
            fpsr_r  <= '0;
            fpiar_r <= '0;
        end else begin
            if (fp_wr_en) begin
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
                    2'd1: fpsr_r  <= ctrl_wr_data;
                    2'd2: fpiar_r <= ctrl_wr_data;
                    default: ;
                endcase
            end
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
