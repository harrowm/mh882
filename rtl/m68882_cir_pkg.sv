`timescale 1ns/1ps
`default_nettype none

// MC68882 CIR field definitions -- single source of truth for register
// offsets, per-CIR properties, and the DSACK/port-size encoding table.
// Confirmed directly against Section 9.1 ("Address Bus"), Section 9.3
// ("SIZE"), Section 9.8 ("DSACK0, DSACK1", Table 9-3), and Section 10.4
// of the manual. Centralizing these mirrors the sibling MH030 project's
// own `rtl/opcode_fields.sv` precedent (avoid hand-copied field
// positions turning into a bug class -- CLAUDE.md's own ext_count
// de-duplication plan).

package m68882_cir_pkg;

    // Table 9-1 -- register select is A4-A1 (a 4-bit field, 2-byte
    // granularity) whenever the FPCP operates on a 16- or 32-bit system
    // data bus; A0 is NOT a free address bit in that mode. Section 9.1:
    // "When the FPCP operates with an 8-bit data bus, the A0 pin is used
    // as an address signal ... When the FPCP operates with a 16- or
    // 32-bit system data bus, both A0 and SIZE pins are strapped" (Table
    // 9-2) to configure port width instead. This RTL targets 16/32-bit
    // system buses only (the companion MH030 project's own 68030 bus is
    // 32-bit) -- an 8-bit-bus mode would need A0 folded back in as a
    // genuine address bit, not modeled here.
    typedef logic [3:0] cir_sel_t;

    localparam cir_sel_t CIR_RESPONSE  = 4'h0; // $00, 16-bit, Read
    localparam cir_sel_t CIR_CONTROL   = 4'h1; // $02, 16-bit, Write
    localparam cir_sel_t CIR_SAVE      = 4'h2; // $04, 16-bit, Read
    localparam cir_sel_t CIR_RESTORE   = 4'h3; // $06, 16-bit, Read/Write
    localparam cir_sel_t CIR_RSVD_08   = 4'h4; // $08, reserved
    localparam cir_sel_t CIR_COMMAND   = 4'h5; // $0A, 16-bit, Write
    localparam cir_sel_t CIR_RSVD_0C   = 4'h6; // $0C, reserved
    localparam cir_sel_t CIR_CONDITION = 4'h7; // $0E, 16-bit, Write
    localparam cir_sel_t CIR_OPERAND   = 4'h8; // $10, 32-bit, Read/Write
    localparam cir_sel_t CIR_REGSELECT = 4'hA; // $14, 16-bit, Read
    localparam cir_sel_t CIR_RSVD_16   = 4'hB; // $16, reserved
    localparam cir_sel_t CIR_INSTRADDR = 4'hC; // $18, 32-bit, Write
    localparam cir_sel_t CIR_OPNDADDR  = 4'hE; // $1C, 32-bit, Read/Write (dead
                                                // on real 68881/68882 silicon)

    function automatic logic is_sync_read(cir_sel_t sel);
        // Section 10.4.1: Response and Save CIRs ONLY use the
        // synchronous-read model; every other CIR read is asynchronous.
        return (sel == CIR_RESPONSE) || (sel == CIR_SAVE);
    endfunction

    function automatic logic is_32bit(cir_sel_t sel);
        // Table 9-1: Operand/Instruction-Address/Operand-Address are the
        // only 32-bit-wide registers; everything else is 16-bit.
        return (sel == CIR_OPERAND) || (sel == CIR_INSTRADDR) || (sel == CIR_OPNDADDR);
    endfunction

    // Port size, derived from SIZE#/A0 per Table 9-2 (both are hardware
    // straps in 16/32-bit bus mode, sampled here like any other
    // synchronized input).
    typedef enum logic [1:0] {
        PORT_8BIT  = 2'b00,
        PORT_16BIT = 2'b01,
        PORT_32BIT = 2'b10
    } port_size_t;

    function automatic port_size_t port_size(logic size_n, logic a0);
        if (!size_n)          // SIZE# asserted -> 8-bit bus (A0 don't-care)
            return PORT_8BIT;
        else if (a0)          // SIZE# negated, A0=1 -> 32-bit bus (Table 9-2)
            return PORT_32BIT;
        else                  // SIZE# negated, A0=0 -> 16-bit bus
            return PORT_16BIT;
    endfunction

    // Table 9-3 -- DSACK encoding as a function of port size and, for the
    // 32-bit-port case only, whether the selected register is in the
    // upper (A4=1, 32-bit-wide) or lower (A4=0, 16-bit-wide) half of the
    // CIR address range. Returned as {dsack1_n, dsack0_n}.
    //
    // CORRECTION (found building Phase 2, superseding Phase 1's own
    // placeholder): Phase 1's `m68882_biu.sv` asserted dsack0_n with
    // dsack1_n held negated as its own "16-bit-port placeholder." Table
    // 9-3 actually shows DSACK1=L/DSACK0=H for a 16-bit port -- the
    // OPPOSITE polarity assignment. Phase 1's encoding was, in fact, the
    // 8-BIT-port encoding, not 16-bit. Replaced here with this real,
    // table-driven function instead of a hand-picked constant.
    function automatic logic [1:0] dsack_encode(port_size_t sz, logic a4);
        case (sz)
            PORT_32BIT: return a4 ? 2'b00   // valid data on D31-D00
                                   : 2'b01;  // valid data on D31-D16 -- Section
                                             // 9.8's own explicit note: this
                                             // chip's 16-bit registers always
                                             // live on D16-D31, never D0-D15,
                                             // in a 32-bit port, regardless of
                                             // the "natural" odd/even word
                                             // address the register sits at
            PORT_16BIT: return 2'b01;       // valid data on D31-D16 or D15-D0
                                             // (system-wiring dependent; not
                                             // resolvable by this chip alone)
            PORT_8BIT:  return 2'b10;       // valid data on whichever byte lane
            default:    return 2'b11;       // not reached
        endcase
    endfunction

    // ────────────────────────────────────────────────────────────────
    // Phase 3: Command word decode (Section 4.7.1, Table 4-11) and the
    // Operand CIR data-format table (confirmed via the FADD/general
    // instruction field description: 000=L, 001=S, 010=X, 011=P, 100=W,
    // 101=D, 110=B, 111=unused).
    // ────────────────────────────────────────────────────────────────

    // The 16-bit word written to the Command CIR ($0A) is the general
    // instruction format's own "command word" (Section 4.7.1): OPCLASS
    // (bits 15-13), RX (bits 12-10), RY (bits 9-7), EXTENSION (bits 6-0).
    function automatic logic [2:0] cmd_opclass(logic [15:0] cmd);
        return cmd[15:13];
    endfunction
    function automatic logic [2:0] cmd_rx(logic [15:0] cmd);
        return cmd[12:10];
    endfunction
    function automatic logic [2:0] cmd_ry(logic [15:0] cmd);
        return cmd[9:7];
    endfunction
    function automatic logic [6:0] cmd_ext(logic [15:0] cmd);
        return cmd[6:0];
    endfunction

    // Data-format code (used as the RX field for opclass 010 "external
    // operand to FPn", and as the RX field for opclass 011 "FPm to
    // external destination") -- confirmed directly (the FADD instruction
    // field description, "Source Specifier Field"): 000=L, 001=S, 010=X,
    // 011=P, 100=W, 101=D, 110=B. 111 is unused for this field (opclass
    // 010 repurposes RX=111 to mean "move constant" instead, a different
    // instruction class entirely -- see Table 4-11).
    localparam logic [2:0] FMT_L = 3'b000; // Long Word Integer, 4 bytes
    localparam logic [2:0] FMT_S = 3'b001; // Single Precision Real, 4 bytes
    localparam logic [2:0] FMT_X = 3'b010; // Extended Precision Real, 12 bytes
    localparam logic [2:0] FMT_P = 3'b011; // Packed Decimal Real, 12 bytes
    localparam logic [2:0] FMT_W = 3'b100; // Word Integer, 2 bytes
    localparam logic [2:0] FMT_D = 3'b101; // Double Precision Real, 8 bytes
    localparam logic [2:0] FMT_B = 3'b110; // Byte Integer, 1 byte

    // Total operand byte count per format (Figure 7-4, Operand CIR Data
    // Alignment: B/W/3-byte/L-or-S are each ONE Operand CIR access;
    // D is TWO; X/P are THREE -- matching each FP register's own 12-byte
    // internal storage size).
    function automatic int unsigned fmt_bytes(logic [2:0] fmt);
        unique case (fmt)
            FMT_B: return 1;
            FMT_W: return 2;
            FMT_L, FMT_S: return 4;
            FMT_D: return 8;
            FMT_X, FMT_P: return 12;
            default: return 4;
        endcase
    endfunction

    // ────────────────────────────────────────────────────────────────
    // Response Primitive protocol (Section 7.4.2). Bit positions
    // CA(15)/PC(14)/DR(13) are confirmed directly against the manual.
    // The primitive-identifying payload in bits[12:0] is NOT confirmed
    // against an explicit numeric encoding table in this project's own
    // manual extraction (the relevant sub-section's exact bit-level
    // primitive codes were not located) -- the 6 values below are this
    // project's OWN internally-consistent assignment, sufficient to
    // drive and test the real dialog-sequencing logic Phase 3 exists to
    // build, but should be treated as unconfirmed against real silicon
    // until cross-checked (e.g. against Musashi's own 68881 emulation
    // source, MH030's tools/musashi/, a clean non-OCR reference this
    // project already has on hand).
    typedef enum logic [12:0] {
        PRIM_NULL       = 13'h0000,
        PRIM_EVAL_EA    = 13'h0001, // Evaluate EA and Transfer Data
        PRIM_XFER_SINGLE= 13'h0002, // Transfer Single Main Processor Register
        PRIM_XFER_MULTI = 13'h0003, // Transfer Multiple Coprocessor Registers
        PRIM_TAKE_PRE   = 13'h0004, // Take Pre-Instruction Exception
        PRIM_TAKE_MID   = 13'h0005  // Take Mid-Instruction Exception
    } prim_id_t;

    function automatic logic [15:0] response_word(
        logic ca, logic pc, logic dr, logic [12:0] payload
    );
        return {ca, pc, dr, payload};
    endfunction

endpackage
