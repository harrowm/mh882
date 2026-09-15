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

endpackage
