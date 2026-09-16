`timescale 1ns/1ps
`default_nettype none

// MC68882 Arithmetic Processing Unit -- Phase 4a: extended-precision
// ADD/SUB only (register-to-register). The rest of the arithmetic set
// (MUL/DIV/SQRT/transcendentals, external-operand format conversion) is
// explicitly deferred to later Phase 4 sub-phases -- plan.md's own
// "needs its own sub-phasing" note.
//
// Internal format (Section 3.5.1/Table 3-3, confirmed directly): each
// 96-bit register is sign(1)/exponent(15,bias 16383)/reserved(16,zero)/
// integer-bit(1)/fraction(63) -- the integer bit + fraction together
// form a 64-bit "j.f" mantissa sitting in bits[63:0]. This is genuinely
// x87-style extended precision with an EXPLICIT integer bit (not IEEE
// double's implicit-1 convention) -- confirmed, not assumed, from the
// manual's own field-size table.
//
// Extension-field opcodes (Table 4-13, confirmed): $22=FADD, $28=FSUB.
//
// Scope, explicitly bounded for this first slice:
//   - Handles normal finite operands, signed zero, infinity, and NaN
//     (propagated, not manufactured from scratch -- if either operand
//     is already a NaN, that NaN passes through; if both are NaN, this
//     project's own choice is to propagate the FIRST operand's NaN,
//     documented here since the manual doesn't specify a tie-break).
//   - Denormals are NOT specially handled: a result that would
//     underflow below the normalized range collapses to a correctly-
//     signed zero rather than a true denormal. A real gap, not a
//     silent one -- revisit alongside UNFL exception support in a
//     later Phase 4 sub-phase.
//   - Exponent overflow saturates to infinity (a coarse OVFL substitute,
//     not real trap/exception-frame semantics -- Phase 4's own
//     exception-byte work covers that properly later).
//   - Rounding: all 4 FPCR rounding modes (Section 1.2's own Mode
//     Control byte bits[5:4]) are implemented via a 3-bit guard/round/
//     sticky tail, including the round-to-nearest-even tie-break and
//     the manual's own explicit "+0.0 in RN/RZ/RP, -0.0 in RM" rule for
//     an exact opposite-sign cancellation (the FADD Operation Table's
//     own Note 1).
//   - Infinity-minus-infinity (opposite effective signs) sets OPERR and
//     produces a NaN result, per the FADD Operation Table's own Note 2.

package m68882_apu_pkg;

    typedef struct packed {
        logic        sign;
        logic [14:0] exp;
        logic [63:0] mant; // j.f, bit63=integer bit
    } fpx_t;

    function automatic fpx_t unpack_fpx(logic [95:0] r);
        fpx_t x;
        x.sign = r[95];
        x.exp  = r[94:80];
        x.mant = r[63:0];
        return x;
    endfunction

    function automatic logic [95:0] pack_fpx(fpx_t x);
        return {x.sign, x.exp, 16'h0, x.mant};
    endfunction

    function automatic logic is_zero_fpx(fpx_t x);
        return (x.exp == 15'h0) && (x.mant == 64'h0);
    endfunction

    function automatic logic is_inf_fpx(fpx_t x);
        return (x.exp == 15'h7FFF) && (x.mant == 64'h8000_0000_0000_0000);
    endfunction

    function automatic logic is_nan_fpx(fpx_t x);
        return (x.exp == 15'h7FFF) && (x.mant != 64'h8000_0000_0000_0000);
    endfunction

    // Rounding-mode encoding (Section 1.2's own Mode Control byte
    // bits[5:4]): 00=Nearest, 01=Toward-Zero, 10=Toward-(-Inf),
    // 11=Toward-(+Inf).
    typedef enum logic [1:0] {
        RND_NEAREST = 2'b00,
        RND_ZERO    = 2'b01,
        RND_MINF    = 2'b10,
        RND_PINF    = 2'b11
    } round_mode_t;

    // Round a 67-bit working mantissa (64-bit result + guard/round/
    // sticky) up or down per the rounding mode and the result's own
    // sign, returning the rounded 64-bit mantissa and a carry-out flag
    // (mantissa overflowed to 65 bits and must be renormalized by the
    // caller).
    task automatic round_mantissa(
        input  logic [63:0] mant_in,
        input  logic         guard,
        input  logic         round_bit,
        input  logic         sticky,
        input  logic         result_sign,
        input  round_mode_t  mode,
        output logic [63:0] mant_out,
        output logic         carry_out
    );
        logic round_up;
        unique case (mode)
            RND_ZERO: round_up = 1'b0;
            RND_MINF: round_up = result_sign && (guard || round_bit || sticky);
            RND_PINF: round_up = !result_sign && (guard || round_bit || sticky);
            default:  // RND_NEAREST, round-to-nearest-even
                round_up = guard && (round_bit || sticky || mant_in[0]);
        endcase

        if (round_up) begin
            {carry_out, mant_out} = {1'b0, mant_in} + 65'd1;
        end else begin
            mant_out  = mant_in;
            carry_out = 1'b0;
        end
    endtask

    // Leading-zero count of a 64-bit value (0-64). Used to renormalize
    // after a magnitude subtraction shortens the mantissa.
    function automatic logic [6:0] lzc64(logic [63:0] v);
        for (int i = 0; i < 64; i++)
            if (v[63-i]) return i[6:0];
        return 7'd64;
    endfunction

    // ADD (is_sub=0) / SUB (is_sub=1) of two extended-precision operands.
    // `a` is the source (command word's own RX field), `b` the
    // destination accumulator (RY field) -- matching the manual's own
    // "Source + FPn -> FPn" (FADD) / "FPn - Source -> FPn" (FSUB, this
    // project's own inferred convention, symmetric with FADD) operation
    // order.
    task automatic fp_add_sub(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        input  logic         is_sub,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr
    );
        fpx_t a, b, eff_a;
        logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;

        a = unpack_fpx(a_raw);
        b = unpack_fpx(b_raw);
        eff_a = a;
        if (is_sub) eff_a.sign = !a.sign; // FSUB: FPn - source == FPn + (-source)

        a_nan  = is_nan_fpx(a);
        b_nan  = is_nan_fpx(b);
        a_inf  = is_inf_fpx(eff_a);
        b_inf  = is_inf_fpx(b);
        a_zero = is_zero_fpx(a);
        b_zero = is_zero_fpx(b);

        flag_operr = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (b_nan) begin
            result = b_raw;
        end else if (a_inf && b_inf) begin
            if (eff_a.sign != b.sign) begin
                // opposite-sign infinities: undefined, OPERR (FADD
                // Operation Table Note 2)
                flag_operr = 1'b1;
                result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000}; // a NaN
            end else begin
                result = {b.sign, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end
        end else if (a_inf) begin
            result = {eff_a.sign, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
        end else if (b_inf) begin
            result = b_raw;
        end else begin
            // Both finite (includes zero as an exp=0/mant=0 subset).
            logic [14:0] exp_a, exp_b, exp_hi, exp_lo, exp_diff;
            logic [66:0] mant_a, mant_b, mant_hi, mant_lo, mant_hi_al, mant_lo_al;
            logic        sign_hi, sign_lo;
            logic        same_sign;
            logic [66:0] sum67;
            logic [14:0] result_exp;
            logic        result_sign;
            logic [63:0] rounded_mant;
            logic        carry;
            logic [6:0]  shift;
            logic [6:0]  lz;

            exp_a  = a.exp;
            exp_b  = b.exp;
            mant_a = {a.mant, 3'b0};
            mant_b = {b.mant, 3'b0};

            if ({exp_a, a.mant} >= {exp_b, b.mant}) begin
                exp_hi = exp_a; mant_hi = mant_a; sign_hi = eff_a.sign;
                exp_lo = exp_b; mant_lo = mant_b; sign_lo = b.sign;
            end else begin
                exp_hi = exp_b; mant_hi = mant_b; sign_hi = b.sign;
                exp_lo = exp_a; mant_lo = mant_a; sign_lo = eff_a.sign;
            end

            exp_diff = exp_hi - exp_lo;
            shift    = (exp_diff > 15'd67) ? 7'd67 : exp_diff[6:0];

            mant_hi_al = mant_hi;
            // Shift the smaller operand right, folding shifted-out bits
            // into a sticky bit (bit 0 of the working register) rather
            // than simply discarding them.
            if (shift == 7'd0) begin
                mant_lo_al = mant_lo;
            end else if (shift >= 7'd67) begin
                mant_lo_al = {66'b0, |mant_lo};
            end else begin
                logic [66:0] shifted;
                logic        sticky_bits;
                shifted     = mant_lo >> shift;
                sticky_bits = |(mant_lo & ((67'b1 << shift) - 67'b1));
                mant_lo_al  = shifted | {66'b0, sticky_bits};
            end

            same_sign = (sign_hi == sign_lo);

            if (same_sign) begin
                // A genuine same-sign add: mant_hi_al ALWAYS has its own
                // bit66 (the normalized integer bit) set to 1 -- that is
                // NOT an overflow signal, just the ordinary "1.xxx"
                // shape. The real overflow condition (result >= 2.0 at
                // this exponent scale, needing a right-shift + exponent
                // bump) can only be detected with a genuine carry-out
                // BEYOND bit66, so the addition needs one extra bit of
                // width. (BUG FOUND via simulation: an earlier version of
                // this code checked sum67[66] itself as the overflow
                // flag, which is always 1 for any normal same-sign add
                // and silently double-counted the exponent bump -- e.g.
                // 1.0+2.0 came out as 7.5 instead of 3.0.)
                logic [67:0] sum68;
                sum68       = {1'b0, mant_hi_al} + {1'b0, mant_lo_al};
                result_sign = sign_hi;
                result_exp  = exp_hi;
                if (sum68[67]) begin
                    logic lost;
                    lost       = sum68[0];
                    sum68      = sum68 >> 1;
                    sum68[0]   = sum68[0] | lost;
                    result_exp = exp_hi + 15'd1;
                end
                sum67 = sum68[66:0];
            end else begin
                sum67       = mant_hi_al - mant_lo_al;
                result_sign = sign_hi;
                result_exp  = exp_hi;
            end

            if (sum67 == 67'b0) begin
                // exact cancellation -- Note 1: +0.0 except in RM (-0.0)
                result_sign = (rmode == RND_MINF);
                result_exp  = 15'h0;
                result      = {result_sign, 15'h0, 16'h0, 64'h0};
                flag_operr  = 1'b0;
            end else begin
                if (!same_sign) begin
                    // magnitude subtraction may have shortened the
                    // mantissa -- renormalize by left-shifting out the
                    // leading zeros (the guard/round/sticky tail is
                    // still bits[2:0], the true j.f is bits[66:3]).
                    lz = lzc64(sum67[66:3]);
                    if (lz != 7'd0) begin
                        if ({15'b0, lz} >= {8'b0, result_exp}) begin
                            // underflows through zero -- Phase 4a's own
                            // documented denormal-free simplification:
                            // collapse to a correctly signed zero.
                            sum67      = 67'b0;
                            result_exp = 15'h0;
                        end else begin
                            sum67      = sum67 << lz;
                            result_exp = result_exp - {8'b0, lz};
                        end
                    end
                end

                round_mantissa(sum67[66:3], sum67[2], sum67[1], sum67[0],
                                result_sign, rmode, rounded_mant, carry);

                if (carry) begin
                    rounded_mant = {1'b1, rounded_mant[63:1]};
                    result_exp   = result_exp + 15'd1;
                end

                if (result_exp >= 15'h7FFF) begin
                    // exponent overflow -- saturate to infinity (coarse
                    // OVFL substitute, see module header)
                    result = {result_sign, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
                end else if (rounded_mant == 64'h0) begin
                    result = {result_sign, 15'h0, 16'h0, 64'h0};
                end else begin
                    result = {result_sign, result_exp, 16'h0, rounded_mant};
                end
            end
        end

        // Condition codes (Section 2.3.1/4.5.5.1): computed from the
        // REAL result, not the operand classification above.
        begin
            fpx_t r;
            r = unpack_fpx(result);
            flag_z   = is_zero_fpx(r);
            flag_n   = r.sign && !flag_z;
            flag_i   = is_inf_fpx(r);
            flag_nan = is_nan_fpx(r);
        end
    endtask

    // FMUL (extension-field opcode $23, Table 4-13) -- Phase 4b.
    //
    // Mantissa multiply: a.mant/b.mant are each a 64-bit unsigned integer
    // representing a fixed-point value in [2^63, 2^64), i.e. j.f with the
    // binary point after bit 63 (bit63 is always the explicit integer
    // bit for a normalized operand). Their 128-bit product therefore
    // represents a value in [2^126, 2^128) -- equivalently, the true
    // product_value = product128 / 2^126 lands in [1,4). If bit127 is
    // set the product is in [2,4) and needs one more right-shift
    // (exponent+1) to renormalize back to the same [1,2)-equivalent
    // 64-bit window convention every other result in this file uses;
    // otherwise bit126 already IS the new integer bit and no extra shift
    // is needed.
    task automatic fp_mul(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr
    );
        fpx_t a, b;
        logic sign_r;
        logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;

        a = unpack_fpx(a_raw);
        b = unpack_fpx(b_raw);
        sign_r = a.sign ^ b.sign;

        a_nan  = is_nan_fpx(a);
        b_nan  = is_nan_fpx(b);
        a_inf  = is_inf_fpx(a);
        b_inf  = is_inf_fpx(b);
        a_zero = is_zero_fpx(a);
        b_zero = is_zero_fpx(b);

        flag_operr = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (b_nan) begin
            result = b_raw;
        end else if ((a_inf && b_zero) || (a_zero && b_inf)) begin
            flag_operr = 1'b1; // 0 * Infinity is undefined
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_inf || b_inf) begin
            result = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
        end else if (a_zero || b_zero) begin
            result = {sign_r, 15'h0, 16'h0, 64'h0};
        end else begin
            logic [127:0] product128;
            logic signed [17:0] exp_sum;
            logic [63:0] mant64;
            logic        guard, round_bit, sticky;
            logic [63:0] rounded_mant;
            logic        carry;

            product128 = a.mant * b.mant;

            if (product128[127]) begin
                mant64    = product128[127:64];
                guard     = product128[63];
                round_bit = product128[62];
                sticky    = |product128[61:0];
                exp_sum   = $signed({3'b0, a.exp}) + $signed({3'b0, b.exp}) - 18'sd16383 + 18'sd1;
            end else begin
                mant64    = product128[126:63];
                guard     = product128[62];
                round_bit = product128[61];
                sticky    = |product128[60:0];
                exp_sum   = $signed({3'b0, a.exp}) + $signed({3'b0, b.exp}) - 18'sd16383;
            end

            round_mantissa(mant64, guard, round_bit, sticky, sign_r, rmode, rounded_mant, carry);
            if (carry) begin
                rounded_mant = {1'b1, rounded_mant[63:1]};
                exp_sum      = exp_sum + 18'sd1;
            end

            if (exp_sum >= 18'sd32767) begin
                // exponent overflow -- saturate to infinity (Phase 4a's
                // own coarse OVFL substitute, same convention here)
                result = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end else if (exp_sum <= 18'sd0 || rounded_mant == 64'h0) begin
                // exponent underflow -- Phase 4a's own denormal-free
                // simplification: collapse to a correctly signed zero
                result = {sign_r, 15'h0, 16'h0, 64'h0};
            end else begin
                result = {sign_r, exp_sum[14:0], 16'h0, rounded_mant};
            end
        end

        begin
            fpx_t r;
            r = unpack_fpx(result);
            flag_z   = is_zero_fpx(r);
            flag_n   = r.sign && !flag_z;
            flag_i   = is_inf_fpx(r);
            flag_nan = is_nan_fpx(r);
        end
    endtask

endpackage
