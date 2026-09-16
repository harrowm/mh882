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

    // Signaling vs non-signaling (quiet) NaN (Section 3.5.4, confirmed
    // directly): "NANs with a leading fraction bit equal to one are
    // non-signaling NANs; NANs with a leading fraction bit equal to
    // zero are signaling NANs" -- the leading fraction bit is bit62 (the
    // bit immediately after the explicit integer bit at bit63).
    function automatic logic is_snan_fpx(fpx_t x);
        return is_nan_fpx(x) && !x.mant[62];
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
        output logic         flag_operr,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
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
        flag_ovfl  = 1'b0;
        flag_unfl  = 1'b0;
        flag_inex2 = 1'b0;

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
                            flag_unfl  = 1'b1;
                            sum67      = 67'b0;
                            result_exp = 15'h0;
                        end else begin
                            sum67      = sum67 << lz;
                            result_exp = result_exp - {8'b0, lz};
                        end
                    end
                end

                flag_inex2 = (sum67[2:0] != 3'b0); // guard/round/sticky nonzero -> inexact

                round_mantissa(sum67[66:3], sum67[2], sum67[1], sum67[0],
                                result_sign, rmode, rounded_mant, carry);

                if (carry) begin
                    rounded_mant = {1'b1, rounded_mant[63:1]};
                    result_exp   = result_exp + 15'd1;
                end

                if (result_exp >= 15'h7FFF) begin
                    // exponent overflow -- saturate to infinity (coarse
                    // OVFL substitute, see module header)
                    flag_ovfl = 1'b1;
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
        output logic         flag_operr,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
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
        flag_ovfl  = 1'b0;
        flag_unfl  = 1'b0;
        flag_inex2 = 1'b0;

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

            flag_inex2 = guard || round_bit || sticky;

            round_mantissa(mant64, guard, round_bit, sticky, sign_r, rmode, rounded_mant, carry);
            if (carry) begin
                rounded_mant = {1'b1, rounded_mant[63:1]};
                exp_sum      = exp_sum + 18'sd1;
            end

            if (exp_sum >= 18'sd32767) begin
                // exponent overflow -- saturate to infinity (Phase 4a's
                // own coarse OVFL substitute, same convention here)
                flag_ovfl = 1'b1;
                result = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end else if (exp_sum <= 18'sd0 || rounded_mant == 64'h0) begin
                // exponent underflow -- Phase 4a's own denormal-free
                // simplification: collapse to a correctly signed zero
                flag_unfl = (exp_sum <= 18'sd0);
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

    // FDIV (extension-field opcode $20, Table 4-13) -- Phase 4b.
    // result = FPn / source (b/a in this task's own a/b naming, matching
    // FADD/FSUB's own "Source + FPn" / "FPn - Source" convention).
    //
    // Mantissa divide: a.mant/b.mant are each a 64-bit unsigned integer
    // representing a value in [1,2) (fixed point, binary point after
    // bit63). Their true ratio b.mant/a.mant therefore lands in (0.5,2).
    // Scale the dividend left by DIV_SHIFT=67 bits (3 more than the
    // 64-bit mantissa window needs, for a guard/round/sticky tail
    // matching fp_add_sub/fp_mul's own convention) before dividing by
    // a.mant: quotient = floor((b.mant << 67) / a.mant) lands in
    // [2^66, 2^68) -- i.e. either bit67 or bit66 is the new integer bit,
    // depending on whether the ratio is >=1 or <1. A single conditional
    // 1-bit left-shift (folding the displaced bit into the sticky tail)
    // normalizes both cases to "integer bit at position 67" uniformly,
    // the same shape fp_add_sub/fp_mul's own normalization step uses.
    localparam int DIV_SHIFT = 67;

    task automatic fp_div(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr,
        output logic         flag_dz,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
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
        flag_dz    = 1'b0;
        flag_ovfl  = 1'b0;
        flag_unfl  = 1'b0;
        flag_inex2 = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (b_nan) begin
            result = b_raw;
        end else if ((a_inf && b_inf) || (a_zero && b_zero)) begin
            flag_operr = 1'b1; // Infinity/Infinity or 0/0 is undefined
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_zero) begin
            flag_dz = 1'b1; // divide by zero (dividend nonzero, confirmed above)
            result  = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
        end else if (a_inf) begin
            // finite / Infinity = signed zero
            result = {sign_r, 15'h0, 16'h0, 64'h0};
        end else if (b_inf) begin
            result = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
        end else if (b_zero) begin
            result = {sign_r, 15'h0, 16'h0, 64'h0};
        end else begin
            logic [199:0] dividend, divisor, quotient, remainder;
            logic [67:0]  q68, q_aligned;
            logic signed [17:0] exp_result;
            logic [63:0] mant64;
            logic        guard, round_bit, sticky;
            logic [63:0] rounded_mant;
            logic        carry;

            dividend = {136'b0, b.mant} << DIV_SHIFT;
            divisor  = {136'b0, a.mant};
            quotient = dividend / divisor;
            remainder = dividend % divisor;
            q68 = quotient[67:0];

            if (q68[67]) begin
                // ratio >= 1: already aligned, top bit at 67
                q_aligned  = q68;
                exp_result = $signed({3'b0, b.exp}) - $signed({3'b0, a.exp}) + 18'sd16383;
            end else begin
                // ratio < 1: shift left 1 to bring q68's own top bit
                // (bit66) up to bit67 -- a pure left-shift, so q68's own
                // bit0 lands at q_aligned[1], nothing is discarded
                q_aligned  = {q68[66:0], 1'b0};
                exp_result = $signed({3'b0, b.exp}) - $signed({3'b0, a.exp}) + 18'sd16383 - 18'sd1;
            end

            mant64    = q_aligned[67:4];
            guard     = q_aligned[3];
            round_bit = q_aligned[2];
            sticky    = q_aligned[1] | q_aligned[0] | (remainder != 200'b0);
            flag_inex2 = guard || round_bit || sticky;

            round_mantissa(mant64, guard, round_bit, sticky, sign_r, rmode, rounded_mant, carry);
            if (carry) begin
                rounded_mant = {1'b1, rounded_mant[63:1]};
                exp_result   = exp_result + 18'sd1;
            end

            if (exp_result >= 18'sd32767) begin
                flag_ovfl = 1'b1;
                result = {sign_r, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end else if (exp_result <= 18'sd0 || rounded_mant == 64'h0) begin
                flag_unfl = (exp_result <= 18'sd0);
                result = {sign_r, 15'h0, 16'h0, 64'h0};
            end else begin
                result = {sign_r, exp_result[14:0], 16'h0, rounded_mant};
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

    // Integer square root via the standard binary digit-recurrence
    // ("shift and subtract") algorithm: processes 2 bits of the input
    // per step, incrementally maintaining a remainder so no per-step
    // squaring is needed. root_out satisfies root_out^2 <= n <
    // (root_out+1)^2; rem_out is the exact remainder n - root_out^2
    // (used for the sqrt's own sticky bit below). Verified by hand
    // against sqrt(25)=5 before use here (see plan.md's own Phase 4b
    // writeup for the worked trace).
    localparam int SQRT_K = 67;

    task automatic isqrt134(
        input  logic [2*SQRT_K-1:0] n,
        output logic [SQRT_K-1:0]   root_out,
        output logic [2*SQRT_K-1:0] rem_out
    );
        logic [2*SQRT_K-1:0] rem;
        logic [SQRT_K-1:0]   root;
        logic [2*SQRT_K-1:0] candidate, test_val;
        int i;

        rem  = '0;
        root = '0;
        for (i = SQRT_K - 1; i >= 0; i--) begin
            candidate = {rem[2*SQRT_K-3:0], n[2*i+1], n[2*i]};
            test_val  = {{(2*SQRT_K-SQRT_K-2){1'b0}}, root, 2'b01}; // 4*root+1
            if (candidate >= test_val) begin
                rem  = candidate - test_val;
                root = {root[SQRT_K-2:0], 1'b1};
            end else begin
                rem  = candidate;
                root = {root[SQRT_K-2:0], 1'b0};
            end
        end
        root_out = root;
        rem_out  = rem;
    endtask

    // FSQRT (extension-field opcode $04, Table 4-13) -- Phase 4b.
    //
    // sqrt(1.f * 2^e): if e (the UNBIASED exponent) is even, equals
    // sqrt(1.f)*2^(e/2) directly. If e is odd, rewritten as
    // sqrt(2*1.f)*2^((e-1)/2) instead (doubling the mantissa into [2,4)
    // so the exponent adjustment stays an exact integer division) --
    // the same "make the exponent arithmetic exact by adjusting the
    // mantissa instead" trick fp_mul/fp_div's own normalization already
    // relies on. `real_exp >>> 1` (arithmetic shift) gives (e-1)/2 for
    // odd e and e/2 for even e uniformly, so no separate branch is
    // needed for the exponent's own halving arithmetic -- only the
    // final mantissa-window bit position needs the usual conditional
    // check (same shape as fp_mul/fp_div's own carry/renormalization
    // handling).
    task automatic fp_sqrt(
        input  logic [95:0]  a_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
    );
        fpx_t a;
        logic a_nan, a_inf, a_zero;

        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_inf  = is_inf_fpx(a);
        a_zero = is_zero_fpx(a);

        flag_operr = 1'b0;
        flag_ovfl  = 1'b0;
        flag_unfl  = 1'b0;
        flag_inex2 = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (a.sign && !a_zero) begin
            // sqrt of a negative, nonzero number is undefined
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_zero) begin
            result = a_raw; // sqrt(+0)=+0, sqrt(-0)=-0
        end else if (a_inf) begin
            result = a_raw; // sqrt(+Infinity) = +Infinity
        end else begin
            // BUG FOUND VIA DIRECT NUMERIC TESTING (Python cross-check,
            // not just inspection -- see plan.md's own Phase 4b writeup):
            // the first version of this code doubled the mantissa for
            // ODD real_exp and used a single fixed shift amount for both
            // parities. That's backwards. The real relationship: a.mant
            // always represents m*2^63 (bit63 fixed at 1), so
            // a.mant<<S always has its OWN top bit at a fixed, exponent-
            // INDEPENDENT position (63+S) -- the choice that actually
            // matters is S's own PARITY relative to real_exp's parity,
            // not any manual doubling step. Empirically and algebraically
            // confirmed: S=69 (odd) when real_exp is EVEN lands the
            // integer-sqrt root's own top bit at position 66 every time
            // (never needs a runtime bucket check); S=68 (even) when
            // real_exp is ODD lands it at position 65 every time -- and
            // BOTH cases use the identical `half_exp + 16383` exponent
            // formula (no separate +/-1 adjustment between the two
            // branches at all, unlike fp_mul/fp_div's own genuinely
            // data-dependent bucket crossover).
            logic signed [17:0] real_exp, half_exp;
            logic         is_even;
            logic [133:0] sqrt_n;
            logic [66:0]  root;
            logic [133:0] rem;
            logic [63:0]  mant64;
            logic         guard, round_bit, sticky;
            logic [63:0]  rounded_mant;
            logic         carry;
            logic signed [17:0] exp_result;

            real_exp = $signed({3'b0, a.exp}) - 18'sd16383;
            is_even  = !real_exp[0];
            half_exp = real_exp >>> 1;

            sqrt_n = is_even ? ({70'b0, a.mant} << 69) : ({70'b0, a.mant} << 68);

            isqrt134(sqrt_n, root, rem);

            if (root[66]) begin
                mant64     = root[66:3];
                guard      = root[2];
                round_bit  = root[1];
                sticky     = root[0] | (rem != 134'b0);
            end else begin
                mant64     = root[65:2];
                guard      = root[1];
                round_bit  = root[0];
                sticky     = (rem != 134'b0);
            end
            exp_result = half_exp + 18'sd16383;
            flag_inex2 = guard || round_bit || sticky;

            round_mantissa(mant64, guard, round_bit, sticky, 1'b0, rmode, rounded_mant, carry);
            if (carry) begin
                rounded_mant = {1'b1, rounded_mant[63:1]};
                exp_result   = exp_result + 18'sd1;
            end

            if (exp_result >= 18'sd32767) begin
                flag_ovfl = 1'b1;
                result = {1'b0, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end else if (exp_result <= 18'sd0 || rounded_mant == 64'h0) begin
                flag_unfl = (exp_result <= 18'sd0);
                result = {1'b0, 15'h0, 16'h0, 64'h0};
            end else begin
                result = {1'b0, exp_result[14:0], 16'h0, rounded_mant};
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

    // ────────────────────────────────────────────────────────────────
    // Phase 4c: external-operand <-> extended-precision conversion.
    //
    // Scope: Long-Word-Integer (L), Single-Precision-Real (S), Double-
    // Precision-Real (D), and Extended-Precision-Real (X, a pure
    // passthrough -- it IS the internal format, Table 3-3) are
    // implemented both directions. Word/Byte Integer (W/B) and Packed
    // Decimal (P) are NOT yet implemented -- documented in plan.md as
    // Phase 4c's own remaining scope, not silently skipped.
    //
    // Every EXTERNAL-TO-EXTENDED conversion below is EXACT (lossless):
    // extended precision has more significant bits (64, an explicit
    // integer bit plus 63 fraction bits) than any of L (32-bit integer),
    // S (24-bit significand), or D (53-bit significand), so no rounding
    // is ever needed converting INTO the internal format. Only the
    // reverse direction (EXTENDED-TO-EXTERNAL, used by opclass 011,
    // FPn-to-memory) can lose precision and needs real rounding --
    // reuses the same guard/round/sticky shape as fp_add_sub/fp_mul/
    // fp_div/fp_sqrt, just against a narrower target mantissa width.
    // ────────────────────────────────────────────────────────────────

    // ── Long-Word Integer (L) <-> extended ──────────────────────────
    task automatic int32_to_ext(input logic signed [31:0] val, output logic [95:0] result);
        if (val == 32'sd0) begin
            result = 96'h0;
        end else begin
            logic signed [32:0] wide_val, wide_mag;
            logic [63:0] mag64;
            logic [6:0]  lz;
            logic [6:0]  shift;
            logic [63:0] mant;
            logic [14:0] exp;

            wide_val = {val[31], val}; // sign-extend to 33 bits, avoids
                                        // the INT_MIN (-2^31) two's-
                                        // complement-negation-overflow trap
            wide_mag = val[31] ? -wide_val : wide_val;
            mag64    = {31'b0, wide_mag[32:0]};
            lz       = lzc64(mag64) - 7'd31; // 0..32: leading zeros within
                                              // the real 33-bit magnitude
            shift    = 7'd31 + lz;           // aligns the magnitude's own
                                              // leading 1 bit to bit63
            mant     = mag64 << shift;
            exp      = 15'd16383 + (15'd32 - {8'b0, lz});

            result = {val[31], exp, 16'h0, mant};
        end
    endtask

    task automatic ext_to_int32(
        input  logic [95:0]  ext,
        input  round_mode_t  rmode,
        output logic [31:0]  val,
        output logic         flag_operr
    );
        fpx_t x;
        logic signed [17:0] real_exp;

        x = unpack_fpx(ext);
        flag_operr = 1'b0;

        if (is_nan_fpx(x) || is_inf_fpx(x)) begin
            flag_operr = 1'b1;
            val = x.sign ? 32'h8000_0000 : 32'h7FFF_FFFF;
        end else if (is_zero_fpx(x)) begin
            val = 32'h0;
        end else begin
            real_exp = $signed({3'b0, x.exp}) - 18'sd16383;
            if (real_exp < 18'sd0) begin
                // magnitude < 1.0: rounds to 0 for RZ, +-1 possible for
                // other modes depending on how close to +-1 -- a real,
                // deliberately narrow simplification: round toward zero
                // unconditionally here (documented gap, matches this
                // phase's own "exact conversions only, narrowing needs
                // more care" scope boundary rather than a silent wrong
                // answer for the more exotic rounding-mode cases)
                val = 32'h0;
            end else if (real_exp > 18'sd30) begin
                // magnitude >= 2^31: out of Long-Word-Integer range
                flag_operr = 1'b1;
                val = x.sign ? 32'h8000_0000 : 32'h7FFF_FFFF;
            end else begin
                // shift the 64-bit mantissa right so its own integer bit
                // (bit63) lands at bit(real_exp) of a 32-bit result --
                // exact whenever real_exp>=63-63=0 covers all fraction
                // bits already being shifted out below bit0, i.e. this
                // is a real truncation needing real rounding
                // Icarus doesn't allow a variable-index bit-select
                // (x.mant[drop-1]) here -- `drop` is data-dependent, not
                // a constant -- so guard/round-bit/sticky are all
                // extracted via dynamic SHIFTS and a MASK instead
                // (`x.mant >> k` and `(1<<k)-1` are both fine with a
                // variable k), never a variable part-select.
                logic [63:0] shifted;
                logic [63:0] dropped_mask;
                logic [63:0] dropped_bits;
                logic        guard, any_lower;
                logic [31:0] mag;
                logic        round_up;
                int unsigned drop;
                drop = 63 - real_exp;
                shifted      = x.mant >> drop;
                dropped_mask = (drop == 0) ? 64'h0 : ((64'h1 << drop) - 64'h1);
                dropped_bits = x.mant & dropped_mask;
                guard        = (drop > 0) && ((dropped_bits >> (drop - 1)) & 64'h1);
                any_lower    = (drop > 1) && ((dropped_bits & ((64'h1 << (drop - 1)) - 64'h1)) != 0);
                round_up     = (guard && ((rmode == RND_NEAREST) && (any_lower || shifted[0]))) ||
                               ((rmode == RND_PINF) && !x.sign && (dropped_bits != 0)) ||
                               ((rmode == RND_MINF) && x.sign && (dropped_bits != 0));
                mag = shifted[31:0] + (round_up ? 32'd1 : 32'd0);
                val = x.sign ? (~mag + 32'd1) : mag;
            end
        end
    endtask

    // ── Single Precision Real (S, IEEE-754 binary32) <-> extended ────
    task automatic single_to_ext(input logic [31:0] bits, output logic [95:0] result);
        logic        sign;
        logic [7:0]  exp8;
        logic [22:0] frac23;
        sign   = bits[31];
        exp8   = bits[30:23];
        frac23 = bits[22:0];

        if (exp8 == 8'h00 && frac23 == 23'h0) begin
            result = {sign, 95'h0}; // signed zero
        end else if (exp8 == 8'hFF) begin
            result = {sign, 15'h7FFF, 16'h0,
                      (frac23 == 23'h0) ? 64'h8000_0000_0000_0000
                                        : {1'b1, 1'b1, frac23, 39'b0}}; // Inf / NaN
        end else if (exp8 == 8'h00) begin
            // denormal single -- not specially normalized here (Phase
            // 4a's own established denormal-free convention); treated
            // as an (inexact but nonzero) very-small normalized value
            // is out of this phase's own scope -- flush to zero
            result = {sign, 95'h0};
        end else begin
            logic [14:0] exp_ext;
            exp_ext = {7'b0, exp8} - 15'd127 + 15'd16383;
            result  = {sign, exp_ext, 16'h0, 1'b1, frac23, 40'b0};
        end
    endtask

    task automatic ext_to_single(
        input  logic [95:0]  ext,
        input  round_mode_t  rmode,
        output logic [31:0]  bits,
        output logic         flag_operr
    );
        fpx_t x;
        logic signed [17:0] real_exp;

        x = unpack_fpx(ext);
        flag_operr = 1'b0;

        if (is_nan_fpx(x)) begin
            bits = {x.sign, 8'hFF, 1'b1, x.mant[61:39]};
        end else if (is_inf_fpx(x)) begin
            bits = {x.sign, 8'hFF, 23'h0};
        end else if (is_zero_fpx(x)) begin
            bits = {x.sign, 31'h0};
        end else begin
            real_exp = $signed({3'b0, x.exp}) - 18'sd16383;
            if (real_exp > 18'sd127) begin
                flag_operr = 1'b1;
                bits = {x.sign, 8'hFF, 23'h0}; // overflow -> Infinity
            end else if (real_exp < -18'sd126) begin
                bits = {x.sign, 31'h0}; // underflow -> zero (denormal-free)
            end else begin
                logic [39:0] dropped;
                logic [22:0] frac23;
                logic        round_up, carry;
                dropped  = x.mant[39:0];
                frac23   = x.mant[62:40];
                round_up = dropped[39] && ((rmode == RND_NEAREST) &&
                             (dropped[38:0] != 0 || frac23[0])) ||
                           ((rmode == RND_PINF) && !x.sign && (dropped != 0)) ||
                           ((rmode == RND_MINF) && x.sign && (dropped != 0));
                {carry, frac23} = {1'b0, frac23} + (round_up ? 24'd1 : 24'd0);
                bits = {x.sign, 8'(real_exp + 18'sd127) + (carry ? 8'd1 : 8'd0), frac23};
            end
        end
    endtask

    // ── Double Precision Real (D, IEEE-754 binary64) <-> extended ────
    task automatic double_to_ext(input logic [63:0] bits, output logic [95:0] result);
        logic         sign;
        logic [10:0]  exp11;
        logic [51:0]  frac52;
        sign   = bits[63];
        exp11  = bits[62:52];
        frac52 = bits[51:0];

        if (exp11 == 11'h0 && frac52 == 52'h0) begin
            result = {sign, 95'h0};
        end else if (exp11 == 11'h7FF) begin
            result = {sign, 15'h7FFF, 16'h0,
                      (frac52 == 52'h0) ? 64'h8000_0000_0000_0000
                                        : {1'b1, 1'b1, frac52, 10'b0}};
        end else if (exp11 == 11'h0) begin
            result = {sign, 95'h0}; // denormal double -- flush to zero (same convention as single)
        end else begin
            logic [14:0] exp_ext;
            exp_ext = {4'b0, exp11} - 15'd1023 + 15'd16383;
            result  = {sign, exp_ext, 16'h0, 1'b1, frac52, 11'b0};
        end
    endtask

    task automatic ext_to_double(
        input  logic [95:0]  ext,
        input  round_mode_t  rmode,
        output logic [63:0]  bits,
        output logic         flag_operr
    );
        fpx_t x;
        logic signed [17:0] real_exp;

        x = unpack_fpx(ext);
        flag_operr = 1'b0;

        if (is_nan_fpx(x)) begin
            bits = {x.sign, 11'h7FF, 1'b1, x.mant[60:10]};
        end else if (is_inf_fpx(x)) begin
            bits = {x.sign, 11'h7FF, 52'h0};
        end else if (is_zero_fpx(x)) begin
            bits = {x.sign, 63'h0};
        end else begin
            real_exp = $signed({3'b0, x.exp}) - 18'sd16383;
            if (real_exp > 18'sd1023) begin
                flag_operr = 1'b1;
                bits = {x.sign, 11'h7FF, 52'h0};
            end else if (real_exp < -18'sd1022) begin
                bits = {x.sign, 63'h0};
            end else begin
                logic [10:0] dropped;
                logic [51:0] frac52;
                logic        round_up, carry;
                dropped  = x.mant[10:0];
                frac52   = x.mant[62:11];
                round_up = dropped[10] && ((rmode == RND_NEAREST) &&
                             (dropped[9:0] != 0 || frac52[0])) ||
                           ((rmode == RND_PINF) && !x.sign && (dropped != 0)) ||
                           ((rmode == RND_MINF) && x.sign && (dropped != 0));
                {carry, frac52} = {1'b0, frac52} + (round_up ? 53'd1 : 53'd0);
                bits = {x.sign, 11'(real_exp + 18'sd1023) + (carry ? 11'd1 : 11'd0), frac52};
            end
        end
    endtask

    // ── Phase 9b: exact auxiliary ops (Table 4-13: $01 FINT, $03
    // FINTRZ, $1E FGETEXP, $1F FGETMAN, $26 FSCALE) -- NOT approximated,
    // unlike the real trig/log/exp transcendental set (Section 4.3's own
    // ~64-ULP-typical tolerance does not apply to any of these 5; they
    // are exact IEEE-shape bit manipulations, like FABS/FNEG). ─────────

    // Truncate (round-toward-zero) an extended-precision value to a
    // signed integer, saturating at +-32767 for any magnitude beyond
    // that -- FSCALE's own real exponent field is only 15 bits, so no
    // legitimate scale factor ever needs a wider range than this; the
    // saturated value just guarantees the OVFL/UNFL path fires
    // correctly regardless of the destination's own starting exponent.
    function automatic logic signed [15:0] trunc_to_int16(fpx_t a);
        logic signed [17:0] real_exp;
        logic [6:0]         frac_bits;
        logic [63:0]        int_part;
        logic signed [17:0] mag;

        if (is_zero_fpx(a)) return 16'sd0;

        real_exp = $signed({3'b0, a.exp}) - 18'sd16383;
        if (real_exp < 18'sd0) return 16'sd0; // |a| < 1.0 -- truncates to 0
        if (real_exp >= 18'sd15) return a.sign ? -16'sd32767 : 16'sd32767; // saturate

        frac_bits = 7'(63 - real_exp);
        int_part  = a.mant >> frac_bits; // low (real_exp+1) bits hold the truncated magnitude
        mag       = $signed({3'b0, int_part[14:0]});
        return a.sign ? -mag[15:0] : mag[15:0];
    endfunction

    // FINT ($01, uses the caller-supplied rounding mode -- FPCR's own
    // current setting) / FINTRZ ($03, caller always passes RND_ZERO
    // regardless of FPCR -- the same task serves both, the only
    // difference is which rounding mode the caller hands in). Rounds
    // the source to the nearest/truncated integer VALUE, in the SAME
    // extended-precision format (not a true integer format -- Section
    // 4.x's own "round to floating-point integer" framing).
    task automatic fp_int(
        input  logic [95:0]  a_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_inex2
    );
        fpx_t a;
        logic signed [17:0] real_exp;

        a = unpack_fpx(a_raw);
        flag_inex2 = 1'b0;

        if (is_nan_fpx(a) || is_inf_fpx(a) || is_zero_fpx(a)) begin
            result = a_raw; // already "integral" -- propagate unchanged
        end else begin
            real_exp = $signed({3'b0, a.exp}) - 18'sd16383;

            if (real_exp >= 18'sd63) begin
                result = a_raw; // no fractional bits present at all
            end else if (real_exp < 18'sd0) begin
                // |value| < 1.0 -- rounds to a signed zero or +-1.0.
                // Always inexact (a genuine nonzero value can never
                // equal its own {0,+-1.0} rounded result).
                logic round_up;
                unique case (rmode)
                    RND_ZERO: round_up = 1'b0;
                    RND_MINF: round_up = a.sign;
                    RND_PINF: round_up = !a.sign;
                    default:  // RND_NEAREST -- exact 0.5 ties to even (0)
                        round_up = (real_exp == -18'sd1) && (a.mant[62:0] != 63'h0);
                endcase
                result = round_up ? {a.sign, 15'd16383, 16'h0, 64'h8000_0000_0000_0000}
                                   : {a.sign, 15'h0, 16'h0, 64'h0};
                flag_inex2 = 1'b1;
            end else begin
                // 0 <= real_exp < 63: split at the real integer/fraction
                // boundary. int_part occupies bits [real_exp:0] of a
                // 64-bit register (all higher bits zero); frac_bits =
                // 63-real_exp is the fractional-bit count being rounded
                // away. Uses dynamic shift+mask throughout, NOT a
                // variable-width part-select (`x.mant[frac_bits-3:0]`)
                // -- the same established Icarus workaround this
                // project has needed before (ext_to_int32, Phase 4c).
                logic [6:0]  frac_bits;
                logic [63:0] int_part, rounded_int, new_mant, sticky_mask;
                logic        guard, round_bit, sticky, carry, overflow;
                logic [17:0] new_real_exp;

                frac_bits   = 7'(63 - real_exp);
                int_part    = a.mant >> frac_bits;
                guard       = (frac_bits >= 7'd1) ? ((a.mant >> (frac_bits - 7'd1)) & 64'h1) : 1'b0;
                round_bit   = (frac_bits >= 7'd2) ? ((a.mant >> (frac_bits - 7'd2)) & 64'h1) : 1'b0;
                sticky_mask = (frac_bits >= 7'd3) ? ((64'h1 << (frac_bits - 7'd2)) - 64'h1) : 64'h0;
                sticky      = |(a.mant & sticky_mask);

                round_mantissa(int_part, guard, round_bit, sticky, a.sign, rmode, rounded_int, carry);
                flag_inex2 = guard | round_bit | sticky;

                // round_mantissa's OWN carry_out only fires if all 64
                // bits of int_part overflow, which int_part's own
                // guaranteed-zero top bit (frac_bits>=1 here) makes
                // unreachable in practice -- the REAL "grew into one
                // more integer bit" case (e.g. 1.111...->10.000...) is
                // instead visible directly in rounded_int's own value,
                // checked here via `overflow` (again dynamic shift+mask,
                // not a variable-index bit-select).
                overflow = |((rounded_int >> (real_exp + 18'sd1)) & 64'h1);
                if (overflow) begin
                    new_real_exp = real_exp + 18'sd1;
                    new_mant     = rounded_int << (frac_bits - 7'd1);
                end else begin
                    new_real_exp = real_exp;
                    new_mant     = rounded_int << frac_bits;
                end
                result = {a.sign, new_real_exp[14:0] + 15'd16383, 16'h0, new_mant};
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

    // FGETEXP ($1E): returns the source's own real (unbiased) exponent,
    // AS A FLOATING-POINT VALUE (e.g. source=8.0 -> result=3.0). Exact,
    // no rounding -- the real exponent always fits comfortably in
    // int32_to_ext's own 32-bit input range, reused directly rather
    // than re-deriving the same int-to-extended conversion a second
    // time.
    task automatic fp_getexp(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr
    );
        fpx_t a;
        logic signed [17:0] real_exp;

        a = unpack_fpx(a_raw);
        flag_operr = 1'b0;

        if (is_nan_fpx(a)) begin
            result = a_raw;
        end else if (is_inf_fpx(a)) begin
            // Undefined operation on an infinite operand -- OPERR, same
            // default-NaN convention as fp_sqrt's own negative-operand
            // case (Section 4.3's own accuracy scope note applies to
            // this project's own choice of default-NaN payload, not
            // confirmed against the manual's real bit pattern -- see
            // plan.md's own Phase 7 FSQRT-vs-Musashi divergence writeup).
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (is_zero_fpx(a)) begin
            result = {a.sign, 15'h0, 16'h0, 64'h0}; // sign-preserving zero
        end else begin
            real_exp = $signed({3'b0, a.exp}) - 18'sd16383;
            int32_to_ext({{14{real_exp[17]}}, real_exp}, result);
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

    // FGETMAN ($1F): returns the source's own normalized mantissa, in
    // [1,2) -- exact, no rounding at all. This project's own internal
    // format ALREADY stores the mantissa exactly this way (explicit
    // integer bit at bit63, Table 3-3), so the whole operation is just
    // "keep the mantissa, force the exponent field to the bias" (2^0).
    task automatic fp_getman(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr
    );
        fpx_t a;
        a = unpack_fpx(a_raw);
        flag_operr = 1'b0;

        if (is_nan_fpx(a)) begin
            result = a_raw;
        end else if (is_inf_fpx(a)) begin
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (is_zero_fpx(a)) begin
            result = a_raw; // signed zero preserved
        end else begin
            result = {a.sign, 15'd16383, 16'h0, a.mant};
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

    // FSCALE ($26): FPn = FPn * 2^trunc(FPm) -- a genuinely DYADIC
    // auxiliary op (unlike FINT/FGETEXP/FGETMAN, all monadic). `a` is
    // the source (RX, the scale factor); `b` is the destination
    // accumulator (RY, the value being scaled AND overwritten) --
    // matching fp_add_sub's own "RX=source, RY=dest" convention exactly.
    // Exact, no rounding -- purely an exponent shift, mantissa
    // untouched. Edge cases the manual doesn't fully specify (an
    // infinite or NaN scale factor) are this project's own reasonable,
    // documented choice: NaN propagates (a before b, matching this
    // project's own established tie-break elsewhere); an infinite scale
    // factor is handled naturally by trunc_to_int16's own saturation
    // (which already guarantees OVFL/UNFL fires regardless of b's own
    // starting exponent).
    task automatic fp_scale(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_ovfl,
        output logic         flag_unfl
    );
        fpx_t a, b;
        logic signed [17:0] new_exp_biased;
        logic signed [15:0] k;

        a = unpack_fpx(a_raw);
        b = unpack_fpx(b_raw);
        flag_ovfl = 1'b0;
        flag_unfl = 1'b0;

        if (is_nan_fpx(a)) begin
            result = a_raw;
        end else if (is_nan_fpx(b)) begin
            result = b_raw;
        end else if (is_zero_fpx(b) || is_inf_fpx(b)) begin
            result = b_raw; // scaling zero or infinity by any finite factor: unchanged
        end else begin
            k = trunc_to_int16(a);
            new_exp_biased = $signed({3'b0, b.exp}) + {{2{k[15]}}, k};

            if (new_exp_biased >= 18'sd32767) begin
                flag_ovfl = 1'b1;
                result = {b.sign, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
            end else if (new_exp_biased <= 18'sd0) begin
                flag_unfl = 1'b1;
                result = {b.sign, 15'h0, 16'h0, 64'h0};
            end else begin
                result = {b.sign, new_exp_biased[14:0], 16'h0, b.mant};
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

    // ── Phase 9c: FSGLDIV ($24) / FSGLMUL ($27) -- Section 4.x,
    // confirmed directly: "divides... Stores the result... rounded to
    // single precision (regardless of the current rounding precision)."
    // Both are EXACT relative to that specification (not subject to the
    // ~64-ULP transcendental tolerance) -- genuine double-rounding
    // (extended-precision divide/multiply, THEN round to single, THEN
    // re-extend), matching the manual's own "converts... divides...
    // stores... rounded to single" ordering directly, and matching
    // Musashi's own implementation shape too (confirmed by inspection of
    // m68kfpu.c's own FSGLDIV/FSGLMUL cases: `double_to_fx80((float)
    // fx80_to_double(floatx80_div(...)))` -- the identical double-
    // rounding structure, though Musashi's own C-cast-based rounding
    // always uses round-to-nearest regardless of FPCR, unlike this
    // project's own rmode-respecting ext_to_single reuse below -- Musashi
    // is therefore only a valid cross-check for round=0/nearest on these
    // two ops specifically). Reuses ext_to_single/single_to_ext directly
    // rather than re-deriving single-precision rounding a second time.
    task automatic fp_sgldiv(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr,
        output logic         flag_dz,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
    );
        logic [95:0] div_result;
        logic        div_z, div_n, div_i, div_nan, div_operr, div_dz, div_ovfl, div_unfl, div_inex2;
        logic [31:0] single_bits;
        logic        single_operr;

        fp_div(a_raw, b_raw, rmode, div_result, div_z, div_n, div_i, div_nan,
               div_operr, div_dz, div_ovfl, div_unfl, div_inex2);

        flag_operr = div_operr;
        flag_dz    = div_dz;

        if (div_nan || div_i || div_z) begin
            // Already exact in any format -- no further rounding needed
            // or meaningful.
            result     = div_result;
            flag_ovfl  = div_ovfl;
            flag_unfl  = div_unfl;
            flag_inex2 = div_inex2;
        end else begin
            ext_to_single(div_result, rmode, single_bits, single_operr);
            single_to_ext(single_bits, result);
            // single_operr here means "exceeded single precision's own
            // (much narrower) range" -- the real OVFL/UNFL this
            // instruction's own Exception Byte table points to (6.1.4/
            // 6.1.5), not a genuine OPERR (that's reserved for the
            // division's own 0/0, inf/inf domain errors, already
            // captured via div_operr above).
            flag_ovfl  = div_ovfl || (single_operr && !is_zero_fpx(unpack_fpx(result)));
            flag_unfl  = div_unfl || (single_operr && is_zero_fpx(unpack_fpx(result)));
            flag_inex2 = div_inex2 || (result != div_result);
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

    task automatic fp_sglmul(
        input  logic [95:0]  a_raw,
        input  logic [95:0]  b_raw,
        input  round_mode_t  rmode,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr,
        output logic         flag_ovfl,
        output logic         flag_unfl,
        output logic         flag_inex2
    );
        logic [95:0] mul_result;
        logic        mul_z, mul_n, mul_i, mul_nan, mul_operr, mul_ovfl, mul_unfl, mul_inex2;
        logic [31:0] single_bits;
        logic        single_operr;

        fp_mul(a_raw, b_raw, rmode, mul_result, mul_z, mul_n, mul_i, mul_nan,
               mul_operr, mul_ovfl, mul_unfl, mul_inex2);

        flag_operr = mul_operr;

        if (mul_nan || mul_i || mul_z) begin
            result     = mul_result;
            flag_ovfl  = mul_ovfl;
            flag_unfl  = mul_unfl;
            flag_inex2 = mul_inex2;
        end else begin
            ext_to_single(mul_result, rmode, single_bits, single_operr);
            single_to_ext(single_bits, result);
            flag_ovfl  = mul_ovfl || (single_operr && !is_zero_fpx(unpack_fpx(result)));
            flag_unfl  = mul_unfl || (single_operr && is_zero_fpx(unpack_fpx(result)));
            flag_inex2 = mul_inex2 || (result != mul_result);
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

    // ── Phase 9c: FMOD ($21) / FREM ($25) -- Section 4.x, confirmed
    // directly: result = FPn - (Source x N), where N = INT(FPn/Source)
    // -- truncated (round-to-zero) for FMOD, round-to-nearest for FREM
    // (the ONLY difference between the two instructions). Also stores
    // the sign and 7 LSBs of the (unsigned) quotient N in the FPSR
    // quotient byte (bits[23:16]: bit23=sign, bits[22:16]=7 LSBs --
    // Section 2.3.2/Figure 2-5, confirmed directly). Undefined (OPERR,
    // NaN result) for a zero source or an infinite destination.
    //
    // N is determined from fp_div's own extended-precision quotient
    // (64-bit mantissa precision, ample margin for correctly determining
    // a realistic integer N in all but deliberately-adversarial inputs
    // sitting exactly on a rounding boundary after 2 rounding steps -- a
    // documented, reasonable approximation, not exact for truly
    // arbitrary inputs) via the SAME dynamic-shift-and-mask boundary-
    // rounding technique fp_int uses (NOT a second call to fp_int
    // itself, deliberately -- see fp_int's own header comment on why a
    // second independent call site of the SAME task is a confirmed
    // Icarus livelock risk this project has already hit once).
    task automatic fp_mod_rem(
        input  logic [95:0]  a_raw,           // RX: source (modulus/divisor)
        input  logic [95:0]  b_raw,           // RY: dest (dividend), also destination
        input  logic         use_round_nearest, // 0=FMOD (truncate quotient), 1=FREM (round-nearest quotient)
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_operr,
        output logic [7:0]   quot_byte // bit7=sign, bits[6:0]=7 LSBs of |quotient|
    );
        fpx_t a, b, qx;
        logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
        logic [95:0] q_ext;
        logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
        logic        quot_sign;
        logic [63:0] n_mag;
        logic signed [17:0] real_exp;

        a = unpack_fpx(a_raw);
        b = unpack_fpx(b_raw);
        a_nan  = is_nan_fpx(a);  b_nan  = is_nan_fpx(b);
        a_inf  = is_inf_fpx(a);  b_inf  = is_inf_fpx(b);
        a_zero = is_zero_fpx(a); b_zero = is_zero_fpx(b);
        flag_operr = 1'b0;
        quot_byte  = 8'h0;

        if (a_nan) begin
            result = a_raw;
        end else if (b_nan) begin
            result = b_raw;
        end else if (a_zero || b_inf) begin
            // Undefined per the manual's own Operation Table -- OPERR, NaN.
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (b_zero) begin
            result = b_raw; // signed zero, dest's own sign preserved
        end else if (a_inf) begin
            result = b_raw; // Note 2: returns FPn's own pre-operation value
        end else begin
            fp_div(a_raw, b_raw, RND_NEAREST, q_ext, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2);
            quot_sign = qn; // sign of FPn/Source == XOR of the operand signs

            if (qz) begin
                result    = b_raw; // |FPn| << |Source| -- quotient rounds to 0
                n_mag     = 64'h0;
                quot_byte = {quot_sign, 7'h0};
            end else begin
                qx = unpack_fpx(q_ext);
                real_exp = $signed({3'b0, qx.exp}) - 18'sd16383;

                if (real_exp < 18'sd0) begin
                    // |q| < 1.0 -- N is 0, unless FREM rounds it up to
                    // exactly 1 (FMOD always truncates to 0 here).
                    logic round_to_1;
                    round_to_1 = use_round_nearest &&
                                 (real_exp == -18'sd1) && (qx.mant[62:0] != 63'h0);
                    n_mag = round_to_1 ? 64'd1 : 64'd0;
                end else if (real_exp >= 18'sd63) begin
                    // Magnitude already has no fractional part to round
                    // away -- saturate at this register's own width
                    // (this project's own documented limit; the
                    // manual's own 7-bit quotient-byte convention
                    // implies N is never expected to need more
                    // precision than this in realistic use).
                    n_mag = 64'hFFFF_FFFF_FFFF_FFFF;
                end else begin
                    logic [6:0]  frac_bits;
                    logic [63:0] int_part, sticky_mask;
                    logic        guard, round_bit, sticky, round_up, carry;

                    frac_bits   = 7'(63 - real_exp);
                    int_part    = qx.mant >> frac_bits;
                    guard       = (frac_bits >= 7'd1) ? ((qx.mant >> (frac_bits - 7'd1)) & 64'h1) : 1'b0;
                    round_bit   = (frac_bits >= 7'd2) ? ((qx.mant >> (frac_bits - 7'd2)) & 64'h1) : 1'b0;
                    sticky_mask = (frac_bits >= 7'd3) ? ((64'h1 << (frac_bits - 7'd2)) - 64'h1) : 64'h0;
                    sticky      = |(qx.mant & sticky_mask);

                    round_up = use_round_nearest && guard && (round_bit || sticky || int_part[0]);
                    {carry, int_part} = {1'b0, int_part} + (round_up ? 65'd1 : 65'd0);
                    n_mag = int_part;
                end

                quot_byte = {quot_sign, n_mag[6:0]};

                if (n_mag == 64'h0) begin
                    result = b_raw;
                end else begin
                    logic [6:0]  n_lz;
                    logic [95:0] n_ext_built, prod;
                    logic        pz, pn, pi, pnan, poperr, povfl, punfl, pinex2;
                    logic        dz2, dn2, di2, dnan2, doperr2, dovfl2, dunfl2, dinex2b;

                    // Build N as a signed extended-precision value
                    // directly (no int32 round-trip needed) so fp_mul/
                    // fp_add_sub -- already-proven, already-tested tasks
                    // -- can compute Source*N and FPn-that without any
                    // new arithmetic core logic.
                    n_lz = lzc64(n_mag);
                    n_ext_built = {quot_sign, 15'd16383 + (15'd63 - {8'b0, n_lz}), 16'h0, n_mag << n_lz};

                    fp_mul(a_raw, n_ext_built, RND_NEAREST, prod, pz, pn, pi, pnan, poperr, povfl, punfl, pinex2);
                    fp_add_sub(prod, b_raw, 1'b1, RND_NEAREST, result, dz2, dn2, di2, dnan2, doperr2, dovfl2, dunfl2, dinex2b);
                end
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

    // ── Phase 9d: FSIN ($0E) / FCOS ($1D) -- the first genuinely
    // APPROXIMATED transcendental this project implements (everything
    // through Phase 9c was exact or double-rounding-exact). Section 4.3
    // "Computational Accuracy" confirms real silicon does NOT compute
    // these bit-exact either: "the worst-case accuracy is 4096 units in
    // the last place... the typical error bound... is approximately 64
    // units in the last place" -- unlike FADD/FSUB/FMUL/FDIV/FSQRT
    // (IEEE-exact to 0.5 ULP since Phase 4a), this task targets that
    // same realistic tolerance, not bit-exact agreement with any
    // reference.
    //
    // Algorithm: classic argument reduction to [-pi/4,+pi/4] (Section
    // 4.x's own text confirms real hardware does exactly this: "If the
    // source operand is not in the range of [-2pi...+2pi], the argument
    // is reduced... large arguments may lose accuracy during
    // reduction, and very large arguments (greater than approximately
    // 10^20) lose all accuracy" -- this project's own reduction below
    // has the IDENTICAL large-argument degradation, for the identical
    // reason: real hardware doesn't solve full Payne-Hanek reduction
    // either, and the manual says so outright), then a 9-term Taylor
    // series (NOT a minimax polynomial -- a plain Taylor series is
    // trivially independently verifiable term-by-term, and the margin
    // it gives at this reduced range comfortably clears the target
    // tolerance, so the extra complexity of deriving and justifying a
    // minimax polynomial wasn't needed). Coefficients and PI_OVER_2
    // below were computed independently in Python to 50 significant
    // decimal digits and rounded to the nearest extended-precision bit
    // pattern -- see plan.md's own Phase 9d writeup for the exact
    // generating script and the accuracy-margin derivation (Taylor
    // truncation error at |r|<=pi/4 after 9 terms is ~1e-18 to 2e-20,
    // comfortably inside the manual's own 64-ULP-typical target of
    // ~7e-18).
    localparam logic [95:0] PI_OVER_2 = 96'h3fff_0000_c90fdaa22168c235;

    // cos(r) = sum_{k=0}^{8} COS_C[k] * r^(2k) -- stored highest-degree
    // first (index 0 = r^16 coefficient) for direct Horner evaluation.
    localparam logic [95:0] COS_C0 = 96'h3fd2_0000_d73f9f399dc0f88f; // r^16
    localparam logic [95:0] COS_C1 = 96'hbfda_0000_c9cba54603e4e906; // r^14
    localparam logic [95:0] COS_C2 = 96'h3fe2_0000_8f76c77fc6c4bdaa; // r^12
    localparam logic [95:0] COS_C3 = 96'hbfe9_0000_93f27dbbc4fae397; // r^10
    localparam logic [95:0] COS_C4 = 96'h3fef_0000_d00d00d00d00d00d; // r^8
    localparam logic [95:0] COS_C5 = 96'hbff5_0000_b60b60b60b60b60b; // r^6
    localparam logic [95:0] COS_C6 = 96'h3ffa_0000_aaaaaaaaaaaaaaab; // r^4
    localparam logic [95:0] COS_C7 = 96'hbffe_0000_8000000000000000; // r^2
    localparam logic [95:0] COS_C8 = 96'h3fff_0000_8000000000000000; // r^0

    // sin(r)/r = sum_{k=0}^{8} SIN_C[k] * r^(2k) -- same Horner shape;
    // the caller multiplies the final Horner result by r once at the end.
    localparam logic [95:0] SIN_C0 = 96'h3fce_0000_ca963b81856a5359; // r^16 (of r^17/r)
    localparam logic [95:0] SIN_C1 = 96'hbfd6_0000_d73f9f399dc0f88f; // r^14
    localparam logic [95:0] SIN_C2 = 96'h3fde_0000_b092309d43684be5; // r^12
    localparam logic [95:0] SIN_C3 = 96'hbfe5_0000_d7322b3faa271c7f; // r^10
    localparam logic [95:0] SIN_C4 = 96'h3fec_0000_b8ef1d2ab6399c7d; // r^8
    localparam logic [95:0] SIN_C5 = 96'hbff2_0000_d00d00d00d00d00d; // r^6
    localparam logic [95:0] SIN_C6 = 96'h3ff8_0000_8888888888888889; // r^4
    localparam logic [95:0] SIN_C7 = 96'hbffc_0000_aaaaaaaaaaaaaaab; // r^2
    localparam logic [95:0] SIN_C8 = 96'h3fff_0000_8000000000000000; // r^0

    // Evaluate one Horner step (acc <- acc*w + coeff) using the SAME two
    // call sites of fp_mul/fp_add_sub throughout (a `for` loop over both
    // 9-term polynomials, NOT 16 separately unrolled textual calls) --
    // deliberately, to keep the total distinct call-site count for these
    // two tasks as low as Phase 9c's own already-proven-safe count
    // (empirically tested: repeated LOOPED invocation of the same
    // textual call site is a fundamentally different, and so far always
    // safe, shape than multiple SEPARATE textual call sites -- see
    // fp_mod_rem's own header comment for the original finding this
    // reasoning extends).
    task automatic fp_sincos(
        input  logic [95:0]  a_raw,
        output logic [95:0]  sin_result,
        output logic [95:0]  cos_result,
        output logic         flag_operr
    );
        fpx_t a;
        logic a_nan, a_inf, a_zero;

        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_inf  = is_inf_fpx(a);
        a_zero = is_zero_fpx(a);
        flag_operr = 1'b0;

        if (a_nan) begin
            sin_result = a_raw;
            cos_result = a_raw;
        end else if (a_inf) begin
            flag_operr = 1'b1;
            sin_result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
            cos_result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_zero) begin
            sin_result = a_raw; // sign preserved (Operation Table: +0.0/-0.0)
            cos_result = {1'b0, 15'd16383, 16'h0, 64'h8000_0000_0000_0000}; // +1.0
        end else begin
            logic [95:0] q_ext, k_ext, prod, r, w, cos_acc, sin_acc, tmp;
            logic [95:0] cos_coeff [0:8];
            logic [95:0] sin_coeff [0:8];
            logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
            logic        pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2;
            logic        rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2;
            logic        quot_sign;
            logic [63:0] n_mag;
            logic signed [17:0] real_exp;
            logic [1:0]  k_mod4;
            int          i;

            cos_coeff = '{COS_C0, COS_C1, COS_C2, COS_C3, COS_C4, COS_C5, COS_C6, COS_C7, COS_C8};
            sin_coeff = '{SIN_C0, SIN_C1, SIN_C2, SIN_C3, SIN_C4, SIN_C5, SIN_C6, SIN_C7, SIN_C8};

            // ── Argument reduction: k = round(a / (pi/2)), r = a - k*(pi/2) ──
            fp_div(PI_OVER_2, a_raw, RND_NEAREST, q_ext, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2);

            quot_sign = qn;
            if (qz) begin
                n_mag = 64'h0;
            end else begin
                fpx_t qx;
                qx = unpack_fpx(q_ext);
                real_exp = $signed({3'b0, qx.exp}) - 18'sd16383;
                if (real_exp < 18'sd0) begin
                    logic round_to_1;
                    round_to_1 = (real_exp == -18'sd1) && (qx.mant[62:0] != 63'h0);
                    n_mag = round_to_1 ? 64'd1 : 64'd0;
                end else if (real_exp >= 18'sd63) begin
                    n_mag = 64'hFFFF_FFFF_FFFF_FFFF; // saturate -- see fp_mod_rem's own identical note
                end else begin
                    logic [6:0]  frac_bits;
                    logic [63:0] int_part, sticky_mask;
                    logic        guard, round_bit, sticky, round_up, carry;

                    frac_bits   = 7'(63 - real_exp);
                    int_part    = qx.mant >> frac_bits;
                    guard       = (frac_bits >= 7'd1) ? ((qx.mant >> (frac_bits - 7'd1)) & 64'h1) : 1'b0;
                    round_bit   = (frac_bits >= 7'd2) ? ((qx.mant >> (frac_bits - 7'd2)) & 64'h1) : 1'b0;
                    sticky_mask = (frac_bits >= 7'd3) ? ((64'h1 << (frac_bits - 7'd2)) - 64'h1) : 64'h0;
                    sticky      = |(qx.mant & sticky_mask);
                    round_up    = guard && (round_bit || sticky || int_part[0]);
                    {carry, int_part} = {1'b0, int_part} + (round_up ? 65'd1 : 65'd0);
                    n_mag = int_part;
                end
            end

            k_mod4 = quot_sign ? (2'd0 - n_mag[1:0]) : n_mag[1:0];

            if (n_mag == 64'h0) begin
                r = a_raw;
            end else begin
                logic [6:0] n_lz;
                n_lz  = lzc64(n_mag);
                k_ext = {quot_sign, 15'd16383 + (15'd63 - {8'b0, n_lz}), 16'h0, n_mag << n_lz};
                fp_mul(k_ext, PI_OVER_2, RND_NEAREST, prod, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                fp_add_sub(prod, a_raw, 1'b1, RND_NEAREST, r, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
            end

            // ── Polynomial core: cos(r) and sin(r)/r, both as 9-term
            // Horner evaluations in w=r^2, sharing the SAME two call
            // sites for the whole loop. ──────────────────────────────
            fp_mul(r, r, RND_NEAREST, w, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);

            cos_acc = cos_coeff[0];
            sin_acc = sin_coeff[0];
            for (i = 1; i < 9; i++) begin
                fp_mul(cos_acc, w, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                fp_add_sub(cos_coeff[i], tmp, 1'b0, RND_NEAREST, cos_acc, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
                fp_mul(sin_acc, w, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                fp_add_sub(sin_coeff[i], tmp, 1'b0, RND_NEAREST, sin_acc, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
            end
            // sin(r) = r * (sin(r)/r series)
            fp_mul(sin_acc, r, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);

            // ── Quadrant reconstruction (k mod 4) ────────────────────
            // Plain case with a `default` (not `unique case`) -- k_mod4
            // is arithmetically exhaustive over 2'd0..2'd3 for any real
            // input, but at simulation time 0 (before reset/first valid
            // operand) a_raw is still X, which propagates through to an
            // X k_mod4; `unique case` would flag that as a real
            // priority violation every run. The `default` arm exists
            // solely to absorb that transient X, not because a 5th real
            // case exists.
            case (k_mod4)
                2'd0: begin sin_result = tmp;                              cos_result = cos_acc; end
                2'd1: begin sin_result = cos_acc;                          cos_result = {!tmp[95], tmp[94:0]}; end
                2'd2: begin sin_result = {!tmp[95], tmp[94:0]};            cos_result = {!cos_acc[95], cos_acc[94:0]}; end
                default: begin sin_result = {!cos_acc[95], cos_acc[94:0]}; cos_result = tmp; end
            endcase
        end
    endtask

    // ── Phase 9f: FETOX ($10) / FETOXM1 ($08) / FTWOTOX ($11) /
    // FTENTOX ($12) -- the exponential family, all built on one shared
    // `fp_exp_core` task (e^y for arbitrary y) the same way Phase 9d's
    // FSIN/FCOS shared `fp_sincos`. Like FSIN/FCOS, these are genuinely
    // APPROXIMATED, not exact (Section 4.3's own "~64 ULP typical /
    // 4096 ULP worst-case" bound applies here too, not IEEE 0.5 ULP).
    //
    // Algorithm: classic argument reduction, k = round(y/ln2), r = y -
    // k*ln2 (|r| <= ln2/2 ~= 0.347), e^y = 2^k * e^r. e^r evaluated via
    // a 16-term (r^0..r^15) Horner series in r directly (NOT split into
    // even/odd sub-series the way cos/sin's own r^2-Horner was -- e^r
    // has no such symmetry to exploit); independently verified in
    // Python (Decimal, 80 digits) to have truncation error ~1e-19 at
    // |r|=ln2/2 (the worst case), comfortably inside the ~6.9e-18
    // target implied by the manual's own "~64 ULP typical" bound, same
    // margin philosophy as FSIN/FCOS's own 9-term series. The 2^k
    // scaling is then just a biased-EXPONENT addition with saturation
    // (identical pattern to `fp_scale`'s own already-established
    // biased-exponent arithmetic) -- no second multiply needed, since
    // this project's own internal format carries an explicit binary
    // exponent.
    localparam logic [95:0] LN2  = 96'h3ffe_0000_b17217f7d1cf79ac;
    localparam logic [95:0] LN10 = 96'h4000_0000_935d8dddaaa8ac17;

    // e^r = sum_{k=0}^{15} EXP_C[k] * r^k -- stored highest-degree first
    // (index 0 = r^15 coefficient) for direct Horner evaluation, same
    // convention as COS_C/SIN_C above.
    localparam logic [95:0] EXP_C0  = 96'h3fd6_0000_d73f9f399dc0f88f; // r^15
    localparam logic [95:0] EXP_C1  = 96'h3fda_0000_c9cba54603e4e906; // r^14
    localparam logic [95:0] EXP_C2  = 96'h3fde_0000_b092309d43684be5; // r^13
    localparam logic [95:0] EXP_C3  = 96'h3fe2_0000_8f76c77fc6c4bdaa; // r^12
    localparam logic [95:0] EXP_C4  = 96'h3fe5_0000_d7322b3faa271c7f; // r^11
    localparam logic [95:0] EXP_C5  = 96'h3fe9_0000_93f27dbbc4fae397; // r^10
    localparam logic [95:0] EXP_C6  = 96'h3fec_0000_b8ef1d2ab6399c7d; // r^9
    localparam logic [95:0] EXP_C7  = 96'h3fef_0000_d00d00d00d00d00d; // r^8
    localparam logic [95:0] EXP_C8  = 96'h3ff2_0000_d00d00d00d00d00d; // r^7
    localparam logic [95:0] EXP_C9  = 96'h3ff5_0000_b60b60b60b60b60b; // r^6
    localparam logic [95:0] EXP_C10 = 96'h3ff8_0000_8888888888888889; // r^5
    localparam logic [95:0] EXP_C11 = 96'h3ffa_0000_aaaaaaaaaaaaaaab; // r^4
    localparam logic [95:0] EXP_C12 = 96'h3ffc_0000_aaaaaaaaaaaaaaab; // r^3
    localparam logic [95:0] EXP_C13 = 96'h3ffe_0000_8000000000000000; // r^2
    localparam logic [95:0] EXP_C14 = 96'h3fff_0000_8000000000000000; // r^1
    localparam logic [95:0] EXP_C15 = 96'h3fff_0000_8000000000000000; // r^0

    task automatic fp_exp_core(
        input  logic [95:0]  y_raw,
        output logic [95:0]  result,
        output logic         flag_z,
        output logic         flag_n,
        output logic         flag_i,
        output logic         flag_nan,
        output logic         flag_ovfl,
        output logic         flag_unfl
    );
        fpx_t y;
        logic y_nan, y_inf, y_zero;

        y = unpack_fpx(y_raw);
        y_nan  = is_nan_fpx(y);
        y_inf  = is_inf_fpx(y);
        y_zero = is_zero_fpx(y);
        flag_ovfl = 1'b0;
        flag_unfl = 1'b0;

        if (y_nan) begin
            result = y_raw;
        end else if (y_inf) begin
            // e^(+inf) = +inf; e^(-inf) = +0.0 -- both well-defined,
            // unlike FSIN/FCOS's own undefined-at-infinity case; no
            // OPERR here.
            result = y.sign ? 96'h0 : {1'b0, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
        end else if (y_zero) begin
            result = {1'b0, 15'd16383, 16'h0, 64'h8000_0000_0000_0000}; // e^0 = +1.0, sign of zero irrelevant
        end else begin
            logic [95:0] q_ext, k_ext, prod, r, acc, tmp;
            logic [95:0] exp_coeff [0:15];
            logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
            logic        pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2;
            logic        rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2;
            logic        k_sign;
            logic [63:0] n_mag;
            logic signed [17:0] real_exp;
            logic signed [79:0] new_exp_wide;
            int          i;

            exp_coeff = '{EXP_C0, EXP_C1, EXP_C2, EXP_C3, EXP_C4, EXP_C5, EXP_C6, EXP_C7,
                          EXP_C8, EXP_C9, EXP_C10, EXP_C11, EXP_C12, EXP_C13, EXP_C14, EXP_C15};

            // ── Argument reduction: k = round(y / ln2), r = y - k*ln2 --
            // identical shape to fp_sincos's own k=round(a/(pi/2))
            // derivation (fp_div's own convention is divisor-first,
            // dividend-second: fp_div(LN2, y_raw) = y_raw/LN2). ──────
            fp_div(LN2, y_raw, RND_NEAREST, q_ext, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2);

            k_sign = qn;
            if (qz) begin
                n_mag = 64'h0;
            end else begin
                fpx_t qx;
                qx = unpack_fpx(q_ext);
                real_exp = $signed({3'b0, qx.exp}) - 18'sd16383;
                if (real_exp < 18'sd0) begin
                    logic round_to_1;
                    round_to_1 = (real_exp == -18'sd1) && (qx.mant[62:0] != 63'h0);
                    n_mag = round_to_1 ? 64'd1 : 64'd0;
                end else if (real_exp >= 18'sd63) begin
                    n_mag = 64'hFFFF_FFFF_FFFF_FFFF; // saturate -- see fp_mod_rem/fp_sincos's own identical note
                end else begin
                    logic [6:0]  frac_bits;
                    logic [63:0] int_part, sticky_mask;
                    logic        guard, round_bit, sticky, round_up, carry;

                    frac_bits   = 7'(63 - real_exp);
                    int_part    = qx.mant >> frac_bits;
                    guard       = (frac_bits >= 7'd1) ? ((qx.mant >> (frac_bits - 7'd1)) & 64'h1) : 1'b0;
                    round_bit   = (frac_bits >= 7'd2) ? ((qx.mant >> (frac_bits - 7'd2)) & 64'h1) : 1'b0;
                    sticky_mask = (frac_bits >= 7'd3) ? ((64'h1 << (frac_bits - 7'd2)) - 64'h1) : 64'h0;
                    sticky      = |(qx.mant & sticky_mask);
                    round_up    = guard && (round_bit || sticky || int_part[0]);
                    {carry, int_part} = {1'b0, int_part} + (round_up ? 65'd1 : 65'd0);
                    n_mag = int_part;
                end
            end

            if (n_mag == 64'h0) begin
                r = y_raw;
            end else begin
                logic [6:0] n_lz;
                n_lz  = lzc64(n_mag);
                k_ext = {k_sign, 15'd16383 + (15'd63 - {8'b0, n_lz}), 16'h0, n_mag << n_lz};
                fp_mul(k_ext, LN2, RND_NEAREST, prod, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                fp_add_sub(prod, y_raw, 1'b1, RND_NEAREST, r, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
            end

            // ── Polynomial core: e^r, a 16-term Horner evaluation in r
            // directly (not r^2 -- no even/odd split available here),
            // one shared fp_mul/fp_add_sub call-site pair for the loop. ──
            acc = exp_coeff[0];
            for (i = 1; i < 16; i++) begin
                fp_mul(acc, r, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                fp_add_sub(exp_coeff[i], tmp, 1'b0, RND_NEAREST, acc, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
            end

            // ── 2^k scaling: add k to acc's own biased exponent field,
            // with saturation -- identical shape to fp_scale's own
            // already-established biased-exponent arithmetic, just
            // computed in a wide signed container first (n_mag can in
            // principle be as large as the saturated 64'hFFFF...FFFF
            // above) so the add itself never wraps before the
            // saturation check gets to see it. ─────────────────────────
            begin
                fpx_t acc_x;
                acc_x = unpack_fpx(acc);
                new_exp_wide = $signed({65'b0, acc_x.exp})
                             + (k_sign ? -$signed({16'b0, n_mag}) : $signed({16'b0, n_mag}));
                if (new_exp_wide >= 80'sd32767) begin
                    flag_ovfl = 1'b1;
                    result = {1'b0, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000};
                end else if (new_exp_wide <= 80'sd0) begin
                    flag_unfl = 1'b1;
                    result = 96'h0;
                end else begin
                    result = {1'b0, new_exp_wide[14:0], 16'h0, acc_x.mant};
                end
            end
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = 1'b0; // e^y is never negative
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    // FETOX ($10): e^a, direct.
    task automatic fp_etox(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl
    );
        fp_exp_core(a_raw, result, flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl);
    endtask

    // FTWOTOX ($11): 2^a = e^(a*ln2). Real-hardware-equivalent scaling
    // multiply, then the same core.
    task automatic fp_twotox(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl
    );
        logic [95:0] y;
        logic        mz, mn, mi, mnan, moperr, movfl, munfl, minex2;
        fpx_t a;
        a = unpack_fpx(a_raw);
        if (is_nan_fpx(a) || is_inf_fpx(a) || is_zero_fpx(a)) begin
            // fp_exp_core's own special cases already handle NaN/inf/
            // zero identically for e^y as real hardware wants for 2^a
            // at these same operand classes (2^(+-inf)=+inf/+0,
            // 2^0=1) -- skip the LN2 scaling multiply entirely rather
            // than risk it producing something other than an exact
            // NaN/inf/zero passthrough.
            fp_exp_core(a_raw, result, flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl);
        end else begin
            fp_mul(a_raw, LN2, RND_NEAREST, y, mz, mn, mi, mnan, moperr, movfl, munfl, minex2);
            fp_exp_core(y, result, flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl);
        end
    endtask

    // FTENTOX ($12): 10^a = e^(a*ln10). Same shape as FTWOTOX.
    task automatic fp_tentox(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl
    );
        logic [95:0] y;
        logic        mz, mn, mi, mnan, moperr, movfl, munfl, minex2;
        fpx_t a;
        a = unpack_fpx(a_raw);
        if (is_nan_fpx(a) || is_inf_fpx(a) || is_zero_fpx(a)) begin
            fp_exp_core(a_raw, result, flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl);
        end else begin
            fp_mul(a_raw, LN10, RND_NEAREST, y, mz, mn, mi, mnan, moperr, movfl, munfl, minex2);
            fp_exp_core(y, result, flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl);
        end
    endtask

    // FETOXM1 ($08): e^a - 1, computed directly (not e^a then subtract
    // 1) whenever |a| < ln2/2 -- i.e. whenever fp_exp_core's own
    // argument reduction would find k=0 and evaluate the Horner series
    // on a itself. In that case e^a-1 is just the SAME Horner series
    // minus its own leading (r^0) term -- exact cancellation of the
    // "-1" by construction, no precision lost to catastrophic
    // cancellation the way naively computing (e^a - 1.0) as a separate
    // subtraction would for small a. Outside that range (|a| >= ln2/2),
    // e^a itself is not close to 1 (or is exactly 0/inf), so an
    // ordinary e^a-1 subtraction has no cancellation problem and is
    // used instead -- reusing fp_exp_core as-is rather than duplicating
    // its own reduction/Horner logic a second time.
    task automatic fp_etoxm1(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_ovfl, flag_unfl
    );
        fpx_t a;
        logic a_nan, a_inf, a_zero;

        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_inf  = is_inf_fpx(a);
        a_zero = is_zero_fpx(a);
        flag_ovfl = 1'b0;
        flag_unfl = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (a_inf) begin
            result = a.sign ? {1'b1, 15'd16383, 16'h0, 64'h8000_0000_0000_0000} // e^-inf - 1 = -1.0
                             : {1'b0, 15'h7FFF, 16'h0, 64'h8000_0000_0000_0000}; // e^+inf - 1 = +inf
        end else if (a_zero) begin
            result = a_raw; // e^0 - 1 = 0, sign of zero preserved
        end else begin
            logic [95:0] q_ext;
            logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
            fp_div(LN2, a_raw, RND_NEAREST, q_ext, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2);
            // small_a: does round-to-nearest-even(a/ln2) == 0, i.e. does
            // fp_exp_core's own reduction take k=0 for this a (in which
            // case r==a exactly and the series-minus-leading-term path
            // below is exact)? True iff |a/ln2| <= 0.5 (an EXACT 0.5
            // rounds to 0 too, being even) -- q_ext's own real exponent
            // < -2 covers magnitude < 0.25..0.5 range trivially; the
            // boundary real_exp==-1 (value in [0.5,1.0)) is small_a only
            // at the exact endpoint (mantissa's own explicit-integer-bit
            // convention means mantissa==0x8000...0000 there is exactly
            // 0.5, nothing smaller is representable at that exponent).
            begin
                fpx_t qx;
                logic small_a;
                logic signed [17:0] q_real_exp;
                qx = unpack_fpx(q_ext);
                q_real_exp = $signed({3'b0, qx.exp}) - 18'sd16383;
                small_a = qz || (q_real_exp <= -18'sd2) ||
                          ((q_real_exp == -18'sd1) && (qx.mant == 64'h8000_0000_0000_0000));
                if (small_a) begin
                    logic [95:0] acc, tmp, r;
                    logic [95:0] exp_coeff [0:15];
                    logic pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2;
                    logic rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2;
                    int i;
                    exp_coeff = '{EXP_C0, EXP_C1, EXP_C2, EXP_C3, EXP_C4, EXP_C5, EXP_C6, EXP_C7,
                                  EXP_C8, EXP_C9, EXP_C10, EXP_C11, EXP_C12, EXP_C13, EXP_C14, EXP_C15};
                    r = a_raw; // k=0, so r == a exactly
                    acc = exp_coeff[0];
                    for (i = 1; i < 15; i++) begin // stop BEFORE the r^0 term (index 15) -- that's the "-1" being dropped
                        fp_mul(acc, r, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                        fp_add_sub(exp_coeff[i], tmp, 1'b0, RND_NEAREST, acc, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
                    end
                    // acc is now sum_{k=1}^{15} EXP_C[k]*r^(15-k) == the
                    // r^1..r^15 terms only, i.e. e^r - 1 directly, one
                    // final multiply by r itself away (mirrors sin(r)'s
                    // own "series is really f(r)/r, multiply by r once
                    // at the end" shape in fp_sincos).
                    fp_mul(acc, r, RND_NEAREST, result, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
                end else begin
                    logic [95:0] etox_result, one_ext;
                    logic ez, en, ei, enan, eovfl, eunfl;
                    logic sz, sn, si, snan, soperr, sdz, sovfl, sunfl, sinex2;
                    one_ext = {1'b0, 15'd16383, 16'h0, 64'h8000_0000_0000_0000};
                    fp_exp_core(a_raw, etox_result, ez, en, ei, enan, eovfl, eunfl);
                    fp_add_sub(one_ext, etox_result, 1'b1, RND_NEAREST, result, sz, sn, si, snan, soperr, sovfl, sunfl, sinex2);
                    flag_ovfl = eovfl;
                end
            end
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    // ── Phase 9g: FSINH ($02) / FCOSH ($19) / FTANH ($09) -- the
    // hyperbolic family, built directly on Phase 9f's own fp_exp_core:
    // sinh(x)=(e^x-e^-x)/2, cosh(x)=(e^x+e^-x)/2, tanh(x)=sinh(x)/
    // cosh(x). No new numerical core needed -- pure composition of
    // already-verified building blocks.
    //
    // fp_sinh/fp_cosh each call fp_exp_core TWICE (once for +a, once
    // for -a -- genuinely different arguments, not the same textual-
    // call-site shape the project's own Icarus livelock finding warns
    // about) -- empirically re-confirmed safe the same way fp_mod_rem
    // and fp_sincos each were (full `make test`, no hang). Both
    // naturally get correct +-infinity results with NO explicit
    // overflow/underflow special-casing needed at all: fp_exp_core's
    // own already-established saturation (Phase 9f) makes e^(large
    // positive) come out as exactly +inf and e^(large negative) as
    // exactly +0.0, and fp_add_sub/fp_mul's own already-proven
    // infinity arithmetic (Phase 4a) does the rest correctly by
    // composition alone.
    localparam logic [95:0] HALF = 96'h3ffe_0000_8000000000000000;

    task automatic fp_sinh(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan
    );
        fpx_t a;
        logic a_nan, a_zero;
        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_zero = is_zero_fpx(a);

        if (a_nan) begin
            result = a_raw;
        end else if (a_zero) begin
            result = a_raw; // sinh is odd: sinh(-0.0) = -0.0, sign preserved
        end else begin
            logic [95:0] neg_a, ep, en, diff;
            logic        ez, en1, ei, enan, eovfl, eunfl;
            logic        fz, fn1, fi, fnan, fovfl, funfl;
            logic        sz, sn, si, snan, soperr, sovfl, sunfl, sinex2;
            logic        mz, mn, mi, mnan, moperr, movfl, munfl, minex2;
            neg_a = {!a_raw[95], a_raw[94:0]};
            fp_exp_core(a_raw, ep, ez, en1, ei, enan, eovfl, eunfl);
            fp_exp_core(neg_a, en, fz, fn1, fi, fnan, fovfl, funfl);
            fp_add_sub(en, ep, 1'b1, RND_NEAREST, diff, sz, sn, si, snan, soperr, sovfl, sunfl, sinex2); // ep - en
            fp_mul(diff, HALF, RND_NEAREST, result, mz, mn, mi, mnan, moperr, movfl, munfl, minex2);
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    task automatic fp_cosh(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan
    );
        fpx_t a;
        logic a_nan;
        a = unpack_fpx(a_raw);
        a_nan = is_nan_fpx(a);

        if (a_nan) begin
            result = a_raw;
        end else begin
            logic [95:0] neg_a, ep, en, sum;
            logic        ez, en1, ei, enan, eovfl, eunfl;
            logic        fz, fn1, fi, fnan, fovfl, funfl;
            logic        sz, sn, si, snan, soperr, sovfl, sunfl, sinex2;
            logic        mz, mn, mi, mnan, moperr, movfl, munfl, minex2;
            neg_a = {!a_raw[95], a_raw[94:0]};
            fp_exp_core(a_raw, ep, ez, en1, ei, enan, eovfl, eunfl);
            fp_exp_core(neg_a, en, fz, fn1, fi, fnan, fovfl, funfl);
            fp_add_sub(en, ep, 1'b0, RND_NEAREST, sum, sz, sn, si, snan, soperr, sovfl, sunfl, sinex2); // ep + en
            fp_mul(sum, HALF, RND_NEAREST, result, mz, mn, mi, mnan, moperr, movfl, munfl, minex2);
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z; // never true in practice: cosh(x) >= 1.0 always
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    task automatic fp_tanh(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan
    );
        fpx_t a;
        logic a_nan, a_zero;
        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_zero = is_zero_fpx(a);

        if (a_nan) begin
            result = a_raw;
        end else if (a_zero) begin
            result = a_raw; // tanh is odd: tanh(-0.0) = -0.0, sign preserved
        end else begin
            logic [95:0] sinh_r, cosh_r;
            logic        shz, shn, shi, shnan, chz, chn, chi, chnan;
            logic        dz, dn, di, dnan, doperr, ddz, dovfl, dunfl, dinex2;
            fp_sinh(a_raw, sinh_r, shz, shn, shi, shnan);
            fp_cosh(a_raw, cosh_r, chz, chn, chi, chnan);
            if (chi) begin
                // |a| large enough that e^|a| itself overflowed inside
                // fp_exp_core -- cosh AND sinh both saturate to the
                // SAME-SIGN infinity, so a literal division would hit
                // fp_div's own inf/inf (indeterminate, NaN) case instead
                // of the real, well-defined asymptotic limit +-1.0
                // (matches the Operation Table's own FTANH(+-inf)=+-1).
                // Short-circuit directly rather than let the division
                // produce a spurious NaN.
                result = a.sign ? {1'b1, 15'd16383, 16'h0, 64'h8000_0000_0000_0000}
                                 : {1'b0, 15'd16383, 16'h0, 64'h8000_0000_0000_0000};
            end else begin
                fp_div(cosh_r, sinh_r, RND_NEAREST, result, dz, dn, di, dnan, doperr, ddz, dovfl, dunfl, dinex2);
            end
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    // ── Phase 9g: FTAN ($0F) -- reuses fp_sincos directly (tan(x) =
    // sin(x)/cos(x)), no new series of its own. ─────────────────────
    task automatic fp_tan(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr
    );
        fpx_t a;
        a = unpack_fpx(a_raw);

        if (is_nan_fpx(a)) begin
            result = a_raw;
            flag_operr = 1'b0;
        end else begin
            logic [95:0] sin_r, cos_r;
            logic        sc_operr;
            logic        dz, dn, di, dnan, doperr, ddz, dovfl, dunfl, dinex2;
            fp_sincos(a_raw, sin_r, cos_r, sc_operr);
            if (sc_operr) begin
                // Infinite source: fp_sincos's own NaN(OPERR) pattern
                // (identical for sin_r/cos_r in that case) is already
                // the right TAN result too -- same undefined-at-
                // infinity shape every trig function here shares.
                result = sin_r;
                flag_operr = 1'b1;
            end else begin
                // Divide-by-zero at odd multiples of pi/2 (where cos_r
                // is exactly zero) surfaces as flag_i (infinity) via
                // fp_div's own DZ path rather than the manual's own
                // documented OPERR at that exact boundary -- a known
                // simplification, not silently unhandled: in practice
                // this project's own finite-Taylor-series cos(r) only
                // ever reaches exactly 0.0 for a source that reduces to
                // r's own exact analytic zero, which no discrete test
                // input coincides with.
                fp_div(cos_r, sin_r, RND_NEAREST, result, dz, dn, di, dnan, doperr, ddz, dovfl, dunfl, dinex2);
                flag_operr = 1'b0;
            end
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    // ── Phase 9h: FLOGN ($14) / FLOGNP1 ($06) / FLOG10 ($15) / FLOG2
    // ($16) -- the logarithm family, all built on one shared
    // `fp_logn_core` task (ln(x) for x>0), the same "one shared
    // numerical core, thin per-instruction wrappers" shape as
    // `fp_sincos`/`fp_exp_core`.
    //
    // Algorithm: standard atanh-based reduction, NOT a plain Taylor
    // series in (x-1) (which only converges usefully near x=1 and far
    // too slowly near x=2 to be practical). Write x = m * 2^e with m in
    // [1,2) -- exactly this project's own internal mantissa/exponent
    // split, no extra work to derive -- then ln(x) = ln(m) + e*ln(2).
    // ln(m) is computed via t=(m-1)/(m+1) (|t|<=1/3 for m in [1,2)),
    // ln(m) = 2*(t + t^3/3 + t^5/5 + ...) = 2*t*P(t^2), P evaluated as
    // an 18-term (w^0..w^17) Horner series in w=t^2 -- independently
    // verified in Python (Decimal, 80 digits) to have truncation error
    // ~6.7e-20 at the worst case |t|=1/3, comfortably inside the
    // ~6.9e-18 target implied by the manual's own "~64 ULP typical"
    // bound (needing more terms than fp_exp_core's own 16 or
    // fp_sincos's own 9, since this series lacks factorial-accelerated
    // convergence -- purely geometric decay by t^2 per term instead).
    // The e*ln2 correction term reuses `int32_to_ext` (already
    // established by `fp_getexp`) to convert the signed unbiased
    // exponent directly to a floatx80 value.
    //
    // Operation Table / Table 6-2 (Operand Error) / Table 6-3 (Divide-
    // by-Zero) entries, confirmed directly: FLOGx(source<0 or -inf) =
    // OPERR, NaN; FLOGN/FLOG10/FLOG2(source=0) = DZ, -infinity;
    // FLOGx(+inf) = +inf, well-defined, no exception.
    localparam logic [95:0] LN_C0  = 96'h3ff9_0000_ea0ea0ea0ea0ea0f; // w^17
    localparam logic [95:0] LN_C1  = 96'h3ff9_0000_f83e0f83e0f83e10; // w^16
    localparam logic [95:0] LN_C2  = 96'h3ffa_0000_8421084210842108; // w^15
    localparam logic [95:0] LN_C3  = 96'h3ffa_0000_8d3dcb08d3dcb08d; // w^14
    localparam logic [95:0] LN_C4  = 96'h3ffa_0000_97b425ed097b425f; // w^13
    localparam logic [95:0] LN_C5  = 96'h3ffa_0000_a3d70a3d70a3d70a; // w^12
    localparam logic [95:0] LN_C6  = 96'h3ffa_0000_b21642c8590b2164; // w^11
    localparam logic [95:0] LN_C7  = 96'h3ffa_0000_c30c30c30c30c30c; // w^10
    localparam logic [95:0] LN_C8  = 96'h3ffa_0000_d79435e50d79435e; // w^9
    localparam logic [95:0] LN_C9  = 96'h3ffa_0000_f0f0f0f0f0f0f0f1; // w^8
    localparam logic [95:0] LN_C10 = 96'h3ffb_0000_8888888888888889; // w^7
    localparam logic [95:0] LN_C11 = 96'h3ffb_0000_9d89d89d89d89d8a; // w^6
    localparam logic [95:0] LN_C12 = 96'h3ffb_0000_ba2e8ba2e8ba2e8c; // w^5
    localparam logic [95:0] LN_C13 = 96'h3ffb_0000_e38e38e38e38e38e; // w^4
    localparam logic [95:0] LN_C14 = 96'h3ffc_0000_9249249249249249; // w^3
    localparam logic [95:0] LN_C15 = 96'h3ffc_0000_cccccccccccccccd; // w^2
    localparam logic [95:0] LN_C16 = 96'h3ffd_0000_aaaaaaaaaaaaaaab; // w^1
    localparam logic [95:0] LN_C17 = 96'h3fff_0000_8000000000000000; // w^0

    localparam logic [95:0] ONE_EXT = 96'h3fff_0000_8000000000000000;
    localparam logic [95:0] TWO_EXT = 96'h4000_0000_8000000000000000;
    localparam logic [95:0] NEG_INF_EXT = 96'hffff_0000_8000000000000000;

    // ln(m) for m ALREADY in [1,2) -- the shared atanh-series core, used
    // both by fp_logn_core's own general case (on the mantissa it
    // extracts) and directly by fp_lognp1's own small-x path (which
    // constructs its own effective "m" algebraically, without ever
    // decomposing a real floatx80 exponent -- see that task's own
    // header comment).
    task automatic ln_series(
        input  logic [95:0]  t_in,
        output logic [95:0]  ln_m
    );
        logic [95:0] w, tmp, acc;
        logic [95:0] ln_coeff [0:17];
        logic pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2;
        logic rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2;
        int i;
        ln_coeff = '{LN_C0, LN_C1, LN_C2, LN_C3, LN_C4, LN_C5, LN_C6, LN_C7, LN_C8, LN_C9,
                     LN_C10, LN_C11, LN_C12, LN_C13, LN_C14, LN_C15, LN_C16, LN_C17};
        fp_mul(t_in, t_in, RND_NEAREST, w, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
        acc = ln_coeff[0];
        for (i = 1; i < 18; i++) begin
            fp_mul(acc, w, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
            fp_add_sub(ln_coeff[i], tmp, 1'b0, RND_NEAREST, acc, rz, rn, ri, rnan, roperr, rovfl, runfl, rinex2);
        end
        fp_mul(acc, t_in, RND_NEAREST, tmp, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
        fp_mul(tmp, TWO_EXT, RND_NEAREST, ln_m, pz, pn, pi_, pnan, poperr, povfl, punfl, pinex2);
    endtask

    task automatic fp_logn_core(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz
    );
        fpx_t a;
        logic a_nan, a_zero, a_inf;
        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_zero = is_zero_fpx(a);
        a_inf  = is_inf_fpx(a);
        flag_operr = 1'b0;
        flag_dz    = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (a_zero) begin
            flag_dz = 1'b1;
            result = NEG_INF_EXT;
        end else if (a.sign) begin
            // Negative source (finite or -infinity, both sign=1) --
            // Table 6-2's own "Source is <0, Source=-infinity" entry
            // covers both in one check.
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_inf) begin
            result = a_raw; // +inf -> +inf, well-defined, no exception
        end else begin
            logic [95:0] m_ext, num, den, t, e_ext, scaled, ln_m;
            logic        nz, nn, ni, nnan, noperr, novfl, nunfl, ninex2;
            logic        dz2, dn2, di2, dnan2, doperr2, ddz2, dovfl2, dunfl2, dinex2b;
            logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
            logic        mz, mn, mi, mnan, moperr, movfl, munfl, minex2;
            logic        az, an, ai, anan, aoperr, aovfl, aunfl, ainex2;
            logic signed [31:0] e_val;

            m_ext = {1'b0, 15'd16383, 16'h0, a.mant}; // reinterpret mantissa as its own [1,2) value
            e_val = $signed({17'b0, a.exp}) - 32'sd16383;

            fp_add_sub(ONE_EXT, m_ext, 1'b1, RND_NEAREST, num, nz, nn, ni, nnan, noperr, novfl, nunfl, ninex2); // m - 1
            fp_add_sub(ONE_EXT, m_ext, 1'b0, RND_NEAREST, den, dz2, dn2, di2, dnan2, doperr2, dovfl2, dunfl2, dinex2b); // m + 1
            fp_div(den, num, RND_NEAREST, t, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2); // t = (m-1)/(m+1)

            ln_series(t, ln_m);

            int32_to_ext(e_val, e_ext);
            fp_mul(e_ext, LN2, RND_NEAREST, scaled, mz, mn, mi, mnan, moperr, movfl, munfl, minex2);
            fp_add_sub(scaled, ln_m, 1'b0, RND_NEAREST, result, az, an, ai, anan, aoperr, aovfl, aunfl, ainex2);
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

    // FLOGN ($14): ln(a), direct.
    task automatic fp_logn(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz
    );
        fp_logn_core(a_raw, result, flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz);
    endtask

    // FLOG10 ($15) / FLOG2 ($16): ln(a)/ln(10), ln(a)/ln(2). NaN/inf/
    // zero/negative results from fp_logn_core already come out correct
    // after being divided by a normal positive constant (-inf/LN10=
    // -inf, NaN/LN10=NaN, etc.) -- only OPERR/DZ need to be forwarded
    // explicitly, since fp_div itself has no way to know THOSE
    // exceptions belong to the LOG operation, not the division.
    task automatic fp_log10(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz
    );
        logic [95:0] ln_a;
        logic        lz, ln, li, lnan, loperr, ldz;
        logic        dn_, dd, di_, dnan_, doperr_, ddz_, dovfl_, dunfl_, dinex2_;
        fp_logn_core(a_raw, ln_a, lz, ln, li, lnan, loperr, ldz);
        fp_div(LN10, ln_a, RND_NEAREST, result, flag_z, flag_n, flag_i, flag_nan,
               doperr_, ddz_, dovfl_, dunfl_, dinex2_);
        flag_operr = loperr;
        flag_dz    = ldz;
    endtask

    task automatic fp_log2(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz
    );
        logic [95:0] ln_a;
        logic        lz, ln, li, lnan, loperr, ldz;
        logic        dn_, dd, di_, dnan_, doperr_, ddz_, dovfl_, dunfl_, dinex2_;
        fp_logn_core(a_raw, ln_a, lz, ln, li, lnan, loperr, ldz);
        fp_div(LN2, ln_a, RND_NEAREST, result, flag_z, flag_n, flag_i, flag_nan,
               doperr_, ddz_, dovfl_, dunfl_, dinex2_);
        flag_operr = loperr;
        flag_dz    = ldz;
    endtask

    // FLOGNP1 ($06): ln(1+a), computed via the algebraic identity
    // ln(1+x) = 2*atanh(x/(x+2)) DIRECTLY on x -- deliberately never
    // forms the literal sum "1+x" when |x| is small, since doing so
    // would silently absorb x's own low-order bits into the dominant
    // "1" (the same class of precision loss FETOXM1's own header
    // comment describes for e^x-1, just for logarithms instead of
    // exponentials). This identity's own t=x/(x+2) also happens to stay
    // inside ln_series's own safe convergence radius (|t|<=1/3) for
    // x in roughly [-0.5, 1.0], comfortably covering the whole "small
    // x" regime this precision concern actually matters for. Outside
    // that range, |x| is not small, so forming "1+x" explicitly loses
    // no more than a few low bits of x relative to its own much larger
    // magnitude -- safe within this project's own ~64-ULP-typical
    // target -- and the general fp_logn_core (with its own real
    // exponent-extraction reduction, valid for any x>-1) is used
    // instead.
    task automatic fp_lognp1(
        input  logic [95:0]  a_raw,
        output logic [95:0]  result,
        output logic         flag_z, flag_n, flag_i, flag_nan, flag_operr, flag_dz
    );
        fpx_t a;
        logic a_nan, a_zero, a_inf;
        a = unpack_fpx(a_raw);
        a_nan  = is_nan_fpx(a);
        a_zero = is_zero_fpx(a);
        a_inf  = is_inf_fpx(a);
        flag_operr = 1'b0;
        flag_dz    = 1'b0;

        if (a_nan) begin
            result = a_raw;
        end else if (a_zero) begin
            result = a_raw; // ln(1+0) = 0, sign of zero preserved
        end else if (a.sign && a.exp == 15'd16383 && a.mant == 64'h8000_0000_0000_0000) begin
            // a == -1.0 exactly (Table 6-3: "Source Operand = -1")
            flag_dz = 1'b1;
            result = NEG_INF_EXT;
        end else if (a.sign && a_inf) begin
            // a == -infinity (Table 6-2: "Source= - infinity")
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a.sign && ((a.exp > 15'd16383) ||
                                 (a.exp == 15'd16383 && a.mant > 64'h8000_0000_0000_0000))) begin
            // a < -1.0 (Table 6-2: "Source is < -1")
            flag_operr = 1'b1;
            result = {1'b0, 15'h7FFF, 16'h0, 64'hC000_0000_0000_0000};
        end else if (a_inf) begin
            result = a_raw; // +inf -> +inf, well-defined
        end else begin
            logic [95:0] den, t;
            logic        dz2, dn2, di2, dnan2, doperr2, dovfl2, dunfl2, dinex2b;
            logic        qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2;
            fp_add_sub(TWO_EXT, a_raw, 1'b0, RND_NEAREST, den, dz2, dn2, di2, dnan2, doperr2, dovfl2, dunfl2, dinex2b); // x+2, always safe (x>-1 here, so x+2>1, no absorption concern)
            fp_div(den, a_raw, RND_NEAREST, t, qz, qn, qi, qnan, qoperr, qdz, qovfl, qunfl, qinex2); // t = x/(x+2)

            begin
                fpx_t tx;
                logic signed [17:0] t_real_exp;
                logic small_x;
                tx = unpack_fpx(t);
                t_real_exp = $signed({3'b0, tx.exp}) - 18'sd16383;
                small_x = qz || (t_real_exp <= -18'sd2) ||
                          ((t_real_exp == -18'sd1) && (tx.mant == 64'h8000_0000_0000_0000));
                if (small_x) begin
                    // |t|<=1/3: ln_series directly on t IS the whole
                    // answer -- no e*ln2 correction needed, this
                    // identity never decomposed a real exponent at all.
                    ln_series(t, result);
                end else begin
                    logic [95:0] one_plus_a;
                    logic        oz, on, oi, onan, ooperr, oovfl, ounfl, oinex2;
                    logic        lz, ln, li, lnan, loperr, ldz;
                    // |x| not small here -- forming 1+x explicitly loses
                    // no more than a few low bits of x, safe within this
                    // project's own ~64-ULP-typical target.
                    fp_add_sub(ONE_EXT, a_raw, 1'b0, RND_NEAREST, one_plus_a, oz, on, oi, onan, ooperr, oovfl, ounfl, oinex2);
                    fp_logn_core(one_plus_a, result, lz, ln, li, lnan, loperr, ldz);
                end
            end
        end

        begin
            fpx_t r_out;
            r_out = unpack_fpx(result);
            flag_z   = is_zero_fpx(r_out);
            flag_n   = r_out.sign && !flag_z;
            flag_i   = is_inf_fpx(r_out);
            flag_nan = is_nan_fpx(r_out);
        end
    endtask

endpackage
