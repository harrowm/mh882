/* tools/musashi_fpu_ref.c — Musashi MC68881/2 FPU golden-reference tool.
 *
 * Phase 7 (verification harness): this project's own RTL arithmetic core
 * needs an independent golden reference to compare against, the same way
 * MH030's own tools/m68ksim.c compares its integer-core RTL against
 * Musashi's own m68k emulation. Musashi's m68kfpu.c (softfloat-based)
 * IS that reference for floating point -- reused here via a project-
 * local copy of tools/musashi/ (plan.md's own documented "own copy"
 * option, keeping this repo independent of MH030's own checkout).
 *
 * Approach: hand-assemble a real F-line "general instruction format"
 * opcode (Section 4.7.1/Table 4-11 -- the SAME command-word encoding
 * this project's own rtl/m68882_cir_pkg.sv::cmd_opclass/cmd_rx/cmd_ry/
 * cmd_ext functions decode, and the same helper every testbench's own
 * cmd_word() function already builds), poke FP0 (source)/FP1 (dest)
 * directly into Musashi's own m68ki_cpu.fpr[] before execution, run
 * exactly one instruction through the real m68k_execute() dispatch
 * path (not a hand-picked internal function -- this exercises Musashi's
 * own real opcode decode, not just its arithmetic core), then read the
 * result back the same way.
 *
 * floatx80 (Musashi's softfloat.h, {bits16 high; bits64 low;}) maps
 * DIRECTLY onto this project's own extended-precision internal format
 * (Table 3-3: sign(1)/exp(15)/reserved(16,always 0)/mantissa(64)) --
 * ext96 = {high[15:0], 16'h0, low[63:0]}. No conversion logic needed
 * beyond that zero-insertion, confirmed directly from the struct
 * layout, not assumed.
 *
 * Vector file format, one test per line (whitespace-separated hex):
 *   <ext_2hex> <rx_high_4hex> <rx_low_16hex> <ry_high_4hex> <ry_low_16hex> <round_1hex>
 * ext is the Table 4-13 extension-field opcode this project already
 * uses (22=FADD,28=FSUB,23=FMUL,20=FDIV,04=FSQRT,18=FABS,1A=FNEG,
 * 38=FCMP,3A=FTST). rx is always the source operand (loaded into FP0),
 * ry is always the pre-existing destination operand (loaded into FP1,
 * matching FCMP's own "compares against the current FPn contents"
 * semantics and giving FADD/FSUB/FMUL/FDIV a real, non-zero starting
 * destination to exercise). round: 0=Nearest,1=Zero,2=-Inf,3=+Inf
 * (FPCR bits[5:4], the same round_mode_t encoding this project's own
 * RTL uses).
 *
 * Output, one line per input line:
 *   <result_high_4hex> <result_low_16hex> <fpsr_8hex>
 *
 * Usage: ./tools/musashi_fpu_ref vectors.txt > results.txt
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "m68k.h"
#include "m68kcpu.h" /* pulls in softfloat/softfloat.h itself, incl. float_rounding_mode */

#define MEM_WORDS 64
static uint32_t g_mem[MEM_WORDS];

static unsigned int g_fc = 0;
static void fc_callback(unsigned int fc) { g_fc = fc; }

static uint32_t mem_r32(uint32_t a) { return g_mem[(a >> 2) & (MEM_WORDS - 1)]; }
static uint16_t mem_r16(uint32_t a) { uint32_t w = mem_r32(a); return (a & 2) ? (uint16_t)w : (uint16_t)(w >> 16); }
static uint8_t  mem_r8 (uint32_t a) { uint32_t w = mem_r32(a); return (uint8_t)(w >> ((3 - (a & 3)) * 8)); }
static void mem_w32(uint32_t a, uint32_t d) { g_mem[(a >> 2) & (MEM_WORDS - 1)] = d; }

unsigned int m68k_read_memory_8(unsigned int a)  { return mem_r8(a); }
unsigned int m68k_read_memory_16(unsigned int a) { return mem_r16(a); }
unsigned int m68k_read_memory_32(unsigned int a) { return mem_r32(a); }
unsigned int m68k_read_disassembler_8(unsigned int a)  { return mem_r8(a); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return mem_r16(a); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return mem_r32(a); }
void m68k_write_memory_8(unsigned int a, unsigned int v)  { (void)a; (void)v; }
void m68k_write_memory_16(unsigned int a, unsigned int v) { (void)a; (void)v; }
void m68k_write_memory_32(unsigned int a, unsigned int v) { (void)a; (void)v; }

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s vectors.txt\n", argv[0]);
        return 1;
    }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 1; }

    m68k_init();
    m68k_set_cpu_type(M68K_CPU_TYPE_68030);
    m68k_set_fc_callback(fc_callback);

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        unsigned int ext, rx_hi, ry_hi, round;
        unsigned long long rx_lo, ry_lo;
        if (sscanf(line, "%x %x %llx %x %llx %x",
                   &ext, &rx_hi, &rx_lo, &ry_hi, &ry_lo, &round) != 6) {
            if (line[0] == '\n' || line[0] == '#' || line[0] == '\0') continue;
            fprintf(stderr, "skipping malformed line: %s", line);
            continue;
        }

        for (int i = 0; i < MEM_WORDS; i++) g_mem[i] = 0x4E714E71u; /* NOP fill */

        /* First word: 0xF200 -- CpID field + EA mode/register = 0 (unused
         * for opclass 000, register-to-register -- no EA to evaluate).
         * Second word: the real Table 4-11 command word, opclass=000,
         * RX=0 (source, FP0), RY=1 (dest, FP1), EXTENSION=ext. */
        uint32_t cmd_word = (0u << 13) | (0u << 10) | (1u << 7) | (ext & 0x7Fu);
        g_mem[0] = (0xF200u << 16) | cmd_word;

        m68k_pulse_reset();
        m68k_set_reg(M68K_REG_SR, 0x2700); /* supervisor, IPL=7, no trace */
        m68k_set_reg(M68K_REG_PC, 0);

        m68ki_cpu.fpr[0].high = (uint16_t)rx_hi;
        m68ki_cpu.fpr[0].low  = (uint64_t)rx_lo;
        m68ki_cpu.fpr[1].high = (uint16_t)ry_hi;
        m68ki_cpu.fpr[1].low  = (uint64_t)ry_lo;
        m68ki_cpu.fpsr = 0;
        m68ki_cpu.fpcr = (round & 0x3u) << 4;
        m68ki_cpu.fpiar = 0;
        /* m68kfpu.c only ever updates softfloat's own float_rounding_mode
         * as a SIDE EFFECT of executing a real "FMOVE <ea>,FPCR"
         * instruction (fmove_fpcr(), m68kfpu.c) -- poking m68ki_cpu.fpcr
         * directly (as above, bypassing instruction execution) never
         * triggers that, so the arithmetic core would silently keep using
         * whatever rounding mode was last active otherwise. Confirmed via
         * a real cross-check: an early version of this harness produced
         * IDENTICAL FDIV 2.0/3.0 results across all 4 requested rounding
         * modes -- softfloat's own rounding-mode enum (float_round_
         * nearest_even=0/to_zero=1/down=2/up=3, softfloat.h) matches this
         * project's own round_mode_t/FPCR bits[5:4] encoding exactly, so
         * setting it directly here is correct, not a workaround. */
        float_rounding_mode = (int8_t)(round & 0x3u);

        m68k_execute(200);

        printf("%04x %016llx %08x\n",
               (unsigned)m68ki_cpu.fpr[1].high,
               (unsigned long long)m68ki_cpu.fpr[1].low,
               (unsigned)m68ki_cpu.fpsr);
    }

    fclose(f);
    return 0;
}
