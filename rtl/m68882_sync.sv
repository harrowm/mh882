`timescale 1ns/1ps
`default_nettype none

// Generic 2-stage synchronizer.
//
// Every host-driven bus input (AS#, DS#, CS#, R/W, SIZE#, and the address
// bus) is genuinely asynchronous to this chip's own clk_4x domain -- there
// is no shared-clock guarantee between an MC68882 and whatever host CPU it
// is attached to (CLAUDE.md, "Every bus input must be synchronized": this
// project's own convention is broader than the sibling MH030 project's,
// which only synchronizes error/arbitration lines, since MH030 assumes a
// shared clock with its own host bus and this chip does not). Reused for
// every such input rather than hand-copied per signal.

module m68882_sync #(
    parameter int WIDTH = 1
) (
    input  logic             clk_4x,
    input  logic             rst_n,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);

    logic [WIDTH-1:0] meta_r;

    always_ff @(posedge clk_4x or negedge rst_n) begin
        if (!rst_n) begin
            meta_r <= '0;
            q      <= '0;
        end else begin
            meta_r <= d;
            q      <= meta_r;
        end
    end

endmodule
