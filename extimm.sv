/*
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 10-18-2025
 *
 * File Description:
 *      This block handles immediate extension for different ARM-style formats.
 *      Based on ImmSrc, we either zero-extend a small immediate or sign-extend
 *      a branch offset. This logic matches the textbook version but is cleaned
 *      up for our pipeline.
 *
 * Cite:
 *      Immediate Extend block from Hennessy & Harris,
 *      HDL Example 7.8 (pg. 421).
 */

module extimm 
(

    // Lower 24 bits of the instruction (where the immediate lives)
    input  logic [23:0]  Instr,

    // Selects which extension style we use
    input  logic [1:0]   ImmSrc,

    // Output: fully extended 32-bit immediate
    output logic [31:0]  ExtImm
);

    // Immediate extension multiplexer (logic unchanged)
    always_comb
        case (ImmSrc)

            // Zero-extend 8-bit immediate
            2'b00: ExtImm = {24'b0, Instr[7:0]};

            // Zero-extend 12-bit immediate
            2'b01: ExtImm = {20'b0, Instr[11:0]};

            // Sign-extend 24-bit branch offset, then shift left by 2
            2'b10: ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00};

            // Default → undefined (matches original behavior)
            default: ExtImm = 32'bx;

        endcase

endmodule
