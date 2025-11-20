/*
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 *
 * Description:
 *      This module evaluates the ARM-style condition field using the NZCV flags.
 *      The output CondEx tells the pipeline whether the current instruction
 *      should actually execute or be suppressed. This matches the truth table
 *      shown in the H&H conditional execution example.
 *
 * Citation:
 *      Conditional Unit reference:
 *          - Hennessy & Harris, HDL Example 7.4 (pg. 416)
 *          - Resettable flip-flop context: HDL Example 7.9 (pg. 422)
 */

module condcheck 
(

    // 4-bit condition from the instruction
    input  logic [3:0]  Cond,

    // Current NZCV flags from the flag registers
    input  logic [3:0]  Flags,

    // Result of condition evaluation
    output logic        CondEx
);

    // Break out the flags for readability in the case table
    logic neg, zero, carry, overflow, ge;

    assign {neg, zero, carry, overflow} = Flags;

    // GE flag (greater/equal) is true when N == V
    assign ge = (neg == overflow);

    // ARM condition decoder — *logic exactly preserved*
    always_comb begin
        case (Cond)
            4'b0000: CondEx = zero;                  // EQ
            4'b0001: CondEx = ~zero;                 // NE
            4'b0010: CondEx = carry;                 // CS/HS
            4'b0011: CondEx = ~carry;                // CC/LO
            4'b0100: CondEx = neg;                   // MI
            4'b0101: CondEx = ~neg;                  // PL
            4'b0110: CondEx = overflow;              // VS
            4'b0111: CondEx = ~overflow;             // VC
            4'b1000: CondEx = carry & ~zero;         // HI
            4'b1001: CondEx = ~(carry & ~zero);      // LS
            4'b1010: CondEx = ge;                    // GE
            4'b1011: CondEx = ~ge;                   // LT
            4'b1100: CondEx = ~zero & ge;            // GT
            4'b1101: CondEx = ~(~zero & ge);         // LE
            4'b1110: CondEx = 1'b1;                  // AL (always)
            default: CondEx = 1'bx;                  // Undefined condition
        endcase
    end

endmodule
