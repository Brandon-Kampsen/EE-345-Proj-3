/*
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 *
 * File Description:
 *      This module is a parameterized D flip-flop with an asynchronous reset.
 *      We use it all over the pipeline whenever we need a small register to
 *      hold values between stages. When reset is high, q is forced to zero.
 *      Otherwise, q updates on the rising edge of CLK just like a normal DFF.
 *
 * cite:
 *      Based on the flip-flop example from Hennessy & Harris,
 *      HDL Example 7.9 (pg. 422).
 */

module flopr #(parameter WIDTH = 8) 
(

    // Flip-flop control signals
    input  logic               CLK,         // rising-edge clock
    input  logic               reset,       // async reset (clears q)

    // Data input
    input  logic [WIDTH-1:0]   d,           // value to latch

    // Data output
    output logic [WIDTH-1:0]   q            // stored output
);

    // Standard async-reset D flip-flop behavior
    always_ff @(posedge CLK, posedge reset) begin
        if (reset)
            q <= '0;          // clear register when reset is active
        else
            q <= d;           // latch input value otherwise
    end

endmodule
