/* 
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 * File Description: Simple 3-input multiplexer we use in multiple parts of our SLP.
 *              This module just picks one of three data inputs based on a 
 *              2-bit select line. Modeled after common mux patterns from H&H.
 */

module mux3 #(parameter WIDTH = 8)
(
    input  logic [WIDTH-1:0] d0,   // first input option
    input  logic [WIDTH-1:0] d1,   // second input option
    input  logic [WIDTH-1:0] d2,   // third input option
    input  logic [1:0]       s,    // select line telling us which input to use
    output logic [WIDTH-1:0] y     // chosen output value
);

    // Basic multiplexer behavior:
    // depending on the 2-bit select signal, we route one of the three inputs to y
    always_comb begin
        case (s)
            2'b00: y = d0;       // when s=00 we choose input d0
            2'b01: y = d1;       // when s=01 we forward d1
            2'b10: y = d2;       // when s=10 we take input d2
            default: y = '0;     // default case just clears output
        endcase
    end

endmodule







/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-13-2025
Description: 3:1 Mux utilized throughout the SLP
cite:
	2:1 Multiplexer gotten from H&H, HDL Example 7.11 found on page 423
***/

/*
module mux3 #(parameter WIDTH = 8)(
	input  logic [WIDTH-1:0] d0,
	input  logic [WIDTH-1:0] d1,
	input  logic [WIDTH-1:0] d2,
	input  logic [1:0]       s,
	output logic [WIDTH-1:0] y
);
	
	always_comb begin
		case (s)
			2'b00: y = d0;
			2'b01: y = d1;
			2'b10: y = d2;
			default: y = '0;
		endcase
	end
	
endmodule


