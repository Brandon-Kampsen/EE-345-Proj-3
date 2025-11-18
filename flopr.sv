/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-15-2025
Description: Parameterized D flip-flop with a reset
citation:
	Resettable Flip-Flop is from H&H textbook, HDL Example 7.9 on pg. 422
*/

module flopr #(parameter WIDTH = 8)
(
	input logic CLK,
	input logic reset,
	input logic [WIDTH-1:0] d,
	output logic [WIDTH-1:0] q
);
	
	always_ff @(posedge CLK, posedge reset) begin
		if (reset) q <= '0;
		else q <= d;
	end
			
endmodule