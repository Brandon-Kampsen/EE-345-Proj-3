/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-16-2025
Description: 2:1 Mux utilized throughout the datapath of the processor
cite:
	Multiplexer is from H&H textbook, HDL Example 7.11, pg 423
*/

module mux3 #(parameter WIDTH = 8)
(	input logic [WIDTH-1:0] d0, d1, d2,
	input logic [1:0] s,
	output logic [WIDTH-1:0] y
);
	
	assign y = s[1] ? d2 : (s[0] ? d1 : d0);
	
endmodule