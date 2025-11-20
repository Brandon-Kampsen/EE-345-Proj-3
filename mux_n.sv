/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-16-2025
Description: 2:1 Mux utilized throughout the datapath of the processor
cite:
	Multiplexer is from H&H textbook, HDL Example 7.11, pg 423
*/

module mux_n #(parameter WIDTH = 8)
(	input logic [WIDTH-1:0] d0, d1,
	input logic s,
	output logic [WIDTH-1:0] y
);
	
	assign y = s ? d1 : d0;
	
endmodule