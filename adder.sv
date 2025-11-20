/*
 * Written by:  Luke Johnson & Brandon Kampsen
 * Date Written: 10-16-2025
 * File Description: Simple adder that increments the PC by 4 bits
 * cite:
 * 		Adder is from H&H textbook, HDL Example 7.7 on pg. 421
 */


module adder #(parameter int WIDTH = 32) 
(
	input  logic [WIDTH-1:0] A,
	output logic [WIDTH-1:0] Y
);
	assign Y = A + WIDTH'(32'd4); // implements width of the module(32) & adds 4 to incoming PC value
	

endmodule

