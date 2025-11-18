/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-16-2025
Description: 3 port with 16 registers register file
citation:
	Register File is from H&H textbook, HDL Example 7.6 on pg. 420
*/

module reg_file 
(	input logic CLK,
	input logic WE3,
	input logic [3:0] RA1, RA2, WA3,
	input logic [31:0] WD3, R15,
	output logic [31:0] RD1, RD2,
	// Peek
	input logic [3:0] peek_sel,
	output logic [31:0] peek_data
);

logic [31:0] rf[14:0];

// This is a 3 ported reg file
// Read two ports using combinational logic
// Reg 15 will read the PC+8 instead of PC+4

always_ff @(posedge CLK) begin
	if (WE3 && WA3 != 4'b1111) rf[WA3] <= WD3;
end

// Combinational ports set to read input
assign RD1 = (RA1 == 4'b1111) ? R15 : rf[RA1];
assign RD2 = (RA2 == 4'b1111) ? R15 : rf[RA2];

// Extern peek 
assign peek_data = (peek_sel == 4'b1111) ? R15 : rf[peek_sel];

endmodule
	