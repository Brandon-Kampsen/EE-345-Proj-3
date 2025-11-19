/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-18-2025
Description:  Reads 32-bit instruction from memfile.dat
citation:
	Instruction Memory module is from H&H texbook, HDL Example 7.15 on pg. 427
*/

module imem (
	input logic [31:0] a,
	output logic [31:0] rd
);
	
	logic [31:0] RAM[0:255];
	initial
	
	// read from memfile.dat file 
	$readmemh("memfile.dat",RAM);
	assign rd = RAM[a[31:2]]; // word aligned
	
endmodule