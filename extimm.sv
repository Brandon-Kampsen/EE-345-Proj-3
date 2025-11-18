/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-18-25
Description: This is the immediate extension block from the book
Citation:
	Immediate Extend block is found in H&H textbook, HDL example 7.8 on pg. 421
*/

module extimm (
	input logic [23:0] Instr,
	input logic [1:0] ImmSrc,
	output logic [31:0] ExtImm);
	
	always_comb
		case(ImmSrc)
	// unsigned immediate value (8-bit)
		2'b00: ExtImm = {24'b0, Instr[7:0]};
	// unsigned immediate value (12-bit)
		2'b01: ExtImm = {20'b0, Instr[11:0]};
	//  2's comp shifted value (24-bit)
		2'b10: ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00};
		
		 // Undefined extened memory
		default: ExtImm = 32'bx; 
	endcase
	
endmodule