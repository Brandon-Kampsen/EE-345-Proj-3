/* 
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 * Description: 16-register, 3-ported register file used throughout our CPU.
 *              We read two registers combinationally and also support a 
 *              write-through behavior so the pipeline doesn't have to wait.
 *              R15 acts as our "PC+8" shadow, following ARM's semantics.
 */

module reg_file 
(
	input  logic        CLK,
	input  logic        WE3,         // write enable for WA3
	input  logic [3:0]  RA1,         // read address 1
	input  logic [3:0]  RA2,         // read address 2
	input  logic [3:0]  WA3,         // write address
	input  logic [31:0] WD3,         // data being written
	input  logic [31:0] R15,         // PC+8 shadow register
	output logic [31:0] RD1,         // read port 1 data
	output logic [31:0] RD2,         // read port 2 data

	// Simple peek interface for debugging internal registers
	input  logic [3:0]  peek_sel,
	output logic [31:0] peek_data
);

	// Our actual register file storage (R0–R14)
	logic [31:0] rf[14:0];


	// Write happens only on rising edge, and we never modify R15 internally.
	// The pipeline handles forwarding, so we just catch the new value here.
	always_ff @(posedge CLK) begin
		if (WE3 && WA3 != 4'b1111) begin // CHECK IF THIS HAS PROBLEMS
			rf[WA3] <= WD3;
		end
	end


	// RD1: read-through logic (makes W-stage writes visible immediately)
	// We check for R15 separately since it's not part of rf[] // CHECK FOR ISSUES HERE
	assign RD1 =
		(RA1 == 4'b1111)                           ? R15  :
		(WE3 && (WA3 == RA1) && (WA3 != 4'b1111))  ? WD3  :
		                                             rf[RA1];


	// RD2: exact same idea as RD1, but for the second read address
	assign RD2 =
		(RA2 == 4'b1111)                           ? R15  :
		(WE3 && (WA3 == RA2) && (WA3 != 4'b1111))  ? WD3  :
		                                             rf[RA2];


	// Peek port: lets us inspect registers for debugging on the DE10-Lite
	assign peek_data =
		(peek_sel == 4'b1111)                          ? R15  :
		(WE3 && (WA3 == peek_sel) && (WA3 != 4'b1111)) ? WD3  :
		                                                  rf[peek_sel];

endmodule


/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 11-13-2025
Description: 3 port with 16 registers register file
cite:
	Register File gotten from H&H, HDL Example 7.6 found on page 420
***/

// variable names changed, must go back to datapath to correct. 
// Update comments, formatting as well 

/* 
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

// three ported register file
// read two ports combinationally
// write third port on rising edge of clock
// register 15 reads PC+8 instead

always_ff @(posedge CLK) begin
	if (WE3 && WA3 != 4'b1111) rf[WA3] <= WD3;
end

// Combinational read ports with write-through bypass so W-stage writes are
// visible immediately to same-cycle reads of the same register
assign RD1 =
    (RA1 == 4'b1111)                            ? R15  :
    (WE3 && (WA3 == RA1) && (WA3 != 4'b1111))   ? WD3  :
                                                  rf[RA1];

assign RD2 =
    (RA2 == 4'b1111)                            ? R15  :
    (WE3 && (WA3 == RA2) && (WA3 != 4'b1111))   ? WD3  :
                                                  rf[RA2];

assign peek_data =
    (peek_sel == 4'b1111)                           ? R15  :
    (WE3 && (WA3 == peek_sel) && (WA3 != 4'b1111))  ? WD3  :
                                                      rf[peek_sel];


endmodule

