/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 11-13-2025
Description: 3 port with 16 registers register file
cite:
	Register File gotten from H&H, HDL Example 7.6 found on page 420
***/

// variable names changed, must go back to datapath to correct. 
// Update comments, formatting as well 

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
