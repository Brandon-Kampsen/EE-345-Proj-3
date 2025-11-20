/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-29-2025
Description: Memory (M) to Writeback (W) pipeline register.
			 Registers control and datapath signals for the WB stage.
***/


// Just update comments, no changes to variables 

module WB_register (
	input  logic        clk,

	// M-stage inputs
	input  logic        PCSrcM,
	input  logic        RegWriteM,
	input  logic        MemtoRegM,
	input  logic [31:0] ReadDataM,
	input  logic [31:0] ALUOutM,
	input  logic [3:0]  WA3M,

	// W-stage outputs
	output logic        PCSrcW,
	output logic        RegWriteW,
	output logic        MemtoRegW,
	output logic [31:0] ReadDataW,
	output logic [31:0] ALUOutW,
	output logic [3:0]  WA3W
);

	// Register on posedge to align all pipeline stages
	always_ff @(posedge clk) begin
		PCSrcW     <= PCSrcM;
		RegWriteW  <= RegWriteM;
		MemtoRegW  <= MemtoRegM;
		ReadDataW  <= ReadDataM;
		ALUOutW    <= ALUOutM;
		WA3W       <= WA3M;
	end

endmodule

