/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-29-2025
Description: Execute (E) to Memory (M) pipeline register.
			 Registers control and datapath signals for the memory stage.
***/

// Change header, comments, formatting
// Make sure instantiation matches in datapath 


module E_register (
	input  logic        clk,
	input  logic        flushE,   // flush E-stage into bubble when 1

	// E-stage inputs
	input  logic        MemtoRegE,
	input  logic        MemWriteE,
	input  logic        RegWriteE,
	input  logic        PCSrcE,
	input  logic [31:0] ALUResultE,
	input  logic [31:0] WriteDataE,
	input  logic [3:0]  WA3E,

	// M-stage outputs
	output logic        MemtoRegM,
	output logic        MemWriteM,
	output logic        RegWriteM,
	output logic        PCSrcM,
	output logic [31:0] ALUResultM,
	output logic [31:0] WriteDataM,
	output logic [3:0]  WA3M
);

	// Register on posedge to align all pipeline stages
	always_ff @(posedge clk) begin
		if (flushE) begin
			// Convert current E-stage into NOP in M-stage
			MemtoRegM  <= 1'b0;
			MemWriteM  <= 1'b0;
			RegWriteM  <= 1'b0;
			PCSrcM     <= 1'b0;
			ALUResultM <= 32'h0;
			WriteDataM <= 32'h0;
			WA3M       <= 4'h0;
		end else begin
			MemtoRegM  <= MemtoRegE;
			MemWriteM  <= MemWriteE;
			RegWriteM  <= RegWriteE;
			PCSrcM     <= PCSrcE;
			ALUResultM <= ALUResultE;
			WriteDataM <= WriteDataE;
			WA3M       <= WA3E;
		end
	end

endmodule
