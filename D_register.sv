/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-29-2025
Description: Pipeline register from Decode (D) to Execute (E) for control signals.
			 Carries all ControlUnit outputs except RegSrcD and ImmSrcD.
***/

// Change header, comments, formatting 



module D_register (
	input  logic        clk,
	input  logic        flushE,    // turn E-stage controls into NOP when 1
	input  logic        stallD,    // hold previous E-stage values when 1 (Decode stall)

	// D-stage controls from ControlUnit
	input  logic        MemtoRegD,
	input  logic        MemWriteD,
	input  logic        ALUSrcD,
	input  logic        RegWriteD,
	input  logic [2:0]  ALUControlD,
	// Future control signals
	input  logic        BranchD,
	input  logic [1:0]  FlagWriteD,
	input  logic        CondTrueD,   // condition evaluation result in D stage

	// D-stage datapath values
	input  logic [3:0]  RA1D,
	input  logic [3:0]  RA2D,
	input  logic [3:0]  WA3D,
	input  logic [31:0] RD1D,
	input  logic [31:0] RD2D,
	input  logic [31:0] ExtImmD,

	// E-stage registered controls
	output logic        MemtoRegE,
	output logic        MemWriteE,
	output logic        ALUSrcE,
	output logic        RegWriteE,
	output logic [2:0]  ALUControlE,
	output logic        BranchE,
	output logic [1:0]  FlagWriteE,
	output logic        CondTrueE,   // pipelined condition result

	// E-stage datapath values
	output logic [3:0]  RA1E,
	output logic [3:0]  RA2E,
	output logic [3:0]  WA3E,
	output logic [31:0] RD1E,
	output logic [31:0] RD2E,
	output logic [31:0] ExtImmE
);

	// On flushE, drive a NOP into E-stage. On stall, hold previous values.
	always_ff @(posedge clk) begin
		if (flushE) begin
			MemtoRegE   <= 1'b0;
			MemWriteE   <= 1'b0;
			ALUSrcE     <= 1'b0;
			RegWriteE   <= 1'b0;
			ALUControlE <= 3'b000; // NOP/ADD default
			BranchE     <= 1'b0;
			FlagWriteE  <= 2'b00;
			CondTrueE   <= 1'b0;
			RA1E		<= 4'b0;
			RA2E		<= 4'b0;
			WA3E		<= 4'b0;
			RD1E        <= 32'h0;
			RD2E        <= 32'h0;
			ExtImmE     <= 32'h0;
		end else if (!stallD) begin
			MemtoRegE   <= MemtoRegD;
			// Gate side-effects by condition result (conditional execution)
			MemWriteE   <= MemWriteD & CondTrueD;
			ALUSrcE     <= ALUSrcD;
			RegWriteE   <= RegWriteD & CondTrueD;
			ALUControlE <= ALUControlD;
			BranchE     <= BranchD;
			FlagWriteE  <= FlagWriteD;
			CondTrueE   <= CondTrueD;
			RA1E		<= RA1D;
			RA2E		<= RA2D;
			WA3E		<= WA3D;
			RD1E        <= RD1D;
			RD2E        <= RD2D;
			ExtImmE     <= ExtImmD;
		end
		// else: hold previous E-stage values on stall
	end

endmodule
