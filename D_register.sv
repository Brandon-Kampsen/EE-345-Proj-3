/*
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 * Description:
 *      Pipeline register between Decode (D) and Execute (E). This stage mainly
 *      holds control/datapath values steady for EX, and supports flushing and
 *      stalling depending on hazards or branch decisions.
 */

module D_register (

    // system clock for pipelining
    input   logic       clk,

    // flush request → inject NOP into EX
    input   logic       flushE,

    // stall request → freeze outputs, hold last values
    input   logic       stallD,

    // control: EX should read from memory on WB stage
    input   logic       MemtoRegD,

    // control: EX/MEM may attempt a store
    input   logic       MemWriteD,

    // control: selects between reg/immediate for ALU B input
    input   logic       ALUSrcD,

    // control: register write enable coming from decode
    input   logic       RegWriteD,

    // control: ALU operation selected by controller
    input   logic [2:0] ALUControlD,

    // control: branch flag for next stage
    input   logic       BranchD,

    // control: ALU flag update mask for condition codes
    input   logic [1:0] FlagWriteD,

    // decode-stage condition check result
    input   logic       CondTrueD,

    // datapath: register source A index
    input   logic [3:0] RA1D,

    // datapath: register source B index
    input   logic [3:0] RA2D,

    // datapath: destination register number
    input   logic [3:0] WA3D,

    // datapath: read value from register A
    input   logic [31:0] RD1D,

    // datapath: read value from register B
    input   logic [31:0] RD2D,

    // datapath: extended immediate from decode
    input   logic [31:0] ExtImmD,


    // EX-stage: MemtoReg after pipelining
    output  logic       MemtoRegE,

    // EX-stage: MemWrite after pipelining (gated by condition)
    output  logic       MemWriteE,

    // EX-stage: ALU source select after pipelining
    output  logic       ALUSrcE,

    // EX-stage: RegWrite after pipelining (gated by condition)
    output  logic       RegWriteE,

    // EX-stage: ALU control bits stored for execution
    output  logic [2:0] ALUControlE,

    // EX-stage: branch flag
    output  logic       BranchE,

    // EX-stage: which flags to update
    output  logic [1:0] FlagWriteE,

    // EX-stage: condition result passed forward
    output  logic       CondTrueE,


    // EX-stage: RA1 index for operand A
    output  logic [3:0] RA1E,

    // EX-stage: RA2 index for operand B
    output  logic [3:0] RA2E,

    // EX-stage: destination register number
    output  logic [3:0] WA3E,

    // EX-stage: forwarded RD1 value
    output  logic [31:0] RD1E,

    // EX-stage: forwarded RD2 value
    output  logic [31:0] RD2E,

    // EX-stage: extended immediate
    output  logic [31:0] ExtImmE
);

    // pipeline update logic — flush overrides, stall freezes, otherwise normal write
    always_ff @(posedge clk) begin

        // flushing inserts a bubble/NOP into EX
        if (flushE) begin
            MemtoRegE   <= 1'b0; // pipelined MemtoReg bit for EX/MEM
            MemWriteE   <= 1'b0; // pipelined memory-write control

			
            ALUSrcE     <= 1'b0; // pipelined ALUSrc into execute
            RegWriteE   <= 1'b0;
            ALUControlE <= 3'b000; // ALU op code as seen in the execute stage
            BranchE     <= 1'b0;
            FlagWriteE  <= 2'b00; // which flags EX should update
            CondTrueE   <= 1'b0;

            RA1E        <= 4'b0; // source A register index passed into EX
            RA2E        <= 4'b0; // source B register index passed into EX

			
            WA3E        <= 4'b0; // destination register forwarded to later stages
			
            RD1E        <= 32'h0; // operand A value latched for EX
            RD2E        <= 32'h0; // operand B value latched for EX
			
            ExtImmE     <= 32'h0;

        // no stall → normal pipeline advance
        end else if (!stallD) begin
            MemtoRegE   <= MemtoRegD;
            MemWriteE   <= MemWriteD & CondTrueD; // pipelined memory-write control

			
            ALUSrcE     <= ALUSrcD; // pipelined ALUSrc into execute
            RegWriteE   <= RegWriteD & CondTrueD;

			
            ALUControlE <= ALUControlD; // ALU op code as seen in the execute stage
            BranchE     <= BranchD;

			
            FlagWriteE  <= FlagWriteD; // which flags EX should update
            CondTrueE   <= CondTrueD;

            RA1E        <= RA1D; // source A register index passed into EX
            RA2E        <= RA2D; // source B register index passed into EX
			
            WA3E        <= WA3D;
			
            RD1E        <= RD1D;  // operand A value latched for EX
            RD2E        <= RD2D; // operand B value latched for EX

			
            ExtImmE     <= ExtImmD; // immediate forwarded to ALU in EX
        end
        // else → stallD is high, so we hold previous values
    end

endmodule



/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-29-2025
Description: Pipeline register from Decode (D) to Execute (E) for control signals.
			 Carries all ControlUnit outputs except RegSrcD and ImmSrcD.
***/

// Change header, comments, formatting 

/*

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
*/

