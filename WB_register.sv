/* 
 * Written by: Luke Johnson and Brandon Kampsen
 * Date Written: 11-5-2025
 *
 * Description:
 *   Pipeline register that transfers all Memory-stage values
 *   into the Writeback stage. This keeps the W-stage signals
 *   fully synchronized with the clock so WB happens at the
 *   correct cycle boundary.
 *
 * note:
 *   All of these logic variables were given my Dr. Fourney in the project file
 */

module WB_register 
(
    //  this register updates on rising edge
    input  logic        clk,

    // Indicates branch resolution from memory stage
    input  logic        PCSrcM,

    // Standard register write-enable that continues into WB
    input  logic        RegWriteM,

    // Chooses whether writeback uses memory data or ALU result
    input  logic        MemtoRegM,

    // Data loaded from memory (if this was a load instruction)
    input  logic [31:0] ReadDataM,

    // ALU output forwarded into WB stage
    input  logic [31:0] ALUOutM,
	
    // Register index that will be written in WB stage
    input  logic [3:0]  WA3M,

    // Our outputs to the W stage:
    // Propagated branch control for the WB stage
    output logic        PCSrcW,

    // Write-enable for the register file in WB stage
    output logic        RegWriteW,

    // Controls the writeback data mux in WB
    output logic        MemtoRegW,

    // Memory data passed into writeback stage
    output logic [31:0] ReadDataW,

    // ALU result passed into writeback stage
    output logic [31:0] ALUOutW,

    // Register destination for the final writeback
    output logic [3:0]  WA3W
);

    // Clocked pipeline register:
    // Each rising edge captures the M-stage values so that
    // the writeback stage receives stable inputs for the
    // entire next cycle.
    always_ff @(posedge clk) begin
        PCSrcW     <= PCSrcM;
        RegWriteW  <= RegWriteM;
        MemtoRegW  <= MemtoRegM;
        ReadDataW  <= ReadDataM;
        ALUOutW    <= ALUOutM;
        WA3W       <= WA3M;
    end

endmodule

/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-29-2025
Description: Memory (M) to Writeback (W) pipeline register.
			 Registers control and datapath signals for the WB stage.
***/


// Just update comments, no changes to variables 
/*
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
*/



