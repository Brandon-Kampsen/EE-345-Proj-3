/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 11-3-2025
Description: Condiotional Unit for Pipeline Processor
cite: This Conditional Unit was found from H&H pg. 416, HDL Example 7.4
	Resettable Flip-Flop gotten from H&H, HDL Example 7.9 found on page 422
***/

// Add header 

module condunit(
	input  logic       CLK,
	input  logic       reset,
	input  logic [3:0] Cond,
	input  logic [3:0] ALUFlags,
	input  logic [1:0] FlagW,
	input  logic       PCS,
	input  logic       RegW,
	input  logic       MemW,
	output logic       PRSrc,
	output logic       RegWrite,
	output logic       MemWrite
);
	logic [1:0] FlagWrite;
	logic [3:0] Flags;
	logic CondEx;

	// Split flag registers (N,Z) and (C,V)
	flopenr #(2) flagreg_hi(
		.CLK(CLK), .reset(reset), .en(FlagWrite[1]),
		.d(ALUFlags[3:2]), .q(Flags[3:2])
	);
	flopenr #(2) flagreg_lo(
		.CLK(CLK), .reset(reset), .en(FlagWrite[0]),
		.d(ALUFlags[1:0]), .q(Flags[1:0])
	);

	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);

	assign FlagWrite = FlagW & {2{CondEx}};
	assign RegWrite  = RegW  & CondEx;
	assign MemWrite  = MemW  & CondEx;
	assign PRSrc     = PCS   & CondEx;
endmodule

