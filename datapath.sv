/* 
 * Written by:  Luke Johnson and Brandon Kampsen 
 * Date Written: 11-19-2025
 * File Description: Datapath that ties together instruction fetch, register file,
 *              ALU operations, and PC update for our pipelined ARM-style core.
 * cite:
 *   Structure is inspired by H&H, HDL Example 7.5 (page 418), but adapted
 *   for our project with extra pipeline stages and debug visibility.
 */ 

module datapath( 
	// Global clock and reset for the whole pipeline
	input  logic        clk, 
	input  logic        reset,

	// Control inputs coming from the main controller
	input  logic [1:0]  RegSrc,
	input  logic        RegWrite,
	input  logic        MemWrite,
	input  logic [1:0]  ImmSrc,
	input  logic        ALUSrc,
	input  logic [2:0]  ALUControl,
	input  logic        MemtoReg,

	// Branch-related controls from decode stage
	input  logic        Branch,          // indicates branch-type instruction in decode
	input  logic [1:0]  FlagWrite,       // which condition flags should be updated

	// Program counter going out to instruction memory
	output logic [31:0] PC,

	// ALU flags from execute stage
	output logic [3:0]  ALUFlagsE,

	// Instruction interface with fetch stage
	input  logic [31:0] InstrF,          // fetched instruction from instruction memory
	output logic [31:0] InstrD,          // instruction after the F->D pipeline register

	// Data path between CPU and data memory
	output logic [31:0] ALUResult, 
	output logic [31:0] WriteData,
	input  logic [31:0] ReadData,

    // Peek and debug taps to inspect the register file and pipeline
	input  logic [3:0]  reg_file_peek_sel,
	output logic [31:0] reg_file_peek_data,
	output logic [31:0] TapRD1,
	output logic [31:0] TapRD2,
	output logic [31:0] TapSrcB,
	output logic [31:0] TapResult,
	output logic [31:0] TapExtImmE,

	// One-cycle debug taps around the execute/mem boundary
	output logic [31:0] TapALUResultE,   // ALU output in E stage before E->M register
	output logic [31:0] TapWriteDataE,   // forwarded RD2 into E->M boundary
	output logic [31:0] TapResultPreWB,  // value that will be used at writeback

	// Memory write signal exposed to top-level from the pipeline
	output logic        MemWriteE_final,

	// Signals showing how the hazard unit is affecting F/D stages
	output logic        stallF_final,
	output logic        flushD_final
);

	// Core datapath signals for PC and general data movement
	logic [31:0] PCNext, PCPlus4, PCPlus8;
	logic [31:0] ExtImmD, SrcA, SrcB, Result;
	logic [31:0] WriteDataD;             // decode-stage RD2 coming out of register file
	logic [3:0]  RA1, RA2;               // register addresses used by the regfile

	
	// Next PC and hazard-cleaned control signals
	

	// Clean versions of PCSrc, stallF, and flushD used during reset
	logic        PCSrcM_clean;
	logic        stallF_clean;
	logic        flushD_clean;

	// While reset is active, hold these control values at known states
	// so we don't spread unknown (X) values through the pipeline
	always_comb begin
		if (reset) begin
			PCSrcM_clean = 1'b0;
			stallF_clean = 1'b0;
			flushD_clean = 1'b1;
		end 
		else begin
			PCSrcM_clean = PCSrcM;
			stallF_clean = stallF_final;
			flushD_clean = flushD_final;
		end
	end


	// Choose the next PC: either sequential PC+4 or the branch target from M stage
	mux_n #(32) pcmux 
	(
		.d0(PCPlus4),
		.d1(ALUResultM),
		.s(PCSrcM_clean),
		.y(PCNext)
	);

	// PC register: updates when fetch is not stalled, otherwise it holds the current PC
	flopenr #(32) pcreg 
	(
		.CLK(clk),
		.reset(reset),
		.en(~stallF_clean),
		.d(PCNext),
		.q(PC)
	);

	// First increment: compute PC + 4 for the next sequential instruction address
	adder pcadd1
	(
		.A(PC),
		.Y(PCPlus4)
	);

	// Second increment: generate PC + 8 (how ARM sees PC when read as R15)
	adder pcadd2(
		.A(PCPlus4),
		.Y(PCPlus8)
	);

	// E-stage view of PC+8, built from the pipelined PCPlus4E
	logic [31:0] PCPlus8E;
	assign PCPlus8E = PCPlus4E + 32'd4;


	// Fetch -> Decode pipeline register
	
	// Latch the fetched instruction into the decode stage,
	// respecting stalls and flushes requested by the hazard unit
	IF_register u_fetch_dff
	(
		.instF(InstrF),
		.stall(stallF_clean),
		.flushD(flushD_clean),
		.clk(clk),
		.instD(InstrD)
	);


	// Register file addressing logic:

	// RA1 is usually bits [19:16], but RegSrc[0] can force us to read R15 instead
	mux_n #(4) ra1mux 
	(
		.d0(InstrD[19:16]),
		.d1(4'b1111),
		.s(RegSrc[0]),
		.y(RA1)
	);

	// RA2 comes from either Rt ([3:0]) or Rd ([15:12]) depending on RegSrc[1]
	mux_n #(4) ra2mux 
	(
		.d0(InstrD[3:0]),
		.d1(InstrD[15:12]),
		.s(RegSrc[1]),
		.y(RA2)
	);
    
	// Writeback helpers for BL (branch with link):
	// RegWriteW_clean and WA3W_clean handle the special case where BL writes LR
	logic       RegWriteW_clean;
	logic [3:0] WA3W_clean;

	assign RegWriteW_clean = RegWriteW | LinkW;
	assign WA3W_clean      = LinkW ? 4'd14 : WA3W;   // if linking, destination is LR (R14)


	// Link signals to be tracked through the pipeline stages
	logic LinkD, LinkE, LinkM, LinkW;


	// Register file instance:

	// Register file:
	//  - RA1/RA2 select read registers
	//  - WA3W_clean and RegWriteW_clean select the writeback behavior
	//  - R15 input is PC+8 so reading R15 matches ARM semantics
	reg_file rf 
	(
		.CLK(clk),
	    .WE3(RegWriteW_clean),
		.RA1(RA1),
		.RA2(RA2),
	    .WA3(WA3W_clean),
        .WD3(Result),
		.R15(PCPlus8),
		.RD1(SrcA),
		.RD2(WriteDataD),
		.peek_sel(reg_file_peek_sel),
		.peek_data(reg_file_peek_data)
	);

	// Immediate extension: builds a 32-bit constant from the instruction and ImmSrc
	extimm Extend
	(
		.Instr(InstrD[23:0]),
		.ImmSrc(ImmSrc),
		.ExtImm(ExtImmD)
	);


	// Hazard and forwarding wires:

    // stallF/stallD/flushE are used to control the pipeline registers
    logic        stallF, stallD, flushE;

    // forwardA and forwardB control where ALU inputs come from (00=original, 01=M, 10=W)
    logic [1:0]  forwardA, forwardB;


	// ALU input selection with forwarding

	// ALUResultE_int holds the E-stage ALU output before it is registered
	logic [31:0] ALUResultE_int;

	// Forwarded versions of the E-stage register data
	logic [31:0] srcA_fwd, srcB_fwd;

	
	// Decide which source A and B to use based on forwarding signals
	always_comb begin
		srcA_fwd = (forwardA == 2'b01) ? ALUResultM :
		           (forwardA == 2'b10) ? Result     :
		                                 RD1E;

		srcB_fwd = (forwardB == 2'b01) ? ALUResultM :
		           (forwardB == 2'b10) ? Result     :
		                                 RD2E;
	end

	// ALU B input is either a forwarded register value or the extended immediate
	mux_n #(32) srcbmux 
	(
		.d0(srcB_fwd),
		.d1(ExtImmE),
		.s(ALUSrcE),
		.y(SrcB)
	);

	// When executing branches, we use PC+8 as the ALU A input; otherwise we use the register
	logic [31:0] SrcA_ALU;
	assign SrcA_ALU = BranchE ? PCPlus8E : srcA_fwd;


	// Actual ALU computation using the E-stage control bits and operands
	alu #(32) alu_inst 
	(
		.aluControl(ALUControlE),
		.SrcA(SrcA_ALU),
		.SrcB(SrcB),
		.aluResult(ALUResultE_int),
		.aluFlags(ALUFlagsE)
	);


	
	// Decode -> Execute pipeline registers:

	// Control signals as they appear in execute stage
	logic        MemtoRegE, MemWriteE, ALUSrcE, RegWriteE;
	logic [2:0]  ALUControlE;
	logic        BranchE;
	logic [1:0]  FlagWriteE;
	logic        CondTrueE;

	// Operand and address signals at execute stage
	logic [31:0] RD1E, RD2E, ExtImmE;
	logic [3:0]  RA1E, RA2E, WA3E;


	// Condition evaluation in decode stage:

	// Flag storage and forwarded flags for condition checking
	logic [3:0] Register_flags;     // flags stored from the last flag-setting instruction
	logic [3:0] Cond_flags;         // flags that will actually be used to test condition
	logic       CondTrueD;          // result of the condition check at decode

	
	// If the current E-stage instruction will update flags, use those;
	// otherwise rely on the stored Register_flags value
	always_comb begin
		Cond_flags = Register_flags;
		if (FlagWriteE) 
			Cond_flags = ALUFlagsE; // speculative: assume these flags will be committed
	end

	// Evaluate the condition field of the instruction using the selected flags
	condcheck u_condcheck
	(
		.Cond(InstrD[31:28]),
		.Flags(Cond_flags),
		.CondEx(CondTrueD)
	);

	// Pipeline register from decode to execute stage
	D_register u_decode_dff 
	(
		.clk(clk),
		.flushE(flushE),
		.stallD(stallD),

		// D-stage control bits
		.MemtoRegD(MemtoReg),
		.MemWriteD(MemWrite),
		.ALUSrcD(ALUSrc),
		.RegWriteD(RegWrite),
		.ALUControlD(ALUControl),
		.BranchD(Branch),
		.FlagWriteD(FlagWrite),
		.CondTrueD(CondTrueD),

		// D-stage data and register addresses
		.RA1D(RA1),
		.RA2D(RA2),
		.RD1D(SrcA),
		.RD2D(WriteDataD),
		.ExtImmD(ExtImmD),
		.WA3D(InstrD[15:12]),

		// E-stage outputs
		.MemtoRegE(MemtoRegE),
		.MemWriteE(MemWriteE),
		.ALUSrcE(ALUSrcE),
		.RegWriteE(RegWriteE),
		.ALUControlE(ALUControlE),
		.BranchE(BranchE),
		.FlagWriteE(FlagWriteE),
		.CondTrueE(CondTrueE),
		.RA1E(RA1E),
		.RA2E(RA2E),
		.RD1E(RD1E),
		.RD2E(RD2E),
		.ExtImmE(ExtImmE),
		.WA3E(WA3E)
	);


	// Register index for writeback stage
	logic [3:0] WA3W;


	// Execute -> Memory pipeline registers

	// Control bits in memory stage
	logic        MemtoRegM, MemWriteM, RegWriteM, PCSrcM;

	// Data values in memory stage
	logic [31:0] ALUResultM, WriteDataM;
	logic [3:0]  WA3M;

	// Signals used in the final writeback stage
	logic        PCSrcW, RegWriteW, MemtoRegW;
	logic [31:0] ReadDataW, ALUOutW;


	// Flag register update at the end of execute

	// When FlagWriteE is active, capture ALUFlagsE into the stored flags register
	always_ff @(posedge clk or posedge reset) begin
		if (reset)
			Register_flags <= 4'h0;
		else if (FlagWriteE)
			Register_flags <= ALUFlagsE;
	end


	// Compute PCSrcE: only true if this is a branch and the condition check passed
	logic PCSrcE;
	assign PCSrcE = BranchE & CondTrueE;


	// Pipeline register from execute to memory stage
	E_register u_execute_dff 
	(
		.clk(clk),
		.flushE(flushE),

	    .MemtoRegE(MemtoRegE),
	    .MemWriteE(MemWriteE),
	    .RegWriteE(RegWriteE),
	    .PCSrcE(PCSrcE),
	    .ALUResultE(ALUResultEWire),
	    .WriteDataE(srcB_fwd),
	    .WA3E(WA3E),

	    .MemtoRegM(MemtoRegM),
	    .MemWriteM(MemWriteM),
	    .RegWriteM(RegWriteM),
	    .PCSrcM(PCSrcM),
	    .ALUResultM(ALUResultM),
	    .WriteDataM(WriteDataM),
	    .WA3M(WA3M)
    );


	// ----------------------------------------------------------------------
	// Hazard unit: forwarding, stalling, and flushing logic
	// ----------------------------------------------------------------------

	logic haz_stall, haz_flush;

	hazardunit hz
	(
		// Execute-stage info used to detect hazards and forwarding
		.destRegE(WA3E),
		.loadE(MemtoRegE),
		.rsE(RA1E),
		.rtE(RA2E),

		// Decode-stage register usage for load-use checks
		.rsD(RA1),
		.rtD(RA2),

		// Info about writes in the memory stage and branch control
		.writeM(RegWriteM),
		.rdM(WA3M),
		.branch_m(PCSrcM),

		// Hazard outputs: stall, flush, and forwarding selections
		.stall(haz_stall),
		.flush(haz_flush),
		.forwardA(forwardA),
		.forwardB(forwardB)
	);

	// When hz_stall is high, we hold both fetch and decode stages
	assign stallF = haz_stall;
	assign stallD = haz_stall;

	// Control hazards result in flushing decode and execute
	assign flushE = haz_flush;
	assign flushD = haz_flush;


	// Memory write signal seen by the data memory comes from M-stage MemWrite
	assign MemWriteEOut = MemWriteM;

	// Clean hazard outputs going back to the top-level for fetch and decode
	assign stallF_final = stallF_clean;
	assign flush_final = flushD_clean;


    // M-stage values going outward to the data memory interface
	assign WriteData = WriteDataM;
	assign ALUResult = ALUResultM;


	// Memory -> Writeback pipeline registers

    WB_register u_write_dff 
	(
	    .clk(clk),

	    // Inputs coming from memory stage
	    .PCSrcM(PCSrcM),
	    .RegWriteM(RegWriteM),
	    .MemtoRegM(MemtoRegM),
	    .ReadDataM(ReadData),
	    .ALUOutM(ALUResultM),
	    .WA3M(WA3M),

	    // Outputs into writeback stage
	    .PCSrcW(PCSrcW),
	    .RegWriteW(RegWriteW),
	    .MemtoRegW(MemtoRegW),
	    .ReadDataW(ReadDataW),
	    .ALUOutW(ALUOutW),
	    .WA3W(WA3W)
    );


	// Debug taps for inspecting what each stage is seeing:

	// Show the values feeding into the decode pipeline register
	assign TapRD1        = SrcA;           // first register file output at D stage
	assign TapRD2        = WriteDataD;     // second register file output at D stage

	// Show the current ALU B operand in the execute stage
	assign TapSrcB       = SrcB;

	// Show the final writeback value returned to the register file
	assign TapResult     = Result;

	// Show the extended immediate as seen in the decode stage
	assign TapExtImmE    = ExtImmD;

	// Show the E-stage ALU result before it is registered into M
	assign TapALUResultE = ALUResultE_int;

	// Show the forwarded RD2 value going into the E/M boundary
	assign TapWriteDataE = srcB_fwd;


	// Pre-WB result view: what the writeback mux will see next cycle

	// TapResultPreWB shows the M-stage candidate value for writeback
	case ({LinkM, MemtoRegM})
		2'b00: TapResultPreWB = ALUResultM; // normal ALU result
		2'b01: TapResultPreWB = ReadData;   // normal load
		default: TapResultPreWB = PCPlus4M; // link case uses PC+4
	endcase


	// Link (BL) support: LR <= PC+4 and then branch to target

	// Identify a BL-style instruction in decode
	// (ARM-like behavior: opcode[27:25] == 3'b101 and bit 24 set)
	assign LinkD = (InstrD[27:25] == 3'b101) && InstrD[24];

	// PC+4 pipelined alongside the link signal
	logic [31:0] PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W;

	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			LinkE    <= 1'b0;
			LinkM    <= 1'b0;
			LinkW    <= 1'b0;
			PCPlus4D <= 32'h0;
			PCPlus4E <= 32'h0;
			PCPlus4M <= 32'h0;
			PCPlus4W <= 32'h0;
		end 
		else begin
			// F -> D: capture PC+4 into the decode-stage copy
			PCPlus4D <= PCPlus4;

			// D -> E: move LinkD and PCPlus4D into execute stage
			LinkE    <= LinkD;
			PCPlus4E <= PCPlus4D;

			// E -> M: move link and PC+4 into memory stage
			LinkM    <= LinkE;
			PCPlus4M <= PCPlus4E;

			// M -> W: final link and PC+4 values for writeback
			LinkW    <= LinkM;
			PCPlus4W <= PCPlus4M;
		end
	end


	
	// Final writeback mux:

	// WB_select chooses between ALUOutW, ReadDataW, and PCPlus4W
	// When LinkW is active, we prioritize PCPlus4W for LR
	logic [1:0] WB_select;

	always_comb 
		WB_select = LinkW ? 2'b10 : {1'b0, MemtoRegW};

	// Three-way mux used to select the value written back into the register file
	mux_three #(32) ry_mux 
	(
		.d0(ALUOutW),
		.d1(ReadDataW),
		.d2(PCPlus4W),
		.s(WB_select),
		.y(Result)
	);

endmodule






















/***
Written by:  Luke Johnson and Brandon Kampsen 
Date Written: 11-19-2025
Description: Datapath that implements instruction fetch, register file, ALU
				 and PC
cite:
	Datapath modified from H&H, HDL Example 7.5 found on page 418
***/

/*
module datapath( 
	input logic clk, reset,
	input logic [1:0] RegSrc,
	input logic RegWrite,
	input logic MemWrite,
	input logic [1:0] ImmSrc,
	input logic ALUSrc,
	input logic [2:0] ALUControl,
	input logic MemtoReg,
	input logic Branch,               // branch control (D stage)
	input logic [1:0] FlagWrite,      // flag write enables (D stage)
	output logic [31:0] PC,
	output logic [3:0] ALUFlagsE,
	input logic [31:0] InstrF,              // instruction from fetch (imem)
	output logic [31:0] InstrD,             // decode stage instruction (after fetch_dff)
	output logic [31:0] ALUResult, WriteData,
	input logic [31:0] ReadData,
    // Peek and debug taps
	input logic [3:0] reg_file_peek_sel,
	output logic [31:0] reg_file_peek_data,
	output logic [31:0] TapRD1,
	output logic [31:0] TapRD2,
	output logic [31:0] TapSrcB,
	output logic [31:0] TapResult,
	output logic [31:0] TapExtImmE,
	// Additional immediate-cycle debug taps
	output logic [31:0] TapALUResultE,   // ALU output before E->M dff
	output logic [31:0] TapWriteDataE,   // WriteDataE (forwarded RD2) before E->M dff
	output logic [31:0] TapResultPreWB,  // Writeback mux value based on M-stage inputs
	// Expose E-stage MemWrite for memory interface
	output logic MemWriteE_final,
	// Hazard visibility for top-level fetch register
	output logic stallF_final,
	output logic flushD_final
);

	logic [31:0] PCNext, PCPlus4, PCPlus8;
	logic [31:0] ExtImmD, SrcA, SrcB, Result;
	logic [31:0] WriteDataD; // D-stage RD2 from Register_File
	logic [3:0] RA1, RA2;

	// Next PC logic
// During reset, force known control values to avoid X-propagation deadlock
logic        PCSrcM_clean;
logic        stallF_clean;
logic        flushD_clean;

always_comb begin
    if (reset) begin
        PCSrcM_clean = 1'b0;
        stallF_clean = 1'b0;
        flushD_clean = 1'b1;
    end else begin
        PCSrcM_clean = PCSrcM;
        stallF_clean = stallF_final;
        flushD_clean = flushD_final;
    end
end


	// Branch PC selection: choose between sequential PCPlus4 and branch target (ALUResultM)
	mux_n #(32) pcmux (
		.d0(PCPlus4),
		.d1(ALUResultM),
		.s(PCSrcM_clean),
		.y(PCNext)
	);
	// PC register with stall support (enable active when not stalling F stage)
	flopenr #(32) pcreg (
		.CLK(clk),
		.reset(reset),
		.en(~stallF_clean),
		.d(PCNext),
		.q(PC)
	);
	adder pcadd1(
		.A(PC),
		.Y(PCPlus4)
	);

	// Compute PC+8 (ARM-style PC read) from PC+4 (F stage)
	adder pcadd2(
		.A(PCPlus4),
		.Y(PCPlus8)
	);
	// Also compute E-stage PC+8 using pipelined PCPlus4E so branches use correct base
	logic [31:0] PCPlus8E;
	assign PCPlus8E = PCPlus4E + 32'd4;

    // Fetch -> Decode pipeline register for instruction
	fetch_dff u_fetch_dff(
		.instF(InstrF),
		.stall(stallF_clean),
		.flushD(flushD_clean),
		.clk(clk),
		.instD(InstrD)
	);

	// Register file logic (use InstrD)
	mux_n #(4) ra1mux (
		.d0(InstrD[19:16]),
		.d1(4'b1111),
		.s(RegSrc[0]),
		.y(RA1)
	);
	mux_n #(4) ra2mux (
		.d0(InstrD[3:0]),
		.d1(InstrD[15:12]),
		.s(RegSrc[1]),
		.y(RA2)
	);
    
	// Effective writeback controls (BL forces write to LR)
logic       RegWriteW_clean;
logic [3:0] WA3W_clean;

assign RegWriteW_clean = RegWriteW | LinkW;
assign WA3W_clean      = LinkW ? 4'd14 : WA3W;   // R14 (LR)


	// Predeclare link pipeline signals for use in writeback gating
	logic LinkD, LinkE, LinkM, LinkW;

	reg_file rf (
		.CLK(clk),
	    .WE3(RegWriteW_clean),
		.RA1(RA1),
		.RA2(RA2),
	    .WA3(WA3W_clean),
        .WD3(Result),
		.R15(PCPlus8),
		.RD1(SrcA),
		.RD2(WriteDataD),
		.peek_sel(reg_file_peek_sel),
		.peek_data(reg_file_peek_data)
	);
	// (writeback mux moved later to support BL link path)
	extimm Extend(
		.Instr(InstrD[23:0]),
		.ImmSrc(ImmSrc),
			.ExtImm(ExtImmD)
	);

    // Hazard / Forwarding wires
    logic        stallF, stallD, flushE;
    logic [1:0]  forwardA, forwardB; // 00=orig,01=M,10=W

	// ALU logic with forwarding
	logic [31:0] ALUResultE_int; // internal E-stage ALU result
	logic [31:0] srcA_fwd, srcB_fwd;

	
	// Forward muxes for SrcA and RD2E before ALUSrc selection
	always_comb begin
    srcA_fwd = (forwardA == 2'b01) ? ALUResultM :
               (forwardA == 2'b10) ? Result     :
                                     RD1E;

    srcB_fwd = (forwardB == 2'b01) ? ALUResultM :
               (forwardB == 2'b10) ? Result     :
                                     RD2E;
	end

	mux_n #(32) srcbmux (
		.d0(srcB_fwd),
		.d1(ExtImmE),
		.s(ALUSrcE),
		.y(SrcB)
	);
	// SrcA uses E-stage PC+8 for branches; otherwise forwarded register operand
	logic [31:0] SrcA_ALU;

	assign SrcA_ALU = BranchE ? PCPlus8E : srcA_fwd;


	alu #(32) alu_inst (
		.ALUControl(ALUControlE),
		.SrcA(SrcA_ALU),
		.SrcB(SrcB),
		.ALUResult(ALUResultE_int),
		.ALUFlags(ALUFlagsE)
	);

	// Decode->Execute pipeline register: register D-stage controls and operands
	logic        MemtoRegE, MemWriteE, ALUSrcE, RegWriteE;
	logic [2:0]  ALUControlE;
	logic        BranchE;
	logic [1:0]  FlagWriteE;
	logic        CondTrueE;
	logic [31:0] RD1E, RD2E, ExtImmE;
	logic [3:0]  RA1E, RA2E, WA3E;

	// Condition evaluation in Decode stage
	logic [3:0] Register_flags;            // stored flags from previous flag-setting instruction
	logic [3:0] Cond_flags;        // forwarded flags (accounts for flag update in same cycle)
	logic       CondTrueD;
	// Forward most recent ALU flags when current EX stage will write them, so the
	// very next instruction (in Decode) sees updated condition codes without a bubble.
	always_comb begin
		Cond_flags = Register_flags;
		if (FlagWriteE) Cond_flags = ALUFlagsE; // speculative forward
	end
	condcheck u_condcheck(
		.Cond(InstrD[31:28]),
		.Flags(Cond_flags),
		.CondEx(CondTrueD)
	);

	decode_dff u_decode_dff (
			.clk(clk),
			.flushE(flushE),
			.stallD(stallD),
			// D-stage controls
			.MemtoRegD(MemtoReg),
			.MemWriteD(MemWrite),
			.ALUSrcD(ALUSrc),
			.RegWriteD(RegWrite),
			.ALUControlD(ALUControl),
			.BranchD(Branch),
			.FlagWriteD(FlagWrite),
			.CondTrueD(CondTrueD),
			// D-stage datapath values
			.RA1D(RA1),
			.RA2D(RA2),
			.RD1D(SrcA),
			.RD2D(WriteDataD),
			.ExtImmD(ExtImmD),
			.WA3D(InstrD[15:12]),
			// E-stage outputs
			.MemtoRegE(MemtoRegE),
			.MemWriteE(MemWriteE),
			.ALUSrcE(ALUSrcE),
			.RegWriteE(RegWriteE),
			.ALUControlE(ALUControlE),
			.BranchE(BranchE),
			.FlagWriteE(FlagWriteE),
			.CondTrueE(CondTrueE),
			.RA1E(RA1E),
			.RA2E(RA2E),
			.RD1E(RD1E),
			.RD2E(RD2E),
			.ExtImmE(ExtImmE),
			.WA3E(WA3E)
	);


	logic [3:0] WA3W;

    // Execute -> Memory pipeline register
	logic        MemtoRegM, MemWriteM, RegWriteM, PCSrcM;
	logic [31:0] ALUResultM, WriteDataM;
	logic [3:0]  WA3M;
	// W-stage signals
	logic        PCSrcW, RegWriteW, MemtoRegW;
	logic [31:0] ReadDataW, ALUOutW;

    // Update flags register at end of Execute stage when instruction writes flags

	always_ff @(posedge clk or posedge reset) begin
    if (reset)
        Register_flags <= 4'h0;
    else if (FlagWriteE)
        Register_flags <= ALUFlagsE; // capture latest ALU flags
end


	// Determine PCSrcE (branch taken) from BranchE & CondTrueE
	logic PCSrcE;
	assign PCSrcE = BranchE & CondTrueE;

	execute_dff u_execute_dff (
		.clk(clk),
		.flushE(flushE),
	    .MemtoRegE(MemtoRegE),
	    .MemWriteE(MemWriteE),
	    .RegWriteE(RegWriteE),
	    .PCSrcE(PCSrcE),
	    .ALUResultE(ALUResultEWire),
	    .WriteDataE(srcB_fwd),
	    .WA3E(WA3E),
	    .MemtoRegM(MemtoRegM),
	    .MemWriteM(MemWriteM),
	    .RegWriteM(RegWriteM),
	    .PCSrcM(PCSrcM),
	    .ALUResultM(ALUResultM),
	    .WriteDataM(WriteDataM),
	    .WA3M(WA3M)
    );

	// Hazard unit (user-provided interface)
	logic hz_stall, hz_flush;
	hazard_unit hz(
		// EX stage (for forwarding)
		.id_ex_rd(WA3E),
		.id_ex_memread(MemtoRegE),
		.id_ex_rs(RA1E),
		.id_ex_rt(RA2E),
		// ID stage (for load-use stall)
		.if_id_rs(RA1),
		.if_id_rt(RA2),
		// Producers and control transfer
		.ex_mem_write(RegWriteM),
		.ex_mem_rd(WA3M),
		.mem_branch(PCSrcM),
		// Outputs
		.stall(hz_stall),
		.flush(hz_flush),
		.forwardA(forwardA),
		.forwardB(forwardB)
	);
	// Map to pipeline controls: stall F and D on hz_stall; flush E and D on hz_flush
	assign stallF = hz_stall;
	assign stallD = hz_stall;
	// Do NOT flush E on load-use stall; keep EX instruction (e.g., load) advancing.
	assign flushE = hz_flush; // only on control transfer (branch)
	assign flushD = hz_flush;

	// Drive outward-facing MemWrite from M-stage
	assign MemWriteEOut = MemWriteM;
	// Export hazard signals for top-level (stall fetch, flush decode)
	assign stallFOut = stallF_clean;
	assign flushDOut = flushD_clean;

    // Export M-stage values to top-level memory interface
	assign WriteData = WriteDataM;
	assign ALUResult = ALUResultM;

    // Memory -> Writeback pipeline register
    WB_register write_inst (
	    .clk(clk),
	    // M-stage inputs
	    .PCSrcM(PCSrcM),
	    .RegWriteM(RegWriteM),
	    .MemtoRegM(MemtoRegM),
	    .ReadDataM(ReadData),
	    .ALUOutM(ALUResultM),
	    .WA3M(WA3M),
	    // W-stage outputs
	    .PCSrcW(PCSrcW),
	    .RegWriteW(RegWriteW),
	    .MemtoRegW(MemtoRegW),
	    .ReadDataW(ReadDataW),
	    .ALUOutW(ALUOutW),
	    .WA3W(WA3W)
    );

	// Debug taps
	// Show inputs to stage DFFs (current-cycle values)
	assign TapRD1      = SrcA;          // D-stage regfile RD1 (input to decode_dff)
	assign TapRD2      = WriteDataD;    // D-stage regfile RD2 (input to decode_dff)
	assign TapSrcB     = SrcB;          // ALU operand B (E-stage combinational)
	assign TapResult   = Result;        // Writeback mux (W-stage comb); kept for RY legacy
	assign TapExtImmE  = ExtImmD;       // Use D-stage extended immediate (input to decode_dff)
	assign TapALUResultE = ALUResultE_int; // ALU output before execute_dff
	assign TapWriteDataE = srcB_fwd;       // Forwarded RD2 before execute_dff

	// Pre-WB result based on M-stage values (input to write_dff)
	case ({LinkM, MemtoRegM})
		2'b00: TapResultPreWB = ALUResultM;
		2'b01: TapResultPreWB = ReadData;
		default: TapResultPreWB = PCPlus4M;
	endcase

	// ------------------------------------------------------------
// Link (BL) support: write LR <= PC+4 and branch to target.
// Detect LinkD from instruction (ARM-like: opcode[27:25]==101 and bit[24]==1)
assign LinkD = (InstrD[27:25] == 3'b101) && InstrD[24];

// Pipeline PC+4 and Link alongside to align with WB
logic [31:0] PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        LinkE    <= 1'b0;
        LinkM    <= 1'b0;
        LinkW    <= 1'b0;
        PCPlus4D <= 32'h0;
        PCPlus4E <= 32'h0;
        PCPlus4M <= 32'h0;
        PCPlus4W <= 32'h0;
    end else begin
        // F->D
        PCPlus4D <= PCPlus4;

        // D->E
        LinkE    <= LinkD;
        PCPlus4E <= PCPlus4D;

        // E->M
        LinkM    <= LinkE;
        PCPlus4M <= PCPlus4E;

        // M->W
        LinkW    <= LinkM;
        PCPlus4W <= PCPlus4M;
    end
end


	// Writeback result mux: prefer link (PC+4) when LinkW asserted
	// 00 -> ALUOutW, 01 -> ReadDataW, 1x -> PCPlus4W
	logic [1:0] WB_select;
	always_comb WB_select = LinkW ? 2'b10 : {1'b0, MemtoRegW};
	mux3 #(32) ry_mux (
		.d0(ALUOutW),
		.d1(ReadDataW),
		.d2(PCPlus4W),
		.s(WB_select),
		.y(Result)
	);

endmodule





