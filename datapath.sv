/***
Written by:  Luke Johnson and Brandon Kampsen 
Date Written: 11-19-2025
Description: Datapath that implements instruction fetch, register file, ALU
				 and PC
cite:
	Datapath modified from H&H, HDL Example 7.5 found on page 418
***/

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


