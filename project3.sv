/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-5-25
Description: Top-level integrating debounced step clock, CPU, instruction/data memories, and PeekTop display.
***/

//Change header, formatting 


module project3(
	input logic KEY0, KEY1,
	input logic CLK50M,
	input logic [9:0] SW,
	output logic [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
	output logic [3:0] ALUFlagsE
);

	// Debounced step clock from KEY0 (used as CPU clock and PeekTop Step)
	logic CLK;
	// KEY0 is active-low; invert so the debounced output idles low and rises on press
	debouncer u_debouncer(
		.A_noisy(~KEY0),
		.CLK50M(CLK50M),
		.A(CLK)
	);

	logic [31:0] PC;
	logic [31:0] InstrF, InstrD;
	logic [31:0] ReadData;
	logic [31:0] WriteData;
	logic [31:0] DataAdr;
	logic MemWrite;
	

	// Data memory with I/O
	logic [7:0] port0_out, port1_out;
	logic [31:0] Tap_p0_data, Tap_p0_ctrl;
	logic [31:0] Tap_p1_data, Tap_p1_ctrl;

	// PeekTop IO override wires and datapath debug
	logic [7:0] io0_in_override, io1_in_override;
	logic [3:0] regfile_peek_sel;
	logic [31:0] regfile_peek_data;
	logic [31:0] Tap_rd1, Tap_rd2, Tap_srcb, Tap_result;
	logic [31:0] Tap_extimmE;
	logic [31:0] Tap_alu_e;
	logic [31:0] Tap_wdata_e;
	logic [31:0] Tap_result_prewb;

	// Instantiate processor and memories (book-style)
	// Debounced peek button (KEY1)
	logic KEY1_DB;
	debouncer u_debouncer_peek(
		.A_noisy(KEY1),
		.CLK50M(CLK50M),
		.A(KEY1_DB)
	);

	arm arm(
		.clk(CLK),
		.reset(SW[9]),
		.PC(PC),
		.InstrF(InstrF),
		.InstrD(InstrD),
		.MemWrite(MemWrite),
		.ALUResult(DataAdr),
		.WriteData(WriteData),
		.ReadData(ReadData),
		// Peek/debug
		.regfile_peek_sel(regfile_peek_sel),
		.regfile_peek_data(regfile_peek_data),
		.TapRD1(Tap_rd1),
		.TapRD2(Tap_rd2),
		.TapSrcB(Tap_srcb),
		.TapResult(Tap_result),
		.TapExtImmE(Tap_extimmE),
		.TapALUResultE(Tap_alu_e),
		.TapWriteDataE(Tap_wdata_e),
		.TapResultPreWB(Tap_result_prewb),
		.stallF(stallF_cpu),
		.flushD(flushD_cpu),
		.ALUFlagsE(ALUFlagsE)
	);
	

	imem imem(
	    .a(PC),
	    .rd(InstrF)
	);
	
	// Fetch register now integrated inside datapath via arm; remove external instance
	logic stallF_cpu, flushD_cpu; // exported from arm/datapath if needed for debug

	dmem_io dmem(
		// Core memory interface
		.CLK(CLK),
		.WE(MemWrite),
		.A(DataAdr),
		.WD(WriteData),
		.RD(ReadData),
		// Port 0
		.Port0In(io0_in_override),
		.Port0Out(port0_out),
		// Port 1
		.Port1In(io1_in_override),
		.Port1Out(port1_out),
		// Debug taps
		.TapPort0Data(Tap_p0_data),
		.TapPort0Control(Tap_p0_ctrl),
		.TapPort1Data(Tap_p1_data),
		.TapPort1Control(Tap_p1_ctrl)
	);

	// Hook up PeekTop (uses debounced CLK as Step)
	peek_logic peek(
		.SW(SW),
		.Step(CLK),
		.Peek(KEY1_DB),
		.HEX5(HEX5), .HEX4(HEX4), .HEX3(HEX3), .HEX2(HEX2), .HEX1(HEX1), .HEX0(HEX0),
		// Debug/peek from CPU
		.TapPC(PC),
		.TapInstruction(InstrF),
		.TapRD1(Tap_rd1),
		.TapRD2(Tap_rd2),
		.TapSrcB(Tap_srcb),
		.TapALUResult(Tap_alu_e),    // ALU result (E-stage input to execute_dff)
		.TapDataMemOut(ReadData),
		.TapResult(Tap_result_prewb),
		.TapWriteDataM(ReadData),
		.TapExtImmE(Tap_extimmE),
		// Memory-mapped IO taps
		.TapPort0Data(Tap_p0_data),
		.TapPort0Control(Tap_p0_ctrl),
		.TapPort1Data(Tap_p1_data),
		.TapPort1Control(Tap_p1_ctrl),
		.Port0Out(port0_out),
		.Port1Out(port1_out),
		.regfile_peek_sel(regfile_peek_sel),
		.regfile_peek_data(regfile_peek_data),
		// IO input overrides to MemoryWithIO
		.IO0InOverride(io0_in_override),
		.IO1InOverride(io1_in_override)
	);

endmodule
