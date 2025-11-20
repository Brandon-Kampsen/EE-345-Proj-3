/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-13-2025
Description: Coordinates instruction decoding, execution, and memeory access
cite:
	Single Cycle Processor gotten from H&H, HDL Example 7.1 found on page 411
***/
// clean up formatting, comments, replace header
//  some variable names changed, double check instantiations 

module arm(
    input  logic        clk,
    input  logic        reset,
    output logic [31:0] PC,
    input  logic [31:0] InstrF,
    output logic [31:0] InstrD,
    output logic        MemWrite,
    output logic [31:0] ALUResult,
	 output logic [3:0] ALUFlagsE,
    output logic [31:0] WriteData,
    input  logic [31:0] ReadData,
    // Peek/debug passthrough from datapath
    input  logic [3:0]  reg_file_peek_sel,
    output logic [31:0] reg_file_peek_data,
    output logic [31:0] TapRD1,
    output logic [31:0] TapRD2,
    output logic [31:0] TapSrcB,
    output logic [31:0] TapResult,
    output logic [31:0] TapRE,
    output logic [31:0] TapALUResultE,
    output logic [31:0] TapWriteDataE,
    output logic [31:0] TapResultPreWB,
    // Hazard visibility to top-level
    output logic        stallF,
    output logic        flushD
);

    // Control signals (from ControlUnit + local branch/flag logic)
    logic        RegWrite, ALUSrc, MemtoReg;
    logic [1:0]  RegSrc, ImmSrc;
    logic [2:0]  ALUControl;
    logic        MemWriteD;
    logic        BranchD;
    logic [1:0]  FlagWriteD;

    // Field extraction for ControlUnit (ARM-like encoding)
    logic [1:0] Op;         // Instr[27:26]
    logic [5:0] Funct;      // {I[25], opcode[24:21], L/S or S bit [20]}
    assign Op    = InstrD[27:26];
    assign Funct = {InstrD[25], InstrD[24:21], InstrD[20]};

    // Instantiate ControlUnit for core decode (daTapath controls)
    ControlUnit cu(
        .Cond   (InstrD[31:28]),
        .Op     (Op),
        .Funct  (Funct),
        .Rd     (InstrD[15:12]),
        .MemtoReg(MemtoReg),
        .MemWrite(MemWriteD),
        .ALUControl(ALUControl),
        .ALUSrc (ALUSrc),
        .ImmSrc (ImmSrc),
        .RegWrite(RegWrite),
        .RegSrc (RegSrc)
    );

    // Branch detection (newest logic consistent with daTapath):
    // Treat Instr[27:25]==3'b101 as B/BL. Immediate is 24-bit <<2 via ImmSrc=2'b10.
    always_comb BranchD = (InstrD[27:25] == 3'b101);

    // Flag write enables: update NZCV on arithmetic ops when data-processing
    always_comb begin
        FlagWriteD = 2'b00;
        if (Op == 2'b00) begin // data-processing
            case (Funct[4:1])
                4'b0010: FlagWriteD = 2'b11; // SUB
                4'b0100: FlagWriteD = 2'b11; // ADD (if opcode mapping uses 0100)
                default: FlagWriteD = 2'b00; // default no flag writes
            endcase
        end
    end

    // Branch overrides: ensure proper immediate selection and ALU op for PC+offset
    logic        ALUSrc_clean;
    logic [1:0]  ImmSrc_clean;
    logic [2:0]  ALUControl_clean;
    always_comb begin
        ALUSrc_clean     = ALUSrc;
        ImmSrc_clean     = ImmSrc;
        ALUControl_clean = ALUControl;
        if (BranchD) begin
            ALUSrc_clean    = 1'b1;     // use immediate offset
            ImmSrc_clean    = 2'b10;    // branch 24-bit imm << 2
            ALUControl_clean = 3'b000;   // ADD PC + offset
        end
    end
	
    datapath dp(
        .clk(clk),
        .reset(reset),
        .RegSrc(RegSrc),
        .RegWrite(RegWrite),
        .MemWrite(MemWriteD),
        .ImmSrc(ImmSrc_final),
        .ALUSrc(ALUSrc_final),
        .ALUControl(ALUControl_final),
        .MemtoReg(MemtoReg),
        .Branch(BranchD),
        .FlagWrite(FlagWriteD),
        .PC(PC),
        .InstrF(InstrF),
        .InstrD(InstrD),
        .ALUResult(ALUResult),
        .WriteData(WriteData),
        .ReadData(ReadData),
        .reg_file_peek_sel(reg_file_peek_sel),
        .reg_file_peek_data(reg_file_peek_data),
        .TapRD1(TapRD1),
        .TapRD2(TapRD2),
        .TapSrcB(TapSrcB),
        .TapResult(TapResult),
        .TapExtImmE(TapExtImmE),
        .TapALUResultE(TapALUResultE),
        .TapWriteDataE(TapWriteDataE),
        .TapResultPreWB(TapResultPreWB),
        .MemWriteEOut(MemWrite),
        .stallF_final(stallF),
        .flushD_final(flushD),
		.ALUFlagsE(ALUFlagsE)
    );
	
endmodule
