module datapath(
    input  logic        clk, reset, 
    input  logic [1:0]  RegSrcD, ImmSrcD, 
    input  logic        ALUSrcE, BranchTakenE, 
    input  logic [2:0]  ALUControlE,  
    input  logic        MemtoRegW, PCSrcW, RegWriteW, 
    input  logic        LinkD,              // NEW: BL in Decode
    output logic [31:0] PCF, 
    input  logic [31:0] InstrF, 
    output logic [31:0] InstrD, 
    output logic [31:0] ALUOutM, WriteDataM, 
    input  logic [31:0] ReadDataM, 
    output logic [3:0]  ALUFlagsE, 
    // hazard logic 
    output logic        Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, Match_12D_E, 
    input  logic [1:0]  ForwardAE, ForwardBE, 
    input  logic        StallF, StallD, FlushD
); 
                           
  logic [31:0] PCPlus4F, PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W;
  logic [31:0] PCnext1F, PCnextF; 
  logic [31:0] ExtImmD, rd1D, rd2D; 
  logic [31:0] rd1E, rd2E, ExtImmE, SrcAE, SrcBE, WriteDataE, ALUResultE; 
  logic [31:0] ReadDataW, ALUOutW, ResultW; 
  logic [31:0] PCPlus8D;  // pipelined PC+8 for BL
  logic [3:0]  RA1D, RA2D, RA1E, RA2E, WA3E, WA3M, WA3W, WA3D; 
  logic        Match_1D_E, Match_2D_E; 
  logic        LinkE, LinkM, LinkW;           // pipelined Link bit

  // ============================================================
  // Fetch stage 
  // ============================================================
  mux2 #(32) pcnextmux(PCPlus4F, ResultW, PCSrcW, PCnext1F); 
  mux2 #(32) branchmux(PCnext1F, ALUResultE, BranchTakenE, PCnextF); 
  flopenr #(32) pcreg(clk, reset, ~StallF, PCnextF, PCF); 
  adder   #(32) pcadd (PCF, 32'h4, PCPlus4F); 

  // pipeline PC+4 to D, then add 4 → PC+8
  flopr   #(32) pc4regD (clk, reset, PCPlus4F, PCPlus4D);
  adder   #(32) pc8add (PCPlus4D, 32'h4, PCPlus8D); // PC + 8 for Register 15

  // ============================================================
  // Decode Stage 
  // ============================================================
  flopenrc #(32) instrreg(clk, reset, ~StallD, FlushD, InstrF, InstrD); 

  // Source register selection (same as before)
  mux2 #(4) ra1mux(InstrD[19:16], 4'b1111,    RegSrcD[0], RA1D); 
  mux2 #(4) ra2mux(InstrD[3:0],   InstrD[15:12], RegSrcD[1], RA2D); 

  // Destination register selection:
  //   normal: Rd (InstrD[15:12])
  //   BL    : R14 (4'b1110)
  mux2 #(4) wa3mux(InstrD[15:12], 4'b1110, LinkD, WA3D);

  // Register file: R15 input is PC+8 for current instruction
  regfile rf(
      clk, RegWriteW,
      RA1D, RA2D,
      WA3W, ResultW, PCPlus8D,
      rd1D, rd2D
  );  

  extend ext(InstrD[23:0], ImmSrcD, ExtImmD); 

  // ============================================================
  // Execute Stage 
  // ============================================================
  flopr #(32) rd1reg (clk, reset, rd1D,     rd1E); 
  flopr #(32) rd2reg (clk, reset, rd2D,     rd2E); 
  flopr #(32) immreg (clk, reset, ExtImmD,  ExtImmE); 
  flopr #(4)  wa3ereg(clk, reset, WA3D,     WA3E); 
  flopr #(4)  ra1reg (clk, reset, RA1D,     RA1E); 
  flopr #(4)  ra2reg (clk, reset, RA2D,     RA2E); 
  flopr #(32) pc8ereg(clk, reset, PCPlus8D, PCPlus8E); // PC+8 into E
  flopr #(1)  linkereg(clk, reset, LinkD,   LinkE);    // Link bit into E

  mux3 #(32) byp1mux(rd1E, ResultW, ALUOutM, ForwardAE, SrcAE); 
  mux3 #(32) byp2mux(rd2E, ResultW, ALUOutM, ForwardBE, WriteDataE); 
  mux2 #(32) srcbmux(WriteDataE, ExtImmE, ALUSrcE, SrcBE); 

  alu alu(SrcAE, SrcBE, ALUControlE, ALUResultE, ALUFlagsE); 

  // ============================================================
  // Memory Stage 
  // ============================================================
  flopr #(32) aluresreg(clk, reset, ALUResultE, ALUOutM); 
  flopr #(32) wdreg     (clk, reset, WriteDataE, WriteDataM); 
  flopr #(4)  wa3mreg   (clk, reset, WA3E,       WA3M); 
  flopr #(32) pc8mreg   (clk, reset, PCPlus8E,   PCPlus8M);
  flopr #(1)  linkmreg  (clk, reset, LinkE,      LinkM);

  // ============================================================
  // Writeback Stage 
  // ============================================================
  flopr #(32) aluoutreg(clk, reset, ALUOutM, ALUOutW); 
  flopr #(32) rdreg    (clk, reset, ReadDataM, ReadDataW); 
  flopr #(4)  wa3wreg  (clk, reset, WA3M,      WA3W); 
  flopr #(1)  linkwreg (clk, reset, LinkM,     LinkW);

  // normal ALU/memory result
  logic [31:0] NormalResultW;
  mux2 #(32) resmux1(ALUOutW, ReadDataW, MemtoRegW, NormalResultW);

  
 mux2 #(32) resmux2(NormalResultW, PCPlus4W, LinkW, ResultW);


  // ============================================================
  // Hazard comparison 
  // ============================================================
  eqcmp #(4) m0(WA3M, RA1E, Match_1E_M); 
  eqcmp #(4) m1(WA3W, RA1E, Match_1E_W); 
  eqcmp #(4) m2(WA3M, RA2E, Match_2E_M); 
  eqcmp #(4) m3(WA3W, RA2E, Match_2E_W); 
  eqcmp #(4) m4a(WA3E, RA1D, Match_1D_E); 
  eqcmp #(4) m4b(WA3E, RA2D, Match_2D_E); 
  assign Match_12D_E = Match_1D_E | Match_2D_E; 

endmodule
