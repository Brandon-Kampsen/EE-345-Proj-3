/* 
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 * Description: Peek/display harness for the DE10-Lite. We use this to inspect
 *              different datapath and IO values during single-step mode. The
 *              switches let us choose what part of the CPU to visualize, and
 *              the HEX displays show either the cycle count or the selected
 *              debug bus value.
 */

module peek_logic 
(

    // Basic board inputs
    input  logic [9:0]   SW,             // slide switches for selecting pages/modes
    input  logic         Step,           // one-pulse-per-press stepping
    input  logic         Peek,           // when held, we force PC display


    // Seven-segment HEX outputs
    output logic [6:0]   HEX5, HEX4,
    output logic [6:0]   HEX3, HEX2, HEX1, HEX0,


    // Datapath debug connections
    input  logic [31:0]  TapPC,
    input  logic [31:0]  TapInstruction,
    input  logic [31:0]  TapRD1,
    input  logic [31:0]  TapRD2,
    input  logic [31:0]  TapSrcB,
    input  logic [31:0]  TapALUResult,
    input  logic [31:0]  TapDataMemOut,
    input  logic [31:0]  TapResult,
    input  logic [31:0]  TapRE,
    input  logic [31:0]  TapRM,


    // IO register taps
    input  logic [31:0]  TapPort0Data,
    input  logic [31:0]  TapPort0Control,
    input  logic [31:0]  TapPort1Data,
    input  logic [31:0]  TapPort1Control,
    input  logic [7:0]   Port0_Out,
    input  logic [7:0]   Port1_Out,


    // Regfile peek interface
    output logic [3:0]   regfile_peek_sel,
    input  logic [31:0]  regfile_peek_data,


    // IO overrides (input-mode)
    output logic [7:0]   IO0InOverride,
    output logic [7:0]   IO1InOverride
);

    // SW8 toggles between showing data and overriding IO inputs
    logic mode_input;
    assign mode_input = SW[8];


    // Simple cycle counter (increments on each Step pulse)
    logic [7:0] cycle_count;
    always_ff @(posedge Step) begin
        if (SW[9]) begin
            cycle_count <= 8'h00;
        end 
        else begin
            cycle_count <= cycle_count + 8'h01;
        end
    end


    // IO override logic depending on mode_input
    always_comb begin
        IO0InOverride = 8'h00;
        IO1InOverride = 8'h00;

        if (mode_input) begin
            if (SW[7] == 1'b0) 
                IO0InOverride = {1'b0, SW[6:0]};
            else 
                IO1InOverride = {1'b0, SW[6:0]};
        end
    end


    // Lower bits of SW choose the peek/debug page
    logic [3:0] peek_select;
    assign peek_select      = SW[3:0];
    assign regfile_peek_sel = peek_select;

    logic [31:0] peek_Value;
    logic [15:0] peek_half;


    // Main debug value selection
    always_comb begin
        peek_Value = 32'h0000_0000;

        // Peek button forces PC view
        if (!Peek) begin
            peek_Value = TapPC;
        end 
        
        else if (SW[4] == 1'b0) begin
            peek_Value = regfile_peek_data;
        end 
        
        else begin
            case (peek_select)
                4'h0: peek_Value = TapPC;
                4'h1: peek_Value = TapInstruction;
                4'h2: peek_Value = TapRD1;
                4'h3: peek_Value = TapRD2;
                4'h4: peek_Value = TapSrcB;
                4'h5: peek_Value = TapALUResult;
                4'h6: peek_Value = TapDataMemOut;
                4'h7: peek_Value = TapResult;
                4'hC: peek_Value = TapRE;
                4'hD: peek_Value = TapALUResult;
                4'hE: peek_Value = TapRM;
                4'hF: peek_Value = TapResult;
                default: ;
            endcase
        end
    end


    // Choose upper/lower 16 bits
    assign peek_half = (SW[5] == 1'b0) ? peek_Value[15:0] :
                                        peek_Value[31:16];


    // Nibbles for HEX display
    logic [3:0] hex5_nibble, hex4_nibble;
    logic [3:0] hex3_nibble, hex2_nibble, hex1_nibble, hex0_nibble;

    assign hex5_nibble = cycle_count[7:4];
    assign hex4_nibble = cycle_count[3:0];


    // Default HEX nibble assignment
    always_comb begin
        hex3_nibble = peek_half[15:12];
        hex2_nibble = peek_half[11:8];
        hex1_nibble = peek_half[7:4];
        hex0_nibble = peek_half[3:0];

        // IO composite display pages
        if (SW[4] && !Peek) begin
            unique case (peek_select)

                4'h8: begin
                    hex3_nibble = TapPort0Control[7:4];
                    hex2_nibble = TapPort0Control[3:0];
                    hex1_nibble = TapPort0Data[7:4];
                    hex0_nibble = TapPort0Data[3:0];
                end

                4'h9: begin
                    hex3_nibble = TapPort1Control[7:4];
                    hex2_nibble = TapPort1Control[3:0];
                    hex1_nibble = TapPort1Data[7:4];
                    hex0_nibble = TapPort1Data[3:0];
                end

                4'hA: begin
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port0_Out[7:4];
                    hex0_nibble = Port0_Out[3:0];
                end

                4'hB: begin
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port1_Out[7:4];
                    hex0_nibble = Port1_Out[3:0];
                end

                default: ;
            endcase
        end
    end


    // HEX display decoders
    Hex7 u_hex5 (.nibble(hex5_nibble), .Seg(HEX5));
    Hex7 u_hex4 (.nibble(hex4_nibble), .Seg(HEX4));
    Hex7 u_hex3 (.nibble(hex3_nibble), .Seg(HEX3));
    Hex7 u_hex2 (.nibble(hex2_nibble), .Seg(HEX2));
    Hex7 u_hex1 (.nibble(hex1_nibble), .Seg(HEX1));
    Hex7 u_hex0 (.nibble(hex0_nibble), .Seg(HEX0));

endmodule



/***
Written by: Devin Nowowiejski & Nathan Arrends
Date Written: 10/19/2025
Peek/display harness for DE10-Lite
***/

//Changed variable names, check Top Level Module 
// Fix comments and formatting 

/*
module peek_logic (
    input  logic [9:0]   SW,                // Slide switches
    input  logic         Step,              // Debounced step pulse (one pulse per press)
    input  logic         Peek,              // KEY1: while pressed, display PC
    output logic [6:0]   HEX5, HEX4,
    output logic [6:0]   HEX3, HEX2, HEX1, HEX0,

    // Debug/peek inputs from CPU/datapath
    input  logic [31:0]  TapPC,
    input  logic [31:0]  TapInstruction,
    input  logic [31:0]  TapRD1,
    input  logic [31:0]  TapRD2,
    input  logic [31:0]  TapSrcB,
    input  logic [31:0]  TapALUResult,
    input  logic [31:0]  TapDataMemOut,
    input  logic [31:0]  TapResult,
    // Added: Execute-stage extended immediate
    input  logic [31:0]  TapRE,
    // Additional M-stage datapath tap
    input  logic [31:0]  TapRM,

    // Memory-mapped IO register taps
    input  logic [31:0]  TapPort0Data,
    input  logic [31:0]  TapPort0Control,
    input  logic [31:0]  TapPort1Data,
    input  logic [31:0]  TapPort1Control,
    input  logic [7:0]   Port0_Out,
    input  logic [7:0]   Port1_Out,


    // Regfile peek 
    output logic [3:0]   regfile_peek_sel,
    input  logic [31:0]  regfile_peek_data,

    // IO inputs in input mode
    output logic [7:0]   IO0InOverride,
    output logic [7:0]   IO1InOverride
);

    logic mode_input;   // SW8: 1=input mode, 0=display mode
    assign mode_input = SW[8];

    // Cycle counter (one tick per step)
    logic [7:0] cycle_count;
    always_ff @(posedge Step) begin
        if (SW[9]) begin
            cycle_count <= 8'h00;
        end else begin
            cycle_count <= cycle_count + 8'h01;
        end
    end

    // IO input override in input mode
    always_comb begin
        IO0InOverride = 8'h00;
        IO1InOverride = 8'h00;
        if (mode_input) begin
            if (SW[7] == 1'b0) begin
                IO0InOverride = {1'b0, SW[6:0]};
            end else begin
                IO1InOverride = {1'b0, SW[6:0]};
            end
        end
    end

    // Regfile index for peek
    logic [3:0] peek_select;
    assign peek_select = SW[3:0];
    assign regfile_peek_sel = peek_select;

    logic [31:0] peek_Value;
    logic [15:0] peek_half;

    always_comb begin
        // Default
        peek_Value = 32'h0000_0000;

        // Show PC while Peek button is pressed; otherwise selected source
        if (!Peek) begin
            peek_Value = TapPC;
        end else if (SW[4] == 1'b0) begin
            // Regfile peek page
            peek_Value = regfile_peek_data;
        end else begin
            // Bus/ALU pages
            case (peekSelect)
                4'h0: peek_Value = TapPC;
                4'h1: peek_Value = TapInstruction;
                4'h2: peek_Value = TapRD1;
                4'h3: peek_Value = TapRD2;
                4'h4: peek_Value = TapSrcB;
                4'h5: peek_Value = TapALUResult;
                4'h6: peek_Value = TapDataMemOut;
                4'h7: peek_Value = TapResult;
                4'hC: peek_Value = TapRE;     // RE now shows ExtImmE (extended immediate)
                4'hD: peek_Value = TapALUResult;   // RZ (ALUResultM)
                4'hE: peek_Value = TapRM;  // RM (WriteDataM)
                4'hF: peek_Value = TapResult;      // RY (Writeback result)
                default: ;
            endcase
        end
    end

    // Upper/lower half select
    assign peek_half = (SW[5] == 1'b0) ? peek_Value[15:0] : peek_Value[31:16];

    //Prepare nibbles for seven-seg modules
    logic [3:0] hex5_nibble, hex4_nibble, hex3_nibble, hex2_nibble, hex1_nibble, hex0_nibble;

    // Cycle count on HEX5:HEX4
    assign hex5_nibble = cycle_count[7:4];
    assign hex4_nibble = cycle_count[3:0];

    // Drive HEX3..HEX0 nibbles
    always_comb begin
        // Defaults to normal peek half
        hex3_nibble = peek_half[15:12];
        hex2_nibble = half[11:8];
        hex1_nibble = half[7:4];
        hex0_nibble = half[3:0];

        // IO composite pages when in display mode and bus-peek is selected (SW4=1)
        // Do not override when Peek is pressed (PC forced display)
        if (SW[4] && !Peek) begin
           unique case (peek_select)
                4'h8: begin
                    // IO0: HEX3:HEX2 = Control[7:0], HEX1:HEX0 = Data[7:0]
                    hex3_nibble = TapPort0Control[7:4];
                    hex2_nibble = TapPort0Control[3:0];
                    hex1_nibble = TapPort0Data[7:4];
                    hex0_nibble = TapPort0Data[3:0];
                end
                4'h9: begin
                    // IO1: HEX3:HEX2 = Control[7:0], HEX1:HEX0 = Data[7:0]
                    hex3_nibble = TapPort1Control[7:4];
                    hex2_nibble = TapPort1Control[3:0];
                    hex1_nibble = TapPort1Data[7:4];
                    hex0_nibble = TapPort1Data[3:0];
                end
                4'hA: begin
                    // Actual IO0 output (low 8 bits) on HEX1:HEX0
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port0Out[7:4];
                    hex0_nibble = Port0Out[3:0];
                end
                4'hB: begin
                    // Actual IO1 output (low 8 bits) on HEX1:HEX0
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port1Out[7:4];
                    hex0_nibble = Port1Out[3:0];
                end
                default: ; // keep normal peek half
            endcase
        end
    end

    // Instantiate the seven-seg hex encoders using our nibble
    Hex7 u_hex5 (.nibble(hex5_nibble), .Seg(HEX5));
    Hex7 u_hex4 (.nibble(hex4_nibble), .Seg(HEX4));
    Hex7 u_hex3 (.nibble(hex3_nibble), .Seg(HEX3));
    Hex7 u_hex2 (.nibble(hex2_nibble), .Seg(HEX2));
    Hex7 u_hex1 (.nibble(hex1_nibble), .Seg(HEX1));
    Hex7 u_hex0 (.nibble(hex0_nibble), .Seg(HEX0));

endmodule


