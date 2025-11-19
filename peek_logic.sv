/*
Written by: Luke Johnson & Brandon Kampsen
Date Written: 10/18/2025
Description: Peek logic to allow visibility of internal values on DE10-Lite HEX displays.
*/

module peek_logic (
    // Board I/O
    input  logic [9:0]   SW,                // switches
    input  logic         Step,              // debounced step pulse (unused)
    input  logic         Peek,              // when low, force PC display
    output logic [6:0]   HEX5, HEX4,
    output logic [6:0]   HEX3, HEX2, HEX1, HEX0,

    // CPU / datapath taps
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
	
    // IO register taps that are mapped by memory
    input  logic [31:0]  TapPort0Data,
    input  logic [31:0]  TapPort0Control,
    input  logic [31:0]  TapPort1Data,
    input  logic [31:0]  TapPort1Control,
    input  logic [7:0]   Port0_Out,
    input  logic [7:0]   Port1_Out,

    // Regfile peek 
    output logic [3:0]   regfile_peek_sel,
    input  logic [31:0]  regfile_peek_data,

    // IO inputs that will override to memory
    output logic [7:0]   IO0InOverride,
    output logic [7:0]   IO1InOverride
);

    // For Sw8, 1 for input mode, 0 for display mode
    logic mode_input;   
    assign mode_input = SW[8];

    // Cycle counter which ticks once per step)
    logic [7:0] cycle_count;
    always_ff @(posedge Step) begin
        if (SW[9]) begin
            cycle_count <= 8'h00;
        end else begin
            cycle_count <= cycle_count + 8'h01;
        end
    end

    // IO override while using the input mode
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
        // Default peek value
        peek_Value= 32'h0000_0000;

        // When peek button not pressed, show the Program Counter, if not use different source
        if (!Peek) begin
            peek_Value = TapPC;
        end else if (SW[4] == 1'b0) begin
            // peek page for reg_file
            peek_Value = regfile_peek_data;
        end else begin
            // Bus and ALU peeks
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
				4'hD  peek_Value = TapALUResult; // RZ
				4'hE: peek_Value = TapRM;
				4'hF: peek_Value = TapResult; // RY Writeback 
                default: ;
            endcase
        end
    end

    // peek_half select for Upper or lower half
    assign peek_half = (SW[5] == 1'b0) ? peek_Value[15:0] : peek_Value[31:16];

    //Prepare nibbles for the seven-seg hex display
    logic [3:0] hex5_nibble, hex4_nibble, hex3_nibble, hex2_nibble, hex1_nibble, hex0_nibble;


    // Cycle count on HEX5:HEX4
    assign hex5_nibble = cycle_count[7:4];
    assign hex4_nibble = cycle_count[3:0];


    // Operate HEX3 through HEX0 nibble
    always_comb begin
        // Defaults to normal peek peek_half
        hex3_nibble = peek_half[15:12];
        hex2_nibble = peek_half[11:8];
        hex1_nibble = peek_half[7:4];
        hex0_nibble = peek_half[3:0];

        // IO peek when in display mode and while the bus peek is still selected, sw4 is up
        if (SW[4]) begin
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
				
                    // Real I/O 0 output displayed on Hex1 through Hex0
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port0_Out[7:4];
                    hex0_nibble = Port0_Out[3:0];
                end
                4'hB: begin
                    // Real I/O 1 output displayed on Hex1 through Hex0
                    hex3_nibble = 4'h0;
                    hex2_nibble = 4'h0;
                    hex1_nibble = Port1_Out[7:4];
                    hex0_nibble = Port1_Out[3:0];
                end
                default: ; // retain the normal peek (peek_half)
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
