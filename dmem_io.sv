/*
Written by: Luke Johnson & Brandon Kampsen 
Date Written: 10-17-25
Description: Handles memory-mapped data reads and writes


*/
module dmem_io (
	input  logic        CLK,        // Clk
	input  logic        WE,         // Write enable
	input  logic [31:0] A,          // Address in bytes
	input  logic [31:0] WD,         // Write data
	output logic [31:0] RD,         // Read data
    
	// I/O Port 0 signals
	input  logic [7:0]  Port0_In, 
	output logic [7:0]  Port0_Out,
    
	// I/O Port 1 signals
	input  logic [7:0]  Port1_In, 
	output logic [7:0]  Port1_Out,

	// Debug register taps for PeekTop to allow for internal visibility
	output logic [31:0] TapPort0Data,
	output logic [31:0] TapPort0Control,
	output logic [31:0] TapPort1Data,
	output logic [31:0] TapPort1Control
);

	// setup RAM
	logic [31:0] RAM [0:1023];					

	// setup Port Registers
	logic [31:0] port0DataReg;
	logic [31:0] port0CtrlReg;

	logic [31:0] port1DataReg;
	logic [31:0] port1CtrlReg;

	// Address select signals (decode I/O and RAM)
	logic port0DataSel, port0CtrlSel;
	logic port1DataSel, port1CtrlSel;
	logic ram_select;

	// Map out I/O address logic
	assign port0DataSel =  ( A == 32'h00002000);			// Selects Port 0 Data Register
	assign port0CtrlSel =  ( A == 32'h00002004);			// Selects Port 0 Control Register
	assign port1DataSel =  ( A == 32'h00003000);			// Selects Port 1 Data Register
	assign port1CtrlSel =  ( A == 32'h00003004);			// Selects Port 1 Control Register

	assign ram_select = (A[31:12] == 20'h00000);				// Selects RAM

	// Define control registers 
	// bit[7]: Enable Port 			active high
	// bit[1]: Data Direction 		active high 

	logic port0_en, port0Isout;
	logic port1_en, port1Isout;

	assign port0_en = port0CtrlReg[7];
	assign port0Isout = port0CtrlReg[1];
	assign port1_en = port1CtrlReg[7];
	assign port1Isout = port1CtrlReg[1];

	// Write Logic
	always_ff @(posedge CLK) begin
		if (WE) begin

			// Write to Port 0 Data Register
			if (port0DataSel) begin
				if (port0_en) begin
					port0DataReg[7:0]  <= WD[7:0];		//write lower 8 bits
					port0DataReg[31:8] <= 24'h0;
				end
			end

			// Write to Port 0 Control Register
			if (port0CtrlSel) begin
				port0CtrlReg <= WD;
			end

			// Write to Port 1 Data Register
			if (port1DataSel) begin
				if (port1_en) begin
					port1DataReg[7:0]  <= WD[7:0];		//write lower 8 bits
					port1DataReg[31:8] <= 24'h0;
				end
			end

			// Write to Port 1 Control Register
			if (port1CtrlSel) begin
				port1CtrlReg <= WD;
			end

			// Write RAM
			if (ram_select) begin
				RAM[A[11:2]] <= WD;
			end
		end
	end

	// Read Logic
	always_comb begin
		RD = 32'h00000000;

		// Port 0 Data Read
		if (port0DataSel) begin
			if (port0_en) begin
				if (port0Isout) begin
					RD = {24'h0, port0DataReg[7:0]};
				end else begin
					RD = {24'h0, Port0_In};
				end
			end else begin
				RD = 32'h00000000;
			end
		end

		// Port 0 Control Read
		// Return all 32 bits
		else if (port0CtrlSel) begin
			RD = port0CtrlReg;
		end

		// Port 1 Data Read
		else if (port1DataSel) begin
			if (port1_en) begin
				if (port1Isout) begin
					RD = {24'h0, port1DataReg[7:0]};
				end else begin
					RD = {24'h0, Port1_In};
				end
			end else begin
				RD = 32'h00000000;
			end
		end

		// Port 1 Control Read
		// Return all 32 bits
		else if (port1CtrlSel) begin
			RD = port1CtrlReg;
		end

		else if (ram_select) begin
			RD = RAM[A[11:2]];
		end

		// Address not mapped case 
		else begin
			RD = 32'h00000000;
		end
	end

	// Output Logic
	always_comb begin
		if (port0_en && port0Isout) begin
			Port0_Out = port0DataReg[7:0];
		end else begin
			Port0_Out = 8'h00;
		end
	end

	always_comb begin
		if (port1_en && port1Isout) begin
			Port1_Out = port1DataReg[7:0];
		end else begin
			Port1_Out = 8'h00;
		end
	end

	// Debug taps to allow for internal visibility
	assign TapPort0Data    = port0DataReg;
	assign TapPort0Control = port0CtrlReg;
	assign TapPort1Data    = port1DataReg;
	assign TapPort1Control = port1CtrlReg;

endmodule
