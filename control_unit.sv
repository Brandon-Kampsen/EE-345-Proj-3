/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-5-25
Description: ARM-style control unit that decodes instruction

***/

// Change header, formatting, comments
// Review instantiation in datapath 

module control_unit (
	input  logic [3:0] Cond,
	input  logic [1:0] Op,
	input  logic [5:0] Funct,
	input  logic [3:0] Rd,

	output logic MemtoReg,
	output logic MemWrite,
	output logic [2:0] ALUControl,
	output logic ALUSrc,
	output logic [1:0] ImmSrc,
	output logic RegWrite,
	output logic [1:0] RegSrc
);

	always_comb begin
		// Default values (safe)
		MemtoReg   = 1'b0;
		MemWrite   = 1'b0;
		RegWrite   = 1'b0;
		ALUSrc     = 1'b0;
		ImmSrc     = 2'b00;
		RegSrc     = 2'b00;
		ALUControl = 3'b000; // default to ADD when ALUOp=0 (e.g., address calc)
        
		case (Op)
			2'b00: begin // Data-processing
				RegWrite = 1'b1;
				MemtoReg = 1'b0;
				MemWrite = 1'b0;
				RegSrc   = 2'b00;
				// ALUSrc based on immediate flag
				if (Funct[5] == 1'b0) begin
					ALUSrc = 1'b0;
					ImmSrc = 2'bxx; // don't care for reg operand
				end else begin
					ALUSrc = 1'b1;
					ImmSrc = 2'b00; // 8-bit immediate
				end

				// ALU operation decode (Funct[4:1] is opcode)
				case (Funct[4:1])
					4'b0010: ALUControl = 3'b001; // SUB
					4'b0000: ALUControl = 3'b010; // AND
					4'b1100: ALUControl = 3'b011; // ORR
					4'b1101: ALUControl = 3'b100; // MOV
					default: ALUControl = 3'b000; // ADD
				endcase
			end
			2'b01: begin // LDR/STR
				ALUSrc = 1'b1;     // Immediate offset
				ImmSrc = 2'b01;    // 12-bit unsigned immediate
				RegSrc = 2'b00;
				if (Funct[0] == 1'b0) begin // STR
					MemWrite = 1'b1;
					RegWrite = 1'b0;
					MemtoReg = 1'bx; // don't care on store
					RegSrc    = 2'b10; // Select RD on RA2 for store data
				end else begin // LDR
					MemWrite = 1'b0;
					RegWrite = 1'b1;
					MemtoReg = 1'b1;
				end
			end
			default: begin
				// Keep defaults (no-op control)
			end
		endcase
	end

endmodule

