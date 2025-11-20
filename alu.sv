/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-13-2025
Description: 3-bit ALU that performs add, or, sub, and, or move
***/

// change header, comments, formatting 
// Variable names updated, review instantiation 

module alu #(parameter N = 32)(
	input  logic [2:0]    aluControl,
	input  logic [N-1:0]  SrcA,
	input  logic [N-1:0]  SrcB,
	output logic [N-1:0]  aluResult,
	output logic [3:0]    aluFlags
);

	// Extended-width adders for proper carry/borrow
	logic [N:0] add_wide;
	logic [N:0] sub_wide;

	always_comb begin
		// Compute core ALU operation
		case (ALUControl)
			3'b000:   aluResult = SrcA + SrcB;   // ADD
			3'b001:   aluResult = SrcA - SrcB;   // SUB
			3'b010:   aluResult = SrcA & SrcB;   // AND
			3'b011:   aluResult = SrcA | SrcB;   // ORR
			3'b100:   aluResult = SrcB;          // MOV
			default:  aluResult = SrcA + SrcB;   // Default ADD
		endcase

		// Precompute add/sub with carry out
		add_wide = {1'b0, SrcA} + {1'b0, SrcB};
		sub_wide = {1'b0, SrcA} + {1'b0, ~SrcB} + {{N{1'b0}}, 1'b1}; // A + (~B) + 1

		// NZ flags
		aluFlags[3] = aluResult[N-1];            // N
		aluFlags[2] = (aluResult == '0);         // Z

		// C and V depend on operation (aluFlags = {N,Z,C,V})
		case (aluControl)
			3'b000: begin // ADD
				aluFlags[1] = add_wide[N]; // C = carry out
				aluFlags[0] = (~(SrcA[N-1] ^ SrcB[N-1])) & (SrcA[N-1] ^ aluResult[N-1]); // V
			end
			3'b001: begin // SUB  (A - B)
				aluFlags[1] = sub_wide [N]; // C = NOT borrow (carry out of A + ~B + 1)
				aluFlags[0] = (SrcA[N-1] ^ SrcB[N-1]) & (SrcA[N-1] ^ ALUResult[N-1]); // V
			end
			default: begin
				// For logical/move ops, C and V set to 0 here; flag write gating should control updates
				aluFlags[1] = 1'b0; // C
				aluFlags[0] = 1'b0; // V
			end
		endcase
	end


endmodule

