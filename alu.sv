/*
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 *
 * Description:
 *      This is our basic 3-bit ALU module. It handles the core ops we use in
 *      the project (ADD, SUB, AND, ORR, MOV). We kept the structure simple and
 *      tried to match the H&H style while still making it easy to follow for
 *      debugging. The wider add/sub versions help us correctly capture the 
 *      carry/borrow bits for flag generation.
 */

module alu #(parameter N = 32)
(

    // ALU opcode from the controller (tells us which operation to run)
    input  logic [2:0]    aluControl,

    // Two operand inputs coming from the datapath
    input  logic [N-1:0]  SrcA,
    input  logic [N-1:0]  SrcB,

    // Outputs: main result + NZCV flags
    output logic [N-1:0]  aluResult,
    output logic [3:0]    aluFlags
);

    // Wider add/sub wires so we can capture carry out cleanly
    logic [N:0] add_wide;
    logic [N:0] sub_wide;


    always_comb begin

        // Core ALU operation selection
        case (aluControl)
            3'b000:   aluResult = SrcA + SrcB;   // ADD
            3'b001:   aluResult = SrcA - SrcB;   // SUB
            3'b010:   aluResult = SrcA & SrcB;   // AND
            3'b011:   aluResult = SrcA | SrcB;   // ORR
            3'b100:   aluResult = SrcB;          // MOV (just forward B)
            default:  aluResult = SrcA + SrcB;   // fall back to ADD
        endcase


        // Precompute wide add/sub for C and V flags
        add_wide = {1'b0, SrcA} + {1'b0, SrcB};
        sub_wide = {1'b0, SrcA} + {1'b0, ~SrcB} + {{N{1'b0}}, 1'b1};  
        // (A + ~B + 1) = A - B using two’s complement


        // N and Z flags (simple)
        aluFlags[3] = aluResult[N-1];             // N flag = sign bit
        aluFlags[2] = (aluResult == '0);          // Z flag = all bits zero


        // C and V depend on the ALU operation
        case (aluControl)

            // ADD operation flags
            3'b000: begin
                aluFlags[1] = add_wide[N];   // C = carry out of addition
                aluFlags[0] = (~(SrcA[N-1] ^ SrcB[N-1])) &
                               (SrcA[N-1] ^ aluResult[N-1]);  // V overflow
            end

            // SUB operation flags
            3'b001: begin
                aluFlags[1] = sub_wide[N];   // C = NOT borrow
                aluFlags[0] = (SrcA[N-1] ^ SrcB[N-1]) &
                               (SrcA[N-1] ^ ALUResult[N-1]);  // V overflow
            end

            // Logical + MOV ops → C and V don't matter here
            default: begin
                aluFlags[1] = 1'b0;
                aluFlags[0] = 1'b0;
            end
        endcase
    end

endmodule


/***
Written by:  Nathan A. & Devin Nowowiejski
Date Written: 10-13-2025
Description: 3-bit ALU that performs add, or, sub, and, or move
***/

// change header, comments, formatting 
// Variable names updated, review instantiation 
/*
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


