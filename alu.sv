/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-17-2025
Description: ALU that performs ADD , OR , SUB , AND, MOV using a 3-bit ALU control signal 
*/

module alu #(parameter N = 32)(
    input logic [2:0] aluControl,
    input logic [N-1:0] SrcA, SrcB,
    output logic [N-1:0] aluResult,
	output logic [3:0] ALU_flags 
	
);

logic neg, zero, carry, overflow; 
 logic [31:0] condinvb; 
  logic [32:0] sum; 
 
  assign condinvb = aluControl[0] ? ~b : b; 
  assign sum = a + condinvb + aluControl[0];
  
always_comb begin
    case (aluControl)
        3'b000:   aluResult = SrcA + SrcB;           // ADD
        3'b001:   aluResult = SrcA - SrcB;           // SUB
        3'b010:   aluResult = SrcA & SrcB;           // AND
        3'b011:   aluResult = SrcA | SrcB;           // OR
		3'b100:   aluResult = SrcB;		             // MOV
        default: aluResult = SrcA + SrcB;			 // Default to Add them together
    endcase
end

 assign neg      = aluResult[31]; 
  assign zero     = (aluResult == 32'b0); 
  assign carry    = (aluControl[1] == 1'b0) & sum[32]; 
  assign overflow = (aluControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ 
aluControl[0]) &  
                                                (a[31] ^ sum[31]);  
  assign ALU_flags = {neg, zero, carry, overflow}; 
endmodule

endmodule
