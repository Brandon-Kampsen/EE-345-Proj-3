/*
Written by:  Luke Johnson & Brandon Kampsen
Date Written: 10-18-25
Description: This is the flip flop register from the textbook 
Citation:
	Flopenr is found in H&H textbook, HDL example 7.9 on pg. 422
*/

module flopenr #(parameter WIDTH = 8) 
                (input  logic             clk, reset, en, 
                 input  logic [WIDTH-1:0] d,  
                 output logic [WIDTH-1:0] q); 
 
  always_ff @(posedge clk, posedge reset) begin 
    if (reset)   q <= 0; 
    else if (en) q <= d; 
	end 
endmodule 
 
