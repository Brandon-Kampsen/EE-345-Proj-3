
// Add header, comments, change formatting 
 
module IF_register (
	input  logic [31:0] instF,
	input  logic        stall,
	input  logic        flushD,
	input  logic        clk,
	output logic [31:0] instD
);

	always_ff @(posedge clk) begin
		if (flushD) instD <= 32'h0000_0000; // flush has priority
		else if (!stall) instD <= instF;    // update only when not stalled
		// when stalled: hold previous instD
	end

endmodule
