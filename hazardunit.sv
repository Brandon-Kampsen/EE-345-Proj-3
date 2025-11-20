// Add header, fix formatting, comments, 
// Double check variables in datapath to make sure they match 




module hazardunit(
    // EX-stage info (current instruction in E)
    input  logic [3:0]  destRegE,      // Destination register in EX
    input  logic        loadE, // 1 if EX instruction is a load (MemtoRegE)
    input  logic [3:0]  rsE,      // Source register A in EX (for forwarding)
    input  logic [3:0]  rtE,      // Source register B in EX (for forwarding)

    // ID-stage info (instruction in D)
    input  logic [3:0]  rsD,      // Source register A in ID (for load-use stall)
    input  logic [3:0]  rtD,      // Source register B in ID (for load-use stall)

    // Downstream (M stage) producer
    input  logic        writeM,  // Writeback flag in EX/MEM
    input  logic [3:0]  rdM,     // EX/MEM dest register

    // Control transfer in M stage
    input  logic        branch_M,    // Branch in MEM

    output logic        stall,         // Stall F and D (for load-use)
    output logic        flush,         // Flush D/E on branch
    output logic [1:0]  forwardA,      // Forwarding selection for A
    output logic [1:0]  forwardB       // Forwarding selection for B
);

    // Forwarding logic (compare M-stage producer to E-stage consumers)
    // Encoding: 00 = original, 01 = from M (ALUResultM), 10 = from W (unused here)
    always_comb begin
        forwardA = 2'b00;
        forwardB = 2'b00;
        if (writeM && (rdM != 4'd0) && (rdM == rsE))
            forwardA = 2'b01;
        if (writeM && (rdM != 4'd0) && (rdM == rtE))
            forwardB = 2'b01;
        // W-stage forwarding not implemented in this unit; keep 10 unused.
    end

    // Load-use hazard detection and stalling logic (compare EX load dest vs ID sources)
    always_comb begin
        stall = 0;
        flush = 0;
        if (loadE && ((destRegE == rsD) || (destRegE == rtD))) begin
            stall = 1; // Stall pipeline when load-use hazard detected
        end
        // Branch hazard flush logic
        if (branch_M) begin
            flush = 1; // Flush pipeline on branch
        end
    end

endmodule
 
