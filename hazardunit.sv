/* 
 * Written by: Brandon Kampsen & Luke Johnson
 * Date Written: 11-19-2025
 * File Description: This hazard detection + lightweight forwarding unit helps our
 *              pipeline avoid load-use stalls and wrong-path execution on
 *              branches. We compare register dependencies across stages and
 *              decide when we need a stall, flush, or simple bypass.
 */

module hazardunit
(

    // EX-stage signals (instruction currently in the execute stage)
    input  logic [3:0]  destRegE,      // register EX will write back to
    input  logic        loadE,         // tells us EX is doing a load
    input  logic [3:0]  rsE,           // first EX operand
    input  logic [3:0]  rtE,           // second EX operand

    
    // ID-stage signals (instruction being decoded)
    input  logic [3:0]  rsD,           // decode operand A
    input  logic [3:0]  rtD,           // decode operand B

    
    // M-stage producer (most recent finished value)
    input  logic        writeM,        // M stage writes a register
    input  logic [3:0]  rdM,           // destination register in M

    // Branch resolution from M stage
    input  logic        branch_M,

    // Outputs telling the pipeline how to react
    output logic        stall,         // hold F and D one cycle
    output logic        flush,         // clear D/E when branch taken
    output logic [1:0]  forwardA,      // chosen source for EX operand A
    output logic [1:0]  forwardB       // chosen source for EX operand B
);

    // Basic forwarding logic:
    // If the M-stage just produced a register that EX needs immediately,
    // we simply forward that result instead of waiting.
    always_comb begin
        forwardA = 2'b00;
        forwardB = 2'b00;

        if (writeM && (rdM != 4'd0) && (rdM == rsE))
            forwardA = 2'b01;

        if (writeM && (rdM != 4'd0) && (rdM == rtE))
            forwardB = 2'b01;

        // Forwarding from W stage (2'b10) isn't part of our design here.
    end


    // Stall + flush control:
    // We check for load-use hazards and control hazards.
    // GO BACK TO THIS IF NOT WORKING
    always_comb begin
    stall = 0;
        flush = 0;

        // Load-use: EX is loading a register that decode immediately needs.
        if (loadE && ((destRegE == rsD) || (destRegE == rtD))) begin
            stall = 1; // set to stall pipeline
        end
        
         // Branch taken at M stage → wipe younger stages.
        if (branch_M) begin
            flush = 1; // flush the pipeline
        end
    end


endmodule





/*




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
 

