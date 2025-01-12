module HazardUnit(
    input [2:0] A,            // Source Register A
    input [2:0] B,            // Source Register B
    input [2:0] WB2,          // Write-Back Register in MEM stage
    input RegWriteE,          // Write-Back Enable in MEM stage
    input [2:0] WB3,          // Write-Back Register in WB stage
    input RegWriteM,          // Write-Back Enable in WB stage
    input BranchD,            // Branch Signal in Decode stage
    input ForSignalD,         // Forwarding Signal in Decode stage
    output reg [1:0] ForwardA,// Forwarding for Source A
    output reg [1:0] ForwardB,// Forwarding for Source B
    output reg Stall,         // Stall Fetch Stage
    input loadE, 
    input [2:0]WB1,              // Load Signal in MEM stage
    input RegWriteW,
    input LoadM
);

    // Forwarding Logic for Source Registers A and B
    always @(*) begin
        // Default values (no forwarding)
        
        ForwardA = 2'b00;
        ForwardB = 2'b00;

        // Forwarding for A
        if (A != 0) begin
            if (A == WB1 && RegWriteE) 
                ForwardA = 2'b01; // Forward from MEM stage
            else if ((A == WB2 && RegWriteM && !LoadM))
                ForwardA = 2'b10; // Forward from WB stage
            else if (A == WB2 && LoadM)
                ForwardA = 2'b11; // Forward from WB stage
        end
            

        // Forwarding for B
        if (B != 0) begin
            if (B == WB1 && RegWriteE) 
                ForwardB = 2'b01; // Forward from MEM stage
            else if (B == WB2 && RegWriteM)
                ForwardB = 2'b10; // Forward from WB stage
            else if (B == WB3 && RegWriteW)
                ForwardB = 2'b11; // Forward from WB stage
        end

    end

    // Load-Use Hazard Detection
    wire lwstall;
    assign lwstall = (loadE && ((WB1 == A) || (WB1 == B)) && RegWriteE); // Only stall if MEM is writing to A or B

    // Branch Hazard Detection
    wire branchstall;
    assign branchstall = BranchD || ForSignalD;

    // Stall Logic for Fetch Stage
    always @(*) begin
        Stall = lwstall || branchstall;  // Stall fetch if there's a load-use hazard or branch hazard
    end

endmodule
