module HazardUnit(
    input [2:0] A,            // Source Register A
    input [2:0] B,            // Source Register B
    input [2:0] WB2,          // Write-Back Register in MEM stage
    input RegWriteM,          // Write-Back Enable in MEM stage
    input [2:0] WB3,          // Write-Back Register in WB stage
    input RegWriteW,          // Write-Back Enable in WB stage
    input BranchD,            // Branch Signal in Decode stage
    input ForSignalD,         // Forwarding Signal in Decode stage
    output reg [1:0] ForwardA,// Forwarding for Source A
    output reg [1:0] ForwardB,// Forwarding for Source B
    output reg Stall,         // Stall Fetch Stage
    input LoadM, 
    input [2:0]WB1              // Load Signal in MEM stage
);

    // Forwarding Logic for Source Registers A and B
    always @(*) begin
        // Default values (no forwarding)
        
        ForwardA = 2'b00;
        ForwardB = 2'b00;

        // Forwarding for A
        if (A != 0) begin
            if (A == WB1 && RegWriteM) 
                ForwardA = 2'b01; // Forward from MEM stage
            else if (A == WB2 && RegWriteW)
                ForwardA = 2'b10; // Forward from WB stage
            
        end

        // Forwarding for B
        if (B != 0) begin
            if (B == WB1 && RegWriteM) 
                ForwardB = 2'b01; // Forward from MEM stage
            else if (B == WB2 && RegWriteW)
                ForwardB = 2'b10; // Forward from WB stage
        end
    end

    // Load-Use Hazard Detection
    wire lwstall;
    assign lwstall = (LoadM && ((WB1 == A) || (WB1 == B)) && RegWriteM); // Only stall if MEM is writing to A or B

    // Branch Hazard Detection
    wire branchstall;
    assign branchstall = BranchD || ForSignalD;

    // Stall Logic for Fetch Stage
    always @(*) begin
        Stall = lwstall || branchstall;  // Stall fetch if there's a load-use hazard or branch hazard
    end

endmodule
