module HazardUnit(
    input [2:0] A,           // Source Register A
    input [2:0] B,           // Source Register B
    input [2:0] WB2,        // Write-Back Register in MEM stage
    input RegWriteM,         // Write-Back Enable in MEM stage
    input [2:0] WB3,         // Write-Back Register in WB stage
    input RegWriteW,         // Write-Back Enable in WB stage
    input BranchD,           // Branch Signal in Decode stage
    input ForSignalD,        // Forwarding Signal in Decode stage
    output reg [1:0] ForwardA, // Forwarding for Source A
    output reg [1:0] ForwardB, // Forwarding for Source B
    output reg Stall,       // Stall Fetch Stage
    input LoadM    // Load Signal in MEM stage
);

    // Forwarding Logic for Source Registers
    always @(*) begin
        ForwardA = 
            (A != 0 && A == WB2 && RegWriteM) ? 2'b01 : // Forward from MEM stage
            (A != 0 && A == WB3 && RegWriteW) ? 2'b10 : // Forward from WB stage
            2'b00; // No forwarding (default)

        ForwardB = 
            (B != 0 && B == WB2 && RegWriteM) ? 2'b01 : // Forward from MEM stage
            (B != 0 && B == WB3 && RegWriteW) ? 2'b10 : // Forward from WB stage
            2'b00; // No forwarding (default)
    end

    // Load-Use Hazard Detection
    wire lwstall;
    assign lwstall = (LoadM && ((WB2 == A) || (WB2 == B)));

    // Branch Hazard Detection
    wire branchstall;
    assign branchstall = BranchD || ForSignalD;

    // Combine Stalling and Flushing Conditions
    always @(*) begin
        Stall = lwstall || branchstall; // Stall Fetch stage
    end

endmodule
