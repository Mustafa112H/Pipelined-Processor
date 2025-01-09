

module hazard_detection(
    input [2:0] A,          
    input [2:0] B,          
    input [2:0] WB2,   
    input RegWriteM,         
    input [2:0] WB3,   
    input RegWriteW,      
    input BranchD,
    input ForSignalD,   
    output [1:0] ForwardA,   
    output [1:0] ForwardB,  
    output reg StallF,       // Stall signal for Fetch stage
    output reg StallD,       // Stall signal for Decode stage
    output reg FlushE        // Flush signal for Execute stage
);

// Forwarding for (First Source Register in Decode stag
assign ForwardA = 
    (A == WB2 && RegWriteM) ? 2'b10 :   // Forward data from MEM stage
    (A == WB3 && RegWriteW) ? 2'b01 :   // Forward data from WB stage
    2'b00;  // No forwarding (default)

// Forwarding for (Second Source Register)
assign ForwardB = 
    (B == WB2 && RegWriteM) ? 2'b10 :   // Forward data from MEM stage
    (B == WB3 && RegWriteW) ? 2'b01 :   // Forward data from WB stage
    2'b00;  // No forwarding (default)

// Load-use hazard detection
wire lwstall;
assign lwstall = ((A == WB2 || B == WB2) && RegWriteM) || ((A == WB3 || B == WB3) && RegWriteW);

wire branchstall;
assign branchstall = BranchD||ForSignalD;  // Stall on branch until resolved in EX stage

// Combine stalling conditions
always @(*) begin
    StallF = lwstall || branchstall;   // Stall fetch stage
    StallD = lwstall || branchstall;   // Stall decode stage
    FlushE = lwstall || branchstall;   // Flush execute stage if hazard detected
end

endmodule

