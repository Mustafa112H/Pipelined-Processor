module PCFetch (
    input clk,
    input rst,
    input [15:0] pc_next,
    input StallF,
    output reg [15:0] pc_out
);
    always @(posedge clk or posedge rst) begin

        
        if (rst) begin
            pc_out <= 16'h0000;
        end 
        else if (StallF == 0) begin
            pc_out <= pc_next;
        end
        else begin
            pc_out <= pc_out;
        end
    end

endmodule
//2: Decode Stage
module DecodeStage(
    input [15:0] InstructionIn,
    input StallD,
    input CLK,
    input Reset,
    output reg [15:0] InstructionOut,
    input ForSignalIn,
    input Load,
    input RType,
    input Logical,
    input WriteToReg,
    input IMM,
    input BNE, 
    input Branch,
    input WriteToMEM,
    input [2:0] AluControl,
    output reg ForSignalOut,
    output reg LoadOut,
    output reg RTypeOut,
    output reg LogicalOut,
    output reg WriteToRegOut,
    output reg IMMOut,
    output reg BNEOut, 
    output reg BranchOut,
    output reg WriteToMEMOut,
    output reg [2:0] aluControlOut
);

always @(posedge CLK or posedge Reset) begin
    if (Reset) begin
        // Reset all outputs to zero
        InstructionOut <= 16'b0; 
        ForSignalOut <= 0;
        LoadOut <= 0;
        RTypeOut <= 0;
        LogicalOut <= 0;
        WriteToRegOut <= 0;
        IMMOut <= 0;
        BNEOut <= 0;
        BranchOut <= 0;
        WriteToMEMOut <= 0;
        aluControlOut <= 3'b0;
    end else if (!StallD) begin
        // Update outputs with input values when not stalled
        InstructionOut <= InstructionIn;
        ForSignalOut <= ForSignalIn;
        LoadOut <= Load;
        RTypeOut <= RType;
        LogicalOut <= Logical;
        WriteToRegOut <= WriteToReg;
        IMMOut <= IMM;
        BNEOut <= BNE;
        BranchOut <= Branch;
        WriteToMEMOut <= WriteToMEM;
        aluControlOut <= AluControl;
    end
    else begin
        // If StallD is high, outputs retain their previous values (no updates)
        InstructionOut <= InstructionOut;
        ForSignalOut <= ForSignalOut;
        LoadOut <= LoadOut;
        RTypeOut <= RTypeOut;
        LogicalOut <= LogicalOut;
        WriteToRegOut <= WriteToRegOut;
        IMMOut <= IMMOut;
        BNEOut <= BNEOut;
        BranchOut <= BranchOut;
        WriteToMEMOut <= WriteToMEMOut;
        aluControlOut <= aluControlOut;

    end
end
endmodule


// Stage 3: ExecuteStage
module ExecuteStage(
    input rst,
    input clk,
    input [15: 0] out1in, 
    input [15: 0] out2in,
    input [2: 0] wb1in,
    input [15: 0] extin,
    input forSignalIn,
    input writeRegIn,
    input [2: 0] aluControl,
    input BNEin,
    input IMMin,
    input branchIn,
    input writeMemoryIn,
    input loadIn,
    input FlushE,
    output [15: 0] out1out,
    output [15: 0] out2out,
    output reg [2: 0] wb1out,
    output [15: 0] extout,
    output forSignalOut,
    output writeRegOut,
    output [2: 0] aluControlOut,
    output BNEOut,
    output IMMOut,
    output branchOut,
    output writeMemoryOut,
    output loadOut
);


    reg [15: 0] out1out, out2out, extout;
    reg forSignalOut, writeRegOut;
    reg [2: 0] aluControlOut;
    reg BNEOut, IMMOut, branchOut, writeMemoryOut, loadOut;


    always @ (posedge clk or posedge rst) begin
        if (rst || FlushE) begin
            out1out <= 16'h0000;
            out2out <= 16'h0000;
            wb1out <= 16'h0000;
            extout <= 16'h0000;
            forSignalOut <= 0;
            writeRegOut <= 0;
            aluControlOut <= 3'b000;
            BNEOut <= 0;
            IMMOut <= 0;
            branchOut <= 0;
            writeMemoryOut <= 0;
            loadOut <= 0;
        end else begin
            out1out <= out1in;
            out2out <= out2in;
            wb1out <= wb1in;
            extout <= extin;
            forSignalOut <= forSignalIn;
            writeRegOut <= writeRegIn;
            aluControlOut <= aluControl;
            BNEOut <= BNEin;
            IMMOut <= IMMin;
            branchOut <= branchIn;
            writeMemoryOut <= writeMemoryIn;
            loadOut <= loadIn;
        end
    end
endmodule

// Stage 4: MemoryStage
module MemoryStage(
    input clk,
    input rst,
    input [15:0] aluIn, 
    input [15:0] DataIn, 
    input [2:0] WB2in, 
    input writeMemoryIn,
    input forSignalIn,
    input loadIn, 
    input writeRegIn,   
    output reg [15:0] aluOut,
    output reg [15:0] DataOut,
    output reg [2:0] WB2out,
    output reg writeMemoryOut,
    output reg forSignalOut,
    output reg loadOut,
    output reg writeRegOut
);

    // Sequential block to update outputs on clock or reset
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all outputs
            aluOut <= 16'h0000;
            DataOut <= 16'h0000;
            WB2out <= 3'b000;
            writeMemoryOut <= 0;
            forSignalOut <= 0;
            loadOut <= 0;
            writeRegOut <= 0;
        end else begin
            // Update outputs based on inputs
            aluOut <= aluIn;
            DataOut <= DataIn;
            WB2out <= WB2in;
            writeMemoryOut <= writeMemoryIn; // Properly updating writeMemoryOut
            forSignalOut <= forSignalIn;
            loadOut <= loadIn;
            writeRegOut <= writeRegIn;
        end
    end
endmodule

// Stage 4: WriteBack
module WriteBackStage(
    input clk,
    input rst,
    input [15: 0] DataIn,
    input [2: 0] WB3in,
    input writeRegIn,
    output [15: 0] DataOut,
    output [2: 0] WB3out,
    output writeRegOut

);

    reg [15: 0] DataOut;
    reg [2: 0] WB3out;
    reg writeRegOut;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            DataOut <= 16'h0000;
            WB3out <= 16'h0000;
            writeRegOut <= 0;
        end else begin
            DataOut <= DataIn;
            WB3out <= WB3in;
            writeRegOut <= writeRegIn;
        end
    end
endmodule