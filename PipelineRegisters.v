module PCFetch (
    input clk,
    input rst,
    input [15:0] pc_next,
    output reg [15:0] pc_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_out <= 16'h0000;
        end else begin
            pc_out <= pc_next;
        end
    end

endmodule
//2: Decode Stage
module DecodeStage(input [15:0] InstructionIn,
input StallD,
input CLK,
input Reset,
output [15:0] InstructionOut);
always @ (posedge CLK or posedge Reset )begin
    if (Reset) begin
        InstructionOut <= 16'b0; 
    end
    if (StallD==0) begin
    InstructionOut <= InstructionIn;
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
    output [15: 0] out1out,
    output [15: 0] out2out,
    output [2: 0] wb1out,
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


    reg [15: 0] out1out, out2out, wb1out, extout;
    reg forSignalOut, writeRegOut;
    reg [2: 0] aluControlOut;
    reg BNEOut, IMMOut, branchOut, writeMemoryOut, loadOut;


    always @ (posedge clk or posedge rst) begin
        if (rst) begin
            out1out = 16'h0000;
            out2out = 16'h0000;
            wb1out = 16'h0000;
            extout = 16'h0000;
            forSignal = 0;
            writeRegOut = 0;
            aluControlOut = 3'b000;
            BNEOut = 0;
            IMMOut = 0;
            branchOut = 0;
            writeMemoryOut = 0;
            loadOut = 0;
        end else begin
            out1out = out1in;
            out2out = out2in;
            wb1out = wb1in;
            extout = extin;
            forSignalIn = forSignalOut;
            writeRegOut = writeRegIn;
            aluControlOut = aluControl;
            BNEOut = BNEin;
            IMMOut = IMMin;
            branchOut = branchIn;
            writeMemoryOut = writeMemoryIn;
            loadOut = loadIn;
        end
    end
endmodule

// Stage 4: MemoryStage
module MemoryStage(
    input clk,
    input rst,
    input [15: 0] aluIn, 
    input [15: 0] DataIn, 
    input [2: 0] WB2in, 
    input writeMemoryIn,
    input forSignalIn,
    input loadIn, 
    input writeRegIn,   
    output [15: 0] aluOut,
    output [15: 0] DataOut,
    output [2: 0] WB2out,
    output writeMemoryOut,
    output forSignalOut,
    output loadOut,
    output writeRegOut
    );

    reg [15: 0] aluOut, DataOut;
    reg [2: 0] WB2out;
    reg writeMemoryOut, forSignalOut, loadOut, writeRegOut;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            aluOut = 16'h0000;
            DataOut = 16'h0000;
            WB2out = 16'h0000;
            writeMemoryOut = 0;
            forSignalOut = 0;
            loadOut = 0;
            writeRegOut = 0;
        end else begin
            aluOut = aluIn;
            DataOut = DataIn;
            WB2out = WB2in;
            writeMemoryOut = writeMemoryIn;
            forSignalOut = forSignalIn;
            loadOut = loadIn;
            writeRegOut = writeRegIn;
        end
    end
endmodule
