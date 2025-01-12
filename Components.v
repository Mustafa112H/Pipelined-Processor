
module RegisterFile (
input clk, // Clock Signal
input rst, // Reset Signal
input WE3, // Write Enable Signal
input [15:0] WD3, // Write Data
input [2:0] A1, // Src. Address 1
input [2:0] A2, // Src. Address 2
input [2:0] A3, // Dst. Address
output reg [15:0] RD1, // Read Data Register
output reg [15:0] RD2 // Read Data Register
);

    integer i; // Loop Variable

    // Register File
    reg [15:0] RegisterFile [7:0];

    // Write Operation
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            // Reset all registers to 0
            for (i = 0; i < 8; i = i + 1) begin
                RegisterFile[i] <= 16'b0;
            end
        end
        else if (WE3) begin
            RegisterFile[A3] <= WD3;
        end
    end

    // Read Operation
    always @* begin
        if (!clk) begin
            RD1 = RegisterFile[A1];
            RD2 = RegisterFile[A2];
        end
    end

endmodule

module ALU(
    input [15:0] R1,
    input [15:0] R2,
    input [2:0] alucontrol,
    output [15:0] Answer,
    output zeroflag
);

    // Intermediate signals for operations
    wire [15:0] and_result;
    wire [15:0] add_result;
    wire [15:0] sub_result;
    wire [15:0] sll_result;
    wire [15:0] srl_result;

    // Perform operations
    assign and_result = R1 & R2;       // AND
    assign add_result = R1 + R2;       // Add
    assign sub_result = R2 - R1;       // Subtract
    assign sll_result = R2 << R1[3:0]; // Shift Left Logical (limited to 4 bits for shift)
    assign srl_result = R2 >> R1[3:0]; // Shift Right Logical (limited to 4 bits for shift)

    // Multiplexer to select the operation based on alucontrol
    assign Answer = (alucontrol == 3'b000) ? and_result :
                    (alucontrol == 3'b001) ? add_result :
                    (alucontrol == 3'b010) ? sub_result :
                    (alucontrol == 3'b011) ? sll_result :
                    (alucontrol == 3'b100) ? srl_result :
                    16'bxxxxxxxxxxxxxxxx;  // Default case

    // Zero flag logic
    assign zeroflag = (Answer == 16'b0);

endmodule



module Mux4x2 #(
    parameter WIDTH = 16
)(
    input [WIDTH-1:0] in1,
    input [WIDTH-1:0] in2,
    input [WIDTH-1:0] in3,
    input [WIDTH-1:0] in4,
    input [1:0] sel,
    output [WIDTH-1:0] out
);

    reg [WIDTH-1:0] mux_out;

    always @(*) begin
        case(sel)
            2'b00: mux_out <= in1;
            2'b01: mux_out <= in2;
            2'b10: mux_out <= in3;
            2'b11: mux_out <= in4;
        endcase
    end

    assign out = mux_out;

endmodule

module Mux2x1 #(
    parameter WIDTH = 16
)(
    input Sel, // Select Signal
    input [WIDTH-1: 0] I0, // Input 0
    input [WIDTH-1: 0] I1, // Input 1
    output reg [WIDTH-1: 0] out // Output Result
);

    always @ (*) begin
        if (Sel == 0) begin 
            out <= I0;
        end
        else begin 
            out <= I1;
        end
    end
endmodule

module NOT_Gate(
    input in,
    output out
);

    assign out = ~in;

endmodule

module AND_Gate(
    input In1,
    input In2,
    output out
);

    assign out = In1 & In2;

endmodule

module Extender(input [5:0] in, output [15:0] out, input logical_signal);
    // if logical signal 1 then unsign extend
    // if logical signal 0 then sign extend

    wire [15:0] sign_extended;
    wire [15:0] unsign_extended;

    assign sign_extended = {{10{in[5]}}, in};
    assign unsign_extended = {{10{1'b0}}, in};


    assign out = logical_signal ? unsign_extended : sign_extended;


endmodule



module Adder(input [15:0] In1, input [15:0] In2, output [15:0] out);
    assign out    = In1 + In2;
endmodule


module Concat(input [6:0] in1, input [8:0] in2, output [15:0] out);
    assign out = {in1, in2};
endmodule
