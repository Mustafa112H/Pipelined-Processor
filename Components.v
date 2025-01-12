
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
    always @(posedge clk or posedge rst) begin
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
        RD1 <= RegisterFile[A1];
        RD2 <= RegisterFile[A2];
    end

endmodule

module ALU(
    input [15:0] R1, 
    input [15:0] R2, 
    input logic [2:0] alucontrol, 
    output logic [15:0] Answer,
    output logic zeroflag
);
    always @* begin 
        case (alucontrol)
            3'b000: Answer = R1 & R2;   // AND
            3'b001: Answer = R1 + R2;  // Add
            3'b010: Answer = R2 - R1;  // Subtract
            3'b011: Answer = R2 << R1; // Shift Left Logical
            3'b100: Answer = R2 >> R1; // Shift Right Logical
            default: Answer = 16'bxxxxxxxxxxxxxx; // Default to addition
        endcase

        // Set zeroflag if the result is 0
        if (Answer == 0)
            zeroflag = 1; 
        else 
            zeroflag = 0; 
    end
endmodule 

module Mux4x2(
    input [15:0] in1,
    input [15:0] in2,
    input [15:0] in3,
    input [15:0] in4,
    input [1:0] sel,
    output [15:0] out
);

    reg [15:0] mux_out;

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

module Mux2x1 (
    input Sel, // Select Signal
    input [15: 0] I0, // Input 0
    input [15: 0] I1, // Input 1
    output reg [15: 0] out // Output Result
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

