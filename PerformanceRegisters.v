module RR(
    input clk,
    input rst,
    input [15:0] in,
    output reg [15:0] out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out <= 16'h0000;
        end else begin
            out <= in;
        end
    end
endmodule



