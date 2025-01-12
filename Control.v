module controller(
    input logic [3:0] opcode, 
    input [2:0] func, 
    output ForSignal, 
    output UpdateRR, 
    output JMP, 
    output SelectPCSrc,
	output Load, 
	output RType,
	output Logical, 
	output WriteToReg, 
	output IMM,
	output BNE, 
	output Branch, 
	output WriteToMEM,
	output [2:0] AluControl);
	
	wire [1:0] aluop;
	mainDec main1(opcode, func, ForSignal, UpdateRR, JMP, SelectPCSrc, Load, RType, Logical, WriteToReg, IMM, BNE, Branch, WriteToMEM, aluop);
	AluDecoder alu1(aluop, func, AluControl);	  

endmodule

`define RTYPE   4'b0000
`define ANDI    4'b0010
`define ADDI    4'b0011
`define BEQ     4'b0110
`define BNE     4'b0111
`define FOR     4'b1000
`define LOAD    4'b0100
`define STORE   4'b0101
`define JMP     3'b000
`define CALL    3'b001
`define RET  3'b010
`define JTYPE   4'b0001

module mainDec(
    input [3:0] opcode, 
    input [2:0] func, 
    output ForSignal, 
    output UpdateRR, 
    output JMP, 
    output SelectPCSrc,
    output Load, 
    output RType,
    output Logical, 
    output WriteToReg, 
    output IMM,
    output BNE, 
    output Branch, 
    output WriteToMEM,
    output [1:0] AluOp
);	   

    assign ForSignal = (opcode == `FOR);
    assign UpdateRR = ((opcode == `FOR) || (opcode == `JTYPE && func == `CALL));
    assign JMP = (opcode == `JTYPE && (func == `CALL || func == `JMP));
    assign SelectPCSrc = (opcode == `JTYPE);
    assign Load = (opcode == `LOAD);
    assign RType = (opcode == `RTYPE); 
    assign Logical = (opcode == `ANDI);
    assign WriteToReg = (opcode != `JTYPE && opcode != `BEQ && opcode != `BNE && opcode != `STORE);
    assign IMM = (opcode == `ADDI || opcode == `ANDI || opcode == `LOAD || opcode == `STORE);
    assign BNE = (opcode == `BNE);
    assign Branch = (opcode == `BEQ || opcode == `BNE);
    assign WriteToMEM = (opcode == `STORE);
    assign AluOp = (opcode == `RTYPE) ? 2'b00 :                  // RTYPE
                   (opcode == `ANDI)  ? 2'b10 :                  // ANDI
                   (opcode == `ADDI || opcode == `LOAD || opcode == `STORE) ? 2'b01 : // ADDI, LOAD, STORE
                   2'b11;        // Default case       

endmodule


module AluDecoder(
    input logic [1:0] aluop,
    input logic [2:0] func,
    output logic [2:0] alucontrol
);
    always @* begin 
        case (aluop)
            2'b10: alucontrol <= 3'b000; //and
            2'b01: alucontrol <= 3'b001; //add
            2'b11: alucontrol <= 3'b010;  //sub
            default: begin
                case (func)
                    3'b000: alucontrol <= 3'b000;  //and
                    3'b001: alucontrol <= 3'b001;  //add
                    3'b010: alucontrol <= 3'b010;  //sub
                    3'b011: alucontrol <= 3'b011;  //ShiftLeftLogical
                    3'b100: alucontrol <= 3'b100;  //ShiftRight
                    default: alucontrol <= 3'bxxx;
                endcase
            end
        endcase
    end
endmodule