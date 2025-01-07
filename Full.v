
`timescale 1ns / 1ps

module Top_tb;

    reg clk; // Clock signal
    reg reset; // Reset signal
    wire [15:0] writeData; // Data to be written
    wire [15:0] DataAdr; // Data memory address
    wire memwrite; // Memory write signal

    // Instantiate the Unit Under Test (UUT)
    Top uut (
        .clk(clk),
        .reset(reset),
        .writeData(writeData),
        .DataAdr(DataAdr),
        .memwrite(memwrite)
    );

    // Clock generation: toggle every 5ns for a 10ns period
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 0;

        // Dump waveforms to a VCD file
        $dumpfile("waveform.vcd"); // Specify VCD file name
        $dumpvars(0, Top_tb); // Dump all signals in the module hierarchy

        // Apply reset
        reset = 1; // Assert reset
        #10;
        reset = 0; // Deassert reset

        // Run simulation for a specified duration
        #5000; // Let the simulation run for 500ns
         // Print memory contents
        uut.dmem.printMemory();

        $finish; // End simulation
    end

endmodule



module Top(input clk, 
    input reset, 
    output [15:0] writeData, 
    output [15: 0] DataAdr, 
    output memwrite); 
    wire [15:0] PC, instr, readData;
    MainProcessor p1(clk,reset, PC, instr, memwrite, readData, DataAdr, writeData);
    InstructionMemory imem(PC,instr);
    DataMemory dmem(DataAdr,writeData,memwrite,clk,readData);
endmodule

module MainProcessor(
    input clk, 
    input rst, 
    output [15:0] PC, 
    input [15:0] instr, 
    output WriteToMEM, 
    input [15:0] readData,
    output [15:0]AluOut, 
    output [15: 0] WriteData);

    wire ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch;
    wire [2:0] AluControl;
    controller c(instr[15:12],instr[2:0],ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch,WriteToMEM,AluControl);
    DataPath data(clk, rst, SelectPCSrc, ForSignal, UpdateRR, JMP, Load, Rtype, Logical, WriteToReg, IMM, BNE, Branch, WriteToMEM, AluControl, readData, instr, PC, AluOut, WriteData);

endmodule

module InstructionMemory(input [15:0] A, output reg [15:0] RD);

    reg [15:0] memory [0:63];  //memory has 64 locations and each cell is 16 bits

    initial begin
    $readmemb("Instruction.txt", memory); //this will read the file to initialize the memory
    end



    always @* begin
        RD = memory[A];
    end
endmodule

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

module mainDec(input [3:0] opcode, input [2:0]func, output ForSignal, output UpdateRR, output JMP, output SelectPCSrc,
	output Load, 
	output RType,
	output Logical, 
	output WriteToReg, 
	output IMM,
	output BNE, 
	output Branch, 
	output WriteToMEM,
	output [1:0] AluOp);	   

    assign ForSignal = (opcode == `FOR);
    assign UpdateRR = ((opcode == `FOR) ^ (opcode == `JTYPE && func == `CALL));
    assign JMP = (opcode == `JTYPE && (func == `CALL ^  func == `JMP));
    assign SelectPCSrc = (opcode == `JTYPE);
    assign Load = (opcode ==`LOAD);
    assign RType = (opcode == `RTYPE); 
    assign Logical = (opcode != `ADDI);
    assign WriteToReg = (opcode != `JTYPE && opcode != `BEQ && opcode != `BNE && opcode != `STORE);
    assign IMM = (opcode == `ADDI ^ opcode ==`ANDI ^ opcode == `LOAD ^ opcode == `STORE);
    assign BNE = (opcode == `BNE);
    assign Branch = (opcode == `BEQ ^ opcode == `BNE);
    assign WriteToMEM = (opcode == `STORE);
    assign AluOp = (opcode == `RTYPE) ? 2'b00 :                  // RTYPE
                   (opcode == `ANDI)  ? 2'b10 :                  // ANDI
                   (opcode == `ADDI ^ opcode == `LOAD ^ opcode == `STORE)  ? 2'b01 :                  // ADDI
                   2'b11;       

endmodule


module AluDecoder(
    input logic [1:0] aluop,
    input logic [2:0] func,
    output logic [2:0] alucontrol
);
    always @* begin 
        case (aluop)
            2'b10: alucontrol = 3'b000; //and
            2'b01: alucontrol = 3'b001; //add
            2'b11: alucontrol = 3'b010;  //sub
            default: begin
                case (func)
                    3'b000: alucontrol = 3'b000;  //and
                    3'b001: alucontrol = 3'b001;  //add
                    3'b010: alucontrol = 3'b010;  //sub
                    3'b011: alucontrol = 3'b011;  //ShiftLeftLogical
                    3'b100: alucontrol = 3'b100;  //ShiftRight
                    default: alucontrol = 3'bxxx;
                endcase
            end
        endcase
    end
endmodule

module DataMemory(input [15:0] A, input [15:0] WD, input WE, CLK, output reg [15:0] RD);

     reg [15:0] memory [0:63];  //memory has 64 locations and each cell is 16 bits

        initial begin  
        $readmemb("Data.txt", memory); //this will read the file to initialize the memory
        end
        always @(negedge CLK) begin
        RD = memory[A];  // Read memory at address A
        end

     always @ (posedge CLK) begin
         if (WE == 1)
             begin
                    memory[A] = WD;
             end
        end 
  // Task to print memory contents
    task printMemory;
        integer i;
        begin
            $display("----- Memory Contents -----");
            for (i = 0; i < 64; i = i + 1) begin
                $display("Memory[%0d] = %0d", i, $signed(memory[i]));
            end
            $display("---------------------------");
        end
    endtask

endmodule

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
        RD1 = RegisterFile[A1];
        RD2 = RegisterFile[A2];
    end

endmodule


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
            2'b00: mux_out = in1;
            2'b01: mux_out = in2;
            2'b10: mux_out = in3;
            2'b11: mux_out = in4;
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
            out = I0;
        end
        else begin 
            out = I1;
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



module PC (
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
module DataPath(
    input clk,
    input rst,
    input slect_pc_src,
    input for_signal,
    input update_rr,
    input jmp_target,
    input load,
    input r_type,
    input logical_signal,
    input write_to_reg,
    input imm,
    input bne,
    input branch_signal,
    input write_to_mem,
    input [2:0] alu_control,
    input [15:0] RD, // data from memory
    input [15: 0] inst_in,
    output [15:0] pc_out,
    output [15:0] alu_out,
    output [15:0] write_on_memory_data
);

    // PC Handling
    wire [15: 0] pc_next;

    PC pc(
        .clk(clk),
        .rst(rst),
        .pc_next(pc_next),
        .pc_out(pc_out)
    );
    
    
    // JMP Target
    wire [15: 0] jmp_target_extended;
    // Branch
    wire [15: 0] branch_extended;
    // PC + 1
    wire [15: 0] pc_plus_1;

     // calculate pc by Adder module
    Adder adder(
        .In1(pc_out),
        .In2(16'h0001),
        .out(pc_plus_1)
    );

    // Concat module for jmp_target
    Concat concat_jmp_target(
        .in1(pc_plus_1[15: 9]),
        .in2(inst_in[11: 3]),
        .out(jmp_target_extended)
    );

    // Extender module for branch (sign extend)
    Extender extender_branch(
        .in(inst_in[5: 0]),
        .out(branch_extended),
        .logical_signal(logical_signal)
    );


    // AND between not_zero and for_signal by AND module
    wire and_out;
    // not_zero signal
    wire not_zero;

    NOT_Gate not_1(
        .in(zero_signal),
        .out(not_zero)
    );

    AND_Gate and_1(
        .In1(not_zero),
        .In2(for_signal),
        .out(and_out)
    );

    // MUX for choosing Zero or not zero
    wire mux_zero_out;

    Mux2x1 mux_zero(
        .I0(zero_signal),
        .I1(not_zero),
        .Sel(bne),
        .out(mux_zero_out)
    );

    // AND between branch and mux_zero by AND module
    wire and_out_2;
    AND_Gate and_2(
        .In1(mux_zero_out),
        .In2(branch_signal),
        .out(and_out_2)
    );
    // Register file

    // Mux2x1 for choosing (A1)
    wire [15: 0] mux_a1_out;
    Mux2x1 mux_a1(
        .I0(inst_in[11: 9]),
        .I1(inst_in[5: 3]),
        .Sel(r_type),
        .out(mux_a1_out)
    );
    // Mux2x1 for choosing (A3)
    wire [15: 0] mux_a3_out;
    Mux2x1 mux_a3(
        .I0(inst_in[8: 6]),
        .I1(inst_in[11: 9]),
        .Sel(r_type),
        .out(mux_a3_out)
    );
    
    // reg_out_2 + -1
    wire [15: 0] reg_out_2_minus_1;
    Adder adder_reg_out_2_minus_1(
        .In1(reg_out2),
        .In2(16'hFFFF),
        .out(reg_out_2_minus_1)
    );
    
    // ALU

    // Mux2x1 for choosing (R1)
    wire [15: 0] mux_r1_out;
    Mux2x1 mux_r1(
        .I0(reg_out1),
        .I1(16'h0000),
        .Sel(for_signal),
        .out(mux_r1_out)
    );
    // Mux2x1 for choosing (R2)
    wire [15: 0] mux_r2_out;
    Mux2x1 mux_r2(
        .I0(reg_out2),
        .I1(branch_extended),
        .Sel(imm),
        .out(mux_r2_out)
    );

    wire zero_signal_alu;
    ALU alu(
        .R1(mux_r1_out),
        .R2(mux_r2_out),
        .alucontrol(alu_control),
        .Answer(alu_out),
        .zeroflag(zero_signal_alu)
    );

    // assign zero_signal = zero_signal_alu;
    assign zero_signal = zero_signal_alu;
    assign write_on_memory_data = reg_out2;

    // Mux2x1 for choosing (WD3)
    wire [15: 0] mux_wd3_1_out;
    Mux2x1 mux_wd3_1(
        .I0(alu_out),
        .I1(reg_out_2_minus_1),
        .Sel(for_signal),
        .out(mux_wd3_1_out)
    );
    // Mux2x1 for choosing (WD3)
    wire [15: 0] mux_wd3_2_out;
    Mux2x1 mux_wd3(
        .I0(mux_wd3_1_out),
        .I1(RD),
        .Sel(load),
        .out(mux_wd3_2_out)
    );
    wire [15: 0] reg_out1;
    wire [15: 0] reg_out2;  
    RegisterFile reg_file (
        .clk(clk),
        .rst(rst),
        .WE3(write_to_reg),
        .WD3(mux_wd3_2_out),
        .A1(mux_a1_out[2: 0]),
        .A2(inst_in[8: 6]),
        .A3(mux_a3_out[2: 0]),
        .RD1(reg_out1),
        .RD2(reg_out2)
);
    // Mux2x1 for choosing (pc + 1 or Rs (for_loop instruction))
    wire [15: 0] mux_out;
    Mux2x1 mux_pc_src(
        .I0(pc_plus_1),
        .I1(reg_out2),
        .Sel(and_out),
        .out(mux_out)
    );


    // Mux4x2 for choosing next pc
    
    // add branch and pc + 1
    wire [15: 0] branch_extended_add_pc_plus_1;
    Adder adder_branch_pc_plus_1(
        .In1(pc_plus_1),
        .In2(branch_extended),
        .out(branch_extended_add_pc_plus_1)
    );
    // RR
    
    // RR current value
    wire [15: 0] rr_current;
    RR rr(
        .clk(clk),
        .rst(rst),
        .in(mux_rr_2_out),
        .out(rr_current)
    );

    // Mux 2x1 for choosing RR input 1
    wire [15: 0] mux_rr_1_out;
    Mux2x1 mux_rr_1(
        .I0(pc_plus_1),
        .I1(pc_out),
        .Sel(for_signal),
        .out(mux_rr_1_out)
    );
    // Mux2x1 to choose RR input
    wire [15: 0] mux_rr_2_out;
    Mux2x1 mux_rr_2(
        .I0(rr_current),
        .I1(mux_rr_1_out),
        .Sel(update_rr),
        .out(mux_rr_2_out)
    );

    // Mux2x1 to choose in3
    wire [15: 0] mux_in3_out;
    Mux2x1 mux_in3(
        .I0(rr_current),
        .I1(jmp_target_extended),
        .Sel(jmp_target),
        .out(mux_in3_out)
    );
    wire [15: 0] mux_next_pc_out;
    Mux4x2 mux_next_pc(
        .in1(mux_out),
        .in2(branch_extended_add_pc_plus_1),
        .in3(mux_in3_out),
        .in4(16'h0000),// nothing
        .sel({slect_pc_src, and_out_2}),
        .out(mux_next_pc_out)
    );  

    assign pc_next = mux_next_pc_out;

endmodule