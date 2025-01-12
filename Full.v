`timescale 1ns / 1ps
`include "Control.v"
`include "Components.v"
`include "DataPath.v"
`include "PipelineRegisters.v"
`include "PerformanceRegisters.v"
`include "Memories.v"
`include "HazardUnit.v"

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
   initial
    begin
      reset <= 1; # 1; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 1; clk <= 0; # 1;
    end

    initial begin

        // Dump waveforms to a VCD file
        $dumpfile("waveform.vcd"); // Specify VCD file name
        $dumpvars(0, Top_tb); // Dump all signals in the module hierarchy



        // Run simulation for a specified duration
        #5000; // Let the simulation run for 500ns
         // Print memory contents
            uut.imem.print_memory();
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
    output WriteToMemM, 
    input [15:0] readData,
    output [15:0]AluOut, 
    output [15: 0] WriteData);

    wire ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch,WriteToMEM;
    wire [2:0] AluControl;
    wire [2:0] A;          
    wire [2:0] B;          
    wire [2:0] WB2;   
    wire RegWriteM;
    wire [2:0] WB3,WB1;   
    wire RegWriteW;
    wire [1:0] ForwardA;   
    wire [1:0] ForwardB;  
    wire Stall; 
    wire [15:0]instructionToDecode;
    HazardUnit unit(A, B, WB2, RegWriteM,WB3, RegWriteW, Branch,ForSignal,ForwardA,ForwardB,Stall,WriteToMEM,WB1);
    controller c(instr[15:12],instr[2:0],ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch,WriteToMEM,AluControl);
    DataPath data(clk, rst, SelectPCSrc, ForSignal, UpdateRR, JMP, Load, Rtype, Logical, WriteToReg, IMM, BNE, Branch, WriteToMEM,WriteToMemM, AluControl, readData, instr,instructionToDecode, PC, AluOut, WriteData,A, B, WB2, RegWriteM,WB3, RegWriteW,ForwardA,ForwardB,Stall,WB1 );

endmodule





