
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
    always #5 clk = ~clk;
    always begin
        #5;
        if (uut.instr == 16'b1111111111111111) begin
            $display("Successfully Exited");
            uut.imem.print_memory();
            uut.dmem.printMemory();
            $finish;
        end
        if(uut.instr == 0) begin
            $display("PC = %0b", uut.PC);
            $display("Invalid Instruction ..... Terminating!");
            uut.imem.print_memory();
            uut.dmem.printMemory();
            $finish;// Terminate the simulation
        end
    end 

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
    output WriteToMEM, 
    input [15:0] readData,
    output [15:0]AluOut, 
    output [15: 0] WriteData);

    wire ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch;
    wire [2:0] AluControl;
    controller c(instr[15:12],instr[2:0],ForSignal, UpdateRR, JMP, SelectPCSrc, Load, Rtype, Logical, WriteToReg,IMM, BNE,Branch,WriteToMEM,AluControl);
    DataPath data(clk, rst, SelectPCSrc, ForSignal, UpdateRR, JMP, Load, Rtype, Logical, WriteToReg, IMM, BNE, Branch, WriteToMEM, AluControl, readData, instr, PC, AluOut, WriteData);

endmodule





