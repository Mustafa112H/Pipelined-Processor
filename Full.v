
`timescale 1ns / 1ps
`include "Control.v"
`include "Components.v"
`include "DataPath.v"
`include "PipelineRegisters.v"
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

// Pipelined Stages

// Stage 1: Instruction Fetch

// Stage 2: Instruction Decode
module DecodeStage(
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

// Stage 3: Execute
module ExecuteStage(
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
    task print_memory;
    integer i;
    begin
        // Loop through all memory locations and print the content
        $display("\n----------Instruction Memory-----------------");
        for (i = 0; i < 64; i = i + 1) begin
              // Set the address to i
            #1;  // Wait for the memory to resolve
            $display("MemoryInstr[%0d] = %b", i,memory[i] );  // Print memory content
        end
        $display("----------------------------------------------");
    end
    endtask
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
            $display("\n----- Memory Contents -----");
            for (i = 0; i < 64; i = i + 1) begin
                $display("Memory[%0d] = %0d", i, $signed(memory[i]));
            end
            $display("---------------------------");
        end
    endtask

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






