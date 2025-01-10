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
    output WriteToMemoryM,
    input [2:0] alu_control,
    input [15:0] RD, // data from memory
    input [15: 0] inst_in,
    output [15:0] instructionToDecode,
    output [15:0] pc_out,
    output [15:0] AluM,
    output [15:0] DataOut,
    output [2:0] A1,
    output [2:0] A2,
    output [2:0] WB2,
    output writeToRegM,
    output [2:0] WB3,
    output writeToRegW,
    input [1:0] ForwardA,
    input [1:0] ForwardB,
    input StallF,
    input StallD,
    input FlushE
);

    // PC Handling
    wire [15: 0] pc_next;

    PCFetch pc(
        .clk(clk),
        .rst(rst),
        .pc_next(pc_next),
        .pc_out(pc_out),
        .StallF(StallF)
    );
    wire [15:0] instructionToDecode;
    DecodeStage dec(inst_in,StallD, clk, rst,instructionToDecode);
    //Forwarding choose

        wire [15:0] WBData;
        wire [15: 0] out1_E;
        wire [15: 0] out2_E;
        wire [15:0] out1, out2; 
    Mux4x2 FWA(
        .in1(reg_out1),
        .in2(DataOut),
        .in3(WBData),
        .in4(16'h0000),
        .sel(ForwardA),
        .out(out1)
    );
    Mux4x2 FWB(
        .in1(reg_out2),
        .in2(DataOut),
        .in3(WBData),
        .in4(16'h0000),
        .sel(ForwardB),
        .out(out2)
    );
    wire [2: 0] WB1;
    wire [15:0] EXT_Out;
    wire ForSignalE,writeToRegE,bneE,immE,BranchE,WriteMemoryE,loadE;
    wire [2:0] aluControlE;
    ExecuteStage execute(rst, clk,out1,out2, mux_a3_out, branch_extended,for_signal,write_to_reg,alu_control,bne,imm,branch_signal,write_to_mem,load,FlushE,out1_E, out2_E,WB1,EXT_Out,ForSignalE,writeToRegE,aluControlE,bneE,immE,BranchE,WriteMemoryE,loadE );
    wire [15:0] AluM;
    wire ForSignalM,LoadM;


    MemoryStage memstage(clk,rst,alu_out,out2_E,WB1, WriteMemoryE, ForSignalE,loadE,writeToRegE, AluM,DataOut,WB2,WriteToMemoryM,ForSignalM,LoadM,writeToRegM);
    WriteBackStage wb(clk, rst, mux_wd3_2_out, WB2, writeToRegM,WBData,WB3, writeToRegW );
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
        .in2(instructionToDecode[11: 3]),
        .out(jmp_target_extended)
    );

    // Extender module for branch (sign extend)
    Extender extender_branch(
        .in(instructionToDecode[5: 0]),
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
        .Sel(bneE),
        .out(mux_zero_out)
    );

    // AND between branch and mux_zero by AND module
    wire and_out_2;
    AND_Gate and_2(
        .In1(mux_zero_out),
        .In2(BranchE),
        .out(and_out_2)
    );
    // Register file

    // Mux2x1 for choosing (A1)
    wire [2:0] A1;
    Mux2x1 mux_a1(
        .I0(instructionToDecode[11: 9]),
        .I1(instructionToDecode[5: 3]),
        .Sel(r_type),
        .out(A1)
    );
    // Mux2x1 for choosing (A3)
    wire [2: 0] mux_a3_out;
    Mux2x1 mux_a3(
        .I0(instructionToDecode[8: 6]),
        .I1(instructionToDecode[11: 9]),
        .Sel(r_type),
        .out(mux_a3_out)
    );
    
    // reg_out_2 + -1
    wire [15: 0] reg_out_2_minus_1;
    Adder adder_reg_out_2_minus_1(
        .In1(DataOut),
        .In2(16'hFFFF),
        .out(reg_out_2_minus_1)
    );
    
    // ALU

    // Mux2x1 for choosing (R1)
    wire [15: 0] mux_r1_out;
    Mux2x1 mux_r1(
        .I0(out1_E),
        .I1(16'h0000),
        .Sel(ForSignalE),
        .out(mux_r1_out)
    );
    // Mux2x1 for choosing (R2)
    wire [15: 0] mux_r2_out;
    Mux2x1 mux_r2(
        .I0(out2_E),
        .I1(EXT_Out),
        .Sel(immE),
        .out(mux_r2_out)
    );
wire [15:0] alu_out;
    wire zero_signal_alu;
    ALU alu(
        .R1(mux_r1_out),
        .R2(mux_r2_out),
        .alucontrol(aluControlE),
        .Answer(alu_out),
        .zeroflag(zero_signal_alu)
    );

    // assign zero_signal = zero_signal_alu;
    assign zero_signal = zero_signal_alu;
    assign write_on_memory_data = reg_out2;

    // Mux2x1 for choosing (WD3)
    wire [15: 0] mux_wd3_1_out;
    Mux2x1 mux_wd3_1(
        .I0(AluM),
        .I1(reg_out_2_minus_1),
        .Sel(ForSignalM),
        .out(mux_wd3_1_out)
    );
    // Mux2x1 for choosing (WD3)
    wire [15: 0] mux_wd3_2_out;
    Mux2x1 mux_wd3(
        .I0(mux_wd3_1_out),
        .I1(RD),
        .Sel(LoadM),
        .out(mux_wd3_2_out)
    );
    wire [15: 0] reg_out1;
    wire [15: 0] reg_out2;  
    wire [2:0] A2; 
    assign A2 = instructionToDecode[8:6];
    RegisterFile reg_file (
        .clk(clk),
        .rst(rst),
        .WE3(writeToRegW),
        .WD3(WBData),
        .A1(A1),
        .A2(A2),
        .A3(WB3),
        .RD1(reg_out1),
        .RD2(reg_out2)
);
    // Mux2x1 for choosing (pc + 1 or Rs (for_loop instruction))
    wire [15: 0] mux_out;
    Mux2x1 mux_pc_src(
        .I0(pc_plus_1),
        .I1(reg_out1),
        .Sel(and_out),
        .out(mux_out)
    );


    // Mux4x2 for choosing next pc
    
    // add branch and pc + 1
    wire [15: 0] branch_extended_add_pc_plus_1;
    Adder adder_branch_pc_plus_1(
        .In1(pc_out),
        .In2(EXT_Out),
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