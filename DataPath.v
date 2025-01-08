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
        .I1(reg_out1),
        .Sel(and_out),
        .out(mux_out)
    );


    // Mux4x2 for choosing next pc
    
    // add branch and pc + 1
    wire [15: 0] branch_extended_add_pc_plus_1;
    Adder adder_branch_pc_plus_1(
        .In1(pc_out),
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