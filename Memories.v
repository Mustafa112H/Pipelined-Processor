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
