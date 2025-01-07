class InstructionEncoder:
    def __init__(self):
        self.register_map = {f"R{i}": format(i, '03b') for i in range(8)}

    def encode_r_type(self, func, rd, rs, rt):
        opcode = "0000"  # Opcode for R-Type instructions
        func_map = {
            "AND": "000",
            "ADD": "001",
            "SUB": "010",
            "SLL": "011",
            "SRL": "100",
        }
        if func not in func_map:
            raise ValueError("Invalid R-Type function")
        
        return opcode + self.register_map[rd] + self.register_map[rs] + self.register_map[rt] + func_map[func]

    def encode_i_type(self, opcode, rs, rt, imm=None):
        opcode_map = {
            "ANDI": "0010",
            "ADDI": "0011",
            "LW": "0100",
            "SW": "0101",
            "BEQ": "0110",
            "BNE": "0111",
            "FOR": "1000",  # Handle FOR opcode here
        }
        if opcode not in opcode_map:
            raise ValueError("Invalid I-Type opcode")

        # Special handling for "FOR" opcode: set immediate value to 0
        if opcode == "FOR":
            imm = 0  # Set immediate field to 0 for FOR instructions
        
        # Sign-extend the immediate value for instructions like LW and SW
        if opcode in ["LW", "SW", "BEQ", "BNE"]:
            imm_bin = format((imm + (1 << 6)) % (1 << 6), '06b')  # 6-bit signed immediate
        else:
            imm_bin = format(imm & 0x3F, '06b')  # Zero-extended for logical instructions
        
        return opcode_map[opcode] + self.register_map[rs] + self.register_map[rt] + imm_bin

    def encode_j_type(self, func, offset):
        opcode = "0001"  # Opcode for J-Type instructions
        func_map = {
            "JMP": "000",
            "CALL": "001",
            "RET": "010",
        }
        if func not in func_map:
            raise ValueError("Invalid J-Type function")

        if func == "RET":
            offset_bin = "000000000"  # Set 9-bit offset to zero for RET
        else:
            offset_bin = format(offset & 0x1FF, '09b')  # 9-bit offset for other functions
    
        return opcode + offset_bin + func_map[func]

    def encode_instruction(self, instruction):
        parts = instruction.split()
        instr_type = parts[0]

        if instr_type in ["AND", "ADD", "SUB", "SLL", "SRL"]:
            return self.encode_r_type(parts[0], parts[1], parts[2], parts[3])
        elif instr_type in ["ANDI", "ADDI"]:
            if len(parts) < 4:
                raise ValueError(f"Malformed I-Type instruction: {instruction}")
            return self.encode_i_type(parts[0], parts[2], parts[1], int(parts[3]))
        elif instr_type in ["LW", "SW"]:
            if len(parts) < 3:
                raise ValueError(f"Malformed I-Type instruction: {instruction}")
            rt = parts[1]
            imm_and_rs = parts[2]  # e.g., "10(R6)"
            if "(" not in imm_and_rs or ")" not in imm_and_rs:
                raise ValueError(f"Malformed LW/SW instruction: {instruction}")
            imm, rs = imm_and_rs.split("(")
            rs = rs.rstrip(")")
            return self.encode_i_type(parts[0], rs, rt, int(imm))
        elif instr_type in ["BEQ", "BNE"]:
            if len(parts) < 4:
                raise ValueError(f"Malformed I-Type instruction: {instruction}")
            return self.encode_i_type(parts[0], parts[1], parts[2], int(parts[3]))
        elif instr_type in ["JMP", "CALL"]:
            if len(parts) < 2:
                raise ValueError(f"Malformed J-Type instruction: {instruction}")
            return self.encode_j_type(parts[0], int(parts[1]))
        elif instr_type == "RET":
            # Special case for RET: No offset is needed, set to 0
            return self.encode_j_type(parts[0], 0)  # Use 0 as the offset for RET
        elif instr_type == "EXIT":
            # Handle the EXIT instruction as a 16-bit exit signal but don't stop processing
            return "1111111111111111"  # 16-bits for EXIT instruction
        elif instr_type == "FOR":
            # Special handling for the FOR instruction
            return self.encode_i_type(instr_type, parts[1], parts[2], 0)  # Immediate field is 0 for FOR
        else:
            raise ValueError(f"Invalid instruction type: {instruction}")

    def process_file(self, input_file, output_file):
        with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
            for line in infile:
                line = line.rstrip()  # Remove only the trailing newline/whitespace
                if line == "" or line.startswith("#"):  # Preserve empty lines or comments
                    outfile.write("0000\n")  # Preserve empty lines
                else:
                    try:
                        machine_code = self.encode_instruction(line)
                        outfile.write(machine_code + "\n")
                    except ValueError as e:
                        outfile.write(f"Error: {e} in instruction: {line}\n")


# Example usage
if __name__ == "__main__":
    encoder = InstructionEncoder()
    encoder.process_file("input_instructions.txt", "instruction.txt")
