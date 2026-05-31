/*
// ==============================================================================
// MODULE: mpt_decoder (Instruction Decoder & Control Signal Generator)
//
// ROLE IN ARCHITECTURE: 
// This module takes a raw 32-bit machine language instruction (`instruction_i`) 
// and splits it up. It extracts register addresses, reconstructs immediate 
// constants, and generates individual 1-bit or multi-bit control flags that drive 
// the Execution (EX), Memory (MEM), and Writeback (WB) stages further down the pipeline.
//
// HARDWARE MAPPING: 
// This is PURE COMBINATIONAL LOGIC. In physical silicon, this translates to 
// an intricate network of Look-Up Tables (LUTs) and multiplexers. 
//
// CRITICAL HARDWARE DESIGN TRICK: 
// Notice that `rs1_addr_o`, `rs2_addr_o`, and `rd_addr_o` are assigned using 
// straight wires directly out of the instruction bus before the `case(OPCODE)` statement 
// even starts! Because RISC-V fixes the positions of these fields, the Register File 
// can start fetching data from physical transistors at the *exact same instant* // the decoder is figuring out what opcode is executing. This saves precious nanoseconds.
// ==============================================================================
*/

`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

module mpt_decoder 
    `include "mpt_definitions.vh"
    `ifdef CUSTOM_DEFINE
        #(parameter REG_DATA_WIDTH      = `REG_DATA_WIDTH,
          parameter REGFILE_ADDR_WIDTH  = `REGFILE_ADDR_WIDTH,
          parameter REGFILE_DEPTH       = `REGFILE_DEPTH,
          parameter ALU_OP_WIDTH        = `ALU_OP_WIDTH
          )
    `else
        #(parameter REG_DATA_WIDTH      = 32,
          parameter REGFILE_ADDR_WIDTH  = 5,
          parameter REGFILE_DEPTH       = 32,
          parameter ALU_OP_WIDTH        = 4
          )
    `endif

    (
    // --- REGFILE ADDRESSES ---
    // These indicate which internal physical registers are being accessed.
    output reg   [4:0]                      rd_addr_o,        // Destination register address (where to save the final answer)
    output reg   [4:0]                      rs1_addr_o,       // Source register 1 address (first number to read)
    output reg   [4:0]                      rs2_addr_o,       // Source register 2 address (second number to read)
    
    // --- ALU OPERANDS & IMMEDIATE CONSTANTS ---
    output reg   [REG_DATA_WIDTH-1 :0]      imm1_o,           // Immediate value explicitly directed to ALU Operand 1
    output reg   [REG_DATA_WIDTH-1 :0]      imm2_o,           // Immediate value explicitly directed to ALU Operand 2
    
    // --- PIPELINE CONTROL SIGNALS ---
    output reg   [1:0]                      alu_source_sel_o, // Controls EX MUXes: Bit [1] forces Op1 to be an immediate; Bit [0] forces Op2 to be an immediate
    output reg   [3:0]                      alu_op_o,         // Sent to ALU: Dictates the operation (ADD, SUB, XOR, etc.)
    output reg   [1:0]                      branch_op_o,      // Sent to Branch Gen: Dictates PC-Relative or Register-Offset jumps
    output reg                              branch_flag_o,    // Tells EX stage to invert branch logic (0 = branch if true, 1 = branch if false)
    output reg                              mem_wr_en_o,      // Sent to MEM: Enables writing data out to RAM (Stores)
    output reg                              mem_rd_en_o,      // Sent to MEM: Enables reading data in from RAM (Loads)
    output reg                              rd_wr_en_o,       // Sent to WB: Enables writing an answer back into the Register File
    output reg                              memtoreg_o,       // Sent to WB MUX: 0 = write ALU result to register; 1 = write RAM load data to register
    output reg                              jump_en_o,        // Asserted if instruction is an unconditional jump (JAL/JALR)
    output reg   [3:0]                      mem_op_o,         // Tells MEM stage how many bytes to read/write (Byte, Halfword, Word, Signed/Unsigned)
    output reg                              exception_o,      // Flag representing an environmental breakpoint/system call exception

    // --- INPUTS ---
    input  wire  [REG_DATA_WIDTH-1 :0]      instruction_i,    // The raw 32-bit machine code instruction from Fetch stage
    input  wire  [31:0]                     pc_i              // The Program Counter address of this specific instruction
    );
    
// ===========================================================================
//                    INSTRUCTION SUB-FIELD WIRES
// ===========================================================================
    reg [6:0]  OPCODE; 
    reg [4:0]  RD;     
    reg [3:0]  FUNCT3; 
    reg        FUNCT7; 
    
    // Internal registers holding decoded immediate options
    reg [31:0] IMM_I; // I-type (used for immediate math and loads)
    reg [31:0] IMM_S; // S-type (used for store offsets)
    reg [31:0] IMM_B; // B-type (used for conditional branches)
    reg [31:0] IMM_U; // U-type (used for upper immediates like LUI)
    reg [31:0] IMM_J; // J-type (used for unconditional jumps)

   
// ===========================================================================
// BLOCK 1: SUB-FIELD PARSING & IMMEDIATE CONSTRUCTORS
// Hardware: Straight physical routing wires and sign-extension trees.
// ===========================================================================
    always@* begin
        // Slicing out fields is structurally free in physical hardware. It uses 0 gates.
        OPCODE      = instruction_i[6:0];
        FUNCT3      = instruction_i[14:12];
        FUNCT7      = instruction_i[30]; // We only need bit 30 from the typical funct7 field
        
        // --- SIGN EXTENSION CIRCUITRY ---
        // Why `{ {20{instruction_i[31]}}, ... }`?
        // Computers represent negative numbers using Two's Complement. If an immediate value 
        // is negative, its highest bit is 1. When expanding a 12-bit constant to a 32-bit register, 
        // we must copy that sign bit all the way to the left, otherwise a negative number 
        // accidentally turns into a massive positive number.
        // In hardware, this replicates bit 31 across 20 distinct parallel wires.
        
        IMM_I       = { {20{instruction_i[31]}}, instruction_i[31:20] }; 
        IMM_S       = { {20{instruction_i[31]}}, instruction_i[31:25], instruction_i[11:7] }; 
        
        // Note: RISC-V intentionally shuffles the bit positions of B-type and J-type immediates 
        // inside the instruction layout. This keeps bit 31 (the sign bit) at the exact same physical 
        // pin on the chip across ALL instruction formats, reducing propagation delay!
        IMM_B       = { {20{instruction_i[31]}}, instruction_i[7], instruction_i[30:25], instruction_i[11:8], 1'b0 }; 
        IMM_U       = { instruction_i[31:12], {12{1'b0}} }; // LUI clears out bottom 12 bits with zeros
        IMM_J       = { {12{instruction_i[31]}}, instruction_i[19:12], instruction_i[20], instruction_i[30:25], instruction_i[24:21], 1'b0};
    end
    
// ===========================================================================
// BLOCK 2: MAIN DECODER CASE STATEMENT
// Hardware: A giant combinational logic decoder driving output multiplexers.
// ===========================================================================
    always@* begin
        // --- DEFAULT ASSIGNMENTS ---
        // Crucial to prevent synthesis tools from creating accidental hardware latches.
        imm1_o           = 32'b0; 
        imm2_o           = 32'b0;
        alu_source_sel_o = 2'b0;  
        alu_op_o         = 0;     
        branch_op_o      = 0;     
        branch_flag_o    = 0;     
        mem_wr_en_o      = 0;     
        mem_rd_en_o      = 0;     
        rd_wr_en_o       = 0;     
        memtoreg_o       = 0;     
        jump_en_o        = 0;     
        mem_op_o         = `MEM_LW;      
        
        // Exploit fixed RISC-V layouts: parse register locations immediately
        rd_addr_o        = instruction_i[11:7]; 
        rs1_addr_o       = instruction_i[19:15];
        rs2_addr_o       = instruction_i[24:20];

        // Check if instruction is a system breakpoint/environment call
        exception_o      = ((instruction_i == `ECALL) || (instruction_i == `EBREAK));

        case(OPCODE)
        
            // ------------------------------------------------------------------
            // R-TYPE INSTRUCTIONS (e.g., ADD, SUB, AND, OR)
            // Architecture: Math performed strictly between two registers.
            // ------------------------------------------------------------------
            `OPCODE_OP: begin 
                rd_wr_en_o = 1; // Yes, we will write an answer back to the Register File
                case(FUNCT3)
                    // ADD and SUB share the exact same FUNCT3 field. 
                    // We inspect FUNCT7 (bit 30) to see if we should invert the adder into a subtractor.
                    `FUNCT3_ADD_SUB: alu_op_o = (FUNCT7) ? `ALU_SUB : `ALU_ADD;
                    `FUNCT3_SLL:     alu_op_o = `ALU_SLL;
                    `FUNCT3_SLT:     alu_op_o = `ALU_SLT;          
                    `FUNCT3_SLTU:    alu_op_o = `ALU_SLTU;
                    `FUNCT3_XOR:     alu_op_o = `ALU_XOR;
                    // SRL (Shift Right Logical) and SRA (Shift Right Arithmetic) also share a FUNCT3.
                    `FUNCT3_SRL_SRA: alu_op_o = (FUNCT7) ? `ALU_SRA : `ALU_SRL;
                    `FUNCT3_OR:      alu_op_o = `ALU_OR;
                    `FUNCT3_AND:     alu_op_o = `ALU_AND;
                    default:         alu_op_o = `ALU_ADD;
                endcase
            end
            
            // ------------------------------------------------------------------
            // I-TYPE MATH INSTRUCTIONS (e.g., ADDI, ANDI)
            // Architecture: Math performed between register 1 and a constant.
            // ------------------------------------------------------------------
            `OPCODE_OP_IMM: begin 
                // Hardware Guard: Register x0 is hardwired to 0. If rd is x0, do not enable writing!
                rd_wr_en_o  = (instruction_i[11:7] == 0) ? 0:1;
                
                alu_source_sel_o = 2'b01; // Force ALU Operand 2 MUX to select the immediate wire
                imm2_o           = IMM_I;  // Bind the decoded I-type immediate to Operand 2
                rs2_addr_o       = 0;      // Zero out rs2 because this instruction doesn't read a second register
                
                case(FUNCT3)
                    `FUNCT3_ADDI:      alu_op_o = `ALU_ADD;  
                    `FUNCT3_ANDI:      alu_op_o = `ALU_AND;
                    `FUNCT3_ORI:       alu_op_o = `ALU_OR;
                    `FUNCT3_XORI:      alu_op_o = `ALU_XOR;
                    `FUNCT3_SLTI:      alu_op_o = `ALU_SLT;
                    `FUNCT3_SLTIU:     alu_op_o = `ALU_SLTU;
                    `FUNCT3_SRAI_SRLI: alu_op_o = (FUNCT7) ? `ALU_SRA : `ALU_SRL; 
                    `FUNCT3_SLLI:      alu_op_o = `ALU_SLL;
                    default:           alu_op_o = `ALU_ADD;
                endcase
            end
            
            // ------------------------------------------------------------------
            // B-TYPE INSTRUCTIONS (Conditional Branches: BEQ, BNE, BLT)
            // Architecture: Compare two registers. If comparison matches, jump.
            // ------------------------------------------------------------------
            `OPCODE_BRANCH: begin
                branch_op_o     = `PC_RELATIVE; // Inform Branch Gen that target calculation is PC + Immediate
                imm2_o          = IMM_B;        // Direct B-type immediate out to the Branch Gen module
                rd_addr_o       = 0;            // Branches never write to a destination register
                
                case(FUNCT3)
                    // WHY INVERT `branch_flag_o`?
                    // The ALU flags return a '1' if a comparison is true.
                    // For BEQ, if ALU says "Equal" (1), we branch if flag matches 0? 
                    // Look closely: your execution stage logic uses `branch_flag_o` to handle inversions.
                    // If `branch_flag_o = 0`, it expects a raw '1' to branch. 
                    // If `branch_flag_o = 1` (like BNE), it branches when the equality checker returns '0' (Not Equal).
                    `FUNCT3_BEQ: begin
                        branch_flag_o = 0;
                        alu_op_o      = `ALU_SEQ;  
                    end
                    `FUNCT3_BNE: begin
                        branch_flag_o = 1;
                        alu_op_o      = `ALU_SEQ;  
                    end
                    `FUNCT3_BLT: begin
                        branch_flag_o = 0;
                        alu_op_o      = `ALU_SLT;  
                    end
                    `FUNCT3_BGE: begin
                        branch_flag_o = 1;
                        alu_op_o      = `ALU_SLT; // BGE is just the inverse outcome of an SLT check!
                    end
                    `FUNCT3_BLTU: begin
                        branch_flag_o = 0;
                        alu_op_o      = `ALU_SLTU; 
                    end
                    `FUNCT3_BGEU: begin
                        branch_flag_o = 1;
                        alu_op_o      = `ALU_SLTU; 
                    end
                    default: begin
                        branch_flag_o = 0;
                        alu_op_o      = `ALU_ADD;
                    end
                endcase    
            end
            
            // ------------------------------------------------------------------
            // U-TYPE INSTRUCTIONS: LUI (Load Upper Immediate)
            // Architecture: Shifts a 20-bit constant into the top of a register.
            // ------------------------------------------------------------------
            `OPCODE_LUI: begin
                rd_wr_en_o       = (instruction_i[11:7] == 0) ? 0:1;
                alu_source_sel_o = 2'b11;    // Force BOTH ALU Operand MUXes to read immediates
                imm1_o           = 32'b0;    // Op1 = 0
                imm2_o           = IMM_U;    // Op2 = Constant loaded in top 20 bits
                alu_op_o         = `ALU_ADD; // 0 + IMM_U = IMM_U. It passes straight through to rd.
                rs1_addr_o       = 0;
                rs2_addr_o       = 0;
            end
            
            // ------------------------------------------------------------------
            // U-TYPE INSTRUCTIONS: AUIPC (Add Upper Immediate to PC)
            // Architecture: Used for finding relative memory locations at runtime.
            // ------------------------------------------------------------------
            `OPCODE_AUIPC: begin
                rd_wr_en_o       = (instruction_i[11:7] == 0) ? 0:1;
                alu_source_sel_o = 2'b11;    // Force both ALU operands to bypass register data
                imm1_o           = pc_i;     // Op1 = Current address location
                imm2_o           = IMM_U;    // Op2 = Upper immediate constant
                alu_op_o         = `ALU_ADD; // Adds them together and pipes to destination register
                rs1_addr_o       = 0;
                rs2_addr_o       = 0;
            end
            
            // ------------------------------------------------------------------
            // J-TYPE INSTRUCTIONS: JAL (Jump and Link)
            // Architecture: Unconditional jump relative to the current PC.
            // ------------------------------------------------------------------
            `OPCODE_JAL: begin
                rd_wr_en_o       = (instruction_i[11:7] == 0) ? 0:1;
                jump_en_o        = 1;            // Signal pipeline control that a jump is occurring
                branch_op_o      = `PC_RELATIVE; // Destination address = PC + IMM_J
                
                // WHY FORCE `alu_source_sel_o = 2'b10`?
                // A jump-and-link instruction must calculate "PC + 4" (the address of the 
                // next instruction) and save it into the destination register so a function 
                // knows how to return. 
                // `2'b10` tells the ALU to put `pc_i` into Operand 1. Further down in your EX 
                // stage file, if `ID_jump_en_i` is active, Operand 2 is hardcoded to force the integer 4!
                alu_source_sel_o = 2'b10; 
                imm1_o           = pc_i; 
                imm2_o           = IMM_J; // Routed separately straight to the Branch Generator
                rs1_addr_o       = 0;
                rs2_addr_o       = 0;
            end
            
            // ------------------------------------------------------------------
            // I-TYPE INSTRUCTIONS: JALR (Jump and Link Register)
            // Architecture: Absolute jump to an address stored inside a register.
            // ------------------------------------------------------------------
            `OPCODE_JALR: begin
                rd_wr_en_o       = (instruction_i[11:7] == 0) ? 0:1;
                jump_en_o        = 1;
                branch_op_o      = `REG_OFFSET; // Tells Branch Gen: Target = Register Value + Immediate
                alu_source_sel_o = 2'b10;        // Sets up ALU to calculate PC + 4 return address
                imm1_o           = pc_i; 
                imm2_o           = IMM_I; 
                rs2_addr_o       = 0;
            end
            
            // ------------------------------------------------------------------
            // LOAD INSTRUCTIONS (e.g., LW, LB, LH)
            // Architecture: Read data from external RAM into a register.
            // ------------------------------------------------------------------
            `OPCODE_LOAD: begin
                rd_wr_en_o       = (instruction_i[11:7] == 0) ? 0:1;
                alu_source_sel_o = 2'b01;  // ALU calculates memory address: rs1 + IMM_I
                imm2_o           = IMM_I;
                mem_rd_en_o      = 1;      // Activate data memory read channel
                memtoreg_o       = 1;      // Direct Writeback MUX to source data from RAM, not the ALU
                rs2_addr_o       = 0;
                
                case(FUNCT3)
                    `FUNCT3_LW:  mem_op_o = `MEM_LW;   // Load Word (32 bits)
                    `FUNCT3_LB:  mem_op_o = `MEM_LB;   // Load Byte (8 bits, signed extended)
                    `FUNCT3_LH:  mem_op_o = `MEM_LH;   // Load Halfword (16 bits, signed extended)
                    `FUNCT3_LBU: mem_op_o = `MEM_LB_U; // Load Byte Unsigned (8 bits, padded with 0s)
                    `FUNCT3_LHU: mem_op_o = `MEM_LH_U; // Load Halfword Unsigned (16 bits, padded with 0s)
                endcase
            end
            
            // ------------------------------------------------------------------
            // STORE INSTRUCTIONS (e.g., SW, SB, SH)
            // Architecture: Write data from a register out into external RAM.
            // ------------------------------------------------------------------
            `OPCODE_STORE: begin
                alu_source_sel_o = 2'b01;  // ALU calculates target memory address: rs1 + IMM_S
                imm2_o           = IMM_S;
                mem_wr_en_o      = 1;      // Activate data memory write enable
                
                case(FUNCT3)
                    `FUNCT3_SB: mem_op_o = `MEM_SB; // Store Byte (Write only 8 bits)
                    `FUNCT3_SH: mem_op_o = `MEM_SH; // Store Halfword (Write only 16 bits)
                    `FUNCT3_SW: mem_op_o = `MEM_SW; // Store Word (Write full 32 bits)
                endcase
            end
            
        endcase   
    end
    
endmodule