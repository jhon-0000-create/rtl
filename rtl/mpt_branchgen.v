`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

// ==============================================================================
// MODULE: mpt_branchgen (Branch Target Generator)
//
// ROLE IN ARCHITECTURE: 
// When the CPU reads a jump (JAL, JALR) or a branch (BEQ, BNE), it needs to 
// know EXACTLY what memory address to go to next. This module calculates that 
// destination address.
//
// WHY DOES IT EXIST?: 
// You might ask: "Why not just use the main ALU to calculate the jump address?"
// Because for instructions like BEQ (Branch if Equal), the main ALU is already 
// busy checking if op1 == op2! The CPU needs a dedicated, separate adder to 
// calculate the target address simultaneously so no time is wasted.
//
// HARDWARE MAPPING: 
// Pure Combinational Logic. Synthesizes into a 32-bit Adder and two Multiplexers (MUXes).
// ==============================================================================

module mpt_branchgen
    `include "mpt_definitions.vh"
    (
    output reg  [31:0] pc_dest_o,    // The final calculated 32-bit memory address to jump to.

    // CONTROL INPUT
    // A 2-bit wire from the Decoder. 
    // Bit [1] High = JALR (Jump based on a Register value).
    // Bit [0] High = JAL or Branch (Jump based on Current PC).
    input  wire [1:0]  ID_branch_op_i,

    // FORWARDING INPUTS (The "Hazard" Detectors)
    input  wire [4:0]  ID_rs1_addr_i,   // Which register does the jump rely on? (e.g., x5)
    input  wire [4:0]  EX_rd_addr_i,    // Which register is the EX stage currently modifying?
    input  wire        EX_rd_wr_en_i,   // Is the EX stage actually writing to a register right now?
    
    // OPERANDS (The numbers to add together)
    input  wire [31:0] ID_pc_i,         // The current Program Counter (where we are now)
    input  wire [31:0] ID_rs1_data_i,   // The "stale" register data straight from the RegFile
    input  wire [31:0] ID_imm2_i,       // The immediate offset (e.g., "Jump forward 16 bytes")
    input  wire [31:0] EX_alu_result_i  // The "fresh" data just calculated by the ALU in the EX stage
    );

    // Internal wire to hold the correct register data
    reg  [31:0] regdata;

    // ==============================================================================
    // BLOCK 1: LOCAL FORWARDING MUX
    // Hardware: A 32-bit 2-to-1 Multiplexer controlled by a logic comparator.
    //
    // THE PROBLEM: Read-After-Write (RAW) Hazard.
    // Imagine this assembly code:
    //    ADD x5, x1, x2   (Instruction 1: Currently in EX stage, calculating the answer)
    //    JALR x0, x5, 0   (Instruction 2: Currently in ID stage, trying to jump to address in x5)
    //
    // Instruction 2 needs the value of x5 to jump. But the RegFile hasn't been 
    // updated yet because Instruction 1 is still in the pipeline! If we just read 
    // from the RegFile, we will jump to old, garbage data.
    //
    // THE SOLUTION: Forwarding.
    // We look at the ALU output wire from the EX stage (`EX_alu_result_i`). If we 
    // realize the EX stage is currently calculating the exact register we need, we 
    // snatch that data mid-air instead of waiting for it to go to the RegFile.
    // ==============================================================================
    always@* begin
        if((ID_branch_op_i[1]) &&             // Are we doing a JALR? (Which requires a register)
           (EX_rd_addr_i == ID_rs1_addr_i) && // Does the EX stage destination match our source?
           (EX_rd_wr_en_i))                   // Is the EX stage actually writing valid data?
           
            regdata = EX_alu_result_i;        // HAZARD CAUGHT: Route the MUX to the "Fresh" ALU data
        else 
            regdata = ID_rs1_data_i;          // NO HAZARD: Route the MUX to the normal RegFile data
    end

    // ==============================================================================
    // BLOCK 2: TARGET ADDRESS ADDER
    // Hardware: A 32-bit Adder and a 2-to-1 Multiplexer.
    //
    // RISC-V defines two ways to calculate where to jump:
    // 1. PC-Relative: "Go 40 bytes forward from where I am right now." (Current PC + Offset)
    // 2. Register-Offset: "Go to the absolute memory address stored in x5." (Register + Offset)
    // ==============================================================================
    always@* begin
        // Default to prevent hardware latches
        pc_dest_o = 0;
        
        case(ID_branch_op_i)
            // PC_RELATIVE (Used for standard Branches and JAL)
            // Hardware Adder connects to the `ID_pc_i` wire.
            // Note the $signed(): If the immediate is negative, we jump backwards in code (like a loop!).
            `PC_RELATIVE: pc_dest_o = ID_pc_i + $signed(ID_imm2_i);
            
            // REG_OFFSET (Used exclusively for JALR - Jump and Link Register)
            // Hardware Adder connects to our `regdata` wire (which we secured in Block 1).
            `REG_OFFSET:  pc_dest_o = regdata + $signed(ID_imm2_i);
        endcase
    end
    
endmodule