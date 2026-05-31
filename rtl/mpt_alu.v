`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

// ==============================================================================
// MODULE: mpt_alu (Arithmetic Logic Unit)
//
// ROLE IN ARCHITECTURE: 
// This is the "calculator" of the CPU. It performs all the math (addition, 
// subtraction) and logic (AND, OR, shifts) required by the instruction. 
//
// HARDWARE MAPPING: 
// This module synthesizes into PURE COMBINATIONAL LOGIC. There are no clocks, 
// no memory, and no flip-flops here. In physical silicon, this becomes a giant 
// cluster of adders, subtractors, and logic gates, all wired into a massive 
// Multiplexer (MUX). The 'alu_ctrl_i' signal acts as the selector switch for 
// that MUX, deciding which answer gets passed to the output wire.
// ==============================================================================

module mpt_alu
    `include "mpt_definitions.vh" // Pulls in the dictionary of macro definitions (like `ALU_ADD)
    (
    // OUTPUTS
    // Note for beginners: 'reg' in Verilog does NOT always mean a physical hardware register (flip-flop).
    // Inside an 'always @*' block, 'reg' simply means "a variable that gets assigned a value."
    // In hardware, these are just physical copper wires carrying the final answer.
    output reg  [31:0]     alu_result_o,   // The 32-bit answer of the math/logic operation
    output reg             test_result_o,  // A 1-bit True/False flag used ONLY for Branch instructions

    // INPUTS
    input  wire [3:0]      alu_ctrl_i,  // The 4-bit instruction from the Decoder. Tells the ALU what to do (e.g., 0000 = ADD)
    input  wire [31:0]     alu_op1_i,   // Operand 1: Usually the data from Register 1 (rs1)
    input  wire [31:0]     alu_op2_i    // Operand 2: Usually the data from Register 2 (rs2), OR an Immediate value
    );

    // ==============================================================================
    // BLOCK 1: MAIN ARITHMETIC & LOGIC
    // Hardware: A massive Multiplexer (MUX) connected to dedicated hardware logic blocks.
    // ==============================================================================
    
    // 'always @*' tells the synthesizer: "Update the output instantly whenever ANY input changes."
    always@* begin
        
        // DEFAULT ASSIGNMENT (CRITICAL FOR HARDWARE):
        // If we don't give the output a default value, the synthesizer thinks the circuit 
        // needs to "remember" the old value. It will accidentally build physical memory 
        // (a Latch) which causes timing errors. Defaulting to 0 prevents latches.
        alu_result_o = 0;
        
        // The 'case' statement synthesizes into a Multiplexer (MUX).
        case(alu_ctrl_i)
            
            // --- ARITHMETIC ---
            // These synthesize into dedicated physical arithmetic circuits.
            `ALU_ADD:  alu_result_o = alu_op1_i + alu_op2_i; // Synthesizes a 32-bit Ripple-Carry or Lookahead Adder
            `ALU_SUB:  alu_result_o = alu_op1_i - alu_op2_i; // Synthesizes an Adder using Two's Complement inversion
            
            // --- BITWISE LOGIC ---
            // These synthesize into parallel arrays of 32 individual logic gates.
            `ALU_AND:  alu_result_o = alu_op1_i & alu_op2_i; // 32 AND gates
            `ALU_OR:   alu_result_o = alu_op1_i | alu_op2_i; // 32 OR gates
            `ALU_XOR:  alu_result_o = alu_op1_i ^ alu_op2_i; // 32 XOR gates

            // --- SHIFTS ---
            // These synthesize into "Barrel Shifters" (complex webs of MUXes that shift bits instantly).
            // WHY [4:0]?: This is a 32-bit CPU. The maximum amount you can shift a 32-bit number is 31 spaces. 
            // 31 in binary is 11111 (5 bits). Therefore, the ALU only looks at the bottom 5 bits of op2.
            
            `ALU_SLL:  alu_result_o = alu_op1_i << alu_op2_i[4:0];  // Shift Left Logical (Pushes 0s in from the right)
            `ALU_SRL:  alu_result_o = alu_op1_i >> alu_op2_i[4:0];  // Shift Right Logical (Pushes 0s in from the left)
            
            // WHY $signed()?: This is an Arithmetic Shift Right. If the number is negative (starts with a 1),
            // a logical shift would push in 0s, accidentally turning it positive. $signed() tells the hardware 
            // to push in 1s instead, preserving the negative sign.
            `ALU_SRA:  alu_result_o = $signed(alu_op1_i) >>> alu_op2_i[4:0];
            `ALU_SLT:  alu_result_o = ($signed(alu_op1_i) < $signed(alu_op2_i)) ? 32'd1 : 32'd0;
            `ALU_SLTU: alu_result_o = (alu_op1_i < alu_op2_i) ? 32'd1 : 32'd0;

            default: alu_result_o = 0;
        endcase
    end

    // ==============================================================================
    // BLOCK 2: BRANCH COMPARATORS
    // Hardware: Dedicated comparison circuits (XNOR gates for equality, subtractors for less-than).
    // Role: This block ONLY matters when the CPU is trying to decide if it should take a branch (like BEQ).
    // ==============================================================================
    
    always@* begin
        // Default to False (0) to prevent hardware latches
        test_result_o = 0;
        
        case(alu_ctrl_i) 
            // Set Equal (SEQ): Are the two numbers identical? Synthesizes to a 32-bit equality checker.
            `ALU_SEQ:  test_result_o = (alu_op1_i == alu_op2_i) ? 1:0; 
            
            // Set Less Than (SLT - Signed): Checks if op1 < op2, assuming they can be negative.
            // Ex: -5 < 2 is TRUE.
            `ALU_SLT:  test_result_o = ($signed(alu_op1_i) < $signed(alu_op2_i)) ? 1:0; 
            
            // Set Less Than Unsigned (SLTU): Checks if op1 < op2, ignoring negative signs (treating them as massive positive numbers).
            `ALU_SLTU: test_result_o = (alu_op1_i < alu_op2_i) ? 1:0;
        endcase
    end
    
endmodule