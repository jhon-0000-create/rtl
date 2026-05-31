`ifdef CUSTOM_DEFINE
    `include "../defines.vh"
`endif

module mpt_IF_stage
    `include "mpt_definitions.vh"
    // Parameterized design allows easily swapping between 32-bit and 64-bit architectures
    `ifdef CUSTOM_DEFINE
        #(parameter REG_DATA_WIDTH  = `REG_DATA_WIDTH
          parameter IMEM_ADDR_WIDTH = `ADDR_DATA_WIDTH)
    `else
        #(parameter REG_DATA_WIDTH = 32,   // Width of data buses (32-bit for RV32I)
          parameter IMEM_ADDR_WIDTH = 32)  // Width of memory address buses
    `endif

    (
    input  wire                               clk_i,
    input  wire                               resetn_i,

    // Outputs feeding the memory system and the next pipeline stage (Decode)
    output reg   [IMEM_ADDR_WIDTH-1:0]        IMEM_addr_o,      // The actual PC register output
    output reg   [REG_DATA_WIDTH-1:0]         IF_instruction_o, // Instruction passed to ID stage
    output reg   [REG_DATA_WIDTH-1:0]         IF_pc_o,          // PC value passed to ID stage
  
    // Inputs coming from memory, compiler configurations, and later pipeline stages
    input  wire  [REG_DATA_WIDTH-1:0]         IMEM_data_i,      // Raw instruction from memory
    input  wire  [IMEM_ADDR_WIDTH-1:0]        boot_addr_i,      // Where to start on reset
    input  wire                               EX_branch_en_i,   // High if branch condition met in EX
    input  wire  [REG_DATA_WIDTH-1:0]         EX_pc_dest_i,     // Branch target address from EX
   
    input  wire                               ID_jump_en_i,     // High if unconditional jump found in ID
    input  wire  [REG_DATA_WIDTH-1:0]         BG_pc_dest_i,     // Jump target address from ID
 
    input  wire                               stall_i,          // High if pipeline must freeze (e.g., load-use hazard)
    input  wire                               flush_i           // High if pipeline must clear (e.g., mispredicted branch)
    );


// ===========================================================================
//                    Parameters, Registers, and Wires
// ===========================================================================    
    // Combinational net used to calculate what the next PC value should be
    // This maps directly to the input of our main PC register Multiplexer.
    reg  [31:0]  pc_next;

// ===========================================================================
//                              Implementation    
// ===========================================================================    

    // HARDWARE MAPPING: A 4-to-1 Multiplexer (MUX) selecting the next PC.
    // This block evaluates combinationally (routes signals instantly based on priority).
    always@* begin
        if      (ID_jump_en_i)    pc_next = BG_pc_dest_i;    // Priority 1: Unconditional Jump taken
        else if (EX_branch_en_i)  pc_next = EX_pc_dest_i;    // Priority 2: Conditional Branch taken
        // SPECIAL QUIRK: Because IMEM_addr_o updates sequentially every clock edge,
        // if a stall happens, the processor has already mistakenly advanced.
        // To fix this, it subtracts 4 to "rewind" and re-fetch the stalled instruction.
        else if (stall_i)         pc_next = IMEM_addr_o - 4; 
        else                      pc_next = IMEM_addr_o + 4; // Normal case: Fetch next sequential instruction
    end

    // HARDWARE MAPPING: Sequential D-Flip-Flops with synchronous/asynchronous control.
    // This block updates state ONLY on the rising edge of the clock.
    always@(posedge clk_i) begin
        if(resetn_i == 1'b0) begin
            IMEM_addr_o      <= boot_addr_i; // Force PC to the boot address (e.g., 0x00000000)
            IF_pc_o          <= 0;           // Clear out the tracking PC
            
        end
        else begin
            IMEM_addr_o      <= pc_next;     // Clock the calculated next PC into the hardware register
            
            // If stalling, hold the current tracking PC value steady. 
            // Otherwise, pass the current memory address down to the Decode stage.
            IF_pc_o          <= (stall_i) ? IF_pc_o : IMEM_addr_o;
            
        end  
    end


    // HARDWARE MAPPING: A 2-to-1 Multiplexer handling pipeline flushes.
    // If a flush occurs (like a bad branch guess), it overwrites the instruction with 0.
    // In RISC-V, 32'h00000000 maps to "addi x0, x0, 0", which is a hardwired NOP (No Operation).
    always@* begin
        if(flush_i) begin
            IF_instruction_o = 0;             // Insert a NOP to clear the stage
        end else begin
            IF_instruction_o = IMEM_data_i;   // Pass the actual instruction from memory through
        end
    end

endmodule
