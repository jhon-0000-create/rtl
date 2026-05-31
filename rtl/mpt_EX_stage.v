`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

// ==============================================================================
// MODULE: mpt_EX_stage (Execute Stage Wrapper & Multiplexing Core)
//
// ROLE IN ARCHITECTURE:
// Evaluates ALU calculations, computes target addresses, and handles conditional 
// branch results. Structurally, it wraps the combinatorial `mpt_alu` and contains 
// the synchronous EX/MEM pipeline registers at its output boundary.
//
// HARDWARE MAPPING:
// Contains the forwarding bypass multiplexers for Operand A and Operand B, allowing
// calculations to resolve instantly when a data hazard is detected.
// ==============================================================================

module mpt_EX_stage
    `include "mpt_definitions.vh"
    (
    input  wire          clk_i,
    input  wire          resetn_i,
    input  wire          flush_i,               // Flushes this stage (converts current instruction to a NOP)
    output reg           EX_exception_o,

    // --- PIPELINE REGISTER OUTPUTS (Piped to MEM Stage) ---
    output reg           EX_mem_wr_en_o,     
    output reg           EX_mem_rd_en_o,
    output reg   [3:0]   EX_mem_op_o,
     
    output reg   [31:0]  EX_rs2_data_o,         // Store data passed forward to memory
    output reg           EX_memtoreg_o,
    output reg           EX_rd_wr_en_o,
    output reg   [4:0]   EX_rd_addr_o,
    output reg   [4:0]   EX_rs2_addr_o,

    output reg   [31:0]  EX_alu_result_o,       // Calculation output or memory address offset
     
    output reg   [31:0]  EX_pc_dest_o,          // Calculated target address for branches
    output reg           EX_branch_en_o,        // Core PC control signal: Asserted if branch is taken

    // --- PIPELINE INPUTS FROM DECODE (ID) ---
    input  wire          ID_mem_wr_en_i,
    input  wire          ID_mem_rd_en_i,
    input  wire  [3:0]   ID_mem_op_i,
    input  wire          ID_memtoreg_i, 
    input  wire          ID_rd_wr_en_i,   
    input  wire  [4:0]   ID_rd_addr_i,
    input  wire  [4:0]   ID_rs2_addr_i,
    
    // Control Paths for Branches & Jumps
    input  wire  [31:0]  ID_pc_dest_i,          // Calculated target branch address from ID stage
    input  wire  [1:0]   ID_branch_op_i,        // Bit [1] indicates structural check active
    input  wire          ID_branch_flag_i,      // 0 = branch on match; 1 = branch on inverted match
    input  wire          ID_jump_en_i,          // Indicates a JAL or JALR instruction
    
    // Interconnect Signals from Forwarding/Hazard Unit
    input  wire  [1:0]   forwardA_i,            // Multiplexer control for Operand 1 bypass
    input  wire  [1:0]   forwardB_i,            // Multiplexer control for Operand 2 bypass
    input  wire  [31:0]  WB_rd_wr_data_i,       // Data from the Writeback stage bus
    
    // ALU Specific Control Codes
    input  wire  [1:0]   ID_alu_source_sel_i,   // [1]=Immediate source for Op1; [0]=Immediate source for Op2
    input  wire  [3:0]   ID_alu_ctrl_i,        
    
    // Raw Decoded Operands
    input  wire  [31:0]  ID_pc_i,
    input  wire  [31:0]  ID_rs1_data_i,
    input  wire  [31:0]  ID_rs2_data_i,
    input  wire  [31:0]  ID_imm1_i,
    input  wire  [31:0]  ID_imm2_i,

    input  wire          ID_exception_i
    );
    
// ===========================================================================
//                    INTERCONNECT REGISTERS & WIRES
// =========================================================================== 
    reg  [31:0] alu_op1, alu_op2;  // Combinatorial variables routing the final selected operands
    wire [31:0] alu_result;       // Calculation return path from the ALU core
    wire        test_result;      // Boolean flag evaluation from the ALU core

// ===========================================================================
// SUB-MODULE INSTANTIATIONS
// =========================================================================== 
    mpt_alu alu_i (
    .alu_result_o   (alu_result    ),
    .test_result_o  (test_result   ),
    .alu_ctrl_i     (ID_alu_ctrl_i ),
    .alu_op1_i      (alu_op1       ),
    .alu_op2_i      (alu_op2       )
    );

// ===========================================================================
// BLOCK 1: EX/MEM PIPELINE BOUNDARY REGISTERS
// Hardware Mapping: Synchronous Flip-Flops with Synchronous Flushes
// =========================================================================== 
    always@(posedge clk_i) begin
        // Reset or Branch Control Flush condition
        if((resetn_i == 1'b0) || (flush_i)) begin 
            EX_mem_wr_en_o      <= 0;
            EX_mem_rd_en_o      <= 0;
            EX_mem_op_o         <= 0;
            EX_memtoreg_o       <= 0;
            EX_rd_addr_o        <= 0;
            EX_rd_wr_en_o       <= 0;
            EX_rs2_addr_o       <= 0;
            // DESIGN DETAIL: If a flush happens, keep the current target address intact
            // rather than destroying it, allowing the program counter fetch logic time to recover.
            EX_pc_dest_o        <= (flush_i) ? EX_pc_dest_o : 0;
            EX_rs2_data_o       <= 0;
            EX_exception_o      <= 0;
            EX_alu_result_o     <= 0;
        end
        else begin
            EX_mem_wr_en_o      <= ID_mem_wr_en_i;  
            EX_mem_rd_en_o      <= ID_mem_rd_en_i;  
            EX_mem_op_o         <= ID_mem_op_i;     
            EX_memtoreg_o       <= ID_memtoreg_i;   
            EX_rd_wr_en_o       <= ID_rd_wr_en_i;   
            EX_rd_addr_o        <= ID_rd_addr_i;   
            EX_rs2_addr_o       <= ID_rs2_addr_i;  
            EX_pc_dest_o        <= ID_pc_dest_i;
            EX_rs2_data_o       <= ID_rs2_data_i;   
            EX_exception_o      <= ID_exception_i;  
            EX_alu_result_o     <= alu_result;       // Captures computed ALU result
        end
    end
    
// ===========================================================================
// BLOCK 2: OPERAND A BYPASS SELECTOR MULTIPLEXER
// Hardware Mapping: Combinatorial 3-to-1 Multiplexer
// =========================================================================== 
    always@* begin
        if(ID_jump_en_i) begin 
            // Architectural Requirement for JAL/JALR instructions:
            // Force Operand A to pass down the Immediate Offset value (e.g., PC data).
            alu_op1 = ID_imm1_i;
        end
        else begin
            case(forwardA_i)
                // Case 2'b10: Intercept and forward data from the previous instruction's ALU calculation 
                // that is currently passing through the MEM stage boundary.
                2'b10: begin
                    alu_op1 = EX_alu_result_o;
                end
                
                // Case 2'b01: Intercept and forward data from an older instruction 
                // that is currently at the Writeback (WB) pipeline stage.
                2'b01: begin
                    alu_op1 = WB_rd_wr_data_i;
                end

                // Default (2'b00): No data dependency hazard found. 
                // Use standard decode signals: evaluate whether to use immediate data or register data.
                default: begin 
                    alu_op1 = (ID_alu_source_sel_i[1]) ? ID_imm1_i : ID_rs1_data_i;
                end
            endcase
        end
    end
    
// ===========================================================================
// BLOCK 3: OPERAND B BYPASS SELECTOR MULTIPLEXER
// Hardware Mapping: Combinatorial 3-to-1 Multiplexer
// =========================================================================== 
    always@* begin
        if(ID_jump_en_i == 1) begin 
            // Architectural Requirement for JAL/JALR instructions:
            // Force Operand B to constant decimal value 4. 
            // Combined with Block 2, this instructs the ALU to execute (PC + 4) 
            // to store the return linkage address in the register file.
            alu_op2 = 32'd4;
        end
        else begin
            case(forwardB_i)
                // Case 2'b10: Forward calculation output from the adjacent MEM stage boundary.
                2'b10: begin
                    alu_op2[31:0] = EX_alu_result_o;
                end
                
                // Case 2'b01: Forward data returning from the Writeback stage bus.
                2'b01: begin
                    alu_op2[31:0] = WB_rd_wr_data_i;
                end

                // Default (2'b00): No data dependency hazard found.
                // Select between the decoded 32-bit immediate or standard register source 2 data.
                default: begin 
                    alu_op2[31:0] = (ID_alu_source_sel_i[0]) ? ID_imm2_i : ID_rs2_data_i;
                end
            endcase
        end
    end
    
// ===========================================================================
// BLOCK 4: CONDITIONAL BRANCH VALIDATION EVALUATOR
// Hardware Mapping: Combinational Logic Evaluation sampled by output register
// =========================================================================== 
    always@(posedge clk_i) begin
        // Check if an active control condition requires a branch test evaluate
        if(ID_branch_op_i[1]) begin
            if(ID_jump_en_i) begin
                // Inconditiional jumps (JAL/JALR) handle routing targets instantly 
                // within the Fetch/Decode stages. Turn off structural execution checks.
                EX_branch_en_o <= 0;
            end
            else begin
                // Match condition evaluation (e.g., BEQ, BGE)
                if(ID_branch_flag_i == 1'b0) begin
                    EX_branch_en_o <= (test_result) ? 1 : 0;            
                end
                // Inverted match condition evaluation (e.g., BNE, BLT)
                else begin
                    EX_branch_en_o <= (test_result) ? 0 : 1;  
                end
            end 
        end
        else begin
            // Structural Fallback: Clear branch signal for non-branch instructions
            EX_branch_en_o <= 0;
        end 
    end 

endmodule