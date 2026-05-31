`timescale 1ns / 1ps
`default_nettype none
// ==============================================================================
// MODULE: mpt_ID_stage (Instruction Decode Stage Wrapper)
//
// ROLE IN ARCHITECTURE: 
// This is a structural wrapper that combines the Decoder and the Register File. 
// It captures raw inputs from the Fetch stage (IF) and holds the synchronous 
// registers that form the boundary between the Decode (ID) and Execute (EX) stages.
//
// HARDWARE MAPPING:
// Contains both combinational blocks (Decoder) and edge-triggered storage elements.
// The output ports prefixed with `ID_` represent physical flip-flops that sample
// their inputs at the rising edge of the clock, acting as a pipeline buffer.
// ==============================================================================

`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

module mpt_ID_stage
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
    input  wire                            clk_i,
    input  wire                            resetn_i,
    input  wire                            flush_i,          // Flushes this stage (converts current instruction to NOP)
    input  wire                            stall_i,          // Freezes pipeline progression due to hazards

    // --- PIPELINE REGISTER OUTPUTS (Piped to EX Stage) ---
    output reg   [REG_DATA_WIDTH-1:0]      ID_pc_o,          // Floped PC value associated with this instruction

    // Control Signals
    output reg   [1:0]                     ID_alu_source_sel_o, 
    output reg   [ALU_OP_WIDTH-1 :0]       ID_alu_ctrl_o,       
    output reg   [1:0]                     ID_branch_op_o,      
    output reg                             ID_branch_flag_o,    
    output reg                             ID_mem_wr_en_o,      
    output reg                             ID_mem_rd_en_o,      
    output reg                             ID_rd_wr_en_o,       
    output reg                             ID_memtoreg_o,       
    output reg                             ID_jump_en_o,        
    output reg   [3:0]                     ID_mem_op_o,         
   
    // Operand Data
    output reg   [REG_DATA_WIDTH-1 :0]     ID_imm1_o,
    output reg   [REG_DATA_WIDTH-1 :0]     ID_imm2_o,
    output reg   [REG_DATA_WIDTH-1 :0]     ID_rs1_data_o,    // Final data for Operand 1 (resolved for WB hazards)
    output reg   [REG_DATA_WIDTH-1 :0]     ID_rs2_data_o,    // Final data for Operand 2 (resolved for WB hazards)
  
    // Register Addresses
    output reg   [REGFILE_ADDR_WIDTH-1:0]  ID_rd_addr_o,
    output reg   [REGFILE_ADDR_WIDTH-1:0]  ID_rs1_addr_o,
    output reg   [REGFILE_ADDR_WIDTH-1:0]  ID_rs2_addr_o,

    output reg                             ID_exception_o,

    // --- INPUTS FROM FETCH (IF) STAGE ---
    input  wire  [REG_DATA_WIDTH-1:0]      IF_pc_i,          // Incoming PC value
    input  wire  [REG_DATA_WIDTH-1:0]      IF_instruction_i, // Incoming 32-bit machine instruction
    
    // --- GHOST INPUTS (Forwarding Hooks) ---
    // Note: These inputs are structurally declared but unused within this specific wrapper file.
    // They are hooks typically preserved for evaluating early branch/jump target calculations.
    input  wire                            EX_rd_wr_en_i,
    input  wire  [REGFILE_ADDR_WIDTH-1:0]  EX_rd_addr_i,
    input  wire  [REG_DATA_WIDTH-1:0]      EX_alu_result_i,

    // --- INPUTS FROM WRITEBACK (WB) STAGE ---
    // These carry the final data being written back into the Register File this cycle.
    input  wire  [REGFILE_ADDR_WIDTH-1:0]  WB_rd_addr_i,
    input  wire  [REG_DATA_WIDTH-1:0]      WB_rd_wr_data_i,
    input  wire                            WB_rd_wr_en_i
    );
    
// ===========================================================================
//                    INTERCONNECT REGISTERS & WIRES
// ===========================================================================
    // Local intermediate wires connecting the Decoder outputs to the Pipeline inputs
    wire [4:0]  rd_addr;
    wire [4:0]  rs1_addr;
    wire [4:0]  rs2_addr;

    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

    reg [31:0]  rs1_rd_data;
    reg [31:0]  rs2_rd_data;
    
    wire [31:0] imm1;
    wire [31:0] imm2;
    
    wire [1:0]  alu_source_sel;
    wire [3:0]  alu_ctrl;
    wire [1:0]  branch_op;
    wire        branch_flag;
    wire        mem_wr_en;
    wire        mem_rd_en;
    wire        rd_wr_en;
    wire        memtoreg;
    wire        jump_en;
    wire [3:0]  mem_op;
    wire        exception;

    // Local flags to evaluate if a Writeback-to-Decode hazard exists
    wire        rd_rs1_match; 
    wire        rd_rs2_match;


// ===========================================================================
// SUB-MODULE INSTANTIATIONS
// ===========================================================================    

    // Instance 1: The Combinational Decoder Core
    mpt_decoder decoder_i(
    .rd_addr_o         (rd_addr           ),
    .rs1_addr_o        (rs1_addr          ),
    .rs2_addr_o        (rs2_addr          ),
    .imm1_o            (imm1              ),
    .imm2_o            (imm2              ),
    .alu_source_sel_o  (alu_source_sel    ),
    .alu_op_o          (alu_ctrl          ),
    .branch_op_o       (branch_op         ),
    .branch_flag_o     (branch_flag       ),
    .mem_wr_en_o       (mem_wr_en         ),
    .mem_rd_en_o       (mem_rd_en         ),
    .rd_wr_en_o        (rd_wr_en          ),
    .memtoreg_o        (memtoreg          ),
    .jump_en_o         (jump_en           ),
    .mem_op_o          (mem_op            ),
    .exception_o       (exception         ),
    .instruction_i     (IF_instruction_i  ),
    .pc_i              (IF_pc_i           )
    );
    
    // Instance 2: The Core CPU Register File Array
    mpt_regfile regfile_i(
    .clk_i             (clk_i             ),
    .resetn_i          (resetn_i          ),
    .rs1_data_o        (rs1_data          ),
    .rs2_data_o        (rs2_data          ),
    .rs1_addr_i        (rs1_addr          ),
    .rs2_addr_i        (rs2_addr          ),
    .rd_addr_i         (WB_rd_addr_i      ),
    .rd_wr_data_i      (WB_rd_wr_data_i   ),
    .rd_wr_en_i        (WB_rd_wr_en_i     )
    );
    
    
// ===========================================================================
// PIPELINE REGISTER IMPLEMENTATION
// ===========================================================================    
    
    // -----------------------------------------------------------------------
    // BLOCK A: Data & Address Pipeline Latches
    // Driven directly by the clock edge.
    // -----------------------------------------------------------------------
    always@(posedge clk_i) begin
        if(resetn_i == 1'b0) begin
            ID_rd_addr_o        <= 0;
            ID_rs1_addr_o       <= 0;
            ID_rs2_addr_o       <= 0;
            ID_imm1_o           <= 0;
            ID_imm2_o           <= 0;
            ID_exception_o      <= 0;
        end
        else begin
            ID_rd_addr_o        <= rd_addr;
            ID_rs1_addr_o       <= rs1_addr;
            ID_rs2_addr_o       <= rs2_addr;
            ID_imm1_o           <= imm1;
            ID_imm2_o           <= imm2;
            ID_exception_o      <= exception;
        end
    end

    // -----------------------------------------------------------------------
    // BLOCK B: Control Path Pipeline Latches & Bubble Injection Logic
    // This is the gatekeeper block. It dictates what happens during system flushes
    // and microarchitectural stalls.
    // -----------------------------------------------------------------------
    always@(posedge clk_i) begin
        // Condition 1: Reset or Control Hazard Flush
        // Instantly wipe all active control signals to 0. This turns the executing
        // instruction into a NOP, protecting state variables downstream.
        if((resetn_i == 1'b0) || (flush_i)) begin
            ID_pc_o             <= 0;
            ID_alu_source_sel_o <= 0;
            ID_alu_ctrl_o       <= 0;
            ID_branch_op_o      <= 0;
            ID_branch_flag_o    <= 0;
            ID_mem_wr_en_o      <= 0;
            ID_mem_rd_en_o      <= 0;
            ID_rd_wr_en_o       <= 0;
            ID_memtoreg_o       <= 0;
            ID_jump_en_o        <= 0;
            ID_mem_op_o         <= 0;
        end
        else begin
            // Condition 2: Load-Use Hazard Stall Active
            // CRITICAL HARDWARE DESIGN: When a stall happens, we keep non-destructive
            // fields (like PC, ALU operations, etc.) frozen at their current values.
            // However, we FORCE state-changing enables (`mem_wr_en`, `mem_rd_en`, `rd_wr_en`)
            // to 0! This passes a safe, non-destructive "bubble" into the EX/MEM stages
            // while the instruction here waits for data to become ready.
            if(stall_i) begin
                ID_pc_o             <= ID_pc_o;
                ID_alu_source_sel_o <= ID_alu_source_sel_o;
                ID_alu_ctrl_o       <= ID_alu_ctrl_o;
                ID_branch_op_o      <= ID_branch_op_o;
                ID_branch_flag_o    <= ID_branch_flag_o;
                ID_mem_wr_en_o      <= 0;  // Kill write enable -> structural bubble
                ID_mem_rd_en_o      <= 0;  // Kill read enable  -> structural bubble
                ID_rd_wr_en_o       <= 0;  // Kill register file writeback -> structural bubble
                ID_memtoreg_o       <= ID_memtoreg_o;
                ID_jump_en_o        <= 0;
                ID_mem_op_o         <= ID_mem_op_o;
            end
            // Condition 3: Standard execution step
            else begin
                ID_pc_o             <= IF_pc_i;
                ID_alu_source_sel_o <= alu_source_sel;
                ID_alu_ctrl_o       <= alu_ctrl;
                ID_branch_op_o      <= branch_op;
                ID_branch_flag_o    <= branch_flag;
                ID_mem_wr_en_o      <= mem_wr_en; 
                ID_mem_rd_en_o      <= mem_rd_en;
                ID_rd_wr_en_o       <= rd_wr_en;
                ID_memtoreg_o       <= memtoreg;
                ID_jump_en_o        <= jump_en;
                ID_mem_op_o         <= mem_op;
            end
        end  
    end


    // -----------------------------------------------------------------------
    // BLOCK C: Internal Clock-Edge Synchronization Latches
    // When the instruction memory outputs data, it is sampled immediately. 
    // We register the data output from the RegFile array to hold it stable 
    // for evaluation over the next full execution clock window.
    // -----------------------------------------------------------------------
    always@(posedge clk_i) begin
        if(resetn_i == 1'b0) begin
            rs1_rd_data         <= 0;
            rs2_rd_data         <= 0;
        end else begin
            rs1_rd_data         <= rs1_data;
            rs2_rd_data         <= rs2_data;
        end
    end

    // -----------------------------------------------------------------------
    // BLOCK D: Writeback-to-Decode Internal Forwarding Bypass
    // Hardware Mapping: Combinational 2-to-1 Multiplexers.
    //
    // THE RAW HAZARD FIX:
    // Because the internal register file samples data on the clock edge, a conflict
    // occurs if an old instruction is writing a value to register x5 at the exact 
    // same moment the current instruction is reading from x5. 
    //
    // Without this block, the instruction would read the *stale* (old) register value 
    // since the new data hasn't been physically written into the array yet.
    //
    // This logic checks if the Writeback target register (`WB_rd_addr_i`) matches 
    // our active source addresses (`ID_rs1_addr_o`), ensures we aren't matching 
    // register x0 (`& |ID_rs1_addr_o`), and combinationally intercepts the stale data, 
    // replacing it with the incoming Writeback data bus instantly!
    // -----------------------------------------------------------------------
    assign rd_rs1_match = (WB_rd_addr_i == ID_rs1_addr_o) & |ID_rs1_addr_o;
    assign rd_rs2_match = (WB_rd_addr_i == ID_rs2_addr_o) & |ID_rs2_addr_o;

    always@* begin
        ID_rs1_data_o = (rd_rs1_match && WB_rd_wr_en_i) ? WB_rd_wr_data_i : rs1_rd_data;
        ID_rs2_data_o = (rd_rs2_match && WB_rd_wr_en_i) ? WB_rd_wr_data_i : rs2_rd_data;
    end
    
endmodule