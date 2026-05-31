/*
================================================================================================
  MODULE ARCHITECTURE OVERVIEW: Pipeline Writeback Stage (WB Stage)
================================================================================================
  The Writeback stage is the 5th and final phase of the classic RISC-V pipeline execution. 
  Its sole purpose is to close the loop and commit computed data back into the CPU's core 
  Register File (x1 - x31).

  ARCHITECTURAL NUANCE: 
  Notice that there is no `always@(posedge clk_i)` block in this module. Because the Register 
  File itself is a sequential memory block that latches data on a clock edge, this WB stage 
  does not need its own output registers. 

  Instead, this module operates entirely as an unclocked, combinational multiplexer. It arbitrates 
  between computational results (ALU) and memory retrieval results (RAM) and routes the winner 
  back down the long feedback wires to the Register File input ports.
================================================================================================
*/

`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

module mpt_WB_stage
    (
    // =========================================================================================
    //                    FEEDBACK OUTPUTS (Routing backward to the Register File)
    // =========================================================================================
    output reg   [4:0]  WB_rd_addr_o,    // Target destination register address (tells the RegFile which x-register to overwrite)
    output reg   [31:0] WB_rd_wr_data_o, // The actual 32-bit data payload destined for the register file
    output reg          WB_rd_wr_en_o,   // Write Enable: Master authorization signal allowing the RegFile write to occur

    // =========================================================================================
    //                        PIPELINE INPUTS (Coming from the MEM stage)
    // =========================================================================================
    input  wire  [4:0]  MEM_rd_addr_i,   // Destination register index passed forward from the memory block
    input  wire  [31:0] MEM_dout_i,      // Formatted read data payload fresh out of the Data Memory (RAM)
    input  wire  [31:0] MEM_alu_result_i,// Calculation results or bypass address passed from the ALU
    input  wire         MEM_memtoreg_i,  // Control selector: 1 = Route RAM data, 0 = Route ALU math data
    input  wire         MEM_rd_wr_en_i   // Original write authorization bit decided way back in the Decoder
    );

    /*
    =============================================================================================
      WRITEBACK SOURCE ROUTING MATRIX
      An unclocked, continuous evaluation block that instantaneously updates when any input shifts.
    =============================================================================================
    */
    always@* begin
        // PASSTHROUGH: Forward the target destination register address without modification
        WB_rd_addr_o       = MEM_rd_addr_i;
        
        // PASSTHROUGH: Forward the write-enable gate signal straight through to the register file
        WB_rd_wr_en_o      = MEM_rd_wr_en_i;

        // CRITICAL INTERSECTION MULTIPLEXER: 
        // This multiplexer selects the final data payload based on the instruction classification:
        //   - If MEM_memtoreg_i == 1: The instruction is a LOAD (e.g., LW, LB, LH). Route the data 
        //     retrieved from RAM (MEM_dout_i).
        //   - If MEM_memtoreg_i == 0: The instruction is standard arithmetic/logic (e.g., ADD, SUB, 
        //     ORI, LUI) or a jump instruction saving a return address. Route the ALU calculation 
        //     result (MEM_alu_result_i).
        WB_rd_wr_data_o    = (MEM_memtoreg_i) ? MEM_dout_i : MEM_alu_result_i;
    end

endmodule