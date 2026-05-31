`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

// ==============================================================================
// MODULE: mpt_regfile (2-Read / 1-Write General Purpose Register File)
//
// ROLE IN ARCHITECTURE: 
// Houses the 32 architectural registers ($x0 - $x31). It exposes two fully 
// independent combinational read ports to fetch operands, and one sequential 
// write port to save results from the Writeback (WB) stage.
//
// MICROARCHITECTURAL HAZARD MITIGATION:
// Contains a fundamental design pattern: **Internal Write-to-Read Forwarding Bypass**.
// If an instruction is updating register $x3$ at the exact same instant a new 
// instruction is reading $x3$, this module intercepts the stale data sitting in 
// the memory array and feeds the fresh write data straight out to the outputs.
// ==============================================================================

module mpt_regfile
    `include "mpt_definitions.vh"
    `ifdef CUSTOM_DEFINE
        #(parameter REG_DATA_WIDTH      = `REG_DATA_WIDTH,
          parameter REGFILE_ADDR_WIDTH  = `REGFILE_ADDR_WIDTH,
          parameter REGFILE_DEPTH       = `REGFILE_DEPTH
          )
    `else
        #(parameter REG_DATA_WIDTH      = 32, // Width of each register (32-bit CPU)
          parameter REGFILE_ADDR_WIDTH  = 5,  // 2^5 = 32 registers total
          parameter REGFILE_DEPTH       = 32
          )
    `endif
    
    (
    input  wire                              clk_i,
    input  wire                              resetn_i,

    // --- READ PORTS (Combinational Outputs) ---
    output wire    [REG_DATA_WIDTH-1 :0]     rs1_data_o,   // Contents of register chosen by rs1_addr_i
    output wire    [REG_DATA_WIDTH-1 :0]     rs2_data_o,   // Contents of register chosen by rs2_addr_i
    
    // --- READ ADDRESSES ---
    input  wire    [REGFILE_ADDR_WIDTH-1 :0] rs1_addr_i, 
    input  wire    [REGFILE_ADDR_WIDTH-1 :0] rs2_addr_i,
 
    // --- WRITE PORT (Synchronous Inputs from Writeback Stage) ---
    input  wire    [REGFILE_ADDR_WIDTH-1 :0] rd_addr_i,     // Register index to overwrite
    input  wire    [REG_DATA_WIDTH-1 :0]     rd_wr_data_i,  // Actual 32-bit data to save
    input  wire                              rd_wr_en_i     // Write enable command flag
    );

// ===========================================================================
//                    Memory Arrays and Internal Wires
// ===========================================================================    
    // Hardware Structure: This models a 2D array of registers. 
    // 32 rows total, each row being 32 bits wide.
    reg [REG_DATA_WIDTH-1:0] regfile_data [0: REGFILE_DEPTH-1];
    
    // This wire explicitly references `resetn_i` to satisfy logic compilers. 
    // It prevents the EDA tool from optimizing out the reset net during RAM inference.
    wire unused_reset = resetn_i; 
    
// ===========================================================================
// BLOCK 1: INTERNAL WRITE-TO-READ FORWARDING (INTERNAL BYPASS)
// Hardware Mapping: Pair of 32-bit Wide 2-to-1 Multiplexers
// ===========================================================================  
    
    // THE SYNCHRONIZATION PROBLEM:
    // Memory writes are edge-triggered (sequential). Memory reads are wires (combinational).
    // If the instruction in Writeback is saving data to $x5, that data will not settle 
    // into the internal storage array until the NEXT positive clock edge. 
    // If the instruction in Decode needs to read $x5 *this* cycle, it will read stale 
    // data because the array has not updated yet.
    //
    // THE SOLUTION (Lookahead Bypass):
    // The continuous assignments below check for three criteria simultaneously:
    //   1. Is the read register address equal to the incoming write register address? (`rs1_addr_i == rd_addr_i`)
    //   2. Are we trying to read something other than register 0? (`rd_addr_i != 0`)
    //   3. Is the Writeback stage actually writing this cycle? (`rd_wr_en_i`)
    //
    // If all three conditions are true, the multiplexer completely bypasses the memory array, 
    // routing `rd_wr_data_i` directly to the output port.
    
    assign rs1_data_o = ((rs1_addr_i == rd_addr_i) &&
                         (rd_addr_i != 0) &&
                         (rd_wr_en_i))  
                         ? rd_wr_data_i : regfile_data[rs1_addr_i];
    
    assign rs2_data_o = ((rs2_addr_i == rd_addr_i) &&
                         (rd_addr_i != 0) &&
                         (rd_wr_en_i))     
                         ? rd_wr_data_i : regfile_data[rs2_addr_i];

// ===========================================================================
// BLOCK 2: SYSTEM INITIALIZATION (Simulation Behavior Only)
// ===========================================================================  
    integer i;

    // This initialization loop is for simulation environments (like ModelSim/Vivado Simulator). 
    // It prevents registers from starting as undefined ('U' or 'X') states. 
    // Note: This initial block does NOT synthesize into real hardware gates on an ASIC.
    initial begin
        for(i=0; i<REGFILE_DEPTH-1; i=i+1) begin
            regfile_data[i] <= 0;
        end      
    end
    
// ===========================================================================
// BLOCK 3: SYNCHRONOUS REGISTRATION & STORAGE ARRAYS
// Hardware Mapping: RAM32M Distributed RAM Macro Cells
// ===========================================================================  
    always@(posedge clk_i) begin
        
        /*
        // --- APPROACH A: FLIP-FLOP SYNTHESIS ---
        // If you un-commented this block, the compiler would synthesize 1,024 individual 
        // D-Flip-Flops (32 registers * 32 bits) alongside massive multiplexer selection trees.
        // This is perfectly functional but consumes an enormous amount of physical silicon real estate.
        if(resetn_i == 1'b0) begin
            for(i=0; i<REGFILE_DEPTH-1; i=i+1) begin
                regfile_data[i] <= 0;
            end       
        end
        else begin    
            if((rd_wr_en_i) && (rd_addr_i != 0)) begin
                regfile_data[rd_addr_i] <= rd_wr_data_i;
            end 
        end 
        */

        // --- APPROACH B: RAM32M INFERENCE (Active) ---
        // By removing the asynchronous reset loop from the block, modern synthesis engines 
        // (like Xilinx Vivado) will automatically map this memory structure directly onto 
        // specialized LUT hardware macros known as **Distributed RAM32M primitives**.
        //
        // This maps multiple multi-ported register bits into single hardware slice structures, 
        // drastically accelerating the layout, minimizing propagation delay, and saving 
        // valuable chip real estate.
        //
        // HARDWARE LOGIC RULE: **Register $x0 is Hardwired to Zero**.
        // Notice the safety condition `(rd_addr_i != 0)`. The RISC-V specification dictates 
        // that register 0 must ALWAYS return 0. If an instruction attempts to write a calculation 
        // out to register 0, this module ignores the write enable flag completely.
        
        if((rd_wr_en_i) && (rd_addr_i != 0)) begin
                regfile_data[rd_addr_i] <= rd_wr_data_i;
        end 

    end 
    
endmodule