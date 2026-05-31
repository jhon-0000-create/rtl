// ==============================================================================
// MODULE: mpt_control (Hazard Detection & Forwarding Unit)
//
// ROLE IN ARCHITECTURE: 
// This module monitors the entire pipeline to resolve Data Hazards (instructions 
// reading registers before they are written) and Control Hazards (instructions 
// executing after a branch/jump before we know if we should take it).
//
// HARDWARE MAPPING: 
// Primarily PURE COMBINATIONAL LOGIC. It consists of dozens of digital 
// comparators (XNOR gates) that match 5-bit register addresses between different 
// stages of the pipeline. It also contains two sequential Flip-Flops to track 
// multi-cycle flushing states.
// ==============================================================================

`ifdef CUSTOM_DEFINE
    `include "defines.vh"
`endif

module mpt_control 
    `include "mpt_definitions.vh"

	`ifdef CUSTOM_DEFINE
		#(parameter REG_DATA_WIDTH  = `REG_DATA_WIDTH,
		  parameter REGFILE_ADDR_WIDTH = `REGFILE_ADDR_WIDTH
		  )
	`else 
		#(parameter REG_DATA_WIDTH  = 32,
		  parameter REGFILE_ADDR_WIDTH  = 5 // 5 bits can represent 32 registers (2^5 = 32)
		 )
    `endif

    (
    	input  wire                           clk_i,
    	input  wire                           resetn_i,
 
    	// --- FORWARDING OUTPUTS ---
        // These control the multiplexer switches right before the ALU inputs.
    	output reg  [1:0]                     forwardA_o,       // Controls MUX for ALU Operand 1
		output reg  [1:0]                     forwardB_o,       // Controls MUX for ALU Operand 2
		output reg                            forwardM_o,       // Controls MUX for Data Memory writes (Stores)
  
		// --- HAZARD DETECTION OUTPUTS ---
        // These signals manipulate the pipeline registers to pause or clear them.
		output reg                            stall_o,          // Pauses the PC and IF/ID stage (freezes execution)
 		output wire                           IF_ID_flush_o,    // Wipes out instructions in Fetch/Decode (turns them into NOPs)
 		output reg                            EX_flush_o,       // Wipes out instructions in Execute stage
  
 		// --- INPUTS FOR FORWARDING DETECTORS ---
		input  wire [1:0]                     ID_alu_source_sel_i, // Tells us if the Decode stage instruction needs registers or immediates
		input  wire [REGFILE_ADDR_WIDTH-1:0]  ID_rs1_addr_i,       // Register Source 1 address from Decode stage
		input  wire [REGFILE_ADDR_WIDTH-1:0]  ID_rs2_addr_i,       // Register Source 2 address from Decode stage
		input  wire [REGFILE_ADDR_WIDTH-1:0]  ID_rd_addr_i,        // Destination Register address from Decode stage
		input  wire [REGFILE_ADDR_WIDTH-1:0]  EX_rd_addr_i,        // Destination Register address currently in Execute stage
		input  wire [REGFILE_ADDR_WIDTH-1:0]  EX_rs2_addr_i,       // Register Source 2 address currently in Execute stage
		input  wire [REGFILE_ADDR_WIDTH-1:0]  MEM_rd_addr_i,       // Destination Register address currently in Memory stage                
		input  wire                           EX_rd_wr_en_i,       // Is the Execute stage instruction going to write to the RegFile?
		input  wire                           MEM_rd_wr_en_i,      // Is the Memory stage instruction going to write to the RegFile?
 
		// --- INPUTS FOR HAZARD DETECTORS ---
 		input  wire [31:0]                    IF_instruction_i,    // The raw 32-bit instruction currently being fetched
	    input  wire 						  ID_mem_rd_en_i,      // Is the Decode stage instruction reading memory? (e.g., LW)
        input  wire                           EX_branch_en_i,      // Evaluated in EX: Is a conditional branch actually taken?
        input  wire                           ID_jump_en_i         // Evaluated in ID: Is an unconditional jump taken?
    );

    // ==============================================================================
    // TRUTH TABLES FOR ALU FORWARDING SELECTORS
    // ------------------------------------------------------------------------------
    // 00 -> Use normal Register File data (No Hazard detected).
    // 10 -> Forward the data directly from the Execute Stage ALU output (EX Hazard).
    // 01 -> Forward the data from the Memory Stage output / Writeback input (MEM Hazard).
    // ==============================================================================
    
    // ------------------------------------------------------------------------------
    // FORWARD A: Controls Operand 1 of the ALU
    // Hardware: A giant combinational branch of AND/OR gates comparing 5-bit numbers.
    // ------------------------------------------------------------------------------
    always@* begin

        // 1. EX HAZARD CHECK
        // Scenario: The instruction immediately before us is calculating a value that we need right now.
        if ( (EX_rd_wr_en_i == 1'b1) &&         // Is the instruction in EX going to write to a register?
             (EX_rd_addr_i       != 0   ) &&    // CRITICAL: Is it writing to x0? (x0 is hardwired to 0, never forward it!)
             (EX_rd_addr_i       == ID_rs1_addr_i) && // Does its destination register match our source register?
             (ID_alu_source_sel_i[1] != 1)      // Ensure our instruction actually needs a register value here
            )
            forwardA_o = 2'b10; // Instantly route EX output back to ALU input A!

        // 2. MEM / WB HAZARD CHECK
        // Scenario: An instruction two steps ahead of us calculated a value, but it hasn't quite 
        // landed back in the RegFile yet. It's sitting in the Memory pipeline stage.
        else if ( (MEM_rd_wr_en_i == 1'b1) &&   // Is the instruction in MEM going to write to a register?
             (MEM_rd_addr_i       != 0   ) &&   // Make sure it isn't register x0
             
             // WHY THIS EXPENSIVE CHECK below? -> Priority Condition:
             // If BOTH an EX hazard and a MEM hazard occur for the same register, the EX stage has 
             // the NEWEST data. We must make sure we don't accidentally forward old data from MEM 
             // if the EX stage is already providing a fresher version.
             ~ ( (EX_rd_wr_en_i  == 1'b1) && (EX_rd_addr_i != 0) && (EX_rd_addr_i == ID_rs1_addr_i)) &&
             
             (MEM_rd_addr_i       == ID_rs1_addr_i)&& // Does the MEM stage destination match our source?
             (ID_alu_source_sel_i[1] != 1)         
            )
            forwardA_o = 2'b01; // Route the Memory stage data back to ALU input A!
            
        else
            forwardA_o = 2'b0;  // Safe! No hazards. Use standard RegFile data.
    end
    
    // ------------------------------------------------------------------------------
    // FORWARD B: Controls Operand 2 of the ALU
    // Hardware: Mirror copy of Forward A logic, but checking rs2_addr instead of rs1_addr.
    // ------------------------------------------------------------------------------
    always@* begin

        // EX HAZARD CHECK
        if ( (EX_rd_wr_en_i == 1'b1) &&     
             (EX_rd_addr_i       != 0   ) &&
             (EX_rd_addr_i       == ID_rs2_addr_i) &&
             (ID_alu_source_sel_i[0] != 1)
            )
            forwardB_o = 2'b10;
            
        // MEM / WB HAZARD CHECK
        else if ( (MEM_rd_wr_en_i == 1'b1) &&
             (MEM_rd_addr_i       != 0   ) &&
             ~ ( (EX_rd_wr_en_i  == 1'b1) && (EX_rd_addr_i != 0) && (EX_rd_addr_i == ID_rs2_addr_i))
             && (MEM_rd_addr_i == ID_rs2_addr_i) &&
             (ID_alu_source_sel_i[0] != 1)
           ) 
            forwardB_o = 2'b01;
            
        else
            forwardB_o = 2'b0;
    end


    // ------------------------------------------------------------------------------
    // MEMORY WRITE FORWARDING (Store Data Hazards)
    // Hardware: A 1-bit multiplexer switch.
    //
    // WHY IT EXISTS: Imagine this scenario:
    //    ADD x5, x1, x2  <- Calculates a new value for x5
    //    SW  x5, 0(x10)  <- Instantly tries to store x5 into memory
    //
    // When the Store Word (SW) instruction reaches the Execute/Memory boundary, it needs 
    // to write the data from x5. But x5 was just generated by the ADD instruction! 
    // This block forwards the newly generated data directly to the memory write channel.
    // ------------------------------------------------------------------------------
    always@* begin
    		if ( 
    			 (MEM_rd_wr_en_i == 1'b1) &&         // Is an older instruction currently committing data?
    			 (EX_rs2_addr_i == MEM_rd_addr_i) && // Does our store data match that instruction's target?
    			 (MEM_rd_addr_i != 0)                // Ignore register x0
    		   )
    		    forwardM_o = 1; // Forward from MEM_ALU_result directly to the memory input buffer
    		else
    			forwardM_o = 0; // Normal: write data coming standard from the pipeline register
    end


    // ==============================================================================
    // DATA LOAD HAZARD DETECTION (The Pipeline Freeze)
    // Hardware: Combinational decoding logic driving a 'stall_o' wire.
    //
    // THE LOAD-USE HAZARD PROBLEM:
    // Forwarding fixes almost everything, EXCEPT when an instruction tries to use a 
    // register immediately after a LOAD instruction (e.g., LW x5, 0(x2)). 
    // A Load instruction doesn't get its data back from the memory chips until the very 
    // end of the MEM stage. The ALU cannot "snatch it mid-air" in the EX stage because 
    // the data literally does not exist yet inside the processor.
    //
    // THE SOLUTION:
    // We must forcefully STALL (freeze) the processor for 1 clock cycle, injecting a 
    // "Bubble" or NOP (No Operation) to buy the memory chip enough time to return the data.
    // ==============================================================================
    wire [6:0] opcode = IF_instruction_i[6:0]; // Slice out the opcode of the instruction we are fetching
    
    always@* begin
        if(ID_mem_rd_en_i == 1) begin // Step 1: Is the instruction currently in Decode a LOAD?
            
            // Step 2a: Check if the incoming instruction in Fetch reads two registers (R-type, Branch, Store)
            if( ( (opcode == `OPCODE_OP) || (opcode == `OPCODE_BRANCH) || (opcode == `OPCODE_STORE) ) &&
                // Step 2b: Does its source registers match the register being loaded into?
                ( (ID_rd_addr_i == IF_instruction_i[19:15]) || (ID_rd_addr_i == IF_instruction_i[24:20]) )
              )     
            begin
                stall_o = 1; // CRITICAL: Assert STALL! Freeze the PC and Fetch stage registers.
            end 
            
            // Step 3a: Check if the incoming instruction reads only one register (I-type math, or another Load)
            else if( ( (opcode == `OPCODE_OP_IMM) ||(opcode == `OPCODE_LOAD) )  &&
                         // Step 3b: Does its single source register match the register being loaded?
                         ( (ID_rd_addr_i == IF_instruction_i[19:15]) )
                       )
            begin
                stall_o = 1; // Assert STALL!
            end     
            else stall_o = 0; // No match: instructions are independent. No stall needed.
        end
        else stall_o = 0; // The instruction in Decode isn't a load. No stall needed.
    end


    // ==============================================================================
    // CONTROL HAZARD DETECTION (The Pipeline Flush / Misprediction Cleanup)
    // Hardware: Sequential flip-flops controlling clean-up muxes.
    //
    // THE PROBLEM: 
    // While a branch or jump is being figured out in the Decode or Execute stages, the 
    // Fetch stage blindly continues loading instructions straight ahead (guessing the 
    // branch won't happen). If the branch IS taken, the CPU has loaded instructions 
    // it was never supposed to touch!
    //
    // THE SOLUTION: FLUSHING.
    // We must clear out those wrongly fetched instructions, changing their control signals 
    // to 0. This turns them into harmless "NOPs" (bubbles) so they do not alter register 
    // data or corrupt state.
    // ==============================================================================

	reg IF_ID_Flush1; // Combinational register tracking immediate branch/jump detections
    reg IF_ID_Flush2; // Sequential flip-flop tracking delayed cleanup cycles


    // If either cleanup signal is active, tell the hardware to erase the Fetch/Decode boundary.
    assign IF_ID_flush_o = (IF_ID_Flush1 || IF_ID_Flush2);

    // Combinational evaluation of a taken jump or resolved branch
    always@* begin
        if((EX_branch_en_i) || (ID_jump_en_i)) 
        	IF_ID_Flush1 = 1; // Target hit: Start the flush immediately
        else                                      
        	IF_ID_Flush1 = 0;
    end
    
    // Sequential Pipeline Shift Register for Flushes:
    // Because it takes time for the new branch address to latch into the PC register and 
    // load the true instruction from instruction memory, a mispredicted branch leaves a 
    // 2-cycle wake of bad data. We use this clock-driven block to hold the flush active 
    // for a second cycle (`IF_ID_Flush2 <= IF_ID_Flush1`).
    always@(posedge clk_i) begin
        if(resetn_i == 1'b0) 
        	IF_ID_Flush2 <= 0;
        else                 
        	IF_ID_Flush2 <= IF_ID_Flush1; // Latch the flush forward by one clock cycle
    end

    // Flush EX: If a conditional branch is resolved as true in the EX stage, 
    // anything that snuck behind it into the EX pipeline buffer must be immediately killed.
    always@* begin
    	EX_flush_o = (EX_branch_en_i) ? 1'b1 : 1'b0;
    end

endmodule