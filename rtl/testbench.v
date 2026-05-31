`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: riscv-tests_tb
// Description: Architectural Testbench for MPT-RV32i Core. 
//              Automatically iterates through the complete 42-test RISC-V 
//              rv32ui (User-Level Integer) verification suite.
//////////////////////////////////////////////////////////////////////////////////

// ==============================================================================
// RISC-V TEST SUITE MACRO DEFINITIONS (ID MAPPINGS 0 to 41)
// ==============================================================================
`define RR_ADD   0
`define RR_SUB   1
`define RR_AND   2
`define RR_OR    3
`define RR_XOR   4
`define RR_SLT   5
`define RR_SLTU  6
`define RR_SLL   7
`define RR_SRL   8
`define RR_SRA   9
 
`define I_ADDI   10
`define I_ANDI   11
`define I_ORI    12
`define I_XORI   13
`define I_SLTI   14
`define I_SLLI   15
`define I_SRLI   16
`define I_SRAI   17
 
`define B_BEQ    18
`define B_BNE    19
`define B_BLT    20
`define B_BGE    21
`define B_BLTU   22
`define B_BGEU   23

`define UI_LUI   24
`define UI_AUIPC 25

`define J_JAL    26
`define J_JALR   27

`define L_LB     28
`define L_LH     29
`define L_LW     30
`define L_LBU    31
`define L_LHU    32

`define S_SB     33
`define S_SH     34
`define S_SW     35

`define I_SLTIU      36  
`define MISC_SIMPLE  37  
`define MISC_FENCE_I 38  
`define MISC_LD_ST   39  
`define MISC_ST_LD   40  
`define MISC_MA_DATA 41  

`define TEST_TO_RUN 39

module testbench();
  
    // ==============================================================================
    // TESTBENCH SIGNALS & DRIVERS
    // ==============================================================================
    reg         Clk = 0;          
    reg         Reset_n;          
    reg  [31:0] IMEM_data;        
    reg  [31:0] DMEM_rd_data;     

    wire [31:0] IMEM_addr;        
    wire [31:0] DMEM_addr;        
    wire [3:0]  DMEM_wr_byte_en;  
    wire [31:0] DMEM_wr_data;     
    wire        DMEM_rst;         
    wire        Exception;        

    // ==============================================================================
    // REGRESSION SUITE SCOREBOARD TRACKING INTEGRATION
    // ==============================================================================
    integer passed_tests = 0;     // Increments on clean verification passes
    integer failed_tests = 0;     // Increments on logic timeouts or exceptions
    reg     suite_has_errors = 0; // Global flag turned True if any single test fails

    // ==============================================================================
    // UNIT UNDER TEST (UUT) INSTANTIATION
    // ==============================================================================
    mpt_top UUT(
        .clk_i             (Clk),
        .resetn_i          (Reset_n),
        .DMEM_wr_byte_en_o (DMEM_wr_byte_en),
        .DMEM_addr_o       (DMEM_addr),
        .DMEM_wr_data_o    (DMEM_wr_data),
        .DMEM_rd_data_i    (DMEM_rd_data),
        .DMEM_rst_o        (DMEM_rst),
        .boot_addr_i       (32'b0),         
        .IMEM_data_i       (IMEM_data),
        .IMEM_addr_o       (IMEM_addr),
        .exception_o       (Exception)
    );
    
    always #(10) Clk = ~Clk;


integer idx; // Declare this at the top of your testbench module or initial block

initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench); // Dumps standard signals
    
    // !!! ADD THIS LOOP TO CAPTURE THE REGISTER ARRAY !!!
    for (idx = 0; idx < 32; idx = idx + 1) begin
        $dumpvars(0, UUT.id_stage_i.regfile_i.regfile_data[idx]);
    end
end

    // -----------------------------------------------------------------------
    // FIX (Test 39 'ld_st' watchdog timeout):
    // The original 0xFFF (4096-word / 16 KB) array was too small. The
    // 'rv32ui-p-ld_st' test places its begin_signature at byte address 0x4000
    // (word index 0x1000), one past the end of the old array. Out-of-range
    // Verilog memory accesses return X for reads and silently drop writes,
    // which made the load return X, propagated X into the PC, and hung the
    // pipeline. Sizing the array to 0x3FFF (16384 words / 64 KB) safely
    // covers begin_signature/tdat areas of every test in the suite.
    // -----------------------------------------------------------------------
    parameter MEMORY_DEPTH  = 32'h3FFF;
    reg [31:0] MEMORY [0:MEMORY_DEPTH];
    integer i;

    always@(posedge Clk, negedge Reset_n) begin
        if(Reset_n == 1'b0) begin
            IMEM_data    <= 0;
            DMEM_rd_data <= 0;
        end
        else begin
            IMEM_data <= MEMORY[IMEM_addr[31:2]];
            if(DMEM_rst)   DMEM_rd_data <= 0;
            else           DMEM_rd_data <= MEMORY[DMEM_addr[31:2]];

            if(DMEM_wr_byte_en[0] == 1'b1) MEMORY[DMEM_addr[31:2]][7:0]   <= DMEM_wr_data[7:0];
            if(DMEM_wr_byte_en[1] == 1'b1) MEMORY[DMEM_addr[31:2]][15:8]  <= DMEM_wr_data[15:8];
            if(DMEM_wr_byte_en[2] == 1'b1) MEMORY[DMEM_addr[31:2]][23:16] <= DMEM_wr_data[23:16];
            if(DMEM_wr_byte_en[3] == 1'b1) MEMORY[DMEM_addr[31:2]][31:24] <= DMEM_wr_data[31:24];
        end
    end

    // ==============================================================================
    // SIMULATION TASK: FILE HANDLING & MEMORY / REGBANK PURGING
    // ==============================================================================
    task LOAD_TEST;
        input integer TESTID;
        integer reg_clear_idx;
        begin
            Reset_n = 0; 
            #10;
            
            // 1. Wipe Unified RAM Memory clean
            for (i=0; i<= MEMORY_DEPTH; i=i+1) begin
                MEMORY[i] = 0;
            end

            // 2. CRITICAL SOLUTION: Forcefully wipe internal core registers to 0.
            // This destroys residual "Pass tracking status flags" left by prior tests.
            for (reg_clear_idx = 0; reg_clear_idx < 32; reg_clear_idx = reg_clear_idx + 1) begin
                UUT.id_stage_i.regfile_i.regfile_data[reg_clear_idx] = 32'b0;
            end

            // Match test IDs with their corresponding hex files
            case(TESTID)
                `RR_ADD:  $readmemh("mem/hex/rv32ui-p-add.hex"  ,MEMORY);
                `RR_SUB:  $readmemh("mem/hex/rv32ui-p-sub.hex"  ,MEMORY);
                `RR_AND:  $readmemh("mem/hex/rv32ui-p-and.hex"  ,MEMORY);
                `RR_OR:   $readmemh("mem/hex/rv32ui-p-or.hex"   ,MEMORY);
                `RR_XOR:  $readmemh("mem/hex/rv32ui-p-xor.hex"  ,MEMORY);
                `RR_SLT:  $readmemh("mem/hex/rv32ui-p-slt.hex"  ,MEMORY);
                `RR_SLTU: $readmemh("mem/hex/rv32ui-p-sltu.hex" ,MEMORY);
                `RR_SLL:  $readmemh("mem/hex/rv32ui-p-sll.hex"  ,MEMORY);
                `RR_SRL:  $readmemh("mem/hex/rv32ui-p-srl.hex"  ,MEMORY);
                `RR_SRA:  $readmemh("mem/hex/rv32ui-p-sra.hex"  ,MEMORY);
        
                `I_ADDI:  $readmemh("mem/hex/rv32ui-p-addi.hex" ,MEMORY);
                `I_ANDI:  $readmemh("mem/hex/rv32ui-p-andi.hex" ,MEMORY);
                `I_ORI:   $readmemh("mem/hex/rv32ui-p-ori.hex"  ,MEMORY);
                `I_XORI:  $readmemh("mem/hex/rv32ui-p-xori.hex" ,MEMORY);
                `I_SLTI:  $readmemh("mem/hex/rv32ui-p-slti.hex" ,MEMORY);
                `I_SLLI:  $readmemh("mem/hex/rv32ui-p-slli.hex" ,MEMORY);
                `I_SRLI:  $readmemh("mem/hex/rv32ui-p-srli.hex" ,MEMORY);
                `I_SRAI:  $readmemh("mem/hex/rv32ui-p-srai.hex" ,MEMORY);
                
                `B_BEQ:   $readmemh("mem/hex/rv32ui-p-beq.hex"  ,MEMORY);
                `B_BNE:   $readmemh("mem/hex/rv32ui-p-bne.hex"  ,MEMORY);
                `B_BLT:   $readmemh("mem/hex/rv32ui-p-blt.hex"  ,MEMORY);
                `B_BGE:   $readmemh("mem/hex/rv32ui-p-bge.hex"  ,MEMORY);
                `B_BLTU:  $readmemh("mem/hex/rv32ui-p-bltu.hex" ,MEMORY);
                `B_BGEU:  $readmemh("mem/hex/rv32ui-p-bgeu.hex" ,MEMORY);
    
                `UI_LUI:  $readmemh("mem/hex/rv32ui-p-lui.hex"  ,MEMORY);
                `UI_AUIPC:$readmemh("mem/hex/rv32ui-p-auipc.hex",MEMORY);
    
                `J_JAL:   $readmemh("mem/hex/rv32ui-p-jal.hex"  ,MEMORY);
                `J_JALR:  $readmemh("mem/hex/rv32ui-p-jalr.hex" ,MEMORY);
    
                `L_LB:    $readmemh("mem/hex/rv32ui-p-lb.hex"   ,MEMORY);
                `L_LH:    $readmemh("mem/hex/rv32ui-p-lh.hex"   ,MEMORY);
                `L_LW:    $readmemh("mem/hex/rv32ui-p-lw.hex"   ,MEMORY);
                `L_LBU:   $readmemh("mem/hex/rv32ui-p-lbu.hex"  ,MEMORY);
                `L_LHU:   $readmemh("mem/hex/rv32ui-p-lhu.hex"  ,MEMORY);
                
                `S_SB:    $readmemh("mem/hex/rv32ui-p-sb.hex"   ,MEMORY);
                `S_SH:    $readmemh("mem/hex/rv32ui-p-sh.hex"   ,MEMORY);
                `S_SW:    $readmemh("mem/hex/rv32ui-p-sw.hex"   ,MEMORY);
                
                `I_SLTIU:      $readmemh("mem/hex/rv32ui-p-sltiu.hex"   ,MEMORY);
                `MISC_SIMPLE:  $readmemh("mem/hex/rv32ui-p-simple.hex"  ,MEMORY);
                `MISC_FENCE_I: $readmemh("mem/hex/rv32ui-p-fence_i.hex" ,MEMORY);
                `MISC_LD_ST:   $readmemh("mem/hex/rv32ui-p-ld_st.hex"   ,MEMORY);
                `MISC_ST_LD:   $readmemh("mem/hex/rv32ui-p-st_ld.hex"   ,MEMORY);
                `MISC_MA_DATA: $readmemh("mem/hex/rv32ui-p-ma_data.hex" ,MEMORY);
                default: $display("Unknown Test ID: %0d", TESTID);
            endcase
            
            #1; 
            if (MEMORY[0] == 32'h0) begin
                $display("CRITICAL ERROR: Memory failed to load for Test ID %0d.", TESTID);
                $finish;
            end
        end
    endtask
    
    // ==============================================================================
    // SIMULATION TASK: PIPELINE METRIC MONITORING LOOP
    // ==============================================================================
    integer t;
    reg     test_active; // Local termination controller flag
    
    task EVAL_TEST;
        input integer TESTID;
        begin
            test_active = 1;
            Reset_n     = 1; // Release core reset to spin pipeline logic
            
            // Allow 5 clock cycles to complete before testing conditions to clear setup edge
            repeat(5) @(posedge Clk);

            for(t=0; t<=100000; t=t+1) begin
                if (test_active) begin
                    @(posedge Clk) begin
                        
                        // Condition A: Evaluation Success Matrix Matches Trajectory
                        if((UUT.id_stage_i.regfile_i.regfile_data[3] == 1) &&   
                           (UUT.id_stage_i.regfile_i.regfile_data[17] == 93) && 
                           (UUT.id_stage_i.regfile_i.regfile_data[10] == 0))    
                        begin
                            $display("[PASS] Test ID: %2d", TESTID);
                            passed_tests = passed_tests + 1;
                            test_active  = 0; // Safely breaks out of execution loop
                        end 

                        // Condition B: Core Hardware Panic Exception Caught
                        else if (Exception == 1) begin
                            $display("[FAIL] Test ID: %2d -> REASON: HARDWARE EXCEPTION TRAP", TESTID);
                            failed_tests     = failed_tests + 1;
                            suite_has_errors = 1;
                            test_active      = 0;
                        end 

                        // Condition C: Watchdog Threshold Exceeded (Pipeline Lockup/Hang)
                        else if (t == 99999) begin
                            $display("[FAIL] Test ID: %2d -> REASON: WATCHDOG TIMEOUT (PIPELINE HUNG)", TESTID);
                            failed_tests     = failed_tests + 1;
                            suite_has_errors = 1;
                            test_active      = 0;
                        end
                    end
                end
            end
        end
    endtask

    // ==============================================================================
    // MAIN REGRESSION SUITE CONTROLLER & AGGREGATED SCOREBOARD
    // ==============================================================================
    integer j;
    initial begin
        Reset_n = 0;
        #100; 

        if($test$plusargs("runall")) begin
            $display("=====================================================================");
            $display(" STARTING RISC-V COMPLIANCE REGRESSION RUN (42 ARCHITECTURAL TESTS)  ");
            $display("=====================================================================");
            
            for(j=`RR_ADD; j<=`MISC_MA_DATA; j=j+1) begin
                LOAD_TEST(j);
                #40; // Guard time for stabilizing clear values across sequential bounds
                EVAL_TEST(j);
            end
            
            // Print Scoreboard Summary
            $display("\n=====================================================================");
            $display("                      SUITE REGRESSION SUMMARY                       ");
            $display("=====================================================================");
            $display("  TOTAL TESTS RUN : %0d", (passed_tests + failed_tests));
            $display("  PASSED          : %0d", passed_tests);
            $display("  FAILED          : %0d", failed_tests);
            $display("=====================================================================");
            
            if (suite_has_errors) begin
                $display(" [RESULT] REGRESSION STATUS: FAILED (Structural Errors Detected)");
                $display("=====================================================================");
                $finish(1); // Exits environment returning error flag to terminal/Makefile
            end else begin
                $display(" [RESULT] REGRESSION STATUS: SUCCESS! ALL CORE STRUCTURAL TESTS PASSED.");
                $display("=====================================================================");
                $finish(0); // Clean Exit code
            end
        end

        // Interactive Diagnostic Single-Test Mode
        else begin
            $display("Running Diagnostic Single-Test ID: %0d", `TEST_TO_RUN);
            LOAD_TEST(`TEST_TO_RUN);
            #40;
            EVAL_TEST(`TEST_TO_RUN);
            $finish;
        end
    end
endmodule