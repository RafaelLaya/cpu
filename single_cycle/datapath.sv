/** The datapath of a single-cycle implementation of a subset of the ARMv8 ISA 
 *
 * Inputs:
 *      clk: The clock signal
 *      reset: An active-high reset signal (synchronous on positive edges of clk)
 *      All others: Control signals
 *
 *	Outputs:
 *      overflow, negative, zero, carry_out: Current flags of the ALU
 *      flag_overflow, flag_negative, flag_zero, flag_carry_out: The values of the status register
 *
 */
`include "delays.sv"
module datapath(
    clk, reset,
    overflow, negative, zero, carry_out, 
    flag_overflow, flag_negative, flag_zero, flag_carry_out,
    Reg2Loc,
    ALUSrc,
    MemToReg,
    RegWrite,
    MemWrite,
    BrTaken,
    UncondBr,
    ALUOp,
    AltALUSrc,
    SetStatus,
    WithKeep,
    SpecialToReg,
    SingleByteToReg,
    MemRead,
    MemXferSize,
    instruction
);
    // external signals
    input logic clk, reset; 
    
    // control signals
    input logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken;
    input logic UncondBr, AltALUSrc, SetStatus, WithKeep, SpecialToReg;
    input logic [2:0] ALUOp;
    input logic SingleByteToReg, MemRead;
    input logic [3:0] MemXferSize;

    // status signals
    output logic overflow, negative, zero, carry_out;
    output logic flag_overflow, flag_negative, flag_zero, flag_carry_out;

    // The current instruction 
    output logic [31:0] instruction; 

    // fields taken from the instruction
    logic [4:0] Rd;
    logic [4:0] Rn;
    logic [4:0] Rm; 
    logic [8:0] DAddr9;
    logic [11:0] Imm12; 
    logic [1:0] shift_sel; 
    logic [18:0] CondAddr19;
    logic [25:0] BrAddr26; 
    logic [15:0] imm16; 

    // Get fields from the current instruction
    assign Rd = instruction[4:0];
    assign Rn = instruction[9:5];
    assign Rm = instruction[20:16];
    assign DAddr9 = instruction[20:12];
    assign Imm12 = instruction[21:10];
    assign shift_sel = instruction[22:21];
    assign CondAddr19 = instruction[23:5];
    assign BrAddr26 = instruction[25:0];
    assign imm16 = instruction[20:5];

    /* regfile */ 
    logic [63:0] regfile_ReadData1;
    logic [63:0] regfile_ReadData2; 
    logic [63:0] regfile_WriteData;
    logic [4:0] regfile_ReadRegister1;
    logic [4:0] regfile_ReadRegister2; 
    logic [4:0] regfile_WriteRegister;
    logic regfile_RegWrite; 
    regfile register_file (
        .ReadData1(regfile_ReadData1), 
        .ReadData2(regfile_ReadData2), 
        .WriteData(regfile_WriteData), 
        .ReadRegister1(regfile_ReadRegister1), 
        .ReadRegister2(regfile_ReadRegister2), 
        .WriteRegister(regfile_WriteRegister), 
        .RegWrite(regfile_RegWrite), 
        .clk(clk)
    );

    /* ALU */
    logic [63:0] alu_A;
    logic [63:0] alu_B;
    logic [2:0] alu_cntrl;
    logic [63:0] alu_result;
    logic alu_negative, alu_zero, alu_overflow, alu_carry_out;
    alu arithmetic_logic_unit (
        .A(alu_A), 
        .B(alu_B), 
        .cntrl(alu_cntrl), 
        .result(alu_result), 
        .negative(alu_negative), 
        .zero(alu_zero), 
        .overflow(alu_overflow), 
        .carry_out(alu_carry_out)
    );

    /* data Memory */
    logic [63:0] datamem_address;
    logic datamem_write_enable;
    logic datamem_read_enable;
    logic [63:0] datamem_write_data;
    logic [3:0] datamem_xfer_size;
    logic [63:0] datamem_read_data;
    datamem data_memory (
        .address(datamem_address),
        .write_enable(datamem_write_enable),
        .read_enable(datamem_read_enable),
        .write_data(datamem_write_data),
        .clk(clk),
        .xfer_size(datamem_xfer_size),
        .read_data(datamem_read_data)
	);

    /* move(z/k) calculator */
    logic [63:0] move_calc_out;
    logic [63:0] move_calc_Data;
    logic [1:0] move_calc_shift_sel;
    logic move_calc_WithKeep;
    logic [15:0] move_calc_imm16;
    move_calc move_calculator (
        .out(move_calc_out), 
        .Data(move_calc_Data), 
        .shift_sel(move_calc_shift_sel), 
        .WithKeep(move_calc_WithKeep), 
        .imm16(move_calc_imm16)
    );

    /* instruction memory */
    logic [63:0] imem_address;
    logic [31:0] imem_instruction;
    instructmem instruction_memory (
	    .address(imem_address),
	    .instruction(imem_instruction),
	    .clk(clk)
	);
    
    /* PC */
    logic [63:0] pc_current;
    logic [63:0] pc_next;
    register #(.WIDTH(64)) program_counter (
        .dout(pc_current), 
        .din(pc_next), 
        .write_en(1'b1), 
        .clk(clk),
        .reset(reset)
    );

    /* carry out flag */
    logic flag_carry_out_current;
    logic flag_carry_out_next; 
    logic flag_carry_out_update;
    register #(.WIDTH(1)) carry_out_status_register (
        .dout(flag_carry_out_current), 
        .din(flag_carry_out_next), 
        .write_en(flag_carry_out_update), 
        .clk(clk),
        .reset(1'b0)
    );

    /* overflow flag */
    logic flag_overflow_current;
    logic flag_overflow_next; 
    logic flag_overflow_update;
    register #(.WIDTH(1)) overflow_status_register (
        .dout(flag_overflow_current), 
        .din(flag_overflow_next), 
        .write_en(flag_overflow_update), 
        .clk(clk),
        .reset(1'b0)
    );

    /* zero flag */
    logic flag_zero_current; 
    logic flag_zero_next;
    logic flag_zero_update;
    register #(.WIDTH(1)) zero_status_register (
        .dout(flag_zero_current), 
        .din(flag_zero_next), 
        .write_en(flag_zero_update), 
        .clk(clk),
        .reset(1'b0)
    );

    /* negative flag */
    logic flag_negative_current;
    logic flag_negative_next; 
    logic flag_negative_update;
    register #(.WIDTH(1)) negative_status_register (
        .dout(flag_negative_current), 
        .din(flag_negative_next), 
        .write_en(flag_negative_update), 
        .clk(clk),
        .reset(1'b0)
    );

    /* Reg2LocMux */
    logic [4:0] reg2locmux_zero_in;
    logic [4:0] reg2locmux_one_in; 
    logic [4:0] reg2locmux_out;
    mux2_1_wide #(.WIDTH(5)) Reg2LocMux (
        .out(reg2locmux_out), 
        .in({reg2locmux_zero_in, reg2locmux_one_in}), 
        .sel(Reg2Loc)
    );

    /* DAddr9 Sign Extender */
    logic [8:0] SE_DAddr9_in;
    logic [63:0] SE_DAddr9_out; 
    sign_extender #(.IN_LENGTH(9), .OUT_LENGTH(64)) SE_Daddr9 (
        .in(SE_DAddr9_in), 
        .out(SE_DAddr9_out)
    );

    /* Imm12 Zero Extender */
    logic [11:0] ZE_Imm12_in;
    logic [63:0] ZE_Imm12_out;
    zero_extender #(.IN_LENGTH(12), .OUT_LENGTH(64)) ZE_Imm12 (
        .in(ZE_Imm12_in),
        .out(ZE_Imm12_out)
    );

    /* AltALUSrc Mux */
    logic [63:0] AltAluSrcMux_zero_in;
    logic [63:0] AltAluSrcMux_one_in; 
    logic [63:0] AltAluSrcMux_out;
    mux2_1_wide #(.WIDTH(64)) AltALUSrcMux (
        .out(AltAluSrcMux_out), 
        .in({AltAluSrcMux_zero_in, AltAluSrcMux_one_in}), 
        .sel(AltALUSrc)
    );

    /* ALUSrc Mux */
    logic [63:0] ALUSrcMux_zero_in;
    logic [63:0] ALUSrcMux_one_in;
    logic [63:0] ALUSrcMux_out; 
    mux2_1_wide #(.WIDTH(64)) ALUSrcMux (
        .out(ALUSrcMux_out),
        .in({ALUSrcMux_zero_in, ALUSrcMux_one_in}),
        .sel(ALUSrc)
    );

    /* MemToReg Mux */
    logic [63:0] MemToRegMux_zero_in;
    logic [63:0] MemToRegMux_one_in;
    logic [63:0] MemToRegMux_out;
    mux2_1_wide #(.WIDTH(64)) MemToRegMux (
        .out(MemToRegMux_out),
        .in({MemToRegMux_zero_in, MemToRegMux_one_in}),
        .sel(MemToReg)
    );

    /* SpecialToReg Mux */
    logic [63:0] SpecialToRegMux_zero_in;
    logic [63:0] SpecialToRegMux_one_in;
    logic [63:0] SpecialToRegMux_out;
    mux2_1_wide #(.WIDTH(64)) SpecialToRegMux (
        .out(SpecialToRegMux_out),
        .in({SpecialToRegMux_zero_in, SpecialToRegMux_one_in}),
        .sel(SpecialToReg)
    );

    /* SingleByteToReg Mux */
    logic [63:0] SingleByteToRegMux_zero_in;
    logic [63:0] SingleByteToRegMux_one_in;
    logic [63:0] SingleByteToRegMux_out;
    mux2_1_wide #(.WIDTH(64)) SingleByteToRegMux (
        .out(SingleByteToRegMux_out),
        .in({SingleByteToRegMux_zero_in, SingleByteToRegMux_one_in}),
        .sel(SingleByteToReg)
    );

    /* Sign Extender for CondAddr19 */
    logic [18:0] SE_CondAddr19_in;
    logic [63:0] SE_CondAddr19_out;
    sign_extender #(.IN_LENGTH(19), .OUT_LENGTH(64)) SE_CondAddr19 (
        .in(SE_CondAddr19_in), 
        .out(SE_CondAddr19_out)
    );

    /* Sign Extender for BrAddr26 */
    logic [25:0] SE_BrAddr26_in;
    logic [63:0] SE_BrAddr26_out;
    sign_extender #(.IN_LENGTH(26), .OUT_LENGTH(64)) SE_BrAddr26 (
        .in(SE_BrAddr26_in),
        .out(SE_BrAddr26_out)
    );

    /* UncondBr Mux */
    logic [63:0] UncondBrMux_zero_in;
    logic [63:0] UncondBrMux_one_in;
    logic [63:0] UncondBrMux_out;
    mux2_1_wide #(.WIDTH(64)) UncondBrMux (
        .out(UncondBrMux_out),
        .in({UncondBrMux_zero_in, UncondBrMux_one_in}),
        .sel(UncondBr)
    );

    /* LSL_branch */
    logic [63:0] LSL_branch_in; 
    logic [63:0] LSL_branch_out;
    lsl_2 #(.IN_LENGTH(64)) LSL_branch (
        .in(LSL_branch_in), 
        .out(LSL_branch_out)
    );

    /* Adder_branch */
    logic [63:0] Adder_branch_A;
    logic [63:0] Adder_branch_B;
    logic [63:0] Adder_branch_sum; 
    adder #(.IN_LENGTH(64)) Adder_branch (
        .A(Adder_branch_A), 
        .B(Adder_branch_B), 
        .sum(Adder_branch_sum)
    );

    /* Adder_pc */
    logic [63:0] Adder_pc_A;
    logic [63:0] Adder_pc_B;
    logic [63:0] Adder_pc_sum;
    adder #(.IN_LENGTH(64)) Adder_pc (
        .A(Adder_pc_A),
        .B(Adder_pc_B),
        .sum(Adder_pc_sum)
    );

    /* BrTaken Mux */
    logic [63:0] BrTakenMux_zero_in;
    logic [63:0] BrTakenMux_one_in;
    logic [63:0] BrTakenMux_out;
    mux2_1_wide #(.WIDTH(64)) BrTakenMux (
        .out(BrTakenMux_out),
        .in({BrTakenMux_zero_in, BrTakenMux_one_in}),
        .sel(BrTaken)
    );

    /* Zero Extender for Single Byte */
    logic [7:0] ZE_SingleByte_in;
    logic [63:0] ZE_SingleByte_out;
    zero_extender #(.IN_LENGTH(8), .OUT_LENGTH(64)) ZE_SingleByte (
        .in(ZE_SingleByte_in), 
        .out(ZE_SingleByte_out)
    );

    /* Make all connections between modules */
    /* The comment refers to who RECEIVES a signal */

    // regfile
    assign regfile_WriteRegister = Rd;
    assign regfile_ReadRegister2 = reg2locmux_out;
    assign regfile_ReadRegister1 = Rn; 
    assign regfile_WriteData = SingleByteToRegMux_out;
    assign regfile_RegWrite = RegWrite;

    // alu
    assign alu_A = regfile_ReadData1;
    assign alu_B = ALUSrcMux_out; 
    assign alu_cntrl = ALUOp;

    // datamem
    assign datamem_address = alu_result;
    assign datamem_write_enable = MemWrite;
    assign datamem_write_data = regfile_ReadData2; 
    assign datamem_xfer_size = MemXferSize; 
    assign datamem_read_enable = MemRead;

    // move_calc
    assign move_calc_WithKeep = WithKeep;
    assign move_calc_shift_sel = shift_sel;
    assign move_calc_Data = alu_result;
    assign move_calc_imm16 = imm16;

    // imem
    assign imem_address = pc_current;

    // instruction
    assign instruction = imem_instruction; 

    // pc
    assign pc_next = BrTakenMux_out;

    // reg2locMux
    assign reg2locmux_one_in = Rm;
    assign reg2locmux_zero_in = Rd; 

    // SE_DAddr9
    assign SE_DAddr9_in = DAddr9; 

    // ZE_Imm12
    assign ZE_Imm12_in = Imm12;

    // AltAluSrcMux
    assign AltAluSrcMux_zero_in = SE_DAddr9_out;
    assign AltAluSrcMux_one_in = ZE_Imm12_out;

    // AluSrcMux
    assign ALUSrcMux_zero_in = regfile_ReadData2;
    assign ALUSrcMux_one_in = AltAluSrcMux_out;

    // MemToRegMux
    assign MemToRegMux_zero_in = alu_result;
    assign MemToRegMux_one_in = datamem_read_data;

    // SpecialToRegMux
    assign SpecialToRegMux_zero_in = MemToRegMux_out;
    assign SpecialToRegMux_one_in = move_calc_out;

    // SingleByteToRegMux
    assign SingleByteToRegMux_zero_in = SpecialToRegMux_out;
    assign SingleByteToRegMux_one_in = ZE_SingleByte_out;

    // ZE_SingleByte
    // take only the lower 8 wires
    assign ZE_SingleByte_in = SpecialToRegMux_out[7:0]; 

    // SE_CondAddr19
    assign SE_CondAddr19_in = CondAddr19;
    
    // SE_BrAddr26
    assign SE_BrAddr26_in = BrAddr26;

    // UncondBrMux
    assign UncondBrMux_zero_in = SE_CondAddr19_out;
    assign UncondBrMux_one_in = SE_BrAddr26_out;

    // LSL_branch
    assign LSL_branch_in = UncondBrMux_out;

    // Adder_branch
    assign Adder_branch_A = LSL_branch_out;
    assign Adder_branch_B = pc_current;

    // Adder_pc
    assign Adder_pc_A = pc_current;
    assign Adder_pc_B = 64'd4; 

    // BrTakenMux
    assign BrTakenMux_zero_in = Adder_pc_sum;
    assign BrTakenMux_one_in = Adder_branch_sum;

    // carry_out_status_register
    assign flag_carry_out_next = alu_carry_out;
    assign flag_carry_out_update = SetStatus;

    // overflow_status_register
    assign flag_overflow_next = alu_overflow;
    assign flag_overflow_update = SetStatus;

    // negative_status_register
    assign flag_negative_next = alu_negative;
    assign flag_negative_update = SetStatus;

    // zero_status_register
    assign flag_zero_next = alu_zero;
    assign flag_zero_update = SetStatus;

    /* Connect Outputs */ 
    assign carry_out = alu_carry_out;   assign flag_carry_out = flag_carry_out_current;
    assign overflow = alu_overflow;     assign flag_overflow = flag_overflow_current;
    assign negative = alu_negative;     assign flag_negative = flag_negative_current;
    assign zero = alu_zero;             assign flag_zero = flag_zero_current;

endmodule /* datapath */

/* testbench for datapath */
`include "benchmark.sv"
module datapath_testbench();
    logic clk, reset; 
    
    logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken;
    logic UncondBr, AltALUSrc, SetStatus, WithKeep, SpecialToReg;
    logic [2:0] ALUOp;
    logic SingleByteToReg, MemRead;
    logic [3:0] MemXferSize;

    logic overflow, negative, zero, carry_out;
    logic flag_overflow, flag_negative, flag_zero, flag_carry_out;

    datapath dut (.*); 

    // simulate clock
    parameter CLOCK_PERIOD = TESTBENCH_DELAY * 5;
    initial begin 
        clk <= 1'b0;
        forever begin 
            #(CLOCK_PERIOD / 2);
            clk <= ~clk;
        end 
    end

    parameter ALU_PASS_B = 3'b000;
    parameter ALU_ADD = 3'b010;
    parameter ALU_SUB = 3'b011; 
    parameter ALU_AND = 3'b100;
    parameter ALU_OR = 3'b101;
    parameter ALU_XOR = 3'b110;
    parameter ALU_DONT_CARE = 3'bxxx;

    logic [19:0] control_config; 
    assign Reg2Loc = control_config[19];
    assign ALUSrc = control_config[18];
    assign MemToReg = control_config[17];
    assign RegWrite = control_config[16];
    assign MemWrite = control_config[15];
    assign BrTaken = control_config[14];
    assign UncondBr = control_config[13];
    assign ALUOp = control_config[12:10];
    assign AltALUSrc = control_config[9];
    assign SetStatus = control_config[8];
    assign WithKeep = control_config[7];
    assign SpecialToReg = control_config[6];
    assign SingleByteToReg = control_config[5];
    assign MemRead = control_config[4];
    assign MemXferSize = control_config[3:0];

    parameter logic [19:0] ADDI_CONFIG = {
        1'bx,
        1'b1,
        1'b0,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_ADD,
        1'b1,
        1'b0,
        1'bx,
        1'b0,
        1'b0,
        1'bx,
        4'bxxxx
    };

    parameter logic [19:0] ADDS_CONFIG = {
        1'b1,
        1'b0,
        1'b0,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_ADD,
        1'bx,
        1'b1,
        1'bx,
        1'b0,
        1'b0,
        1'bx,
        4'bxxxx
    };

    parameter logic [19:0] B_CONFIG = {
        1'bx,
        1'bx,
        1'bx,
        1'b0,
        1'b0,
        1'b1,
        1'b1,
        ALU_DONT_CARE,
        1'bx,
        1'b0,
        1'bx,
        1'bx,
        1'bx,
        1'bx,
        4'bxxxx
    };

    parameter logic [19:0] LDUR_CONFIG = {
        1'bx,
        1'b1,
        1'b1,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_ADD,
        1'b0,
        1'b0,
        1'bx,
        1'b0,
        1'b0,
        1'b1,
        4'b1000
    };

    parameter logic [19:0] LDURB_CONFIG = {
        1'bx,
        1'b1,
        1'b1,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_ADD,
        1'b0,
        1'b0,
        1'bx,
        1'b0,
        1'b1,
        1'b1,
        4'b0001
    };

    parameter logic [19:0] MOVK_CONFIG = {
        1'b0,
        1'b0,
        1'bx,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_PASS_B,
        1'bx,
        1'b0,
        1'b1,
        1'b1,
        1'b0,
        1'bx,
        4'bxxxx
    };

    parameter logic [19:0] MOVZ_CONFIG = {
        1'bx,
        1'bx,
        1'bx,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_DONT_CARE,
        1'bx,
        1'b0,
        1'b0,
        1'b1,
        1'b0,
        1'bx,
        4'bxxxx
    };

    parameter logic [19:0] STUR_CONFIG = {
        1'b0,
        1'b1,
        1'bx,
        1'b0,
        1'b1,
        1'b0,
        1'bx,
        ALU_ADD,
        1'b0,
        1'b0,
        1'bx,
        1'bx,
        1'bx,
        1'bx,
        4'b1000
    };

    parameter logic [19:0] STURB_CONFIG = {
        1'b0,
        1'b1,
        1'bx,
        1'b0,
        1'b1,
        1'b0,
        1'bx,
        ALU_ADD,
        1'b0,
        1'b0,
        1'bx,
        1'bx,
        1'bx,
        1'bx,
        4'b0001
    };

    parameter logic [19:0] SUBS_CONFIG = {
        1'b1,
        1'b0,
        1'b0,
        1'b1,
        1'b0,
        1'b0,
        1'bx,
        ALU_SUB,
        1'bx,
        1'b1,
        1'bx,
        1'b0,
        1'b0,
        1'bx,
        4'bxxxx
    };

    logic [19:0] B_LT_CONFIG = {
        1'bx,
        1'bx,
        1'bx,
        1'b0,
        1'b0,
        flag_overflow != flag_negative,
        1'b0,
        ALU_DONT_CARE,
        1'bx,
        1'b0,
        1'bx,
        1'bx,
        1'bx,
        1'bx,
        4'bxxxx
    };

    logic [19:0] CBZ_CONFIG = {
        1'b0,
        1'b0,
        1'bx,
        1'b0,
        1'b0,
        zero,
        1'b0,
        ALU_PASS_B,
        1'bx,
        1'b0,
        1'bx,
        1'bx,
        1'bx,
        1'bx,
        4'bxxxx
    };

    // provide inputs
    initial begin 
        $display("Resetting the system...");
        reset <= 1'b1;
        repeat (10) @(posedge clk);
        reset <= 1'b0; 

        if (`BENCHMARK == "benchmarks/test01_AddiB.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);
            // ADDI X0, X31, #0 // X0 = 0
            control_config <= ADDI_CONFIG; @(posedge clk); 

            // ADDI X1, X0, #1 // X1 = 1
            control_config <= ADDI_CONFIG; @(posedge clk); 

            // ADDI X2, X1, #1 // X2 = 2
            control_config <= ADDI_CONFIG; @(posedge clk); 

            // ADDI X3, X1, #2 // X3 = 3
            control_config <= ADDI_CONFIG; @(posedge clk); 

            // ADDI X4, X0, #4 // X4 = 4
            control_config <= ADDI_CONFIG; @(posedge clk); 

            // B 0
            control_config <= B_CONFIG;   
            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test02_AddsSubs.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            // ADDI X0, X31, #1     // X0 =  1
            control_config <= ADDI_CONFIG; @(posedge clk);

            // SUBS X1, X31, X0     // X1 = -1
            control_config <= SUBS_CONFIG; @(posedge clk);

            // SUBS X2, X0, X1      // X2 =  2
            control_config <= SUBS_CONFIG; @(posedge clk);

            // SUBS X3, X1, X2      // X3 = -3
            control_config <= SUBS_CONFIG; @(posedge clk);

            // SUBS X4, X3, X1      // X4 = -2
            control_config <= SUBS_CONFIG; @(posedge clk);

            // ADDS X5, X3, X4      // X5 = -5
            control_config <= ADDS_CONFIG; @(posedge clk);

            // ADDS X6, X0, X1      // X6 = 0
            control_config <= ADDS_CONFIG; @(posedge clk);

            // ADDS X7, X1, X5      // X7 = -6. Flags: negative, carry-out
            control_config <= ADDS_CONFIG; @(posedge clk);

            // ADDI X31, X31, #0    // NOOP - should NOT write the flags.
            control_config <= ADDI_CONFIG; @(posedge clk);

            // B 0
            control_config <= B_CONFIG;

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test03_CbzB.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            control_config <= ADDI_CONFIG; @(posedge clk); // 0
            control_config <= ADDI_CONFIG; @(posedge clk); // 4
            control_config <= ADDI_CONFIG; @(posedge clk); // 8
            control_config <= ADDI_CONFIG; @(posedge clk); // 12
            control_config <= ADDI_CONFIG; @(posedge clk); // 16
            control_config <= ADDI_CONFIG; @(posedge clk); // 20
            control_config <= B_CONFIG; @(posedge clk); // 24 ; +12*4

            control_config <= ADDI_CONFIG; @(posedge clk); // 72
            control_config <= B_CONFIG; @(posedge clk); // 76 ; -7*4

            control_config <= ADDI_CONFIG; @(posedge clk); // 48
            control_config <= CBZ_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= zero; @(posedge clk); // 52 ; +20*4

            control_config <= ADDI_CONFIG; @(posedge clk); // 132
            control_config <= CBZ_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= zero; @(posedge clk); // 136 ; -10*4

            control_config <= ADDI_CONFIG; @(posedge clk); // 96
            control_config <= CBZ_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= zero; @(posedge clk); // 100
            control_config <= ADDI_CONFIG; @(posedge clk); // 104
            control_config <= ADDI_CONFIG; @(posedge clk); // 108 
            control_config <= ADDI_CONFIG; @(posedge clk); // 112

            control_config <= B_CONFIG; // HALT @ 116

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test04_LdurStur.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            // ADDI X0, X31, #1     // X0 = 1
            control_config <= ADDI_CONFIG; @(posedge clk);

            // ADDI X1, X31, #2     // X1 = 2
            control_config <= ADDI_CONFIG; @(posedge clk);

            // ADDI X2, X31, #3     // X2 = 3
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // ADDI X3, X31, #8     // X3 = 8
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // ADDI X4, X31, #11    // X4 = 11
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STUR X0, [X31, #0]   // Mem[0] = 1
            control_config <= STUR_CONFIG; @(posedge clk);

            // STUR X1, [X4,  #-3]  // Mem[8] = 2
            control_config <= STUR_CONFIG; @(posedge clk);

            // STUR X2, [X3, #8]    // Mem[16] = 3
            control_config <= STUR_CONFIG; @(posedge clk);
            
            // LDUR X7, [X4, #5]    // X7 = Mem[16] = 3
            control_config <= LDUR_CONFIG; @(posedge clk);

            // LDUR X5, [X3, #-8]   // X5 = Mem[0] = 1\
            control_config <= LDUR_CONFIG; @(posedge clk);

            // LDUR X6, [X2, #5]    // X6 = Mem[8] = 2
            control_config <= LDUR_CONFIG; @(posedge clk);
            
            // HALT:B HALT          // HALT = 0
            control_config <= B_CONFIG; @(posedge clk);

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test05_Blt.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            // ADDI X0, X31, #1     // X0 = 1, comparison target.
            control_config <= ADDI_CONFIG; @(posedge clk);

            // ADDI X1, X31, #0     // X1 = 0, only set to 1 if we get it all right.
            control_config <= ADDI_CONFIG; @(posedge clk);

            // SUBS X31, X0, X0     // 1-1, not less than.
            control_config <= SUBS_CONFIG; @(posedge clk);

            // B.LT ERROR           // Don't take (+8)
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);

            // ADDI X31, X31, #0    // NOOP
            control_config <= ADDI_CONFIG; @(posedge clk);

            // SUBS X31, X0, X31    // 1 - 0, not less than.
            control_config <= SUBS_CONFIG; @(posedge clk);

            // B.LT ERROR           // Don't take (+5)
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);

            // ADDI X31, X31, #0    // NOOP
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // SUBS X31, X31, X0    // 0 - 1, is less than.
            control_config <= SUBS_CONFIG; @(posedge clk);

            // B.LT SUCCESS         // Take this. (+4)
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);
            
            // SUCCESS:
            // ADDI X1, X1, #1      // Signal correct operation.
            control_config <= ADDI_CONFIG; @(posedge clk);

            // B HALT               // Loop forever (0).
            control_config <= B_CONFIG; @(posedge clk);

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test06_MovkMovz.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            //MAIN:
            // ADDI X0, X31, #0xAAA
            control_config <= ADDI_CONFIG; @(posedge clk);

            // ADDI X1, X31, #0xBBB
            control_config <= ADDI_CONFIG; @(posedge clk);

            // ADDI X2, X31, #0xCCC
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // MOVZ X0, #0xDEAD, LSL 0
            control_config <= MOVZ_CONFIG; @(posedge clk);

            // MOVZ X0, #0xBEEF, LSL 16
            control_config <= MOVZ_CONFIG; @(posedge clk);

            // MOVZ X0, #0xBADD, LSL 32
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            // MOVZ X0, #0xCAFE, LSL 48
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            // ADDI X1, X0, #0
            control_config <= ADDI_CONFIG; @(posedge clk);

            // MOVK X1, #0xDEAD, LSL 32
            control_config <= MOVK_CONFIG; @(posedge clk);

            // MOVK X1, #0xBEEF, LSL 16
            control_config <= MOVK_CONFIG; @(posedge clk);

            // MOVK X1, #0xBADD, LSL 0
            control_config <= MOVK_CONFIG; @(posedge clk);
            
            // ADDI X2, X1, #0
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // MOVZ X2, #0xFADE, LSL 48
            control_config <= MOVZ_CONFIG; @(posedge clk);

            // MOVZ X2, #0xADE, LSL 32
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            // MOVZ X2, #0xDE, LSL 16
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            // MOVZ X2, #0xE, LSL 0
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            // MOVZ X2, #0x0, LSL 0
            control_config <= MOVZ_CONFIG; @(posedge clk);
            
            //END:
            // B to HALT (+0)
            control_config <= B_CONFIG; @(posedge clk);

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test07_LdurbSturb.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            // ADDI X0, X31, #222	// X0 = 0xDE
            control_config <= ADDI_CONFIG; @(posedge clk);

            // STURB X0, [X31, #7]
            control_config <= STURB_CONFIG; @(posedge clk);

            // STURB X0, [X31, #3]
            control_config <= STURB_CONFIG; @(posedge clk);

            // ADDI X0, X31, #173	// X0 = 0xAD
            control_config <= ADDI_CONFIG; @(posedge clk);

            // STURB X0, [X31, #6]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // STURB X0, [X31, #0]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // ADDI X0, X31, #190	// X0 = 0xBE
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STURB X0, [X31, #5]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // ADDI X0, X31, #239	// X0 = 0xEF
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STURB X0, [X31, #4]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // ADDI X0, X31, #202	// X0 = 0xCA
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STURB X0, [X31, #2]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // ADDI X0, X31, #251	// X0 = 0xFB
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STURB X0, [X31, #1]
            control_config <= STURB_CONFIG; @(posedge clk);
            
            // LDUR X1, [X31, #0]	// Read back the full 64 bits
            control_config <= LDUR_CONFIG; @(posedge clk);

            // ADDI X0, X31, #2748	// X0 = 0xABC
            control_config <= ADDI_CONFIG; @(posedge clk);
            
            // STUR X0, [X31, #8]	// Write 0xABC to [15:7], so we can read it back.
            control_config <= STUR_CONFIG; @(posedge clk);

            // LDURB X8, [X31, #8]
            control_config <= LDURB_CONFIG; @(posedge clk);

            // LDURB X9, [X31, #9]
            control_config <= LDURB_CONFIG; @(posedge clk);

            // LDURB X10, [X31, #10]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // LDURB X11, [X31, #11]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // LDURB X12, [X31, #12]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // LDURB X13, [X31, #13]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // LDURB X14, [X31, #14]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // LDURB X15, [X31, #15]
            control_config <= LDURB_CONFIG; @(posedge clk);
            
            // HALT:B HALT          // HALT = 0
            control_config <= B_CONFIG; @(posedge clk);

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else if (`BENCHMARK == "benchmarks/test10_forwarding.arm") begin 
            $display("Running %s at time %0t", `BENCHMARK, $time);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);

            control_config <= STUR_CONFIG; @(posedge clk); 
            control_config <= ADDI_CONFIG; @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= STUR_CONFIG; @(posedge clk);
            control_config <= LDUR_CONFIG; @(posedge clk); 
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);

            control_config <= ADDS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);

            control_config <= ADDS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);

            control_config <= ADDS_CONFIG; @(posedge clk);
            control_config <= SUBS_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= B_LT_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= (flag_overflow != flag_negative); @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= CBZ_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= zero; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= CBZ_CONFIG; #(CLOCK_PERIOD / 4); control_config[14] <= zero; @(posedge clk);

            control_config <= ADDI_CONFIG; @(posedge clk);
            control_config <= B_CONFIG; @(posedge clk);

            repeat (5) begin  
                @(posedge clk);  
            end
            $display("Finished %s at time %0t", `BENCHMARK, $time); 
        end
        else begin 
            $display("Could not find %s", `BENCHMARK);
        end 

        repeat (5) @(posedge clk);  
        $stop; 
    end 

endmodule /* datapath_testbench */