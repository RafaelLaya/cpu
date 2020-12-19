/** The datapath of a 5-stage pipelined implementation of a subset of the ARMv8 ISA 
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
    overflow, negative, zero, carry_out, zero_acc,
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
    SingleByteToReg,
    MemRead,
    MemXferSize,
    instruction,
    busa_forward, busb_forward, busa_forward_src, busb_forward_src,
    Rd_no_delay, Rd_one_delay, Rd_two_delay, Rm_no_delay, Rn_no_delay,
    alu_out_src
);
    // external signals
    input logic clk, reset; 
    
    // control signals
    input logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken;
    input logic UncondBr, AltALUSrc, SetStatus, WithKeep;
    input logic [2:0] ALUOp;
    input logic SingleByteToReg, MemRead;
    input logic [3:0] MemXferSize;
    input logic busa_forward, busb_forward, busa_forward_src, busb_forward_src; 
    input logic alu_out_src;

    // status signals
    output logic overflow, negative, zero, carry_out;
    output logic flag_overflow, flag_negative, flag_zero, flag_carry_out;
    output logic zero_acc;
    output logic [4:0] Rd_no_delay, Rd_one_delay, Rd_two_delay, Rm_no_delay, Rn_no_delay;

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

    // forward signals 
    logic [63:0] alu_forward_val;
    logic [63:0] mem_forward_val;

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

    /* Rd_to_regfile_reg1 */
    logic [4:0] Rd_to_regfile_reg1_dout;
    logic [4:0] Rd_to_regfile_reg1_din;
    register #(.WIDTH(5)) Rd_to_regfile_reg1 (
        .dout(Rd_to_regfile_reg1_dout), 
        .din(Rd_to_regfile_reg1_din), 
        .write_en(1'b1), .reset(reset), .clk(clk)
    );

    /* Rd_to_regfile_reg2 */
    logic [4:0] Rd_to_regfile_reg2_dout;
    logic [4:0] Rd_to_regfile_reg2_din;
    register #(.WIDTH(5)) Rd_to_regfile_reg2 (
        .dout(Rd_to_regfile_reg2_dout), 
        .din(Rd_to_regfile_reg2_din), 
        .write_en(1'b1), 
        .reset(reset), 
        .clk(clk)
    );

    /* Rd_to_regfile_reg3 */
    logic [4:0] Rd_to_regfile_reg3_dout;
    logic [4:0] Rd_to_regfile_reg3_din;
    register #(.WIDTH(5)) Rd_to_regfile_reg3 (
        .dout(Rd_to_regfile_reg3_dout), 
        .din(Rd_to_regfile_reg3_din), 
        .write_en(1'b1), 
        .reset(reset), 
        .clk(clk)
    );

    /* SingleByteToRegMux_to_regfile_reg */
    logic [63:0] SingleByteToRegMux_to_regfile_reg_dout;
    logic [63:0] SingleByteToRegMux_to_regfile_reg_din;
    register #(.WIDTH(64)) SingleByteToRegMux_to_regfile_reg (
        .dout(SingleByteToRegMux_to_regfile_reg_dout), 
        .din(SingleByteToRegMux_to_regfile_reg_din), 
        .write_en(1'b1), 
        .reset(reset), 
        .clk(clk)
    );

    /* regfile_bus1_to_alu_reg */
    logic [63:0] regfile_bus1_to_alu_reg_din;
    logic [63:0] regfile_bus1_to_alu_reg_dout;
    register #(.WIDTH(64)) regfile_bus1_to_alu_reg (
        .dout(regfile_bus1_to_alu_reg_dout),
        .din(regfile_bus1_to_alu_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* regfile_bus2_to_ALUSrcMux_reg */
    logic [63:0] regfile_bus2_to_ALUSrcMux_reg_dout;
    logic [63:0] regfile_bus2_to_ALUSrcMux_reg_din;
    register #(.WIDTH(64)) regfile_bus2_to_ALUSrcMux_reg (
        .dout(regfile_bus2_to_ALUSrcMux_reg_dout),
        .din(regfile_bus2_to_ALUSrcMux_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* regfile_bus2_to_datamem_reg */
    logic [63:0] regfile_bus2_to_datamem_reg_din;
    logic [63:0] regfile_bus2_to_datamem_reg_dout;
    register #(.WIDTH(64)) regfile_bus2_to_datamem_reg (
        .dout(regfile_bus2_to_datamem_reg_dout),
        .din(regfile_bus2_to_datamem_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* alu_to_mem_reg */
    logic [63:0] alu_to_mem_reg_din;
    logic [63:0] alu_to_mem_reg_dout;
    register #(.WIDTH(64)) alu_to_mem_reg (
        .dout(alu_to_mem_reg_dout),
        .din(alu_to_mem_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* pc_to_branch_reg */
    logic [63:0] pc_to_branch_reg_dout;
    logic [63:0] pc_to_branch_reg_din;
    register #(.WIDTH(64)) pc_to_branch_reg (
        .dout(pc_to_branch_reg_dout),
        .din(pc_to_branch_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );
    
    /* instruction_reg */
    logic [31:0] instruction_reg_din;
    logic [31:0] instruction_reg_dout;
    register #(.WIDTH(32)) instruction_reg (
        .dout(instruction_reg_dout),
        .din(instruction_reg_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* Daddr9_reg */
    logic [8:0] Daddr9_reg_din;
    logic [8:0] Daddr9_reg_dout;
    register #(.WIDTH(9)) Daddr9_reg (
        .dout(Daddr9_reg_dout), 
        .din(Daddr9_reg_din), 
        .write_en(1'b1), 
        .reset(reset), 
        .clk(clk)
    );

    /* Imm12_reg */
    logic [11:0] Imm12_reg_din;
    logic [11:0] Imm12_reg_dout;
    register #(.WIDTH(12)) Imm12_reg (
        .dout(Imm12_reg_dout), 
        .din(Imm12_reg_din), 
        .write_en(1'b1), 
        .reset(reset), 
        .clk(clk)
    );

    /* Imm16_reg1 */
    logic [15:0] Imm16_reg1_din;
    logic [15:0] Imm16_reg1_dout;
    register #(.WIDTH(16)) Imm16_reg1 (
        .dout(Imm16_reg1_dout),
        .din(Imm16_reg1_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* shift_sel_reg1 */
    logic [1:0] shift_sel_reg1_din;
    logic [1:0] shift_sel_reg1_dout;
    register #(.WIDTH(2)) shift_sel_reg1 (
        .dout(shift_sel_reg1_dout),
        .din(shift_sel_reg1_din),
        .write_en(1'b1),
        .reset(reset),
        .clk(clk)
    );

    /* zero_checker_acc */
    logic [63:0] zero_checker_acc_in;
    logic zero_checker_acc_out;
    zero_checker zero_checker_acc (.out(zero_checker_acc_out), .in(zero_checker_acc_in));

    /* busa_forward_mux */
    logic [63:0] busa_forward_mux_zero_in;
    logic [63:0] busa_forward_mux_one_in;
    logic [63:0] busa_forward_mux_out; 
    mux2_1_wide #(.WIDTH(64)) busa_forward_mux (
        .out(busa_forward_mux_out), 
        .in({busa_forward_mux_zero_in, busa_forward_mux_one_in}),
        .sel(busa_forward)
    );

    /* busa_forward_src_mux */
    logic [63:0] busa_forward_src_mux_zero_in;
    logic [63:0] busa_forward_src_mux_one_in;
    logic [63:0] busa_forward_src_mux_out;
    mux2_1_wide #(.WIDTH(64)) busa_forward_src_mux (
        .out(busa_forward_src_mux_out),
        .in({busa_forward_src_mux_zero_in, busa_forward_src_mux_one_in}),
        .sel(busa_forward_src)
    );

    /* busb_forward_mux */
    logic [63:0] busb_forward_mux_zero_in;
    logic [63:0] busb_forward_mux_one_in;
    logic [63:0] busb_forward_mux_out; 
    mux2_1_wide #(.WIDTH(64)) busb_forward_mux (
        .out(busb_forward_mux_out), 
        .in({busb_forward_mux_zero_in, busb_forward_mux_one_in}),
        .sel(busb_forward)
    );

    /* busb_forward_src_mux */
    logic [63:0] busb_forward_src_mux_zero_in;
    logic [63:0] busb_forward_src_mux_one_in;
    logic [63:0] busb_forward_src_mux_out;
    mux2_1_wide #(.WIDTH(64)) busb_forward_src_mux (
        .out(busb_forward_src_mux_out),
        .in({busb_forward_src_mux_zero_in, busb_forward_src_mux_one_in}),
        .sel(busb_forward_src)
    );

    /* alu_out_mux */
    logic [63:0] alu_out_src_mux_zero_in;
    logic [63:0] alu_out_src_mux_one_in;
    logic [63:0] alu_out_src_mux_out;
    mux2_1_wide #(.WIDTH(64)) alu_out_src_mux (
        .out(alu_out_src_mux_out), 
        .in({alu_out_src_mux_zero_in, alu_out_src_mux_one_in}), 
        .sel(alu_out_src)
    );

    /* move_calculator */
    logic [63:0] move_calculator_out;
    logic [63:0] move_calculator_Data;
    move_calc move_calculator (
        .out(move_calculator_out), 
        .Data(move_calculator_Data), 
        .shift_sel(shift_sel), 
        .WithKeep(WithKeep), 
        .imm16(imm16)
    );

    /* Make all connections between modules */
    /* The comment refers to who RECEIVES a signal */

    // regfile
    assign regfile_WriteRegister = Rd_to_regfile_reg3_dout;
    assign regfile_ReadRegister2 = reg2locmux_out;
    assign regfile_ReadRegister1 = Rn; 
    assign regfile_WriteData = SingleByteToRegMux_to_regfile_reg_dout;
    assign regfile_RegWrite = RegWrite;

    // alu
    assign alu_A = regfile_bus1_to_alu_reg_dout;
    assign alu_B = ALUSrcMux_out; 
    assign alu_cntrl = ALUOp;

    // datamem
    assign datamem_address = alu_to_mem_reg_dout;
    assign datamem_write_enable = MemWrite;
    assign datamem_write_data = regfile_bus2_to_datamem_reg_dout; 
    assign datamem_xfer_size = MemXferSize; 
    assign datamem_read_enable = MemRead;

    // imem
    assign imem_address = pc_current;

    // instruction
    assign instruction = instruction_reg_dout; 

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
    assign ALUSrcMux_zero_in = regfile_bus2_to_ALUSrcMux_reg_dout;
    assign ALUSrcMux_one_in = AltAluSrcMux_out;

    // MemToRegMux
    assign MemToRegMux_zero_in = alu_to_mem_reg_dout;
    assign MemToRegMux_one_in = datamem_read_data;

    // SingleByteToRegMux
    assign SingleByteToRegMux_zero_in = MemToRegMux_out;
    assign SingleByteToRegMux_one_in = ZE_SingleByte_out;

    // ZE_SingleByte
    // take only the lower 8 wires
    assign ZE_SingleByte_in = MemToRegMux_out[7:0]; 

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
    assign Adder_branch_B = pc_to_branch_reg_dout;

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

    // Rd_to_regfile_reg1 
    assign Rd_to_regfile_reg1_din = Rd; 

    // Rd_to_regfile_reg2
    assign Rd_to_regfile_reg2_din = Rd_to_regfile_reg1_dout;

    // Rd_to_regfile_reg3 
    assign Rd_to_regfile_reg3_din = Rd_to_regfile_reg2_dout; 

    // regfile_bus1_to_alu_reg
    assign regfile_bus1_to_alu_reg_din = busa_forward_mux_out;

    // regfile_bus2_to_ALUSrcMux_reg
    assign regfile_bus2_to_ALUSrcMux_reg_din = busb_forward_mux_out;
    
    // regfile_bus2_to_datamem_reg
    assign regfile_bus2_to_datamem_reg_din = regfile_bus2_to_ALUSrcMux_reg_dout;

    // alu_to_mem_reg
    assign alu_to_mem_reg_din = alu_out_src_mux_out;

    // SingleByteToRegMux_to_regfile_reg
    assign SingleByteToRegMux_to_regfile_reg_din = SingleByteToRegMux_out;

    // pc_to_branch_reg
    assign pc_to_branch_reg_din = pc_current;

    // instruction_reg
    assign instruction_reg_din = imem_instruction; 
    
    // Daddr9_reg
    assign Daddr9_reg_din = instruction[20:12];

    // Imm12_reg
    assign Imm12_reg_din = instruction[21:10];

    // shift_sel_reg1
    assign shift_sel_reg1_din = instruction[22:21];

    // Imm16_reg1
    assign Imm16_reg1_din = instruction[20:5];

    // zero_checker_acc
    assign zero_checker_acc_in = busb_forward_mux_out;

    // busa_forward_mux
    assign busa_forward_mux_zero_in = regfile_ReadData1;
    assign busa_forward_mux_one_in = busa_forward_src_mux_out;

    // busa_forward_src_mux
    assign busa_forward_src_mux_zero_in = alu_forward_val;
    assign busa_forward_src_mux_one_in = mem_forward_val;

    // busb_forward_mux
    assign busb_forward_mux_zero_in = regfile_ReadData2;
    assign busb_forward_mux_one_in = busb_forward_src_mux_out;

    // busb_forward_src_mux
    assign busb_forward_src_mux_zero_in = alu_forward_val;
    assign busb_forward_src_mux_one_in = mem_forward_val;

    // alu_out_src_mux
    assign alu_out_src_mux_zero_in = alu_result;
    assign alu_out_src_mux_one_in = move_calculator_out;

    // move_calculator
    assign move_calculator_Data = ALUSrcMux_out;

    // connect forward signals 
    assign alu_forward_val = alu_out_src_mux_out;
    assign mem_forward_val = SingleByteToRegMux_out;

    // connect status signals needed for forwarding
    assign Rd_no_delay = Rd;
    assign Rd_one_delay = Rd_to_regfile_reg1_dout;
    assign Rd_two_delay = Rd_to_regfile_reg2_dout;
    assign Rm_no_delay = Rm;
    assign Rn_no_delay = Rn;

    // Get fields from the current instruction
    assign Rd = instruction[4:0];
    assign Rn = instruction[9:5];
    assign Rm = instruction[20:16];
    assign DAddr9 = Daddr9_reg_dout;
    assign Imm12 = Imm12_reg_dout;
    assign shift_sel = shift_sel_reg1_dout;
    assign CondAddr19 = instruction[23:5];
    assign BrAddr26 = instruction[25:0];
    assign imm16 = Imm16_reg1_dout;

    /* Connect Outputs */ 
    assign carry_out = alu_carry_out;   assign flag_carry_out = flag_carry_out_current;
    assign overflow = alu_overflow;     assign flag_overflow = flag_overflow_current;
    assign negative = alu_negative;     assign flag_negative = flag_negative_current;
    assign zero = alu_zero;             assign flag_zero = flag_zero_current;
    assign zero_acc = zero_checker_acc_out;

endmodule /* datapath */
