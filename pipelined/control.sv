/** The control unit of a 5-stage pipelined implementation of a subset of the ARMv8 ISA 
 *
 * Inputs:
 *      overflow, negative, zero, carry_out: Current flags of the ALU
 *      flag_overflow, flag_negative, flag_zero, flag_carry_out: The values of the status register
 *      instruction: The current instruction being executed
 *
 *	Outputs:
 *      The control signals that the datapath will respond to
 *
 */
`include "delays.sv"
module control(
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
    clk, reset,
    busa_forward, busb_forward, busa_forward_src, busb_forward_src,
    Rd_no_delay, Rd_one_delay, Rd_two_delay, Rm_no_delay, Rn_no_delay,
    alu_out_src
);
    // external signals
    input logic clk, reset;

    // ALU signals
    input logic overflow, negative, zero, carry_out, zero_acc;

    // status registers
    input logic flag_overflow, flag_negative, flag_zero, flag_carry_out;

    // other status signals
    input logic [4:0] Rd_no_delay, Rd_one_delay, Rd_two_delay, Rm_no_delay, Rn_no_delay;

    // control signals (outputs)
    output logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken, UncondBr;
    output logic AltALUSrc, SetStatus, WithKeep, SingleByteToReg, MemRead;
    output logic [3:0] MemXferSize;
    output logic [2:0] ALUOp;
    output logic busa_forward, busb_forward, busa_forward_src, busb_forward_src;
    output logic alu_out_src;

    // current instruction to consider
    input logic [31:0] instruction;

    // control signals (with zero delay)
    logic Reg2Loc_no_delay, ALUSrc_no_delay, MemToReg_no_delay, RegWrite_no_delay, MemWrite_no_delay, BrTaken_no_delay, UncondBr_no_delay;
    logic AltALUSrc_no_delay, SetStatus_no_delay, WithKeep_no_delay, SingleByteToReg_no_delay, MemRead_no_delay;
    logic [3:0] MemXferSize_no_delay;
    logic [2:0] ALUOp_no_delay;
    logic busa_forward_no_delay, busb_forward_no_delay, busa_forward_src_no_delay, busb_forward_src_no_delay; 
    logic alu_out_src_no_delay;

    // control signals (with one delay)
    logic ALUSrc_one_delay, MemToReg_one_delay, RegWrite_one_delay, MemWrite_one_delay;
    logic AltALUSrc_one_delay, SetStatus_one_delay, WithKeep_one_delay, SingleByteToReg_one_delay;
    logic MemRead_one_delay;
    logic [3:0] MemXferSize_one_delay;
    logic [2:0] ALUOp_one_delay;
    logic alu_out_src_one_delay;

    // control signals (with two delays)
    logic MemToReg_two_delay, RegWrite_two_delay, MemWrite_two_delay;
    logic SingleByteToReg_two_delay;
    logic MemRead_two_delay;
    logic [3:0] MemXferSize_two_delay;

    // control signals (with three delays)
    logic RegWrite_three_delay;

    // opcodes
    enum logic [10:0] {
        ADDI_OPCODE =   11'b 100_1000_100x,
        ADDS_OPCODE =   11'b 101_0101_1000,
        B_OPCODE    =   11'b 000_101x_xxxx,
        B_LT_OPCODE =   11'b 010_1010_0xxx,
        CBZ_OPCODE  =   11'b 101_1010_0xxx,
        LDUR_OPCODE =   11'b 111_1100_0010,
        LDURB_OPCODE=   11'b 001_1100_0010,
        MOVK_OPCODE =   11'b 111_1001_01xx,
        MOVZ_OPCODE =   11'b 110_1001_01xx,
        STUR_OPCODE =   11'b 111_1100_0000,
        STURB_OPCODE=   11'b 001_1100_0000,
        SUBS_OPCODE =   11'b 111_0101_1000    
    } OP_CODES;

    // ALU operations 
    enum logic [2:0] {
        ALU_PASS_B = 3'b000,
        ALU_ADD = 3'b010, 
        ALU_SUB = 3'b011,
        ALU_AND = 3'b100,
        ALU_OR = 3'b101,
        ALU_XOR = 3'b110,
        ALU_DONT_CARE = 3'bxxx
    } ALU_CNTRL_CODES;

    // forwarding logic 
    always_comb begin 
        busa_forward_no_delay = 1'b0;
        busb_forward_no_delay = 1'b0;
        busa_forward_src_no_delay = 1'b0;
        busb_forward_src_no_delay = 1'b0;

        // if memory stage wants to write to the regfile 
        if (RegWrite_two_delay) begin 
            // if the register that the memory stage wants to write matches the current read on bus a
            if (Rd_two_delay == Rn_no_delay && (Rn_no_delay != 5'd31)) begin 
                // forward the newer value from memory to bus a
                busa_forward_no_delay = 1'b1;
                busa_forward_src_no_delay = 1'b1;
            end
        end 

        // if alu stage wants to write to the regfile
        if (RegWrite_one_delay) begin 
            // if the register that the alu stage wants to write matches the current read on bus a
            if (Rd_one_delay == Rn_no_delay && (Rn_no_delay != 5'd31)) begin 
                // forward value from alu to bus a (takes precedence over a write on memory stage)
                busa_forward_no_delay = 1'b1;
                busa_forward_src_no_delay = 1'b0;
            end 
        end 

        // if memory stage wants to write to the regfile 
        if (RegWrite_two_delay) begin 
            // if the register that the memory stage wants to write matches the current read on bus b
            if (((Rd_two_delay == Rm_no_delay) && Reg2Loc) && (Rm_no_delay != 5'd31)) begin 
                // forward the value from memory stage to bus b
                busb_forward_no_delay = 1'b1;
                busb_forward_src_no_delay = 1'b1;
            end 
            else if (((Rd_two_delay == Rd_no_delay) && !Reg2Loc) && (Rd_no_delay != 5'd31)) begin 
                // forward the value from memory stage to bus b
                busb_forward_no_delay = 1'b1;
                busb_forward_src_no_delay = 1'b1;
            end
        end 

        // if alu stage wants to write to the regfile 
        if (RegWrite_one_delay) begin 
            // if the register that the alu stage wants to write matches the current read on bus b
            if (((Rd_one_delay == Rm_no_delay) && Reg2Loc) && (Rm_no_delay != 5'd31)) begin 
                // forward value from alu to bus b (takes precedence over a write on memory stage)
                busb_forward_no_delay = 1'b1;
                busb_forward_src_no_delay = 1'b0;
            end
            else if (((Rd_one_delay == Rd_no_delay) && !Reg2Loc) && (Rd_no_delay != 5'd31)) begin 
                // forward value from alu to bus b (takes precedence over a write on memory stage)
                busb_forward_no_delay = 1'b1;
                busb_forward_src_no_delay = 1'b0;
            end 
        end
    end 

    // control logic on decode stage
    always_comb begin 
        casex (instruction[31:21])
            ADDI_OPCODE: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b 1; 
                MemToReg_no_delay =          1'b 0; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b 1;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'b0;
            end 

            ADDS_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 1;
                ALUSrc_no_delay =            1'b 0; 
                MemToReg_no_delay =          1'b 0; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 1;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'b0;
            end 

            B_OPCODE: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b x; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 1;
                UncondBr_no_delay =          1'b 1; 
                ALUOp_no_delay =             ALU_DONT_CARE;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x; 
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'bx;
            end 

            B_LT_OPCODE: begin // no need to check lower bits, since we didn't implement any other B.cond 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b x; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 0;
                UncondBr_no_delay =          1'b 0; 
                ALUOp_no_delay =             ALU_DONT_CARE;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x; 
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'bx;

                if (SetStatus) begin 
                    // If the next instruction is about to update the flags, then take these newer flag values
                    BrTaken_no_delay = overflow != negative;
                end 
                else begin 
                    // Otherwise use the flags
                    BrTaken_no_delay = flag_overflow != flag_negative;
                end                                 
            end 

            CBZ_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 0;
                ALUSrc_no_delay =            1'b 0; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           zero_acc;
                UncondBr_no_delay =          1'b 0; 
                ALUOp_no_delay =             ALU_PASS_B;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx;
                alu_out_src_no_delay =       1'b0; 
            end 

            LDUR_OPCODE: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b 1; 
                MemToReg_no_delay =          1'b 1; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b 0;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b 1; 
                MemXferSize_no_delay =       4'b 1000; 
                alu_out_src_no_delay =       1'b0;
            end 

            LDURB_OPCODE: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b 1; 
                MemToReg_no_delay =          1'b 1; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b 0;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b 1; 
                MemRead_no_delay =           1'b 1; 
                MemXferSize_no_delay =       4'b 0001; 
                alu_out_src_no_delay =       1'b0;
            end 

            MOVK_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 0;
                ALUSrc_no_delay =            1'b 0; 
                MemToReg_no_delay =          1'b 0; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_PASS_B;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b 1;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx;
                alu_out_src_no_delay =       1'b1; 
            end 

            MOVZ_OPCODE: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b x; 
                MemToReg_no_delay =          1'b 0; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_DONT_CARE;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b 0;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'b1;
            end

            STUR_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 0;
                ALUSrc_no_delay =            1'b 1; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 1;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b 0;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b 1000; 
                alu_out_src_no_delay =       1'b0;
            end
            
            STURB_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 0;
                ALUSrc_no_delay =            1'b 1; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 1;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_ADD;
                AltALUSrc_no_delay =         1'b 0;
                SetStatus_no_delay =         1'b 0;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b 0001; 
                alu_out_src_no_delay =       1'b0;
            end

            SUBS_OPCODE: begin 
                Reg2Loc_no_delay =           1'b 1;
                ALUSrc_no_delay =            1'b 0; 
                MemToReg_no_delay =          1'b 0; 
                RegWrite_no_delay =          1'b 1;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_SUB;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b 1;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b 0; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'b0;
            end

            default: begin 
                Reg2Loc_no_delay =           1'b x;
                ALUSrc_no_delay =            1'b x; 
                MemToReg_no_delay =          1'b x; 
                RegWrite_no_delay =          1'b 0;
                MemWrite_no_delay =          1'b 0;
                BrTaken_no_delay =           1'b 0;
                UncondBr_no_delay =          1'b x; 
                ALUOp_no_delay =             ALU_DONT_CARE;
                AltALUSrc_no_delay =         1'b x;
                SetStatus_no_delay =         1'b x;
                WithKeep_no_delay =          1'b x;
                SingleByteToReg_no_delay =   1'b x; 
                MemRead_no_delay =           1'b x; 
                MemXferSize_no_delay =       4'b xxxx; 
                alu_out_src_no_delay =       1'bx;
            end
        endcase 
    end

    // connect signals that don't need a delay
    assign UncondBr = UncondBr_no_delay;
    assign BrTaken = BrTaken_no_delay;
    assign Reg2Loc = Reg2Loc_no_delay;
    assign busa_forward = busa_forward_no_delay;
    assign busb_forward = busb_forward_no_delay;
    assign busa_forward_src = busa_forward_src_no_delay;
    assign busb_forward_src = busb_forward_src_no_delay;

    // produce signals with one delay
    always_ff @(posedge clk) begin 
        if (reset) begin 
            ALUSrc_one_delay <= 1'b0;
            MemToReg_one_delay <= 1'b0;
            RegWrite_one_delay <= 1'b0;
            MemWrite_one_delay <= 1'b0;
            AltALUSrc_one_delay <= 1'b0;
            SetStatus_one_delay <= 1'b0;
            WithKeep_one_delay <= 1'b0;
            SingleByteToReg_one_delay <= 1'b0;
            MemRead_one_delay <= 1'b0;
            MemXferSize_one_delay <= 4'b0000;
            ALUOp_one_delay <= 3'b000;
            alu_out_src_one_delay <= 1'b0;
        end 
        else begin 
            ALUSrc_one_delay <= ALUSrc_no_delay;
            MemToReg_one_delay <= MemToReg_no_delay;
            RegWrite_one_delay <= RegWrite_no_delay;
            MemWrite_one_delay <= MemWrite_no_delay;
            AltALUSrc_one_delay <= AltALUSrc_no_delay;
            SetStatus_one_delay <= SetStatus_no_delay;
            WithKeep_one_delay <= WithKeep_no_delay;
            SingleByteToReg_one_delay <= SingleByteToReg_no_delay;
            MemRead_one_delay <= MemRead_no_delay;
            MemXferSize_one_delay <= MemXferSize_no_delay;
            ALUOp_one_delay <= ALUOp_no_delay;
            alu_out_src_one_delay <= alu_out_src_no_delay;
        end
    end 

    // connect signals that only need one delay
    assign ALUOp = ALUOp_one_delay;
    assign SetStatus = SetStatus_one_delay;
    assign AltALUSrc = AltALUSrc_one_delay;
    assign ALUSrc = ALUSrc_one_delay;
    assign alu_out_src = alu_out_src_one_delay;
    assign WithKeep = WithKeep_one_delay;

    // produce signals that need two delays
    always_ff @(posedge clk) begin 
        if (reset) begin 
            MemToReg_two_delay <= 1'b0;
            RegWrite_two_delay <= 1'b0;
            MemWrite_two_delay <= 1'b0;
            SingleByteToReg_two_delay <= 1'b0;
            MemRead_two_delay <= 1'b0;
            MemXferSize_two_delay <= 4'b0000;
        end 
        else begin 
            MemToReg_two_delay <= MemToReg_one_delay;
            RegWrite_two_delay <= RegWrite_one_delay;
            MemWrite_two_delay <= MemWrite_one_delay;
            SingleByteToReg_two_delay <= SingleByteToReg_one_delay;
            MemRead_two_delay <= MemRead_one_delay;
            MemXferSize_two_delay <= MemXferSize_one_delay;
        end 
    end

    // connect signals that only need two delays
    assign MemWrite = MemWrite_two_delay;
    assign MemToReg = MemToReg_two_delay;
    assign MemRead = MemRead_two_delay;
    assign MemXferSize = MemXferSize_two_delay;
    assign SingleByteToReg = SingleByteToReg_two_delay;

    // produce signals with three delays
    always_ff @(posedge clk) begin 
        if (reset) begin 
            RegWrite_three_delay <= 1'b0;
        end 
        else begin 
            RegWrite_three_delay <= RegWrite_two_delay;
        end 
    end 

    // connect signals that only need three delays
    assign RegWrite = RegWrite_three_delay;

endmodule /* control */
