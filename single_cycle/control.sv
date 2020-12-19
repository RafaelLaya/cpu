/** The control unit of a single-cycle implementation of a subset of the ARMv8 ISA 
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
    // ALU signals
    input logic overflow, negative, zero, carry_out;

    // status registers
    input logic flag_overflow, flag_negative, flag_zero, flag_carry_out;

    // control signals
    output logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken, UncondBr;
    output logic AltALUSrc, SetStatus, WithKeep, SpecialToReg, SingleByteToReg, MemRead;
    output logic [3:0] MemXferSize;
    output logic [2:0] ALUOp;

    // current instruction to consider
    input logic [31:0] instruction;

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

    always_comb begin 
        casex (instruction[31:21])
            ADDI_OPCODE: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b 1; 
                MemToReg =          1'b 0; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b 1;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b 0; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            ADDS_OPCODE: begin 
                Reg2Loc =           1'b 1;
                ALUSrc =            1'b 0; 
                MemToReg =          1'b 0; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 1;
                WithKeep =          1'b x;
                SpecialToReg =      1'b 0; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            B_OPCODE: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b x; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 0;
                MemWrite =          1'b 0;
                BrTaken =           1'b 1;
                UncondBr =          1'b 1; 
                ALUOp =             ALU_DONT_CARE;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            B_LT_OPCODE: begin // no need to check lower bits, since we didn't implement any other B.cond 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b x; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 0;
                MemWrite =          1'b 0;
                BrTaken =           flag_overflow != flag_negative;
                UncondBr =          1'b 0; 
                ALUOp =             ALU_DONT_CARE;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            CBZ_OPCODE: begin 
                Reg2Loc =           1'b 0;
                ALUSrc =            1'b 0; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 0;
                MemWrite =          1'b 0;
                BrTaken =           zero;
                UncondBr =          1'b 0; 
                ALUOp =             ALU_PASS_B;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            LDUR_OPCODE: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b 1; 
                MemToReg =          1'b 1; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b 0;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b 0; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b 1; 
                MemXferSize =       4'b 1000; 
            end 

            LDURB_OPCODE: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b 1; 
                MemToReg =          1'b 1; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b 0;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b 0; 
                SingleByteToReg =   1'b 1; 
                MemRead =           1'b 1; 
                MemXferSize =       4'b 0001; 
            end 

            MOVK_OPCODE: begin 
                Reg2Loc =           1'b 0;
                ALUSrc =            1'b 0; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_PASS_B;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 0;
                WithKeep =          1'b 1;
                SpecialToReg =      1'b 1; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end 

            MOVZ_OPCODE: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b x; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_DONT_CARE;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 0;
                WithKeep =          1'b 0;
                SpecialToReg =      1'b 1; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end

            STUR_OPCODE: begin 
                Reg2Loc =           1'b 0;
                ALUSrc =            1'b 1; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 0;
                MemWrite =          1'b 1;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b 0;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b 1000; 
            end
            
            STURB_OPCODE: begin 
                Reg2Loc =           1'b 0;
                ALUSrc =            1'b 1; 
                MemToReg =          1'b x; 
                RegWrite =          1'b 0;
                MemWrite =          1'b 1;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_ADD;
                AltALUSrc =         1'b 0;
                SetStatus =         1'b 0;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b 0001; 
            end

            SUBS_OPCODE: begin 
                Reg2Loc =           1'b 1;
                ALUSrc =            1'b 0; 
                MemToReg =          1'b 0; 
                RegWrite =          1'b 1;
                MemWrite =          1'b 0;
                BrTaken =           1'b 0;
                UncondBr =          1'b x; 
                ALUOp =             ALU_SUB;
                AltALUSrc =         1'b x;
                SetStatus =         1'b 1;
                WithKeep =          1'b x;
                SpecialToReg =      1'b 0; 
                SingleByteToReg =   1'b 0; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end

            default: begin 
                Reg2Loc =           1'b x;
                ALUSrc =            1'b x; 
                MemToReg =          1'b x; 
                RegWrite =          1'b x;
                MemWrite =          1'b x;
                BrTaken =           1'b x;
                UncondBr =          1'b x; 
                ALUOp =             ALU_DONT_CARE;
                AltALUSrc =         1'b x;
                SetStatus =         1'b x;
                WithKeep =          1'b x;
                SpecialToReg =      1'b x; 
                SingleByteToReg =   1'b x; 
                MemRead =           1'b x; 
                MemXferSize =       4'b xxxx; 
            end
        endcase 
    end

endmodule /* control */

/* testbench for control */
module control_testbench();
    // ALU signals
    logic overflow, negative, zero, carry_out;

    // status registers
    logic flag_overflow, flag_negative, flag_zero, flag_carry_out;

    // control signals
    logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken, UncondBr;
    logic AltALUSrc, SetStatus, WithKeep, SpecialToReg, SingleByteToReg, MemRead;
    logic [3:0] MemXferSize;
    logic [2:0] ALUOp;

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

    // current instruction to consider
    logic [31:0] instruction;

    control dut (.*);

    initial begin 
        $display("Starting...");
        overflow = 1'b0;
        negative = 1'b0; 
        zero = 1'b0; 
        carry_out = 1'b0;
        flag_overflow = 1'b0; 
        flag_negative = 1'b0;
        flag_zero = 1'b0; 
        flag_carry_out = 1'b0;

        // ADDI
        instruction = 32'b1001000100_000000000001_11111_00000;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===           1'b x);
        assert(ALUSrc ==             1'b 1); 
        assert(MemToReg ==           1'b 0); 
        assert(RegWrite ==           1'b 1);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            1'b 0);
        assert(UncondBr ===          1'b x); 
        assert(ALUOp ==              ALU_ADD);
        assert(AltALUSrc ==          1'b 1);
        assert(SetStatus ==          1'b 0);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ==       1'b 0); 
        assert(SingleByteToReg ==    1'b 0); 
        assert(MemRead ===           1'b x); 
        assert(MemXferSize ===       4'b xxxx);

        #(TESTBENCH_DELAY);

        // ADDS
        instruction = 32'b10101011000_00001_000000_00000_00110;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==            1'b 1);
        assert(ALUSrc ==             1'b 0); 
        assert(MemToReg ==           1'b 0); 
        assert(RegWrite ==           1'b 1);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            1'b 0);
        assert(UncondBr ===          1'b x); 
        assert(ALUOp ==              ALU_ADD);
        assert(AltALUSrc ===         1'b x);
        assert(SetStatus ==          1'b 1);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ==       1'b 0); 
        assert(SingleByteToReg ==    1'b 0); 
        assert(MemRead ===           1'b x); 
        assert(MemXferSize ===       4'b xxxx);

        #(TESTBENCH_DELAY);

        // B
        instruction = 32'b000101_00000000000000000000000000;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===            1'b x);
        assert(ALUSrc ===             1'b x); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 0);
        assert(MemWrite ==            1'b 0);
        assert(BrTaken ==             1'b 1);
        assert(UncondBr ==            1'b 1); 
        assert(ALUOp ===              ALU_DONT_CARE);
        assert(AltALUSrc ===          1'b x);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ===           1'b x);
        assert(SpecialToReg ===       1'b x); 
        assert(SingleByteToReg ===    1'b x); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ===        4'b xxxx);

        #(TESTBENCH_DELAY);

        // B_LT
        instruction = 32'b01010100_0000000000000001000_01011;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===            1'b x);
        assert(ALUSrc ===             1'b x); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 0);
        assert(MemWrite ==            1'b 0);
        assert(BrTaken ==             flag_overflow != flag_negative);
        assert(UncondBr ==            1'b 0); 
        assert(ALUOp ===              ALU_DONT_CARE);
        assert(AltALUSrc ===          1'b x);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ===           1'b x);
        assert(SpecialToReg ===       1'b x); 
        assert(SingleByteToReg ===    1'b x); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ===        4'b xxxx);

        #(TESTBENCH_DELAY);

        // CBZ
        instruction = 32'b10110100_0000000000000000110_00000;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==            1'b 0);
        assert(ALUSrc ==             1'b 0); 
        assert(MemToReg ===          1'b x); 
        assert(RegWrite ==           1'b 0);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            zero);
        assert(UncondBr ==           1'b 0); 
        assert(ALUOp ==              ALU_PASS_B);
        assert(AltALUSrc ===         1'b x);
        assert(SetStatus ==          1'b 0);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ===      1'b x); 
        assert(SingleByteToReg ===   1'b x); 
        assert(MemRead ===           1'b x); 
        assert(MemXferSize ===       4'b xxxx);

        #(TESTBENCH_DELAY);

        // LDUR
        instruction = 32'b11111000010_000000101_00_00100_00111;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===           1'b x);
        assert(ALUSrc ==             1'b 1); 
        assert(MemToReg ==           1'b 1); 
        assert(RegWrite ==           1'b 1);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            1'b 0);
        assert(UncondBr ===          1'b x); 
        assert(ALUOp ==              ALU_ADD);
        assert(AltALUSrc ==          1'b 0);
        assert(SetStatus ==          1'b 0);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ==       1'b 0); 
        assert(SingleByteToReg ==    1'b 0); 
        assert(MemRead ==            1'b 1); 
        assert(MemXferSize ==        4'b 1000);

        #(TESTBENCH_DELAY);

        // LDURB
        instruction = 32'b00111000010_000001000_00_11111_01000;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===           1'b x);
        assert(ALUSrc ==             1'b 1); 
        assert(MemToReg ==           1'b 1); 
        assert(RegWrite ==           1'b 1);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            1'b 0);
        assert(UncondBr ===          1'b x); 
        assert(ALUOp ==              ALU_ADD);
        assert(AltALUSrc ==          1'b 0);
        assert(SetStatus ==          1'b 0);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ==       1'b 0); 
        assert(SingleByteToReg ==    1'b 1); 
        assert(MemRead ==            1'b 1); 
        assert(MemXferSize ==        4'b 0001);

        #(TESTBENCH_DELAY);

        // MOVK
        instruction = 32'b111100101_10_1101111010101101_00001;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==             1'b 0);
        assert(ALUSrc ==              1'b 0); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 1);
        assert(MemWrite ==            1'b 0);
        assert(BrTaken ==             1'b 0);
        assert(UncondBr ===           1'b x); 
        assert(ALUOp ==               ALU_PASS_B);
        assert(AltALUSrc ===          1'b x);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ==            1'b 1);
        assert(SpecialToReg ==        1'b 1); 
        assert(SingleByteToReg ==     1'b 0); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ===        4'b xxxx);

        #(TESTBENCH_DELAY);

        // MOVZ
        instruction = 32'b110100101_11_1111101011011110_00010;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ===            1'b x);
        assert(ALUSrc ===             1'b x); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 1);
        assert(MemWrite ==            1'b 0);
        assert(BrTaken ==             1'b 0);
        assert(UncondBr ===           1'b x); 
        assert(ALUOp ===              ALU_DONT_CARE);
        assert(AltALUSrc ===          1'b x);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ==            1'b 0);
        assert(SpecialToReg ==        1'b 1); 
        assert(SingleByteToReg ==     1'b 0); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ===        4'b xxxx);

        #(TESTBENCH_DELAY);

        // STUR
        instruction = 32'b11111000000_111111101_00_00100_00001;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==             1'b 0);
        assert(ALUSrc ==              1'b 1); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 0);
        assert(MemWrite ==            1'b 1);
        assert(BrTaken ==             1'b 0);
        assert(UncondBr ===           1'b x); 
        assert(ALUOp ==               ALU_ADD);
        assert(AltALUSrc ==           1'b 0);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ===           1'b x);
        assert(SpecialToReg ===       1'b x); 
        assert(SingleByteToReg ===    1'b x); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ==        4'b 1000);

        #(TESTBENCH_DELAY);

        // STURB
        instruction = 32'b00111000000_000000001_00_11111_00000;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==             1'b 0);
        assert(ALUSrc ==              1'b 1); 
        assert(MemToReg ===           1'b x); 
        assert(RegWrite ==            1'b 0);
        assert(MemWrite ==            1'b 1);
        assert(BrTaken ==             1'b 0);
        assert(UncondBr ===           1'b x); 
        assert(ALUOp ==               ALU_ADD);
        assert(AltALUSrc ==           1'b 0);
        assert(SetStatus ==           1'b 0);
        assert(WithKeep ===           1'b x);
        assert(SpecialToReg ===       1'b x); 
        assert(SingleByteToReg ===    1'b x); 
        assert(MemRead ===            1'b x); 
        assert(MemXferSize ==         4'b 0001);

        #(TESTBENCH_DELAY);

        // SUBS
        instruction = 32'b11101011000_00000_000000_11111_00001;
        #(TESTBENCH_DELAY);

        assert(Reg2Loc ==            1'b 1);
        assert(ALUSrc ==             1'b 0); 
        assert(MemToReg ==           1'b 0); 
        assert(RegWrite ==           1'b 1);
        assert(MemWrite ==           1'b 0);
        assert(BrTaken ==            1'b 0);
        assert(UncondBr ===          1'b x); 
        assert(ALUOp ==              ALU_SUB);
        assert(AltALUSrc ===         1'b x);
        assert(SetStatus ==          1'b 1);
        assert(WithKeep ===          1'b x);
        assert(SpecialToReg ==       1'b 0); 
        assert(SingleByteToReg ==    1'b 0); 
        assert(MemRead ===           1'b x); 
        assert(MemXferSize ===       4'b xxxx);

        #(TESTBENCH_DELAY);
        
        $display("Finished!");
        $stop; 
    end 
    
endmodule /* control_testbench */

