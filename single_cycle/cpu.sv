/** A single-cycle implementation of a subset of the ARMv8 ISA 
 *
 * Inputs:
 *      clk: The clock signal
 *      reset: An active-high reset signal (synchronous on positive edges of clk)
 */
`include "delays.sv"
`include "benchmark.sv"
module cpu(clk, reset);
    input logic clk, reset;
    
    // control signals
    logic Reg2Loc, ALUSrc, MemToReg, RegWrite, MemWrite, BrTaken;
    logic UncondBr, AltALUSrc, SetStatus, WithKeep, SpecialToReg;
    logic [2:0] ALUOp;
    logic SingleByteToReg, MemRead;
    logic [3:0] MemXferSize;

    // status signals
    logic overflow, negative, zero, carry_out;
    logic flag_overflow, flag_negative, flag_zero, flag_carry_out;
    logic [31:0] instruction; 

    datapath datapath_unit (
        .clk, .reset,
        .overflow, .negative, .zero, .carry_out, 
        .flag_overflow, .flag_negative, .flag_zero, .flag_carry_out,
        .Reg2Loc,
        .ALUSrc,
        .MemToReg,
        .RegWrite,
        .MemWrite,
        .BrTaken,
        .UncondBr,
        .ALUOp,
        .AltALUSrc,
        .SetStatus,
        .WithKeep,
        .SpecialToReg,
        .SingleByteToReg,
        .MemRead,
        .MemXferSize,
        .instruction
    );

    control control_unit (
        .overflow, .negative, .zero, .carry_out,
        .flag_overflow, .flag_negative, .flag_zero, .flag_carry_out,
        .Reg2Loc,
        .ALUSrc,
        .MemToReg,
        .RegWrite,
        .MemWrite,
        .BrTaken,
        .UncondBr,
        .ALUOp,
        .AltALUSrc,
        .SetStatus,
        .WithKeep,
        .SpecialToReg,
        .SingleByteToReg,
        .MemRead,
        .MemXferSize,
        .instruction
    );
endmodule /* cpu */

/* testbench for cpu */
module cpu_testbench();
    logic clk, reset; 

    cpu dut (.clk, .reset);

    // simulate the clock
    initial begin 
        clk <= 1'b0;
        forever begin 
            #(TESTBENCH_DELAY);
            clk <= ~clk;
        end
    end 

    initial begin 
        // reset system to a known state
        reset <= 1'b1; 
        @(posedge clk);
        reset <= 1'b0;
        $display("Starting...");
        @(posedge clk);

        // wait enough cycles for program to finish 
        repeat (1000) @(posedge clk);

        if (`BENCHMARK == "benchmarks/test01_AddiB.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == 64'd3);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == 64'd4);
        end 
        else if (`BENCHMARK == "benchmarks/test02_AddsSubs.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == -64'd1);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == -64'd3);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == -64'd2);
            assert(dut.datapath_unit.register_file.eachReg[5].reg_i.dout == -64'd5);
            assert(dut.datapath_unit.register_file.eachReg[6].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[7].reg_i.dout == -64'd6);
            assert(dut.datapath_unit.flag_negative == 1'b1);
            assert(dut.datapath_unit.flag_carry_out == 1'b1);
            assert(dut.datapath_unit.flag_overflow == 1'b0);
            assert(dut.datapath_unit.flag_zero == 1'b0);
        end 
        else if (`BENCHMARK == "benchmarks/test03_CbzB.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == 64'd31);
            assert(dut.datapath_unit.register_file.eachReg[5].reg_i.dout == 64'd0);
        end 
        else if (`BENCHMARK == "benchmarks/test04_LdurStur.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd3);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == 64'd8);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == 64'd11);
            assert(dut.datapath_unit.register_file.eachReg[5].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[6].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[7].reg_i.dout == 64'd3);
            
            assert(dut.datapath_unit.data_memory.mem[7] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[6] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[5] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[4] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[3] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[2] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[1] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[0] == 8'd1);

            assert(dut.datapath_unit.data_memory.mem[15] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[14] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[13] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[12] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[11] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[10] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[9] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[8] == 8'd2);

            assert(dut.datapath_unit.data_memory.mem[23] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[22] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[21] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[20] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[19] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[18] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[17] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[16] == 8'd3);
        end 
        else if (`BENCHMARK == "benchmarks/test05_Blt.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd1);
        end 
        else if (`BENCHMARK == "benchmarks/test06_MovkMovz.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'hCAFE000000000000);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'hCAFEDEADBEEFBADD);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'h0);
        end 
        else if (`BENCHMARK == "benchmarks/test07_LdurbSturb.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'hABC);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'hDEADBEEFDECAFBAD);
            assert(dut.datapath_unit.register_file.eachReg[8].reg_i.dout == 64'hBC);
            assert(dut.datapath_unit.register_file.eachReg[9].reg_i.dout == 64'h0A);
            assert(dut.datapath_unit.register_file.eachReg[10].reg_i.dout == 64'h0);
            assert(dut.datapath_unit.register_file.eachReg[11].reg_i.dout == 64'h0);
            assert(dut.datapath_unit.register_file.eachReg[12].reg_i.dout == 64'h0);
            assert(dut.datapath_unit.register_file.eachReg[13].reg_i.dout == 64'h0);
            assert(dut.datapath_unit.register_file.eachReg[14].reg_i.dout == 64'h0);
            assert(dut.datapath_unit.register_file.eachReg[15].reg_i.dout == 64'h0);

            assert(dut.datapath_unit.data_memory.mem[7] == 8'hDE);
            assert(dut.datapath_unit.data_memory.mem[6] == 8'hAD);
            assert(dut.datapath_unit.data_memory.mem[5] == 8'hBE);
            assert(dut.datapath_unit.data_memory.mem[4] == 8'hEF);
            assert(dut.datapath_unit.data_memory.mem[3] == 8'hDE);
            assert(dut.datapath_unit.data_memory.mem[2] == 8'hCA);
            assert(dut.datapath_unit.data_memory.mem[1] == 8'hFB);
            assert(dut.datapath_unit.data_memory.mem[0] == 8'hAD);
            

            assert(dut.datapath_unit.data_memory.mem[15] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[14] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[13] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[12] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[11] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[10] == 8'h00);
            assert(dut.datapath_unit.data_memory.mem[9] == 8'h0A);
            assert(dut.datapath_unit.data_memory.mem[8] == 8'hBC);
        end 
        else if (`BENCHMARK == "benchmarks/test10_forwarding.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd8);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == 64'd5);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == 64'd7);
            assert(dut.datapath_unit.register_file.eachReg[5].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[6].reg_i.dout == -64'd2);
            assert(dut.datapath_unit.register_file.eachReg[7].reg_i.dout == -64'd2);
            assert(dut.datapath_unit.register_file.eachReg[8].reg_i.dout == 64'd0);
            assert(dut.datapath_unit.register_file.eachReg[9].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[10].reg_i.dout == -64'd4);
            assert(dut.datapath_unit.register_file.eachReg[14].reg_i.dout == 64'd5);
            assert(dut.datapath_unit.register_file.eachReg[15].reg_i.dout == 64'd8);
            assert(dut.datapath_unit.register_file.eachReg[16].reg_i.dout == 64'd9);
            assert(dut.datapath_unit.register_file.eachReg[17].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[18].reg_i.dout == 64'd99);
            
            assert(dut.datapath_unit.data_memory.mem[7] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[6] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[5] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[4] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[3] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[2] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[1] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[0] == 8'd8);

            assert(dut.datapath_unit.data_memory.mem[15] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[14] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[13] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[12] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[11] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[10] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[9] == 8'd0);
            assert(dut.datapath_unit.data_memory.mem[8] == 8'd5);
        end
        else if (`BENCHMARK == "benchmarks/test11_Sort.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[11].reg_i.dout == 64'd1);
            assert(dut.datapath_unit.register_file.eachReg[12].reg_i.dout == 64'd2);
            assert(dut.datapath_unit.register_file.eachReg[13].reg_i.dout == 64'd3);
            assert(dut.datapath_unit.register_file.eachReg[14].reg_i.dout == 64'd4);
            assert(dut.datapath_unit.register_file.eachReg[15].reg_i.dout == 64'd5);
            assert(dut.datapath_unit.register_file.eachReg[16].reg_i.dout == 64'd6);
            assert(dut.datapath_unit.register_file.eachReg[17].reg_i.dout == 64'd7);
            assert(dut.datapath_unit.register_file.eachReg[18].reg_i.dout == 64'd8);
            assert(dut.datapath_unit.register_file.eachReg[19].reg_i.dout == 64'd9);
            assert(dut.datapath_unit.register_file.eachReg[20].reg_i.dout == 64'd10);
        end 
        else if (`BENCHMARK == "benchmarks/test12_ToUpper.arm") begin 
            $display("Running %s", `BENCHMARK);
            assert(dut.datapath_unit.register_file.eachReg[0].reg_i.dout == 64'd80);
            assert(dut.datapath_unit.register_file.eachReg[1].reg_i.dout == 64'd79);
            assert(dut.datapath_unit.register_file.eachReg[2].reg_i.dout == 64'd97);
            assert(dut.datapath_unit.register_file.eachReg[3].reg_i.dout == 64'd122);
            assert(dut.datapath_unit.register_file.eachReg[4].reg_i.dout == 64'd32);
            assert(dut.datapath_unit.register_file.eachReg[5].reg_i.dout == 64'd87);
            assert(dut.datapath_unit.register_file.eachReg[7].reg_i.dout == 64'd114028278768209169);
            assert(dut.datapath_unit.register_file.eachReg[8].reg_i.dout == 64'd1);

            assert(dut.datapath_unit.data_memory.mem[80] =="W");
            assert(dut.datapath_unit.data_memory.mem[81] =="O");
            assert(dut.datapath_unit.data_memory.mem[82] =="W");
            assert(dut.datapath_unit.data_memory.mem[83] ==",");
            assert(dut.datapath_unit.data_memory.mem[84] ==" ");
            assert(dut.datapath_unit.data_memory.mem[85] =="Y");
            assert(dut.datapath_unit.data_memory.mem[86] =="O");
            assert(dut.datapath_unit.data_memory.mem[87] =="U");
            assert(dut.datapath_unit.data_memory.mem[88] ==" ");
            assert(dut.datapath_unit.data_memory.mem[89] =="A");
            assert(dut.datapath_unit.data_memory.mem[90] =="C");
            assert(dut.datapath_unit.data_memory.mem[91] =="T");
            assert(dut.datapath_unit.data_memory.mem[92] =="U");
            assert(dut.datapath_unit.data_memory.mem[93] =="A");
            assert(dut.datapath_unit.data_memory.mem[94] =="L");
            assert(dut.datapath_unit.data_memory.mem[95] =="L");
            assert(dut.datapath_unit.data_memory.mem[96] =="Y");
            assert(dut.datapath_unit.data_memory.mem[97] ==" ");
            assert(dut.datapath_unit.data_memory.mem[98] =="T");
            assert(dut.datapath_unit.data_memory.mem[99] =="R");
            assert(dut.datapath_unit.data_memory.mem[100] =="A");
            assert(dut.datapath_unit.data_memory.mem[101] =="N");
            assert(dut.datapath_unit.data_memory.mem[102] =="S");
            assert(dut.datapath_unit.data_memory.mem[103] =="L");
            assert(dut.datapath_unit.data_memory.mem[104] =="A");
            assert(dut.datapath_unit.data_memory.mem[105] =="T");
            assert(dut.datapath_unit.data_memory.mem[106] =="E");
            assert(dut.datapath_unit.data_memory.mem[107] =="D");
            assert(dut.datapath_unit.data_memory.mem[108] ==" ");
            assert(dut.datapath_unit.data_memory.mem[109] =="A");
            assert(dut.datapath_unit.data_memory.mem[110] =="L");
            assert(dut.datapath_unit.data_memory.mem[111] =="L");
            assert(dut.datapath_unit.data_memory.mem[112] ==" ");
            assert(dut.datapath_unit.data_memory.mem[113] =="O");
            assert(dut.datapath_unit.data_memory.mem[114] =="F");
            assert(dut.datapath_unit.data_memory.mem[115] ==" ");
            assert(dut.datapath_unit.data_memory.mem[116] =="T");
            assert(dut.datapath_unit.data_memory.mem[117] =="H");
            assert(dut.datapath_unit.data_memory.mem[118] =="I");
            assert(dut.datapath_unit.data_memory.mem[119] =="S");
            assert(dut.datapath_unit.data_memory.mem[120] =="?");
            assert(dut.datapath_unit.data_memory.mem[121] ==" ");
            assert(dut.datapath_unit.data_memory.mem[122] =="N");
            assert(dut.datapath_unit.data_memory.mem[123] =="I");
            assert(dut.datapath_unit.data_memory.mem[124] =="C");
            assert(dut.datapath_unit.data_memory.mem[125] =="E");
            assert(dut.datapath_unit.data_memory.mem[126] ==" ");
            assert(dut.datapath_unit.data_memory.mem[127] =="J");
            assert(dut.datapath_unit.data_memory.mem[128] =="O");
            assert(dut.datapath_unit.data_memory.mem[129] =="B");
            assert(dut.datapath_unit.data_memory.mem[130] =="!");
        end 
        else $display("Could not find %s", `BENCHMARK);

        $display("Finished!");
        $stop;
    end 
endmodule /* cpu_testbench */
