/** 1-bit slice of the ALU
 *
 * Inputs:
 *		A: One bit of the first operand
 *      B: One bit of the second operand
 *      carry_in: The input bit on the carry chain for this slice
 *      op_select:  Indicates which operation is performed. See alu.sv for more
 *                  information
 *
 *	Outputs:
 *		carry_out: The output bit on the carry chain for this slice
 *      ALU_bit: The output bit of this slice of the ALU
 *
 */
`include "delays.sv"
module ALU_bit_slice(carry_out, ALU_bit, A, B, carry_in, op_select);
    input logic A, B, carry_in;
    input logic [2:0] op_select;
    output logic carry_out, ALU_bit; 

    logic hold_B_result, full_adder_result, and_result, or_result, xor_result;

    // do the simple operations
    assign hold_B_result = B; 
    and #(GATE_DELAY) (and_result, A, B);
    or #(GATE_DELAY) (or_result, A, B);
    xor #(GATE_DELAY) (xor_result, A, B);

    // addition and substraction
    logic nB, B_full_adder;
    not #(GATE_DELAY) (nB, B);
    mux2_1_wide #(.WIDTH(1)) B_selector (.out(B_full_adder), .in({B, nB}), .sel(op_select[0]));
    full_adder adder (.carry_out(carry_out), .S(full_adder_result), .carry_in(carry_in), .A(A), .B(B_full_adder));

    mux8_1_wide #(.WIDTH(1)) result_selector (
        .out(ALU_bit), 
        .in({hold_B_result, 0, full_adder_result, full_adder_result, and_result, or_result, xor_result, 0}), 
        .sel(op_select)
    );

endmodule /* ALU_bit_slice */

/* testbench for ALU_bit_slice */
module ALU_bit_slice_testbench();
    logic A, B, carry_in;
    logic [2:0] op_select;
    logic carry_out, ALU_bit; 

    ALU_bit_slice dut (.*);

    parameter HOLD_B_CODE = 3'b000;
    parameter ADD_CODE = 3'b010;
    parameter SUBSTRACT_CODE = 3'b011;
    parameter AND_CODE = 3'b100;
    parameter OR_CODE = 3'b101;
    parameter XOR_CODE = 3'b110;

    // provide inputs
    logic [1:0] expected;
    initial begin
        $display("Testing HOLD_B");
        op_select = HOLD_B_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            assert(ALU_bit == B) 
            else $error("Expected ALU_bit=%0x but got %0x at time %0t", B, ALU_bit, $time);
        end
        
        $display("Testing ADD_CODE");
        op_select = ADD_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            expected = A + B + carry_in;
            assert({carry_out, ALU_bit} == expected) 
            else $error("Expected {carry_out, ALU_bit}=%0x but got %0x at time %0t", expected, {carry_out, ALU_bit}, $time);
        end

        $display("Testing SUBSTRACT_B");
        op_select = SUBSTRACT_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            expected = 2'(A) + 1'(~B) + 2'(carry_in);
            assert({carry_out, ALU_bit} == expected)
            else $error("Expected {carry_out, ALU_bit}=%0x but got %0x at time %0t", expected, {carry_out, ALU_bit}, $time);
            #(TESTBENCH_DELAY);
        end

        $display("Testing AND_B");
        op_select = AND_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            assert(ALU_bit == (A & B)) 
            else $error("Expected ALU_bit=%0x but got %0x at time %0t", A&B, ALU_bit, $time);
        end

        $display("Testing OR_B");
        op_select = OR_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            assert(ALU_bit == (A | B)) 
            else $error("Expected ALU_bit=%0x but got %0x at time %0t", A|B, ALU_bit, $time);
        end

        $display("Testing XOR_B");
        op_select = XOR_CODE;
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} = i;
            #(TESTBENCH_DELAY);
            assert(ALU_bit == (A ^ B)) 
            else $error("Expected ALU_bit=%0x but got %0x at time %0t", A^B, ALU_bit, $time);
        end

        #(TESTBENCH_DELAY);
        $stop;
    end

endmodule /* ALU_bit_slice_testbench */