

/** Checks two 5-bit quantities for equality
 *
 * Inputs:
 *		A: One operand
 *      B: Another operand
 *
 *	Outputs:
 *		equal: True if A==B, otherwise false
 *
 */
`include "delays.sv"
module equality_checker(equal, A, B);
    input logic [4:0] A;
    input logic [4:0] B;
    output logic equal;

    // xor(a, b) == 0 if and only if a==b
    logic [4:0] xor_bus; 
    genvar i;
	generate
		for (i=0; i < 5; i++) begin : each_xor
            xor #(GATE_DELAY) xor_i (xor_bus[i], A[i], B[i]);
		end
	endgenerate 

    // Thus the whole vector xor_bus==64'b0 if and only if A[i]=B[i] for 0<=i<64
    // Which means A==B
    logic intermediate_0_1, intermediate_2_3;
    logic nequal;
    or #(GATE_DELAY) o0 (intermediate_0_1, xor_bus[0], xor_bus[1]);
    or #(GATE_DELAY) o1 (intermediate_2_3, xor_bus[2], xor_bus[3]);
    or #(GATE_DELAY) o2 (intermediate_0_3, intermediate_0_1, intermediate_2_3);
    or #(GATE_DELAY) o3 (nequal, intermediate_0_3, xor_bus[4]);
    not #(GATE_DELAY) n0 (equal, nequal);

endmodule /* equality_checker */

/* testbench for equality_checker */
module equality_checker_testbench();
    parameter NUMBER_OF_TESTS = 2048 * 10;
    logic [4:0] A;
    logic [4:0] B;
    logic equal;

    equality_checker dut (.*);

    // provide inputs
    initial begin 
        for (integer i = 0; i < 2 ** 4; i++) begin 
            for (integer j = 0; j < 2 ** 4; j++) begin 
                A = i;
                B = j;
                #(TESTBENCH_DELAY);
                assert(equal == (A == B));
            end 
        end 

        $stop;
    end 
endmodule /* equality_checker */