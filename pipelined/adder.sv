
/** Performs an addition of 2's complements or unsigned numbers
 *  
 *  Parameters:
 *      IN_LENGTH: The length of the input signals
 *
 *  Inputs:
 *		A: The first operand
 *      B: The second operand
 *
 *	Outputs:
 *		sum: The sum of A and B
 */
`include "delays.sv"
module adder #(parameter IN_LENGTH=64) (A, B, sum);
    input logic [IN_LENGTH-1:0] A;
    input logic [IN_LENGTH-1:0] B;
    output logic [IN_LENGTH-1:0] sum;

    // There are one more carry than adders
    logic [IN_LENGTH:0] carry_chain;

    // Generate all but the first full adder
    genvar i;
	generate
		for (i=1; i < IN_LENGTH; i++) begin : each_slice
            full_adder adder_i (.carry_out(carry_chain[i+1]), .S(sum[i]), .carry_in(carry_chain[i]), .A(A[i]), .B(B[i]));
		end
	endgenerate 

    // do the first full adder manually
    full_adder adder_0 (.carry_out(carry_chain[1]), .S(sum[0]), .carry_in(1'b0), .A(A[0]), .B(B[0]));

endmodule /* adder */

/* testbench for adder */
module adder_testbench();
    parameter IN_LENGTH=64;

    logic [IN_LENGTH-1:0] A;
    logic [IN_LENGTH-1:0] B;
    logic signed [IN_LENGTH-1:0] A_signed;
    logic signed [IN_LENGTH-1:0] B_signed;
    
    logic [IN_LENGTH-1:0] sum;

    adder #(.IN_LENGTH(IN_LENGTH)) dut (.*);

    parameter NUMBER_OF_FIXED_TEST_VECTORS = 25;
    parameter NUMBER_OF_RANDOM_TEST_VECTORS = 2048;
    parameter logic [63:0] FIXED_TEST_VECTORS[] = {
        64'h_10_00_00_00_00_00_00_00,
        64'h_00_00_00_00_00_00_00_00,
        64'h_00_00_00_00_00_00_00_01,
        64'h_00_00_00_00_00_00_00_02,
        64'h_00_00_00_00_00_00_00_03,
        64'h_00_00_00_00_00_00_00_04,
        64'h_00_00_00_00_00_00_00_05,
        64'h_00_00_00_00_00_00_00_06,
        64'h_00_00_00_00_00_00_00_07,
        64'h_FF_FF_FF_FF_FF_FF_FF_FF,
        64'h_FF_FF_FF_FF_FF_FF_FF_FE,
        64'h_FF_FF_FF_FF_FF_FF_FF_FD,
        64'h_FF_FF_FF_FF_FF_FF_FF_FC,
        64'h_FF_FF_FF_FF_FF_FF_FF_FB,
        64'h_FF_FF_FF_FF_FF_FF_FF_FA,
        64'h_FF_FF_FF_FF_FF_FF_FF_F9,
        64'h_FF_FF_FF_FF_FF_FF_FF_F8,
        64'h_FF_FF_FF_FF_FF_FF_FF_F7,
        64'h_AA_AA_AA_AA_AA_AA_AA_AA,
        64'h_55_55_55_55_55_55_55_55,
        64'h_FF_ED_CB_A9_87_65_43_21,
        64'h_CA_BC_DE_F0_12_34_56_78,
        64'h_BA_AA_AA_AA_AA_AA_AA_AA,
        64'h_00_00_00_01_00_00_00_00,
        64'h_00_00_00_FF_00_00_00_00
    };
    
    // provide inputs
    initial begin 
        $display("Running Fixed Tests");
        for (integer i = 0; i < NUMBER_OF_FIXED_TEST_VECTORS; i++) begin 
            for (integer j = 0; j < NUMBER_OF_FIXED_TEST_VECTORS; j++) begin 
                A = FIXED_TEST_VECTORS[i]; A_signed = A;
                B = FIXED_TEST_VECTORS[j]; B_signed = B;
                #(TESTBENCH_DELAY);
                assert(sum == (A+B));

                A_signed = FIXED_TEST_VECTORS[i]; A = A_signed;
                B_signed = FIXED_TEST_VECTORS[j]; B = B_signed;
                #(TESTBENCH_DELAY);
                assert(sum == (A_signed + B_signed));
            end 
        end

        $display("Running Random Tests");
        for (integer i = 0; i < NUMBER_OF_RANDOM_TEST_VECTORS; i++) begin 
            A = {$urandom(), $urandom()}; A_signed = A;
            B = {$urandom(), $urandom()}; B_signed = B;
            #(TESTBENCH_DELAY);
            assert(sum == (A+B)); 
            
            A_signed = {$random(), $random()}; A = A_signed;
            B_signed = {$random(), $random()}; B = B_signed;
            #(TESTBENCH_DELAY); 
            assert(sum == (A_signed+B_signed)); 
        end

        $stop;
    end 

endmodule /* adder_testbench */