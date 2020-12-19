/** Checks if the input is zero when interpreted as an integer (unsigned or 2's complement)
 *
 * Inputs:
 *		in: The input to check
 *      
 *	Outputs:
 *      out: True if the input is zero. False otherwise.
 */
`include "delays.sv"
module zero_checker(out, in);
    input logic [63:0] in;
    output logic out;

    // the input is of 64-bits, so we need log2(64)=6 levels of 2-input
    // or-gates. Then we will negate the result. This module is essentially
    // a very big NOR gate
    logic [31:0] first_level;
    genvar i;
	generate
		for (i=0; i < 32; i++) begin : each_first_level_pair
            or #(GATE_DELAY) (first_level[i], in[i*2], in[i*2+1]);
		end
	endgenerate 

    logic [15:0] second_level;
    generate 
        for (i=0; i < 16; i++) begin : each_second_level_pair
            or #(GATE_DELAY) (second_level[i], first_level[i*2], first_level[i*2+1]);
        end
    endgenerate

    logic [7:0] third_level;
    generate 
        for (i=0; i < 8; i++) begin : each_third_level_pair
            or #(GATE_DELAY) (third_level[i], second_level[i*2], second_level[i*2+1]);
        end
    endgenerate

    logic [3:0] fourth_level;
    generate 
        for (i=0; i < 4; i++) begin : each_fourth_level_pair
            or #(GATE_DELAY) (fourth_level[i], third_level[i*2], third_level[i*2+1]);
        end
    endgenerate

    logic [1:0] fifth_level;
    logic nout;
    or #(GATE_DELAY) (fifth_level[0], fourth_level[0], fourth_level[1]);
    or #(GATE_DELAY) (fifth_level[1], fourth_level[2], fourth_level[3]);

    // sixth level
    or #(GATE_DELAY) (nout, fifth_level[0], fifth_level[1]);
    
    // make it a nor gate by negating the output
    not #(GATE_DELAY) (out, nout);
endmodule /* zero_checker */

/* testbench for zero_checker */
module zero_checker_testbench();
    parameter NUMBER_OF_RANDOM_TESTS = 1024 * 10;
    logic [63:0] in;
    logic out;

    zero_checker dut (.*);

    // provide inputs
    initial begin 
        // test the first 1024 numbers
        for (integer i = 0; i < 24; i++) begin 
            in <= i; 
            #(TESTBENCH_DELAY); 
            if(in == 0) assert(out == 1'b1) else $error("Expected out=1 but got %0b while i=%0x", out, in);
            else assert (out == 1'b0) else $error("Expected out=0 but got %0b while i=%0x", out, in);
        end

        // test the last 1024 numbers
        for (integer i = 0; i < 1024; i++) begin 
            in <= 64'hFF_FF_FF_FF_FF_FF_FF_FF - unsigned'(i); 
            #(TESTBENCH_DELAY); 
            if(in == 0) assert(out == 1'b1) else $error("Expected out=1 but got %0b while i=%0x", out, in);
            else assert (out == 1'b0) else $error("Expected out=0 but got %0b while i=%0x", out, in);
        end

        // test some random numbers
        for (integer i = 0; i < NUMBER_OF_RANDOM_TESTS; i++) begin 
            in <= {$urandom(), $urandom()};
            #(TESTBENCH_DELAY); 
            if(in == 0) assert(out == 1'b1) else $error("Expected out=1 but got %0b while i=%0x", out, in);
            else assert (out == 1'b0) else $error("Expected out=0 but got %0b while i=%0x", out, in);
        end

        $stop;
    end
endmodule /* zero_checker_testbench */