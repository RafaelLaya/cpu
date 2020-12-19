
/** Performs a logical left shift by two
 *  
 *  Parameters:
 *      IN_LENGTH: The length of the input signal to left-shift
 *
 *  Inputs:
 *		in: The input signal to left-shift
 *
 *	Outputs:
 *		out: The input after it has been left-shifted
 */
`include "delays.sv"
module lsl_2 #(parameter IN_LENGTH=8) (in, out);
    input logic [IN_LENGTH-1:0] in;
    output logic [IN_LENGTH-1:0] out;

    // The upper wires are all just passed
    assign out[IN_LENGTH-1:2] = in[IN_LENGTH-3:0];

    // The lower two are zero
    assign out[1:0] = 2'b00; 

endmodule /* lsl_2 */

/* testbench for lsl_2 */
module lsl_2_testbench();
    parameter NUMBER_OF_TESTS=2048;
    parameter IN_LENGTH=64;
    logic [IN_LENGTH-1:0] in;
    logic [IN_LENGTH-1:0] out;

    lsl_2 #(.IN_LENGTH(IN_LENGTH)) dut (.*);
    
    // provide inputs
    initial begin 
        for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin 
            in = i; 
            #(TESTBENCH_DELAY);
            assert(out == (in << 2)); 

            in = -i; 
            #(TESTBENCH_DELAY); 
            assert(out == (in << 2)); 
        end

        for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin
            in = {$random(), $random()};
            #(TESTBENCH_DELAY);
            assert(out == (in << 2));

            in = -in; 
            #(TESTBENCH_DELAY);
            assert(out == (in << 2));
        end
        $stop;
    end 

endmodule /* lsl_testbench */