
/** Implements a simple zero extender unit
 *  
 *  Parameters:
 *      IN_LENGTH: The length of the input signal to zero extend
 *      OUT_LENGTH: The length of the zero-extended quantity
 *
 *  Inputs:
 *		in: The input signal to zero extend
 *
 *	Outputs:
 *		out: The zero-extended quantity
 *
 *  Zero extends a number by connecting wires as appropriate
 */
`include "delays.sv"
module zero_extender #(parameter IN_LENGTH=8, OUT_LENGTH=64) (in, out);
    input logic [IN_LENGTH-1:0] in;
    output logic [OUT_LENGTH-1:0] out;

    // The first IN_LENGTH bits follow the relation: out[i] = in[i]
    // i.e.: Just pass the wires
    assign out[IN_LENGTH-1:0] = in;

    // The rest are just zeros
    assign out[OUT_LENGTH-1:IN_LENGTH] = { (OUT_LENGTH - IN_LENGTH) {1'b0} };

endmodule /* zero_extender */

/* testbench for zero_extender */
module zero_extender_testbench();
    parameter NUMBER_OF_TESTS=2048;
    parameter IN_LENGTH=9;
    parameter OUT_LENGTH=64;
    logic [IN_LENGTH-1:0] in;
    logic [OUT_LENGTH-1:0] out;

    zero_extender #(.IN_LENGTH(IN_LENGTH), .OUT_LENGTH(OUT_LENGTH)) dut (.*);
    
    // provide inputs
    initial begin 
        for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin 
            in = i; 
            #(TESTBENCH_DELAY);
            assert(out == in); 

            in = -i; 
            #(TESTBENCH_DELAY); 
            assert(out == in); 
        end

        for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin
            in = $random();
            #(TESTBENCH_DELAY);
            assert(out == in);

            in = -in; 
            #(TESTBENCH_DELAY);
            assert(out == in);
        end
        $stop;
    end 

endmodule /* zero_extender_testbench */