/** 4:1 multiplexer of parametrized width
 *
 * Inputs:
 *		in: 4 input signals of 64 bits each, out of which the multiplexer selects one
 *		sel: The selector signal of the multiplexer
 *
 *	Outputs:
 *		out: in[sel]
 *
 * Selects 1 signal out of 4 input signals based on sel
 */
`include "delays.sv"
module mux4_1_wide #(parameter WIDTH=64) (out, in, sel);
	output logic [WIDTH-1:0] out;
	input logic [WIDTH-1:0] in [0:3];
	input logic [1:0] sel;
	
	// semi_final meaning there are only two possibilities left
	logic [WIDTH-1:0] in_semi_final [0:1];
	
	// each mux selects one out of the semi-finalists by ignoring the most significant selection bit
	mux2_1_wide #(.WIDTH(WIDTH)) m0(.out(in_semi_final[0]), .in(in[0:1]), .sel(sel[0]));
	mux2_1_wide #(.WIDTH(WIDTH)) m1(.out(in_semi_final[1]), .in(in[2:3]), .sel(sel[0]));
	
	// the most significant selection bit selects one of the semi-finalists
	mux2_1_wide #(.WIDTH(WIDTH)) m2(.out(out), .in(in_semi_final), .sel(sel[1]));
endmodule // mux4_1_wide

/* testbench for mux4_1_wide */
module mux4_1_wide_testbench();
	parameter WIDTH=64;
	parameter NUMBER_OF_TESTS=128;
	logic [WIDTH-1:0] out;
	logic [WIDTH-1:0] in [0:3];
	logic [1:0] sel;
	
	mux4_1_wide #(.WIDTH(WIDTH)) dut(.*);
	
	// provide inputs
	initial begin
		sel <= 2'b0;
		#(TESTBENCH_DELAY);
		
		// do a reasonable amount of tests
		for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin
			// randomize the values
			for (integer j = 0; j < 4; j++) in[j] <= $urandom_range(0, 2 ** WIDTH - 1);
			
			// select one on a round robin basis
			sel <= sel + 2'b1;
			#(TESTBENCH_DELAY);
			
			// check the output is correct
			assert(out == in[sel]) else $error("Expected %0x but got %0x at time %0t", in[sel], out, $time);
		end 
		$stop;
	end 
endmodule /* mux4_1_wide_testbench */

