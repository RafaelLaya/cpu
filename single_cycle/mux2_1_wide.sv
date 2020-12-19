/** 2:1 multiplexer of parametrized width
 *
 * Inputs:
 *		in: 2 input signals of 64 bits each, out of which the multiplexer selects one
 *		sel: The selector bit of the multiplexer
 *
 *	Outputs:
 *		out: in[sel]
 *
 * Selects 1 signal out of 2 input signals based on sel
 */
`include "delays.sv"
module mux2_1_wide #(parameter WIDTH=64) (out, in, sel);
	input logic [WIDTH-1:0] in [0:1];
	input logic sel;
	output logic [WIDTH-1:0] out;
	
	genvar i;
	generate
		for (i=0; i < WIDTH; i++) begin : eachMux
			mux2_1 m (.out(out[i]), .i0(in[0][i]), .i1(in[1][i]), .sel(sel));
		end 
	endgenerate 
endmodule // mux2_1_wide

/* testbench for mux2_1_wide */
module mux2_1_wide_testbench();
	parameter WIDTH=64;
	parameter NUMBER_OF_TESTS=32;
	logic [WIDTH-1:0] in [0:1];
	logic [WIDTH-1:0] out;
	logic sel;
	
	mux2_1_wide #(.WIDTH(WIDTH)) dut (.*);
	
	// provide inputs
	initial begin
		sel <= 1'b0;
		#(TESTBENCH_DELAY);
		for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin
			// randomize the values
			for (integer j = 0; j < 2; j++) in[j] = $urandom_range(0, 2 ** WIDTH - 1);
			
			sel <= ~sel;
			#(TESTBENCH_DELAY);
			
			assert(out == in[sel]) else $error("Expected %0x but got %0x at time %0t", in[sel], out, $time);
		end 
		$stop;
	end // initial
endmodule // mux2_1_wide_testbench