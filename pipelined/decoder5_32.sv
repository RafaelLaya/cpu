
/** 5:32 enabled decoder
 *
 * Inputs:
 *		en: active-high enable signal
 *		sel: chooses which output wire will be set to true
 *
 *	Outputs:
 *		out: 	When enabled, out[sel] is true and others are false
 *				When not enabled, all bits are zero
 *
 */
`include "delays.sv"
module decoder5_32(out, sel, en);
	input logic en;
	input logic [4:0] sel;
	output logic [31:0] out;
	
	// The most significant selection bit discards half of the signals
	logic [1:0] decoder_selected;
	decoder1_2 d0(.out(decoder_selected), .sel(sel[4]), .en(en));
	
	// All other bits select the correct signal
	decoder4_16 d1(.out(out[31:16]), .sel(sel[3:0]), .en(decoder_selected[1]));
	decoder4_16 d2(.out(out[15:0]), .sel(sel[3:0]), .en(decoder_selected[0]));
endmodule /* decoder5_32 */

/* testbench for decoder5_32 */
module decoder5_32_testbench();
	logic en;
	logic [4:0] sel;
	logic [31:0] out;
	
	decoder5_32 dut(.*);
	
	// provide inputs
	initial begin
		// test all patterns
		for (integer i = 0; i < 2 ** 6; i++) begin
			{en, sel} = i; #(TESTBENCH_DELAY);
			
			// check each of the output bits
			for (integer j = 0; j < 32; j++) begin
				if (!en) 		assert(!out[j]) 	else $error("Expected out[%0d]==false at %0t", sel, $time);
				else if (j == sel) 	assert(out[j]) 	else $error("Expected out[%0d]==true at %0t", sel, $time);
				else assert(!out[j]) 	else $error("Expected out[%0d]==false at %0t", sel, $time);
			end
		end
		$stop;
	end
endmodule /* decoder5_32_testbench */

