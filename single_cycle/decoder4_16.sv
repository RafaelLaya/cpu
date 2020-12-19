
/** 4:16 enabled decoder
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
module decoder4_16(out, sel, en);
	input logic en;
	input logic [3:0] sel;
	output logic [15:0] out;
	
	// The most significant selection bit discards half of the signals
	logic [1:0] decoder_selected;
	decoder1_2 d0(.out(decoder_selected), .sel(sel[3]), .en(en));
	
	// All other bits select the correct signal
	decoder3_8 d1(.out(out[15:8]), .sel(sel[2:0]), .en(decoder_selected[1]));
	decoder3_8 d2(.out(out[7:0]), .sel(sel[2:0]), .en(decoder_selected[0]));
endmodule /* decoder4_16 */

/* testbench for decoder4_16 */
module decoder4_16_testbench();
	logic en;
	logic [3:0] sel;
	logic [15:0] out;
	
	decoder4_16 dut(.*);
	
	// provide inputs
	initial begin
		// test all patterns
		for (integer i = 0; i < 2 ** 5; i++) begin
			{en, sel} = i; #(TESTBENCH_DELAY);
			
			// check each of the output bits
			for (integer j = 0; j < 16; j++) begin
				if (!en) 		assert(!out[j]) else $error("Expected out[%0d]==false at %0t", sel, $time);
				else if (j == sel) 	assert(out[j]) else $error("Expected out[%0d]==true at %0t", sel, $time);
				else assert(!out[j]) else $error("Expected out[%0d]==false at %0t", sel, $time);
			end
		end
		$stop;
	end
endmodule /* decoder4_16_testbench */

