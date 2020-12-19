
/** 2:4 enabled decoder
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
module decoder2_4(out, sel, en);
	input logic en;
	input logic [1:0] sel;
	output logic [3:0] out;
	
	// The most significant selection bit discards half of the signals
	logic [1:0] decoder_selected;
	decoder1_2 d0(.out(decoder_selected), .sel(sel[1]), .en(en));
	
	// All other bits select the correct signal
	decoder1_2 d1(.out(out[3:2]), .sel(sel[0]), .en(decoder_selected[1]));
	decoder1_2 d2(.out(out[1:0]), .sel(sel[0]), .en(decoder_selected[0]));
endmodule /* decoder2_4 */

/* testbench for decoder2_4 */
module decoder2_4_testbench();
	logic en;
	logic [1:0] sel;
	logic [3:0] out;
	
	decoder2_4 dut(.*);
	
	// provide inputs
	initial begin
		// test all patterns
		for (integer i = 0; i < 2 ** 3; i++) begin
			{en, sel} = i; #(TESTBENCH_DELAY);
			
			// check each of the output bits
			for (integer j = 0; j < 4; j++) begin
				if (!en) assert(!out[j]) else $error("Expected out[%0d]==false at %0t", sel, $time);
				else if	(j == sel) assert(out[j]) else $error("Expected out[%0d]==true at %0t", sel, $time);
				else assert(!out[j]) else $error("Expected out[%0d]==false at %0t", sel, $time);
			end 
		end
		$stop;
	end
endmodule /* decoder2_4_testbench */

