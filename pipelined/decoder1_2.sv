
/** 1:2 enabled decoder
 *
 * Inputs:
 *		en: active-high enable signal
 *		sel: chooses which output wire will be set to true
 *
 *	Outputs:
 *		out: 	When enabled, out[sel] is true and others are false
 *				When not enabled, all bits are zero
 */
`include "delays.sv"
module decoder1_2(out, sel, en);
	input logic en, sel;
	output logic [1:0] out;
	
	logic nsel;
	not #(GATE_DELAY) (nsel, sel);
	
	and #(GATE_DELAY) (out[1], en, sel);
	and #(GATE_DELAY) (out[0], en, nsel);
endmodule /* decoder1_2 */

/* testbench for decoder1_2 */
module decoder1_2_testbench();
	logic sel, en;
	logic [1:0] out;
	decoder1_2 dut(.*);
	
	// provide inputs
	initial begin
		en <= 1'b0; sel <= 1'b0; #(TESTBENCH_DELAY);
		en <= 1'b0; sel <= 1'b1; #(TESTBENCH_DELAY);
		en <= 1'b1; sel <= 1'b0; #(TESTBENCH_DELAY);
		en <= 1'b1; sel <= 1'b1; #(TESTBENCH_DELAY);
		$stop;
	end 
endmodule /* decoder1_2_testbench */