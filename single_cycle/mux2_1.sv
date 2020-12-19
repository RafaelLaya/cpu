
/** Simple 2:1 multiplexer
 *
 * Inputs:
 *		i0: Selected when sel==0
 *		i1: Selected when sel==1
 *		sel: selects which signal is passed to the output of the multiplexer
 *
 *	Outputs:
 *		out: This is i0 or i1, based on the value of sel
 *
 * Selects one signal out of two input signals based on sel
 */
`include "delays.sv"

module mux2_1(out, i0, i1, sel);
	output logic out;
	input logic i0, i1, sel;
	
	logic nsel, i1_sel, i0_nsel;
	
	not  #(GATE_DELAY) (nsel, sel);
	and #(GATE_DELAY) (i1_sel, i1, sel);
	and #(GATE_DELAY) (i0_nsel, i0, nsel);
	or #(GATE_DELAY) (out, i1_sel, i0_nsel);

endmodule /* mux2_1 */

/* Testbench for mux2_1 */
module mux2_1_testbench();
	logic out, i0, i1, sel;
	
	mux2_1 dut(.*);
	
	initial begin
		// test all patterns
		sel=0; i0=0; i1=0; #TESTBENCH_DELAY;
		sel=0; i0=0; i1=1; #TESTBENCH_DELAY;
		sel=0; i0=1; i1=0; #TESTBENCH_DELAY;
		sel=0; i0=1; i1=1; #TESTBENCH_DELAY;
		sel=1; i0=0; i1=0; #TESTBENCH_DELAY;
		sel=1; i0=0; i1=1; #TESTBENCH_DELAY;
		sel=1; i0=1; i1=0; #TESTBENCH_DELAY;
		sel=1; i0=1; i1=1; #TESTBENCH_DELAY;
	end 

endmodule /* mux2_1_testbench */