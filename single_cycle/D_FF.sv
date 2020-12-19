/** Simple positive edge-triggered D Flip Flop (D_FF) module
 *
 * Inputs:
 *		d: Input of the D_FF
 *		reset: Active-high synchronous reset signal
 *		clk: The clock signal
 *
 *	Outputs:
 *		q: Output of the D_FF, d is passed to q on positive edges of clk
 *
 * Provided by the EE469 course staff as our state-holding primitive element
 */
`include "delays.sv"
module D_FF(q, d, reset, clk);
	output reg q;
	input d, reset, clk;
	
	always_ff @(posedge clk)
		if (reset)
			q <= 0; // On reset, set to 0
		else
			q <= d; // Otherwise out = d
endmodule /* D_FF */

/* Testbench for D_FF */
module D_FF_testbench();
	logic q, d, reset, clk;
	
	D_FF dut(.*);
	
	// simulate the cock
	parameter CLOCK_PERIOD = TESTBENCH_DELAY * 2;
	initial begin
		clk <= 1'b0;
		forever begin
			#(CLOCK_PERIOD / 2);
			clk <= ~clk;
		end 
	end 
	
	// provide inputs
	initial begin
		reset <= 1'b1;
		d <= 1'b0;
		@(posedge clk);
		reset <= 1'b0;
		
		// hold zero
		d <= 1'b0;
		repeat (5) @(posedge clk);
		
		// hold one
		d <= 1'b1;
		repeat (5) @(posedge clk);
		
		// quick oscillations
		repeat (6) begin
			d <= ~d;
			@(posedge clk);
		end 
		
		@(posedge clk);
		$stop;
	end 
endmodule /* D_FF_testbench */