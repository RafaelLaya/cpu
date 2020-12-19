/** Simple positive edge-triggered D Flip Flop (D_FF) module with an enable signal
 *
 * Inputs:
 *		d: Input of the D_FF
 *		reset: Active-high synchronous reset signal
 *		en: active-high enable signal
 *		clk: The clock signal
 *
 *	Outputs:
 *		q: The output of the DFF
 *
 * When enabled, behaves like a regular DFF and d is passed to q on positive edges of clk
 * When not enabled, the output q is held
 */
`include "delays.sv"
module D_EN_FF(q, d, en, reset, clk);
	input logic d, en, reset, clk;
	output logic q;
	
	// Select the next output based on the enable signal
	logic d_selected;
	mux2_1 d_selector(.out(d_selected), .i0(q), .i1(d), .sel(en));
	
	D_FF flip_flop(.q(q), .d(d_selected), .reset(reset), .clk(clk)); 

endmodule /* D_EN_FF */

/* testbench for D_EN_FF */
module D_EN_FF_testbench();
	logic d, en, reset, clk, q;
	
	D_EN_FF dut(.*);
	// simulate clock
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
		d <= 1'b0;
		en <= 1'b0;
		reset <= 1'b1;
		@(posedge clk);
		reset <= 1'b0;
		
		// hold 0 
		repeat (6) @(posedge clk);
		
		// keep holding 0 
		d <= 1'b1;
		repeat (6) @(posedge clk);
		
		// keep holding 0 
		d <= 1'b0;
		en <= 1'b1;
		repeat (6) @(posedge clk);
		
		// now take 1 
		d <= 1'b1;
		en <= 1'b1;
		repeat (6) @(posedge clk);
		
		// hold 1
		en <= 1'b0;
		d <= 1'b1;
		repeat (6) @(posedge clk);
		
		// keep holding 1 
		d <= 1'b0;
		repeat (6) @(posedge clk);
		
		// quick oscillations (not enabled -- hold state)
		en <= 1'b0;
		repeat (6) begin
			d <= ~d;
			@(posedge clk);
		end 
		
		// quick oscillations (enabled -- act like a D_FF)
		en <= 1'b1;
		repeat (6) begin
			d <= ~d;
			@(posedge clk);
		end 
		
		@(posedge clk);
		$stop;
	end 
endmodule /* D_EN_FF_testbench */