/** Implements a parametrized register of size WIDTH bits
 *
 * Inputs:
 *		din: The input data to the register 
 *		write_en: Active-high signal that indicates whether din is to be written into the register
 *		clk: The clock signal
 *
 *	Outputs:
 *		dout: The value held in the register
 */
`include "delays.sv"
module register #(parameter WIDTH=64) (dout, din, write_en, reset, clk);
	input logic write_en, reset, clk;
	input logic [WIDTH-1:0] din;
	output logic [WIDTH-1:0] dout;
	
	genvar i;
	generate
		for (i=0; i < WIDTH; i++) begin : each_D_EN_FF
			D_EN_FF d_en_ff_i (.q(dout[i]), .d(din[i]), .en(write_en), .reset(reset), .clk(clk));
		end
	endgenerate 
endmodule /* reg_64 */

/* testbench for register */
module register_testbench();
	parameter WIDTH=64;
	logic write_en, reset, clk;
	logic [WIDTH-1:0] din;
	logic [WIDTH-1:0] dout;
	
	register #(.WIDTH(WIDTH)) dut (.*);
	
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
		// clear the register
		reset <= 1'b1;
		write_en <= 1'b0;
		@(posedge clk);
		reset <= 1'b0;
		
		// go through all powers of two
		din <= 1;
		write_en <= 1'b1;
		@(posedge clk);
		for (integer i=0; i < WIDTH; i++) begin
			din <= din << 1;
			@(posedge clk);
		end 
		
		// hold state
		write_en <= 1'b0;
		repeat (10) @(posedge clk);
		
		// count
		write_en <= 1'b1;
		for (integer i = 0; i < 10; i++) begin
			din <= din + 1;
			@(posedge clk);
		end 
		
		@(posedge clk);
		$stop;
	end
endmodule /* register_testbench */