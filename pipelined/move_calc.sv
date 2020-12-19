

/** Puts a 16-bit constant in a 64-bit value, either by (z)ero-ing the rest of the bits, 
 *  or by (k)eeping them. The position of the constant must be 16-bit aligned within the
 *  64-bit result. 
 *
 * Inputs:
 *		Data: The 64-bit value
 *		shift_sel: Selects the position of the 16-bit constant
 *		WithKeep: If true, we keep the rest of the bits. Otherwise they are zero-ed
 *		imm16: The 16-bit constant
 *
 *	Outputs:
 *		out: The result of introducing imm16 into Data
 */
`include "delays.sv"
module move_calc(out, Data, shift_sel, WithKeep, imm16);
	input logic [1:0] shift_sel;
	input logic WithKeep;
	input logic [15:0] imm16;
	input logic [63:0] Data; 
	output logic [63:0] out; 

	logic [63:0] out_WithKeep;
	logic [63:0] out_NoKeep; 

	// We will select one of these based on shift_sel
	logic [63:0] out_NoKeep_possibilities [0:3];
	
	// Each of these simply connects the 16 wires of imm16 in the correct place
	// And all other wires are just zeros 
	assign out_NoKeep_possibilities[0][63:0] = {{48 {1'b0}}, imm16[15:0]};
	assign out_NoKeep_possibilities[1][63:0] = {{32 {1'b0}}, imm16[15:0], {16 {1'b0}}};
	assign out_NoKeep_possibilities[2][63:0] = {{16 {1'b0}}, imm16[15:0], {32 {1'b0}}};
	assign out_NoKeep_possibilities[3][63:0] = {imm16[15:0], {48 {1'b0}}};

	// Select one of the combinations above
	mux4_1_wide #(.WIDTH(64)) NoKeep_Selector (.out(out_NoKeep), .in(out_NoKeep_possibilities), .sel(shift_sel));

	// When using (K)eep, use four 2:1 16-bit multiplexers and put conditions on shift_sel
	// Each mux corresponds to 16 bits in the output
	// And shift_sel chooses whether those 16 bits come from the immediate or the input 
	logic nsel0, nsel1;
	not #(GATE_DELAY) (nsel0, shift_sel[0]);
	not #(GATE_DELAY) (nsel1, shift_sel[1]);

	logic nsel1_nsel0;
	and #(GATE_DELAY) (nsel1_nsel0, nsel1, nsel0);

	logic nsel1_sel0;
	and #(GATE_DELAY) (nsel1_sel0, nsel1, shift_sel[0]);

	logic sel1_nsel0;
	and #(GATE_DELAY) (sel1_nsel0, shift_sel[1], nsel0);

	logic sel1_sel0;
	and #(GATE_DELAY) (sel1_sel0, shift_sel[1], shift_sel[0]);

	mux2_1_wide #(.WIDTH(16)) m0 (.out(out_WithKeep[15:0]),  .in({Data[15:0],  imm16}), .sel(nsel1_nsel0));
	mux2_1_wide #(.WIDTH(16)) m1 (.out(out_WithKeep[31:16]), .in({Data[31:16], imm16}), .sel(nsel1_sel0));
	mux2_1_wide #(.WIDTH(16)) m2 (.out(out_WithKeep[47:32]), .in({Data[47:32], imm16}), .sel(sel1_nsel0));
	mux2_1_wide #(.WIDTH(16)) m3 (.out(out_WithKeep[63:48]), .in({Data[63:48], imm16}), .sel(sel1_sel0));

	// Finally, select either (k)eep or (z)ero
	mux2_1_wide #(.WIDTH(64)) m4 (.out(out), .in({out_NoKeep, out_WithKeep}), .sel(WithKeep));

endmodule /* move_calc */

/* testbench for move_calc */
module move_calc_testbench();
	parameter NUMBER_OF_TESTS = 2048; 
	logic [1:0] shift_sel;
	logic WithKeep;
	logic [15:0] imm16;
	logic [63:0] Data; 
	logic [63:0] out; 

	move_calc dut (.*);

	initial begin
		$display("Running tests without Keep");
		WithKeep = 1'b0; 
		for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin 
			Data = {$urandom(), $urandom()};
			imm16 = $urandom();
			for (integer j = 0; j < 2 ** 2; j++) begin 
				shift_sel = j; 
				#(TESTBENCH_DELAY);
				assert(out == imm16 << (16 * shift_sel));
			end
		end 
		
		$display("Running tests with Keep");
		WithKeep = 1'b1;
		for (integer i = 0; i < NUMBER_OF_TESTS; i++) begin 
			Data = {$urandom(), $urandom()};
			imm16 = $urandom();
			for (integer j = 0; j < 2 ** 2; j++) begin 
				shift_sel = j; 
				#(TESTBENCH_DELAY);
				
				if (shift_sel == 0) 
					assert(out == {Data[63:16], imm16[15:0]}); 
				else if (shift_sel == 1) 
					assert(out == {Data[63:32], imm16[15:0], Data[15:0]}); 
				else if (shift_sel == 2) 
					assert(out == {Data[63:48], imm16[15:0], Data[31:0]});
				else 
					assert(out == {imm16[15:0], Data[47:0]});
			end
		end 
		$stop;
	end
endmodule /* move_calc */