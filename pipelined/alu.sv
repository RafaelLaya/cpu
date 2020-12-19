/** An Arithmetic Logic Unit (ALU)
 *
 * Inputs:
 *		A: First operand
 *      B: Second operand

 *      cntrl:			Operation						Notes:
 *      000:			result = B						value of overflow and carry_out unimportant
 *      010:			result = A + B
 *      011:			result = A - B
 *      100:			result = bitwise A & B		    value of overflow and carry_out unimportant
 *      101:			result = bitwise A | B		    value of overflow and carry_out unimportant
 *      110:			result = bitwise A XOR B	    value of overflow and carry_out unimportant
 *      
 *	Outputs:
 *		result: Result of the ALU operation
 *      negative: whether the result output is negative if interpreted as 2's comp. 
 *      zero: whether the result output was a 64-bit zero.
 *      overflow: on an add or subtract, whether the computation overflowed if the inputs are interpreted as 2's comp.
 *      carry_out: on an add or subtract, whether the computation produced a carry-out.
 */
`include "delays.sv"
module alu(A, B, cntrl, result, negative, zero, overflow, carry_out);
    input logic [63:0] A;
    input logic [63:0] B;
    input logic [2:0] cntrl;
    output logic [63:0] result;
    output logic negative, zero, overflow, carry_out;

    // there is one more carry than there are slices
    logic [64:0] carry_chain;
    assign carry_chain[0] = cntrl[0];
	genvar i;
	generate
		for (i=0; i < 64; i++) begin : each_slice
            ALU_bit_slice slice_i (.carry_out(carry_chain[i+1]), .ALU_bit(result[i]), .A(A[i]), .B(B[i]), .carry_in(carry_chain[i]), .op_select(cntrl));
		end
	endgenerate 

    assign carry_out = carry_chain[64];
    assign negative = result[63];

    xor #(GATE_DELAY) (overflow, carry_chain[64], carry_chain[63]);

    zero_checker zero_flag_machine (.out(zero), .in(result));

endmodule /* alu */

