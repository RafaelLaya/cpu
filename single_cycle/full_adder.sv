
/** A simple full adder module
 *
 * Inputs:
 *		carry_in: The carry input bit
 *      A: The first operand
 *      B: The second operand
 *
 *	Outputs:
 *		carry_out: The output carry bit (i.e.: (A+B)[1])
 *      S: The sum bit (i.e.: (A+B)[0])
 */
`include "delays.sv"
module full_adder(carry_out, S, carry_in, A, B);
    input logic carry_in, A, B;
    output logic carry_out, S;

    // The following block produces carry_out = AB + Acarry_in + Bcarry_in
    logic AB, Bcarry_in, Acarry_in;
    logic carry_intermediate;
    and #(GATE_DELAY) (AB, A, B);
    and #(GATE_DELAY) (Bcarry_in, B, carry_in);
    and #(GATE_DELAY) (Acarry_in, A, carry_in);
    or #(GATE_DELAY) (carry_intermediate, AB, Bcarry_in);
    or #(GATE_DELAY) (carry_out, carry_intermediate, Acarry_in);

    // The following block produces S = A xor B xor carry_in
    // Note: is_odd(A, B, C) = is_odd(is_odd(A, B), C))
    logic S_intermediate;
    xor #(GATE_DELAY) (S_intermediate, A, B);
    xor #(GATE_DELAY) (S, S_intermediate, carry_in);

endmodule /* full_adder */

/* testbench for full_adder */
module full_adder_testbench();
    logic carry_out, S, carry_in, A, B;

    full_adder dut (.*);

    // provide inputs
    initial begin
        for (integer i = 0; i < 2 ** 3; i++) begin
            {A, B, carry_in} <= i;
            #(TESTBENCH_DELAY);
            assert(2'(A) + 2'(B) + 2'(carry_in) == {carry_out, S}) 
            else $error("Expected {carry_out, S}=%0x but got %0x at %0t", 2'(A) + 2'(B) + 2'(carry_in), {carry_out, S}, $time);
        end
        #(TESTBENCH_DELAY);
        $stop;
    end

endmodule /* full_adder_testbench */