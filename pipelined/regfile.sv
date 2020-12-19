/** Implements a register file of 32 registers, 64 bits each
 *
 * Inputs:
 *		WriteData: The data to be written through the unique write port
 *		WriteRegister: Unsigned number that represents which register will be written through the unique write port
 *		RegWrite: true to write WriteData onto WriteRegister. Otherwise no write is performed
 *		ReadRegister1: Unsigned number that represents which register is read on read port #1
 *		ReadRegister2: Unsigned number that represents which register is read on read port #2
 *		clk: The clock signal
 *
 *	Outputs:
 *		ReadData1: The read port #1
 *		ReadData2: The read port #2
 *
 * Note: register #31 is always zero, no matter what is written to it
 * Note: When writing and reading from the same register, the new data is sent through the read port
 */
`include "delays.sv"
module regfile(
	ReadData1, 
	ReadData2, 
	WriteData, 
	ReadRegister1, 
	ReadRegister2, 
	WriteRegister, 
	RegWrite, 
	clk
);
	input logic clk;
	
	input logic [4:0] ReadRegister1;
	input logic [4:0] ReadRegister2;
	
	input logic RegWrite;
	input logic [4:0] WriteRegister;
	input logic [63:0] WriteData;
	
	output logic [63:0] ReadData1;
	output logic [63:0] ReadData2;
	
	
	logic [63:0] read_port_data [0:31];
	logic [31:0] enable_bus;
	
	// Connect the registers
	genvar i;
	generate
		for(i=0; i < 31; i++) begin : eachReg
			register #(.WIDTH(64)) reg_i (.dout(read_port_data[i]), .din(WriteData), .write_en(enable_bus[i]), .clk(clk), .reset(1'b0));
		end
	endgenerate
	
	// X31 is always 0
	assign read_port_data[31] = 64'b0;
	
	// Manage writes
	decoder5_32 write_register_selector (.out(enable_bus), .sel(WriteRegister), .en(RegWrite));
	
	// A regular regfile would be completed with the two following multiplexers
	logic [63:0] ReadData1_regular;
	logic [63:0] ReadData2_regular;
	mux32_1_wide #(.WIDTH(64)) read_port_1_regular (.out(ReadData1_regular), .in(read_port_data), .sel(ReadRegister1));
	mux32_1_wide #(.WIDTH(64)) read_port_2_regular (.out(ReadData2_regular), .in(read_port_data), .sel(ReadRegister2));

	// This regfile is special: When writing and reading from the same register, the write data is passed to the respective read port
	
	// Check if there is a read/write match
	logic ReadRegister1_equals_WriteRegister;
	logic ReadRegister2_equals_WriteRegister;
	equality_checker WriteRegister_ReadRegister1_equality_checker (
		.equal(ReadRegister1_equals_WriteRegister), 
		.A(ReadRegister1), 
		.B(WriteRegister)
	);
	equality_checker WriteRegister_ReadRegister2_equality_checker (
		.equal(ReadRegister2_equals_WriteRegister), 
		.A(ReadRegister2), 
		.B(WriteRegister)
	);

	// Also check that the write is valid
	logic ReadRegister1_equals_WriteRegister_and_writing;
	logic ReadRegister2_equals_WriteRegister_and_writing;
	and #(GATE_DELAY) (ReadRegister1_equals_WriteRegister_and_writing, ReadRegister1_equals_WriteRegister, RegWrite);
	and #(GATE_DELAY) (ReadRegister2_equals_WriteRegister_and_writing, ReadRegister2_equals_WriteRegister, RegWrite);


	// Finally, check that this is not X31, since X31 is always zero
	logic ReadRegister1_is_x31, ReadRegister1_is_not_x31;
	logic ReadRegister2_is_x31, ReadRegister2_is_not_x31;
	equality_checker x31_checker_read1_port (
		.equal(ReadRegister1_is_x31),
		.A(ReadRegister1),
		.B(5'b11111)
	);
	equality_checker x31_checker_read2_port (
		.equal(ReadRegister2_is_x31),
		.A(ReadRegister2),
		.B(5'b11111)
	);
	not #(GATE_DELAY) (ReadRegister1_is_not_x31, ReadRegister1_is_x31);
	not #(GATE_DELAY) (ReadRegister2_is_not_x31, ReadRegister2_is_x31);

	logic read1_pass_write;
	logic read2_pass_write;
	and #(GATE_DELAY) (read1_pass_write, ReadRegister1_equals_WriteRegister_and_writing, ReadRegister1_is_not_x31);
	and #(GATE_DELAY) (read2_pass_write, ReadRegister2_equals_WriteRegister_and_writing, ReadRegister2_is_not_x31);

	// Send the output
	mux2_1_wide #(.WIDTH(64)) read_port_1 (
		.out(ReadData1),
		.in({ReadData1_regular, WriteData}),
		.sel(read1_pass_write)
	);
	mux2_1_wide #(.WIDTH(64)) read_port_2 (
		.out(ReadData2),
		.in({ReadData2_regular, WriteData}),
		.sel(read2_pass_write)
	);

endmodule /* regfile */
