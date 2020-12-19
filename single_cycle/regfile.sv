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
	
	genvar i;
	generate
		for(i=0; i < 31; i++) begin : eachReg
			register #(.WIDTH(64)) reg_i (.dout(read_port_data[i]), .din(WriteData), .write_en(enable_bus[i]), .clk(clk), .reset(1'b0));
		end
	endgenerate
	
	assign read_port_data[31] = 64'b0;
	
	decoder5_32 write_register_selector (.out(enable_bus), .sel(WriteRegister), .en(RegWrite));
	
	mux32_1_wide #(.WIDTH(64)) read_port_1 (.out(ReadData1), .in(read_port_data), .sel(ReadRegister1));
	mux32_1_wide #(.WIDTH(64)) read_port_2 (.out(ReadData2), .in(read_port_data), .sel(ReadRegister2));

endmodule /* regfile */
