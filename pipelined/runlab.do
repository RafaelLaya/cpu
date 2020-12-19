# Create work library
vlib work

# Compile Verilog
#     All Verilog files that are part of this design should have
#     their own "vlog" line below.
vlog "./zero_checker.sv"
vlog "./datamem.sv"
vlog "./mux2_1.sv"
vlog "./mux2_1_wide.sv"
vlog "./mux4_1_wide.sv"
vlog "./mux8_1_wide.sv"
vlog "./mux16_1_wide.sv" 
vlog "./mux32_1_wide.sv" 
vlog "./decoder1_2.sv"
vlog "./decoder2_4.sv"
vlog "./decoder3_8.sv"
vlog "./decoder4_16.sv"
vlog "./decoder5_32.sv"
vlog "./full_adder.sv"
vlog "./adder.sv"
vlog "./instructmem.sv"
vlog "./lsl_2.sv"
vlog "./move_calc.sv"
vlog "./mux2_1.sv"
vlog "./D_FF.sv"
vlog "./D_EN_FF.sv"
vlog "./register.sv"
vlog "./sign_extender.sv"
vlog "./zero_extender.sv"
vlog "./equality_checker.sv"
vlog "./regfile.sv"
vlog "./ALU_bit_slice.sv"
vlog "./alu.sv"
vlog "./datapath.sv"
vlog "./control.sv"
vlog "./cpu.sv"

# Call vsim to invoke simulator
#     Make sure the last item on the line is the name of the
#     testbench module you want to execute.
vsim -voptargs="+acc" -t 1ps -lib work cpu_testbench

# Source the wave do file
#     This should be the file that sets up the signal window for
#     the module you are testing.
do cpu_wave.do

# Set the window types
view wave
view structure
view signals

# Run the simulation
run -all

# End
