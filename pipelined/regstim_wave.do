onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /regstim/i
add wave -noupdate -radix unsigned /regstim/WriteRegister
add wave -noupdate -radix hexadecimal /regstim/WriteData
add wave -noupdate /regstim/RegWrite
add wave -noupdate -radix unsigned /regstim/ReadRegister1
add wave -noupdate -radix hexadecimal /regstim/ReadData1
add wave -noupdate -radix unsigned /regstim/ReadRegister2
add wave -noupdate -radix hexadecimal /regstim/ReadData2
add wave -noupdate /regstim/clk
add wave -noupdate /regstim/ClockDelay
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {36529 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 139
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {23743880 ps}
