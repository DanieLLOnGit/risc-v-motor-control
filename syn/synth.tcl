# Read all RTL sources
read_verilog -sv ../rtl/dsp/fir_filter.sv
read_verilog -sv ../rtl/hpc/crc16_ibm.sv
read_verilog -sv ../rtl/hpc/uart_dynamixel.sv
read_verilog -sv ../rtl/soc/hpc_regfile.sv
read_verilog -sv ../rtl/soc/soc.sv
read_verilog    ../rtl/cpu/picorv32.v

# Elaborate top module
hierarchy -check -top soc

# Synthesize
synth -top soc

# Map to generic cells and report area
# When SKY130_LIB is set, also map to Sky130 HD standard cells
stat

tee -o results/area.rpt stat

# Write gate-level netlist
write_verilog -noattr results/soc_netlist.v
