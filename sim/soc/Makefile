SIM              = verilator
TOPLEVEL_LANG    = verilog
VERILOG_SOURCES  = ../../rtl/cpu/picorv32.v \
                   ../../rtl/hpc/crc16_ibm.sv \
                   ../../rtl/hpc/uart_dynamixel.sv \
                   ../../rtl/dsp/fir_filter.sv \
                   ../../rtl/soc/hpc_regfile.sv \
                   ../../rtl/soc/soc.sv
TOPLEVEL         = soc
MODULE           = test_soc
COMPILE_ARGS    += --timing --sv --top-module soc
EXTRA_ARGS      += --trace-fst
SIM_BUILD        = /tmp/robo-v-sim/soc

include $(shell cocotb-config --makefiles)/Makefile.sim
