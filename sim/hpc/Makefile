SIM              = verilator
TOPLEVEL_LANG    = verilog
VERILOG_SOURCES  = ../../rtl/hpc/crc16_ibm.sv
TOPLEVEL         = crc16_ibm
MODULE           = test_crc
COMPILE_ARGS    += --timing --sv
EXTRA_ARGS      += --trace-fst
SIM_BUILD        = /tmp/robo-v-sim/hpc

include $(shell cocotb-config --makefiles)/Makefile.sim
