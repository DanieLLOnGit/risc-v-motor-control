SIM              = verilator
TOPLEVEL_LANG    = verilog
VERILOG_SOURCES  = ../../rtl/dsp/fir_filter.sv
TOPLEVEL         = fir_filter
MODULE           = test_fir
COMPILE_ARGS    += --timing --sv
EXTRA_ARGS      += --trace-fst
SIM_BUILD        = /tmp/robo-v-sim/dsp

include $(shell cocotb-config --makefiles)/Makefile.sim
