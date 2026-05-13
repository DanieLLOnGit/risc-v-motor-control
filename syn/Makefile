YOSYS    ?= yosys
PDK_ROOT ?= $(HOME)/tools/open_pdks/sky130

SKY130_LIB ?= $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib

.PHONY: all clean

all: results/soc_netlist.v

results/:
	mkdir -p results

results/soc_netlist.v: synth.tcl constraints.sdc | results/
	SKY130_LIB=$(SKY130_LIB) $(YOSYS) -l results/synth.log -s synth.tcl
	@echo ""
	@echo "=== Synthesis complete ==="
	@echo "  Netlist : results/soc_netlist.v"
	@echo "  Log     : results/synth.log"
	@grep -A 30 "=== design hierarchy ===" results/synth.log | head -40 || true
	@grep "Chip area" results/synth.log || true

clean:
	rm -rf results/
