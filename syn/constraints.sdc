# Primary clock: 50 MHz (20 ns period) on clk_i
create_clock -name clk_i -period 20.000 [get_ports clk_i]

# Input delays — assume 2 ns propagation from pad to first flop
set_input_delay -clock clk_i 2.0 [get_ports rst_ni]
set_input_delay -clock clk_i 2.0 [get_ports uart_rx_i]

# Output delays — assume 2 ns from last flop to pad
set_output_delay -clock clk_i 2.0 [get_ports uart_tx_o]
set_output_delay -clock clk_i 2.0 [get_ports tx_en_o]
