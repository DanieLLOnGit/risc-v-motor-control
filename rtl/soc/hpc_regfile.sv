`default_nettype none

module hpc_regfile (
    input wire clk_i,
    input wire rst_ni,

    // Wishbone B4 (from upstream master)
    input wire cyc_i,
    input wire stb_i,
    input wire we_i,
    input wire [7:2] adr_i,
    input wire [31:0] dat_i,
    output logic [31:0] dat_o,
    output logic ack_o,
    input wire [3:0] sel_i,

    // Physical UART pins (pass-through to uart_dynamixel)
    output wire uart_tx_o,
    input wire uart_rx_i,
    output wire tx_en_o
);

    // Address decode — adr_i[7] selects the slave
    // adr_i is [7:2], so bit [7] is index 5 of the vector
    wire sel_uart = adr_i[7];
    wire sel_crc = ~adr_i[7];

    // CRC slave Wishbone signals
    wire [31:0] crc_dat_o;
    wire crc_ack_o;

    crc16_ibm u_crc (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .cyc_i (cyc_i & sel_crc),
        .stb_i (stb_i & sel_crc),
        .we_i (we_i),
        .adr_i (adr_i),
        .dat_i (dat_i),
        .dat_o (crc_dat_o),
        .ack_o (crc_ack_o),
        .sel_i (sel_i)
    );

    // UART slave Wishbone signals
    wire [31:0] uart_dat_o;
    wire uart_ack_o;

    uart_dynamixel u_uart (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .cyc_i (cyc_i & sel_uart),
        .stb_i (stb_i & sel_uart),
        .we_i (we_i),
        .adr_i (adr_i),
        .dat_i (dat_i),
        .dat_o (uart_dat_o),
        .ack_o (uart_ack_o),
        .sel_i (sel_i),
        .uart_tx_o (uart_tx_o),
        .uart_rx_i (uart_rx_i),
        .tx_en_o (tx_en_o)
    );

    // Mux outputs
    assign ack_o = crc_ack_o | uart_ack_o;
    assign dat_o = sel_uart ? uart_dat_o : crc_dat_o;

endmodule

`default_nettype wire
