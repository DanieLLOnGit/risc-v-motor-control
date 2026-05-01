`default_nettype none

module soc (
    input wire clk_i,
    input wire rst_ni,

    // Physical UART (pass-through from hpc_regfile → uart_dynamixel)
    output wire uart_tx_o,
    input wire uart_rx_i,
    output wire tx_en_o
);

    // PicoRV32 interface wires
    wire mem_valid;
    wire mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0] mem_wstrb;
    wire [31:0] mem_rdata;

    // PicoRV32 CPU instantiation
    wire cpu_trap;
    wire cpu_mem_instr;
    wire cpu_mem_la_read;
    wire cpu_mem_la_write;
    wire [31:0] cpu_mem_la_addr;
    wire [31:0] cpu_mem_la_wdata;
    wire [3:0] cpu_mem_la_wstrb;
    wire cpu_pcpi_valid;
    wire [31:0] cpu_pcpi_insn;
    wire [31:0] cpu_pcpi_rs1;
    wire [31:0] cpu_pcpi_rs2;
    wire [31:0] cpu_eoi;
    wire cpu_trace_valid;
    wire [35:0] cpu_trace_data;

    picorv32 #(
        .ENABLE_MUL(0),
        .ENABLE_DIV(0),
        .BARREL_SHIFTER(1),
        .COMPRESSED_ISA(0),
        .ENABLE_IRQ(0),
        .ENABLE_IRQ_QREGS(0),
        .ENABLE_TRACE(0)
    ) u_cpu (
        .clk (clk_i),
        .resetn (rst_ni),
        // Memory interface
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),
        .mem_addr (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata),
        .mem_instr (cpu_mem_instr),
        // Look-ahead interface (unused)
        .mem_la_read (cpu_mem_la_read),
        .mem_la_write (cpu_mem_la_write),
        .mem_la_addr (cpu_mem_la_addr),
        .mem_la_wdata (cpu_mem_la_wdata),
        .mem_la_wstrb (cpu_mem_la_wstrb),
        // Co-processor interface (unused — tied off)
        .pcpi_valid (cpu_pcpi_valid),
        .pcpi_insn (cpu_pcpi_insn),
        .pcpi_rs1 (cpu_pcpi_rs1),
        .pcpi_rs2 (cpu_pcpi_rs2),
        .pcpi_wr (1'b0),
        .pcpi_rd (32'h0),
        .pcpi_wait (1'b0),
        .pcpi_ready (1'b0),
        // IRQ interface (unused)
        .irq (32'h0),
        .eoi (cpu_eoi),
        // Trap / trace
        .trap (cpu_trap),
        .trace_valid (cpu_trace_valid),
        .trace_data (cpu_trace_data)
    );

    // Address decode — mem_addr[17:16]
    wire sel_imem = mem_valid & (mem_addr[17:16] == 2'b00);
    wire sel_dmem = mem_valid & (mem_addr[17:16] == 2'b01);
    wire sel_dsp = mem_valid & (mem_addr[17:16] == 2'b10);
    wire sel_hpc = mem_valid & (mem_addr[17:16] == 2'b11);

    // IMEM — 4 KB instruction memory
    // Loaded at elaboration from firmware.hex (relative to sim directory).
    // Read: combinational. Write: disabled (firmware is read-only).
    logic [31:0] imem [0:1023];

    // synthesis translate_off
    initial $readmemh("../../firmware/firmware.hex", imem);
    // synthesis translate_on

    wire [31:0] imem_rdata = imem[mem_addr[11:2]];

    // One-cycle registered ack (same pattern as all other slaves)
    logic imem_ack;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) imem_ack <= 1'b0;
        else imem_ack <= sel_imem & ~imem_ack;
    end

    // DMEM — 4 KB data/stack memory
    // Full read/write with per-byte enables.
    logic [31:0] dmem [0:1023];

    logic [31:0] dmem_rdata;
    logic dmem_ack;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            dmem_ack <= 1'b0;
            dmem_rdata <= '0;
        end else begin
            dmem_ack <= sel_dmem & ~dmem_ack;
            if (sel_dmem & ~dmem_ack) begin
                if (mem_wstrb[0]) dmem[mem_addr[11:2]][7:0] <= mem_wdata[7:0];
                if (mem_wstrb[1]) dmem[mem_addr[11:2]][15:8] <= mem_wdata[15:8];
                if (mem_wstrb[2]) dmem[mem_addr[11:2]][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) dmem[mem_addr[11:2]][31:24] <= mem_wdata[31:24];
                dmem_rdata <= dmem[mem_addr[11:2]];
            end
        end
    end

    // Wishbone bridge signals (shared by both accelerator slaves)
    wire wb_we = |mem_wstrb;
    wire [7:2] wb_adr = mem_addr[7:2];
    wire [31:0] wb_dat_w = mem_wdata;
    wire [3:0] wb_sel = mem_wstrb;

    // DSP engine — fir_filter
    wire [31:0] dsp_dat_o;
    wire dsp_ack_o;

    // Unused fir_filter streaming output ports
    wire fir_s_ready_o;
    wire fir_m_valid_o;
    wire [15:0] fir_m_data_o;

    fir_filter u_fir (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .cyc_i (sel_dsp),
        .stb_i (sel_dsp),
        .we_i (wb_we),
        .adr_i (wb_adr),
        .dat_i (wb_dat_w),
        .dat_o (dsp_dat_o),
        .ack_o (dsp_ack_o),
        .sel_i (wb_sel),
        // Streaming ports tied off — CPU drives filter via Wishbone only
        .s_valid_i (1'b0),
        .s_data_i (16'h0),
        .s_ready_o (fir_s_ready_o),
        .m_ready_i (1'b1),
        .m_valid_o (fir_m_valid_o),
        .m_data_o (fir_m_data_o)
    );

    // HPC engine — hpc_regfile (CRC + UART)
    wire [31:0] hpc_dat_o;
    wire hpc_ack_o;

    hpc_regfile u_hpc (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .cyc_i (sel_hpc),
        .stb_i (sel_hpc),
        .we_i (wb_we),
        .adr_i (wb_adr),
        .dat_i (wb_dat_w),
        .dat_o (hpc_dat_o),
        .ack_o (hpc_ack_o),
        .sel_i (wb_sel),
        .uart_tx_o (uart_tx_o),
        .uart_rx_i (uart_rx_i),
        .tx_en_o (tx_en_o)
    );

    // Read data and ready mux
    assign mem_ready = imem_ack | dmem_ack | dsp_ack_o | hpc_ack_o;

    assign mem_rdata = sel_imem ? imem_rdata :
                       sel_dmem ? dmem_rdata :
                       sel_dsp  ? dsp_dat_o  :
                       sel_hpc  ? hpc_dat_o  :
                                  32'hDEAD_BEEF;

endmodule

`default_nettype wire
