`default_nettype none

module uart_dynamixel (
    input wire clk_i,
    input wire rst_ni,

    // Wishbone B4
    input wire cyc_i,
    input wire stb_i,
    input wire we_i,
    input wire [7:2] adr_i,
    input wire [31:0] dat_i,
    output logic [31:0] dat_o,
    output logic ack_o,
    input wire [3:0] sel_i,

    // Physical UART
    output logic uart_tx_o,
    input wire uart_rx_i,
    output logic tx_en_o
);

    // Parameters
    localparam integer DEFAULT_BAUD_DIV = 868;
    localparam integer TIMEOUT_LIMIT = 200000;

    // Inline CRC-16/IBM function (poly 0xA001, bit-serial, LSB-first)
    // Identical algorithm to crc16_ibm.sv wire chain.
    function automatic logic [15:0] crc16_update(
        input logic [15:0] crc,
        input logic [7:0] data
    );
        logic [15:0] c;
        logic [7:0] d;
        begin
            c = crc;
            d = data;
            for (int i = 0; i < 8; i++) begin
                if ((c[0] ^ d[0]) == 1'b1)
                    c = (c >> 1) ^ 16'hA001;
                else
                    c = c >> 1;
                d = d >> 1;
            end
            crc16_update = c;
        end
    endfunction

    // Wishbone registers
    logic [9:0] baud_div;
    logic [7:0] tx_id;

    logic rx_done_r;
    logic crc_err_r;
    logic timeout_err_r;
    logic [31:0] rx_position_r;
    logic [7:0] rx_error_r;

    // Wishbone decode (combinational)
    logic wb_access;
    logic wb_wr_ctrl;
    logic wb_wr_baud;
    logic wb_wr_id;
    logic send_cmd;
    logic soft_rst;

    always_comb begin
        wb_access = cyc_i & stb_i & ~ack_o;
        wb_wr_ctrl = wb_access & we_i & (adr_i[4:2] == 3'd0);
        wb_wr_baud = wb_access & we_i & (adr_i[4:2] == 3'd2);
        wb_wr_id = wb_access & we_i & (adr_i[4:2] == 3'd3);
        send_cmd = wb_wr_ctrl & dat_i[0];
        soft_rst = wb_wr_ctrl & dat_i[1];
    end

    // Wishbone read mux — NOT gated by ~ack_o (Verilator/cocotb VPI timing)
    always_comb begin
        dat_o = 32'h0;
        if (cyc_i & stb_i & ~we_i) begin
            unique case (adr_i[4:2])
                3'd1: dat_o = {27'h0, timeout_err_r, crc_err_r,
                               rx_done_r, rx_busy_o, tx_busy_o};
                3'd4: dat_o = rx_position_r;
                3'd5: dat_o = {24'h0, rx_error_r};
                default: dat_o = 32'h0;
            endcase
        end
    end

    // TX baud generator (free-running while TX is active)
    logic [9:0] baud_cnt;
    logic baud_tick;
    logic baud_en;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            baud_cnt <= '0;
        end else if (baud_en) begin
            if (baud_cnt == baud_div - 10'd1)
                baud_cnt <= '0;
            else
                baud_cnt <= baud_cnt + 10'd1;
        end else begin
            baud_cnt <= '0;
        end
    end

    assign baud_tick = baud_en & (baud_cnt == baud_div - 10'd1);

    // RX baud generator (separate — resets on start-bit detection)
    // Gives accurate center-sampling independent of TX phase.
    logic [9:0] rx_baud_cnt;
    logic rx_baud_tick;
    logic rx_baud_half_tick;
    logic rx_baud_en;
    logic rx_baud_rst;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rx_baud_cnt <= '0;
        end else if (rx_baud_rst) begin
            rx_baud_cnt <= '0;
        end else if (rx_baud_en) begin
            if (rx_baud_cnt == baud_div - 10'd1)
                rx_baud_cnt <= '0;
            else
                rx_baud_cnt <= rx_baud_cnt + 10'd1;
        end
    end

    assign rx_baud_tick = rx_baud_en & (rx_baud_cnt == baud_div - 10'd1);
    assign rx_baud_half_tick = rx_baud_en & (rx_baud_cnt == (baud_div >> 1) - 10'd1);

    // TX byte FSM
    // States: TX_IDLE → TX_START → TX_DATA (8 bits) → TX_STOP → TX_DONE
    typedef enum logic [2:0] {
        TX_IDLE = 3'd0,
        TX_START = 3'd1,
        TX_DATA = 3'd2,
        TX_STOP = 3'd3,
        TX_DONE = 3'd4
    } txbyte_state_t;
    txbyte_state_t txb_state;

    logic [7:0] txb_shreg;
    logic [2:0] txb_bitcnt;
    logic txb_load;
    logic [7:0] txb_byte;
    logic txb_done;
    logic txb_busy;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            txb_state <= TX_IDLE;
            txb_shreg <= 8'hFF;
            txb_bitcnt <= '0;
            uart_tx_o <= 1'b1;
            txb_done <= 1'b0;
        end else begin
            txb_done <= 1'b0;

            unique case (txb_state)
                TX_IDLE: begin
                    uart_tx_o <= 1'b1;
                    if (txb_load) begin
                        txb_shreg <= txb_byte;
                        txb_bitcnt <= '0;
                        txb_state <= TX_START;
                    end
                end

                TX_START: begin
                    uart_tx_o <= 1'b0;
                    if (baud_tick)
                        txb_state <= TX_DATA;
                end

                TX_DATA: begin
                    uart_tx_o <= txb_shreg[0];
                    if (baud_tick) begin
                        txb_shreg <= {1'b1, txb_shreg[7:1]};
                        if (txb_bitcnt == 3'd7)
                            txb_state <= TX_STOP;
                        else
                            txb_bitcnt <= txb_bitcnt + 3'd1;
                    end
                end

                TX_STOP: begin
                    uart_tx_o <= 1'b1;
                    if (baud_tick) begin
                        txb_done <= 1'b1;
                        txb_state <= TX_IDLE;
                    end
                end

                default: txb_state <= TX_IDLE;
            endcase
        end
    end

    assign txb_busy = (txb_state != TX_IDLE);

    // TX packet sequencer
    // Builds and sends 14-byte Dynamixel READ_POSITION command.
    // Packet: FF FF FD 00 [ID] 07 00 02 92 00 04 00 [CRL] [CRH]
    // CRC covers bytes [4..11] (ID .. last param).

    typedef enum logic [2:0] {
        PKT_IDLE = 3'd0,
        PKT_BUILD = 3'd1,
        PKT_TX = 3'd2,
        PKT_TURNAROUND = 3'd3,
        PKT_RX_WAIT = 3'd4
    } pkt_state_t;
    pkt_state_t pkt_state;

    logic [7:0] pkt_buf [0:13];
    logic [3:0] pkt_idx;
    logic tx_busy_o;
    logic rx_busy_o;

    // Timeout watchdog (20-bit)
    logic [19:0] timeout_cnt;
    logic timeout_en;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pkt_state <= PKT_IDLE;
            pkt_idx <= '0;
            txb_load <= 1'b0;
            txb_byte <= '0;
            tx_en_o <= 1'b0;
            baud_div <= DEFAULT_BAUD_DIV[9:0];
            tx_id <= 8'd1;
            rx_done_r <= 1'b0;
            crc_err_r <= 1'b0;
            timeout_err_r <= 1'b0;
            rx_position_r <= '0;
            rx_error_r <= '0;
            timeout_cnt <= '0;
            for (int i = 0; i < 14; i++) pkt_buf[i] <= '0;
        end else begin
            // Wishbone register writes
            if (wb_wr_baud) baud_div <= dat_i[9:0];
            if (wb_wr_id) tx_id <= dat_i[7:0];

            if (soft_rst) begin
                pkt_state <= PKT_IDLE;
                pkt_idx <= '0;
                txb_load <= 1'b0;
                tx_en_o <= 1'b0;
                rx_done_r <= 1'b0;
                crc_err_r <= 1'b0;
                timeout_err_r <= 1'b0;
                timeout_cnt <= '0;
            end else begin
                txb_load <= 1'b0;

                unique case (pkt_state)

                    PKT_IDLE: begin
                        tx_en_o <= 1'b0;
                        if (send_cmd) begin
                            rx_done_r <= 1'b0;
                            crc_err_r <= 1'b0;
                            timeout_err_r <= 1'b0;
                            pkt_state <= PKT_BUILD;
                        end
                    end

                    PKT_BUILD: begin
                        pkt_buf[0] <= 8'hFF;
                        pkt_buf[1] <= 8'hFF;
                        pkt_buf[2] <= 8'hFD;
                        pkt_buf[3] <= 8'h00;
                        pkt_buf[4] <= tx_id;
                        pkt_buf[5] <= 8'h07;
                        pkt_buf[6] <= 8'h00;
                        pkt_buf[7] <= 8'h02;
                        pkt_buf[8] <= 8'h92;
                        pkt_buf[9] <= 8'h00;
                        pkt_buf[10] <= 8'h04;
                        pkt_buf[11] <= 8'h00;
                        begin
                            logic [15:0] c;
                            c = 16'h0000;
                            c = crc16_update(c, tx_id);
                            c = crc16_update(c, 8'h07);
                            c = crc16_update(c, 8'h00);
                            c = crc16_update(c, 8'h02);
                            c = crc16_update(c, 8'h92);
                            c = crc16_update(c, 8'h00);
                            c = crc16_update(c, 8'h04);
                            c = crc16_update(c, 8'h00);
                            pkt_buf[12] <= c[7:0];
                            pkt_buf[13] <= c[15:8];
                        end
                        pkt_idx <= '0;
                        tx_en_o <= 1'b1;
                        pkt_state <= PKT_TX;
                    end

                    PKT_TX: begin
                        if (!txb_busy && !txb_load) begin
                            if (pkt_idx == 4'd14) begin
                                tx_en_o <= 1'b0;
                                timeout_cnt <= '0;
                                pkt_state <= PKT_TURNAROUND;
                            end else begin
                                txb_byte <= pkt_buf[pkt_idx];
                                txb_load <= 1'b1;
                                pkt_idx <= pkt_idx + 4'd1;
                            end
                        end
                    end

                    PKT_TURNAROUND: begin
                        if (baud_tick) begin
                            timeout_cnt <= '0;
                            pkt_state <= PKT_RX_WAIT;
                        end else begin
                            timeout_cnt <= timeout_cnt + 20'd1;
                        end
                    end

                    PKT_RX_WAIT: begin
                        if (rx_done_r || crc_err_r) begin
                            pkt_state <= PKT_IDLE;
                        end else if (timeout_cnt == TIMEOUT_LIMIT[19:0]) begin
                            timeout_err_r <= 1'b1;
                            pkt_state <= PKT_IDLE;
                        end else begin
                            timeout_cnt <= timeout_cnt + 20'd1;
                        end
                    end

                    default: pkt_state <= PKT_IDLE;
                endcase

                // Latch results from RX parser
                if (parse_done_pulse) begin
                    if (parse_crc_ok) begin
                        rx_done_r <= 1'b1;
                        rx_position_r <= parse_position;
                        rx_error_r <= parse_error;
                    end else begin
                        crc_err_r <= 1'b1;
                    end
                end
            end
        end
    end

    assign tx_busy_o = (pkt_state == PKT_TX) || (pkt_state == PKT_BUILD) ||
                       (pkt_state == PKT_TURNAROUND);
    assign rx_busy_o = (pkt_state == PKT_RX_WAIT);

    // baud counter enabled whenever pkt_state is TX or turnaround or RX_WAIT
    assign baud_en = (pkt_state != PKT_IDLE) && (pkt_state != PKT_BUILD);

    // RX 2-FF synchronizer (metastability protection)
    logic rx_meta, rx_sync;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= uart_rx_i;
            rx_sync <= rx_meta;
        end
    end

    // RX byte FSM
    // States: RX_B_IDLE → RX_B_START → RX_B_DATA (8 bits) → RX_B_STOP
    // Center-samples using baud_half_tick for start, baud_tick for data.
    typedef enum logic [1:0] {
        RX_B_IDLE = 2'd0,
        RX_B_START = 2'd1,
        RX_B_DATA = 2'd2,
        RX_B_STOP = 2'd3
    } rxbyte_state_t;
    rxbyte_state_t rxb_state;

    logic [7:0] rxb_shreg;
    logic [2:0] rxb_bitcnt;
    logic rxb_done;
    logic [7:0] rxb_byte_out;
    logic rx_active;

    assign rx_active = (pkt_state == PKT_RX_WAIT);

    // rx_baud_en / rx_baud_rst wiring: counter runs while not IDLE, resets on start edge
    assign rx_baud_en = (rxb_state != RX_B_IDLE);
    assign rx_baud_rst = rx_active && !rx_sync && (rxb_state == RX_B_IDLE);

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rxb_state <= RX_B_IDLE;
            rxb_shreg <= '0;
            rxb_bitcnt <= '0;
            rxb_done <= 1'b0;
            rxb_byte_out <= '0;
        end else begin
            rxb_done <= 1'b0;

            unique case (rxb_state)
                RX_B_IDLE: begin
                    if (rx_active && !rx_sync) begin
                        // falling edge on RX — start bit detected, rx_baud_cnt reset this cycle
                        rxb_bitcnt <= '0;
                        rxb_state <= RX_B_START;
                    end
                end

                RX_B_START: begin
                    // wait for half-baud to reach center of start bit, then verify
                    if (rx_baud_half_tick) begin
                        if (!rx_sync)
                            rxb_state <= RX_B_DATA;
                        else
                            rxb_state <= RX_B_IDLE;  // glitch, abort
                    end
                end

                RX_B_DATA: begin
                    if (rx_baud_tick) begin
                        rxb_shreg <= {rx_sync, rxb_shreg[7:1]};
                        if (rxb_bitcnt == 3'd7)
                            rxb_state <= RX_B_STOP;
                        else
                            rxb_bitcnt <= rxb_bitcnt + 3'd1;
                    end
                end

                RX_B_STOP: begin
                    if (rx_baud_tick) begin
                        rxb_byte_out <= rxb_shreg;
                        rxb_done <= 1'b1;
                        rxb_state <= RX_B_IDLE;
                    end
                end

                default: rxb_state <= RX_B_IDLE;
            endcase
        end
    end

    // RX packet parser (layered on RX byte FSM)
    // Parses 15-byte STATUS response:
    //   FF FF FD 00 [ID] 08 00 55 [ERR] [P0][P1][P2][P3] [CRC_L][CRC_H]
    // CRC covers bytes [4..12] (ID through P3).
    typedef enum logic [3:0] {
        PARSE_H0 = 4'd0,
        PARSE_H1 = 4'd1,
        PARSE_H2 = 4'd2,
        PARSE_H3 = 4'd3,
        PARSE_ID = 4'd4,
        PARSE_LEN_L = 4'd5,
        PARSE_LEN_H = 4'd6,
        PARSE_INST = 4'd7,
        PARSE_ERR = 4'd8,
        PARSE_P0 = 4'd9,
        PARSE_P1 = 4'd10,
        PARSE_P2 = 4'd11,
        PARSE_P3 = 4'd12,
        PARSE_CRC_L = 4'd13,
        PARSE_CRC_H = 4'd14
    } parse_state_t;
    parse_state_t parse_state;

    logic [15:0] parse_crc_run;
    logic [7:0] parse_crc_l;
    logic [31:0] parse_position;
    logic [7:0] parse_error;
    logic parse_done_pulse;
    logic parse_crc_ok;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            parse_state <= PARSE_H0;
            parse_crc_run <= 16'h0000;
            parse_crc_l <= '0;
            parse_position <= '0;
            parse_error <= '0;
            parse_done_pulse <= 1'b0;
            parse_crc_ok <= 1'b0;
        end else begin
            parse_done_pulse <= 1'b0;

            // Reset parser when pkt_state leaves PKT_RX_WAIT
            if (!rx_active && parse_state != PARSE_H0) begin
                parse_state <= PARSE_H0;
                parse_crc_run <= 16'h0000;
            end

            if (rxb_done && rx_active) begin
                unique case (parse_state)
                    PARSE_H0: if (rxb_byte_out == 8'hFF) parse_state <= PARSE_H1;
                    PARSE_H1: if (rxb_byte_out == 8'hFF) parse_state <= PARSE_H2;
                              else                        parse_state <= PARSE_H0;
                    PARSE_H2: if (rxb_byte_out == 8'hFD) parse_state <= PARSE_H3;
                              else                        parse_state <= PARSE_H0;
                    PARSE_H3: if (rxb_byte_out == 8'h00) begin
                                  parse_crc_run <= 16'h0000;
                                  parse_state <= PARSE_ID;
                              end else                    parse_state <= PARSE_H0;

                    PARSE_ID: begin
                        parse_crc_run <= crc16_update(16'h0000, rxb_byte_out);
                        parse_state <= PARSE_LEN_L;
                    end
                    PARSE_LEN_L: begin
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_LEN_H;
                    end
                    PARSE_LEN_H: begin
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_INST;
                    end
                    PARSE_INST: begin
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_ERR;
                    end
                    PARSE_ERR: begin
                        parse_error <= rxb_byte_out;
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_P0;
                    end
                    PARSE_P0: begin
                        parse_position[7:0] <= rxb_byte_out;
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_P1;
                    end
                    PARSE_P1: begin
                        parse_position[15:8] <= rxb_byte_out;
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_P2;
                    end
                    PARSE_P2: begin
                        parse_position[23:16] <= rxb_byte_out;
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_P3;
                    end
                    PARSE_P3: begin
                        parse_position[31:24] <= rxb_byte_out;
                        parse_crc_run <= crc16_update(parse_crc_run, rxb_byte_out);
                        parse_state <= PARSE_CRC_L;
                    end
                    PARSE_CRC_L: begin
                        parse_crc_l <= rxb_byte_out;
                        parse_state <= PARSE_CRC_H;
                    end
                    PARSE_CRC_H: begin
                        begin
                            logic [15:0] received_crc;
                            received_crc = {rxb_byte_out, parse_crc_l};
                            parse_crc_ok <= (received_crc == parse_crc_run);
                            parse_done_pulse <= 1'b1;
                            parse_state <= PARSE_H0;
                        end
                    end
                    default: parse_state <= PARSE_H0;
                endcase
            end
        end
    end

    // Wishbone ack
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            ack_o <= 1'b0;
        else
            ack_o <= cyc_i & stb_i & ~ack_o;
    end

endmodule

`default_nettype wire
