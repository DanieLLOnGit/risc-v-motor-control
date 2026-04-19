`default_nettype none

module crc16_ibm (
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
    input wire [3:0] sel_i
);

    // Internal state
    logic [15:0] crc_reg;

    // Combinational: 8-step CRC-16/ARC LFSR unrolled as wire chain
    wire [15:0] crc_s0, crc_s1, crc_s2, crc_s3,
                crc_s4, crc_s5, crc_s6, crc_s7, crc_s8;

    assign crc_s0 = crc_reg;

    assign crc_s1 = (crc_s0[0] ^ dat_i[0]) ? ((crc_s0 >> 1) ^ 16'hA001) : (crc_s0 >> 1);
    assign crc_s2 = (crc_s1[0] ^ dat_i[1]) ? ((crc_s1 >> 1) ^ 16'hA001) : (crc_s1 >> 1);
    assign crc_s3 = (crc_s2[0] ^ dat_i[2]) ? ((crc_s2 >> 1) ^ 16'hA001) : (crc_s2 >> 1);
    assign crc_s4 = (crc_s3[0] ^ dat_i[3]) ? ((crc_s3 >> 1) ^ 16'hA001) : (crc_s3 >> 1);
    assign crc_s5 = (crc_s4[0] ^ dat_i[4]) ? ((crc_s4 >> 1) ^ 16'hA001) : (crc_s4 >> 1);
    assign crc_s6 = (crc_s5[0] ^ dat_i[5]) ? ((crc_s5 >> 1) ^ 16'hA001) : (crc_s5 >> 1);
    assign crc_s7 = (crc_s6[0] ^ dat_i[6]) ? ((crc_s6 >> 1) ^ 16'hA001) : (crc_s6 >> 1);
    assign crc_s8 = (crc_s7[0] ^ dat_i[7]) ? ((crc_s7 >> 1) ^ 16'hA001) : (crc_s7 >> 1);

    wire [15:0] crc_next;
    assign crc_next = crc_s8;

    // Wishbone decode
    logic wb_access;
    logic do_update;
    logic sync_rst;

    always_comb begin
        wb_access = cyc_i & stb_i & ~ack_o;
        dat_o     = 32'h0;
        do_update = 1'b0;
        sync_rst  = 1'b0;

        // Read path: dat_o valid whenever bus is active (not gated by ~ack_o)
        // This lets the testbench sample dat_o at the ACK cycle in the VPI
        // active region (before ack NBA resolves).
        if (cyc_i & stb_i & ~we_i) begin
            unique case (adr_i[4:2])
                3'd1: dat_o = {16'h0000, crc_reg};
                default: ;
            endcase
        end

        // Write path: only trigger on first cycle (gated by ~ack_o)
        if (wb_access & we_i) begin
            unique case (adr_i[4:2])
                3'd0: do_update = 1'b1;
                3'd2: sync_rst  = dat_i[0];
                default: ;
            endcase
        end
    end

    // Sequential logic
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            crc_reg <= 16'h0000;
            ack_o   <= 1'b0;
        end else begin
            ack_o <= cyc_i & stb_i & ~ack_o;

            if (sync_rst) begin
                crc_reg <= 16'h0000;
            end else if (do_update) begin
                crc_reg <= crc_next;
            end
        end
    end

endmodule

`default_nettype wire
