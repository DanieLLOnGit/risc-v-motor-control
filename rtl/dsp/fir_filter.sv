`default_nettype none

module fir_filter (
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

    // Streaming input
    input wire s_valid_i,
    output logic s_ready_o,
    input wire [15:0] s_data_i,

    // Streaming output
    output logic m_valid_o,
    input wire m_ready_i,
    output logic [15:0] m_data_o
);

    // Coefficients — 32-tap Hamming-windowed lowpass, Fc=1kHz, Fs=50MHz
    // Q1.15 format: value = real * 2^15, symmetric linear-phase FIR
    localparam logic [15:0] COEFF_INIT [0:31] = '{
        16'h009C, 16'h00AE, 16'h00E4, 16'h013C,
        16'h01B3, 16'h0242, 16'h02E5, 16'h0394,
        16'h0449, 16'h04FD, 16'h05A7, 16'h0641,
        16'h06C4, 16'h072C, 16'h0773, 16'h0798,
        16'h0798, 16'h0773, 16'h072C, 16'h06C4,
        16'h0641, 16'h05A7, 16'h04FD, 16'h0449,
        16'h0394, 16'h02E5, 16'h0242, 16'h01B3,
        16'h013C, 16'h00E4, 16'h00AE, 16'h009C
    };

    // Internal state
    typedef enum logic [1:0] {IDLE, COMPUTE, OUTPUT} state_t;
    state_t state;

    logic signed [15:0] sample_sr [0:31];
    logic signed [15:0] coeff [0:31];
    logic [4:0] mac_cnt;
    logic signed [31:0] accumulator;
    logic signed [15:0] data_out_reg;
    logic output_valid;
    logic overflow_sticky;
    logic [4:0] coeff_addr_r;
    logic [15:0] coeff_data_r;

     // Wishbone decode (combinational)
     logic wb_access;
    logic wb_write_data_in;
    logic wb_write_coeff_wr;
    logic wb_write_coeff_addr;
    logic wb_write_coeff_data;
    logic wb_write_ctrl;
    logic soft_reset;
    logic clr_ovf;

    always_comb begin
        wb_access = cyc_i & stb_i & ~ack_o;
        wb_write_data_in = wb_access & we_i & (adr_i[4:2] == 3'd0);
        wb_write_ctrl = wb_access & we_i & (adr_i[4:2] == 3'd3);
        wb_write_coeff_addr = wb_access & we_i & (adr_i[4:2] == 3'd4);
        wb_write_coeff_data = wb_access & we_i & (adr_i[4:2] == 3'd5);
        wb_write_coeff_wr = wb_access & we_i & (adr_i[4:2] == 3'd6);
        soft_reset = wb_write_ctrl & dat_i[0];
        clr_ovf = wb_write_ctrl & dat_i[1];
    end

     // New sample trigger (combinational)
     logic new_sample;
    logic signed [15:0] new_sample_data;

    always_comb begin
        if (wb_write_data_in) begin
            new_sample = 1'b1;
            new_sample_data = $signed(dat_i[15:0]);
        end else if (s_valid_i & s_ready_o) begin
            new_sample = 1'b1;
            new_sample_data = $signed(s_data_i);
        end else begin
            new_sample = 1'b0;
            new_sample_data = '0;
        end
    end

    // Wishbone read mux (combinational)
    // Read path NOT gated by ~ack_o — allows testbench VPI to sample
    // dat_o in the active region before NBA resolves ack_o.
    always_comb begin
        dat_o = 32'h0;
        if (cyc_i & stb_i & ~we_i) begin
            unique case (adr_i[4:2])
                3'd0: dat_o = 32'h0;
                3'd1: dat_o = {14'h0, overflow_sticky, 1'b0, 16'(signed'(data_out_reg))};
                3'd2: dat_o = {29'h0, overflow_sticky, (state != IDLE), output_valid};
                3'd3: dat_o = 32'h0;
                3'd4: dat_o = {27'h0, coeff_addr_r};
                3'd5: dat_o = {16'h0, coeff[coeff_addr_r]};
                3'd6: dat_o = 32'h0;
                default: dat_o = 32'h0;
            endcase
        end
    end

    // MAC product (combinational — infers one 16x16 signed multiplier)
    logic signed [31:0] mac_product;
    always_comb begin
        mac_product = sample_sr[mac_cnt] * coeff[mac_cnt];
    end

    // Rounding and saturation (combinational)
    // Round-half-up: add 0x4000 (2^14) then take bits[30:15]
    logic signed [32:0] acc_rounded;
    logic signed [15:0] saturated;
    logic sat_overflow;

    always_comb begin
        acc_rounded = {accumulator[31], accumulator} + 33'sh0000_4000;
        if (!acc_rounded[32] && acc_rounded[30]) begin
            saturated = 16'h7FFF;
            sat_overflow = 1'b1;
        end else if (acc_rounded[32] && !acc_rounded[30]) begin
            saturated = 16'h8000;
            sat_overflow = 1'b1;
        end else begin
            saturated = acc_rounded[30:15];
            sat_overflow = 1'b0;
        end
    end

    // Sequential: FSM + datapath
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state <= IDLE;
            mac_cnt <= '0;
            accumulator <= '0;
            data_out_reg <= '0;
            output_valid <= 1'b0;
            overflow_sticky <= 1'b0;
            coeff_addr_r <= '0;
            coeff_data_r <= '0;
            ack_o <= 1'b0;
            for (int i = 0; i < 32; i++) begin
                sample_sr[i] <= '0;
                coeff[i] <= COEFF_INIT[i];
            end
        end else begin
            ack_o <= cyc_i & stb_i & ~ack_o;

            if (soft_reset) begin
                state <= IDLE;
                mac_cnt <= '0;
                accumulator <= '0;
                data_out_reg <= '0;
                output_valid <= 1'b0;
                overflow_sticky <= 1'b0;
                for (int i = 0; i < 32; i++)
                    sample_sr[i] <= '0;
            end else begin
                if (clr_ovf)
                    overflow_sticky <= 1'b0;

                if (wb_write_coeff_addr)
                    coeff_addr_r <= dat_i[4:0];
                if (wb_write_coeff_data)
                    coeff_data_r <= dat_i[15:0];
                if (wb_write_coeff_wr)
                    coeff[coeff_addr_r] <= coeff_data_r;

                unique case (state)
                    IDLE: begin
                        if (new_sample) begin
                            sample_sr[0] <= new_sample_data;
                            for (int i = 1; i < 32; i++)
                                sample_sr[i] <= sample_sr[i-1];
                            output_valid <= 1'b0;
                            accumulator <= '0;
                            mac_cnt <= '0;
                            state <= COMPUTE;
                        end
                    end

                    COMPUTE: begin
                        accumulator <= accumulator + mac_product;
                        if (mac_cnt == 5'd31) begin
                            state <= OUTPUT;
                        end else begin
                            mac_cnt <= mac_cnt + 5'd1;
                        end
                    end

                    OUTPUT: begin
                        data_out_reg <= saturated;
                        output_valid <= 1'b1;
                        overflow_sticky <= overflow_sticky | sat_overflow;
                        state <= IDLE;
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

    // Streaming ports
    assign s_ready_o = (state == IDLE);
    assign m_valid_o = output_valid;
    assign m_data_o = 16'(signed'(data_out_reg));

endmodule

`default_nettype wire
