`default_nettype none

module tt_um_shailesh_spo2_engine (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Suppress unused-signal warnings
    wire _unused = &{ena, uio_in, ui_in[7:3], 1'b0};

    // Pin mapping
    wire serial_in   = ui_in[0];
    wire sample_clk  = ui_in[1]; // SPI clock
    wire channel_sel = ui_in[2]; // 0=Red, 1=IR

    // 1. SPI Deserializer
    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [7:0] red_sample, ir_sample;
    reg sample_ready;

    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg    <= 8'b0;
            bit_cnt      <= 3'b0;
            sample_ready <= 1'b0;
            red_sample   <= 8'b0;
            ir_sample    <= 8'b0;
        end else begin
            shift_reg <= {shift_reg[6:0], serial_in};
            if (bit_cnt == 3'd7) begin
                if (!channel_sel) red_sample <= {shift_reg[6:0], serial_in};
                else              ir_sample  <= {shift_reg[6:0], serial_in};
                sample_ready <= 1'b1;
                bit_cnt      <= 3'b0;
            end else begin
                bit_cnt      <= bit_cnt + 1'b1;
                sample_ready <= 1'b0;
            end
        end
    end

    // 2. DC Extractor (4-sample moving average)
    reg sample_ready_sync, sample_ready_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_ready_sync <= 1'b0;
            sample_ready_prev <= 1'b0;
        end else begin
            sample_ready_sync <= sample_ready;
            sample_ready_prev <= sample_ready_sync;
        end
    end
    wire sample_pulse = sample_ready_sync && !sample_ready_prev;

    reg [7:0] r_m0, r_m1, r_m2;
    reg [7:0] i_m0, i_m1, i_m2;
    reg [9:0] r_sum, i_sum;
    wire [7:0] r_dc = r_sum[9:2];
    wire [7:0] i_dc = i_sum[9:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_m0 <= 0; r_m1 <= 0; r_m2 <= 0;
            i_m0 <= 0; i_m1 <= 0; i_m2 <= 0;
            r_sum <= 0; i_sum <= 0;
        end else if (sample_pulse) begin
            if (!channel_sel) begin
                r_m0 <= red_sample; r_m1 <= r_m0; r_m2 <= r_m1;
                r_sum <= {2'b0, red_sample} + {2'b0, r_m0} + {2'b0, r_m1} + {2'b0, r_m2};
            end else begin
                i_m0 <= ir_sample; i_m1 <= i_m0; i_m2 <= i_m1;
                i_sum <= {2'b0, ir_sample} + {2'b0, i_m0} + {2'b0, i_m1} + {2'b0, i_m2};
            end
        end
    end

    // 3. AC Peak Holder
    reg [7:0] r_ac_max, i_ac_max;
    reg [7:0] cycle_cnt;
    wire [7:0] r_diff = (red_sample > r_dc) ? (red_sample - r_dc) : (r_dc - red_sample);
    wire [7:0] i_diff = (ir_sample  > i_dc) ? (ir_sample  - i_dc) : (i_dc - ir_sample);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_ac_max <= 0; i_ac_max <= 0; cycle_cnt <= 0;
        end else if (sample_pulse) begin
            if (cycle_cnt == 8'd255) begin
                r_ac_max <= 0; i_ac_max <= 0; cycle_cnt <= 0;
            end else begin
                if (!channel_sel && r_diff > r_ac_max) r_ac_max <= r_diff;
                if (channel_sel  && i_diff > i_ac_max) i_ac_max <= i_diff;
                cycle_cnt <= cycle_cnt + 1'b1;
            end
        end
    end

    // 4. Ratio Unit
    wire [15:0] num = r_ac_max * i_dc;
    wire [15:0] den = i_ac_max * r_dc;
    reg [3:0] r_idx;
    reg [19:0] accum_calc;
    reg [19:0] num_calc;
    reg [19:0] den_ext;   // 20-bit to match accum_calc width

    // 5. Calibration LUT
    reg [6:0] spo2;
    always @(*) begin
        case (r_idx)
            4'd0:  spo2 = 99;  4'd1:  spo2 = 98;  4'd2:  spo2 = 97;  4'd3:  spo2 = 96;
            4'd4:  spo2 = 95;  4'd5:  spo2 = 94;  4'd6:  spo2 = 93;  4'd7:  spo2 = 92;
            4'd8:  spo2 = 91;  4'd9:  spo2 = 90;  4'd10: spo2 = 88;  4'd11: spo2 = 86;
            4'd12: spo2 = 84;  4'd13: spo2 = 82;  4'd14: spo2 = 80;  4'd15: spo2 = 75;
            default: spo2 = 0;
        endcase
    end

    // 6. Control FSM
    reg [1:0] st;
    reg vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= 0;
            vld        <= 0;
            r_idx      <= 0;
            accum_calc <= 0;
            num_calc   <= 0;
            den_ext    <= 0;
        end else begin
            case (st)
                2'd0: if (sample_pulse) st <= 2'd1;
                2'd1: if (cycle_cnt == 8'd254) begin
                          st         <= 2'd2;
                          num_calc   <= {4'b0, num} << 3;
                          den_ext    <= {4'b0, den};
                          accum_calc <= {4'b0, den};
                          r_idx      <= 0;
                      end else st <= 2'd0;
                2'd2: begin
                          if (den_ext == 0) begin
                              r_idx <= 0;
                              st    <= 2'd3;
                              vld   <= 1'b1;
                          end else if (r_idx == 4'd15 || accum_calc > num_calc) begin
                              st  <= 2'd3;
                              vld <= 1'b1;
                          end else begin
                              accum_calc <= accum_calc + den_ext;
                              r_idx      <= r_idx + 1;
                          end
                      end
                2'd3: begin st <= 2'd0; vld <= 1'b0; end
            endcase
        end
    end

    assign uo_out  = {vld, spo2};
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

endmodule
