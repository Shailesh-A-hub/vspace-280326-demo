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

    // Pin mapping
    wire serial_in   = ui_in[0];
    wire sample_clk  = ui_in[1];
    wire channel_sel = ui_in[2]; // 0=Red, 1=IR

    // =========================================================
    // 1. SPI Deserializer
    // =========================================================
    reg [6:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [7:0] red_sample, ir_sample;
    reg       sample_ready;

    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg    <= 7'b0;
            bit_cnt      <= 3'b0;
            sample_ready <= 1'b0;
            red_sample   <= 8'b0;
            ir_sample    <= 8'b0;
        end else begin
            shift_reg <= {shift_reg[5:0], serial_in};
            if (bit_cnt == 3'd7) begin
                if (!channel_sel) red_sample <= {shift_reg, serial_in};
                else              ir_sample  <= {shift_reg, serial_in};
                sample_ready <= 1'b1;
                bit_cnt      <= 3'b0;
            end else begin
                bit_cnt      <= bit_cnt + 1'b1;
                sample_ready <= 1'b0;
            end
        end
    end

    // =========================================================
    // 2. Clock-Domain Crossing
    // =========================================================
    reg sync1, sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sync1 <= 0; sync2 <= 0; end
        else begin sync1 <= sample_ready; sync2 <= sync1; end
    end
    wire sample_pulse = sync1 && !sync2;

    // =========================================================
    // 3. DC Extractor (2-sample average)
    // =========================================================
    reg [7:0] r_prev, i_prev;
    wire [7:0] r_dc = {1'b0, red_sample[7:1]} + {1'b0, r_prev[7:1]};
    wire [7:0] i_dc = {1'b0, ir_sample[7:1]}  + {1'b0, i_prev[7:1]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin r_prev <= 0; i_prev <= 0; end
        else if (sample_pulse) begin
            if (!channel_sel) r_prev <= red_sample;
            else              i_prev <= ir_sample;
        end
    end

    // =========================================================
    // 4. AC Peak Holder
    // =========================================================
    reg [7:0] r_ac_pk, i_ac_pk;
    reg [7:0] cycle_cnt;
    wire [7:0] r_diff = (red_sample > r_dc) ? (red_sample - r_dc) : (r_dc - red_sample);
    wire [7:0] i_diff = (ir_sample  > i_dc) ? (ir_sample  - i_dc) : (i_dc - ir_sample);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_ac_pk <= 0; i_ac_pk <= 0; cycle_cnt <= 0;
        end else if (sample_pulse) begin
            if (cycle_cnt == 8'd255) begin
                r_ac_pk <= 0; i_ac_pk <= 0; cycle_cnt <= 0;
            end else begin
                if (!channel_sel && r_diff > r_ac_pk) r_ac_pk <= r_diff;
                if ( channel_sel && i_diff > i_ac_pk) i_ac_pk <= i_diff;
                cycle_cnt <= cycle_cnt + 1'b1;
            end
        end
    end

    // =========================================================
    // 5. Ratio via Iterative Division (NO MULTIPLIER)
    //    Computes r_idx = (r_ac_pk << 3) / i_ac_pk
    //    using repeated addition/comparison.
    //    This gives the AC ratio scaled by 8 for LUT precision.
    //    (DC correction is baked into the LUT values.)
    // =========================================================
    reg [2:0] st;
    reg [3:0] r_idx;
    reg [10:0] num_val;   // r_ac_pk << 3 (max 255*8=2040, fits 11 bits)
    reg [10:0] accum;     // running sum of i_ac_pk
    reg vld;

    localparam S_IDLE    = 3'd0,
               S_COLLECT = 3'd1,
               S_DIV     = 3'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            vld     <= 0;
            r_idx   <= 0;
            num_val <= 0;
            accum   <= 0;
        end else begin
            case (st)
                S_IDLE: begin
                    if (sample_pulse) st <= S_COLLECT;
                end

                S_COLLECT: begin
                    if (cycle_cnt == 8'd254) begin
                        if (i_ac_pk == 0) begin
                            // Avoid division by zero
                            r_idx <= 4'd0;
                            vld   <= 1'b1;
                            st    <= S_IDLE;
                        end else begin
                            num_val <= {r_ac_pk, 3'b0};   // r_ac_pk * 8
                            accum   <= {3'b0, i_ac_pk};   // start with 1x den
                            r_idx   <= 4'd0;
                            st      <= S_DIV;
                        end
                    end else begin
                        st <= S_IDLE;
                    end
                end

                S_DIV: begin
                    if (r_idx == 4'd15 || accum > num_val) begin
                        vld <= 1'b1;
                        st  <= S_IDLE;
                    end else begin
                        accum <= accum + {3'b0, i_ac_pk};
                        r_idx <= r_idx + 1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // =========================================================
    // 6. Calibration LUT (adjusted for AC-only ratio)
    // =========================================================
    reg [6:0] spo2;
    always @(*) begin
        case (r_idx)
            4'd0:  spo2 = 7'd75;  4'd1:  spo2 = 7'd80;
            4'd2:  spo2 = 7'd84;  4'd3:  spo2 = 7'd88;
            4'd4:  spo2 = 7'd90;  4'd5:  spo2 = 7'd92;
            4'd6:  spo2 = 7'd94;  4'd7:  spo2 = 7'd95;
            4'd8:  spo2 = 7'd96;  4'd9:  spo2 = 7'd97;
            4'd10: spo2 = 7'd97;  4'd11: spo2 = 7'd98;
            4'd12: spo2 = 7'd98;  4'd13: spo2 = 7'd99;
            4'd14: spo2 = 7'd99;  4'd15: spo2 = 7'd99;
            default: spo2 = 7'd0;
        endcase
    end

    // =========================================================
    // Outputs
    // =========================================================
    assign uo_out  = {vld, spo2};
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Suppress unused-signal warnings
    wire _unused = &{ena, uio_in, ui_in[7:3], 1'b0};

endmodule
