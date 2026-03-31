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
    wire sample_clk  = ui_in[1]; // SPI clock
    wire channel_sel = ui_in[2]; // 0=Red, 1=IR

    // =========================================================
    // 1. SPI Deserializer (clocked on external SPI clock)
    // =========================================================
    reg [6:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [7:0] red_sample, ir_sample;
    reg sample_ready;

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
    // 2. Clock-Domain Crossing: sample_ready -> sample_pulse
    // =========================================================
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

    // =========================================================
    // 3. DC Extractor (4-sample moving average, per channel)
    // =========================================================
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
    // 5. Ratio Computation via Sequential Multiply + Compare
    //    Goal: compute r_idx = (r_ac_pk * i_dc * 8) / (i_ac_pk * r_dc)
    //    Method: iteratively accumulate (i_ac_pk * r_dc) and count
    //    how many times it fits into (r_ac_pk * i_dc * 8).
    //    Both multiplies are done sequentially (shift-and-add)
    //    to save area vs. combinational multipliers.
    // =========================================================

    // --- Sequential multiplier ---
    reg [15:0] mul_result;
    reg [7:0]  mul_a, mul_b;
    reg [3:0]  mul_step;
    reg        mul_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_result <= 0;
            mul_step   <= 0;
            mul_busy   <= 0;
            mul_a      <= 0;
            mul_b      <= 0;
        end else if (mul_busy) begin
            if (mul_step < 4'd8) begin
                if (mul_b[mul_step[2:0]])
                    mul_result <= mul_result + ({8'b0, mul_a} << mul_step[2:0]);
                mul_step <= mul_step + 1;
            end else begin
                mul_busy <= 0;
            end
        end
    end

    // =========================================================
    // 6. Control FSM
    //    States: IDLE(0) -> COLLECT(1) -> MUL_NUM(2) -> MUL_DEN(3)
    //            -> DIVIDE(4) -> DONE(5)
    // =========================================================
    reg [2:0] st;
    reg [3:0] r_idx;
    reg [18:0] num_val;    // r_ac_pk * i_dc * 8
    reg [15:0] den_val;    // i_ac_pk * r_dc
    reg [18:0] accum;
    reg vld;

    localparam S_IDLE     = 3'd0;
    localparam S_COLLECT  = 3'd1;
    localparam S_MUL_NUM  = 3'd2;
    localparam S_MUL_DEN  = 3'd3;
    localparam S_DIVIDE   = 3'd4;
    localparam S_DONE     = 3'd5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            vld      <= 0;
            r_idx    <= 0;
            num_val  <= 0;
            den_val  <= 0;
            accum    <= 0;
        end else begin
            case (st)
                S_IDLE: begin
                    if (sample_pulse) st <= S_COLLECT;
                end

                S_COLLECT: begin
                    if (cycle_cnt == 8'd254) begin
                        // Start multiplying: num = r_ac_pk * i_dc
                        mul_a      <= r_ac_pk;
                        mul_b      <= i_dc;
                        mul_result <= 0;
                        mul_step   <= 0;
                        mul_busy   <= 1;
                        st         <= S_MUL_NUM;
                    end else begin
                        st <= S_IDLE;
                    end
                end

                S_MUL_NUM: begin
                    if (!mul_busy) begin
                        // num multiply done, save result * 8
                        num_val <= {mul_result[15:0], 3'b0};
                        // Start multiplying: den = i_ac_pk * r_dc
                        mul_a      <= i_ac_pk;
                        mul_b      <= r_dc;
                        mul_result <= 0;
                        mul_step   <= 0;
                        mul_busy   <= 1;
                        st         <= S_MUL_DEN;
                    end
                end

                S_MUL_DEN: begin
                    if (!mul_busy) begin
                        den_val <= mul_result;
                        accum   <= {3'b0, mul_result};
                        r_idx   <= 0;
                        if (mul_result == 0) begin
                            // Division by zero - default
                            r_idx <= 0;
                            st    <= S_DONE;
                            vld   <= 1'b1;
                        end else begin
                            st <= S_DIVIDE;
                        end
                    end
                end

                S_DIVIDE: begin
                    if (r_idx == 4'd15 || accum > num_val) begin
                        st  <= S_DONE;
                        vld <= 1'b1;
                    end else begin
                        accum <= accum + {3'b0, den_val};
                        r_idx <= r_idx + 1;
                    end
                end

                S_DONE: begin
                    // vld stays latched high; go idle for next window
                    st <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // =========================================================
    // 7. Calibration LUT (R-ratio index -> SpO2 %)
    // =========================================================
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

    // =========================================================
    // Output assignments
    // =========================================================
    assign uo_out  = {vld, spo2};
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Suppress unused-signal warnings
    wire _unused = &{ena, uio_in, ui_in[7:3], r_sum[1:0], i_sum[1:0], 1'b0};

endmodule
