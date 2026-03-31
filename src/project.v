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
    // 3. DC Extractor (2-sample average — area-optimized)
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
    // 5. Sequential Multiplier (shift-add, NO barrel shifter)
    //    Uses simple 1-bit shifts per clock to save area
    // =========================================================
    reg [15:0] mul_acc;
    reg [15:0] mul_a_sh;   // multiplicand, shifts left each step
    reg [7:0]  mul_b_sh;   // multiplier, shifts right each step
    reg        mul_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_acc  <= 0;
            mul_a_sh <= 0;
            mul_b_sh <= 0;
            mul_done <= 1;
        end else if (!mul_done) begin
            if (mul_b_sh == 0) begin
                mul_done <= 1;
            end else begin
                if (mul_b_sh[0])
                    mul_acc <= mul_acc + mul_a_sh;
                mul_a_sh <= {mul_a_sh[14:0], 1'b0};
                mul_b_sh <= {1'b0, mul_b_sh[7:1]};
            end
        end
    end

    // =========================================================
    // 6. Control FSM
    //    IDLE -> COLLECT -> MUL_NUM -> MUL_DEN -> DIVIDE -> IDLE
    // =========================================================
    reg [2:0] st;
    reg [3:0] r_idx;
    reg [18:0] num_x8;     // r_ac_pk * i_dc * 8
    reg [15:0] den_val;    // i_ac_pk * r_dc
    reg [18:0] accum;
    reg vld;

    localparam S_IDLE    = 3'd0,
               S_COLLECT = 3'd1,
               S_MULN    = 3'd2,
               S_MULD    = 3'd3,
               S_DIV     = 3'd4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            vld     <= 0;
            r_idx   <= 0;
            num_x8  <= 0;
            den_val <= 0;
            accum   <= 0;
        end else begin
            case (st)
                S_IDLE: begin
                    if (sample_pulse) st <= S_COLLECT;
                end

                S_COLLECT: begin
                    if (cycle_cnt == 8'd254) begin
                        // Kick off multiply: num = r_ac_pk * i_dc
                        mul_acc  <= 0;
                        mul_a_sh <= {8'b0, r_ac_pk};
                        mul_b_sh <= i_dc;
                        mul_done <= 0;
                        st       <= S_MULN;
                    end else begin
                        st <= S_IDLE;
                    end
                end

                S_MULN: begin
                    if (mul_done) begin
                        num_x8 <= {mul_acc[15:0], 3'b0};
                        // Kick off multiply: den = i_ac_pk * r_dc
                        mul_acc  <= 0;
                        mul_a_sh <= {8'b0, i_ac_pk};
                        mul_b_sh <= r_dc;
                        mul_done <= 0;
                        st       <= S_MULD;
                    end
                end

                S_MULD: begin
                    if (mul_done) begin
                        den_val <= mul_acc;
                        r_idx   <= 0;
                        if (mul_acc == 0) begin
                            vld <= 1'b1;
                            st  <= S_IDLE;
                        end else begin
                            accum <= {3'b0, mul_acc};
                            st    <= S_DIV;
                        end
                    end
                end

                S_DIV: begin
                    if (r_idx == 4'd15 || accum > num_x8) begin
                        vld <= 1'b1;
                        st  <= S_IDLE;
                    end else begin
                        accum <= accum + {3'b0, den_val};
                        r_idx <= r_idx + 1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // =========================================================
    // 7. Calibration LUT
    // =========================================================
    reg [6:0] spo2;
    always @(*) begin
        case (r_idx)
            4'd0:  spo2 = 7'd99; 4'd1:  spo2 = 7'd98;
            4'd2:  spo2 = 7'd97; 4'd3:  spo2 = 7'd96;
            4'd4:  spo2 = 7'd95; 4'd5:  spo2 = 7'd94;
            4'd6:  spo2 = 7'd93; 4'd7:  spo2 = 7'd92;
            4'd8:  spo2 = 7'd91; 4'd9:  spo2 = 7'd90;
            4'd10: spo2 = 7'd88; 4'd11: spo2 = 7'd86;
            4'd12: spo2 = 7'd84; 4'd13: spo2 = 7'd82;
            4'd14: spo2 = 7'd80; 4'd15: spo2 = 7'd75;
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
