// SPDX-License-Identifier: MIT
// PTP Subsystem Top
// Integrates clock, timestamp capture, packet processor, and servo
// This is the complete IEEE 1588 hardware engine

module ptp_top (
    input  wire        clk,
    input  wire        rst,

    // Configuration
    input  wire [63:0] local_clock_id,
    input  wire [31:0] kp,                  // servo P gain
    input  wire [31:0] ki,                  // servo I gain
    input  wire [31:0] step_threshold_ns,   // phase step threshold

    // PTP RX path (filtered packets from packet router)
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,
    input  wire        rx_sfd_pulse,        // from MAC RX (per-frame SFD)

    // PTP TX path (to packet router → MAC)
    output wire [7:0]  tx_axis_tdata,
    output wire        tx_axis_tvalid,
    output wire        tx_axis_tlast,
    input  wire        tx_axis_tready,
    input  wire        tx_sfd_pulse,        // from MAC TX (per-frame SFD)

    // Current PTP time (for use by RTP engine, audio clock, etc.)
    output wire [47:0] ptp_sec,
    output wire [31:0] ptp_nsec,
    output wire        pps,

    // Status
    output wire [1:0]  lock_state,
    output wire [31:0] offset_filtered_ns,
    output wire [31:0] path_delay_filtered_ns,
    output wire [31:0] sync_count,
    output wire [15:0] last_seq_id
);

    // ---- Internal signals ----
    wire [31:0] increment;
    wire        increment_wr;
    wire        phase_adj_en;
    wire signed [31:0] phase_adj_ns;
    wire        sec_set_en;
    wire [47:0] sec_set_val;

    wire [47:0] tx_ts_sec, rx_ts_sec;
    wire [31:0] tx_ts_nsec, rx_ts_nsec;
    wire        tx_ts_valid, rx_ts_valid;
    wire        tx_ts_consume, rx_ts_consume;

    wire        offset_valid;
    wire signed [63:0] offset_ns_w;
    wire signed [63:0] path_delay_ns_w;

    wire signed [31:0] freq_adj_ppb;
    wire        freq_adj_valid;
    wire signed [31:0] phase_step_ns;
    wire        phase_step_valid;

    // ---- Frequency adjustment: convert ppb to increment delta ----
    // increment is in 8.24 fixed-point ns. Nominal = 0x08000000 (8.0 ns at 125 MHz)
    // 1 ppb = 0.001 ns/sec / 8 ns/cycle increment ≈ 8e-9 of the increment
    // delta_increment = (freq_adj_ppb * NOMINAL_INCR) / 1e9
    //                 ≈ freq_adj_ppb * 0.134 (in LSBs)
    // For simplicity: shift-based approximation
    reg [31:0] increment_reg;
    reg        increment_wr_reg;
    localparam [31:0] NOMINAL_INCR = 32'h08_000000;

    always @(posedge clk) begin
        if (rst) begin
            increment_reg    <= NOMINAL_INCR;
            increment_wr_reg <= 0;
        end else begin
            increment_wr_reg <= 0;
            if (freq_adj_valid) begin
                // delta = freq_adj_ppb * 134 (approx 0.134 LSB per ppb)
                // = ppb * 128 + ppb * 4 + ppb * 2  (134 ≈ 128+4+2)
                increment_reg <= NOMINAL_INCR + (freq_adj_ppb <<< 7)
                                              + (freq_adj_ppb <<< 2)
                                              + (freq_adj_ppb <<< 1);
                increment_wr_reg <= 1;
            end
        end
    end

    assign increment    = increment_reg;
    assign increment_wr = increment_wr_reg;
    assign phase_adj_en = phase_step_valid;
    assign phase_adj_ns = phase_step_ns;

    // ---- PTP Clock ----
    ptp_clock u_clock (
        .clk          (clk),
        .rst          (rst),
        .ts_sec       (ptp_sec),
        .ts_nsec      (ptp_nsec),
        .ts_frac      (),
        .increment    (increment),
        .increment_wr (increment_wr),
        .phase_adj_en (phase_adj_en),
        .phase_adj_ns (phase_adj_ns),
        .sec_set_en   (sec_set_en),
        .sec_set_val  (sec_set_val),
        .pps          (pps)
    );

    // ---- Timestamp Capture ----
    ptp_timestamp u_ts (
        .clk            (clk),
        .rst            (rst),
        .ts_sec         (ptp_sec),
        .ts_nsec        (ptp_nsec),
        .tx_sfd_pulse   (tx_sfd_pulse),
        .tx_ts_sec      (tx_ts_sec),
        .tx_ts_nsec     (tx_ts_nsec),
        .tx_ts_valid    (tx_ts_valid),
        .tx_ts_read     (tx_ts_consume),
        .rx_sfd_pulse   (rx_sfd_pulse),
        .rx_ts_sec      (rx_ts_sec),
        .rx_ts_nsec     (rx_ts_nsec),
        .rx_ts_valid    (rx_ts_valid),
        .rx_ts_read     (rx_ts_consume),
        .tx_ts_overflow (),
        .rx_ts_overflow ()
    );

    // ---- Packet Processor ----
    ptp_pp u_pp (
        .clk             (clk),
        .rst             (rst),
        .rx_axis_tdata   (rx_axis_tdata),
        .rx_axis_tvalid  (rx_axis_tvalid),
        .rx_axis_tlast   (rx_axis_tlast),
        .rx_axis_tready  (rx_axis_tready),
        .rx_ts_sec       (rx_ts_sec),
        .rx_ts_nsec      (rx_ts_nsec),
        .rx_ts_valid     (rx_ts_valid),
        .rx_ts_consume   (rx_ts_consume),
        .tx_ts_sec       (tx_ts_sec),
        .tx_ts_nsec      (tx_ts_nsec),
        .tx_ts_valid     (tx_ts_valid),
        .tx_ts_consume   (tx_ts_consume),
        .tx_axis_tdata   (tx_axis_tdata),
        .tx_axis_tvalid  (tx_axis_tvalid),
        .tx_axis_tlast   (tx_axis_tlast),
        .tx_axis_tready  (tx_axis_tready),
        .offset_valid    (offset_valid),
        .offset_ns       (offset_ns_w),
        .path_delay_ns   (path_delay_ns_w),
        .sec_set_en      (sec_set_en),
        .sec_set_val     (sec_set_val),
        .local_clock_id  (local_clock_id),
        .last_seq_id     (last_seq_id),
        .sync_count      (sync_count)
    );

    // ---- Servo ----
    ptp_servo u_servo (
        .clk                    (clk),
        .rst                    (rst),
        .offset_valid           (offset_valid),
        .offset_ns              (offset_ns_w),
        .path_delay_ns          (path_delay_ns_w),
        .kp                     (kp),
        .ki                     (ki),
        .step_threshold         (step_threshold_ns),
        .freq_adj_ppb           (freq_adj_ppb),
        .freq_adj_valid         (freq_adj_valid),
        .phase_step_ns          (phase_step_ns),
        .phase_step_valid       (phase_step_valid),
        .lock_state             (lock_state),
        .offset_filtered_ns     (offset_filtered_ns),
        .path_delay_filtered_ns (path_delay_filtered_ns)
    );

endmodule
