// SPDX-License-Identifier: MIT
// PTP Subsystem Top - integrates clock, hardware timestamping with
// asymmetry compensation, packet processor, low-pass filter, and PI servo.
// Implements the slave-side IEEE 1588-2008 hardware engine.

module ptp_top (
    input  wire        clk,
    input  wire        rst,

    // Configuration
    input  wire [63:0] local_clock_id,
    input  wire [31:0] kp,
    input  wire [31:0] ki,
    input  wire [31:0] step_threshold_ns,

    // Asymmetry compensation (RFC §7.4) - signed nanoseconds
    input  wire signed [31:0] tx_delay_ns,
    input  wire signed [31:0] rx_delay_ns,
    input  wire [3:0]  filter_shift,         // 0..15 (low-pass time constant)

    // Mode (RFC BMC: master vs slave) and master Sync interval
    input  wire        mode_is_master,
    input  wire [31:0] sync_interval_cycles, // master Sync TX period

    // PTP RX path (UDP payload from packet router)
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,
    input  wire        rx_sfd_pulse,

    // PTP TX path (UDP payload to TX wrapper)
    output wire [7:0]  tx_axis_tdata,
    output wire        tx_axis_tvalid,
    output wire        tx_axis_tlast,
    input  wire        tx_axis_tready,
    input  wire        tx_sfd_pulse,

    // Live PTP time (consumed by RTP / audio NCO)
    output wire [47:0] ptp_sec,
    output wire [31:0] ptp_nsec,
    output wire        pps,

    // Frequency-correction broadcast (consumed by audio NCO)
    output wire signed [31:0] freq_adj_ppb,
    output wire        freq_adj_valid,

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

    wire        offset_valid_raw;
    wire signed [63:0] offset_ns_raw;
    wire signed [63:0] path_delay_ns_w;

    wire        offset_valid_filt;
    wire signed [63:0] offset_ns_filt;

    wire signed [31:0] phase_step_ns;
    wire        phase_step_valid;

    // ---- Frequency adjustment to clock NCO ----
    // The PTP clock NCO increment is in 8.24 fixed-point ns/cycle.
    // Nominal at 125 MHz = 8.0 ns = 0x08000000.
    // Sensitivity: 1 ppb = 1e-9 of 8.0 ns/cycle = 8e-9 ns/cycle
    //            = 8e-9 / 2^-24 LSB = 8e-9 * 16777216 ≈ 0.134 LSB.
    // delta_increment ≈ ppb * 134 / 1000 = (ppb << 7) >> ~9.9
    // We approximate with 134/1024 (within 0.7%) and use a multiply.
    reg [31:0] increment_reg;
    reg        increment_wr_reg;
    localparam [31:0] NOMINAL_INCR = 32'h08_000000;
    reg signed [63:0] freq_mult;

    always @(posedge clk) begin
        if (rst) begin
            increment_reg    <= NOMINAL_INCR;
            increment_wr_reg <= 0;
            freq_mult        <= 0;
        end else begin
            increment_wr_reg <= 0;
            if (freq_adj_valid) begin
                // delta = ppb * 137 / 1024 (≈ 0.1338, error < 0.1%)
                freq_mult <= ($signed({{32{freq_adj_ppb[31]}}, freq_adj_ppb})
                              * 64'sd137) >>> 10;
                increment_reg    <= NOMINAL_INCR + freq_mult[31:0];
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

    // ---- Hardware timestamp capture (with asymmetry compensation) ----
    ptp_timestamp u_ts (
        .clk            (clk),
        .rst            (rst),
        .ts_sec         (ptp_sec),
        .ts_nsec        (ptp_nsec),
        .tx_delay_ns    (tx_delay_ns),
        .rx_delay_ns    (rx_delay_ns),
        .tx_sfd_pulse   (tx_sfd_pulse),
        .rx_sfd_pulse   (rx_sfd_pulse),
        .tx_ts_sec      (tx_ts_sec),
        .tx_ts_nsec     (tx_ts_nsec),
        .tx_ts_valid    (tx_ts_valid),
        .tx_ts_read     (tx_ts_consume),
        .rx_ts_sec      (rx_ts_sec),
        .rx_ts_nsec     (rx_ts_nsec),
        .rx_ts_valid    (rx_ts_valid),
        .rx_ts_read     (rx_ts_consume),
        .tx_ts_overflow (),
        .rx_ts_overflow ()
    );

    // ---- Packet Processor ----
    ptp_pp u_pp (
        .clk                  (clk),
        .rst                  (rst),
        .mode_is_master       (mode_is_master),
        .sync_interval_cycles (sync_interval_cycles),
        .rx_axis_tdata        (rx_axis_tdata),
        .rx_axis_tvalid       (rx_axis_tvalid),
        .rx_axis_tlast        (rx_axis_tlast),
        .rx_axis_tready       (rx_axis_tready),
        .rx_ts_sec            (rx_ts_sec),
        .rx_ts_nsec           (rx_ts_nsec),
        .rx_ts_valid          (rx_ts_valid),
        .rx_ts_consume        (rx_ts_consume),
        .tx_ts_sec            (tx_ts_sec),
        .tx_ts_nsec           (tx_ts_nsec),
        .tx_ts_valid          (tx_ts_valid),
        .tx_ts_consume        (tx_ts_consume),
        .tx_axis_tdata        (tx_axis_tdata),
        .tx_axis_tvalid       (tx_axis_tvalid),
        .tx_axis_tlast        (tx_axis_tlast),
        .tx_axis_tready       (tx_axis_tready),
        .offset_valid         (offset_valid_raw),
        .offset_ns            (offset_ns_raw),
        .path_delay_ns        (path_delay_ns_w),
        .sec_set_en           (sec_set_en),
        .sec_set_val          (sec_set_val),
        .local_clock_id       (local_clock_id),
        .last_seq_id          (last_seq_id),
        .sync_count           (sync_count)
    );

    // ---- Low-pass filter on offset (drives the servo input) ----
    ptp_filter u_filt (
        .clk              (clk),
        .rst              (rst),
        .valid            (offset_valid_raw),
        .in               (offset_ns_raw),
        .shift            (filter_shift),
        // Bypass for any offset > 10 µs (jumps go straight through)
        .bypass_threshold (64'd10_000),
        .out_valid        (offset_valid_filt),
        .out              (offset_ns_filt)
    );

    // ---- PI Servo ----
    ptp_servo u_servo (
        .clk                    (clk),
        .rst                    (rst),
        .offset_valid           (offset_valid_filt),
        .offset_ns              (offset_ns_filt),
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
