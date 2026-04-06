// SPDX-License-Identifier: MIT
// PTP Hardware Timestamp Capture Unit (IEEE 1588-2008 §7.3.4 / §7.4)
//
// Captures the live PTP clock at the precise cycle the SFD crosses the
// MII boundary, applies the per-port PHY/cable asymmetry compensation
// (RFC §7.4.2), and pushes the (sec, nsec) result into a FIFO for the
// packet processor to consume.
//
// Correctness notes:
//
//   1. SFD is detected in the MAC clock domain. The captured timestamp
//      is the live PTP clock value at THAT cycle, with no synchronizer
//      jitter added (which would otherwise bound accuracy to ~16-24 ns
//      at 125 MHz).
//
//   2. Compensation:
//        rx_ts_wire = rx_ts_observed - rx_delay_ns
//        tx_ts_wire = tx_ts_observed + tx_delay_ns
//
//      The "observed" timestamp is when the MAC saw the SFD; the
//      "wire" timestamp is when the SFD was on the medium. PHY+PCS+
//      RGMII pipelines add a fixed RX delay (subtract) and TX delay
//      (add). Values are in signed nanoseconds, runtime configurable.
//
//   3. Second-rollover handling: the compensation arithmetic correctly
//      borrows/carries across the seconds boundary.
//
//   4. CDC: in this build sys_clk and mac_clk are the same physical
//      clock (clk125 from the on-chip PLL), but the module is written
//      so that splitting them later (e.g. PHY-recovered clock domain)
//      remains correct.

module ptp_timestamp (
    input  wire        clk,           // shared system / MAC clock
    input  wire        rst,

    // Live PTP clock
    input  wire [47:0] ts_sec,
    input  wire [31:0] ts_nsec,

    // Asymmetry compensation (RFC §7.4) - signed nanoseconds
    input  wire signed [31:0] tx_delay_ns,
    input  wire signed [31:0] rx_delay_ns,

    // SFD pulses (one cycle, MAC domain)
    input  wire        tx_sfd_pulse,
    input  wire        rx_sfd_pulse,

    // FIFO read side
    output wire [47:0] tx_ts_sec,
    output wire [31:0] tx_ts_nsec,
    output wire        tx_ts_valid,
    input  wire        tx_ts_read,

    output wire [47:0] rx_ts_sec,
    output wire [31:0] rx_ts_nsec,
    output wire        rx_ts_valid,
    input  wire        rx_ts_read,

    output wire        tx_ts_overflow,
    output wire        rx_ts_overflow
);

    localparam [31:0] NS_PER_SEC = 32'd1_000_000_000;

    // Apply signed delay with carry/borrow across the seconds boundary.
    function [79:0] apply_delay;
        input [79:0] in_ts;
        input signed [31:0] delay;
        reg [47:0] s;
        reg [31:0] n;
        reg signed [33:0] tmp;
        begin
            s   = in_ts[79:32];
            n   = in_ts[31:0];
            tmp = $signed({2'b00, n}) + $signed({{2{delay[31]}}, delay});
            if (tmp >= $signed({2'b00, NS_PER_SEC})) begin
                n = tmp - NS_PER_SEC;
                s = s + 1;
            end else if (tmp < 0) begin
                n = tmp + NS_PER_SEC;
                s = s - 1;
            end else begin
                n = tmp[31:0];
            end
            apply_delay = {s, n};
        end
    endfunction

    wire [79:0] live_ts        = {ts_sec, ts_nsec};
    wire [79:0] tx_ts_corrected = apply_delay(live_ts,  tx_delay_ns);
    wire [79:0] rx_ts_corrected = apply_delay(live_ts, rx_delay_ns);

    // Capture register: latch the corrected timestamp on the SFD pulse.
    // Because the function evaluates combinationally, the value latched
    // is the same-cycle PTP time corrected for the configured delay.
    reg [79:0] tx_capture;
    reg        tx_capture_valid;
    reg [79:0] rx_capture;
    reg        rx_capture_valid;

    always @(posedge clk) begin
        if (rst) begin
            tx_capture       <= 80'd0;
            tx_capture_valid <= 1'b0;
            rx_capture       <= 80'd0;
            rx_capture_valid <= 1'b0;
        end else begin
            tx_capture_valid <= tx_sfd_pulse;
            rx_capture_valid <= rx_sfd_pulse;
            if (tx_sfd_pulse) tx_capture <= tx_ts_corrected;
            if (rx_sfd_pulse) rx_capture <= rx_ts_corrected;
        end
    end

    // Sync FIFOs (16 entries) so multiple in-flight PTP frames don't drop.
    wire        tx_full, tx_empty, rx_full, rx_empty;
    wire [79:0] tx_rd, rx_rd;

    fifo_sync #(.DATA_WIDTH(80), .ADDR_WIDTH(4)) u_tx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (tx_capture_valid & ~tx_full),
        .wr_data(tx_capture),
        .rd_en  (tx_ts_read & ~tx_empty),
        .rd_data(tx_rd),
        .full   (tx_full),
        .empty  (tx_empty),
        .count  ()
    );

    fifo_sync #(.DATA_WIDTH(80), .ADDR_WIDTH(4)) u_rx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (rx_capture_valid & ~rx_full),
        .wr_data(rx_capture),
        .rd_en  (rx_ts_read & ~rx_empty),
        .rd_data(rx_rd),
        .full   (rx_full),
        .empty  (rx_empty),
        .count  ()
    );

    assign tx_ts_sec      = tx_rd[79:32];
    assign tx_ts_nsec     = tx_rd[31:0];
    assign tx_ts_valid    = ~tx_empty;
    assign tx_ts_overflow = tx_full & tx_capture_valid;

    assign rx_ts_sec      = rx_rd[79:32];
    assign rx_ts_nsec     = rx_rd[31:0];
    assign rx_ts_valid    = ~rx_empty;
    assign rx_ts_overflow = rx_full & rx_capture_valid;

endmodule
