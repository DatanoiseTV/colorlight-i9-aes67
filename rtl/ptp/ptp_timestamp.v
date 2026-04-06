// SPDX-License-Identifier: MIT
// PTP Hardware Timestamp Capture Unit
//
// Captures 80-bit timestamps (48-bit sec + 32-bit nsec) at MAC SFD events.
// Separate FIFO for TX and RX timestamps.
// CPU reads timestamps paired with frame metadata for PTP processing.

module ptp_timestamp (
    input  wire        clk,
    input  wire        rst,

    // Current PTP clock
    input  wire [47:0] ts_sec,
    input  wire [31:0] ts_nsec,

    // TX timestamp capture
    input  wire        tx_sfd_pulse,   // from MAC TX (single cycle at SFD)
    output wire [47:0] tx_ts_sec,
    output wire [31:0] tx_ts_nsec,
    output wire        tx_ts_valid,
    input  wire        tx_ts_read,     // CPU acknowledges/pops

    // RX timestamp capture
    input  wire        rx_sfd_pulse,   // from MAC RX (single cycle at SFD)
    output wire [47:0] rx_ts_sec,
    output wire [31:0] rx_ts_nsec,
    output wire        rx_ts_valid,
    input  wire        rx_ts_read,     // CPU acknowledges/pops

    // Overflow indicators
    output wire        tx_ts_overflow,
    output wire        rx_ts_overflow
);

    localparam TS_FIFO_DEPTH = 4; // 2^4 = 16 entries

    // TX timestamp FIFO
    wire        tx_fifo_full, tx_fifo_empty;
    wire [79:0] tx_fifo_rd_data;

    fifo_sync #(
        .DATA_WIDTH(80),
        .ADDR_WIDTH(TS_FIFO_DEPTH)
    ) u_tx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (tx_sfd_pulse && !tx_fifo_full),
        .wr_data({ts_sec, ts_nsec}),
        .rd_en  (tx_ts_read && !tx_fifo_empty),
        .rd_data(tx_fifo_rd_data),
        .full   (tx_fifo_full),
        .empty  (tx_fifo_empty),
        .count  ()
    );

    assign tx_ts_sec      = tx_fifo_rd_data[79:32];
    assign tx_ts_nsec     = tx_fifo_rd_data[31:0];
    assign tx_ts_valid    = ~tx_fifo_empty;
    assign tx_ts_overflow = tx_fifo_full & tx_sfd_pulse;

    // RX timestamp FIFO
    wire        rx_fifo_full, rx_fifo_empty;
    wire [79:0] rx_fifo_rd_data;

    fifo_sync #(
        .DATA_WIDTH(80),
        .ADDR_WIDTH(TS_FIFO_DEPTH)
    ) u_rx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (rx_sfd_pulse && !rx_fifo_full),
        .wr_data({ts_sec, ts_nsec}),
        .rd_en  (rx_ts_read && !rx_fifo_empty),
        .rd_data(rx_fifo_rd_data),
        .full   (rx_fifo_full),
        .empty  (rx_fifo_empty),
        .count  ()
    );

    assign rx_ts_sec      = rx_fifo_rd_data[79:32];
    assign rx_ts_nsec     = rx_fifo_rd_data[31:0];
    assign rx_ts_valid    = ~rx_fifo_empty;
    assign rx_ts_overflow = rx_fifo_full & rx_sfd_pulse;

endmodule
