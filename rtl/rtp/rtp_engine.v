// SPDX-License-Identifier: MIT
// RTP Engine Top
//
// Combines RTP TX, RTP RX, and jitter buffer for a single audio stream.
// Multiple instances can be parameterized for multi-stream operation.
//
// Note: Ethernet/IP/UDP header construction is delegated to the packet router
// (which fills in src/dst MAC, src/dst IP, ports). This module deals only with
// the UDP payload (the RTP packet itself).

module rtp_engine #(
    parameter SAMPLE_WIDTH = 24,
    parameter NUM_CHANNELS = 8,
    parameter JBUF_ADDR_W  = 10
) (
    input  wire        clk,
    input  wire        rst,

    // Stream config (from CPU CSR)
    input  wire        rx_enable,
    input  wire        tx_enable,
    input  wire [31:0] tx_ssrc,
    input  wire [31:0] rx_expected_ssrc,
    input  wire [6:0]  payload_type,
    input  wire [3:0]  num_channels,
    input  wire [10:0] samples_per_packet,
    input  wire [JBUF_ADDR_W-1:0] jbuf_target_depth,

    // RTP RX from packet router (UDP payload)
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // RTP TX to packet router (UDP payload)
    output wire [7:0]  tx_axis_tdata,
    output wire        tx_axis_tvalid,
    output wire        tx_axis_tlast,
    input  wire        tx_axis_tready,

    // Audio output (to I2S/TDM master) - read by audio side
    input  wire [3:0]  audio_rd_ch,
    input  wire        audio_rd_en,
    output wire [SAMPLE_WIDTH-1:0] audio_rd_data,

    // Audio input (from I2S/TDM master) - written for TX
    input  wire [SAMPLE_WIDTH-1:0] audio_wr_data,
    input  wire [10:0]             audio_wr_addr,
    input  wire                    audio_wr_en,

    // TX trigger from audio frame timer
    input  wire        send_packet_tick,
    input  wire [31:0] rtp_timestamp,

    // Status
    output wire [31:0] rx_packets_received,
    output wire [31:0] rx_packets_dropped,
    output wire [31:0] rx_seq_errors,
    output wire [31:0] tx_packets_sent,
    output wire [JBUF_ADDR_W:0] jbuf_depth,
    output wire        jbuf_underrun,
    output wire        jbuf_overrun
);

    // ---- RX path ----
    wire [SAMPLE_WIDTH-1:0] rx_sample_data;
    wire [3:0]              rx_sample_ch;
    wire                    rx_sample_valid;
    wire                    rx_packet_start;
    wire                    rx_packet_end;
    wire [31:0]             rx_rtp_ts;
    wire [15:0]             rx_rtp_seq;

    rtp_rx #(
        .NUM_CHANNELS(NUM_CHANNELS),
        .SAMPLE_WIDTH(SAMPLE_WIDTH)
    ) u_rx (
        .clk             (clk),
        .rst             (rst | ~rx_enable),
        .rx_axis_tdata   (rx_axis_tdata),
        .rx_axis_tvalid  (rx_axis_tvalid),
        .rx_axis_tlast   (rx_axis_tlast),
        .rx_axis_tready  (rx_axis_tready),
        .expected_ssrc   (rx_expected_ssrc),
        .expected_pt     ({1'b0, payload_type}),
        .num_channels    (num_channels),
        .sample_data     (rx_sample_data),
        .sample_ch       (rx_sample_ch),
        .sample_valid    (rx_sample_valid),
        .packet_start    (rx_packet_start),
        .packet_end      (rx_packet_end),
        .rtp_timestamp   (rx_rtp_ts),
        .rtp_seq         (rx_rtp_seq),
        .packets_received(rx_packets_received),
        .packets_dropped (rx_packets_dropped),
        .seq_errors      (rx_seq_errors)
    );

    // ---- Jitter buffer ----
    jitter_buffer #(
        .SAMPLE_WIDTH(SAMPLE_WIDTH),
        .NUM_CHANNELS(NUM_CHANNELS),
        .ADDR_WIDTH  (JBUF_ADDR_W)
    ) u_jbuf (
        .clk          (clk),
        .rst          (rst),
        .wr_data      (rx_sample_data),
        .wr_ch        (rx_sample_ch),
        .wr_en        (rx_sample_valid),
        .packet_start (rx_packet_start),
        .packet_ts    (rx_rtp_ts),
        .rd_ch        (audio_rd_ch),
        .rd_en        (audio_rd_en),
        .rd_data      (audio_rd_data),
        .num_channels (num_channels),
        .target_depth (jbuf_target_depth),
        .depth_frames (jbuf_depth),
        .underrun     (jbuf_underrun),
        .overrun      (jbuf_overrun)
    );

    // ---- TX sample buffer (BRAM holding the samples for the current packet) ----
    // Written by I2S/TDM RX path, read by RTP TX
    reg [SAMPLE_WIDTH-1:0] tx_sample_mem [0:2047];
    reg [SAMPLE_WIDTH-1:0] tx_sample_rd_data;
    wire [10:0] tx_sample_rd_addr;
    wire        tx_sample_rd_en;

    always @(posedge clk) begin
        if (audio_wr_en)
            tx_sample_mem[audio_wr_addr] <= audio_wr_data;
        if (tx_sample_rd_en)
            tx_sample_rd_data <= tx_sample_mem[tx_sample_rd_addr];
    end

    // ---- TX path ----
    rtp_tx #(
        .NUM_CHANNELS    (NUM_CHANNELS),
        .SAMPLE_WIDTH    (SAMPLE_WIDTH),
        .MAX_SAMPLES_PKT (96)
    ) u_tx (
        .clk                (clk),
        .rst                (rst | ~tx_enable),
        .ssrc               (tx_ssrc),
        .payload_type       (payload_type),
        .num_channels       (num_channels),
        .samples_per_packet (samples_per_packet),
        .send_packet        (send_packet_tick),
        .rtp_timestamp_in   (rtp_timestamp),
        .sample_rd_addr     (tx_sample_rd_addr),
        .sample_rd_en       (tx_sample_rd_en),
        .sample_rd_data     (tx_sample_rd_data),
        .tx_axis_tdata      (tx_axis_tdata),
        .tx_axis_tvalid     (tx_axis_tvalid),
        .tx_axis_tlast      (tx_axis_tlast),
        .tx_axis_tready     (tx_axis_tready),
        .packets_sent       (tx_packets_sent)
    );

endmodule
