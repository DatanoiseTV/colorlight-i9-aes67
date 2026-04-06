// SPDX-License-Identifier: MIT
// Multi-Stream RTP Wrapper
//
// Instantiates NUM_STREAMS independent rtp_engine blocks plus the
// glue needed to share a single audio path and a single MAC TX:
//
//   - rtp_stream_router: fans the incoming RTP UDP-payload stream out
//     to all engines (each engine's rtp_rx filters by SSRC)
//
//   - per-stream TX wrappers (instantiated outside, in aes67_top): one
//     tx_udp_wrapper per engine, then a small round-robin arbiter
//     merges them into a single Ethernet frame stream that joins the
//     top-level TX arbiter
//
//   - per-stream channel offset: each engine handles a contiguous block
//     of channels in the I2S/TDM frame. With CHANNELS_PER_STREAM = 8 and
//     NUM_STREAMS = 2, stream 0 owns channels 0-7 and stream 1 owns
//     channels 8-15. The audio crossbar (in aes67_top) routes the
//     I2S/TDM slot index to the right stream based on the slot range.
//
// This module exposes a flattened bus interface so it can be used with
// genvar loops in the top-level wiring without needing a 2D port list.

module rtp_multistream #(
    parameter NUM_STREAMS         = 2,
    parameter SAMPLE_WIDTH        = 24,
    parameter CHANNELS_PER_STREAM = 8,
    parameter JBUF_ADDR_W         = 10
) (
    input  wire        clk,
    input  wire        rst,

    // Per-stream config (flattened, 1 entry per stream)
    input  wire [NUM_STREAMS-1:0]    rx_enable,
    input  wire [NUM_STREAMS-1:0]    tx_enable,
    input  wire [NUM_STREAMS*32-1:0] tx_ssrc,
    input  wire [NUM_STREAMS*32-1:0] rx_expected_ssrc,
    input  wire [NUM_STREAMS*7-1:0]  payload_type,
    input  wire [NUM_STREAMS*4-1:0]  num_channels,
    input  wire [NUM_STREAMS*11-1:0] samples_per_packet,
    input  wire [NUM_STREAMS*JBUF_ADDR_W-1:0] jbuf_target_depth,

    // Single shared RX from packet router
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // Per-stream TX UDP payload outputs (flattened) - each goes to its
    // own tx_udp_wrapper instantiated by the parent
    output wire [NUM_STREAMS*8-1:0] tx_axis_tdata,
    output wire [NUM_STREAMS-1:0]   tx_axis_tvalid,
    output wire [NUM_STREAMS-1:0]   tx_axis_tlast,
    input  wire [NUM_STREAMS-1:0]   tx_axis_tready,

    // Audio path (one shared TDM slot bus, channels split per stream)
    input  wire [3:0]                              audio_rd_ch,    // global slot
    input  wire                                    audio_rd_en,
    output wire [SAMPLE_WIDTH-1:0]                 audio_rd_data,  // muxed
    input  wire [SAMPLE_WIDTH-1:0]                 audio_wr_data,
    input  wire [10:0]                             audio_wr_addr,
    input  wire                                    audio_wr_en,

    // TX trigger and timestamp
    input  wire                                    send_packet_tick,
    input  wire [31:0]                             rtp_timestamp,

    // Per-stream stats (flattened)
    output wire [NUM_STREAMS*32-1:0]               rx_packets_received,
    output wire [NUM_STREAMS*32-1:0]               tx_packets_sent,
    output wire [NUM_STREAMS*32-1:0]               rx_seq_errors,
    output wire [NUM_STREAMS*(JBUF_ADDR_W+1)-1:0]  jbuf_depth
);

    // ---- Fanout incoming RX to all engines ----
    wire [NUM_STREAMS*8-1:0] rx_fan_tdata;
    wire [NUM_STREAMS-1:0]   rx_fan_tvalid;
    wire [NUM_STREAMS-1:0]   rx_fan_tlast;

    rtp_stream_router #(
        .NUM_STREAMS(NUM_STREAMS)
    ) u_rx_router (
        .clk           (clk),
        .rst           (rst),
        .s_axis_tdata  (rx_axis_tdata),
        .s_axis_tvalid (rx_axis_tvalid),
        .s_axis_tlast  (rx_axis_tlast),
        .s_axis_tready (rx_axis_tready),
        .m_axis_tdata  (rx_fan_tdata),
        .m_axis_tvalid (rx_fan_tvalid),
        .m_axis_tlast  (rx_fan_tlast)
    );

    // ---- Per-stream audio_rd_data mux ----
    // Each stream owns a contiguous block of channels.
    // Stream `i` owns channels [i*CHANNELS_PER_STREAM .. (i+1)*CHANNELS_PER_STREAM-1].
    wire [SAMPLE_WIDTH-1:0] stream_audio_rd_data [0:NUM_STREAMS-1];

    // The block index into which the global audio_rd_ch falls
    wire [3:0] global_ch  = audio_rd_ch;
    // CHANNELS_PER_STREAM is power of 2; use a clog2-shift to compute stream idx
    // (for the default of 8 channels/stream, stream_idx = global_ch[3])
    wire [$clog2(NUM_STREAMS)-1:0] active_stream =
        global_ch[$clog2(NUM_STREAMS) + $clog2(CHANNELS_PER_STREAM) - 1
                  -: $clog2(NUM_STREAMS)];

    assign audio_rd_data = stream_audio_rd_data[active_stream];

    // ---- Per-stream rtp_engine instances ----
    genvar s;
    generate
        for (s = 0; s < NUM_STREAMS; s = s + 1) begin : gen_eng
            wire local_rd_en = audio_rd_en && (active_stream == s);

            rtp_engine #(
                .SAMPLE_WIDTH(SAMPLE_WIDTH),
                .NUM_CHANNELS(CHANNELS_PER_STREAM),
                .JBUF_ADDR_W (JBUF_ADDR_W)
            ) u_eng (
                .clk                  (clk),
                .rst                  (rst),
                .rx_enable            (rx_enable[s]),
                .tx_enable            (tx_enable[s]),
                .tx_ssrc              (tx_ssrc          [s*32 +: 32]),
                .rx_expected_ssrc     (rx_expected_ssrc [s*32 +: 32]),
                .payload_type         (payload_type     [s*7  +: 7]),
                .num_channels         (num_channels     [s*4  +: 4]),
                .samples_per_packet   (samples_per_packet[s*11 +: 11]),
                .jbuf_target_depth    (jbuf_target_depth[s*JBUF_ADDR_W +: JBUF_ADDR_W]),
                .rx_axis_tdata        (rx_fan_tdata [s*8 +: 8]),
                .rx_axis_tvalid       (rx_fan_tvalid[s]),
                .rx_axis_tlast        (rx_fan_tlast [s]),
                .rx_axis_tready       (),
                .tx_axis_tdata        (tx_axis_tdata [s*8 +: 8]),
                .tx_axis_tvalid       (tx_axis_tvalid[s]),
                .tx_axis_tlast        (tx_axis_tlast [s]),
                .tx_axis_tready       (tx_axis_tready[s]),
                // Audio: each stream sees a local channel index = global - (s * CHPS)
                .audio_rd_ch          (global_ch - (s * CHANNELS_PER_STREAM)),
                .audio_rd_en          (local_rd_en),
                .audio_rd_data        (stream_audio_rd_data[s]),
                .audio_wr_data        (audio_wr_data),
                .audio_wr_addr        (audio_wr_addr),
                .audio_wr_en          (audio_wr_en),
                .send_packet_tick     (send_packet_tick),
                .rtp_timestamp        (rtp_timestamp),
                .rx_packets_received  (rx_packets_received[s*32 +: 32]),
                .rx_packets_dropped   (),
                .rx_seq_errors        (rx_seq_errors      [s*32 +: 32]),
                .tx_packets_sent      (tx_packets_sent    [s*32 +: 32]),
                .jbuf_depth           (jbuf_depth         [s*(JBUF_ADDR_W+1) +: (JBUF_ADDR_W+1)]),
                .jbuf_underrun        (),
                .jbuf_overrun         ()
            );
        end
    endgenerate

endmodule
