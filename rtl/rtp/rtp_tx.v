// SPDX-License-Identifier: MIT
// RTP Transmit Engine
//
// Reads audio samples from a per-channel buffer and constructs RTP packets.
// Packet generation is gated by the audio frame timer (PTP-locked).
//
// This module emits the UDP payload (RTP header + L24 audio).
// A wrapper module (rtp_engine.v) prepends Ethernet/IP/UDP headers.

module rtp_tx #(
    parameter NUM_CHANNELS    = 8,
    parameter SAMPLE_WIDTH    = 24,
    parameter MAX_SAMPLES_PKT = 96     // up to 96 samples/packet (96kHz × 1ms)
) (
    input  wire        clk,
    input  wire        rst,

    // Stream config
    input  wire [31:0] ssrc,
    input  wire [6:0]  payload_type,
    input  wire [3:0]  num_channels,
    input  wire [10:0] samples_per_packet,

    // Trigger from audio frame timer (one pulse per packet)
    input  wire        send_packet,
    input  wire [31:0] rtp_timestamp_in,  // current PTP-derived sample counter

    // Sample read port (drives external BRAM holding samples)
    output reg  [10:0] sample_rd_addr,
    output reg         sample_rd_en,
    input  wire [SAMPLE_WIDTH-1:0] sample_rd_data,

    // Output AXI-Stream (UDP payload)
    output reg  [7:0]  tx_axis_tdata,
    output reg         tx_axis_tvalid,
    output reg         tx_axis_tlast,
    input  wire        tx_axis_tready,

    // Status
    output reg  [31:0] packets_sent
);

    localparam [3:0]
        S_IDLE     = 4'd0,
        S_HDR      = 4'd1,
        S_FETCH    = 4'd2,
        S_S0       = 4'd3,
        S_S1       = 4'd4,
        S_S2       = 4'd5,
        S_DONE     = 4'd6;

    reg [3:0]  state;
    reg [3:0]  hdr_idx;
    reg [10:0] sample_idx;     // current sample index in packet
    reg [3:0]  ch_idx;
    reg [15:0] seq_num;
    reg [31:0] ts_latch;

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            hdr_idx        <= 0;
            sample_idx     <= 0;
            ch_idx         <= 0;
            seq_num        <= 0;
            ts_latch       <= 0;
            tx_axis_tdata  <= 0;
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
            sample_rd_addr <= 0;
            sample_rd_en   <= 0;
            packets_sent   <= 0;
        end else begin
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
            sample_rd_en   <= 0;

            case (state)
                S_IDLE: begin
                    if (send_packet) begin
                        state      <= S_HDR;
                        hdr_idx    <= 0;
                        sample_idx <= 0;
                        ch_idx     <= 0;
                        ts_latch   <= rtp_timestamp_in;
                    end
                end

                S_HDR: begin
                    tx_axis_tvalid <= 1;
                    case (hdr_idx)
                        4'd0:  tx_axis_tdata <= 8'h80;              // V=2, P=0, X=0, CC=0
                        4'd1:  tx_axis_tdata <= {1'b0, payload_type}; // M=0, PT
                        4'd2:  tx_axis_tdata <= seq_num[15:8];
                        4'd3:  tx_axis_tdata <= seq_num[7:0];
                        4'd4:  tx_axis_tdata <= ts_latch[31:24];
                        4'd5:  tx_axis_tdata <= ts_latch[23:16];
                        4'd6:  tx_axis_tdata <= ts_latch[15:8];
                        4'd7:  tx_axis_tdata <= ts_latch[7:0];
                        4'd8:  tx_axis_tdata <= ssrc[31:24];
                        4'd9:  tx_axis_tdata <= ssrc[23:16];
                        4'd10: tx_axis_tdata <= ssrc[15:8];
                        4'd11: tx_axis_tdata <= ssrc[7:0];
                    endcase
                    if (tx_axis_tready) begin
                        hdr_idx <= hdr_idx + 1;
                        if (hdr_idx == 11) begin
                            state          <= S_FETCH;
                            sample_rd_addr <= 0;
                            sample_rd_en   <= 1;
                        end
                    end
                end

                S_FETCH: begin
                    // Wait one cycle for BRAM data
                    state <= S_S0;
                end

                S_S0: begin
                    tx_axis_tvalid <= 1;
                    tx_axis_tdata  <= sample_rd_data[23:16];
                    if (tx_axis_tready) state <= S_S1;
                end

                S_S1: begin
                    tx_axis_tvalid <= 1;
                    tx_axis_tdata  <= sample_rd_data[15:8];
                    if (tx_axis_tready) state <= S_S2;
                end

                S_S2: begin
                    tx_axis_tvalid <= 1;
                    tx_axis_tdata  <= sample_rd_data[7:0];
                    if (tx_axis_tready) begin
                        // Advance to next channel/sample
                        if (ch_idx == num_channels - 1) begin
                            ch_idx <= 0;
                            if (sample_idx == samples_per_packet - 1) begin
                                tx_axis_tlast <= 1;
                                state         <= S_DONE;
                                packets_sent  <= packets_sent + 1;
                                seq_num       <= seq_num + 1;
                            end else begin
                                sample_idx     <= sample_idx + 1;
                                sample_rd_addr <= sample_rd_addr + 1;
                                sample_rd_en   <= 1;
                                state          <= S_FETCH;
                            end
                        end else begin
                            ch_idx         <= ch_idx + 1;
                            sample_rd_addr <= sample_rd_addr + 1;
                            sample_rd_en   <= 1;
                            state          <= S_FETCH;
                        end
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
