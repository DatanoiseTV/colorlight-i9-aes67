// SPDX-License-Identifier: MIT
// RTP Receive Engine
//
// Parses incoming UDP/RTP packets, extracts L24 audio samples, writes to jitter buffer.
// Assumes the packet has been pre-stripped to start at the UDP payload (RTP header onward).
//
// RTP Header (12 bytes):
//   Octet 0:    V(2)|P(1)|X(1)|CC(4)
//   Octet 1:    M(1)|PT(7)
//   Octet 2-3:  Sequence Number
//   Octet 4-7:  Timestamp (RTP, in audio sample units)
//   Octet 8-11: SSRC
//   Octet 12+:  Payload (L24 audio samples)
//
// L24 packing: 3 bytes per sample, big-endian, channels interleaved per frame
// e.g. for stereo: [L_hi, L_mid, L_lo, R_hi, R_mid, R_lo, L_hi, ...]

module rtp_rx #(
    parameter NUM_CHANNELS = 8,        // max channels per stream
    parameter SAMPLE_WIDTH = 24
) (
    input  wire        clk,
    input  wire        rst,

    // RTP packet input (already stripped to UDP payload)
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // Stream config (from CPU)
    input  wire [31:0] expected_ssrc,
    input  wire [7:0]  expected_pt,
    input  wire [3:0]  num_channels,    // active channels (1-8)

    // Output to jitter buffer (one sample per channel per write)
    output reg  [SAMPLE_WIDTH-1:0] sample_data,
    output reg  [3:0]              sample_ch,
    output reg                     sample_valid,
    output reg                     packet_start, // pulse at start of packet
    output reg                     packet_end,   // pulse at end of packet
    output reg  [31:0]             rtp_timestamp,
    output reg  [15:0]             rtp_seq,

    // Status
    output reg  [31:0] packets_received,
    output reg  [31:0] packets_dropped,
    output reg  [31:0] seq_errors
);

    assign rx_axis_tready = 1'b1;

    localparam [3:0]
        S_HDR0     = 4'd0,
        S_HDR_M_PT = 4'd1,
        S_HDR_SEQ  = 4'd2,
        S_HDR_TS   = 4'd3,
        S_HDR_SSRC = 4'd4,
        S_PAYLOAD  = 4'd5,
        S_DROP     = 4'd6;

    reg [3:0]  state;
    reg [2:0]  byte_in_field;
    reg [1:0]  sample_byte;     // 0,1,2 for L24 byte position
    reg [3:0]  current_ch;
    reg [23:0] sample_accum;
    reg [15:0] last_seq;
    reg        first_packet;
    reg [7:0]  pt_field;
    reg [31:0] ssrc_field;
    reg [15:0] seq_field;
    reg [31:0] ts_field;

    always @(posedge clk) begin
        if (rst) begin
            state           <= S_HDR0;
            byte_in_field   <= 0;
            sample_byte     <= 0;
            current_ch      <= 0;
            sample_accum    <= 0;
            sample_data     <= 0;
            sample_ch       <= 0;
            sample_valid    <= 0;
            packet_start    <= 0;
            packet_end      <= 0;
            rtp_timestamp   <= 0;
            rtp_seq         <= 0;
            last_seq        <= 0;
            first_packet    <= 1;
            pt_field        <= 0;
            ssrc_field      <= 0;
            seq_field       <= 0;
            ts_field        <= 0;
            packets_received <= 0;
            packets_dropped  <= 0;
            seq_errors       <= 0;
        end else begin
            sample_valid <= 0;
            packet_start <= 0;
            packet_end   <= 0;

            if (rx_axis_tvalid) begin
                case (state)
                    S_HDR0: begin
                        // V/P/X/CC byte; check version = 2
                        if (rx_axis_tdata[7:6] == 2'b10) begin
                            state <= S_HDR_M_PT;
                            byte_in_field <= 0;
                        end else begin
                            state <= S_DROP;
                            packets_dropped <= packets_dropped + 1;
                        end
                    end

                    S_HDR_M_PT: begin
                        pt_field <= rx_axis_tdata;
                        state    <= S_HDR_SEQ;
                        byte_in_field <= 0;
                    end

                    S_HDR_SEQ: begin
                        if (byte_in_field == 0) begin
                            seq_field[15:8] <= rx_axis_tdata;
                            byte_in_field   <= 1;
                        end else begin
                            seq_field[7:0] <= rx_axis_tdata;
                            state          <= S_HDR_TS;
                            byte_in_field  <= 0;
                        end
                    end

                    S_HDR_TS: begin
                        case (byte_in_field)
                            0: ts_field[31:24] <= rx_axis_tdata;
                            1: ts_field[23:16] <= rx_axis_tdata;
                            2: ts_field[15:8]  <= rx_axis_tdata;
                            3: ts_field[7:0]   <= rx_axis_tdata;
                        endcase
                        if (byte_in_field == 3) begin
                            state         <= S_HDR_SSRC;
                            byte_in_field <= 0;
                        end else
                            byte_in_field <= byte_in_field + 1;
                    end

                    S_HDR_SSRC: begin
                        case (byte_in_field)
                            0: ssrc_field[31:24] <= rx_axis_tdata;
                            1: ssrc_field[23:16] <= rx_axis_tdata;
                            2: ssrc_field[15:8]  <= rx_axis_tdata;
                            3: ssrc_field[7:0]   <= rx_axis_tdata;
                        endcase
                        if (byte_in_field == 3) begin
                            // Validate SSRC and PT (allow 0 = wildcard)
                            if ((expected_ssrc == 32'h0 || ssrc_field == expected_ssrc) &&
                                (expected_pt == 8'h0 || (pt_field & 8'h7F) == expected_pt)) begin
                                state         <= S_PAYLOAD;
                                byte_in_field <= 0;
                                sample_byte   <= 0;
                                current_ch    <= 0;
                                packet_start  <= 1;
                                rtp_timestamp <= ts_field;
                                rtp_seq       <= seq_field;
                                packets_received <= packets_received + 1;
                                if (!first_packet && (seq_field != last_seq + 16'd1))
                                    seq_errors <= seq_errors + 1;
                                last_seq     <= seq_field;
                                first_packet <= 0;
                            end else begin
                                state           <= S_DROP;
                                packets_dropped <= packets_dropped + 1;
                            end
                        end else
                            byte_in_field <= byte_in_field + 1;
                    end

                    S_PAYLOAD: begin
                        case (sample_byte)
                            2'd0: sample_accum[23:16] <= rx_axis_tdata;
                            2'd1: sample_accum[15:8]  <= rx_axis_tdata;
                            2'd2: begin
                                sample_data <= {sample_accum[23:8], rx_axis_tdata};
                                sample_ch    <= current_ch;
                                sample_valid <= 1;
                                if (current_ch == num_channels - 1)
                                    current_ch <= 0;
                                else
                                    current_ch <= current_ch + 1;
                            end
                        endcase

                        if (sample_byte == 2'd2)
                            sample_byte <= 0;
                        else
                            sample_byte <= sample_byte + 1;

                        if (rx_axis_tlast) begin
                            packet_end <= 1;
                            state      <= S_HDR0;
                        end
                    end

                    S_DROP: begin
                        if (rx_axis_tlast)
                            state <= S_HDR0;
                    end

                    default: state <= S_HDR0;
                endcase
            end
        end
    end

endmodule
