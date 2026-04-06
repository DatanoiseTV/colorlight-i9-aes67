// SPDX-License-Identifier: MIT
// Packet Router / Classifier
//
// RX: inspects incoming Ethernet frames at line rate and routes them:
//   1. PTP   (UDP port 319/320 OR EtherType 0x88F7)            → ptp_rx
//   2. RTP   (UDP port 5004, multicast)                        → rtp_rx
//   3. Other (ARP, ICMP, DHCP, mDNS, HTTP, SAP, ...)           → cpu_rx
//
// TX: PTP/RTP TX paths come in as Ethernet frames already wrapped by their
// respective tx_udp_wrapper instances. CPU TX is fully formatted by the
// firmware. The TX arbiter at the top level mixes the three streams.
//
// Notes on PTP/RTP TX:
//   - The PTP and RTP engines emit UDP payloads only
//   - Wrapping with Ethernet/IP/UDP headers is done by tx_udp_wrapper
//     instances OUTSIDE this module (in aes67_top), so this router is
//     purely an RX classifier.

module packet_router (
    input  wire        clk,
    input  wire        rst,

    // From MAC RX (full Ethernet frames)
    input  wire [7:0]  mac_rx_tdata,
    input  wire        mac_rx_tvalid,
    input  wire        mac_rx_tlast,
    input  wire        mac_rx_tuser,    // CRC error
    output wire        mac_rx_tready,

    // To PTP processor (UDP payload only)
    output reg  [7:0]  ptp_rx_tdata,
    output reg         ptp_rx_tvalid,
    output reg         ptp_rx_tlast,
    input  wire        ptp_rx_tready,

    // To RTP engine (UDP payload only)
    output reg  [7:0]  rtp_rx_tdata,
    output reg         rtp_rx_tvalid,
    output reg         rtp_rx_tlast,
    input  wire        rtp_rx_tready,

    // To CPU (full Ethernet frame, replayed from header buffer)
    output reg  [7:0]  cpu_rx_tdata,
    output reg         cpu_rx_tvalid,
    output reg         cpu_rx_tlast,
    input  wire        cpu_rx_tready,

    // Configuration
    input  wire [47:0] local_mac,
    input  wire [31:0] local_ip,
    input  wire [31:0] rtp_mcast_ip,
    input  wire [15:0] rtp_port
);

    assign mac_rx_tready = 1'b1;

    // ---- Decision pipeline ----
    // We need to know the destination by byte ~38 (UDP dst port hi) before
    // we can confidently route. We buffer the first 64 bytes of every frame
    // and replay them to whichever destination we choose.

    localparam BUF_BYTES = 128;
    reg [7:0]  hdr_buf [0:BUF_BYTES-1];
    reg [10:0] hdr_wr;
    reg [10:0] hdr_rd;
    reg        in_frame;
    reg [10:0] frame_len;       // total bytes in current frame

    // Header fields
    reg [15:0] f_ethertype;
    reg [7:0]  f_proto;
    reg [15:0] f_dst_port;
    reg [31:0] f_dst_ip;
    reg [47:0] f_dst_mac;

    // Routing decision
    localparam [2:0]
        DEST_NONE = 3'd0,
        DEST_PTP  = 3'd1,
        DEST_RTP  = 3'd2,
        DEST_CPU  = 3'd3,
        DEST_DROP = 3'd4;

    reg [2:0] dest;
    reg       dest_valid;

    // ---- Capture incoming frame ----
    always @(posedge clk) begin
        if (rst) begin
            hdr_wr      <= 0;
            in_frame    <= 0;
            frame_len   <= 0;
            f_ethertype <= 0;
            f_proto     <= 0;
            f_dst_port  <= 0;
            f_dst_ip    <= 0;
            f_dst_mac   <= 0;
        end else begin
            if (mac_rx_tvalid) begin
                if (hdr_wr < BUF_BYTES)
                    hdr_buf[hdr_wr] <= mac_rx_tdata;
                hdr_wr   <= hdr_wr + 1;
                in_frame <= 1;

                case (hdr_wr)
                    11'd0:  f_dst_mac[47:40] <= mac_rx_tdata;
                    11'd1:  f_dst_mac[39:32] <= mac_rx_tdata;
                    11'd2:  f_dst_mac[31:24] <= mac_rx_tdata;
                    11'd3:  f_dst_mac[23:16] <= mac_rx_tdata;
                    11'd4:  f_dst_mac[15:8]  <= mac_rx_tdata;
                    11'd5:  f_dst_mac[7:0]   <= mac_rx_tdata;
                    11'd12: f_ethertype[15:8] <= mac_rx_tdata;
                    11'd13: f_ethertype[7:0]  <= mac_rx_tdata;
                    11'd23: f_proto           <= mac_rx_tdata;
                    11'd30: f_dst_ip[31:24]   <= mac_rx_tdata;
                    11'd31: f_dst_ip[23:16]   <= mac_rx_tdata;
                    11'd32: f_dst_ip[15:8]    <= mac_rx_tdata;
                    11'd33: f_dst_ip[7:0]     <= mac_rx_tdata;
                    11'd36: f_dst_port[15:8]  <= mac_rx_tdata;
                    11'd37: f_dst_port[7:0]   <= mac_rx_tdata;
                    default: ;
                endcase

                if (mac_rx_tlast) begin
                    frame_len <= hdr_wr + 1;
                    in_frame  <= 0;
                end
            end
        end
    end

    // ---- Decision: when frame ends, classify and start replay ----
    always @(posedge clk) begin
        if (rst) begin
            dest        <= DEST_NONE;
            dest_valid  <= 0;
        end else begin
            if (mac_rx_tvalid && mac_rx_tlast) begin
                if (mac_rx_tuser) begin
                    dest <= DEST_DROP;
                end
                // Layer-2 PTP
                else if (f_ethertype == 16'h88F7) begin
                    dest <= DEST_PTP;
                end
                // IPv4
                else if (f_ethertype == 16'h0800 && f_proto == 8'd17) begin
                    if (f_dst_port == 16'd319 || f_dst_port == 16'd320)
                        dest <= DEST_PTP;
                    else if (f_dst_port == rtp_port && f_dst_ip == rtp_mcast_ip)
                        dest <= DEST_RTP;
                    else
                        dest <= DEST_CPU;
                end
                else begin
                    dest <= DEST_CPU;
                end
                dest_valid <= 1;
            end
        end
    end

    // ---- Replay state machine ----
    localparam [2:0]
        R_IDLE  = 3'd0,
        R_PTP   = 3'd1,
        R_RTP   = 3'd2,
        R_CPU   = 3'd3,
        R_DROP  = 3'd4;

    reg [2:0] rstate;
    reg [10:0] replay_idx;
    // PTP/RTP only get the UDP payload, so they start at byte 42 (Eth14+IP20+UDP8)
    // CPU gets the full Ethernet frame (start at byte 0)
    localparam UDP_PAYLOAD_OFFSET = 11'd42;

    always @(posedge clk) begin
        if (rst) begin
            rstate         <= R_IDLE;
            replay_idx     <= 0;
            ptp_rx_tdata   <= 0;
            ptp_rx_tvalid  <= 0;
            ptp_rx_tlast   <= 0;
            rtp_rx_tdata   <= 0;
            rtp_rx_tvalid  <= 0;
            rtp_rx_tlast   <= 0;
            cpu_rx_tdata   <= 0;
            cpu_rx_tvalid  <= 0;
            cpu_rx_tlast   <= 0;
            hdr_rd         <= 0;
        end else begin
            ptp_rx_tvalid <= 0;
            ptp_rx_tlast  <= 0;
            rtp_rx_tvalid <= 0;
            rtp_rx_tlast  <= 0;
            cpu_rx_tvalid <= 0;
            cpu_rx_tlast  <= 0;

            case (rstate)
                R_IDLE: begin
                    if (dest_valid) begin
                        case (dest)
                            DEST_PTP:  begin rstate <= R_PTP;  replay_idx <= UDP_PAYLOAD_OFFSET; end
                            DEST_RTP:  begin rstate <= R_RTP;  replay_idx <= UDP_PAYLOAD_OFFSET; end
                            DEST_CPU:  begin rstate <= R_CPU;  replay_idx <= 0; end
                            DEST_DROP: begin rstate <= R_IDLE; end
                            default:   rstate <= R_IDLE;
                        endcase
                    end
                end

                R_PTP: begin
                    if (ptp_rx_tready && replay_idx < frame_len) begin
                        ptp_rx_tdata  <= hdr_buf[replay_idx[10:0]];
                        ptp_rx_tvalid <= 1;
                        ptp_rx_tlast  <= (replay_idx == frame_len - 1);
                        if (replay_idx == frame_len - 1) begin
                            rstate <= R_IDLE;
                        end
                        replay_idx <= replay_idx + 1;
                    end
                end

                R_RTP: begin
                    if (rtp_rx_tready && replay_idx < frame_len) begin
                        rtp_rx_tdata  <= hdr_buf[replay_idx[10:0]];
                        rtp_rx_tvalid <= 1;
                        rtp_rx_tlast  <= (replay_idx == frame_len - 1);
                        if (replay_idx == frame_len - 1) rstate <= R_IDLE;
                        replay_idx <= replay_idx + 1;
                    end
                end

                R_CPU: begin
                    if (cpu_rx_tready && replay_idx < frame_len) begin
                        cpu_rx_tdata  <= hdr_buf[replay_idx[10:0]];
                        cpu_rx_tvalid <= 1;
                        cpu_rx_tlast  <= (replay_idx == frame_len - 1);
                        if (replay_idx == frame_len - 1) rstate <= R_IDLE;
                        replay_idx <= replay_idx + 1;
                    end
                end

                default: rstate <= R_IDLE;
            endcase

            // Reset header capture for the next frame after replay starts
            if (rstate != R_IDLE && replay_idx == frame_len - 1) begin
                hdr_rd <= 0;
            end
        end
    end

endmodule
