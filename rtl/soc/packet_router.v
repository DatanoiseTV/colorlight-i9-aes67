// SPDX-License-Identifier: MIT
// Packet Router / Classifier - CUT-THROUGH
//
// Inspects the first 38 bytes of every incoming Ethernet frame to make
// a routing decision, then streams subsequent bytes directly to the
// chosen sink with no per-frame buffering. The first 42 bytes (Eth+IP+UDP)
// are mirrored from a tiny ring buffer once the decision is committed,
// so a CPU sink sees a complete frame.
//
// Latency vs the previous store-and-forward router:
//   - Old: ~12 µs (1518-byte frame at 1 Gbps) per RTP packet
//   - New: ~336 ns (42-byte header + sink fan-out)
//
// Routing rules:
//   - PTP        : EtherType 0x88F7 OR UDP dst port 319/320  -> ptp_rx (UDP payload)
//   - RTP        : UDP dst port == rtp_port AND dst IP == rtp_mcast_ip -> rtp_rx (UDP payload)
//   - everything : -> cpu_rx (full Ethernet frame)
//
// Frames with CRC errors are dropped at tlast.

module packet_router (
    input  wire        clk,
    input  wire        rst,

    // From MAC RX
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

    // To CPU netif (full Ethernet frame)
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

    // Always ready - the sinks must keep up with line rate
    assign mac_rx_tready = 1'b1;

    localparam UDP_PAYLOAD_OFFSET = 11'd42;
    localparam HEADER_REPLAY_LEN  = 6'd42;

    // ---- Header field accumulators ----
    reg [10:0] byte_idx;
    reg [15:0] f_ethertype;
    reg [7:0]  f_proto;
    reg [15:0] f_dst_port;
    reg [31:0] f_dst_ip;

    // Decision
    localparam [2:0]
        D_NONE = 3'd0,
        D_PTP  = 3'd1,
        D_RTP  = 3'd2,
        D_CPU  = 3'd3,
        D_DROP = 3'd4;
    reg [2:0] dest;

    // CPU header replay buffer (only used for CPU path)
    reg [7:0] cpu_replay [0:41];
    reg [5:0] replay_idx;
    reg       replay_active;

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            byte_idx       <= 0;
            f_ethertype    <= 0;
            f_proto        <= 0;
            f_dst_port     <= 0;
            f_dst_ip       <= 0;
            dest           <= D_NONE;
            replay_idx     <= 0;
            replay_active  <= 0;
            ptp_rx_tdata   <= 0;
            ptp_rx_tvalid  <= 0;
            ptp_rx_tlast   <= 0;
            rtp_rx_tdata   <= 0;
            rtp_rx_tvalid  <= 0;
            rtp_rx_tlast   <= 0;
            cpu_rx_tdata   <= 0;
            cpu_rx_tvalid  <= 0;
            cpu_rx_tlast   <= 0;
        end else begin
            ptp_rx_tvalid <= 0;
            ptp_rx_tlast  <= 0;
            rtp_rx_tvalid <= 0;
            rtp_rx_tlast  <= 0;
            cpu_rx_tvalid <= 0;
            cpu_rx_tlast  <= 0;

            // ---- Replay header bytes for CPU path (one per cycle) ----
            if (replay_active) begin
                cpu_rx_tdata  <= cpu_replay[replay_idx];
                cpu_rx_tvalid <= 1;
                replay_idx    <= replay_idx + 1;
                if (replay_idx == HEADER_REPLAY_LEN - 1)
                    replay_active <= 0;
            end

            if (mac_rx_tvalid) begin
                // Buffer header bytes for potential CPU replay
                if (byte_idx < {5'd0, HEADER_REPLAY_LEN})
                    cpu_replay[byte_idx[5:0]] <= mac_rx_tdata;

                // Capture key header fields
                case (byte_idx)
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

                // ---- Decision after byte 37 (UDP dst port lo) ----
                if (byte_idx == 11'd37) begin
                    if (f_ethertype == 16'h88F7) begin
                        dest <= D_PTP;
                    end else if (f_ethertype == 16'h0800 && f_proto == 8'd17) begin
                        // Use the just-arrived byte (low byte of dst port)
                        if (({f_dst_port[15:8], mac_rx_tdata} == 16'd319) ||
                            ({f_dst_port[15:8], mac_rx_tdata} == 16'd320)) begin
                            dest <= D_PTP;
                        end else if (({f_dst_port[15:8], mac_rx_tdata} == rtp_port) &&
                                     (f_dst_ip == rtp_mcast_ip)) begin
                            dest <= D_RTP;
                        end else begin
                            dest          <= D_CPU;
                            replay_active <= 1;
                            replay_idx    <= 0;
                        end
                    end else begin
                        dest          <= D_CPU;
                        replay_active <= 1;
                        replay_idx    <= 0;
                    end
                end

                // ---- Stream from byte 42+ to the chosen sink ----
                if (byte_idx >= UDP_PAYLOAD_OFFSET) begin
                    case (dest)
                        D_PTP: begin
                            ptp_rx_tdata  <= mac_rx_tdata;
                            ptp_rx_tvalid <= 1;
                            ptp_rx_tlast  <= mac_rx_tlast & ~mac_rx_tuser;
                        end
                        D_RTP: begin
                            rtp_rx_tdata  <= mac_rx_tdata;
                            rtp_rx_tvalid <= 1;
                            rtp_rx_tlast  <= mac_rx_tlast & ~mac_rx_tuser;
                        end
                        D_CPU: if (!replay_active) begin
                            cpu_rx_tdata  <= mac_rx_tdata;
                            cpu_rx_tvalid <= 1;
                            cpu_rx_tlast  <= mac_rx_tlast & ~mac_rx_tuser;
                        end
                        default: ;
                    endcase
                end

                byte_idx <= byte_idx + 1;
                if (mac_rx_tlast) begin
                    byte_idx    <= 0;
                    dest        <= D_NONE;
                    f_ethertype <= 0;
                    f_proto     <= 0;
                    f_dst_port  <= 0;
                    f_dst_ip    <= 0;
                end
            end
        end
    end

endmodule
