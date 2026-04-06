// SPDX-License-Identifier: MIT
// Generic UDP/IP/Ethernet TX wrapper
//
// Wraps an arbitrary AXI-Stream UDP payload in a complete Ethernet frame:
//   [14 byte Ethernet header] [20 byte IPv4 header] [8 byte UDP header] [payload]
//
// The caller asserts `start` together with the payload length and the dest
// addressing info; the wrapper then drives the MAC TX stream until done.
//
// IPv4 checksum is computed on-the-fly. UDP checksum is set to 0 (legal in
// IPv4 per RFC 768).
//
// The payload source is a small AXI-Stream input.

module tx_udp_wrapper (
    input  wire        clk,
    input  wire        rst,

    // Request from upper layer
    input  wire        start,
    input  wire [15:0] payload_len,    // bytes of UDP payload (NOT including headers)
    input  wire [47:0] dst_mac,
    input  wire [47:0] src_mac,
    input  wire [31:0] dst_ip,
    input  wire [31:0] src_ip,
    input  wire [15:0] dst_port,
    input  wire [15:0] src_port,
    output reg         busy,

    // Payload input (AXI-Stream)
    input  wire [7:0]  pl_axis_tdata,
    input  wire        pl_axis_tvalid,
    input  wire        pl_axis_tlast,
    output reg         pl_axis_tready,

    // MAC TX output
    output reg  [7:0]  mac_axis_tdata,
    output reg         mac_axis_tvalid,
    output reg         mac_axis_tlast,
    input  wire        mac_axis_tready
);

    localparam [3:0]
        S_IDLE      = 4'd0,
        S_CSUM_INIT = 4'd1,
        S_CSUM_FEED = 4'd2,
        S_CSUM_WAIT = 4'd3,
        S_TX_HDR    = 4'd4,
        S_TX_PL     = 4'd5,
        S_TX_DONE   = 4'd6;

    reg [3:0]  state;
    reg [5:0]  hdr_idx;
    reg [15:0] total_ip_len;     // 20 + 8 + payload
    reg [15:0] udp_len;          // 8 + payload
    reg [15:0] ip_id;            // increments per packet

    // IP header field bytes (20 bytes total) - registered after start
    reg [7:0]  ip_hdr [0:19];

    // Checksum calculator
    reg        csum_clear;
    reg        csum_word_valid;
    reg [15:0] csum_word;
    reg        csum_done;
    wire [15:0] csum_out;
    reg [3:0]  csum_idx;

    ip_checksum u_csum (
        .clk        (clk),
        .rst        (rst),
        .clear      (csum_clear),
        .word_valid (csum_word_valid),
        .word       (csum_word),
        .done       (csum_done),
        .checksum   (csum_out)
    );

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            state           <= S_IDLE;
            busy            <= 0;
            hdr_idx         <= 0;
            mac_axis_tdata  <= 0;
            mac_axis_tvalid <= 0;
            mac_axis_tlast  <= 0;
            pl_axis_tready  <= 0;
            csum_clear      <= 0;
            csum_word_valid <= 0;
            csum_word       <= 0;
            csum_done       <= 0;
            csum_idx        <= 0;
            ip_id           <= 16'd1;
            total_ip_len    <= 0;
            udp_len         <= 0;
            for (i = 0; i < 20; i = i + 1) ip_hdr[i] <= 0;
        end else begin
            mac_axis_tvalid <= 0;
            mac_axis_tlast  <= 0;
            pl_axis_tready  <= 0;
            csum_word_valid <= 0;
            csum_done       <= 0;

            case (state)
                S_IDLE: begin
                    busy       <= 0;
                    csum_clear <= 1;
                    if (start) begin
                        busy         <= 1;
                        total_ip_len <= 16'd28 + payload_len;
                        udp_len      <= 16'd8  + payload_len;
                        // Build IP header (constants + computed length/IPs)
                        ip_hdr[0]  <= 8'h45;                      // Version=4, IHL=5
                        ip_hdr[1]  <= 8'h00;                      // DSCP/ECN
                        ip_hdr[2]  <= (16'd28 + payload_len) >> 8;
                        ip_hdr[3]  <= (16'd28 + payload_len) & 16'hFF;
                        ip_hdr[4]  <= ip_id[15:8];
                        ip_hdr[5]  <= ip_id[7:0];
                        ip_hdr[6]  <= 8'h00;                      // Flags / frag hi
                        ip_hdr[7]  <= 8'h00;                      // Frag lo
                        ip_hdr[8]  <= 8'd64;                      // TTL
                        ip_hdr[9]  <= 8'd17;                      // Protocol = UDP
                        ip_hdr[10] <= 8'h00;                      // Checksum hi (placeholder)
                        ip_hdr[11] <= 8'h00;                      // Checksum lo
                        ip_hdr[12] <= src_ip[31:24];
                        ip_hdr[13] <= src_ip[23:16];
                        ip_hdr[14] <= src_ip[15:8];
                        ip_hdr[15] <= src_ip[7:0];
                        ip_hdr[16] <= dst_ip[31:24];
                        ip_hdr[17] <= dst_ip[23:16];
                        ip_hdr[18] <= dst_ip[15:8];
                        ip_hdr[19] <= dst_ip[7:0];
                        ip_id      <= ip_id + 1;
                        state      <= S_CSUM_INIT;
                    end
                end

                S_CSUM_INIT: begin
                    csum_clear <= 0;
                    csum_idx   <= 0;
                    state      <= S_CSUM_FEED;
                end

                S_CSUM_FEED: begin
                    // Feed the IP header as 10 16-bit words
                    csum_word_valid <= 1;
                    case (csum_idx)
                        4'd0: csum_word <= {ip_hdr[0],  ip_hdr[1]};
                        4'd1: csum_word <= {ip_hdr[2],  ip_hdr[3]};
                        4'd2: csum_word <= {ip_hdr[4],  ip_hdr[5]};
                        4'd3: csum_word <= {ip_hdr[6],  ip_hdr[7]};
                        4'd4: csum_word <= {ip_hdr[8],  ip_hdr[9]};
                        4'd5: csum_word <= 16'h0000;            // checksum field placeholder
                        4'd6: csum_word <= {ip_hdr[12], ip_hdr[13]};
                        4'd7: csum_word <= {ip_hdr[14], ip_hdr[15]};
                        4'd8: csum_word <= {ip_hdr[16], ip_hdr[17]};
                        4'd9: csum_word <= {ip_hdr[18], ip_hdr[19]};
                        default: csum_word <= 16'h0000;
                    endcase
                    csum_idx <= csum_idx + 1;
                    if (csum_idx == 4'd9) begin
                        state <= S_CSUM_WAIT;
                    end
                end

                S_CSUM_WAIT: begin
                    csum_done <= 1;
                    // checksum is registered next cycle
                    ip_hdr[10] <= csum_out[15:8];
                    ip_hdr[11] <= csum_out[7:0];
                    state      <= S_TX_HDR;
                    hdr_idx    <= 0;
                end

                S_TX_HDR: begin
                    mac_axis_tvalid <= 1;
                    case (hdr_idx)
                        // Ethernet (14)
                        6'd0:  mac_axis_tdata <= dst_mac[47:40];
                        6'd1:  mac_axis_tdata <= dst_mac[39:32];
                        6'd2:  mac_axis_tdata <= dst_mac[31:24];
                        6'd3:  mac_axis_tdata <= dst_mac[23:16];
                        6'd4:  mac_axis_tdata <= dst_mac[15:8];
                        6'd5:  mac_axis_tdata <= dst_mac[7:0];
                        6'd6:  mac_axis_tdata <= src_mac[47:40];
                        6'd7:  mac_axis_tdata <= src_mac[39:32];
                        6'd8:  mac_axis_tdata <= src_mac[31:24];
                        6'd9:  mac_axis_tdata <= src_mac[23:16];
                        6'd10: mac_axis_tdata <= src_mac[15:8];
                        6'd11: mac_axis_tdata <= src_mac[7:0];
                        6'd12: mac_axis_tdata <= 8'h08;          // EtherType IPv4
                        6'd13: mac_axis_tdata <= 8'h00;
                        // IPv4 (20)
                        6'd14: mac_axis_tdata <= ip_hdr[0];
                        6'd15: mac_axis_tdata <= ip_hdr[1];
                        6'd16: mac_axis_tdata <= ip_hdr[2];
                        6'd17: mac_axis_tdata <= ip_hdr[3];
                        6'd18: mac_axis_tdata <= ip_hdr[4];
                        6'd19: mac_axis_tdata <= ip_hdr[5];
                        6'd20: mac_axis_tdata <= ip_hdr[6];
                        6'd21: mac_axis_tdata <= ip_hdr[7];
                        6'd22: mac_axis_tdata <= ip_hdr[8];
                        6'd23: mac_axis_tdata <= ip_hdr[9];
                        6'd24: mac_axis_tdata <= ip_hdr[10];
                        6'd25: mac_axis_tdata <= ip_hdr[11];
                        6'd26: mac_axis_tdata <= ip_hdr[12];
                        6'd27: mac_axis_tdata <= ip_hdr[13];
                        6'd28: mac_axis_tdata <= ip_hdr[14];
                        6'd29: mac_axis_tdata <= ip_hdr[15];
                        6'd30: mac_axis_tdata <= ip_hdr[16];
                        6'd31: mac_axis_tdata <= ip_hdr[17];
                        6'd32: mac_axis_tdata <= ip_hdr[18];
                        6'd33: mac_axis_tdata <= ip_hdr[19];
                        // UDP (8)
                        6'd34: mac_axis_tdata <= src_port[15:8];
                        6'd35: mac_axis_tdata <= src_port[7:0];
                        6'd36: mac_axis_tdata <= dst_port[15:8];
                        6'd37: mac_axis_tdata <= dst_port[7:0];
                        6'd38: mac_axis_tdata <= udp_len[15:8];
                        6'd39: mac_axis_tdata <= udp_len[7:0];
                        6'd40: mac_axis_tdata <= 8'h00;          // UDP checksum hi (0 = unused)
                        6'd41: mac_axis_tdata <= 8'h00;          // UDP checksum lo
                    endcase
                    if (mac_axis_tready) begin
                        if (hdr_idx == 6'd41)
                            state <= S_TX_PL;
                        hdr_idx <= hdr_idx + 1;
                    end
                end

                S_TX_PL: begin
                    pl_axis_tready  <= mac_axis_tready;
                    mac_axis_tdata  <= pl_axis_tdata;
                    mac_axis_tvalid <= pl_axis_tvalid;
                    mac_axis_tlast  <= pl_axis_tvalid & pl_axis_tlast;
                    if (pl_axis_tvalid && pl_axis_tlast && mac_axis_tready)
                        state <= S_TX_DONE;
                end

                S_TX_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
