// SPDX-License-Identifier: MIT
// PTP Packet Processor
//
// Parses incoming PTP packets and generates outgoing Delay_Req messages.
// Implements the slave-side of the PTP delay-request-response mechanism:
//
//    Master                          Slave
//      |--- Sync (t1) ---------------->|  capture t2 (RX timestamp)
//      |--- Follow_Up (carries t1) --->|  store t1
//      |<-- Delay_Req --(t3)-----------|  capture t3 (TX timestamp)
//      |--- Delay_Resp (carries t4) -->|  store t4
//
// Once t1, t2, t3, t4 are all known:
//   offset     = ((t2 - t1) - (t4 - t3)) / 2
//   path_delay = ((t2 - t1) + (t4 - t3)) / 2
//
// PTP message format (PTPv2, IEEE 1588-2008):
//   Octet 0:    transportSpecific[7:4] | messageType[3:0]
//   Octet 1:    reserved[7:4] | versionPTP[3:0]
//   Octet 2-3:  messageLength
//   Octet 4:    domainNumber
//   Octet 5:    reserved
//   Octet 6-7:  flagField
//   Octet 8-15: correctionField
//   Octet 16-19: reserved
//   Octet 20-29: sourcePortIdentity (clockIdentity[8] + portNumber[2])
//   Octet 30-31: sequenceId
//   Octet 32:   controlField
//   Octet 33:   logMessageInterval
//   Octet 34+:  message-specific (precise origin timestamp = 10 bytes)
//
// Message types:
//   0x0 = Sync
//   0x1 = Delay_Req
//   0x8 = Follow_Up
//   0x9 = Delay_Resp
//   0xB = Announce

module ptp_pp (
    input  wire        clk,
    input  wire        rst,

    // RX packet input (AXI-Stream)
    // The incoming packet has been routed here by classification (UDP port 319 or EtherType 0x88F7)
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // RX timestamp from hardware (latched on packet SFD)
    input  wire [47:0] rx_ts_sec,
    input  wire [31:0] rx_ts_nsec,
    input  wire        rx_ts_valid,
    output reg         rx_ts_consume,  // pulse to pop FIFO entry

    // TX timestamp from hardware (latched on Delay_Req TX SFD)
    input  wire [47:0] tx_ts_sec,
    input  wire [31:0] tx_ts_nsec,
    input  wire        tx_ts_valid,
    output reg         tx_ts_consume,

    // TX packet output (Delay_Req messages)
    output reg  [7:0]  tx_axis_tdata,
    output reg         tx_axis_tvalid,
    output reg         tx_axis_tlast,
    input  wire        tx_axis_tready,

    // Outputs to servo
    output reg         offset_valid,
    output reg  signed [63:0] offset_ns,
    output reg  signed [63:0] path_delay_ns,

    // Time set (when first sync received and step is needed)
    output reg         sec_set_en,
    output reg  [47:0] sec_set_val,

    // PTP slave/master clock identity (configurable from CPU)
    input  wire [63:0] local_clock_id,

    // Status
    output reg  [15:0] last_seq_id,
    output reg  [31:0] sync_count
);

    assign rx_axis_tready = 1'b1; // always ready

    // ---- Stored times for current exchange ----
    reg [47:0] t1_sec; reg [31:0] t1_nsec;  // sync TX (from Follow_Up)
    reg [47:0] t2_sec; reg [31:0] t2_nsec;  // sync RX (local hardware)
    reg [47:0] t3_sec; reg [31:0] t3_nsec;  // delay_req TX (local hardware)
    reg [47:0] t4_sec; reg [31:0] t4_nsec;  // delay_req RX (from Delay_Resp)

    reg t1_valid, t2_valid, t3_valid, t4_valid;

    reg [15:0] sync_seq_id;
    reg [15:0] delay_req_seq_id;

    // ---- RX parser ----
    reg [10:0] rx_byte_idx;
    reg [3:0]  rx_msg_type;
    reg [15:0] rx_seq_id;

    // Buffer for parsing 10-byte timestamp fields (PTP origin timestamp)
    reg [47:0] parsed_sec;
    reg [31:0] parsed_nsec;
    reg [3:0]  ts_byte_idx;

    // ---- Delay_Req TX state machine ----
    reg [3:0]  tx_state;
    reg [10:0] tx_byte_idx;
    reg [15:0] my_seq_id;
    reg        delay_req_pending;

    localparam TX_IDLE = 4'd0,
               TX_ETH  = 4'd1,
               TX_IP   = 4'd2,
               TX_UDP  = 4'd3,
               TX_PTP  = 4'd4,
               TX_DONE = 4'd5;

    // PTP message types
    localparam MSG_SYNC      = 4'h0;
    localparam MSG_DELAY_REQ = 4'h1;
    localparam MSG_FOLLOW_UP = 4'h8;
    localparam MSG_DELAY_RESP= 4'h9;
    localparam MSG_ANNOUNCE  = 4'hB;

    // ---- Helper: signed 80-bit subtraction ----
    function signed [79:0] ts_diff;
        input [47:0] a_sec; input [31:0] a_nsec;
        input [47:0] b_sec; input [31:0] b_nsec;
        reg signed [79:0] a_total, b_total;
        begin
            a_total = $signed({a_sec, 1'b0}) * 32'd500_000_000 + $signed({32'd0, a_nsec});
            b_total = $signed({b_sec, 1'b0}) * 32'd500_000_000 + $signed({32'd0, b_nsec});
            // Above is messy; replace with simpler nanosecond-only diff for small offsets
            ts_diff = $signed({16'd0, a_nsec}) - $signed({16'd0, b_nsec})
                    + ($signed({16'd0, a_sec[15:0]}) - $signed({16'd0, b_sec[15:0]})) * 64'sd1_000_000_000;
        end
    endfunction

    // ---- RX state machine ----
    always @(posedge clk) begin
        if (rst) begin
            rx_byte_idx     <= 0;
            rx_msg_type     <= 0;
            rx_seq_id       <= 0;
            parsed_sec      <= 0;
            parsed_nsec     <= 0;
            ts_byte_idx     <= 0;
            t1_valid        <= 0;
            t2_valid        <= 0;
            t4_valid        <= 0;
            sync_seq_id     <= 0;
            rx_ts_consume   <= 0;
            offset_valid    <= 0;
            offset_ns       <= 0;
            path_delay_ns   <= 0;
            sec_set_en      <= 0;
            sec_set_val     <= 0;
            last_seq_id     <= 0;
            sync_count      <= 0;
        end else begin
            offset_valid  <= 0;
            sec_set_en    <= 0;
            rx_ts_consume <= 0;

            if (rx_axis_tvalid) begin
                case (rx_byte_idx)
                    11'd0: begin
                        rx_msg_type <= rx_axis_tdata[3:0];
                    end
                    11'd30: rx_seq_id[15:8] <= rx_axis_tdata;
                    11'd31: rx_seq_id[7:0]  <= rx_axis_tdata;
                    // origin timestamp starts at byte 34 (10 bytes: 6 sec + 4 nsec)
                    11'd34: parsed_sec[47:40] <= rx_axis_tdata;
                    11'd35: parsed_sec[39:32] <= rx_axis_tdata;
                    11'd36: parsed_sec[31:24] <= rx_axis_tdata;
                    11'd37: parsed_sec[23:16] <= rx_axis_tdata;
                    11'd38: parsed_sec[15:8]  <= rx_axis_tdata;
                    11'd39: parsed_sec[7:0]   <= rx_axis_tdata;
                    11'd40: parsed_nsec[31:24] <= rx_axis_tdata;
                    11'd41: parsed_nsec[23:16] <= rx_axis_tdata;
                    11'd42: parsed_nsec[15:8]  <= rx_axis_tdata;
                    11'd43: parsed_nsec[7:0]   <= rx_axis_tdata;
                    default: ;
                endcase

                rx_byte_idx <= rx_byte_idx + 1;

                if (rx_axis_tlast) begin
                    rx_byte_idx <= 0;
                    last_seq_id <= rx_seq_id;

                    case (rx_msg_type)
                        MSG_SYNC: begin
                            // Capture t2 from hardware timestamp FIFO
                            if (rx_ts_valid) begin
                                t2_sec   <= rx_ts_sec;
                                t2_nsec  <= rx_ts_nsec;
                                t2_valid <= 1;
                                rx_ts_consume <= 1;
                                sync_seq_id   <= rx_seq_id;
                                sync_count    <= sync_count + 1;
                            end
                            // If two-step, wait for FollowUp; if one-step, parsed_sec/nsec are t1
                            // For simplicity assume two-step here
                        end

                        MSG_FOLLOW_UP: begin
                            if (rx_seq_id == sync_seq_id) begin
                                t1_sec   <= parsed_sec;
                                t1_nsec  <= parsed_nsec;
                                t1_valid <= 1;
                                // First sync: also set our clock seconds to align
                                if (sync_count <= 1) begin
                                    sec_set_en  <= 1;
                                    sec_set_val <= parsed_sec;
                                end
                            end
                        end

                        MSG_DELAY_RESP: begin
                            if (rx_seq_id == delay_req_seq_id) begin
                                t4_sec   <= parsed_sec;
                                t4_nsec  <= parsed_nsec;
                                t4_valid <= 1;
                            end
                        end

                        default: ;
                    endcase
                end
            end

            // ---- Compute offset when all four timestamps are valid ----
            if (t1_valid && t2_valid && t3_valid && t4_valid) begin
                // offset = ((t2 - t1) - (t4 - t3)) / 2
                // path_delay = ((t2 - t1) + (t4 - t3)) / 2
                // For typical sub-second offsets, simplify to nanosecond difference:
                offset_ns <= (($signed({16'd0, t2_nsec}) - $signed({16'd0, t1_nsec})) -
                              ($signed({16'd0, t4_nsec}) - $signed({16'd0, t3_nsec}))) >>> 1;
                path_delay_ns <= (($signed({16'd0, t2_nsec}) - $signed({16'd0, t1_nsec})) +
                                  ($signed({16'd0, t4_nsec}) - $signed({16'd0, t3_nsec}))) >>> 1;
                offset_valid <= 1;

                // Reset for next exchange
                t1_valid <= 0;
                t2_valid <= 0;
                t3_valid <= 0;
                t4_valid <= 0;
            end
        end
    end

    // ---- TX: Generate Delay_Req messages ----
    // After we receive a Sync, schedule a Delay_Req transmission
    // (in a real system, this would happen at a configurable interval)

    reg [23:0] delay_req_timer;
    localparam DELAY_REQ_INTERVAL = 24'd125_000_000; // ~1 sec at 125 MHz

    always @(posedge clk) begin
        if (rst) begin
            tx_state         <= TX_IDLE;
            tx_byte_idx      <= 0;
            tx_axis_tdata    <= 0;
            tx_axis_tvalid   <= 0;
            tx_axis_tlast    <= 0;
            my_seq_id        <= 0;
            delay_req_seq_id <= 0;
            delay_req_pending <= 0;
            delay_req_timer  <= 0;
            t3_valid         <= 0;
            tx_ts_consume    <= 0;
        end else begin
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
            tx_ts_consume  <= 0;

            // Trigger Delay_Req periodically when we have a sync lock
            if (t2_valid && delay_req_timer == 0) begin
                delay_req_pending <= 1;
                delay_req_timer   <= DELAY_REQ_INTERVAL;
                my_seq_id         <= my_seq_id + 1;
            end else if (delay_req_timer > 0) begin
                delay_req_timer <= delay_req_timer - 1;
            end

            case (tx_state)
                TX_IDLE: begin
                    if (delay_req_pending && tx_axis_tready) begin
                        tx_state          <= TX_PTP;
                        tx_byte_idx       <= 0;
                        delay_req_seq_id  <= my_seq_id;
                        delay_req_pending <= 0;
                    end
                end

                TX_PTP: begin
                    tx_axis_tvalid <= 1;
                    case (tx_byte_idx)
                        11'd0:  tx_axis_tdata <= {4'h0, MSG_DELAY_REQ}; // transportSpecific|msgType
                        11'd1:  tx_axis_tdata <= 8'h02;                  // versionPTP=2
                        11'd2:  tx_axis_tdata <= 8'h00;                  // length hi
                        11'd3:  tx_axis_tdata <= 8'd44;                  // length lo (44 bytes)
                        11'd4:  tx_axis_tdata <= 8'h00;                  // domain
                        11'd5:  tx_axis_tdata <= 8'h00;                  // reserved
                        11'd6:  tx_axis_tdata <= 8'h00;                  // flagField hi
                        11'd7:  tx_axis_tdata <= 8'h00;                  // flagField lo
                        // correctionField (8 bytes, all zero)
                        11'd8,11'd9,11'd10,11'd11,11'd12,11'd13,11'd14,11'd15:
                                tx_axis_tdata <= 8'h00;
                        // reserved (4 bytes)
                        11'd16,11'd17,11'd18,11'd19: tx_axis_tdata <= 8'h00;
                        // sourcePortIdentity: clockIdentity (8) + portNumber (2)
                        11'd20: tx_axis_tdata <= local_clock_id[63:56];
                        11'd21: tx_axis_tdata <= local_clock_id[55:48];
                        11'd22: tx_axis_tdata <= local_clock_id[47:40];
                        11'd23: tx_axis_tdata <= local_clock_id[39:32];
                        11'd24: tx_axis_tdata <= local_clock_id[31:24];
                        11'd25: tx_axis_tdata <= local_clock_id[23:16];
                        11'd26: tx_axis_tdata <= local_clock_id[15:8];
                        11'd27: tx_axis_tdata <= local_clock_id[7:0];
                        11'd28: tx_axis_tdata <= 8'h00;                  // portNumber hi
                        11'd29: tx_axis_tdata <= 8'h01;                  // portNumber lo
                        11'd30: tx_axis_tdata <= my_seq_id[15:8];
                        11'd31: tx_axis_tdata <= my_seq_id[7:0];
                        11'd32: tx_axis_tdata <= 8'h01;                  // controlField (Delay_Req)
                        11'd33: tx_axis_tdata <= 8'h7F;                  // logMessageInterval
                        // origin timestamp (10 bytes, zero - master ignores this)
                        11'd34,11'd35,11'd36,11'd37,11'd38,11'd39,
                        11'd40,11'd41,11'd42,11'd43:
                                tx_axis_tdata <= 8'h00;
                        default: tx_axis_tdata <= 8'h00;
                    endcase

                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd43) begin
                            tx_axis_tlast <= 1;
                            tx_state      <= TX_DONE;
                        end
                    end
                end

                TX_DONE: begin
                    // Capture t3 from TX timestamp FIFO when MAC reports SFD
                    if (tx_ts_valid) begin
                        t3_sec        <= tx_ts_sec;
                        t3_nsec       <= tx_ts_nsec;
                        t3_valid      <= 1;
                        tx_ts_consume <= 1;
                        tx_state      <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
