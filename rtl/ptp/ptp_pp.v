// SPDX-License-Identifier: MIT
// PTP Packet Processor (IEEE 1588-2008)
//
// Implements the slave-side delay-request-response mechanism end-to-end
// in hardware:
//
//    Master                          Slave
//      |--- Sync (t1) ---------------->|  capture t2 (RX SFD timestamp)
//      |--- Follow_Up (carries t1) --->|  store t1, correctionField_S
//      |<-- Delay_Req --(t3)-----------|  capture t3 (TX SFD timestamp)
//      |--- Delay_Resp (carries t4) -->|  store t4, correctionField_R
//
// Per RFC 1588-2008 §11.3, the offset and mean path delay are:
//
//   offset = ((t2 - t1 - corrField_S) - (t4 - t3 - corrField_R)) / 2
//   delay  = ((t2 - t1 - corrField_S) + (t4 - t3 - corrField_R)) / 2
//
// where corrField_S is the Sync correctionField (from Follow_Up if
// twoStepFlag is set, else from the Sync itself) and corrField_R is the
// Delay_Resp correctionField. The correctionField is in units of
// (nanoseconds × 2^16) and is added by transparent clocks along the path.
//
// PTP message format (PTPv2):
//   Octet 0:    transportSpecific[7:4] | messageType[3:0]
//   Octet 1:    reserved[7:4] | versionPTP[3:0]
//   Octet 2-3:  messageLength
//   Octet 4:    domainNumber
//   Octet 5:    reserved
//   Octet 6:    flagField hi (twoStepFlag = bit 1 of low nibble = 0x02)
//   Octet 7:    flagField lo
//   Octet 8-15: correctionField (signed 64-bit, ns << 16)
//   Octet 16-19: reserved
//   Octet 20-29: sourcePortIdentity
//   Octet 30-31: sequenceId
//   Octet 32:   controlField
//   Octet 33:   logMessageInterval
//   Octet 34-43: originTimestamp (6-byte sec, 4-byte nsec)
//
// This processor sees only the UDP payload (router strips Eth/IP/UDP).

module ptp_pp (
    input  wire        clk,
    input  wire        rst,

    // PTP UDP payload from packet router
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // RX timestamp FIFO (sec/nsec, popped per Sync)
    input  wire [47:0] rx_ts_sec,
    input  wire [31:0] rx_ts_nsec,
    input  wire        rx_ts_valid,
    output reg         rx_ts_consume,

    // TX timestamp FIFO (popped per Delay_Req)
    input  wire [47:0] tx_ts_sec,
    input  wire [31:0] tx_ts_nsec,
    input  wire        tx_ts_valid,
    output reg         tx_ts_consume,

    // PTP UDP payload to TX wrapper (Delay_Req)
    output reg  [7:0]  tx_axis_tdata,
    output reg         tx_axis_tvalid,
    output reg         tx_axis_tlast,
    input  wire        tx_axis_tready,

    // Servo input
    output reg         offset_valid,
    output reg signed [63:0] offset_ns,
    output reg signed [63:0] path_delay_ns,

    // Time-set (only on initial sync, large offset)
    output reg         sec_set_en,
    output reg  [47:0] sec_set_val,

    input  wire [63:0] local_clock_id,

    // Status
    output reg  [15:0] last_seq_id,
    output reg  [31:0] sync_count
);

    assign rx_axis_tready = 1'b1;

    // ---- PTP message types (low nibble of octet 0) ----
    localparam MSG_SYNC        = 4'h0;
    localparam MSG_DELAY_REQ   = 4'h1;
    localparam MSG_FOLLOW_UP   = 4'h8;
    localparam MSG_DELAY_RESP  = 4'h9;
    localparam MSG_ANNOUNCE    = 4'hB;

    // ---- Stored times for current exchange ----
    reg [47:0] t1_sec; reg [31:0] t1_nsec;
    reg [47:0] t2_sec; reg [31:0] t2_nsec;
    reg [47:0] t3_sec; reg [31:0] t3_nsec;
    reg [47:0] t4_sec; reg [31:0] t4_nsec;
    reg signed [63:0] cf_sync;     // corrField from Sync (one-step) or FollowUp
    reg signed [63:0] cf_delay;    // corrField from Delay_Resp
    reg t1_valid, t2_valid, t3_valid, t4_valid;
    reg [15:0] sync_seq_id;
    reg [15:0] delay_req_seq_id;

    // ---- RX parser scratch ----
    reg [10:0] rx_byte_idx;
    reg [3:0]  rx_msg_type;
    reg        rx_two_step;
    reg signed [63:0] rx_corr;
    reg [15:0] rx_seq_id;
    reg [47:0] parsed_sec;
    reg [31:0] parsed_nsec;
    reg        sync_one_step_pending; // captured ts but no follow_up needed

    // ---- Helper: signed full timestamp difference, ns ----
    // Returns (a - b) in nanoseconds as a 64-bit signed integer.
    // Handles seconds wrap correctly.
    function signed [63:0] ts_sub;
        input [47:0] a_sec; input [31:0] a_nsec;
        input [47:0] b_sec; input [31:0] b_nsec;
        reg signed [79:0] a_total, b_total;
        begin
            a_total = $signed({a_sec, 32'b0}) - $signed({a_sec, 32'b0}) // dummy
                    + ($signed({16'b0, a_sec}) * $signed(64'd1_000_000_000))
                    + $signed({32'b0, a_nsec});
            b_total = ($signed({16'b0, b_sec}) * $signed(64'd1_000_000_000))
                    + $signed({32'b0, b_nsec});
            ts_sub = (a_total - b_total) >>> 0;
        end
    endfunction

    // ---- Compute offset / path_delay when all timestamps are present ----
    // RFC 1588-2008 §11.3:
    //   master_to_slave_delay = (t2 - t1) - corrField_sync
    //   slave_to_master_delay = (t4 - t3) - corrField_resp
    //   meanPathDelay = (master_to_slave_delay + slave_to_master_delay) / 2
    //   offsetFromMaster = master_to_slave_delay - meanPathDelay
    task compute_pi;
        reg signed [63:0] m2s, s2m, mpd, off;
        begin
            m2s = ts_sub(t2_sec, t2_nsec, t1_sec, t1_nsec) - (cf_sync >>> 16);
            s2m = ts_sub(t4_sec, t4_nsec, t3_sec, t3_nsec) - (cf_delay >>> 16);
            mpd = (m2s + s2m) >>> 1;
            off = m2s - mpd;
            offset_ns     <= off;
            path_delay_ns <= mpd;
            offset_valid  <= 1'b1;
        end
    endtask

    // ---- RX state machine ----
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            rx_byte_idx           <= 0;
            rx_msg_type           <= 0;
            rx_two_step           <= 0;
            rx_corr               <= 0;
            rx_seq_id             <= 0;
            parsed_sec            <= 0;
            parsed_nsec           <= 0;
            t1_valid              <= 0;
            t2_valid              <= 0;
            t4_valid              <= 0;
            sync_seq_id           <= 0;
            sync_one_step_pending <= 0;
            cf_sync               <= 0;
            cf_delay              <= 0;
            rx_ts_consume         <= 0;
            offset_valid          <= 0;
            offset_ns             <= 0;
            path_delay_ns         <= 0;
            sec_set_en            <= 0;
            sec_set_val           <= 0;
            last_seq_id           <= 0;
            sync_count            <= 0;
        end else begin
            offset_valid  <= 0;
            sec_set_en    <= 0;
            rx_ts_consume <= 0;

            if (rx_axis_tvalid) begin
                case (rx_byte_idx)
                    11'd0:  rx_msg_type        <= rx_axis_tdata[3:0];
                    11'd6:  rx_two_step        <= rx_axis_tdata[1];   // flagField hi, bit 1 = TWO_STEP
                    11'd8:  rx_corr[63:56]     <= rx_axis_tdata;
                    11'd9:  rx_corr[55:48]     <= rx_axis_tdata;
                    11'd10: rx_corr[47:40]     <= rx_axis_tdata;
                    11'd11: rx_corr[39:32]     <= rx_axis_tdata;
                    11'd12: rx_corr[31:24]     <= rx_axis_tdata;
                    11'd13: rx_corr[23:16]     <= rx_axis_tdata;
                    11'd14: rx_corr[15:8]      <= rx_axis_tdata;
                    11'd15: rx_corr[7:0]       <= rx_axis_tdata;
                    11'd30: rx_seq_id[15:8]    <= rx_axis_tdata;
                    11'd31: rx_seq_id[7:0]     <= rx_axis_tdata;
                    11'd34: parsed_sec[47:40]  <= rx_axis_tdata;
                    11'd35: parsed_sec[39:32]  <= rx_axis_tdata;
                    11'd36: parsed_sec[31:24]  <= rx_axis_tdata;
                    11'd37: parsed_sec[23:16]  <= rx_axis_tdata;
                    11'd38: parsed_sec[15:8]   <= rx_axis_tdata;
                    11'd39: parsed_sec[7:0]    <= rx_axis_tdata;
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
                            // Pop the RX timestamp FIFO entry that the
                            // hardware capture unit produced for this frame.
                            if (rx_ts_valid) begin
                                t2_sec        <= rx_ts_sec;
                                t2_nsec       <= rx_ts_nsec;
                                t2_valid      <= 1'b1;
                                rx_ts_consume <= 1'b1;
                                sync_seq_id   <= rx_seq_id;
                                sync_count    <= sync_count + 1;

                                // One-step: Sync carries t1 directly in
                                // the originTimestamp field, no Follow_Up.
                                if (!rx_two_step) begin
                                    t1_sec   <= parsed_sec;
                                    t1_nsec  <= parsed_nsec;
                                    t1_valid <= 1'b1;
                                    cf_sync  <= rx_corr;
                                    if (sync_count == 0) begin
                                        sec_set_en  <= 1'b1;
                                        sec_set_val <= parsed_sec;
                                    end
                                end else begin
                                    // Two-step: also keep the Sync's own
                                    // correctionField. Per RFC §11.4.4
                                    // the receiver must add the Sync corr
                                    // field to the Follow_Up corr field.
                                    cf_sync <= rx_corr;
                                end
                            end
                        end

                        MSG_FOLLOW_UP: begin
                            if (rx_seq_id == sync_seq_id) begin
                                t1_sec   <= parsed_sec;
                                t1_nsec  <= parsed_nsec;
                                t1_valid <= 1'b1;
                                // Sum sync corr + follow_up corr
                                cf_sync  <= cf_sync + rx_corr;
                                if (sync_count <= 32'd1) begin
                                    sec_set_en  <= 1'b1;
                                    sec_set_val <= parsed_sec;
                                end
                            end
                        end

                        MSG_DELAY_RESP: begin
                            if (rx_seq_id == delay_req_seq_id) begin
                                t4_sec   <= parsed_sec;
                                t4_nsec  <= parsed_nsec;
                                t4_valid <= 1'b1;
                                cf_delay <= rx_corr;
                            end
                        end

                        default: ;
                    endcase
                end
            end

            // Compute when all four timestamps + corr fields are in.
            if (t1_valid && t2_valid && t3_valid && t4_valid) begin
                compute_pi();
                t1_valid <= 0;
                t2_valid <= 0;
                t3_valid <= 0;
                t4_valid <= 0;
            end
        end
    end

    // ---- Delay_Req TX state machine ----
    localparam TX_IDLE = 4'd0,
               TX_PTP  = 4'd1,
               TX_WAIT = 4'd2;

    reg [3:0]  tx_state;
    reg [10:0] tx_byte_idx;
    reg [15:0] my_seq_id;
    reg        delay_req_pending;
    reg [23:0] delay_req_timer;
    localparam DELAY_REQ_INTERVAL = 24'd125_000_000; // ~1 sec at 125 MHz

    always @(posedge clk) begin
        if (rst) begin
            tx_state          <= TX_IDLE;
            tx_byte_idx       <= 0;
            tx_axis_tdata     <= 0;
            tx_axis_tvalid    <= 0;
            tx_axis_tlast     <= 0;
            my_seq_id         <= 0;
            delay_req_seq_id  <= 0;
            delay_req_pending <= 0;
            delay_req_timer   <= 0;
            t3_valid          <= 0;
            tx_ts_consume     <= 0;
        end else begin
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
            tx_ts_consume  <= 0;

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
                        11'd0:  tx_axis_tdata <= {4'h0, MSG_DELAY_REQ};
                        11'd1:  tx_axis_tdata <= 8'h02;     // versionPTP=2
                        11'd2:  tx_axis_tdata <= 8'h00;     // length hi
                        11'd3:  tx_axis_tdata <= 8'd44;     // length lo
                        11'd4:  tx_axis_tdata <= 8'h00;     // domain
                        11'd5:  tx_axis_tdata <= 8'h00;
                        11'd6:  tx_axis_tdata <= 8'h00;     // flagField hi
                        11'd7:  tx_axis_tdata <= 8'h00;     // flagField lo
                        11'd8,11'd9,11'd10,11'd11,
                        11'd12,11'd13,11'd14,11'd15:
                                tx_axis_tdata <= 8'h00;     // correctionField
                        11'd16,11'd17,11'd18,11'd19:
                                tx_axis_tdata <= 8'h00;
                        11'd20: tx_axis_tdata <= local_clock_id[63:56];
                        11'd21: tx_axis_tdata <= local_clock_id[55:48];
                        11'd22: tx_axis_tdata <= local_clock_id[47:40];
                        11'd23: tx_axis_tdata <= local_clock_id[39:32];
                        11'd24: tx_axis_tdata <= local_clock_id[31:24];
                        11'd25: tx_axis_tdata <= local_clock_id[23:16];
                        11'd26: tx_axis_tdata <= local_clock_id[15:8];
                        11'd27: tx_axis_tdata <= local_clock_id[7:0];
                        11'd28: tx_axis_tdata <= 8'h00;
                        11'd29: tx_axis_tdata <= 8'h01;     // portNumber=1
                        11'd30: tx_axis_tdata <= my_seq_id[15:8];
                        11'd31: tx_axis_tdata <= my_seq_id[7:0];
                        11'd32: tx_axis_tdata <= 8'h01;     // controlField (Delay_Req)
                        11'd33: tx_axis_tdata <= 8'h7F;     // logMsgInterval
                        11'd34,11'd35,11'd36,11'd37,11'd38,11'd39,
                        11'd40,11'd41,11'd42,11'd43:
                                tx_axis_tdata <= 8'h00;     // origin ts (zero)
                        default: tx_axis_tdata <= 8'h00;
                    endcase

                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd43) begin
                            tx_axis_tlast <= 1;
                            tx_state      <= TX_WAIT;
                        end
                    end
                end

                TX_WAIT: begin
                    // The MAC will produce a TX SFD pulse, which the
                    // timestamp unit captures and pushes into tx_ts FIFO.
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
