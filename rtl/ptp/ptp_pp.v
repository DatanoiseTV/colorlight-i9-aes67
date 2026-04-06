// SPDX-License-Identifier: MIT
// PTP Packet Processor (IEEE 1588-2008)
//
// Implements BOTH master and slave sides of the delay-request-response
// mechanism end-to-end in hardware. Mode is selected by `mode_is_master`.
//
// SLAVE (mode_is_master = 0):
//
//    Master                          Us (slave)
//      |--- Sync (t1) ---------------->|  capture t2 = RX SFD ts
//      |--- Follow_Up (t1) ----------->|  store t1, sum corrField
//      |<-- Delay_Req --(t3)-----------|  capture t3 = TX SFD ts
//      |--- Delay_Resp (t4) ---------->|  store t4
//
//    offset = ((t2 - t1 - cf_sync) - (t4 - t3 - cf_resp)) / 2
//    delay  = ((t2 - t1 - cf_sync) + (t4 - t3 - cf_resp)) / 2
//
// MASTER (mode_is_master = 1):
//
//    Us (master)                     Slave
//      |--- Sync ----(t1)------------>|  we capture our own t1 = TX SFD ts
//      |--- Follow_Up (carrying t1) ->|
//      |<-- Delay_Req ---(t3)---------|  we capture t4 = our RX SFD ts
//      |--- Delay_Resp (t4) ---------->|  echo back the slave's source ID
//
// Single always block to avoid multi-driver synthesis errors.

module ptp_pp (
    input  wire        clk,
    input  wire        rst,

    input  wire        mode_is_master,
    input  wire [31:0] sync_interval_cycles,

    // PTP UDP payload from packet router
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output wire        rx_axis_tready,

    // RX timestamp FIFO
    input  wire [47:0] rx_ts_sec,
    input  wire [31:0] rx_ts_nsec,
    input  wire        rx_ts_valid,
    output reg         rx_ts_consume,

    // TX timestamp FIFO
    input  wire [47:0] tx_ts_sec,
    input  wire [31:0] tx_ts_nsec,
    input  wire        tx_ts_valid,
    output reg         tx_ts_consume,

    // PTP UDP payload to TX wrapper
    output reg  [7:0]  tx_axis_tdata,
    output reg         tx_axis_tvalid,
    output reg         tx_axis_tlast,
    input  wire        tx_axis_tready,

    // Servo input (slave mode only)
    output reg         offset_valid,
    output reg signed [63:0] offset_ns,
    output reg signed [63:0] path_delay_ns,

    output reg         sec_set_en,
    output reg  [47:0] sec_set_val,

    input  wire [63:0] local_clock_id,

    output reg  [15:0] last_seq_id,
    output reg  [31:0] sync_count
);

    assign rx_axis_tready = 1'b1;

    // ---- PTP message types ----
    localparam MSG_SYNC        = 4'h0;
    localparam MSG_DELAY_REQ   = 4'h1;
    localparam MSG_FOLLOW_UP   = 4'h8;
    localparam MSG_DELAY_RESP  = 4'h9;
    localparam MSG_ANNOUNCE    = 4'hB;

    // ---- Stored timestamps ----
    reg [47:0] t1_sec; reg [31:0] t1_nsec;
    reg [47:0] t2_sec; reg [31:0] t2_nsec;
    reg [47:0] t3_sec; reg [31:0] t3_nsec;
    reg [47:0] t4_sec; reg [31:0] t4_nsec;
    reg signed [63:0] cf_sync;
    reg signed [63:0] cf_delay;
    reg t1_valid, t2_valid, t3_valid, t4_valid;
    reg [15:0] sync_seq_id;
    reg [15:0] delay_req_seq_id;

    // ---- Master-side: Delay_Req receive metadata ----
    reg [79:0] dreq_rx_ts;
    reg [79:0] dreq_src_clk_id;
    reg [15:0] dreq_src_port;
    reg [15:0] dreq_seq_id;
    reg        dreq_pending;

    // ---- Pipelined offset compute state ----
    // 5-cycle pipeline so each stage has only one heavy operation.
    //   stage 1: pi_t2t1 <= t2 - t1                  (ts_sub, slow)
    //   stage 2: pi_m2s  <= pi_t2t1 - (cf_sync >>> 16)
    //   stage 3: pi_t4t3 <= t4 - t3                  (ts_sub)
    //   stage 4: pi_s2m  <= pi_t4t3 - (cf_delay >>> 16)
    //   stage 5: offset_ns / path_delay_ns
    reg [2:0]  pi_stage;
    reg signed [63:0] pi_t2t1;
    reg signed [63:0] pi_t4t3;
    reg signed [63:0] pi_m2s;
    reg signed [63:0] pi_s2m;

    // ---- RX parser scratch ----
    reg [10:0] rx_byte_idx;
    reg [3:0]  rx_msg_type;
    reg        rx_two_step;
    reg signed [63:0] rx_corr;
    reg [15:0] rx_seq_id;
    reg [47:0] parsed_sec;
    reg [31:0] parsed_nsec;
    reg [79:0] parsed_src_clk_id;
    reg [15:0] parsed_src_port;

    // ---- Master Sync timer / pending flags ----
    reg [31:0] sync_timer;
    reg        sync_pending;
    reg        followup_pending;
    reg [47:0] last_sync_tx_sec;
    reg [31:0] last_sync_tx_nsec;
    reg [15:0] master_seq_id;

    // ---- TX state machine ----
    localparam TX_IDLE        = 4'd0,
               TX_DREQ        = 4'd1,
               TX_DREQ_WAIT   = 4'd2,
               TX_SYNC        = 4'd3,
               TX_SYNC_WAIT   = 4'd4,
               TX_FOLLOWUP    = 4'd5,
               TX_DELAY_RESP  = 4'd6;

    reg [3:0]  tx_state;
    reg [10:0] tx_byte_idx;
    reg [15:0] my_seq_id;
    reg        delay_req_pending;
    reg [23:0] delay_req_timer;
    localparam DELAY_REQ_INTERVAL = 24'd125_000_000;

    // ---- 64-bit signed nanosecond timestamp difference ----
    // Optimized: avoids the 48×64 bit multiply that wrecks timing.
    // Once a slave is locked, t1/t2 are always in the same second or one
    // apart, so we only need to handle sec_a - sec_b ∈ {-1, 0, +1, +2}.
    // For larger gaps the result is clamped (slave will phase-step).
    function signed [63:0] ts_sub;
        input [47:0] a_sec; input [31:0] a_nsec;
        input [47:0] b_sec; input [31:0] b_nsec;
        reg signed [33:0] sec_diff;
        reg signed [63:0] r;
        begin
            sec_diff = $signed({2'b00, a_sec[31:0]}) - $signed({2'b00, b_sec[31:0]});
            r = $signed({32'b0, a_nsec}) - $signed({32'b0, b_nsec});
            // Apply seconds carry/borrow, clamped to ±2 seconds
            case (sec_diff[2:0])
                3'b000: ts_sub = r;
                3'b001: ts_sub = r + 64'sd1_000_000_000;
                3'b010: ts_sub = r + 64'sd2_000_000_000;
                3'b111: ts_sub = r - 64'sd1_000_000_000;
                3'b110: ts_sub = r - 64'sd2_000_000_000;
                default: ts_sub = (sec_diff[33] ? -64'sd2_000_000_000 :
                                                   64'sd2_000_000_000);
            endcase
        end
    endfunction

    // Common-case PTP header byte emitter
    `define HDR_BYTE(IDX, MSG, CTRL, LEN, FLAG_HI, FLAG_LO) \
        case (IDX) \
            11'd0:  tx_axis_tdata <= {4'h0, MSG}; \
            11'd1:  tx_axis_tdata <= 8'h02; \
            11'd2:  tx_axis_tdata <= 8'h00; \
            11'd3:  tx_axis_tdata <= LEN; \
            11'd4:  tx_axis_tdata <= 8'h00; \
            11'd5:  tx_axis_tdata <= 8'h00; \
            11'd6:  tx_axis_tdata <= FLAG_HI; \
            11'd7:  tx_axis_tdata <= FLAG_LO; \
            11'd8,11'd9,11'd10,11'd11,11'd12,11'd13,11'd14,11'd15: \
                    tx_axis_tdata <= 8'h00; \
            11'd16,11'd17,11'd18,11'd19: tx_axis_tdata <= 8'h00; \
            11'd20: tx_axis_tdata <= local_clock_id[63:56]; \
            11'd21: tx_axis_tdata <= local_clock_id[55:48]; \
            11'd22: tx_axis_tdata <= local_clock_id[47:40]; \
            11'd23: tx_axis_tdata <= local_clock_id[39:32]; \
            11'd24: tx_axis_tdata <= local_clock_id[31:24]; \
            11'd25: tx_axis_tdata <= local_clock_id[23:16]; \
            11'd26: tx_axis_tdata <= local_clock_id[15:8]; \
            11'd27: tx_axis_tdata <= local_clock_id[7:0]; \
            11'd28: tx_axis_tdata <= 8'h00; \
            11'd29: tx_axis_tdata <= 8'h01; \
            11'd32: tx_axis_tdata <= CTRL; \
            11'd33: tx_axis_tdata <= 8'hFD; \
            default: ; \
        endcase

    // ---- ONE big always block (single writer per signal) ----
    always @(posedge clk) begin
        if (rst) begin
            // RX state
            rx_byte_idx       <= 0;
            rx_msg_type       <= 0;
            rx_two_step       <= 0;
            rx_corr           <= 0;
            rx_seq_id         <= 0;
            parsed_sec        <= 0;
            parsed_nsec       <= 0;
            parsed_src_clk_id <= 0;
            parsed_src_port   <= 0;
            t1_sec <= 0; t1_nsec <= 0;
            t2_sec <= 0; t2_nsec <= 0;
            t3_sec <= 0; t3_nsec <= 0;
            t4_sec <= 0; t4_nsec <= 0;
            t1_valid <= 0; t2_valid <= 0; t3_valid <= 0; t4_valid <= 0;
            sync_seq_id       <= 0;
            cf_sync           <= 0;
            cf_delay          <= 0;
            rx_ts_consume     <= 0;
            offset_valid      <= 0;
            offset_ns         <= 0;
            path_delay_ns     <= 0;
            sec_set_en        <= 0;
            sec_set_val       <= 0;
            last_seq_id       <= 0;
            sync_count        <= 0;
            dreq_rx_ts        <= 0;
            dreq_src_clk_id   <= 0;
            dreq_src_port     <= 0;
            dreq_seq_id       <= 0;
            dreq_pending      <= 0;

            // TX state
            tx_state          <= TX_IDLE;
            tx_byte_idx       <= 0;
            tx_axis_tdata     <= 0;
            tx_axis_tvalid    <= 0;
            tx_axis_tlast     <= 0;
            my_seq_id         <= 0;
            delay_req_seq_id  <= 0;
            delay_req_pending <= 0;
            delay_req_timer   <= 0;
            tx_ts_consume     <= 0;

            // Master-mode state
            sync_timer        <= 0;
            sync_pending      <= 0;
            followup_pending  <= 0;
            last_sync_tx_sec  <= 0;
            last_sync_tx_nsec <= 0;
            master_seq_id     <= 0;

            // Pipelined PI compute
            pi_stage <= 0;
            pi_t2t1  <= 0;
            pi_t4t3  <= 0;
            pi_m2s   <= 0;
            pi_s2m   <= 0;
        end else begin
            // ---- Default 1-cycle pulses ----
            offset_valid   <= 0;
            sec_set_en     <= 0;
            rx_ts_consume  <= 0;
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
            tx_ts_consume  <= 0;

            // ================================================================
            // RX byte parsing
            // ================================================================
            if (rx_axis_tvalid) begin
                case (rx_byte_idx)
                    11'd0:  rx_msg_type        <= rx_axis_tdata[3:0];
                    11'd6:  rx_two_step        <= rx_axis_tdata[1];
                    11'd8:  rx_corr[63:56]     <= rx_axis_tdata;
                    11'd9:  rx_corr[55:48]     <= rx_axis_tdata;
                    11'd10: rx_corr[47:40]     <= rx_axis_tdata;
                    11'd11: rx_corr[39:32]     <= rx_axis_tdata;
                    11'd12: rx_corr[31:24]     <= rx_axis_tdata;
                    11'd13: rx_corr[23:16]     <= rx_axis_tdata;
                    11'd14: rx_corr[15:8]      <= rx_axis_tdata;
                    11'd15: rx_corr[7:0]       <= rx_axis_tdata;
                    11'd20: parsed_src_clk_id[79:72] <= rx_axis_tdata;
                    11'd21: parsed_src_clk_id[71:64] <= rx_axis_tdata;
                    11'd22: parsed_src_clk_id[63:56] <= rx_axis_tdata;
                    11'd23: parsed_src_clk_id[55:48] <= rx_axis_tdata;
                    11'd24: parsed_src_clk_id[47:40] <= rx_axis_tdata;
                    11'd25: parsed_src_clk_id[39:32] <= rx_axis_tdata;
                    11'd26: parsed_src_clk_id[31:24] <= rx_axis_tdata;
                    11'd27: parsed_src_clk_id[23:16] <= rx_axis_tdata;
                    11'd28: parsed_src_port[15:8]    <= rx_axis_tdata;
                    11'd29: parsed_src_port[7:0]     <= rx_axis_tdata;
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

                    if (!mode_is_master) begin
                        // Slave-side message handling
                        case (rx_msg_type)
                            MSG_SYNC: if (rx_ts_valid) begin
                                t2_sec        <= rx_ts_sec;
                                t2_nsec       <= rx_ts_nsec;
                                t2_valid      <= 1'b1;
                                rx_ts_consume <= 1'b1;
                                sync_seq_id   <= rx_seq_id;
                                sync_count    <= sync_count + 1;
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
                                    cf_sync <= rx_corr;
                                end
                            end

                            MSG_FOLLOW_UP: if (rx_seq_id == sync_seq_id) begin
                                t1_sec   <= parsed_sec;
                                t1_nsec  <= parsed_nsec;
                                t1_valid <= 1'b1;
                                cf_sync  <= cf_sync + rx_corr;
                                if (sync_count <= 32'd1) begin
                                    sec_set_en  <= 1'b1;
                                    sec_set_val <= parsed_sec;
                                end
                            end

                            MSG_DELAY_RESP: if (rx_seq_id == delay_req_seq_id) begin
                                t4_sec   <= parsed_sec;
                                t4_nsec  <= parsed_nsec;
                                t4_valid <= 1'b1;
                                cf_delay <= rx_corr;
                            end

                            default: ;
                        endcase
                    end else begin
                        // Master-side: capture Delay_Req and queue Delay_Resp
                        if (rx_msg_type == MSG_DELAY_REQ && rx_ts_valid) begin
                            dreq_rx_ts      <= {rx_ts_sec, rx_ts_nsec};
                            dreq_src_clk_id <= parsed_src_clk_id;
                            dreq_src_port   <= parsed_src_port;
                            dreq_seq_id     <= rx_seq_id;
                            dreq_pending    <= 1;
                            rx_ts_consume   <= 1;
                        end
                    end
                end
            end

            // ================================================================
            // Slave: pipelined offset/delay compute (5 stages)
            // ================================================================
            case (pi_stage)
                3'd0: if (!mode_is_master && t1_valid && t2_valid &&
                          t3_valid && t4_valid) begin
                    pi_t2t1  <= ts_sub(t2_sec, t2_nsec, t1_sec, t1_nsec);
                    pi_stage <= 3'd1;
                end
                3'd1: begin
                    pi_m2s   <= pi_t2t1 - (cf_sync >>> 16);
                    pi_stage <= 3'd2;
                end
                3'd2: begin
                    pi_t4t3  <= ts_sub(t4_sec, t4_nsec, t3_sec, t3_nsec);
                    pi_stage <= 3'd3;
                end
                3'd3: begin
                    pi_s2m   <= pi_t4t3 - (cf_delay >>> 16);
                    pi_stage <= 3'd4;
                end
                3'd4: begin
                    offset_ns     <= (pi_m2s - pi_s2m) >>> 1;
                    path_delay_ns <= (pi_m2s + pi_s2m) >>> 1;
                    offset_valid  <= 1'b1;
                    pi_stage      <= 3'd0;
                    t1_valid <= 0;
                    t2_valid <= 0;
                    t3_valid <= 0;
                    t4_valid <= 0;
                end
                default: pi_stage <= 0;
            endcase

            // ================================================================
            // Master Sync period timer
            // ================================================================
            if (mode_is_master) begin
                if (sync_timer == 0) begin
                    sync_timer   <= sync_interval_cycles;
                    sync_pending <= 1;
                end else begin
                    sync_timer <= sync_timer - 1;
                end
            end else begin
                sync_timer       <= 0;
                sync_pending     <= 0;
                followup_pending <= 0;
            end

            // ================================================================
            // Slave Delay_Req trigger timer
            // ================================================================
            if (!mode_is_master) begin
                if (t2_valid && delay_req_timer == 0) begin
                    delay_req_pending <= 1;
                    delay_req_timer   <= DELAY_REQ_INTERVAL;
                    my_seq_id         <= my_seq_id + 1;
                end else if (delay_req_timer > 0) begin
                    delay_req_timer <= delay_req_timer - 1;
                end
            end else begin
                delay_req_pending <= 0;
                delay_req_timer   <= 0;
            end

            // ================================================================
            // TX state machine
            // ================================================================
            case (tx_state)
                TX_IDLE: begin
                    if (mode_is_master && sync_pending && tx_axis_tready) begin
                        tx_state    <= TX_SYNC;
                        tx_byte_idx <= 0;
                    end else if (mode_is_master && followup_pending && tx_axis_tready) begin
                        tx_state    <= TX_FOLLOWUP;
                        tx_byte_idx <= 0;
                    end else if (mode_is_master && dreq_pending && tx_axis_tready) begin
                        tx_state    <= TX_DELAY_RESP;
                        tx_byte_idx <= 0;
                    end else if (!mode_is_master && delay_req_pending && tx_axis_tready) begin
                        tx_state          <= TX_DREQ;
                        tx_byte_idx       <= 0;
                        delay_req_seq_id  <= my_seq_id;
                        delay_req_pending <= 0;
                    end
                end

                // ---- Slave: Delay_Req ----
                TX_DREQ: begin
                    tx_axis_tvalid <= 1;
                    `HDR_BYTE(tx_byte_idx, MSG_DELAY_REQ, 8'h01, 8'd44, 8'h00, 8'h00)
                    if (tx_byte_idx == 11'd30) tx_axis_tdata <= my_seq_id[15:8];
                    if (tx_byte_idx == 11'd31) tx_axis_tdata <= my_seq_id[7:0];
                    if (tx_byte_idx >= 11'd34 && tx_byte_idx <= 11'd43)
                        tx_axis_tdata <= 8'h00;
                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd43) begin
                            tx_axis_tlast <= 1;
                            tx_state      <= TX_DREQ_WAIT;
                        end
                    end
                end

                TX_DREQ_WAIT: if (tx_ts_valid) begin
                    t3_sec        <= tx_ts_sec;
                    t3_nsec       <= tx_ts_nsec;
                    t3_valid      <= 1;
                    tx_ts_consume <= 1;
                    tx_state      <= TX_IDLE;
                end

                // ---- Master: Sync ----
                TX_SYNC: begin
                    tx_axis_tvalid <= 1;
                    `HDR_BYTE(tx_byte_idx, MSG_SYNC, 8'h00, 8'd44, 8'h02, 8'h00)
                    if (tx_byte_idx == 11'd30) tx_axis_tdata <= master_seq_id[15:8];
                    if (tx_byte_idx == 11'd31) tx_axis_tdata <= master_seq_id[7:0];
                    if (tx_byte_idx >= 11'd34 && tx_byte_idx <= 11'd43)
                        tx_axis_tdata <= 8'h00;
                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd43) begin
                            tx_axis_tlast <= 1;
                            tx_state      <= TX_SYNC_WAIT;
                            sync_pending  <= 0;
                        end
                    end
                end

                TX_SYNC_WAIT: if (tx_ts_valid) begin
                    last_sync_tx_sec  <= tx_ts_sec;
                    last_sync_tx_nsec <= tx_ts_nsec;
                    tx_ts_consume     <= 1;
                    followup_pending  <= 1;
                    tx_state          <= TX_IDLE;
                end

                // ---- Master: Follow_Up ----
                TX_FOLLOWUP: begin
                    tx_axis_tvalid <= 1;
                    `HDR_BYTE(tx_byte_idx, MSG_FOLLOW_UP, 8'h02, 8'd44, 8'h00, 8'h00)
                    if (tx_byte_idx == 11'd30) tx_axis_tdata <= master_seq_id[15:8];
                    if (tx_byte_idx == 11'd31) tx_axis_tdata <= master_seq_id[7:0];
                    if (tx_byte_idx == 11'd34) tx_axis_tdata <= last_sync_tx_sec[47:40];
                    if (tx_byte_idx == 11'd35) tx_axis_tdata <= last_sync_tx_sec[39:32];
                    if (tx_byte_idx == 11'd36) tx_axis_tdata <= last_sync_tx_sec[31:24];
                    if (tx_byte_idx == 11'd37) tx_axis_tdata <= last_sync_tx_sec[23:16];
                    if (tx_byte_idx == 11'd38) tx_axis_tdata <= last_sync_tx_sec[15:8];
                    if (tx_byte_idx == 11'd39) tx_axis_tdata <= last_sync_tx_sec[7:0];
                    if (tx_byte_idx == 11'd40) tx_axis_tdata <= last_sync_tx_nsec[31:24];
                    if (tx_byte_idx == 11'd41) tx_axis_tdata <= last_sync_tx_nsec[23:16];
                    if (tx_byte_idx == 11'd42) tx_axis_tdata <= last_sync_tx_nsec[15:8];
                    if (tx_byte_idx == 11'd43) tx_axis_tdata <= last_sync_tx_nsec[7:0];
                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd43) begin
                            tx_axis_tlast    <= 1;
                            tx_state         <= TX_IDLE;
                            followup_pending <= 0;
                            master_seq_id    <= master_seq_id + 1;
                        end
                    end
                end

                // ---- Master: Delay_Resp (54 bytes) ----
                TX_DELAY_RESP: begin
                    tx_axis_tvalid <= 1;
                    `HDR_BYTE(tx_byte_idx, MSG_DELAY_RESP, 8'h03, 8'd54, 8'h00, 8'h00)
                    if (tx_byte_idx == 11'd30) tx_axis_tdata <= dreq_seq_id[15:8];
                    if (tx_byte_idx == 11'd31) tx_axis_tdata <= dreq_seq_id[7:0];
                    if (tx_byte_idx == 11'd34) tx_axis_tdata <= dreq_rx_ts[79:72];
                    if (tx_byte_idx == 11'd35) tx_axis_tdata <= dreq_rx_ts[71:64];
                    if (tx_byte_idx == 11'd36) tx_axis_tdata <= dreq_rx_ts[63:56];
                    if (tx_byte_idx == 11'd37) tx_axis_tdata <= dreq_rx_ts[55:48];
                    if (tx_byte_idx == 11'd38) tx_axis_tdata <= dreq_rx_ts[47:40];
                    if (tx_byte_idx == 11'd39) tx_axis_tdata <= dreq_rx_ts[39:32];
                    if (tx_byte_idx == 11'd40) tx_axis_tdata <= dreq_rx_ts[31:24];
                    if (tx_byte_idx == 11'd41) tx_axis_tdata <= dreq_rx_ts[23:16];
                    if (tx_byte_idx == 11'd42) tx_axis_tdata <= dreq_rx_ts[15:8];
                    if (tx_byte_idx == 11'd43) tx_axis_tdata <= dreq_rx_ts[7:0];
                    if (tx_byte_idx == 11'd44) tx_axis_tdata <= dreq_src_clk_id[79:72];
                    if (tx_byte_idx == 11'd45) tx_axis_tdata <= dreq_src_clk_id[71:64];
                    if (tx_byte_idx == 11'd46) tx_axis_tdata <= dreq_src_clk_id[63:56];
                    if (tx_byte_idx == 11'd47) tx_axis_tdata <= dreq_src_clk_id[55:48];
                    if (tx_byte_idx == 11'd48) tx_axis_tdata <= dreq_src_clk_id[47:40];
                    if (tx_byte_idx == 11'd49) tx_axis_tdata <= dreq_src_clk_id[39:32];
                    if (tx_byte_idx == 11'd50) tx_axis_tdata <= dreq_src_clk_id[31:24];
                    if (tx_byte_idx == 11'd51) tx_axis_tdata <= dreq_src_clk_id[23:16];
                    if (tx_byte_idx == 11'd52) tx_axis_tdata <= dreq_src_port[15:8];
                    if (tx_byte_idx == 11'd53) tx_axis_tdata <= dreq_src_port[7:0];
                    if (tx_axis_tready) begin
                        tx_byte_idx <= tx_byte_idx + 1;
                        if (tx_byte_idx == 11'd53) begin
                            tx_axis_tlast <= 1;
                            tx_state      <= TX_IDLE;
                            dreq_pending  <= 0;
                        end
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
