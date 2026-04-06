// SPDX-License-Identifier: MIT
// Ethernet MAC Transmit Path
// Prepends preamble/SFD, appends CRC-32, drives GMII TX
// Provides SFD output pulse for PTP TX timestamping

module eth_mac_tx (
    input  wire        clk,
    input  wire        rst,

    // AXI-Stream input (frame payload: dst_mac ... payload, no CRC)
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tlast,
    output reg         s_axis_tready,

    // GMII output (to RGMII serializer)
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en,
    output reg         gmii_tx_err,

    // PTP timestamp trigger
    output reg         sfd_pulse,    // single-cycle pulse at SFD transmission

    // Statistics
    output reg  [31:0] tx_frame_count
);

    localparam [2:0]
        S_IDLE     = 3'd0,
        S_PREAMBLE = 3'd1,
        S_SFD      = 3'd2,
        S_PAYLOAD  = 3'd3,
        S_PAD      = 3'd4,
        S_CRC      = 3'd5,
        S_IFG      = 3'd6;

    localparam MIN_FRAME_SIZE = 60; // bytes excluding CRC
    localparam IFG_COUNT      = 12; // 96-bit inter-frame gap

    reg [2:0]  state;
    reg [3:0]  cnt;
    reg [10:0] byte_cnt;

    // CRC generator
    wire [31:0] crc_out;
    reg         crc_rst;
    reg         crc_en;
    reg  [7:0]  crc_data;

    crc32 u_crc (
        .clk      (clk),
        .rst      (crc_rst),
        .enable   (crc_en),
        .data_in  (crc_data),
        .crc_out  (crc_out),
        .crc_valid()
    );

    // Registered CRC for byte-by-byte output
    reg [31:0] crc_latch;
    reg [1:0]  crc_byte_idx;

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            cnt            <= 0;
            byte_cnt       <= 0;
            gmii_txd       <= 8'h0;
            gmii_tx_en     <= 0;
            gmii_tx_err    <= 0;
            s_axis_tready  <= 0;
            sfd_pulse      <= 0;
            crc_rst        <= 1;
            crc_en         <= 0;
            crc_data       <= 0;
            crc_latch      <= 0;
            crc_byte_idx   <= 0;
            tx_frame_count <= 0;
        end else begin
            sfd_pulse     <= 0;
            crc_en        <= 0;
            s_axis_tready <= 0;

            case (state)
                S_IDLE: begin
                    gmii_tx_en  <= 0;
                    gmii_tx_err <= 0;
                    crc_rst     <= 1;
                    byte_cnt    <= 0;
                    if (s_axis_tvalid) begin
                        state <= S_PREAMBLE;
                        cnt   <= 0;
                    end
                end

                S_PREAMBLE: begin
                    gmii_tx_en <= 1;
                    gmii_txd   <= 8'h55;
                    crc_rst    <= 1;
                    cnt <= cnt + 1;
                    if (cnt == 6) begin
                        state <= S_SFD;
                    end
                end

                S_SFD: begin
                    gmii_txd  <= 8'hD5;
                    sfd_pulse <= 1;
                    crc_rst   <= 0;
                    state     <= S_PAYLOAD;
                end

                S_PAYLOAD: begin
                    s_axis_tready <= 1;
                    if (s_axis_tvalid) begin
                        gmii_txd <= s_axis_tdata;
                        crc_en   <= 1;
                        crc_data <= s_axis_tdata;
                        byte_cnt <= byte_cnt + 1;

                        if (s_axis_tlast) begin
                            s_axis_tready <= 0;
                            if (byte_cnt + 1 < MIN_FRAME_SIZE)
                                state <= S_PAD;
                            else begin
                                state <= S_CRC;
                                crc_latch    <= crc_out; // will be valid next cycle
                                crc_byte_idx <= 0;
                            end
                        end
                    end
                end

                S_PAD: begin
                    gmii_txd <= 8'h00;
                    crc_en   <= 1;
                    crc_data <= 8'h00;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt + 1 >= MIN_FRAME_SIZE) begin
                        state        <= S_CRC;
                        crc_byte_idx <= 0;
                    end
                end

                S_CRC: begin
                    // Latch CRC on first CRC byte
                    if (crc_byte_idx == 0)
                        crc_latch <= crc_out;

                    case (crc_byte_idx)
                        2'd0: gmii_txd <= crc_out[7:0];
                        2'd1: gmii_txd <= crc_latch[15:8];
                        2'd2: gmii_txd <= crc_latch[23:16];
                        2'd3: gmii_txd <= crc_latch[31:24];
                    endcase

                    crc_byte_idx <= crc_byte_idx + 1;
                    if (crc_byte_idx == 3) begin
                        state          <= S_IFG;
                        cnt            <= 0;
                        tx_frame_count <= tx_frame_count + 1;
                    end
                end

                S_IFG: begin
                    gmii_tx_en <= 0;
                    gmii_txd   <= 8'h0;
                    cnt <= cnt + 1;
                    if (cnt == IFG_COUNT - 1)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
