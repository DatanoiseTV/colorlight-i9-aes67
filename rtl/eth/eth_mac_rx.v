// SPDX-License-Identifier: MIT
// Ethernet MAC Receive Path
// Strips preamble/SFD, checks CRC-32, outputs frame via AXI-Stream
// Provides SFD detection pulse for PTP timestamping

module eth_mac_rx (
    input  wire        clk,
    input  wire        rst,

    // GMII input (from RGMII deserializer)
    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rx_dv,
    input  wire        gmii_rx_err,

    // AXI-Stream output (frame payload: dst_mac ... payload, no CRC)
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tlast,
    output reg         m_axis_tuser, // high on tlast if frame had CRC error
    input  wire        m_axis_tready,

    // PTP timestamp trigger
    output reg         sfd_pulse,    // single-cycle pulse at SFD detection

    // Statistics
    output reg  [31:0] rx_frame_count,
    output reg  [31:0] rx_crc_err_count
);

    localparam [2:0]
        S_IDLE     = 3'd0,
        S_PREAMBLE = 3'd1,
        S_PAYLOAD  = 3'd2,
        S_DRAIN    = 3'd3;

    reg [2:0]  state;
    reg [3:0]  preamble_cnt;
    reg [10:0] byte_cnt;

    // CRC checker runs on all payload bytes (including the 4-byte CRC at end)
    wire [31:0] crc_out;
    wire        crc_valid;
    reg         crc_rst;
    reg         crc_en;

    crc32 u_crc (
        .clk      (clk),
        .rst      (crc_rst),
        .enable   (crc_en),
        .data_in  (gmii_rxd),
        .crc_out  (crc_out),
        .crc_valid(crc_valid)
    );

    // We need to delay output by 4 bytes to strip the CRC
    // Use a small shift register
    reg [7:0]  delay_data [0:3];
    reg [3:0]  delay_valid;
    reg        frame_active;
    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            preamble_cnt   <= 0;
            byte_cnt       <= 0;
            sfd_pulse      <= 0;
            m_axis_tdata   <= 0;
            m_axis_tvalid  <= 0;
            m_axis_tlast   <= 0;
            m_axis_tuser   <= 0;
            crc_rst        <= 1;
            crc_en         <= 0;
            frame_active   <= 0;
            delay_valid    <= 0;
            rx_frame_count <= 0;
            rx_crc_err_count <= 0;
            for (i = 0; i < 4; i = i + 1)
                delay_data[i] <= 0;
        end else begin
            sfd_pulse     <= 0;
            m_axis_tvalid <= 0;
            m_axis_tlast  <= 0;
            m_axis_tuser  <= 0;

            case (state)
                S_IDLE: begin
                    crc_rst      <= 1;
                    crc_en       <= 0;
                    frame_active <= 0;
                    delay_valid  <= 0;
                    byte_cnt     <= 0;
                    preamble_cnt <= 0;
                    if (gmii_rx_dv && gmii_rxd == 8'h55) begin
                        state        <= S_PREAMBLE;
                        preamble_cnt <= 1;
                    end
                end

                S_PREAMBLE: begin
                    if (!gmii_rx_dv) begin
                        state <= S_IDLE;
                    end else if (gmii_rxd == 8'hD5) begin
                        // SFD detected
                        state     <= S_PAYLOAD;
                        sfd_pulse <= 1;
                        crc_rst   <= 0;
                        crc_en    <= 0;
                    end else if (gmii_rxd == 8'h55) begin
                        preamble_cnt <= preamble_cnt + 1;
                    end else begin
                        state <= S_IDLE; // invalid preamble
                    end
                end

                S_PAYLOAD: begin
                    if (!gmii_rx_dv || gmii_rx_err) begin
                        // End of frame - output remaining delayed bytes with tlast
                        state        <= S_DRAIN;
                        crc_en       <= 0;
                        frame_active <= 0;
                    end else begin
                        crc_en <= 1;

                        // Shift delay pipeline
                        delay_data[3] <= delay_data[2];
                        delay_data[2] <= delay_data[1];
                        delay_data[1] <= delay_data[0];
                        delay_data[0] <= gmii_rxd;
                        delay_valid   <= {delay_valid[2:0], 1'b1};

                        byte_cnt <= byte_cnt + 1;

                        // Output delayed data (4 bytes behind to strip CRC)
                        if (delay_valid[3]) begin
                            m_axis_tdata  <= delay_data[3];
                            m_axis_tvalid <= 1;
                            frame_active  <= 1;
                        end
                    end
                end

                S_DRAIN: begin
                    // Frame ended. The last 4 bytes in delay pipeline are CRC.
                    // Don't output them. Just signal tlast on the previous byte.
                    if (frame_active) begin
                        m_axis_tlast <= 1;
                        m_axis_tuser <= ~crc_valid; // flag CRC error
                        m_axis_tvalid <= 1;
                        m_axis_tdata <= m_axis_tdata; // hold last valid byte
                        rx_frame_count <= rx_frame_count + 1;
                        if (!crc_valid)
                            rx_crc_err_count <= rx_crc_err_count + 1;
                    end
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
