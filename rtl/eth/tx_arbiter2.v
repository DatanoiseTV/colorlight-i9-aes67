// SPDX-License-Identifier: MIT
// 2-Way TX Arbiter (priority: port 0 > port 1)
//
// Used inside the multi-stream RTP TX path to merge per-stream RTP
// Ethernet frame outputs into a single stream that feeds the top-level
// 3-way arbiter (PTP > RTP > CPU).

module tx_arbiter2 (
    input  wire        clk,
    input  wire        rst,

    input  wire [7:0]  s0_axis_tdata,
    input  wire        s0_axis_tvalid,
    input  wire        s0_axis_tlast,
    output reg         s0_axis_tready,

    input  wire [7:0]  s1_axis_tdata,
    input  wire        s1_axis_tvalid,
    input  wire        s1_axis_tlast,
    output reg         s1_axis_tready,

    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tlast,
    input  wire        m_axis_tready
);

    localparam G_IDLE = 2'd2,
               G_P0   = 2'd0,
               G_P1   = 2'd1;

    reg [1:0] grant;

    always @(posedge clk) begin
        if (rst) grant <= G_IDLE;
        else begin
            case (grant)
                G_IDLE: begin
                    if      (s0_axis_tvalid) grant <= G_P0;
                    else if (s1_axis_tvalid) grant <= G_P1;
                end
                G_P0: if (s0_axis_tvalid && s0_axis_tlast && m_axis_tready) grant <= G_IDLE;
                G_P1: if (s1_axis_tvalid && s1_axis_tlast && m_axis_tready) grant <= G_IDLE;
                default: grant <= G_IDLE;
            endcase
        end
    end

    always @(*) begin
        s0_axis_tready  = 0;
        s1_axis_tready  = 0;
        m_axis_tdata    = 8'h0;
        m_axis_tvalid   = 0;
        m_axis_tlast    = 0;
        case (grant)
            G_P0: begin
                m_axis_tdata    = s0_axis_tdata;
                m_axis_tvalid   = s0_axis_tvalid;
                m_axis_tlast    = s0_axis_tlast;
                s0_axis_tready  = m_axis_tready;
            end
            G_P1: begin
                m_axis_tdata    = s1_axis_tdata;
                m_axis_tvalid   = s1_axis_tvalid;
                m_axis_tlast    = s1_axis_tlast;
                s1_axis_tready  = m_axis_tready;
            end
            default: ;
        endcase
    end

endmodule
