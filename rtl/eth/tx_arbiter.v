// SPDX-License-Identifier: MIT
// 3-Way TX Arbiter
//
// Multiplexes three AXI-Stream Ethernet frame sources into a single MAC TX:
//   - Port 0: PTP TX (highest priority - timing critical)
//   - Port 1: RTP TX (high priority - media)
//   - Port 2: CPU TX (lowest priority - management)
//
// Strict priority: a higher-priority request will not preempt an in-progress
// frame, but the arbiter will schedule it as soon as the current tlast passes.

module tx_arbiter (
    input  wire        clk,
    input  wire        rst,

    // Port 0: PTP
    input  wire [7:0]  s0_axis_tdata,
    input  wire        s0_axis_tvalid,
    input  wire        s0_axis_tlast,
    output reg         s0_axis_tready,

    // Port 1: RTP
    input  wire [7:0]  s1_axis_tdata,
    input  wire        s1_axis_tvalid,
    input  wire        s1_axis_tlast,
    output reg         s1_axis_tready,

    // Port 2: CPU
    input  wire [7:0]  s2_axis_tdata,
    input  wire        s2_axis_tvalid,
    input  wire        s2_axis_tlast,
    output reg         s2_axis_tready,

    // Output to MAC
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tlast,
    input  wire        m_axis_tready
);

    localparam [1:0] G_IDLE = 2'd3,
                     G_P0   = 2'd0,
                     G_P1   = 2'd1,
                     G_P2   = 2'd2;

    reg [1:0] grant;

    always @(posedge clk) begin
        if (rst) begin
            grant <= G_IDLE;
        end else begin
            case (grant)
                G_IDLE: begin
                    if      (s0_axis_tvalid) grant <= G_P0;
                    else if (s1_axis_tvalid) grant <= G_P1;
                    else if (s2_axis_tvalid) grant <= G_P2;
                end

                G_P0: if (s0_axis_tvalid && s0_axis_tlast && m_axis_tready) grant <= G_IDLE;
                G_P1: if (s1_axis_tvalid && s1_axis_tlast && m_axis_tready) grant <= G_IDLE;
                G_P2: if (s2_axis_tvalid && s2_axis_tlast && m_axis_tready) grant <= G_IDLE;

                default: grant <= G_IDLE;
            endcase
        end
    end

    always @(*) begin
        s0_axis_tready  = 0;
        s1_axis_tready  = 0;
        s2_axis_tready  = 0;
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
            G_P2: begin
                m_axis_tdata    = s2_axis_tdata;
                m_axis_tvalid   = s2_axis_tvalid;
                m_axis_tlast    = s2_axis_tlast;
                s2_axis_tready  = m_axis_tready;
            end
            default: ;
        endcase
    end

endmodule
