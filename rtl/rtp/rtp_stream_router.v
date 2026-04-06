// SPDX-License-Identifier: MIT
// RTP Stream Fanout (RX side)
//
// Multi-stream RTP receive: fans the single incoming RTP stream from
// the packet router out to N rtp_engine instances. Each engine has its
// own SSRC filter inside rtp_rx, so the fanout doesn't need to do any
// classification - each engine simply ignores packets that don't match
// its `rx_expected_ssrc`.
//
// This is a 1→N replication: every byte of every packet appears on
// every output. The downstream rtp_rx blocks decide independently
// whether to consume or drop each packet based on the SSRC field they
// see in the RTP header.

module rtp_stream_router #(
    parameter NUM_STREAMS = 2
) (
    input  wire        clk,
    input  wire        rst,

    // Single RX stream from packet router (UDP payload)
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tlast,
    output wire        s_axis_tready,

    // Per-engine output streams (flattened)
    output wire [NUM_STREAMS*8-1:0] m_axis_tdata,
    output wire [NUM_STREAMS-1:0]   m_axis_tvalid,
    output wire [NUM_STREAMS-1:0]   m_axis_tlast
);

    assign s_axis_tready = 1'b1;

    genvar i;
    generate
        for (i = 0; i < NUM_STREAMS; i = i + 1) begin : gen_fanout
            assign m_axis_tdata [i*8 +: 8] = s_axis_tdata;
            assign m_axis_tvalid[i]        = s_axis_tvalid;
            assign m_axis_tlast [i]        = s_axis_tlast;
        end
    endgenerate

endmodule
