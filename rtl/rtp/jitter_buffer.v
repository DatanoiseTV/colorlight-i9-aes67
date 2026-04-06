// SPDX-License-Identifier: MIT
// Per-Stream Jitter Buffer
//
// BRAM-based circular buffer holding incoming RTP audio samples.
// Write side: clocked by RTP RX (variable rate, packet bursts).
// Read side: clocked by audio frame timer (PTP-locked, isochronous).
//
// Provides depth and underrun/overrun status for adaptive control.

module jitter_buffer #(
    parameter SAMPLE_WIDTH = 24,
    parameter NUM_CHANNELS = 8,
    parameter ADDR_WIDTH   = 10        // depth = 2^10 frames = 1024 frames per channel
) (
    input  wire                       clk,
    input  wire                       rst,

    // Write port (from RTP RX)
    input  wire [SAMPLE_WIDTH-1:0]    wr_data,
    input  wire [3:0]                 wr_ch,
    input  wire                       wr_en,
    input  wire                       packet_start,
    input  wire [31:0]                packet_ts,

    // Read port (to audio output)
    input  wire [3:0]                 rd_ch,
    input  wire                       rd_en,
    output reg  [SAMPLE_WIDTH-1:0]    rd_data,

    // Stream config
    input  wire [3:0]                 num_channels,
    input  wire [ADDR_WIDTH-1:0]      target_depth,  // target jitter buffer depth (frames)

    // Status
    output reg  [ADDR_WIDTH:0]        depth_frames,
    output reg                        underrun,
    output reg                        overrun
);

    // Memory: one BRAM per channel (so reads/writes can be independent)
    // Total: NUM_CHANNELS * 2^ADDR_WIDTH * SAMPLE_WIDTH bits
    // For 8ch * 1024 * 24 = 196 kbits (~12% of ECP5-45F BRAM)

    reg [SAMPLE_WIDTH-1:0] mem [0:(NUM_CHANNELS * (1<<ADDR_WIDTH))-1];

    // Per-channel write/read pointers (frame index, not sample)
    reg [ADDR_WIDTH-1:0] wr_frame_ptr;
    reg [ADDR_WIDTH-1:0] rd_frame_ptr;

    // Compute byte address: ch * frame_depth + frame
    wire [ADDR_WIDTH+3:0] wr_addr = (wr_ch * (1<<ADDR_WIDTH)) + wr_frame_ptr;
    wire [ADDR_WIDTH+3:0] rd_addr = (rd_ch * (1<<ADDR_WIDTH)) + rd_frame_ptr;

    always @(posedge clk) begin
        if (rst) begin
            wr_frame_ptr <= 0;
            rd_frame_ptr <= 0;
            depth_frames <= 0;
            underrun     <= 0;
            overrun      <= 0;
            rd_data      <= 0;
        end else begin
            // Write
            if (wr_en) begin
                mem[wr_addr] <= wr_data;
                // Advance frame pointer only after last channel of a frame
                if (wr_ch == num_channels - 1) begin
                    wr_frame_ptr <= wr_frame_ptr + 1;
                    if (depth_frames < (1<<ADDR_WIDTH))
                        depth_frames <= depth_frames + 1;
                    else
                        overrun <= 1;
                end
            end

            // Read
            if (rd_en) begin
                rd_data <= mem[rd_addr];
                // Advance frame pointer after last channel
                if (rd_ch == num_channels - 1) begin
                    if (depth_frames > 0) begin
                        rd_frame_ptr <= rd_frame_ptr + 1;
                        depth_frames <= depth_frames - 1;
                    end else begin
                        underrun <= 1;
                    end
                end
            end
        end
    end

endmodule
