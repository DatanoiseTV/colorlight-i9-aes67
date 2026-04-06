// SPDX-License-Identifier: MIT
// Audio Capture FIFO
//
// Taps the audio data path (post-RTP-decode, post-DSP) and pushes the
// samples for selected channels into a CPU-readable FIFO. Used for
// browser audio preview - the firmware reads samples from the FIFO and
// streams them as a WAV file over HTTP.
//
// Channel mask: 16-bit, one bit per channel. Only samples whose channel
// matches a set bit are written to the FIFO. The CPU sets the mask via
// CSR to select the stereo pair (or any subset) it wants to monitor.
//
// FIFO entry: 4-bit channel + 16-bit signed sample (top 16 bits of L24).
// Capacity: 1024 entries = ~5 ms at 48 kHz × 2 ch.

module audio_capture #(
    parameter NUM_CHANNELS = 16,
    parameter FIFO_ADDR_W  = 10
) (
    input  wire        clk,
    input  wire        rst,

    // Source: tapped from the audio data path
    input  wire [3:0]  src_ch,
    input  wire [23:0] src_sample,
    input  wire        src_tick,

    // Channel mask (CSR)
    input  wire [NUM_CHANNELS-1:0] channel_mask,
    input  wire        enable,

    // CPU read interface (CSR)
    input  wire        cpu_rd,
    output wire [19:0] cpu_rd_data,    // {ch[3:0], sample[15:0]}
    output wire        cpu_empty,
    output wire [FIFO_ADDR_W:0] cpu_level,
    output wire        cpu_overflow
);

    wire ch_active = enable & channel_mask[src_ch];
    wire wr        = src_tick & ch_active;

    // Downsample L24 → int16: keep top 16 bits, drop the lower 8.
    // For musical content this is generally inaudible.
    wire [15:0] sample16 = src_sample[23:8];
    wire [19:0] wr_data  = {src_ch, sample16};

    wire fifo_full;

    fifo_sync #(
        .DATA_WIDTH(20),
        .ADDR_WIDTH(FIFO_ADDR_W)
    ) u_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (wr & ~fifo_full),
        .wr_data(wr_data),
        .rd_en  (cpu_rd),
        .rd_data(cpu_rd_data),
        .full   (fifo_full),
        .empty  (cpu_empty),
        .count  (cpu_level)
    );

    // Sticky overflow flag (cleared by CPU read of overflow CSR)
    reg overflow_sticky;
    always @(posedge clk) begin
        if (rst)
            overflow_sticky <= 0;
        else if (wr & fifo_full)
            overflow_sticky <= 1;
        else if (cpu_rd & cpu_empty)  // ack on first empty read
            overflow_sticky <= 0;
    end

    assign cpu_overflow = overflow_sticky;

endmodule
