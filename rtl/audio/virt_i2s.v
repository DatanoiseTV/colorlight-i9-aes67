// SPDX-License-Identifier: MIT
// Virtual I2S Peripheral
//
// CPU-accessible audio FIFOs that integrate into the audio data path.
// The CPU sees a memory-mapped pair of FIFOs (TX and RX) that operate at
// the audio sample rate.
//
// TX FIFO: CPU writes samples; they are mixed into the outgoing audio frame
//          (added to the corresponding RTP TX channel) or, if mix_mode=0,
//          replace the channel.
// RX FIFO: incoming audio samples (from the I2S ADC or from a received RTP
//          stream) are pushed in; CPU reads them.
//
// This lets the CPU generate test tones, run software DSP, record audio to
// SD/flash/network, or feed network audio into the AES67 stream — all
// without touching the timing-critical RTP data path.
//
// Sample format: 32-bit signed left-justified (24-bit audio in MSBs).

module virt_i2s #(
    parameter NUM_CHANNELS  = 8,
    parameter FIFO_ADDR_W   = 8     // 256 entries per direction
) (
    input  wire        clk,
    input  wire        rst,

    // CPU interface (read by LiteX CSR wrapper)
    // TX FIFO write
    input  wire        cpu_tx_wr,
    input  wire [31:0] cpu_tx_data,    // sample (left-justified 24-bit in MSBs)
    input  wire [3:0]  cpu_tx_ch,
    output wire        cpu_tx_full,
    output wire [FIFO_ADDR_W:0] cpu_tx_level,

    // RX FIFO read
    input  wire        cpu_rx_rd,
    output wire [31:0] cpu_rx_data,
    output wire [3:0]  cpu_rx_ch,
    output wire        cpu_rx_empty,
    output wire [FIFO_ADDR_W:0] cpu_rx_level,

    // Mix configuration
    input  wire        mix_enable,     // 1 = mix CPU samples with RTP, 0 = replace
    input  wire [NUM_CHANNELS-1:0] cpu_channel_mask, // which channels CPU drives

    // Audio path: outgoing samples (to RTP TX / I2S TX)
    input  wire [3:0]  audio_tx_ch,
    input  wire [23:0] audio_tx_in,    // sample from RTP RX or other source
    output reg  [23:0] audio_tx_out,   // sample to send out (after CPU mix)
    input  wire        audio_tx_tick,  // pulse: a new outgoing sample is being computed

    // Audio path: incoming samples (from I2S RX / RTP RX)
    input  wire [3:0]  audio_rx_ch,
    input  wire [23:0] audio_rx_in,    // captured sample
    input  wire        audio_rx_tick   // pulse: a new incoming sample is available
);

    // ---- TX FIFO: CPU → audio (one entry per channel, fed by single FIFO with ch tag) ----
    wire [35:0] tx_fifo_wr_data = {cpu_tx_ch, cpu_tx_data[31:0]};
    wire [35:0] tx_fifo_rd_data;
    wire        tx_fifo_empty;
    wire        tx_fifo_rd_en;

    fifo_sync #(
        .DATA_WIDTH(36),
        .ADDR_WIDTH(FIFO_ADDR_W)
    ) u_tx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (cpu_tx_wr),
        .wr_data(tx_fifo_wr_data),
        .rd_en  (tx_fifo_rd_en),
        .rd_data(tx_fifo_rd_data),
        .full   (cpu_tx_full),
        .empty  (tx_fifo_empty),
        .count  (cpu_tx_level)
    );

    // Per-channel hold register: latest CPU sample for each channel
    reg [23:0] cpu_sample [0:NUM_CHANNELS-1];
    reg        cpu_sample_valid [0:NUM_CHANNELS-1];

    // Pop FIFO entries continuously into the per-channel holding regs
    assign tx_fifo_rd_en = ~tx_fifo_empty;

    integer i;
    initial begin
        for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
            cpu_sample[i]       = 24'd0;
            cpu_sample_valid[i] = 1'b0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
                cpu_sample[i]       <= 24'd0;
                cpu_sample_valid[i] <= 1'b0;
            end
        end else begin
            if (tx_fifo_rd_en) begin
                cpu_sample      [tx_fifo_rd_data[35:32]] <= tx_fifo_rd_data[31:8];
                cpu_sample_valid[tx_fifo_rd_data[35:32]] <= 1'b1;
            end
        end
    end

    // ---- Mixing: CPU samples + audio_tx_in → audio_tx_out ----
    wire ch_active = cpu_channel_mask[audio_tx_ch] & cpu_sample_valid[audio_tx_ch];

    always @(posedge clk) begin
        if (rst) begin
            audio_tx_out <= 0;
        end else begin
            if (audio_tx_tick) begin
                if (ch_active) begin
                    if (mix_enable) begin
                        // Saturated add (24-bit signed)
                        // Simple: average to avoid overflow, scaled by 2 later if needed
                        audio_tx_out <= $signed(audio_tx_in[23:1]) + $signed(cpu_sample[audio_tx_ch][23:1]);
                    end else begin
                        audio_tx_out <= cpu_sample[audio_tx_ch];
                    end
                end else begin
                    audio_tx_out <= audio_tx_in;
                end
            end
        end
    end

    // ---- RX FIFO: audio → CPU ----
    wire [27:0] rx_fifo_wr_data = {audio_rx_ch, audio_rx_in};
    wire [27:0] rx_fifo_rd_data;
    wire        rx_fifo_full;

    fifo_sync #(
        .DATA_WIDTH(28),
        .ADDR_WIDTH(FIFO_ADDR_W)
    ) u_rx_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (audio_rx_tick & ~rx_fifo_full),
        .wr_data(rx_fifo_wr_data),
        .rd_en  (cpu_rx_rd),
        .rd_data(rx_fifo_rd_data),
        .full   (rx_fifo_full),
        .empty  (cpu_rx_empty),
        .count  (cpu_rx_level)
    );

    assign cpu_rx_data = {{8{rx_fifo_rd_data[23]}}, rx_fifo_rd_data[23:0]}; // sign-extend to 32
    assign cpu_rx_ch   = rx_fifo_rd_data[27:24];

endmodule
