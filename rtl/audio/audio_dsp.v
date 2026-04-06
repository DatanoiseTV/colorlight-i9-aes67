// SPDX-License-Identifier: MIT
// Audio DSP Slot - Per-Channel Gain, Mute, Peak Meter
//
// Combinational gain + mute on the audio sample path. Sits between the
// RTP decoder and the virt_tdm16 mixer:
//
//   rtp_engine.audio_rd_data ─► audio_dsp ─► virt_tdm16 ─► I2S/TDM
//
// Per-channel state:
//   gain  (Q1.15 unsigned, default 0x7FFF ≈ 1.0)
//   mute  (1 bit)
//   peak  (24-bit absolute max, latched until CPU reads + clears)
//
// Provides the foundation for adding biquad EQ and other transforms
// later: a future audio_eq_biquad module can be inserted in the same
// path with the same per-channel CSR pattern.
//
// CSR access uses a select-and-write pattern: CPU writes ch_sel, then
// gain_val/mute_val, then pulses gain_we/mute_we.

module audio_dsp #(
    parameter NUM_CHANNELS = 16
) (
    input  wire        clk,
    input  wire        rst,

    // CSR write port
    input  wire [3:0]  cfg_ch_sel,
    input  wire        cfg_gain_we,
    input  wire [15:0] cfg_gain_val,    // Q1.15 unsigned
    input  wire        cfg_mute_we,
    input  wire        cfg_mute_val,

    // CSR meter readback (CPU writes meter_ch_sel, reads meter_value)
    input  wire [3:0]  meter_ch_sel,
    output reg  [23:0] meter_value,
    input  wire        meter_clear,

    // Inline audio path
    input  wire [3:0]  audio_ch,
    input  wire [23:0] audio_in,
    input  wire        audio_tick,
    output wire [23:0] audio_out
);

    // ---- Per-channel state ----
    reg [15:0] gain [0:NUM_CHANNELS-1];
    reg        mute [0:NUM_CHANNELS-1];
    reg [23:0] peak [0:NUM_CHANNELS-1];

    integer i;
    initial begin
        for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
            gain[i] = 16'h7FFF;
            mute[i] = 1'b0;
            peak[i] = 24'd0;
        end
    end

    // ---- CSR writes ----
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
                gain[i] <= 16'h7FFF;
                mute[i] <= 1'b0;
            end
        end else begin
            if (cfg_gain_we) gain[cfg_ch_sel] <= cfg_gain_val;
            if (cfg_mute_we) mute[cfg_ch_sel] <= cfg_mute_val;
        end
    end

    // ---- Combinational gain + mute ----
    // signed 24 × unsigned 16 = signed 40, then shift right by 15 to get
    // back the audio LSB (Q1.15 unity = 0x7FFF).
    wire signed [39:0] product = $signed(audio_in) * $signed({1'b0, gain[audio_ch]});
    wire signed [24:0] scaled  = product[39:15];
    // Saturate to 24-bit signed
    wire [23:0] sat = (scaled > 25'sd8388607)  ? 24'h7FFFFF :
                      (scaled < -25'sd8388608) ? 24'h800000 :
                                                  scaled[23:0];
    assign audio_out = mute[audio_ch] ? 24'd0 : sat;

    // ---- Peak meter ----
    wire [23:0] abs_in = audio_in[23] ? (~audio_in + 1) : audio_in;
    always @(posedge clk) begin
        if (audio_tick && abs_in > peak[audio_ch])
            peak[audio_ch] <= abs_in;
        if (meter_clear)
            peak[meter_ch_sel] <= 24'd0;
        meter_value <= peak[meter_ch_sel];
    end

endmodule
