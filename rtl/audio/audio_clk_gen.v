// SPDX-License-Identifier: MIT
// PTP-Disciplined Audio Clock Generator
//
// Generates BCLK / LRCLK / MCLK from a numerically-controlled oscillator
// (NCO) whose increment is actively pulled by the PTP servo. As the PTP
// clock servo applies frequency corrections (in ppb), the audio NCO
// receives the same correction so the audio sample rate stays locked
// to the grandmaster - even if the local oscillator drifts.
//
//   bclk_freq = (clk125 * increment) / 2^32
//
// For 48 kHz x 8 slots x 32 bit TDM = 12.288 MHz BCLK:
//   nominal_increment = 12.288e6 * 2^32 / 125e6 ~= 422_212_466 (0x192_4924)
//
// PTP discipline:
//   effective_increment = nominal_increment + delta(ppb)
//   delta(ppb)         = (ppb * nominal_increment) / 1e9
//
// The audio frame tick (one pulse per LRCLK frame) drives RTP TX,
// guaranteeing packets leave on PTP-aligned sample boundaries.

module audio_clk_gen #(
    parameter SLOTS_PER_FRAME = 8,
    parameter BITS_PER_SLOT   = 32
) (
    input  wire        clk125,
    input  wire        rst,

    // PTP timebase (for monitoring; the actual discipline path is
    // through the freq_adj_ppb signal below)
    input  wire [47:0] ptp_sec,
    input  wire [31:0] ptp_nsec,

    // Active discipline from the PTP servo
    input  wire signed [31:0] freq_adj_ppb,
    input  wire        freq_adj_valid,

    // Base NCO increment (CPU-writable, set per sample rate)
    input  wire [31:0] nco_increment,
    input  wire [31:0] sample_rate,

    // Generated clocks (synchronous to clk125)
    output reg         bclk,
    output reg         lrclk,
    output reg         mclk,
    output reg         frame_pulse,
    output reg         slot_pulse,
    output reg [3:0]   slot_index,
    output reg [5:0]   bit_index,

    output reg [31:0]  rtp_sample_counter,
    output reg         frame_tick
);

    localparam BCLK_PER_FRAME = SLOTS_PER_FRAME * BITS_PER_SLOT;

    // ---- Compute disciplined increment ----
    //
    // delta_increment ≈ ppb * NOMINAL_INCREMENT / 1e9
    // For the nominal NCO target of 422_212_466 (12.288 MHz BCLK at 125 MHz):
    //   ppb * 422_212_466 / 1e9 ≈ ppb * 0.4222
    //                          ≈ ppb * 432 / 1024     (≤2% error)
    //                          = ((ppb<<8) + (ppb<<7) + (ppb<<5) + (ppb<<4)) >>> 10
    //
    // Constant-coefficient is purely shifts + adds — yosys synthesises it as
    // a 4-input adder with no DSP slice and no long carry chains. Meets
    // 125 MHz timing comfortably. The PTP servo iterates to absorb the
    // approximation error so absolute precision is fine.

    reg signed [31:0] delta_inc;
    reg [31:0]        effective_increment;

    wire signed [33:0] ppb_ext = $signed({{2{freq_adj_ppb[31]}}, freq_adj_ppb});
    wire signed [33:0] sum432  = (ppb_ext <<< 8) + (ppb_ext <<< 7) +
                                  (ppb_ext <<< 5) + (ppb_ext <<< 4);

    always @(posedge clk125) begin
        if (rst) begin
            delta_inc           <= 0;
            effective_increment <= 32'd422_212_466;
        end else begin
            if (freq_adj_valid)
                delta_inc <= (sum432 >>> 10);
            effective_increment <= nco_increment + delta_inc;
        end
    end

    // ---- BCLK NCO ----
    reg [31:0] bclk_phase;
    reg        bclk_phase_msb_d;

    // ---- MCLK NCO (256x sample rate, derived from BCLK x 4) ----
    reg [31:0] mclk_phase;
    reg        mclk_phase_msb_d;
    wire [31:0] mclk_increment = effective_increment << 2;

    always @(posedge clk125) begin
        if (rst) begin
            bclk_phase         <= 0;
            mclk_phase         <= 0;
            bclk               <= 0;
            mclk               <= 0;
            lrclk              <= 0;
            slot_index         <= 0;
            bit_index          <= 0;
            frame_pulse        <= 0;
            slot_pulse         <= 0;
            frame_tick         <= 0;
            rtp_sample_counter <= 0;
            bclk_phase_msb_d   <= 0;
            mclk_phase_msb_d   <= 0;
        end else begin
            frame_pulse <= 0;
            slot_pulse  <= 0;
            frame_tick  <= 0;

            bclk_phase       <= bclk_phase + effective_increment;
            bclk_phase_msb_d <= bclk_phase[31];

            mclk_phase       <= mclk_phase + mclk_increment;
            mclk_phase_msb_d <= mclk_phase[31];

            if (mclk_phase[31] != mclk_phase_msb_d)
                mclk <= ~mclk;

            // BCLK rising edge
            if (bclk_phase[31] && !bclk_phase_msb_d) begin
                bclk <= 1;
            end
            // BCLK falling edge - advance bit / slot / frame counters
            else if (!bclk_phase[31] && bclk_phase_msb_d) begin
                bclk <= 0;
                if (bit_index == BITS_PER_SLOT - 1) begin
                    bit_index  <= 0;
                    slot_pulse <= 1;
                    if (slot_index == SLOTS_PER_FRAME - 1) begin
                        slot_index         <= 0;
                        lrclk              <= ~lrclk;
                        frame_pulse        <= 1;
                        rtp_sample_counter <= rtp_sample_counter + 1;
                        frame_tick         <= 1;
                    end else begin
                        slot_index <= slot_index + 1;
                    end
                end else begin
                    bit_index <= bit_index + 1;
                end
            end
        end
    end

endmodule
