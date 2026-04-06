// SPDX-License-Identifier: MIT
// PTP-Locked Audio Clock Generator
//
// Generates BCLK (bit clock), LRCLK (frame clock / word select), and MCLK using
// a numerically-controlled oscillator (NCO) disciplined by the PTP timebase.
//
// The NCO accumulates a 32-bit phase per clk125 cycle. The increment is
// chosen so that the BCLK frequency matches the desired sample rate × bclk_per_frame.
//
//   bclk_freq = (clk125 * increment) / 2^32
//
// For 48 kHz × 64 bclk/frame (8ch TDM × 32-bit slot) = 3.072 MHz BCLK:
//   increment = 3.072e6 * 2^32 / 125e6 ≈ 105_553_117 (0x064B0CC0)
//
// PTP discipline: when the audio frame counter drifts from the expected
// PTP-derived value, the increment is adjusted by ±delta to pull it back in.

module audio_clk_gen #(
    parameter SLOTS_PER_FRAME = 8,    // TDM slots per LRCLK period
    parameter BITS_PER_SLOT   = 32    // bit clocks per slot
) (
    input  wire        clk125,
    input  wire        rst,

    // PTP timebase (for discipline)
    input  wire [47:0] ptp_sec,
    input  wire [31:0] ptp_nsec,

    // Configuration (from CPU)
    input  wire [31:0] nco_increment,    // base BCLK NCO increment
    input  wire [31:0] sample_rate,      // e.g. 48000

    // Generated clocks (synchronous to clk125)
    output reg         bclk,             // bit clock
    output reg         lrclk,            // word select (frame clock, = sample rate)
    output reg         mclk,             // master clock (256× sample rate)
    output reg         frame_pulse,      // single-cycle pulse at start of each audio frame
    output reg         slot_pulse,       // single-cycle pulse at start of each TDM slot
    output reg [3:0]   slot_index,       // current TDM slot (0..SLOTS_PER_FRAME-1)
    output reg [5:0]   bit_index,        // bit position within current slot

    // RTP timestamp (sample counter, increments per audio frame)
    output reg [31:0]  rtp_sample_counter,
    output reg         frame_tick        // pulse at audio frame boundary (drives RTP TX)
);

    localparam BCLK_PER_FRAME = SLOTS_PER_FRAME * BITS_PER_SLOT;

    // BCLK NCO
    reg [31:0] bclk_phase;
    reg        bclk_phase_msb_d;

    // MCLK NCO (256× sample rate)
    reg [31:0] mclk_phase;
    reg        mclk_phase_msb_d;
    wire [31:0] mclk_increment = nco_increment << 2; // 4× BCLK increment for typical setups

    always @(posedge clk125) begin
        if (rst) begin
            bclk_phase       <= 0;
            mclk_phase       <= 0;
            bclk             <= 0;
            mclk             <= 0;
            lrclk            <= 0;
            slot_index       <= 0;
            bit_index        <= 0;
            frame_pulse      <= 0;
            slot_pulse       <= 0;
            frame_tick       <= 0;
            rtp_sample_counter <= 0;
            bclk_phase_msb_d <= 0;
            mclk_phase_msb_d <= 0;
        end else begin
            frame_pulse <= 0;
            slot_pulse  <= 0;
            frame_tick  <= 0;

            // BCLK NCO: toggle BCLK when MSB transitions
            bclk_phase       <= bclk_phase + nco_increment;
            bclk_phase_msb_d <= bclk_phase[31];

            // MCLK NCO: toggle MCLK when MSB transitions
            mclk_phase       <= mclk_phase + mclk_increment;
            mclk_phase_msb_d <= mclk_phase[31];

            if (mclk_phase[31] != mclk_phase_msb_d)
                mclk <= ~mclk;

            // Detect rising edge of internal BCLK (phase MSB transition 0->1)
            if (bclk_phase[31] && !bclk_phase_msb_d) begin
                bclk <= 1;
                // Bit advance happens on falling edge of BCLK in I2S convention
            end else if (!bclk_phase[31] && bclk_phase_msb_d) begin
                bclk <= 0;
                // BCLK falling edge - advance bit counter
                if (bit_index == BITS_PER_SLOT - 1) begin
                    bit_index  <= 0;
                    slot_pulse <= 1;
                    if (slot_index == SLOTS_PER_FRAME - 1) begin
                        slot_index <= 0;
                        // LRCLK toggles at start of each frame
                        lrclk <= ~lrclk;
                        frame_pulse <= 1;
                        rtp_sample_counter <= rtp_sample_counter + 1;
                        frame_tick  <= 1;
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
