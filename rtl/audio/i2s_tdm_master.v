// SPDX-License-Identifier: MIT
// I2S / TDM Master
//
// Configurable: I2S (2 channels per LRCLK period) or TDM (up to 8/16 channels per period)
// Operates as bus master (generates BCLK, LRCLK, MCLK).
//
// Modes:
//   FORMAT_I2S       : 1-bit BCLK delay from LRCLK edge, MSB-first, LRCLK low = left
//   FORMAT_TDM_SHORT : LRCLK is a 1-bit pulse at start of frame
//   FORMAT_TDM_LONG  : LRCLK is high for first slot, low for rest
//
// Provides:
//   - TX: read samples from external buffer, shift out on DOUT
//   - RX: shift in from DIN, write samples to external buffer
//
// Up to 16 TDM slots × 32 bits each. Sample width within slot is configurable.

module i2s_tdm_master #(
    parameter MAX_SLOTS    = 16,
    parameter SAMPLE_WIDTH = 24,
    parameter SLOT_BITS    = 32
) (
    input  wire        clk125,
    input  wire        rst,

    // Generated clocks (from audio_clk_gen)
    input  wire        bclk_int,        // internal bit clock (toggles each transition)
    input  wire        lrclk_int,       // internal frame clock
    input  wire        slot_pulse,      // pulse at start of each slot
    input  wire        frame_pulse,     // pulse at start of each frame
    input  wire [3:0]  slot_index,      // current slot
    input  wire [5:0]  bit_index,       // current bit within slot

    // Configuration
    input  wire [1:0]  format_mode,     // 0=I2S, 1=TDM_SHORT, 2=TDM_LONG
    input  wire [3:0]  num_slots,       // active slots per frame (1..16)
    input  wire [5:0]  sample_bits,     // 16/24/32

    // External pins
    output reg         pin_bclk,
    output reg         pin_lrck,
    output reg         pin_dout,
    input  wire        pin_din,

    // TX sample interface (one sample per slot)
    output reg  [3:0]  tx_slot_idx,     // which slot is being shifted out
    output reg         tx_sample_rd,    // pulse to fetch next sample
    input  wire [SAMPLE_WIDTH-1:0] tx_sample_data,

    // RX sample interface
    output reg  [3:0]  rx_slot_idx,
    output reg         rx_sample_wr,
    output reg  [SAMPLE_WIDTH-1:0] rx_sample_data
);

    // Shift registers
    reg [SLOT_BITS-1:0] tx_shift;
    reg [SLOT_BITS-1:0] rx_shift;

    // Drive output pins synchronously
    always @(posedge clk125) begin
        if (rst) begin
            pin_bclk <= 0;
            pin_lrck <= 0;
            pin_dout <= 0;
        end else begin
            pin_bclk <= bclk_int;

            // LRCLK shape depends on format
            case (format_mode)
                2'd0: pin_lrck <= lrclk_int;                            // I2S: 50% duty
                2'd1: pin_lrck <= frame_pulse;                          // TDM short: 1 pulse
                2'd2: pin_lrck <= (slot_index == 0);                    // TDM long: high for slot 0
                default: pin_lrck <= lrclk_int;
            endcase

            // Output current MSB
            pin_dout <= tx_shift[SLOT_BITS-1];
        end
    end

    // ---- TX shifting ----
    // At the start of each slot, load the new sample (left-justified)
    // On each BCLK falling edge (advance), shift left
    reg bclk_d;
    wire bclk_falling = bclk_d & ~bclk_int;
    wire bclk_rising  = ~bclk_d & bclk_int;

    always @(posedge clk125) begin
        if (rst) begin
            bclk_d       <= 0;
            tx_shift     <= 0;
            tx_slot_idx  <= 0;
            tx_sample_rd <= 0;
            rx_shift     <= 0;
            rx_slot_idx  <= 0;
            rx_sample_wr <= 0;
            rx_sample_data <= 0;
        end else begin
            bclk_d       <= bclk_int;
            tx_sample_rd <= 0;
            rx_sample_wr <= 0;

            // At slot boundary, latch the next TX sample
            if (slot_pulse) begin
                tx_slot_idx  <= slot_index;
                tx_sample_rd <= 1;
                // Load sample left-justified into shift register
                tx_shift <= {tx_sample_data, {(SLOT_BITS - SAMPLE_WIDTH){1'b0}}};
            end else if (bclk_falling) begin
                // Shift out next bit on falling edge
                tx_shift <= {tx_shift[SLOT_BITS-2:0], 1'b0};
            end

            // ---- RX shifting (sample on rising edge of BCLK) ----
            if (bclk_rising) begin
                rx_shift <= {rx_shift[SLOT_BITS-2:0], pin_din};
            end

            // At end of slot (just before next slot_pulse), commit RX sample
            if (slot_pulse && slot_index != 0) begin
                // The previous slot just finished
                rx_sample_data <= rx_shift[SLOT_BITS-1 -: SAMPLE_WIDTH];
                rx_slot_idx    <= slot_index - 1;
                rx_sample_wr   <= 1;
            end
        end
    end

endmodule
