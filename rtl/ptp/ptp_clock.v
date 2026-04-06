// SPDX-License-Identifier: MIT
// IEEE 1588 PTP Hardware Clock
//
// 96-bit timestamp: 48-bit seconds + 32-bit nanoseconds
// Plus 32-bit sub-nanosecond fractional accumulator for fine frequency adjustment
//
// Frequency control: increment value in 8.24 fixed-point nanoseconds per clock cycle
//   - At 125 MHz nominal: increment = 8.0 ns = 32'h08_000000
//   - Adjusting by +1 LSB ≈ +0.06 ppb (parts per billion)
//
// Phase control: one-shot nanosecond offset correction (positive or negative)
//
// Wishbone register interface for CPU access

module ptp_clock (
    input  wire        clk,          // 125 MHz
    input  wire        rst,

    // Timestamp output (active every cycle)
    output reg  [47:0] ts_sec,       // seconds
    output reg  [31:0] ts_nsec,      // nanoseconds (0 to 999_999_999)
    output reg  [31:0] ts_frac,      // sub-nanosecond fraction (for servo)

    // Frequency adjustment (from servo or CPU)
    input  wire [31:0] increment,    // 8.24 fixed-point ns per clock (default 0x08000000)
    input  wire        increment_wr, // pulse to latch new increment

    // Phase adjustment (one-shot, signed)
    input  wire        phase_adj_en,
    input  wire signed [31:0] phase_adj_ns, // signed nanosecond correction

    // Seconds set (for initial time from CPU/PTP)
    input  wire        sec_set_en,
    input  wire [47:0] sec_set_val,

    // PPS output (pulse-per-second, 1 cycle wide at second rollover)
    output reg         pps
);

    localparam [31:0] NS_PER_SEC     = 32'd1_000_000_000;
    localparam [31:0] DEFAULT_INCR   = 32'h08_000000; // 8.0 ns at 125 MHz

    reg [31:0] incr_reg;

    // Accumulator: 32-bit fractional + 32-bit nanosecond
    reg [63:0] accum; // [63:32] = nanoseconds, [31:0] = fraction

    // Latch increment
    always @(posedge clk) begin
        if (rst)
            incr_reg <= DEFAULT_INCR;
        else if (increment_wr)
            incr_reg <= increment;
    end

    // Main clock accumulator
    always @(posedge clk) begin
        if (rst) begin
            ts_sec  <= 48'd0;
            ts_nsec <= 32'd0;
            ts_frac <= 32'd0;
            pps     <= 1'b0;
        end else begin
            pps <= 1'b0;

            // Set seconds (from CPU or PTP master)
            if (sec_set_en) begin
                ts_sec <= sec_set_val;
            end

            // Phase adjustment (one-shot correction)
            if (phase_adj_en) begin
                // Apply signed offset to nanoseconds
                if (phase_adj_ns[31]) begin
                    // Negative offset
                    if (ts_nsec < (~phase_adj_ns + 1)) begin
                        // Borrow from seconds
                        ts_nsec <= NS_PER_SEC - (~phase_adj_ns + 1) + ts_nsec;
                        ts_sec  <= ts_sec - 1;
                    end else begin
                        ts_nsec <= ts_nsec + phase_adj_ns; // add negative = subtract
                    end
                end else begin
                    // Positive offset
                    ts_nsec <= ts_nsec + phase_adj_ns;
                end
            end else begin
                // Normal increment
                {ts_nsec, ts_frac} <= {ts_nsec, ts_frac} + {32'd0, incr_reg};
            end

            // Second rollover
            if (ts_nsec >= NS_PER_SEC) begin
                ts_nsec <= ts_nsec - NS_PER_SEC;
                ts_sec  <= ts_sec + 1;
                pps     <= 1'b1;
            end
        end
    end

endmodule
