// SPDX-License-Identifier: MIT
// PTP Offset Low-Pass Filter
//
// First-order IIR (single-pole) on the offset measurement to reject
// per-sample noise from the master/network jitter before it reaches
// the PI servo. Pole position is selected by `shift` (right shift),
// so:
//   y[n] = y[n-1] + ((x[n] - y[n-1]) >>> shift)
// shift = 0  -> y = x (no filtering)
// shift = 4  -> ~16-sample time constant (default)
// shift = 8  -> ~256-sample time constant (very smooth)
//
// All math is signed 64-bit. The filter passes through unfiltered when
// the magnitude of the input exceeds `bypass_threshold` so that big
// step changes (clock jumps) are not delayed.

module ptp_filter (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid,
    input  wire signed [63:0] in,
    input  wire [3:0]  shift,             // pole selector
    input  wire [63:0] bypass_threshold,  // unsigned magnitude
    output reg         out_valid,
    output reg signed [63:0] out
);

    reg signed [63:0] state;
    wire signed [63:0] err = in - state;
    wire [63:0] abs_in = in[63] ? (~in + 1) : in;

    always @(posedge clk) begin
        if (rst) begin
            state     <= 0;
            out       <= 0;
            out_valid <= 0;
        end else begin
            out_valid <= 0;
            if (valid) begin
                if (abs_in > bypass_threshold) begin
                    state <= in;
                    out   <= in;
                end else begin
                    state <= state + (err >>> shift);
                    out   <= state + (err >>> shift);
                end
                out_valid <= 1;
            end
        end
    end

endmodule
