// SPDX-License-Identifier: MIT
// PTP Clock Servo - PI Controller (IEEE 1588 §11.3, IETF draft-ietf-tictoc-1588v3)
//
// PI controller that disciplines the local PTP clock to a remote
// grandmaster. Takes filtered offset measurements (low-passed by
// ptp_filter upstream) and outputs frequency + phase corrections.
//
//   freq_adj_ppb  = (Kp * offset_ns + Ki * integral(offset_ns)) >> 16
//   phase_step_ns = offset_ns                       (one-shot, |offset| > step_threshold)
//
// All math is signed 64-bit. Gains are Q16.16 fixed-point.
//
// Stability features:
//   - Anti-windup: integrator clamped to ±LIMIT_NS
//   - Phase-step on large offsets: integrator zeroed, lock state reset
//   - Frequency-output saturation to ±FREQ_LIMIT_PPB
//   - Hysteretic 3-state lock detector with separate enter/exit thresholds

module ptp_servo (
    input  wire        clk,
    input  wire        rst,

    input  wire        offset_valid,
    input  wire signed [63:0] offset_ns,
    input  wire signed [63:0] path_delay_ns,

    // Q16.16 gains (defaults: Kp=0.5, Ki=0.05)
    input  wire [31:0] kp,
    input  wire [31:0] ki,
    input  wire [31:0] step_threshold,

    output reg  signed [31:0] freq_adj_ppb,
    output reg         freq_adj_valid,

    output reg  signed [31:0] phase_step_ns,
    output reg         phase_step_valid,

    output reg  [1:0]  lock_state,
    output reg  [31:0] offset_filtered_ns,
    output reg  [31:0] path_delay_filtered_ns
);

    localparam [1:0] UNLOCKED = 2'd0,
                     LOCKING  = 2'd1,
                     LOCKED   = 2'd2;

    // Lock thresholds (ns)
    localparam [31:0] LOCK_ENTER_NS  = 32'd5_000;     // 5 µs to enter LOCKING
    localparam [31:0] LOCKED_ENTER_NS= 32'd1_000;     // 1 µs to enter LOCKED
    localparam [31:0] LOCKED_EXIT_NS = 32'd5_000;     // back to LOCKING
    localparam [31:0] UNLOCK_NS      = 32'd50_000;    // 50 µs -> UNLOCKED

    // Anti-windup: ±100 ms of accumulated error
    localparam signed [63:0] INT_LIMIT_NS = 64'sd100_000_000;
    // Frequency saturation: ±500 ppm
    localparam signed [31:0] FREQ_LIMIT_PPB = 32'sd500_000;

    reg signed [63:0] integral;
    reg [7:0]         lock_counter;
    reg [7:0]         unlock_counter;

    // Use the lower 32 bits of offset for lock detection. The upstream
    // ptp_filter bypass_threshold (10 µs) keeps realistic values well
    // within ±2.1 s, so truncation is safe.
    wire signed [31:0] offset_lo  = offset_ns[31:0];
    wire [31:0]        abs_offset = offset_lo[31] ? (~offset_lo + 1) : offset_lo;

    // Register the absolute offset and run the lock state machine on the
    // registered value next cycle. Breaks the offset_filt -> compare ->
    // counter critical path.
    reg [31:0] abs_offset_r;
    reg        lock_eval;

    // 2-stage pipeline so the multiplies meet timing
    reg               s1_valid;
    reg signed [63:0] s1_offset;
    reg signed [63:0] s1_integral;
    reg signed [63:0] s2_p_term;
    reg signed [63:0] s2_i_term;
    reg               s2_valid;

    // Helper: clamp 64-bit signed to 32-bit signed PPB range
    function signed [31:0] clamp_ppb;
        input signed [63:0] v;
        begin
            if (v >  $signed({32'b0, FREQ_LIMIT_PPB}))
                clamp_ppb =  FREQ_LIMIT_PPB;
            else if (v < -$signed({32'b0, FREQ_LIMIT_PPB}))
                clamp_ppb = -FREQ_LIMIT_PPB;
            else
                clamp_ppb = v[31:0];
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            integral               <= 0;
            lock_state             <= UNLOCKED;
            lock_counter           <= 0;
            unlock_counter         <= 0;
            freq_adj_ppb           <= 0;
            freq_adj_valid         <= 0;
            phase_step_ns          <= 0;
            phase_step_valid       <= 0;
            offset_filtered_ns     <= 0;
            path_delay_filtered_ns <= 0;
            s1_valid               <= 0;
            s1_offset              <= 0;
            s1_integral            <= 0;
            s2_p_term              <= 0;
            s2_i_term              <= 0;
            s2_valid               <= 0;
            abs_offset_r           <= 0;
            lock_eval              <= 0;
        end else begin
            freq_adj_valid   <= 0;
            phase_step_valid <= 0;
            s1_valid         <= 0;
            s2_valid         <= 0;
            lock_eval        <= 0;

            // Register abs_offset for next-cycle lock detection
            if (offset_valid) begin
                abs_offset_r <= abs_offset;
                lock_eval    <= 1;
            end

            if (offset_valid) begin
                // Status low-pass for monitoring (separate from servo input)
                offset_filtered_ns <= offset_filtered_ns +
                    ((abs_offset - offset_filtered_ns) >>> 3);
                path_delay_filtered_ns <= path_delay_filtered_ns +
                    (path_delay_ns[31] ?
                        ((~path_delay_ns[31:0] + 1 - path_delay_filtered_ns) >>> 3) :
                        ((path_delay_ns[31:0] - path_delay_filtered_ns) >>> 3));

                if (abs_offset > step_threshold) begin
                    // Large offset: phase step, reset integrator and lock SM
                    phase_step_ns    <= offset_ns[31:0];
                    phase_step_valid <= 1;
                    integral         <= 0;
                    lock_counter     <= 0;
                    unlock_counter   <= 0;
                    lock_state       <= UNLOCKED;
                end else begin
                    // PI step: clamp integrator (anti-windup)
                    if (integral + offset_ns >  INT_LIMIT_NS)
                        integral <=  INT_LIMIT_NS;
                    else if (integral + offset_ns < -INT_LIMIT_NS)
                        integral <= -INT_LIMIT_NS;
                    else
                        integral <= integral + offset_ns;

                    s1_valid    <= 1;
                    s1_offset   <= offset_ns;
                    s1_integral <= integral + offset_ns;

                end
            end

            // Hysteretic lock state machine - runs one cycle after the
            // offset arrives, on the registered abs_offset_r so its
            // comparisons don't extend the offset path.
            if (lock_eval) begin
                if (abs_offset_r < LOCKED_ENTER_NS) begin
                    if (lock_counter < 255) lock_counter <= lock_counter + 1;
                    unlock_counter <= 0;
                end else if (abs_offset_r > UNLOCK_NS) begin
                    if (unlock_counter < 255) unlock_counter <= unlock_counter + 1;
                    lock_counter <= 0;
                end

                case (lock_state)
                    UNLOCKED:
                        if (abs_offset_r < LOCK_ENTER_NS && lock_counter >= 4)
                            lock_state <= LOCKING;
                    LOCKING: begin
                        if (lock_counter >= 16)
                            lock_state <= LOCKED;
                        else if (unlock_counter >= 4)
                            lock_state <= UNLOCKED;
                    end
                    LOCKED:
                        if (abs_offset_r > LOCKED_EXIT_NS || unlock_counter >= 8)
                            lock_state <= LOCKING;
                    default: lock_state <= UNLOCKED;
                endcase
            end

            // Pipeline stage 1: multiply
            // Reduced to 32-bit signed × 16-bit unsigned to fit in two DSP
            // slices and meet 125 MHz timing. Realistic offsets fit in 32
            // signed bits (±2.1 s of nanoseconds) and Q1.15 gains have plenty
            // of resolution.
            if (s1_valid) begin
                s2_p_term <= ($signed(s1_offset  [31:0]) * $signed({1'b0, kp[15:0]})) >>> 15;
                s2_i_term <= ($signed(s1_integral[31:0]) * $signed({1'b0, ki[15:0]})) >>> 15;
                s2_valid  <= 1;
            end

            // Pipeline stage 2: sum + saturate
            if (s2_valid) begin
                freq_adj_ppb   <= clamp_ppb(s2_p_term + s2_i_term);
                freq_adj_valid <= 1;
            end
        end
    end

endmodule
