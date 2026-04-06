// SPDX-License-Identifier: MIT
// PTP Clock Servo — PI Controller
//
// Disciplines the local PTP clock to a remote grandmaster.
// Takes offset measurements and outputs frequency + phase corrections.
//
// PI Controller:
//   freq_adj  = Kp * offset + Ki * integral(offset)
//   phase_adj = offset (one-shot correction when |offset| > threshold)
//
// All arithmetic is 64-bit signed to handle nanosecond-scale values.
// Gains are in Q16.16 fixed-point.

module ptp_servo (
    input  wire        clk,
    input  wire        rst,

    // Offset measurement input (from PTP packet processor)
    input  wire        offset_valid,        // pulse: new measurement ready
    input  wire signed [63:0] offset_ns,    // measured offset in nanoseconds (signed)
    input  wire signed [63:0] path_delay_ns,// measured path delay in nanoseconds

    // Servo gains (Q16.16 fixed-point, set by CPU)
    input  wire [31:0] kp,                  // proportional gain (default: 0x00010000 = 1.0)
    input  wire [31:0] ki,                  // integral gain     (default: 0x00000100 ≈ 0.004)

    // Phase step threshold (ns) — if |offset| > threshold, apply one-shot step
    input  wire [31:0] step_threshold,      // default: 1000 (1 µs)

    // Output: frequency adjustment (ppb, signed, added to clock increment)
    output reg  signed [31:0] freq_adj_ppb,
    output reg         freq_adj_valid,

    // Output: phase step correction (ns, signed, one-shot)
    output reg  signed [31:0] phase_step_ns,
    output reg         phase_step_valid,

    // Lock status
    output reg  [1:0]  lock_state,          // 0=UNLOCKED, 1=LOCKING, 2=LOCKED
    output reg  [31:0] offset_filtered_ns,  // low-pass filtered |offset| for monitoring
    output reg  [31:0] path_delay_filtered_ns
);

    localparam [1:0] UNLOCKED = 2'd0,
                     LOCKING  = 2'd1,
                     LOCKED   = 2'd2;

    // Lock thresholds (nanoseconds)
    localparam LOCK_THRESHOLD   = 32'd1_000;      // 1 µs to enter LOCKING
    localparam LOCKED_THRESHOLD = 32'd100;         // 100 ns to enter LOCKED
    localparam UNLOCK_THRESHOLD = 32'd10_000;      // 10 µs to go back to UNLOCKED

    // Internal state
    reg signed [63:0] integral;             // accumulated offset integral
    reg signed [63:0] last_offset;
    reg [7:0]         lock_counter;         // consecutive good measurements
    reg [7:0]         unlock_counter;       // consecutive bad measurements

    // Absolute value helper
    wire [63:0] abs_offset = offset_ns[63] ? (~offset_ns + 1) : offset_ns;

    // PI computation (pipelined)
    reg        compute_stage;
    reg signed [63:0] p_term;
    reg signed [63:0] i_term;

    always @(posedge clk) begin
        if (rst) begin
            integral             <= 0;
            last_offset          <= 0;
            lock_state           <= UNLOCKED;
            lock_counter         <= 0;
            unlock_counter       <= 0;
            freq_adj_ppb         <= 0;
            freq_adj_valid       <= 0;
            phase_step_ns        <= 0;
            phase_step_valid     <= 0;
            offset_filtered_ns   <= 0;
            path_delay_filtered_ns <= 0;
            compute_stage        <= 0;
            p_term               <= 0;
            i_term               <= 0;
        end else begin
            freq_adj_valid   <= 0;
            phase_step_valid <= 0;

            if (offset_valid) begin
                last_offset <= offset_ns;

                // Low-pass filter for monitoring: y = y + (x - y) >> 3
                offset_filtered_ns <= offset_filtered_ns +
                    ((abs_offset[31:0] - offset_filtered_ns) >>> 3);
                path_delay_filtered_ns <= path_delay_filtered_ns +
                    ((path_delay_ns[31:0] - path_delay_filtered_ns) >>> 3);

                // Check if offset is large enough for phase step
                if (abs_offset > {32'd0, step_threshold}) begin
                    // Large offset: apply one-shot phase correction
                    phase_step_ns    <= offset_ns[31:0];
                    phase_step_valid <= 1;
                    integral         <= 0; // reset integrator after step
                    lock_counter     <= 0;
                    unlock_counter   <= 0;
                    lock_state       <= UNLOCKED;
                end else begin
                    // Normal PI operation
                    // Update integral with anti-windup (clamp)
                    if (integral + offset_ns > 64'sh0000_0000_3B9A_CA00)       // +1s
                        integral <= 64'sh0000_0000_3B9A_CA00;
                    else if (integral + offset_ns < -64'sh0000_0000_3B9A_CA00) // -1s
                        integral <= -64'sh0000_0000_3B9A_CA00;
                    else
                        integral <= integral + offset_ns;

                    // P term: (kp * offset) >> 16  (Q16.16 multiply)
                    p_term <= (offset_ns * $signed({1'b0, kp})) >>> 16;
                    // I term: (ki * integral) >> 16
                    i_term <= (integral * $signed({1'b0, ki})) >>> 16;

                    compute_stage <= 1;

                    // Lock state machine
                    if (abs_offset < {32'd0, LOCKED_THRESHOLD}) begin
                        if (lock_counter < 255) lock_counter <= lock_counter + 1;
                        unlock_counter <= 0;
                    end else if (abs_offset > {32'd0, UNLOCK_THRESHOLD}) begin
                        if (unlock_counter < 255) unlock_counter <= unlock_counter + 1;
                        lock_counter <= 0;
                    end

                    case (lock_state)
                        UNLOCKED: begin
                            if (abs_offset < {32'd0, LOCK_THRESHOLD} && lock_counter >= 4)
                                lock_state <= LOCKING;
                        end
                        LOCKING: begin
                            if (lock_counter >= 10)
                                lock_state <= LOCKED;
                            else if (unlock_counter >= 3)
                                lock_state <= UNLOCKED;
                        end
                        LOCKED: begin
                            if (unlock_counter >= 5)
                                lock_state <= LOCKING;
                        end
                        default: lock_state <= UNLOCKED;
                    endcase
                end
            end

            // Pipeline stage 2: sum P + I and output
            if (compute_stage) begin
                compute_stage  <= 0;
                freq_adj_ppb   <= p_term[31:0] + i_term[31:0];
                freq_adj_valid <= 1;
            end
        end
    end

endmodule
