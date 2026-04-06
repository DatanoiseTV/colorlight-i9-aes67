// SPDX-License-Identifier: MIT
// Testbench for PTP clock module

`timescale 1ns / 1ps

module tb_ptp_clock;

    reg         clk = 0;
    reg         rst = 1;
    wire [47:0] ts_sec;
    wire [31:0] ts_nsec;
    wire [31:0] ts_frac;
    reg  [31:0] increment    = 32'h08000000;
    reg         increment_wr = 0;
    reg         phase_adj_en = 0;
    reg signed [31:0] phase_adj_ns = 0;
    reg         sec_set_en   = 0;
    reg  [47:0] sec_set_val  = 0;
    wire        pps;

    always #4 clk = ~clk;  // 125 MHz

    ptp_clock dut (
        .clk          (clk),
        .rst          (rst),
        .ts_sec       (ts_sec),
        .ts_nsec      (ts_nsec),
        .ts_frac      (ts_frac),
        .increment    (increment),
        .increment_wr (increment_wr),
        .phase_adj_en (phase_adj_en),
        .phase_adj_ns (phase_adj_ns),
        .sec_set_en   (sec_set_en),
        .sec_set_val  (sec_set_val),
        .pps          (pps)
    );

    initial begin
        $dumpfile("tb_ptp_clock.vcd");
        $dumpvars(0, tb_ptp_clock);

        #100 rst = 0;

        // Run for 2 seconds (simulated)
        #2_000_000_000;

        $display("Final: sec=%d nsec=%d", ts_sec, ts_nsec);
        $finish;
    end

    always @(posedge pps)
        $display("PPS at sec=%d", ts_sec);

endmodule
