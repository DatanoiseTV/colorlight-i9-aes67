// SPDX-License-Identifier: MIT
// RGMII Receive Interface for Lattice ECP5
// Deserializes DDR RGMII to 8-bit Gigabit MII
// Uses ECP5 IDDRX1F primitives for DDR input sampling

module rgmii_rx (
    input  wire       clk125,       // 125 MHz system clock
    input  wire       rst,

    // RGMII pins (directly from PHY)
    input  wire       rgmii_rxc,    // RX clock from PHY (125 MHz DDR)
    input  wire [3:0] rgmii_rxd,    // RX data (DDR)
    input  wire       rgmii_rx_ctl, // RX control (DDR: DV on rising, DV^ERR on falling)

    // Internal GMII-style interface (synchronous to clk125)
    output reg  [7:0] gmii_rxd,
    output reg        gmii_rx_dv,
    output reg        gmii_rx_err,
    output reg        gmii_rx_clk   // recovered clock for timestamp reference
);

    // DDR deserialization
    // ECP5 IDDRX1F: Q0 = data sampled on rising edge, Q1 = data sampled on falling edge
    wire [3:0] rxd_rise, rxd_fall;
    wire       ctl_rise, ctl_fall;

    // Use the RX clock from the PHY as the sampling clock
    // In real implementation, these would be IDDRX1F primitives:
    //   IDDRX1F iddr_rxd0 (.D(rgmii_rxd[0]), .SCLK(rgmii_rxc), .RST(rst),
    //                       .Q0(rxd_rise[0]), .Q1(rxd_fall[0]));
    // For synthesis portability, we use behavioral DDR:

    reg [3:0] rxd_r, rxd_f;
    reg       ctl_r, ctl_f;

    // Sample on rising edge of RX clock
    always @(posedge rgmii_rxc) begin
        rxd_r <= rgmii_rxd;
        ctl_r <= rgmii_rx_ctl;
    end

    // Sample on falling edge of RX clock
    always @(negedge rgmii_rxc) begin
        rxd_f <= rgmii_rxd;
        ctl_f <= rgmii_rx_ctl;
    end

    assign rxd_rise = rxd_r;
    assign rxd_fall = rxd_f;
    assign ctl_rise = ctl_r;
    assign ctl_fall = ctl_f;

    // Cross to clk125 domain with 2-stage sync
    reg [3:0] rxd_rise_s1, rxd_rise_s2;
    reg [3:0] rxd_fall_s1, rxd_fall_s2;
    reg       ctl_rise_s1, ctl_rise_s2;
    reg       ctl_fall_s1, ctl_fall_s2;

    always @(posedge clk125) begin
        if (rst) begin
            rxd_rise_s1 <= 0; rxd_rise_s2 <= 0;
            rxd_fall_s1 <= 0; rxd_fall_s2 <= 0;
            ctl_rise_s1 <= 0; ctl_rise_s2 <= 0;
            ctl_fall_s1 <= 0; ctl_fall_s2 <= 0;
        end else begin
            rxd_rise_s1 <= rxd_rise; rxd_rise_s2 <= rxd_rise_s1;
            rxd_fall_s1 <= rxd_fall; rxd_fall_s2 <= rxd_fall_s1;
            ctl_rise_s1 <= ctl_rise; ctl_rise_s2 <= ctl_rise_s1;
            ctl_fall_s1 <= ctl_fall; ctl_fall_s2 <= ctl_fall_s1;
        end
    end

    // Reconstruct GMII signals
    // RGMII: rising edge = bits [3:0], falling edge = bits [7:4]
    // RGMII rx_ctl: rising = DV, falling = DV XOR ERR
    always @(posedge clk125) begin
        if (rst) begin
            gmii_rxd    <= 8'h0;
            gmii_rx_dv  <= 1'b0;
            gmii_rx_err <= 1'b0;
            gmii_rx_clk <= 1'b0;
        end else begin
            gmii_rxd    <= {rxd_fall_s2, rxd_rise_s2};
            gmii_rx_dv  <= ctl_rise_s2;
            gmii_rx_err <= ctl_rise_s2 ^ ctl_fall_s2; // DV XOR (DV^ERR) = ERR
            gmii_rx_clk <= ~gmii_rx_clk; // toggle for reference
        end
    end

endmodule
