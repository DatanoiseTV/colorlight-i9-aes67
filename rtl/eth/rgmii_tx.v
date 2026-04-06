// SPDX-License-Identifier: MIT
// RGMII Transmit Interface for Lattice ECP5
// Serializes 8-bit GMII to DDR RGMII
// Uses ECP5 ODDRX1F primitives for DDR output

module rgmii_tx (
    input  wire       clk125,       // 125 MHz system clock
    input  wire       clk125_90,    // 125 MHz shifted 90 degrees (for GTXCLK)
    input  wire       rst,

    // Internal GMII-style interface
    input  wire [7:0] gmii_txd,
    input  wire       gmii_tx_en,
    input  wire       gmii_tx_err,

    // RGMII pins (directly to PHY)
    output wire       rgmii_gtxclk, // TX clock to PHY (125 MHz, 90deg shifted)
    output reg  [3:0] rgmii_txd,    // TX data (DDR)
    output reg        rgmii_tx_ctl  // TX control (DDR: EN on rising, EN^ERR on falling)
);

    // GTXCLK is clk125 phase-shifted by 90 degrees
    // This ensures data is stable at the PHY's sampling edge
    // In ECP5, use ODDRX1F or route through PLL with phase shift
    assign rgmii_gtxclk = clk125_90;

    // DDR serialization
    // Rising edge: lower nibble [3:0], TX_EN
    // Falling edge: upper nibble [7:4], TX_EN ^ TX_ERR

    reg [3:0] txd_rise, txd_fall;
    reg       ctl_rise, ctl_fall;

    always @(*) begin
        txd_rise = gmii_txd[3:0];
        txd_fall = gmii_txd[7:4];
        ctl_rise = gmii_tx_en;
        ctl_fall = gmii_tx_en ^ gmii_tx_err;
    end

    // Output DDR registers
    // ECP5 ODDRX1F: drives D0 on rising edge, D1 on falling edge
    // Behavioral equivalent:

    always @(posedge clk125) begin
        if (rst) begin
            rgmii_txd   <= 4'h0;
            rgmii_tx_ctl <= 1'b0;
        end else begin
            rgmii_txd    <= txd_rise;
            rgmii_tx_ctl <= ctl_rise;
        end
    end

    // On falling edge, output high nibble
    always @(negedge clk125) begin
        if (rst) begin
            rgmii_txd    <= 4'h0;
            rgmii_tx_ctl <= 1'b0;
        end else begin
            rgmii_txd    <= txd_fall;
            rgmii_tx_ctl <= ctl_fall;
        end
    end

endmodule
