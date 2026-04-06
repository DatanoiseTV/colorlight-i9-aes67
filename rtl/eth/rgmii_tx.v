// SPDX-License-Identifier: MIT
// RGMII Transmit Interface for Lattice ECP5
//
// Serializes 8-bit GMII to DDR RGMII using the ECP5 ODDRX1F primitive
// (or a behavioral DDR mux for simulation). The GTXCLK output is the
// system 125 MHz clock phase-shifted by 90° so the data is stable at
// the PHY's sampling edge.

module rgmii_tx (
    input  wire       clk125,
    input  wire       clk125_90,    // 90° shifted, drives GTXCLK
    input  wire       rst,

    // Internal GMII-style interface
    input  wire [7:0] gmii_txd,
    input  wire       gmii_tx_en,
    input  wire       gmii_tx_err,

    // RGMII pins (directly to PHY)
    output wire       rgmii_gtxclk,
    output wire [3:0] rgmii_txd,
    output wire       rgmii_tx_ctl
);

    // GTXCLK is the 90° clock so it samples mid-data at the PHY
    assign rgmii_gtxclk = clk125_90;

    // Per-RGMII spec: rising edge carries TXD[3:0] and TX_EN,
    //                 falling edge carries TXD[7:4] and (TX_EN XOR TX_ERR).
    wire [3:0] txd_rise = gmii_txd[3:0];
    wire [3:0] txd_fall = gmii_txd[7:4];
    wire       ctl_rise = gmii_tx_en;
    wire       ctl_fall = gmii_tx_en ^ gmii_tx_err;

`ifdef SIM_PLL
    // ---- Behavioral DDR for simulation ----
    reg [3:0] txd_r;
    reg       ctl_r;
    always @(posedge clk125) begin
        if (rst) begin txd_r <= 0; ctl_r <= 0; end
        else     begin txd_r <= txd_rise; ctl_r <= ctl_rise; end
    end
    reg [3:0] txd_f;
    reg       ctl_f;
    always @(negedge clk125) begin
        if (rst) begin txd_f <= 0; ctl_f <= 0; end
        else     begin txd_f <= txd_fall; ctl_f <= ctl_fall; end
    end
    assign rgmii_txd    = clk125 ? txd_r : txd_f;
    assign rgmii_tx_ctl = clk125 ? ctl_r : ctl_f;
`else
    // ---- ECP5 ODDRX1F primitives (one per output bit) ----
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_txd_ddr
            ODDRX1F u_txd_ddr (
                .D0   (txd_rise[i]),
                .D1   (txd_fall[i]),
                .SCLK (clk125),
                .RST  (rst),
                .Q    (rgmii_txd[i])
            );
        end
    endgenerate

    ODDRX1F u_ctl_ddr (
        .D0   (ctl_rise),
        .D1   (ctl_fall),
        .SCLK (clk125),
        .RST  (rst),
        .Q    (rgmii_tx_ctl)
    );
`endif

endmodule
