// SPDX-License-Identifier: MIT
// Complete Ethernet MAC with RGMII interface
// Instantiates RGMII PHY interface, MAC RX, MAC TX
// Provides AXI-Stream interfaces and PTP timestamp triggers

module eth_mac (
    input  wire        clk125,      // 125 MHz system clock
    input  wire        clk125_90,   // 125 MHz 90-degree phase shifted
    input  wire        rst,

    // RGMII PHY interface
    output wire        rgmii_gtxclk,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_tx_ctl,
    input  wire        rgmii_rxc,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rx_ctl,

    // AXI-Stream RX output
    output wire [7:0]  rx_axis_tdata,
    output wire        rx_axis_tvalid,
    output wire        rx_axis_tlast,
    output wire        rx_axis_tuser,  // CRC error flag
    input  wire        rx_axis_tready,

    // AXI-Stream TX input
    input  wire [7:0]  tx_axis_tdata,
    input  wire        tx_axis_tvalid,
    input  wire        tx_axis_tlast,
    output wire        tx_axis_tready,

    // PTP timestamp triggers (active for 1 clock cycle)
    output wire        rx_sfd_pulse,
    output wire        tx_sfd_pulse,

    // Statistics
    output wire [31:0] rx_frame_count,
    output wire [31:0] rx_crc_err_count,
    output wire [31:0] tx_frame_count
);

    // Internal GMII signals
    wire [7:0] gmii_rxd;
    wire       gmii_rx_dv;
    wire       gmii_rx_err;
    wire       gmii_rx_clk;

    wire [7:0] gmii_txd;
    wire       gmii_tx_en;
    wire       gmii_tx_err;

    // RGMII RX
    rgmii_rx u_rgmii_rx (
        .clk125       (clk125),
        .rst          (rst),
        .rgmii_rxc    (rgmii_rxc),
        .rgmii_rxd    (rgmii_rxd),
        .rgmii_rx_ctl (rgmii_rx_ctl),
        .gmii_rxd     (gmii_rxd),
        .gmii_rx_dv   (gmii_rx_dv),
        .gmii_rx_err  (gmii_rx_err),
        .gmii_rx_clk  (gmii_rx_clk)
    );

    // RGMII TX
    rgmii_tx u_rgmii_tx (
        .clk125       (clk125),
        .clk125_90    (clk125_90),
        .rst          (rst),
        .gmii_txd     (gmii_txd),
        .gmii_tx_en   (gmii_tx_en),
        .gmii_tx_err  (gmii_tx_err),
        .rgmii_gtxclk (rgmii_gtxclk),
        .rgmii_txd    (rgmii_txd),
        .rgmii_tx_ctl (rgmii_tx_ctl)
    );

    // MAC RX
    eth_mac_rx u_mac_rx (
        .clk             (clk125),
        .rst             (rst),
        .gmii_rxd        (gmii_rxd),
        .gmii_rx_dv      (gmii_rx_dv),
        .gmii_rx_err     (gmii_rx_err),
        .m_axis_tdata    (rx_axis_tdata),
        .m_axis_tvalid   (rx_axis_tvalid),
        .m_axis_tlast    (rx_axis_tlast),
        .m_axis_tuser    (rx_axis_tuser),
        .m_axis_tready   (rx_axis_tready),
        .sfd_pulse       (rx_sfd_pulse),
        .rx_frame_count  (rx_frame_count),
        .rx_crc_err_count(rx_crc_err_count)
    );

    // MAC TX
    eth_mac_tx u_mac_tx (
        .clk            (clk125),
        .rst            (rst),
        .s_axis_tdata   (tx_axis_tdata),
        .s_axis_tvalid  (tx_axis_tvalid),
        .s_axis_tlast   (tx_axis_tlast),
        .s_axis_tready  (tx_axis_tready),
        .gmii_txd       (gmii_txd),
        .gmii_tx_en     (gmii_tx_en),
        .gmii_tx_err    (gmii_tx_err),
        .sfd_pulse      (tx_sfd_pulse),
        .tx_frame_count (tx_frame_count)
    );

endmodule
