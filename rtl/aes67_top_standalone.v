// SPDX-License-Identifier: MIT
// Standalone wrapper around aes67_top for synthesis verification.
//
// aes67_top is normally instantiated by the LiteX SoC, which connects all
// the CSR/status ports to its CPU bus internally. For standalone yosys+
// nextpnr verification (`make synth`), we need to tie those ports off so
// they don't end up as physical pins (which would exceed the BG381 IO
// budget). This wrapper does the bare minimum to let nextpnr place-and-
// route the design with only physical pins exposed.

module aes67_top_standalone (
    input  wire        clk25,
    output wire        led,

    output wire        eth0_gtxclk,
    output wire [3:0]  eth0_txd,
    output wire        eth0_tx_en,
    input  wire        eth0_rxc,
    input  wire [3:0]  eth0_rxd,
    input  wire        eth0_rx_dv,

    output wire        eth1_gtxclk,
    output wire [3:0]  eth1_txd,
    output wire        eth1_tx_en,
    input  wire        eth1_rxc,
    input  wire [3:0]  eth1_rxd,
    input  wire        eth1_rx_dv,

    inout  wire        eth_mdio,
    output wire        eth_mdc,
    output wire        eth_rst_n,

    output wire        i2s_mclk,
    output wire        i2s_bclk,
    output wire        i2s_lrck,
    output wire        i2s_dout,
    input  wire        i2s_din
);

    // Tied-off CSR/status ports (LiteX would normally drive these)
    wire [23:0] dsp_meter_value_w;
    wire [19:0] cap_rd_data_w;
    wire        cap_empty_w, cap_overflow_w;
    wire [47:0] stat_ptp_sec_w;
    wire [31:0] stat_ptp_nsec_w;
    wire [1:0]  stat_ptp_lock_state_w;
    wire [31:0] stat_ptp_offset_filt_w;
    wire [31:0] stat_ptp_path_delay_filt_w;
    wire [31:0] stat_ptp_sync_count_w;
    wire [15:0] stat_ptp_last_seq_w;
    wire [31:0] stat_rtp_packets_rx_w;
    wire [31:0] stat_rtp_packets_tx_w;
    wire [31:0] stat_rtp_seq_errors_w;
    wire [10:0] stat_jbuf_depth_w;
    wire [7:0]  cpunet_rx_data_w;
    wire        cpunet_rx_ready_w;
    wire [15:0] cpunet_rx_length_w;
    wire        cpunet_tx_pending_w;
    wire        cpunet_irq_w;
    wire        virtaud_tx_full_w;
    wire [31:0] virtaud_rx_data_w;
    wire [3:0]  virtaud_rx_ch_w;
    wire        virtaud_rx_empty_w;

    aes67_top u_top (
        .clk25            (clk25),
        .led              (led),

        .eth0_gtxclk      (eth0_gtxclk),
        .eth0_txd         (eth0_txd),
        .eth0_tx_en       (eth0_tx_en),
        .eth0_rxc         (eth0_rxc),
        .eth0_rxd         (eth0_rxd),
        .eth0_rx_dv       (eth0_rx_dv),

        .eth1_gtxclk      (eth1_gtxclk),
        .eth1_txd         (eth1_txd),
        .eth1_tx_en       (eth1_tx_en),
        .eth1_rxc         (eth1_rxc),
        .eth1_rxd         (eth1_rxd),
        .eth1_rx_dv       (eth1_rx_dv),

        .eth_mdio         (eth_mdio),
        .eth_mdc          (eth_mdc),
        .eth_rst_n        (eth_rst_n),

        .i2s_mclk         (i2s_mclk),
        .i2s_bclk         (i2s_bclk),
        .i2s_lrck         (i2s_lrck),
        .i2s_dout         (i2s_dout),
        .i2s_din          (i2s_din),

        // ---- CSR config inputs (tie to defaults) ----
        .cfg_local_clock_id     (64'h00_11_22_FF_FE_33_44_55),
        .cfg_local_mac          (48'h02_AE_67_00_00_01),
        .cfg_local_ip           (32'd0),
        .cfg_rtp_mcast_ip       (32'hEF45_0001),
        .cfg_rtp_dst_mac        (48'h0100_5E45_0001),
        .cfg_rtp_port           (16'd5004),
        .cfg_kp                 (32'h00008000),
        .cfg_ki                 (32'h00000CCC),
        .cfg_step_threshold_ns  (32'd1000),
        .cfg_tx_ssrc            (32'hDEADBEEF),
        .cfg_rx_expected_ssrc   (32'd0),
        .cfg_tx_ssrc_1          (32'hCAFEBABE),
        .cfg_rx_expected_ssrc_1 (32'd0),
        .cfg_payload_type       (7'd98),
        .cfg_num_channels       (4'd2),
        .cfg_samples_per_pkt    (11'd6),
        .cfg_jbuf_target_depth  (10'd24),
        .cfg_nco_increment      (32'd422_212_466),
        .cfg_tx_delay_ns        (32'd0),
        .cfg_rx_delay_ns        (32'd0),
        .cfg_filter_shift       (4'd4),
        .cfg_mode_is_master     (1'b0),
        .cfg_sync_interval_cycles(32'd15_625_000),

        // ---- Status outputs (no-connect) ----
        .stat_ptp_sec             (stat_ptp_sec_w),
        .stat_ptp_nsec            (stat_ptp_nsec_w),
        .stat_ptp_lock_state      (stat_ptp_lock_state_w),
        .stat_ptp_offset_filt     (stat_ptp_offset_filt_w),
        .stat_ptp_path_delay_filt (stat_ptp_path_delay_filt_w),
        .stat_ptp_sync_count      (stat_ptp_sync_count_w),
        .stat_ptp_last_seq        (stat_ptp_last_seq_w),
        .stat_rtp_packets_rx      (stat_rtp_packets_rx_w),
        .stat_rtp_packets_tx      (stat_rtp_packets_tx_w),
        .stat_rtp_seq_errors      (stat_rtp_seq_errors_w),
        .stat_jbuf_depth          (stat_jbuf_depth_w),

        // ---- CPU netif (idle) ----
        .cpunet_rx_addr    (11'd0),
        .cpunet_rx_data    (cpunet_rx_data_w),
        .cpunet_tx_addr    (11'd0),
        .cpunet_tx_wdata   (8'd0),
        .cpunet_tx_we      (1'b0),
        .cpunet_rx_ready   (cpunet_rx_ready_w),
        .cpunet_rx_length  (cpunet_rx_length_w),
        .cpunet_rx_ack     (1'b0),
        .cpunet_tx_pending (cpunet_tx_pending_w),
        .cpunet_tx_send    (1'b0),
        .cpunet_tx_length  (16'd0),
        .cpunet_irq        (cpunet_irq_w),

        // ---- Virtual TDM-16 (idle) ----
        .virtaud_tx_wr        (1'b0),
        .virtaud_tx_data      (32'd0),
        .virtaud_tx_ch        (4'd0),
        .virtaud_tx_full      (virtaud_tx_full_w),
        .virtaud_rx_rd        (1'b0),
        .virtaud_rx_data      (virtaud_rx_data_w),
        .virtaud_rx_ch        (virtaud_rx_ch_w),
        .virtaud_rx_empty     (virtaud_rx_empty_w),
        .virtaud_mix_enable   (1'b0),
        .virtaud_channel_mask (16'd0),

        // ---- DSP slot (idle) ----
        .dsp_cfg_ch_sel    (4'd0),
        .dsp_gain_we       (1'b0),
        .dsp_gain_val      (16'd0),
        .dsp_mute_we       (1'b0),
        .dsp_mute_val      (1'b0),
        .dsp_meter_ch_sel  (4'd0),
        .dsp_meter_value   (dsp_meter_value_w),
        .dsp_meter_clear   (1'b0),

        // ---- Audio capture (idle) ----
        .cap_chan_mask  (16'd0),
        .cap_enable     (1'b0),
        .cap_rd         (1'b0),
        .cap_rd_data    (cap_rd_data_w),
        .cap_empty      (cap_empty_w),
        .cap_overflow   (cap_overflow_w),

        // ---- MDIO (idle) ----
        .mdio_cmd_start        (1'b0),
        .mdio_cmd_op_read      (1'b1),
        .mdio_cmd_phy_addr     (5'd0),
        .mdio_cmd_reg_addr     (5'd0),
        .mdio_cmd_write_data   (16'd0),
        .mdio_read_data        (),
        .mdio_busy             (),

        // ---- Multi-stream stats (no-connect) ----
        .stat_rtp_packets_rx_1 (),
        .stat_rtp_packets_tx_1 (),
        .stat_rtp_seq_errors_1 (),
        .stat_jbuf_depth_1     ()
    );

endmodule
