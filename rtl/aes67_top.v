// SPDX-License-Identifier: MIT
// AES67 FPGA Top-Level
//
// Target: Colorlight i9 v7.2 (Lattice ECP5 LFE5U-45F)
//
// Architecture:
//   - 25 MHz input → ECP5 PLL → 125 MHz (RGMII), 125 MHz @ 90° (GTXCLK)
//   - Custom 1 Gbps Ethernet MAC (RGMII) with SFD pulses for PTP
//   - Hardware PTP engine (clock, timestamp, packet processor, servo)
//   - Hardware RTP engine (TX + RX + jitter buffer)
//   - PTP-locked audio NCO + I2S/TDM master
//   - Virtual I2S peripheral (CPU can mix into the audio path)
//   - cpu_netif (CPU shares the same MAC for DHCP, mDNS, HTTP, etc.)
//   - 3-way TX arbiter mixes PTP, RTP, and CPU traffic onto the MAC
//
// All AES67 timing-critical paths are in hardware. The CPU runs lwIP for
// management traffic on the same physical Ethernet, without touching the
// RTP data path.

module aes67_top (
    // Clock
    input  wire        clk25,

    // LED
    output wire        led,

    // Ethernet PHY 0 (primary - media + PTP + management)
    output wire        eth0_gtxclk,
    output wire [3:0]  eth0_txd,
    output wire        eth0_tx_en,
    input  wire        eth0_rxc,
    input  wire [3:0]  eth0_rxd,
    input  wire        eth0_rx_dv,

    // Ethernet PHY 1 (reserved for redundancy / second network)
    output wire        eth1_gtxclk,
    output wire [3:0]  eth1_txd,
    output wire        eth1_tx_en,
    input  wire        eth1_rxc,
    input  wire [3:0]  eth1_rxd,
    input  wire        eth1_rx_dv,

    // Shared MDIO
    inout  wire        eth_mdio,
    output wire        eth_mdc,
    output wire        eth_rst_n,

    // I2S / TDM audio interface
    output wire        i2s_mclk,
    output wire        i2s_bclk,
    output wire        i2s_lrck,
    output wire        i2s_dout,
    input  wire        i2s_din,

    // ---- CSR interface (driven by LiteX SoC wrapper) ----
    // Configuration inputs
    input  wire [63:0] cfg_local_clock_id,
    input  wire [47:0] cfg_local_mac,
    input  wire [31:0] cfg_local_ip,
    input  wire [31:0] cfg_rtp_mcast_ip,
    input  wire [47:0] cfg_rtp_dst_mac,
    input  wire [15:0] cfg_rtp_port,
    input  wire [31:0] cfg_kp,
    input  wire [31:0] cfg_ki,
    input  wire [31:0] cfg_step_threshold_ns,
    input  wire [31:0] cfg_tx_ssrc,
    input  wire [31:0] cfg_rx_expected_ssrc,
    input  wire [31:0] cfg_tx_ssrc_1,        // stream 1
    input  wire [31:0] cfg_rx_expected_ssrc_1,
    input  wire [6:0]  cfg_payload_type,
    input  wire [3:0]  cfg_num_channels,
    input  wire [10:0] cfg_samples_per_pkt,
    input  wire [9:0]  cfg_jbuf_target_depth,
    input  wire [31:0] cfg_nco_increment,
    input  wire signed [31:0] cfg_tx_delay_ns,    // PHY/cable TX asymmetry
    input  wire signed [31:0] cfg_rx_delay_ns,    // PHY/cable RX asymmetry
    input  wire [3:0]  cfg_filter_shift,          // PTP offset LP filter pole
    input  wire        cfg_mode_is_master,        // BMC: master vs slave
    input  wire [31:0] cfg_sync_interval_cycles,  // master Sync TX interval

    // Status outputs
    output wire [47:0] stat_ptp_sec,
    output wire [31:0] stat_ptp_nsec,
    output wire [1:0]  stat_ptp_lock_state,
    output wire [31:0] stat_ptp_offset_filt,
    output wire [31:0] stat_ptp_path_delay_filt,
    output wire [31:0] stat_ptp_sync_count,
    output wire [15:0] stat_ptp_last_seq,
    output wire [31:0] stat_rtp_packets_rx,
    output wire [31:0] stat_rtp_packets_tx,
    output wire [31:0] stat_rtp_seq_errors,
    output wire [10:0] stat_jbuf_depth,

    // ---- CPU network interface (BRAM ports + control) ----
    input  wire [10:0] cpunet_rx_addr,
    output wire [7:0]  cpunet_rx_data,
    input  wire [10:0] cpunet_tx_addr,
    input  wire [7:0]  cpunet_tx_wdata,
    input  wire        cpunet_tx_we,
    output wire        cpunet_rx_ready,
    output wire [15:0] cpunet_rx_length,
    input  wire        cpunet_rx_ack,
    output wire        cpunet_tx_pending,
    input  wire        cpunet_tx_send,
    input  wire [15:0] cpunet_tx_length,
    output wire        cpunet_irq,

    // ---- Virtual I2S (CPU audio access) ----
    input  wire        virtaud_tx_wr,
    input  wire [31:0] virtaud_tx_data,
    input  wire [3:0]  virtaud_tx_ch,
    output wire        virtaud_tx_full,
    input  wire        virtaud_rx_rd,
    output wire [31:0] virtaud_rx_data,
    output wire [3:0]  virtaud_rx_ch,
    output wire        virtaud_rx_empty,
    input  wire        virtaud_mix_enable,
    input  wire [15:0] virtaud_channel_mask,

    // ---- Audio DSP slot (per-channel gain/mute/meter) ----
    input  wire [3:0]  dsp_cfg_ch_sel,
    input  wire        dsp_gain_we,
    input  wire [15:0] dsp_gain_val,
    input  wire        dsp_mute_we,
    input  wire        dsp_mute_val,
    input  wire [3:0]  dsp_meter_ch_sel,
    output wire [23:0] dsp_meter_value,
    input  wire        dsp_meter_clear,

    // ---- Audio capture FIFO (browser preview) ----
    input  wire [15:0] cap_chan_mask,
    input  wire        cap_enable,
    input  wire        cap_rd,
    output wire [19:0] cap_rd_data,
    output wire        cap_empty,
    output wire        cap_overflow,

    // ---- MDIO management (CPU controls PHY registers) ----
    input  wire        mdio_cmd_start,
    input  wire        mdio_cmd_op_read,
    input  wire [4:0]  mdio_cmd_phy_addr,
    input  wire [4:0]  mdio_cmd_reg_addr,
    input  wire [15:0] mdio_cmd_write_data,
    output wire [15:0] mdio_read_data,
    output wire        mdio_busy,

    // ---- Multi-stream RTP stats (stream 1; stream 0 reuses existing stat_*) ----
    output wire [31:0] stat_rtp_packets_rx_1,
    output wire [31:0] stat_rtp_packets_tx_1,
    output wire [31:0] stat_rtp_seq_errors_1,
    output wire [10:0] stat_jbuf_depth_1
);

    // ---- Clock generation ----
    wire clk125;
    wire clk125_90;
    wire pll_locked;

    pll_25_to_125 u_pll (
        .clkin   (clk25),
        .clkout0 (clk125),
        .clkout1 (clk125_90),
        .locked  (pll_locked)
    );

    // Synchronous reset (async assert when PLL drops lock, sync release)
    wire rst;
    reset_sync u_rst_sync (
        .clk         (clk125),
        .async_rst_n (pll_locked),
        .sync_rst    (rst)
    );

    // ---- Ethernet MAC (PHY 0) ----
    wire [7:0] mac0_rx_tdata;
    wire       mac0_rx_tvalid, mac0_rx_tlast, mac0_rx_tuser, mac0_rx_tready;
    wire [7:0] mac0_tx_tdata;
    wire       mac0_tx_tvalid, mac0_tx_tlast, mac0_tx_tready;
    wire       eth0_rx_sfd, eth0_tx_sfd;

    eth_mac u_eth0 (
        .clk125          (clk125),
        .clk125_90       (clk125_90),
        .rst             (rst),
        .rgmii_gtxclk    (eth0_gtxclk),
        .rgmii_txd       (eth0_txd),
        .rgmii_tx_ctl    (eth0_tx_en),
        .rgmii_rxc       (eth0_rxc),
        .rgmii_rxd       (eth0_rxd),
        .rgmii_rx_ctl    (eth0_rx_dv),
        .rx_axis_tdata   (mac0_rx_tdata),
        .rx_axis_tvalid  (mac0_rx_tvalid),
        .rx_axis_tlast   (mac0_rx_tlast),
        .rx_axis_tuser   (mac0_rx_tuser),
        .rx_axis_tready  (mac0_rx_tready),
        .tx_axis_tdata   (mac0_tx_tdata),
        .tx_axis_tvalid  (mac0_tx_tvalid),
        .tx_axis_tlast   (mac0_tx_tlast),
        .tx_axis_tready  (mac0_tx_tready),
        .rx_sfd_pulse    (eth0_rx_sfd),
        .tx_sfd_pulse    (eth0_tx_sfd),
        .rx_frame_count  (),
        .rx_crc_err_count(),
        .tx_frame_count  ()
    );

    // ---- PTP Subsystem ----
    wire [7:0] ptp_rx_tdata;
    wire       ptp_rx_tvalid, ptp_rx_tlast, ptp_rx_tready;
    wire [7:0] ptp_payload_tdata;
    wire       ptp_payload_tvalid, ptp_payload_tlast, ptp_payload_tready;

    wire signed [31:0] ptp_freq_adj_ppb;
    wire        ptp_freq_adj_valid;

    ptp_top u_ptp (
        .clk                    (clk125),
        .rst                    (rst),
        .local_clock_id         (cfg_local_clock_id),
        .kp                     (cfg_kp),
        .ki                     (cfg_ki),
        .step_threshold_ns      (cfg_step_threshold_ns),
        .tx_delay_ns            (cfg_tx_delay_ns),
        .rx_delay_ns            (cfg_rx_delay_ns),
        .filter_shift           (cfg_filter_shift),
        .mode_is_master         (cfg_mode_is_master),
        .sync_interval_cycles   (cfg_sync_interval_cycles),
        .rx_axis_tdata          (ptp_rx_tdata),
        .rx_axis_tvalid         (ptp_rx_tvalid),
        .rx_axis_tlast          (ptp_rx_tlast),
        .rx_axis_tready         (ptp_rx_tready),
        .rx_sfd_pulse           (eth0_rx_sfd),
        .tx_axis_tdata          (ptp_payload_tdata),
        .tx_axis_tvalid         (ptp_payload_tvalid),
        .tx_axis_tlast          (ptp_payload_tlast),
        .tx_axis_tready         (ptp_payload_tready),
        .tx_sfd_pulse           (eth0_tx_sfd),
        .ptp_sec                (stat_ptp_sec),
        .ptp_nsec               (stat_ptp_nsec),
        .pps                    (),
        .freq_adj_ppb           (ptp_freq_adj_ppb),
        .freq_adj_valid         (ptp_freq_adj_valid),
        .lock_state             (stat_ptp_lock_state),
        .offset_filtered_ns     (stat_ptp_offset_filt),
        .path_delay_filtered_ns (stat_ptp_path_delay_filt),
        .sync_count             (stat_ptp_sync_count),
        .last_seq_id            (stat_ptp_last_seq)
    );

    // PTP TX wrapper: prepend Eth/IP/UDP headers (port 319 = PTP event)
    wire [7:0] ptp_eth_tdata;
    wire       ptp_eth_tvalid, ptp_eth_tlast, ptp_eth_tready;
    wire       ptp_wrap_busy;

    // PTP packet length is fixed at 44 bytes (Delay_Req message)
    // The wrapper accepts a `start` pulse with this length
    reg ptp_start;
    always @(posedge clk125)
        ptp_start <= ptp_payload_tvalid & ~ptp_wrap_busy & ~ptp_start;

    tx_udp_wrapper u_ptp_wrap (
        .clk             (clk125),
        .rst             (rst),
        .start           (ptp_start),
        .payload_len     (16'd44),
        .dst_mac         (48'h01_1B_19_00_00_00),  // PTP multicast MAC
        .src_mac         (cfg_local_mac),
        .dst_ip          (32'hE0_00_01_81),        // 224.0.1.129
        .src_ip          (cfg_local_ip),
        .dst_port        (16'd319),
        .src_port        (16'd319),
        .busy            (ptp_wrap_busy),
        .pl_axis_tdata   (ptp_payload_tdata),
        .pl_axis_tvalid  (ptp_payload_tvalid),
        .pl_axis_tlast   (ptp_payload_tlast),
        .pl_axis_tready  (ptp_payload_tready),
        .mac_axis_tdata  (ptp_eth_tdata),
        .mac_axis_tvalid (ptp_eth_tvalid),
        .mac_axis_tlast  (ptp_eth_tlast),
        .mac_axis_tready (ptp_eth_tready)
    );

    // ---- Audio clock generator ----
    wire        bclk_int, lrclk_int, mclk_int;
    wire        slot_pulse, frame_pulse, frame_tick;
    wire [3:0]  slot_index;
    wire [5:0]  bit_index;
    wire [31:0] rtp_sample_counter;

    audio_clk_gen #(
        .SLOTS_PER_FRAME(8),
        .BITS_PER_SLOT  (32)
    ) u_audio_clk (
        .clk125             (clk125),
        .rst                (rst),
        .ptp_sec            (stat_ptp_sec),
        .ptp_nsec           (stat_ptp_nsec),
        .freq_adj_ppb       (ptp_freq_adj_ppb),
        .freq_adj_valid     (ptp_freq_adj_valid),
        .nco_increment      (cfg_nco_increment),
        .sample_rate        (32'd48000),
        .bclk               (bclk_int),
        .lrclk              (lrclk_int),
        .mclk               (mclk_int),
        .frame_pulse        (frame_pulse),
        .slot_pulse         (slot_pulse),
        .slot_index         (slot_index),
        .bit_index          (bit_index),
        .rtp_sample_counter (rtp_sample_counter),
        .frame_tick         (frame_tick)
    );

    // ---- I2S/TDM Master ----
    wire [3:0]  tdm_tx_slot, tdm_rx_slot;
    wire        tdm_tx_rd, tdm_rx_wr;
    wire [23:0] tdm_tx_sample_rtp;     // raw from RTP RX path
    wire [23:0] tdm_tx_sample_dsp;     // after audio_dsp (gain/mute)
    wire [23:0] tdm_tx_sample_mixed;   // after virt_tdm16 CPU mix
    wire [23:0] tdm_rx_sample;

    // ---- Audio DSP slot (per-channel gain/mute/meter) ----
    audio_dsp #(
        .NUM_CHANNELS(16)
    ) u_dsp (
        .clk          (clk125),
        .rst          (rst),
        .cfg_ch_sel   (dsp_cfg_ch_sel),
        .cfg_gain_we  (dsp_gain_we),
        .cfg_gain_val (dsp_gain_val),
        .cfg_mute_we  (dsp_mute_we),
        .cfg_mute_val (dsp_mute_val),
        .meter_ch_sel (dsp_meter_ch_sel),
        .meter_value  (dsp_meter_value),
        .meter_clear  (dsp_meter_clear),
        .audio_ch     (tdm_tx_slot),
        .audio_in     (tdm_tx_sample_rtp),
        .audio_tick   (tdm_tx_rd),
        .audio_out    (tdm_tx_sample_dsp)
    );

    // ---- Audio capture FIFO (browser preview) ----
    audio_capture #(
        .NUM_CHANNELS(16),
        .FIFO_ADDR_W (10)
    ) u_cap (
        .clk          (clk125),
        .rst          (rst),
        .src_ch       (tdm_tx_slot),
        .src_sample   (tdm_tx_sample_dsp),
        .src_tick     (tdm_tx_rd),
        .channel_mask (cap_chan_mask),
        .enable       (cap_enable),
        .cpu_rd       (cap_rd),
        .cpu_rd_data  (cap_rd_data),
        .cpu_empty    (cap_empty),
        .cpu_level    (),
        .cpu_overflow (cap_overflow)
    );

    i2s_tdm_master #(
        .MAX_SLOTS   (8),
        .SAMPLE_WIDTH(24),
        .SLOT_BITS   (32)
    ) u_i2s_tdm (
        .clk125         (clk125),
        .rst            (rst),
        .bclk_int       (bclk_int),
        .lrclk_int      (lrclk_int),
        .slot_pulse     (slot_pulse),
        .frame_pulse    (frame_pulse),
        .slot_index     (slot_index),
        .bit_index      (bit_index),
        .format_mode    (2'd1),
        .num_slots      (4'd8),
        .sample_bits    (6'd24),
        .pin_bclk       (i2s_bclk),
        .pin_lrck       (i2s_lrck),
        .pin_dout       (i2s_dout),
        .pin_din        (i2s_din),
        .tx_slot_idx    (tdm_tx_slot),
        .tx_sample_rd   (tdm_tx_rd),
        .tx_sample_data (tdm_tx_sample_mixed),
        .rx_slot_idx    (tdm_rx_slot),
        .rx_sample_wr   (tdm_rx_wr),
        .rx_sample_data (tdm_rx_sample)
    );

    assign i2s_mclk = mclk_int;

    // ---- Virtual I2S (CPU mix-in) ----
    virt_tdm16 #(
        .NUM_CHANNELS(16),
        .FIFO_ADDR_W (8)
    ) u_virt_tdm16 (
        .clk              (clk125),
        .rst              (rst),
        .cpu_tx_wr        (virtaud_tx_wr),
        .cpu_tx_data      (virtaud_tx_data),
        .cpu_tx_ch        (virtaud_tx_ch),
        .cpu_tx_full      (virtaud_tx_full),
        .cpu_tx_level     (),
        .cpu_rx_rd        (virtaud_rx_rd),
        .cpu_rx_data      (virtaud_rx_data),
        .cpu_rx_ch        (virtaud_rx_ch),
        .cpu_rx_empty     (virtaud_rx_empty),
        .cpu_rx_level     (),
        .mix_enable       (virtaud_mix_enable),
        .cpu_channel_mask (virtaud_channel_mask),
        .audio_tx_ch      (tdm_tx_slot),
        .audio_tx_in      (tdm_tx_sample_dsp),
        .audio_tx_out     (tdm_tx_sample_mixed),
        .audio_tx_tick    (tdm_tx_rd),
        .audio_rx_ch      (tdm_rx_slot),
        .audio_rx_in      (tdm_rx_sample),
        .audio_rx_tick    (tdm_rx_wr)
    );

    // ---- Multi-Stream RTP Engine (NUM_STREAMS independent streams) ----
    localparam NUM_STREAMS         = 2;
    localparam CHANNELS_PER_STREAM = 4;   // 2 streams × 4ch = 8 TDM slots

    wire [7:0] rtp_rx_tdata;
    wire       rtp_rx_tvalid, rtp_rx_tlast, rtp_rx_tready;

    // Per-stream TX UDP payload outputs (flattened bus)
    wire [NUM_STREAMS*8-1:0] ms_tx_tdata;
    wire [NUM_STREAMS-1:0]   ms_tx_tvalid, ms_tx_tlast, ms_tx_tready;

    // Pack the per-stream config into the flat buses rtp_multistream expects
    wire [NUM_STREAMS*32-1:0] ms_tx_ssrc       = {cfg_tx_ssrc_1,        cfg_tx_ssrc};
    wire [NUM_STREAMS*32-1:0] ms_rx_ssrc       = {cfg_rx_expected_ssrc_1,cfg_rx_expected_ssrc};
    wire [NUM_STREAMS*7-1:0]  ms_payload_type  = {cfg_payload_type, cfg_payload_type};
    wire [NUM_STREAMS*4-1:0]  ms_num_channels  = {cfg_num_channels, cfg_num_channels};
    wire [NUM_STREAMS*11-1:0] ms_spp           = {cfg_samples_per_pkt, cfg_samples_per_pkt};
    wire [NUM_STREAMS*10-1:0] ms_jbuf_depth    = {cfg_jbuf_target_depth, cfg_jbuf_target_depth};

    wire [NUM_STREAMS*32-1:0] ms_rx_packets, ms_tx_packets, ms_seq_err;
    wire [NUM_STREAMS*11-1:0] ms_jbuf_d;

    rtp_multistream #(
        .NUM_STREAMS         (NUM_STREAMS),
        .SAMPLE_WIDTH        (24),
        .CHANNELS_PER_STREAM (CHANNELS_PER_STREAM),
        .JBUF_ADDR_W         (10)
    ) u_rtp_ms (
        .clk                  (clk125),
        .rst                  (rst),
        .rx_enable            ({NUM_STREAMS{1'b1}}),
        .tx_enable            ({NUM_STREAMS{1'b1}}),
        .tx_ssrc              (ms_tx_ssrc),
        .rx_expected_ssrc     (ms_rx_ssrc),
        .payload_type         (ms_payload_type),
        .num_channels         (ms_num_channels),
        .samples_per_packet   (ms_spp),
        .jbuf_target_depth    (ms_jbuf_depth),
        .rx_axis_tdata        (rtp_rx_tdata),
        .rx_axis_tvalid       (rtp_rx_tvalid),
        .rx_axis_tlast        (rtp_rx_tlast),
        .rx_axis_tready       (rtp_rx_tready),
        .tx_axis_tdata        (ms_tx_tdata),
        .tx_axis_tvalid       (ms_tx_tvalid),
        .tx_axis_tlast        (ms_tx_tlast),
        .tx_axis_tready       (ms_tx_tready),
        .audio_rd_ch          (tdm_tx_slot),
        .audio_rd_en          (tdm_tx_rd),
        .audio_rd_data        (tdm_tx_sample_rtp),
        .audio_wr_data        (tdm_rx_sample),
        .audio_wr_addr        ({7'd0, tdm_rx_slot}),
        .audio_wr_en          (tdm_rx_wr),
        .send_packet_tick     (frame_tick),
        .rtp_timestamp        (rtp_sample_counter),
        .rx_packets_received  (ms_rx_packets),
        .tx_packets_sent      (ms_tx_packets),
        .rx_seq_errors        (ms_seq_err),
        .jbuf_depth           (ms_jbuf_d)
    );

    assign stat_rtp_packets_rx   = ms_rx_packets[0*32 +: 32];
    assign stat_rtp_packets_tx   = ms_tx_packets[0*32 +: 32];
    assign stat_rtp_seq_errors   = ms_seq_err   [0*32 +: 32];
    assign stat_jbuf_depth       = ms_jbuf_d    [0*11 +: 11];
    assign stat_rtp_packets_rx_1 = ms_rx_packets[1*32 +: 32];
    assign stat_rtp_packets_tx_1 = ms_tx_packets[1*32 +: 32];
    assign stat_rtp_seq_errors_1 = ms_seq_err   [1*32 +: 32];
    assign stat_jbuf_depth_1     = ms_jbuf_d    [1*11 +: 11];

    // ---- Per-stream TX wrappers (Eth/IP/UDP header prepend) ----
    wire [7:0] rtp0_eth_tdata, rtp1_eth_tdata;
    wire       rtp0_eth_tvalid, rtp0_eth_tlast, rtp0_eth_tready;
    wire       rtp1_eth_tvalid, rtp1_eth_tlast, rtp1_eth_tready;
    wire       rtp0_wrap_busy, rtp1_wrap_busy;
    reg        rtp0_start, rtp1_start;

    wire [15:0] rtp_payload_len =
        16'd12 + ({5'd0, cfg_samples_per_pkt} * {12'd0, cfg_num_channels} * 16'd3);

    always @(posedge clk125) begin
        rtp0_start <= ms_tx_tvalid[0] & ~rtp0_wrap_busy & ~rtp0_start;
        rtp1_start <= ms_tx_tvalid[1] & ~rtp1_wrap_busy & ~rtp1_start;
    end

    tx_udp_wrapper u_rtp_wrap0 (
        .clk             (clk125),
        .rst             (rst),
        .start           (rtp0_start),
        .payload_len     (rtp_payload_len),
        .dst_mac         (cfg_rtp_dst_mac),
        .src_mac         (cfg_local_mac),
        .dst_ip          (cfg_rtp_mcast_ip),
        .src_ip          (cfg_local_ip),
        .dst_port        (cfg_rtp_port),
        .src_port        (cfg_rtp_port),
        .busy            (rtp0_wrap_busy),
        .pl_axis_tdata   (ms_tx_tdata[0*8 +: 8]),
        .pl_axis_tvalid  (ms_tx_tvalid[0]),
        .pl_axis_tlast   (ms_tx_tlast[0]),
        .pl_axis_tready  (ms_tx_tready[0]),
        .mac_axis_tdata  (rtp0_eth_tdata),
        .mac_axis_tvalid (rtp0_eth_tvalid),
        .mac_axis_tlast  (rtp0_eth_tlast),
        .mac_axis_tready (rtp0_eth_tready)
    );

    tx_udp_wrapper u_rtp_wrap1 (
        .clk             (clk125),
        .rst             (rst),
        .start           (rtp1_start),
        .payload_len     (rtp_payload_len),
        .dst_mac         (cfg_rtp_dst_mac),
        .src_mac         (cfg_local_mac),
        .dst_ip          (cfg_rtp_mcast_ip),
        .src_ip          (cfg_local_ip),
        .dst_port        (cfg_rtp_port + 16'd2),    // stream 1 on next port
        .src_port        (cfg_rtp_port + 16'd2),
        .busy            (rtp1_wrap_busy),
        .pl_axis_tdata   (ms_tx_tdata[1*8 +: 8]),
        .pl_axis_tvalid  (ms_tx_tvalid[1]),
        .pl_axis_tlast   (ms_tx_tlast[1]),
        .pl_axis_tready  (ms_tx_tready[1]),
        .mac_axis_tdata  (rtp1_eth_tdata),
        .mac_axis_tvalid (rtp1_eth_tvalid),
        .mac_axis_tlast  (rtp1_eth_tlast),
        .mac_axis_tready (rtp1_eth_tready)
    );

    // ---- Merge per-stream RTP TX into a single line ----
    wire [7:0] rtp_eth_tdata;
    wire       rtp_eth_tvalid, rtp_eth_tlast, rtp_eth_tready;

    tx_arbiter2 u_rtp_arb (
        .clk            (clk125),
        .rst            (rst),
        .s0_axis_tdata  (rtp0_eth_tdata),
        .s0_axis_tvalid (rtp0_eth_tvalid),
        .s0_axis_tlast  (rtp0_eth_tlast),
        .s0_axis_tready (rtp0_eth_tready),
        .s1_axis_tdata  (rtp1_eth_tdata),
        .s1_axis_tvalid (rtp1_eth_tvalid),
        .s1_axis_tlast  (rtp1_eth_tlast),
        .s1_axis_tready (rtp1_eth_tready),
        .m_axis_tdata   (rtp_eth_tdata),
        .m_axis_tvalid  (rtp_eth_tvalid),
        .m_axis_tlast   (rtp_eth_tlast),
        .m_axis_tready  (rtp_eth_tready)
    );

    // ---- Packet Router (RX classification) ----
    wire [7:0] cpu_rx_tdata, cpu_tx_tdata;
    wire       cpu_rx_tvalid, cpu_rx_tlast, cpu_rx_tready;
    wire       cpu_tx_tvalid, cpu_tx_tlast, cpu_tx_tready;

    packet_router u_router (
        .clk           (clk125),
        .rst           (rst),
        .mac_rx_tdata  (mac0_rx_tdata),
        .mac_rx_tvalid (mac0_rx_tvalid),
        .mac_rx_tlast  (mac0_rx_tlast),
        .mac_rx_tuser  (mac0_rx_tuser),
        .mac_rx_tready (mac0_rx_tready),
        .ptp_rx_tdata  (ptp_rx_tdata),
        .ptp_rx_tvalid (ptp_rx_tvalid),
        .ptp_rx_tlast  (ptp_rx_tlast),
        .ptp_rx_tready (ptp_rx_tready),
        .rtp_rx_tdata  (rtp_rx_tdata),
        .rtp_rx_tvalid (rtp_rx_tvalid),
        .rtp_rx_tlast  (rtp_rx_tlast),
        .rtp_rx_tready (rtp_rx_tready),
        .cpu_rx_tdata  (cpu_rx_tdata),
        .cpu_rx_tvalid (cpu_rx_tvalid),
        .cpu_rx_tlast  (cpu_rx_tlast),
        .cpu_rx_tready (cpu_rx_tready),
        .local_mac     (cfg_local_mac),
        .local_ip      (cfg_local_ip),
        .rtp_mcast_ip  (cfg_rtp_mcast_ip),
        .rtp_port      (cfg_rtp_port)
    );

    // ---- CPU netif ----
    cpu_netif u_cpu_netif (
        .clk            (clk125),
        .rst            (rst),
        .rx_axis_tdata  (cpu_rx_tdata),
        .rx_axis_tvalid (cpu_rx_tvalid),
        .rx_axis_tlast  (cpu_rx_tlast),
        .rx_axis_tready (cpu_rx_tready),
        .tx_axis_tdata  (cpu_tx_tdata),
        .tx_axis_tvalid (cpu_tx_tvalid),
        .tx_axis_tlast  (cpu_tx_tlast),
        .tx_axis_tready (cpu_tx_tready),
        .rx_mem_addr    (cpunet_rx_addr),
        .rx_mem_data    (cpunet_rx_data),
        .tx_mem_addr    (cpunet_tx_addr),
        .tx_mem_wdata   (cpunet_tx_wdata),
        .tx_mem_we      (cpunet_tx_we),
        .rx_ready       (cpunet_rx_ready),
        .rx_length      (cpunet_rx_length),
        .rx_ack         (cpunet_rx_ack),
        .tx_pending     (cpunet_tx_pending),
        .tx_send        (cpunet_tx_send),
        .tx_length      (cpunet_tx_length),
        .irq            (cpunet_irq)
    );

    // ---- TX Arbiter (PTP > RTP > CPU) ----
    tx_arbiter u_tx_arb (
        .clk            (clk125),
        .rst            (rst),
        .s0_axis_tdata  (ptp_eth_tdata),
        .s0_axis_tvalid (ptp_eth_tvalid),
        .s0_axis_tlast  (ptp_eth_tlast),
        .s0_axis_tready (ptp_eth_tready),
        .s1_axis_tdata  (rtp_eth_tdata),
        .s1_axis_tvalid (rtp_eth_tvalid),
        .s1_axis_tlast  (rtp_eth_tlast),
        .s1_axis_tready (rtp_eth_tready),
        .s2_axis_tdata  (cpu_tx_tdata),
        .s2_axis_tvalid (cpu_tx_tvalid),
        .s2_axis_tlast  (cpu_tx_tlast),
        .s2_axis_tready (cpu_tx_tready),
        .m_axis_tdata   (mac0_tx_tdata),
        .m_axis_tvalid  (mac0_tx_tvalid),
        .m_axis_tlast   (mac0_tx_tlast),
        .m_axis_tready  (mac0_tx_tready)
    );

    // ---- PHY 1 (unused for now - tied off) ----
    assign eth1_gtxclk = 1'b0;
    assign eth1_txd    = 4'h0;
    assign eth1_tx_en  = 1'b0;

    // ---- MDIO Master (Clause 22) ----
    assign eth_rst_n = pll_locked;

    wire        mdio_oe, mdio_o, mdio_i;
    wire        mdc_w;

    mdio_master #(
        .MDC_DIV(50)        // 1.25 MHz MDC at 125 MHz
    ) u_mdio (
        .clk             (clk125),
        .rst             (rst),
        .cmd_start       (mdio_cmd_start),
        .cmd_op_read     (mdio_cmd_op_read),
        .cmd_phy_addr    (mdio_cmd_phy_addr),
        .cmd_reg_addr    (mdio_cmd_reg_addr),
        .cmd_write_data  (mdio_cmd_write_data),
        .read_data       (mdio_read_data),
        .busy            (mdio_busy),
        .mdc             (mdc_w),
        .mdio_oe         (mdio_oe),
        .mdio_o          (mdio_o),
        .mdio_i          (mdio_i)
    );

    assign eth_mdc  = mdc_w;
    assign eth_mdio = mdio_oe ? mdio_o : 1'bz;
    assign mdio_i   = eth_mdio;

    // ---- Status LED ----
    reg [25:0] led_cnt;
    always @(posedge clk125) led_cnt <= led_cnt + 1;
    assign led = (stat_ptp_lock_state == 2'd2) ? led_cnt[25] : led_cnt[22];

endmodule
