#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# LiteX SoC for AES67 on Colorlight i9 v7.2
#
# Architecture:
#   - VexRiscv soft CPU running lwIP for management plane
#   - 8 MB SDRAM for code/data/heap
#   - AES67 hardware engine (PTP, RTP, audio) as a CSR-mapped peripheral
#   - cpu_netif BRAM-mapped Ethernet interface (lwIP netif)
#   - Virtual I2S peripheral for CPU audio access (mixes into RTP path)
#
# Build:
#   python3 soc.py --build           # generate gateware + firmware skeleton
#   python3 soc.py --build --load    # build and program SRAM
#   python3 soc.py --build --flash   # build and program SPI flash

import os
from migen import *

from litex.build.generic_platform import *
from litex.build.lattice import LatticePlatform

from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSR

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY


# ---------------------------------------------------------------------------
# Platform definition for the Colorlight i9 v7.2
# ---------------------------------------------------------------------------
_io = [
    ("clk25", 0, Pins("P3"), IOStandard("LVCMOS33")),
    ("user_led_n", 0, Pins("L2"), IOStandard("LVCMOS33")),

    # Ethernet PHY 0 (RGMII)
    ("eth0_rgmii", 0,
        Subsignal("gtxclk", Pins("U19")),
        Subsignal("txd",    Pins("U20 T19 T20 R20")),
        Subsignal("tx_en",  Pins("P19")),
        Subsignal("rxc",    Pins("L19")),
        Subsignal("rxd",    Pins("P20 N19 N20 M19")),
        Subsignal("rx_dv",  Pins("M20")),
        IOStandard("LVCMOS33"),
    ),

    # Ethernet PHY 1 (RGMII)
    ("eth1_rgmii", 0,
        Subsignal("gtxclk", Pins("G1")),
        Subsignal("txd",    Pins("G2 H1 J1 J3")),
        Subsignal("tx_en",  Pins("K1")),
        Subsignal("rxc",    Pins("H2")),
        Subsignal("rxd",    Pins("K2 L1 N1 P1")),
        Subsignal("rx_dv",  Pins("P2")),
        IOStandard("LVCMOS33"),
    ),

    # Shared MDIO + reset
    ("eth_mdio", 0, Pins("N5"),  IOStandard("LVCMOS33")),
    ("eth_mdc",  0, Pins("P5"),  IOStandard("LVCMOS33")),
    ("eth_rst_n",0, Pins("P4"),  IOStandard("LVCMOS33")),

    # I2S / TDM
    ("i2s", 0,
        Subsignal("mclk", Pins("F15")),
        Subsignal("bclk", Pins("E17")),
        Subsignal("lrck", Pins("F17")),
        Subsignal("dout", Pins("G18")),
        Subsignal("din",  Pins("H17")),
        IOStandard("LVCMOS33"),
    ),

    # SDRAM (M12L64322A 8MB 32-bit)
    ("sdram_clock", 0, Pins("B9"), IOStandard("LVCMOS33")),
    ("sdram", 0,
        Subsignal("a",   Pins("A7 A6 A5 A4 C7 C6 C5 C4 E7 D6 D5")),
        Subsignal("ba",  Pins("B11 C8")),
        Subsignal("cs_n", Pins("B7")),
        Subsignal("cke", Pins("B12")),
        Subsignal("ras_n", Pins("B10")),
        Subsignal("cas_n", Pins("A10")),
        Subsignal("we_n",  Pins("A9")),
        Subsignal("dm",  Pins("A8 B8 E9 D9")),
        Subsignal("dq",  Pins(
            "C2 C1 D2 D1 E2 E1 F2 F1",
            "G5 H4 H5 J4 J5 K4 K5 L4",
            "E11 D11 D12 C12 D13 C13 D14 C14",
            "E14 D15 E15 C15 E16 D16 D17 C17",
        )),
        IOStandard("LVCMOS33"),
        Misc("SLEWRATE=FAST"),
    ),
]


class Platform(LatticePlatform):
    default_clk_name   = "clk25"
    default_clk_period = 1e9 / 25e6

    def __init__(self):
        LatticePlatform.__init__(self, "LFE5U-45F-6BG381C", _io, toolchain="trellis")

    def create_programmer(self):
        from litex.build.openfpgaloader import OpenFPGALoader
        return OpenFPGALoader(cable="cmsis-dap")


# ---------------------------------------------------------------------------
# Clock and Reset
# ---------------------------------------------------------------------------
class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_sys_ps = ClockDomain()  # SDRAM phase shifted

        clk25 = platform.request("clk25")

        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=180)

        self.specials += Instance("ODDRX1F",
            i_D0=0, i_D1=1, i_SCLK=ClockSignal("sys_ps"),
            o_Q=platform.request("sdram_clock"))


# ---------------------------------------------------------------------------
# AES67 hardware engine peripheral
# ---------------------------------------------------------------------------
class AES67Engine(Module, AutoCSR):
    """Wraps aes67_top.v and exposes its config/status as LiteX CSRs."""

    def __init__(self, platform):
        # ---- Configuration CSRs ----
        self.local_clock_id     = CSRStorage(64, reset=0x0011_22FF_FE33_4455)
        self.local_mac          = CSRStorage(48, reset=0x02AE_6700_0001)
        self.local_ip           = CSRStorage(32, reset=0)
        self.rtp_mcast_ip       = CSRStorage(32, reset=0xEF45_0001)
        self.rtp_dst_mac        = CSRStorage(48, reset=0x0100_5E45_0001)
        self.rtp_port           = CSRStorage(16, reset=5004)
        # PI gains: Kp=0.5, Ki=0.05 (Q16.16) - tuned for AES67 stability
        self.kp                 = CSRStorage(32, reset=0x00008000)
        self.ki                 = CSRStorage(32, reset=0x00000CCC)
        self.step_threshold_ns  = CSRStorage(32, reset=1000)
        # Asymmetry compensation (RFC §7.4) and offset low-pass pole
        self.tx_delay_ns        = CSRStorage(32, reset=0)
        self.rx_delay_ns        = CSRStorage(32, reset=0)
        self.filter_shift       = CSRStorage(4,  reset=4)  # ~16-sample TC

        # PTP role: 0 = slave (default), 1 = master. Set by BMC firmware.
        self.mode_is_master       = CSRStorage(1,  reset=0)
        # Master Sync interval in clk125 cycles. 15.625e6 cycles = 125 ms.
        self.sync_interval_cycles = CSRStorage(32, reset=15_625_000)

        self.tx_ssrc            = CSRStorage(32, reset=0xDEADBEEF)
        self.rx_expected_ssrc   = CSRStorage(32, reset=0)
        self.payload_type       = CSRStorage(7,  reset=98)
        self.num_channels       = CSRStorage(4,  reset=2)
        # Ultra-low-latency defaults: 6 samples = 125 µs @ 48 kHz
        self.samples_per_packet = CSRStorage(11, reset=6)
        # Jitter buffer: 24 frames = 500 µs @ 48 kHz
        self.jbuf_target_depth  = CSRStorage(10, reset=24)

        self.nco_increment      = CSRStorage(32, reset=422_212_466)
        self.sample_rate        = CSRStorage(32, reset=48000)

        # ---- Status CSRs ----
        self.ptp_sec            = CSRStatus(48)
        self.ptp_nsec           = CSRStatus(32)
        self.ptp_lock_state     = CSRStatus(2)
        self.ptp_offset_filt_ns = CSRStatus(32)
        self.ptp_path_delay_ns  = CSRStatus(32)
        self.ptp_sync_count     = CSRStatus(32)
        self.ptp_last_seq       = CSRStatus(16)
        self.rtp_packets_rx     = CSRStatus(32)
        self.rtp_packets_tx     = CSRStatus(32)
        self.rtp_seq_errors     = CSRStatus(32)
        self.jbuf_depth         = CSRStatus(11)

        # ---- Pins ----
        eth0 = platform.request("eth0_rgmii")
        eth1 = platform.request("eth1_rgmii")
        i2s  = platform.request("i2s")
        led  = platform.request("user_led_n")
        mdio = platform.request("eth_mdio")
        mdc  = platform.request("eth_mdc")
        rst_n= platform.request("eth_rst_n")

        # ---- CPU netif BRAM ports (memory-mapped via Wishbone wrapper below) ----
        self.cpunet_rx_addr  = Signal(11)
        self.cpunet_rx_data  = Signal(8)
        self.cpunet_tx_addr  = Signal(11)
        self.cpunet_tx_wdata = Signal(8)
        self.cpunet_tx_we    = Signal()

        self.rx_ready    = CSRStatus()
        self.rx_length   = CSRStatus(16)
        self.rx_ack      = CSR()
        self.tx_pending  = CSRStatus()
        self.tx_send     = CSR()
        self.tx_length   = CSRStorage(16)
        self.irq_status  = CSRStatus()

        # ---- Virtual TDM-16 CSRs ----
        self.virtaud_tx_data    = CSRStorage(32)
        self.virtaud_tx_ch      = CSRStorage(4)
        self.virtaud_tx_wr      = CSR()
        self.virtaud_tx_full    = CSRStatus()
        self.virtaud_rx_data    = CSRStatus(32)
        self.virtaud_rx_ch      = CSRStatus(4)
        self.virtaud_rx_rd      = CSR()
        self.virtaud_rx_empty   = CSRStatus()
        self.virtaud_mix_enable = CSRStorage()
        self.virtaud_chan_mask  = CSRStorage(16)

        # ---- Add Verilog sources ----
        rtl_dir = os.path.join(os.path.dirname(__file__), "..", "rtl")
        for sub in ["util", "eth", "ptp", "rtp", "audio", "soc"]:
            d = os.path.join(rtl_dir, sub)
            for f in sorted(os.listdir(d)):
                if f.endswith(".v"):
                    platform.add_source(os.path.join(d, f))
        platform.add_source(os.path.join(rtl_dir, "pll_25_to_125.v"))
        platform.add_source(os.path.join(rtl_dir, "aes67_top.v"))

        # ---- Instantiate aes67_top ----
        self.specials += Instance("aes67_top",
            i_clk25      = ClockSignal("sys"),
            o_led        = led,

            o_eth0_gtxclk = eth0.gtxclk,
            o_eth0_txd    = eth0.txd,
            o_eth0_tx_en  = eth0.tx_en,
            i_eth0_rxc    = eth0.rxc,
            i_eth0_rxd    = eth0.rxd,
            i_eth0_rx_dv  = eth0.rx_dv,

            o_eth1_gtxclk = eth1.gtxclk,
            o_eth1_txd    = eth1.txd,
            o_eth1_tx_en  = eth1.tx_en,
            i_eth1_rxc    = eth1.rxc,
            i_eth1_rxd    = eth1.rxd,
            i_eth1_rx_dv  = eth1.rx_dv,

            io_eth_mdio  = mdio,
            o_eth_mdc    = mdc,
            o_eth_rst_n  = rst_n,

            o_i2s_mclk   = i2s.mclk,
            o_i2s_bclk   = i2s.bclk,
            o_i2s_lrck   = i2s.lrck,
            o_i2s_dout   = i2s.dout,
            i_i2s_din    = i2s.din,

            # Configuration
            i_cfg_local_clock_id     = self.local_clock_id.storage,
            i_cfg_local_mac          = self.local_mac.storage,
            i_cfg_local_ip           = self.local_ip.storage,
            i_cfg_rtp_mcast_ip       = self.rtp_mcast_ip.storage,
            i_cfg_rtp_dst_mac        = self.rtp_dst_mac.storage,
            i_cfg_rtp_port           = self.rtp_port.storage,
            i_cfg_kp                 = self.kp.storage,
            i_cfg_ki                 = self.ki.storage,
            i_cfg_step_threshold_ns  = self.step_threshold_ns.storage,
            i_cfg_tx_ssrc            = self.tx_ssrc.storage,
            i_cfg_rx_expected_ssrc   = self.rx_expected_ssrc.storage,
            i_cfg_payload_type       = self.payload_type.storage,
            i_cfg_num_channels       = self.num_channels.storage,
            i_cfg_samples_per_pkt    = self.samples_per_packet.storage,
            i_cfg_jbuf_target_depth  = self.jbuf_target_depth.storage,
            i_cfg_nco_increment      = self.nco_increment.storage,
            i_cfg_tx_delay_ns        = self.tx_delay_ns.storage,
            i_cfg_rx_delay_ns        = self.rx_delay_ns.storage,
            i_cfg_filter_shift       = self.filter_shift.storage,
            i_cfg_mode_is_master     = self.mode_is_master.storage,
            i_cfg_sync_interval_cycles = self.sync_interval_cycles.storage,

            # Status
            o_stat_ptp_sec             = self.ptp_sec.status,
            o_stat_ptp_nsec            = self.ptp_nsec.status,
            o_stat_ptp_lock_state      = self.ptp_lock_state.status,
            o_stat_ptp_offset_filt     = self.ptp_offset_filt_ns.status,
            o_stat_ptp_path_delay_filt = self.ptp_path_delay_ns.status,
            o_stat_ptp_sync_count      = self.ptp_sync_count.status,
            o_stat_ptp_last_seq        = self.ptp_last_seq.status,
            o_stat_rtp_packets_rx      = self.rtp_packets_rx.status,
            o_stat_rtp_packets_tx      = self.rtp_packets_tx.status,
            o_stat_rtp_seq_errors      = self.rtp_seq_errors.status,
            o_stat_jbuf_depth          = self.jbuf_depth.status,

            # CPU netif
            i_cpunet_rx_addr   = self.cpunet_rx_addr,
            o_cpunet_rx_data   = self.cpunet_rx_data,
            i_cpunet_tx_addr   = self.cpunet_tx_addr,
            i_cpunet_tx_wdata  = self.cpunet_tx_wdata,
            i_cpunet_tx_we     = self.cpunet_tx_we,
            o_cpunet_rx_ready  = self.rx_ready.status,
            o_cpunet_rx_length = self.rx_length.status,
            i_cpunet_rx_ack    = self.rx_ack.re,
            o_cpunet_tx_pending= self.tx_pending.status,
            i_cpunet_tx_send   = self.tx_send.re,
            i_cpunet_tx_length = self.tx_length.storage,
            o_cpunet_irq       = self.irq_status.status,

            # Virtual I2S
            i_virtaud_tx_wr        = self.virtaud_tx_wr.re,
            i_virtaud_tx_data      = self.virtaud_tx_data.storage,
            i_virtaud_tx_ch        = self.virtaud_tx_ch.storage,
            o_virtaud_tx_full      = self.virtaud_tx_full.status,
            i_virtaud_rx_rd        = self.virtaud_rx_rd.re,
            o_virtaud_rx_data      = self.virtaud_rx_data.status,
            o_virtaud_rx_ch        = self.virtaud_rx_ch.status,
            o_virtaud_rx_empty     = self.virtaud_rx_empty.status,
            i_virtaud_mix_enable   = self.virtaud_mix_enable.storage,
            i_virtaud_channel_mask = self.virtaud_chan_mask.storage,
        )


# ---------------------------------------------------------------------------
# Wishbone-mapped BRAM wrapper for cpu_netif
# ---------------------------------------------------------------------------
class CPUNetifBRAMs(Module):
    """Exposes the cpu_netif RX/TX BRAMs at fixed addresses on the CPU bus."""
    def __init__(self, aes67):
        self.rx_bus = wishbone.Interface(data_width=8)
        self.tx_bus = wishbone.Interface(data_width=8)

        # RX: read-only by CPU
        self.comb += [
            aes67.cpunet_rx_addr.eq(self.rx_bus.adr[:11]),
            self.rx_bus.dat_r.eq(aes67.cpunet_rx_data),
        ]
        self.sync += self.rx_bus.ack.eq(self.rx_bus.cyc & self.rx_bus.stb & ~self.rx_bus.ack)

        # TX: write-only by CPU
        self.comb += [
            aes67.cpunet_tx_addr.eq(self.tx_bus.adr[:11]),
            aes67.cpunet_tx_wdata.eq(self.tx_bus.dat_w),
            aes67.cpunet_tx_we.eq(self.tx_bus.cyc & self.tx_bus.stb & self.tx_bus.we & ~self.tx_bus.ack),
        ]
        self.sync += self.tx_bus.ack.eq(self.tx_bus.cyc & self.tx_bus.stb & ~self.tx_bus.ack)


# ---------------------------------------------------------------------------
# SoC top
# ---------------------------------------------------------------------------
class AES67SoC(SoCCore):
    def __init__(self, sys_clk_freq=int(60e6), **kwargs):
        platform = Platform()

        SoCCore.__init__(self, platform,
            clk_freq            = sys_clk_freq,
            cpu_type            = "vexriscv",
            cpu_variant         = "lite",
            integrated_rom_size = 0x10000,
            integrated_sram_size= 0x4000,
            ident               = "AES67 SoC on Colorlight i9 v7.2",
            ident_version       = True,
            **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SDRAM
        self.submodules.sdrphy = GENSDRPHY(platform.request("sdram"), sys_clk_freq)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = M12L64322A(sys_clk_freq, "1:1"),
            l2_cache_size = 8192,
        )

        # AES67 hardware engine
        self.submodules.aes67 = AES67Engine(platform)
        self.add_csr("aes67")

        # cpu_netif BRAMs as Wishbone memory regions
        self.submodules.cpunet_brams = CPUNetifBRAMs(self.aes67)

        cpunet_rx_base = 0x90000000
        cpunet_tx_base = 0x90001000
        self.bus.add_slave("cpu_netif_rx", self.cpunet_brams.rx_bus,
                            wishbone.SoCRegion(origin=cpunet_rx_base, size=0x800, mode="r"))
        self.bus.add_slave("cpu_netif_tx", self.cpunet_brams.tx_bus,
                            wishbone.SoCRegion(origin=cpunet_tx_base, size=0x800, mode="w"))

        # Expose addresses via constants in generated mem.h
        self.add_constant("CPU_NETIF_RX_BUF_BASE", cpunet_rx_base)
        self.add_constant("CPU_NETIF_TX_BUF_BASE", cpunet_tx_base)


# ---------------------------------------------------------------------------
# Build entry point
# ---------------------------------------------------------------------------
def main():
    import argparse
    parser = argparse.ArgumentParser(description="AES67 SoC for Colorlight i9 v7.2")
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--load",  action="store_true")
    parser.add_argument("--flash", action="store_true")
    parser.add_argument("--sys-clk-freq", default=60e6, type=float)
    args = parser.parse_args()

    soc = AES67SoC(sys_clk_freq=int(args.sys_clk_freq))
    builder = Builder(soc, output_dir="build", csr_csv="build/csr.csv")

    if args.build:
        builder.build()

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream("build/gateware/colorlight_i9_v7_2.bit")

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, "build/gateware/colorlight_i9_v7_2.bit")


if __name__ == "__main__":
    main()
