# AES67 on Colorlight i9 v7.2 (Lattice ECP5)

A complete hardware AES67/RAVENNA Audio-over-IP implementation targeting the
Colorlight i9 v7.2 FPGA board, with **IEEE 1588 PTP fully implemented in
Verilog** and a VexRiscv soft CPU running **lwIP** for the management plane
on the same physical Ethernet — without ever touching the RTP data path.

## Features

| | |
|---|---|
| Hardware MAC | Custom 1 Gbps RGMII MAC with PTP SFD pulses |
| PTP | Full IEEE 1588 in Verilog: clock, hardware timestamping, packet processor (Sync/FollowUp/DelayReq/DelayResp), PI servo, lock detector |
| RTP | Hardware TX + RX engines with L24 codec, BRAM jitter buffer |
| Audio I/O | I2S **and TDM** master (up to 16 channels) clocked by a PTP-locked NCO |
| TX path | IP/UDP/Ethernet wrappers for PTP and RTP, 3-way arbiter (PTP > RTP > CPU) |
| Soft CPU | LiteX VexRiscv with 8 MB SDRAM, lwIP TCP/IP stack |
| DHCP | Full lwIP DHCP client (DISCOVER/OFFER/REQUEST/ACK/RENEW) |
| mDNS | lwIP mdns_responder advertising `_ravenna._udp` and `_http._tcp` |
| HTTP | lwIP httpd with `/status.cgi` JSON endpoint |
| Virtual I2S | CPU-accessible audio FIFO that mixes into the RTP TX path or replaces channels |
| PLL | Real ECP5 `EHXPLLL` (25 → 125 MHz @ 0° + 90°) |
| Toolchain | Reproducible Docker image with OSS CAD Suite + LiteX + lwIP + RV32 GCC |

## Architecture

```
                              ┌──────────────────────────────────┐
                              │      Lattice ECP5-45F            │
                              │                                  │
                              │  ┌─────────────┐                 │
              PHY0 (RGMII) ───┼──┤ Custom MAC  │                 │
                              │  │ + SFD pulses│                 │
                              │  └──────┬──────┘                 │
                              │         │                        │
                              │  ┌──────┴──────┐                 │
                              │  │   Packet    │   classifier    │
                              │  │   Router    │                 │
                              │  └──┬──┬──┬───┘                 │
                              │     │  │  │                      │
                              │  ┌──┘  │  └──┐                   │
                              │  │     │     │                   │
                              │  ▼     ▼     ▼                   │
                              │ ┌──┐  ┌──┐  ┌──────┐              │
                              │ │PTP│ │RTP│ │ CPU  │ ←─ lwIP      │
                              │ │HW │ │HW │ │netif │  on VexRiscv│
                              │ └─┬┘  └─┬┘  └──┬───┘              │
                              │   │     │      │                  │
                              │   ▼     ▼      ▼                  │
                              │ ┌──────────────────┐              │
                              │ │  TX wrappers     │              │
                              │ │ (IP/UDP/Eth hdr) │              │
                              │ └────────┬─────────┘              │
                              │          │                        │
                              │  ┌───────┴────────┐               │
                              │  │  TX Arbiter    │               │
                              │  │  (PTP>RTP>CPU) │               │
                              │  └───────┬────────┘               │
                              │          │                        │
                              │  back to MAC TX                   │
                              │                                   │
                              │  ┌─────────────────────────┐      │
                              │  │  PTP-locked Audio NCO   │      │
                              │  │      ↓                  │      │
                              │  │  I2S/TDM Master ←─ Virtual I2S│
                              │  │      ↓             (CPU mix) │
                              │  └────────┬────────────────┘      │
                              │           │                       │
                              └───────────┼───────────────────────┘
                                          ↓
                                    Audio Codec(s)
```

| Plane | Where it runs | Function |
|-------|---------------|----------|
| Data plane (timing-critical) | FPGA fabric | Ethernet MAC, PTP, RTP, audio NCO, I2S/TDM |
| Management plane | VexRiscv + lwIP | DHCP, ARP, ICMP, IGMP, mDNS, HTTP |
| User audio plane | Virtual I2S | CPU can mix into / tap from the audio path |

## Sharing the Same Ethernet

The CPU's lwIP stack and the hardware RTP/PTP engines share **one physical
PHY**. The packet router classifies inbound frames at line rate:

- **PTP** (UDP 319/320 or EtherType 0x88F7) → hardware PTP engine
- **RTP** (UDP 5004 to multicast group) → hardware RTP engine
- **Everything else** (ARP, ICMP, DHCP, mDNS, HTTP, …) → CPU netif

For TX, three streams are merged by the arbiter without bothering the RTP
engine: PTP DelayReq packets, RTP audio packets (each wrapped with full
IP/UDP/Ethernet headers by `tx_udp_wrapper`), and CPU frames coming from
lwIP via the `cpu_netif` BRAM.

The result: **DHCP works, mDNS works, HTTP works, and the RTP data path is
never touched by software**.

## Virtual I2S

The `virt_i2s` peripheral exposes a CPU-accessible FIFO pair that hooks
into the audio data path. The CPU can:

- **Generate audio** (test tones, software synths) and have it appear on
  selected RTP channels (mix or replace mode)
- **Record audio** from incoming RTP or the I2S ADC into RAM/SD/network
- **Run software DSP** in a feedback loop with the hardware audio path

All without disturbing the timing-critical RTP TX/RX. The FIFO uses
clock-domain-safe Sarwate-style sample staging so even slow CPU writes
don't drop samples.

## RTL Module Overview

```
rtl/
├── aes67_top.v                Top-level integration
├── pll_25_to_125.v            Real ecppll EHXPLLL (with SIM_PLL fallback)
├── eth/
│   ├── rgmii_rx.v             DDR RGMII receive (ECP5 IDDRX1F)
│   ├── rgmii_tx.v             DDR RGMII transmit (ECP5 ODDRX1F)
│   ├── eth_mac_rx.v           Frame RX, CRC check, SFD pulse
│   ├── eth_mac_tx.v           Frame TX, CRC gen, SFD pulse
│   ├── eth_mac.v              MAC wrapper
│   ├── ip_checksum.v          Streaming RFC-1071 16-bit checksum
│   ├── tx_udp_wrapper.v       Ethernet/IP/UDP header prepender
│   └── tx_arbiter.v           3-way priority TX arbiter (PTP>RTP>CPU)
├── ptp/                       ★ Full IEEE 1588 in HW
│   ├── ptp_clock.v            96-bit timestamp + 8.24 NCO + phase step
│   ├── ptp_timestamp.v        SFD-triggered TX/RX timestamp FIFOs
│   ├── ptp_pp.v               Sync/FollowUp/DelayResp parser, DelayReq gen
│   ├── ptp_servo.v            PI controller, anti-windup, lock detector
│   └── ptp_top.v              Subsystem wrapper
├── rtp/
│   ├── rtp_rx.v               UDP→RTP→L24→samples
│   ├── rtp_tx.v               samples→L24→RTP→UDP
│   ├── jitter_buffer.v        Per-channel BRAM jitter buffer
│   └── rtp_engine.v           Wrapper
├── audio/
│   ├── audio_clk_gen.v        PTP-locked NCO (BCLK/LRCLK/MCLK)
│   ├── i2s_tdm_master.v       I2S + TDM master (configurable slots)
│   └── virt_i2s.v             CPU-accessible audio FIFO + mixer
├── soc/
│   ├── packet_router.v        RX classifier (PTP / RTP / CPU)
│   └── cpu_netif.v            CPU Ethernet interface (BRAM-buffered)
└── util/                      crc32, fifo_sync, fifo_async (CDC)
```

## Firmware (lwIP)

```
firmware/
├── main.c                  lwIP init, dhcp_start, mdns_resp, httpd_init
├── litex_netif.c/.h        lwIP netif bridging cpu_netif BRAMs to pbufs
├── sys_now.c               sys_now() and lwip_rand() backed by PTP clock
├── aes67_status.c          /status.cgi JSON handler
├── lwipopts.h              lwIP NO_SYS configuration (~32 KB heap)
├── arch/cc.h, sys_arch.h   lwIP arch port for RV32/GCC
└── Makefile                builds with $(LWIP_DIR)=/opt/lwip
```

The firmware uses lwIP's built-in:
- `dhcp.c` — full DHCP client
- `etharp.c` — ARP cache + responder
- `icmp.c` — ICMP echo replies
- `igmp.c` — multicast group management
- `apps/httpd/` — HTTP server with CGI
- `apps/mdns/` — mDNS service responder

`sys_now()` is backed by the hardware PTP clock — when the SoC is locked to
a grandmaster, all lwIP timers (DHCP renew, mDNS scheduling, TCP RTT) are
PTP-disciplined.

## Quick Start

### 1. Build the toolchain image (one-time, ~3.5 GB)

```bash
./docker/build.sh
```

Installs:
- OSS CAD Suite (Yosys, nextpnr-ecp5, prjtrellis, Icarus, Verilator,
  GTKWave, openFPGAloader)
- LiteX ecosystem (migen, litex, litedram, liteeth)
- **lwIP STABLE-2_2_0_RELEASE** at `/opt/lwip`
- RISC-V GCC for VexRiscv firmware

### 2. Build everything

```bash
./docker/run.sh make litex      # full SoC + gateware + lwIP firmware
```

This produces `litex/build/colorlight_i9_v7_2/gateware/colorlight_i9_v7_2.bit`
and `firmware/firmware.bin`.

### 3. Program the board

Linux (with the i9 ext-board's DAPLink connected over USB):

```bash
./docker/run.sh make program       # SRAM (volatile)
./docker/run.sh make flash         # SPI flash (persistent)
```

macOS / Windows: programming requires native USB. Either install
`openFPGAloader` natively (Homebrew/MSYS2) and only use Docker for the build,
or forward the USB device into a Linux VM running Docker.

### 4. Watch DHCP work

```bash
./docker/run.sh openFPGAloader -b colorlight-i9 build/colorlight_i9_v7_2.bit
# (or use the Makefile target)
# then connect a USB-UART to the board's debug header to see:
#
#   === AES67 SoC firmware (lwIP) starting ===
#   DHCP: started
#   netif: link UP, ip=192.168.1.137
#   mDNS: announced aes67.local
#   httpd: listening
#   Services up. Entering main loop.
#   [192.168.1.137] PTP=LOCKED sec=... offset=12 delay=87 sync=42 ...
```

You can then visit `http://aes67.local/` (or the DHCP-assigned IP) to see
the live status, and `discovery-tool` will see the `_ravenna._udp` service.

## Cross-Platform Toolchain

| Host | Build | Simulate | Program |
|------|-------|----------|---------|
| Linux x64    | ✓ | ✓ | ✓ native USB passthrough |
| Linux ARM    | ✓ | ✓ | ✓ native USB passthrough |
| macOS Intel  | ✓ | ✓ | use native openFPGAloader |
| macOS Apple Silicon | ✓ | ✓ | use native openFPGAloader |
| Windows (WSL2) | ✓ | ✓ | ✓ via usbipd-win |

## Configuration

All AES67 parameters are CSR-mapped in the LiteX SoC. The firmware writes
sensible defaults at startup; runtime changes can be made via the `/status`
JSON endpoint (read) or by direct CSR access from the firmware.

| CSR | Default | Notes |
|-----|---------|-------|
| `local_mac` | 02:AE:67:00:00:01 | unit MAC (locally administered) |
| `local_ip` | 0.0.0.0 | written by DHCP after lease |
| `rtp_mcast_ip` | 239.69.0.1 | AES67 default multicast group |
| `rtp_dst_mac` | 01:00:5E:45:00:01 | derived from `rtp_mcast_ip` |
| `rtp_port` | 5004 | RTP destination port |
| `payload_type` | 98 | RTP PT for L24 |
| `num_channels` | 2 | active channels per stream |
| `samples_per_packet` | 48 | 1 ms @ 48 kHz |
| `kp` / `ki` | 1.0 / ~0.004 | PTP servo gains (Q16.16) |
| `step_threshold_ns` | 1000 | phase step trigger |
| `nco_increment` | 422_212_466 | BCLK NCO base value |
| `virtaud_mix_enable` | 0 | 1 = mix CPU into RTP TX |
| `virtaud_chan_mask` | 0 | bit per channel that CPU drives |

## Resource Estimate (LFE5U-45F)

| Block | LUTs (est.) | BRAM |
|-------|------------|------|
| Custom Ethernet MAC | ~2,500 | 0 |
| PTP subsystem | ~3,000 | 1 EBR |
| TX wrappers + arbiter | ~1,200 | 0 |
| Packet router | ~1,500 | 1 EBR |
| RTP engine (1 stream) | ~3,500 | 8 EBR |
| Audio NCO + I2S/TDM | ~1,500 | 0 |
| Virtual I2S | ~800 | 1 EBR |
| CPU netif | ~600 | 4 EBR |
| LiteX VexRiscv (lite) | ~3,000 | 16 EBR |
| LiteDRAM (SDRAM ctrl) | ~2,000 | 2 EBR |
| **Total** | **~19 K / 44 K LUT** | **~33 / 108 EBR** |

Comfortable headroom for additional streams or PHY 1 bring-up
(redundant Ethernet for ST 2022-7).

## Status / Limitations

Implemented and wired together:
- Real ECP5 PLL (`EHXPLLL`)
- Custom 1 Gbps RGMII MAC with PTP SFD pulses
- Full IEEE 1588 in HW (slave mode)
- PI servo with lock detector
- RTP TX/RX with L24, jitter buffer
- PTP-locked audio NCO + I2S/TDM master
- Virtual I2S with mix/replace
- 3-way TX arbiter
- CPU netif (BRAM-mapped)
- lwIP + DHCP + ARP + ICMP + IGMP + mDNS + HTTP
- Docker toolchain (Linux/macOS/Windows)

Known not-yet-done:
- PTP **grandmaster** mode (only slave is wired up; Announce/Sync gen is
  the next addition to `ptp_pp.v`)
- PHY 1 currently tied off — bring up for redundancy / second network
- MDIO management driven from CPU (currently tied off)
- Multi-stream RTP (one stream per `rtp_engine` instance today)

## License

MIT.
