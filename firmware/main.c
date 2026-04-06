// SPDX-License-Identifier: MIT
// AES67 SoC Firmware - Management Plane (lwIP)
//
// Runs on the VexRiscv soft CPU. Brings up:
//   - Hardware AES67 engine (PTP, RTP, audio) via CSRs
//   - lwIP TCP/IP stack via the cpu_netif hardware peripheral
//   - DHCP client (lwIP built-in)
//   - mDNS responder (_ravenna._udp.local, lwIP built-in)
//   - HTTP server with /status endpoint (lwIP httpd)
//
// All timing-critical paths (RTP, PTP, audio) run untouched in hardware.

#include <generated/csr.h>
#include <generated/mem.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/mdns.h"
#include "netif/ethernet.h"

#include "litex_netif.h"
#include "sap.h"
#include "ptp_bmc.h"
#include "httpd_lite.h"
#include "api.h"
#include "audio_stream.h"
#include "web_index.h"

// Default locally-administered MAC. The lower 24 bits become the unit
// serial and are used to derive the hostname (aes67br-XXXXXX) and the
// EUI-64 PTP clock identity.
static const uint8_t s_mac[6] = { 0x02, 0xAE, 0x67, 0x00, 0x00, 0x01 };

static struct netif s_netif;

// ---- AES67 hardware bring-up ----------------------------------------------
static void aes67_configure_defaults(void)
{
    // Servo gains (Q16.16): Kp=0.5, Ki=0.05 - tuned for AES67 stability
    aes67_kp_write(0x00008000);
    aes67_ki_write(0x00000CCC);
    aes67_step_threshold_ns_write(1000);

    // Asymmetry compensation - measured per-board, default 0
    aes67_tx_delay_ns_write(0);
    aes67_rx_delay_ns_write(0);
    aes67_filter_shift_write(4);   // ~16-sample LP filter

    // Ultra-low-latency: stereo L24 @ 48 kHz, 125 µs packets, 500 µs jitter buffer
    aes67_tx_ssrc_write(0xDEADBEEF);
    aes67_rx_expected_ssrc_write(0);
    aes67_payload_type_write(98);
    aes67_num_channels_write(2);
    aes67_samples_per_packet_write(6);    // 6 samples = 125 µs @ 48 kHz
    aes67_jbuf_target_depth_write(24);    // 24 frames = 500 µs @ 48 kHz

    // Audio NCO for 48 kHz × 8 slot × 32 bit TDM (12.288 MHz BCLK)
    aes67_nco_increment_write(422212466);
    aes67_sample_rate_write(48000);

    // Default RTP multicast: 239.69.0.1:5004
    aes67_rtp_mcast_ip_write(0xEF450001);
    aes67_rtp_dst_mac_write(0x01005E450001ULL);
    aes67_rtp_port_write(5004);

    // Local MAC into the hardware engine (used by TX header wrappers)
    uint64_t mac64 = 0;
    for (int i = 0; i < 6; i++)
        mac64 = (mac64 << 8) | s_mac[i];
    aes67_local_mac_write(mac64);
}

// ---- mDNS service callback (TXT records for _ravenna._udp) ----------------
static void srv_txt(struct mdns_service *service, void *txt_userdata)
{
    (void)txt_userdata;
    err_t res;
    res  = mdns_resp_add_service_txtitem(service, "proto=ravenna",  13);
    res |= mdns_resp_add_service_txtitem(service, "rate=48000",     10);
    res |= mdns_resp_add_service_txtitem(service, "channels=2",     10);
    if (res != ERR_OK)
        printf("mDNS: TXT add failed\n");
}

// ---- Status callbacks -----------------------------------------------------
static void netif_status_cb(struct netif *netif)
{
    if (netif_is_up(netif)) {
        printf("netif: link UP, ip=%s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
        // (Re)announce mDNS once we have a real address
        mdns_resp_announce(netif);
        aes67_local_ip_write(ntohl(ip4_addr_get_u32(netif_ip4_addr(netif))));
    } else {
        printf("netif: link DOWN\n");
    }
}

// ---- Main -----------------------------------------------------------------
static const char *lock_state_name(uint32_t s) {
    switch (s) {
        case 0: return "UNLOCKED";
        case 1: return "LOCKING";
        case 2: return "LOCKED";
        default: return "?";
    }
}

static void print_periodic_status(void)
{
    static uint32_t last = 0;
    uint32_t now = sys_now();
    if (now - last < 1000) return;
    last = now;

    printf("[%s] PTP=%s sec=%llu offset=%ld delay=%ld sync=%lu  RTP rx=%lu tx=%lu seq=%lu jbuf=%lu\n",
           ip4addr_ntoa(netif_ip4_addr(&s_netif)),
           lock_state_name(aes67_ptp_lock_state_read()),
           (unsigned long long)aes67_ptp_sec_read(),
           (long)aes67_ptp_offset_filt_ns_read(),
           (long)aes67_ptp_path_delay_ns_read(),
           (unsigned long)aes67_ptp_sync_count_read(),
           (unsigned long)aes67_rtp_packets_rx_read(),
           (unsigned long)aes67_rtp_packets_tx_read(),
           (unsigned long)aes67_rtp_seq_errors_read(),
           (unsigned long)aes67_jbuf_depth_read());
}

int main(void)
{
    printf("\n=== AES67 SoC firmware (lwIP) starting ===\n");

    aes67_configure_defaults();

    // Initialise lwIP core
    lwip_init();

    // Add our netif (litex_netif → cpu_netif hardware)
    ip4_addr_t ipaddr = { 0 }, netmask = { 0 }, gw = { 0 };
    netif_add(&s_netif, &ipaddr, &netmask, &gw,
              NULL, litex_netif_init, ethernet_input);
    memcpy(s_netif.hwaddr, s_mac, 6);
    netif_set_default(&s_netif);
    netif_set_status_callback(&s_netif, netif_status_cb);
    netif_set_link_up(&s_netif);
    netif_set_up(&s_netif);

    // Generate hostname (aes67br-XXXXXX) and clock id from MAC
    unit_hostname_init();
    const char *hn = unit_hostname();
    printf("Unit hostname: %s\n", hn);
    netif_set_hostname(&s_netif, hn);

    // Start DHCP
    if (dhcp_start(&s_netif) != ERR_OK) {
        printf("dhcp_start failed\n");
    } else {
        printf("DHCP: started\n");
    }

    // mDNS responder
    mdns_resp_init();
    mdns_resp_add_netif(&s_netif, hn);
    mdns_resp_add_service(&s_netif, hn, "_ravenna._udp",
                           DNSSD_PROTO_UDP, 5004, srv_txt, NULL);
    mdns_resp_add_service(&s_netif, hn, "_http._tcp",
                           DNSSD_PROTO_TCP, 80,   NULL,    NULL);

    // HTTP server with REST API + WebUI + audio preview streaming
    httpd_lite_init(80);
    web_index_register();
    api_register_routes();
    audio_stream_register();

    // SAP / SDP session announcement (RFC 2974)
    sap_init();

    // Best Master Clock algorithm (RFC 1588 §9)
    // Drives the hardware mode_is_master CSR based on Announce messages
    ptp_bmc_init();

    printf("Services up. Entering main loop.\n");

    // Main loop: poll netif, drive lwIP timers, run SAP + BMC, print status
    while (1) {
        litex_netif_poll(&s_netif);
        sys_check_timeouts();
        sap_poll();
        ptp_bmc_poll();
        print_periodic_status();
    }

    return 0;
}
