// SPDX-License-Identifier: MIT
// lwIP options for the AES67 SoC firmware (NO_SYS, RV32, ~64 KB SRAM budget)

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// ---- Core ---------------------------------------------------------------
#define NO_SYS                          1   // bare metal, no OS
#define LWIP_TIMERS                     1
#define SYS_LIGHTWEIGHT_PROT            0

#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define LWIP_RAW                        0

// ---- Memory -------------------------------------------------------------
#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (32 * 1024)
#define MEMP_NUM_PBUF                   24
#define MEMP_NUM_UDP_PCB                8
#define MEMP_NUM_TCP_PCB                4
#define MEMP_NUM_TCP_PCB_LISTEN         2
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_REASSDATA              4
#define MEMP_NUM_FRAG_PBUF              8
#define MEMP_NUM_ARP_QUEUE              4
#define MEMP_NUM_IGMP_GROUP             4
#define MEMP_NUM_SYS_TIMEOUT            8

// PBUF pool — large enough for several MTU-sized frames
#define PBUF_POOL_SIZE                  16
#define PBUF_POOL_BUFSIZE               1536

// ---- IP / Protocol stack ------------------------------------------------
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define ARP_TABLE_SIZE                  10
#define ARP_QUEUEING                    1
#define LWIP_ICMP                       1
#define LWIP_BROADCAST_PING             1

#define LWIP_UDP                        1
#define LWIP_UDPLITE                    0
#define LWIP_TCP                        1
#define TCP_MSS                         1460
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_WND                         (4 * TCP_MSS)

#define LWIP_DHCP                       1
#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DNS                        1

#define LWIP_IGMP                       1   // multicast for mDNS / IPTV-style RTP discovery

// ---- Netif --------------------------------------------------------------
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_API                  0
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1

// ---- Apps ---------------------------------------------------------------
#define LWIP_HTTPD                      1
#define LWIP_HTTPD_CGI                  1
#define LWIP_HTTPD_SSI                  0
#define HTTPD_USE_CUSTOM_FSDATA         0

#define LWIP_MDNS_RESPONDER             1
#define MDNS_MAX_SERVICES               4
#define LWIP_NUM_NETIF_CLIENT_DATA      (LWIP_MDNS_RESPONDER)

// ---- Stats --------------------------------------------------------------
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0

// ---- Checksum offload (we compute in software) --------------------------
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

// ---- Debug --------------------------------------------------------------
#define LWIP_DEBUG                      0

#endif // LWIPOPTS_H
