// SPDX-License-Identifier: MIT
// SAP - Session Announcement Protocol (RFC 2974)
//
// Generates an AES67/RAVENNA-compliant SDP description and announces it
// over multicast UDP to 239.255.255.255:9875. Listens on the same group
// for announcements from other devices and caches them.
//
// SAP packet header (RFC 2974 §3):
//
//      0                   1                   2                   3
//      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//     | V=1 |A|R|T|E|C|   auth len    |        msg id hash            |
//     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//     |              originating source (IPv4: 4 bytes)               |
//     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//     | optional payload type (e.g. "application/sdp\0")              |
//     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//     | SDP description                                               |
//
// Flag bits in octet 0 (low to high): V=1 (version 1), A=address type
// (0=IPv4), R=reserved, T=message type (0=announcement, 1=deletion),
// E=encrypted, C=compressed.

#include "sap.h"
#include "api.h"     // unit_hostname()
#include "lwip/udp.h"
#include "lwip/igmp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

#define SAP_PORT          9875
#define SAP_GROUP_OCT_A   239
#define SAP_GROUP_OCT_B   255
#define SAP_GROUP_OCT_C   255
#define SAP_GROUP_OCT_D   255
#define SAP_PAYLOAD_TYPE  "application/sdp"
#define SAP_INTERVAL_MS   10000   // RFC 2974 recommends adaptive; 10s is safe
#define SAP_REMOTE_TIMEOUT_MS 30000

static struct udp_pcb *s_pcb;
static ip_addr_t       s_group;
static uint32_t        s_last_announce_ms;
static char            s_local_sdp[SAP_SDP_MAX];
static uint16_t        s_local_sdp_len;
static uint16_t        s_msg_id_hash;
static sap_remote_t    s_remote[SAP_MAX_REMOTE];

extern struct netif *netif_default;

// CRC-16-CCITT for the message ID hash
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}

// ---- Build the AES67-compliant SDP for our local source ------------------
static void sap_build_local_sdp(void)
{
    uint32_t local_ip   = aes67_local_ip_read();
    uint32_t mcast_ip   = aes67_rtp_mcast_ip_read();
    uint16_t rtp_port   = aes67_rtp_port_read();
    uint8_t  pt         = aes67_payload_type_read();
    uint8_t  channels   = aes67_num_channels_read();
    uint32_t sample_rate= aes67_sample_rate_read();
    uint64_t clock_id   = aes67_local_clock_id_read();
    uint16_t pkt_samples = aes67_samples_per_packet_read();
    uint32_t ptime_us   = (pkt_samples * 1000000U) / sample_rate;

    // Session-level
    int n = snprintf(s_local_sdp, sizeof(s_local_sdp),
        "v=0\r\n"
        "o=- %u 0 IN IP4 %u.%u.%u.%u\r\n"
        "s=%s\r\n"
        "c=IN IP4 %u.%u.%u.%u/64\r\n"
        "t=0 0\r\n"
        "a=clock-domain:PTPv2 0\r\n"
        "m=audio %u RTP/AVP %u\r\n"
        "i=AES67 24-bit %uch %u kHz\r\n"
        "a=rtpmap:%u L24/%u/%u\r\n"
        "a=sendonly\r\n"
        "a=ptime:%u.%03u\r\n"
        "a=ts-refclk:ptp=IEEE1588-2008:%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X:0\r\n"
        "a=mediaclk:direct=0\r\n"
        "a=framecount:%u\r\n",
        (unsigned)(aes67_ptp_sec_read() & 0xFFFFFFFF),
        (unsigned)((local_ip >> 24) & 0xFF),
        (unsigned)((local_ip >> 16) & 0xFF),
        (unsigned)((local_ip >>  8) & 0xFF),
        (unsigned)( local_ip        & 0xFF),
        unit_hostname(),
        (unsigned)((mcast_ip >> 24) & 0xFF),
        (unsigned)((mcast_ip >> 16) & 0xFF),
        (unsigned)((mcast_ip >>  8) & 0xFF),
        (unsigned)( mcast_ip        & 0xFF),
        (unsigned)rtp_port, (unsigned)pt,
        (unsigned)channels, (unsigned)(sample_rate / 1000),
        (unsigned)pt, (unsigned)sample_rate, (unsigned)channels,
        (unsigned)(ptime_us / 1000), (unsigned)(ptime_us % 1000),
        (unsigned)((clock_id >> 56) & 0xFF), (unsigned)((clock_id >> 48) & 0xFF),
        (unsigned)((clock_id >> 40) & 0xFF), (unsigned)((clock_id >> 32) & 0xFF),
        (unsigned)((clock_id >> 24) & 0xFF), (unsigned)((clock_id >> 16) & 0xFF),
        (unsigned)((clock_id >>  8) & 0xFF), (unsigned)( clock_id        & 0xFF),
        (unsigned)pkt_samples);

    if (n < 0 || n >= (int)sizeof(s_local_sdp))
        n = (int)sizeof(s_local_sdp) - 1;
    s_local_sdp_len = (uint16_t)n;

    // Hash for SAP message-id field
    s_msg_id_hash = crc16_ccitt((const uint8_t *)s_local_sdp, s_local_sdp_len);
}

// ---- Send a SAP announcement (or deletion) -------------------------------
static void sap_send(uint8_t msg_type)
{
    if (!s_pcb) return;

    sap_build_local_sdp();

    size_t pt_len    = strlen(SAP_PAYLOAD_TYPE) + 1;     // includes NUL
    size_t total_len = 8 + pt_len + s_local_sdp_len;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total_len, PBUF_RAM);
    if (!p) return;

    uint8_t *buf = (uint8_t *)p->payload;
    // SAP header
    buf[0] = 0x20 | (msg_type ? 0x04 : 0x00);   // V=1, A=0 (IPv4), T=msg_type
    buf[1] = 0;                                  // auth length
    buf[2] = (s_msg_id_hash >> 8) & 0xFF;
    buf[3] =  s_msg_id_hash       & 0xFF;
    uint32_t local_ip = aes67_local_ip_read();
    buf[4] = (local_ip >> 24) & 0xFF;
    buf[5] = (local_ip >> 16) & 0xFF;
    buf[6] = (local_ip >>  8) & 0xFF;
    buf[7] =  local_ip        & 0xFF;
    // Payload type
    memcpy(buf + 8, SAP_PAYLOAD_TYPE, pt_len);
    // SDP
    memcpy(buf + 8 + pt_len, s_local_sdp, s_local_sdp_len);

    udp_sendto(s_pcb, p, &s_group, SAP_PORT);
    pbuf_free(p);
}

void sap_announce_now(void)
{
    sap_send(0);
    s_last_announce_ms = sys_now();
}

// ---- Receive callback -----------------------------------------------------
static void sap_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (!p || p->tot_len < 8) { if (p) pbuf_free(p); return; }

    uint8_t hdr[8];
    pbuf_copy_partial(p, hdr, 8, 0);

    // Decode header
    uint8_t  flags   = hdr[0];
    uint8_t  ver     = (flags >> 5) & 0x07;
    uint8_t  msg_type= (flags >> 2) & 0x01;
    uint8_t  auth    = hdr[1];
    uint16_t msg_id  = (hdr[2] << 8) | hdr[3];

    if (ver != 1) { pbuf_free(p); return; }

    // Strip header + auth + payload type string
    size_t off = 8 + (size_t)auth * 4;

    // Find end of payload type string (NUL-terminated)
    char ptype[32];
    size_t i = 0;
    while (off + i < p->tot_len && i < sizeof(ptype) - 1) {
        uint8_t b;
        pbuf_copy_partial(p, &b, 1, off + i);
        ptype[i++] = (char)b;
        if (b == 0) break;
    }
    ptype[i] = 0;
    if (i == 0 || ptype[i - 1] != 0) i = 0;
    off += i;

    if (off >= p->tot_len) { pbuf_free(p); return; }
    size_t sdp_len = p->tot_len - off;
    if (sdp_len >= SAP_SDP_MAX) sdp_len = SAP_SDP_MAX - 1;

    // Find or allocate cache slot for this message id
    int slot = -1, free_slot = -1;
    for (int k = 0; k < SAP_MAX_REMOTE; k++) {
        if (s_remote[k].valid && s_remote[k].msg_id_hash == msg_id) { slot = k; break; }
        if (!s_remote[k].valid && free_slot < 0) free_slot = k;
    }
    if (slot < 0) slot = free_slot;
    if (slot < 0) slot = 0;

    if (msg_type == 1) {
        // Deletion
        s_remote[slot].valid = 0;
    } else {
        s_remote[slot].valid       = 1;
        s_remote[slot].msg_id_hash = msg_id;
        s_remote[slot].last_seen_ms= sys_now();
        pbuf_copy_partial(p, s_remote[slot].sdp, sdp_len, off);
        s_remote[slot].sdp[sdp_len] = 0;
        s_remote[slot].sdp_len = sdp_len;
    }

    pbuf_free(p);
}

// ---- Init / poll ----------------------------------------------------------
void sap_init(void)
{
    IP4_ADDR(&s_group, SAP_GROUP_OCT_A, SAP_GROUP_OCT_B,
                       SAP_GROUP_OCT_C, SAP_GROUP_OCT_D);

    s_pcb = udp_new();
    if (!s_pcb) return;

    udp_recv(s_pcb, sap_recv, NULL);
    udp_bind(s_pcb, IP_ADDR_ANY, SAP_PORT);

    if (netif_default) {
        igmp_joingroup_netif(netif_default, ip_2_ip4(&s_group));
    }

    s_last_announce_ms = 0;
    memset(s_remote, 0, sizeof(s_remote));
    printf("SAP: bound, group=239.255.255.255:9875\n");
}

void sap_poll(void)
{
    uint32_t now = sys_now();

    if ((now - s_last_announce_ms) >= SAP_INTERVAL_MS) {
        sap_announce_now();
    }

    // Age out remote entries
    for (int k = 0; k < SAP_MAX_REMOTE; k++) {
        if (s_remote[k].valid &&
            (now - s_remote[k].last_seen_ms) > SAP_REMOTE_TIMEOUT_MS) {
            s_remote[k].valid = 0;
        }
    }
}

const sap_remote_t *sap_get_remote(int idx)
{
    if (idx < 0 || idx >= SAP_MAX_REMOTE) return NULL;
    return s_remote[idx].valid ? &s_remote[idx] : NULL;
}

int sap_remote_count(void)
{
    int c = 0;
    for (int k = 0; k < SAP_MAX_REMOTE; k++) if (s_remote[k].valid) c++;
    return c;
}
