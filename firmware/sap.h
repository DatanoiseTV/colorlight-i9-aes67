// SPDX-License-Identifier: MIT
// SAP - Session Announcement Protocol (RFC 2974)
//
// Periodically multicasts an SDP description of the local AES67 source
// session to 239.255.255.255:9875 and listens for remote announcements.
// Discovered remote sources are stored in a small cache for the CPU
// firmware to inspect (and potentially auto-subscribe to).

#ifndef SAP_H
#define SAP_H

#include <stdint.h>
#include "lwip/ip_addr.h"

#define SAP_MAX_REMOTE 16
#define SAP_SDP_MAX    1024

typedef struct {
    int       valid;
    uint16_t  msg_id_hash;
    ip_addr_t origin_ip;
    uint32_t  last_seen_ms;
    char      sdp[SAP_SDP_MAX];
    uint16_t  sdp_len;
} sap_remote_t;

void sap_init(void);
void sap_poll(void);

const sap_remote_t *sap_get_remote(int idx);
int  sap_remote_count(void);

// Manually trigger an announcement of the local session (also done
// periodically by sap_poll() at the SAP rate).
void sap_announce_now(void);

#endif
