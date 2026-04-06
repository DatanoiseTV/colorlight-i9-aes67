// SPDX-License-Identifier: MIT
// LiteX netif for lwIP
//
// The cpu_netif hardware exposes a single TX and a single RX BRAM slot,
// each up to 2 KB. We poll for incoming frames in `litex_netif_poll()`,
// allocate a pbuf, and feed it to lwIP. Outgoing frames are linearized
// into the TX BRAM by `litex_netif_linkoutput()`.

#include "litex_netif.h"
#include <generated/csr.h>
#include <generated/mem.h>

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "lwip/snmp.h"
#include "netif/ethernet.h"
#include <string.h>

#define IFNAME0 'l'
#define IFNAME1 'x'

#define ETH_FRAME_MAX 1518

// ---- Low-level link output -------------------------------------------------
static err_t litex_netif_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;

    // Wait for a free TX slot
    while (cpu_netif_tx_pending_read())
        ;

    volatile uint8_t *txbuf = (volatile uint8_t *)CPU_NETIF_TX_BUF_BASE;
    uint16_t off = 0;

    for (struct pbuf *q = p; q != NULL; q = q->next) {
        if (off + q->len > ETH_FRAME_MAX)
            return ERR_BUF;
        for (uint16_t i = 0; i < q->len; i++)
            txbuf[off + i] = ((const uint8_t *)q->payload)[i];
        off += q->len;
    }

    cpu_netif_tx_length_write(off);
    cpu_netif_tx_send_write(1);

    MIB2_STATS_NETIF_ADD(netif, ifoutoctets, off);
    if (((const uint8_t *)p->payload)[0] & 1)
        MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
    else
        MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
    return ERR_OK;
}

// ---- Low-level RX poll -----------------------------------------------------
void litex_netif_poll(struct netif *netif)
{
    if (!cpu_netif_rx_ready_read())
        return;

    uint16_t len = cpu_netif_rx_length_read();
    if (len == 0 || len > ETH_FRAME_MAX) {
        cpu_netif_rx_ack_write(1);
        return;
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p == NULL) {
        cpu_netif_rx_ack_write(1);
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
        return;
    }

    volatile uint8_t *rxbuf = (volatile uint8_t *)CPU_NETIF_RX_BUF_BASE;
    uint16_t off = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        for (uint16_t i = 0; i < q->len; i++)
            ((uint8_t *)q->payload)[i] = rxbuf[off + i];
        off += q->len;
    }
    cpu_netif_rx_ack_write(1);

    MIB2_STATS_NETIF_ADD(netif, ifinoctets, len);
    if (((const uint8_t *)p->payload)[0] & 1)
        MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
    else
        MIB2_STATS_NETIF_INC(netif, ifinucastpkts);

    if (netif->input(p, netif) != ERR_OK) {
        pbuf_free(p);
    }
}

// ---- Init ------------------------------------------------------------------
err_t litex_netif_init(struct netif *netif)
{
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->output     = etharp_output;
    netif->linkoutput = litex_netif_linkoutput;

    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
                   NETIF_FLAG_ETHERNET  | NETIF_FLAG_IGMP;
    netif->hwaddr_len = ETH_HWADDR_LEN;

    // MAC will be set by the caller before netif_add()
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, 100000000);

    return ERR_OK;
}
