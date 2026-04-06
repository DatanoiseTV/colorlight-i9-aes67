// SPDX-License-Identifier: MIT
// LiteX netif for lwIP — bridges the cpu_netif hardware peripheral
// (BRAM-buffered Ethernet RX/TX) to lwIP's pbuf interface.

#ifndef LITEX_NETIF_H
#define LITEX_NETIF_H

#include "lwip/netif.h"
#include "lwip/err.h"

err_t litex_netif_init(struct netif *netif);
void  litex_netif_poll(struct netif *netif);

#endif
