// SPDX-License-Identifier: MIT
// PTP Best Master Clock Algorithm (RFC 1588-2008 §9)
//
// Implementation overview:
//
//   1. Bind a UDP socket to port 320 (PTP general).
//   2. On each received Announce message:
//        - parse grandmaster data set (priority1, clock quality, ...)
//        - update foreign master cache, qualifying after 2 received
//          announces within ANNOUNCE_RECEIPT_TIMEOUT (RFC §9.3.2.4.4)
//   3. Periodically (every announce interval) run the BMC:
//        - select the best foreign master (Erbest) per §9.3.4
//        - compare with our local clock (Ebest determination per §9.3.5)
//        - if local wins -> state = MASTER, set hardware mode_is_master = 1
//        - if remote wins -> state = SLAVE, set hardware mode_is_master = 0
//   4. When MASTER, periodically send Announce messages over UDP to
//      224.0.1.129:320 with our own clock data set.
//
// Hardware ptp_pp generates Sync/Follow_Up (master) or processes them
// (slave) once the mode CSR is set.

#include "ptp_bmc.h"
#include "lwip/udp.h"
#include "lwip/igmp.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/ip_addr.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

#define PTP_GENERAL_PORT      320
#define PTP_ANNOUNCE_INTERVAL_MS  1000   // log = 0  (1 s)
#define PTP_ANNOUNCE_TIMEOUT_MS   3500   // 3.5 × interval (RFC default)
#define PTP_BMC_TICK_MS           250

// PTP message types
#define MSG_ANNOUNCE 0xB

// PTP multicast: All Nodes group, IPv4
static const uint8_t PTP_MCAST_OCT[4] = {224, 0, 1, 129};

extern struct netif *netif_default;

static struct udp_pcb       *s_pcb;
static ip_addr_t             s_group;
static ptp_port_state_t      s_state = PTP_STATE_INITIALIZING;
static uint32_t              s_last_announce_tx_ms;
static uint32_t              s_last_bmc_tick_ms;
static uint16_t              s_seq_id;

// ---- Local clock data set ------------------------------------------------
static uint8_t  s_local_clock_id[8] = {0x02, 0xAE, 0x67, 0xFF, 0xFE, 0x00, 0x00, 0x01};
static uint8_t  s_local_priority1   = 128;   // typical default
static uint8_t  s_local_priority2   = 128;
static uint8_t  s_local_clock_class = 248;   // default class for ordinary clock
static uint8_t  s_local_clock_accuracy   = 0xFE;  // unknown
static uint16_t s_local_offset_var       = 0xFFFF;
static uint8_t  s_local_time_source      = 0xA0;  // INTERNAL_OSCILLATOR

static ptp_foreign_master_t s_foreign[PTP_BMC_MAX_FOREIGN];

// ---- API setters ----------------------------------------------------------
void ptp_bmc_set_priority1(uint8_t p)      { s_local_priority1   = p; }
void ptp_bmc_set_priority2(uint8_t p)      { s_local_priority2   = p; }
void ptp_bmc_set_clock_class(uint8_t c)    { s_local_clock_class = c; }
void ptp_bmc_set_clock_accuracy(uint8_t a) { s_local_clock_accuracy = a; }
void ptp_bmc_set_local_clock_id(const uint8_t id[8]) {
    memcpy(s_local_clock_id, id, 8);
}

ptp_port_state_t ptp_bmc_state(void) { return s_state; }

// ---- Compare two clock identities (lower wins, RFC §9.3.4 D5) ------------
static int clock_id_cmp(const uint8_t a[8], const uint8_t b[8]) {
    return memcmp(a, b, 8);
}

// ---- Data set comparison algorithm (RFC §9.3.4) --------------------------
// Returns < 0 if A is better, > 0 if B is better, 0 if equal.
typedef struct {
    uint8_t  clock_id[8];
    uint8_t  priority1;
    uint8_t  clock_class;
    uint8_t  clock_accuracy;
    uint16_t offset_var;
    uint8_t  priority2;
    uint16_t steps_removed;
} bmc_dataset_t;

static int bmc_compare(const bmc_dataset_t *a, const bmc_dataset_t *b)
{
    // Path 1: identity comparison (RFC §9.3.4 figure 27)
    if (a->priority1 != b->priority1)
        return (int)a->priority1 - (int)b->priority1;
    if (a->clock_class != b->clock_class)
        return (int)a->clock_class - (int)b->clock_class;
    if (a->clock_accuracy != b->clock_accuracy)
        return (int)a->clock_accuracy - (int)b->clock_accuracy;
    if (a->offset_var != b->offset_var)
        return (int)a->offset_var - (int)b->offset_var;
    if (a->priority2 != b->priority2)
        return (int)a->priority2 - (int)b->priority2;
    return clock_id_cmp(a->clock_id, b->clock_id);
}

// ---- Local data set ------------------------------------------------------
static void local_dataset(bmc_dataset_t *out)
{
    memcpy(out->clock_id, s_local_clock_id, 8);
    out->priority1      = s_local_priority1;
    out->clock_class    = s_local_clock_class;
    out->clock_accuracy = s_local_clock_accuracy;
    out->offset_var     = s_local_offset_var;
    out->priority2      = s_local_priority2;
    out->steps_removed  = 0;
}

// ---- Foreign master cache management -------------------------------------
static ptp_foreign_master_t *find_or_alloc_foreign(const uint8_t clock_id[8])
{
    int free_slot = -1;
    uint32_t now = sys_now();
    uint32_t oldest_age = 0;
    int oldest_slot = 0;

    for (int i = 0; i < PTP_BMC_MAX_FOREIGN; i++) {
        if (s_foreign[i].valid && memcmp(s_foreign[i].clock_id, clock_id, 8) == 0)
            return &s_foreign[i];
        if (!s_foreign[i].valid && free_slot < 0)
            free_slot = i;
        if (s_foreign[i].valid) {
            uint32_t age = now - s_foreign[i].last_seen_ms;
            if (age > oldest_age) { oldest_age = age; oldest_slot = i; }
        }
    }
    if (free_slot >= 0) return &s_foreign[free_slot];
    // Evict oldest
    return &s_foreign[oldest_slot];
}

// ---- Announce message TX -------------------------------------------------
//
// Announce body (after PTP common header, 34 bytes):
//   34-43: originTimestamp (10 bytes)
//   44-45: currentUtcOffset
//   46:    reserved
//   47:    grandmasterPriority1
//   48:    grandmasterClockClass
//   49:    grandmasterClockAccuracy
//   50-51: grandmasterOffsetScaledLogVariance
//   52:    grandmasterPriority2
//   53-60: grandmasterIdentity
//   61-62: stepsRemoved
//   63:    timeSource
// Total length: 64 bytes

static void send_announce(void)
{
    if (!s_pcb) return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 64, PBUF_RAM);
    if (!p) return;

    uint8_t *b = (uint8_t *)p->payload;
    memset(b, 0, 64);

    b[0]  = MSG_ANNOUNCE;       // transportSpecific=0, msgType=0xB
    b[1]  = 0x02;               // versionPTP=2
    b[2]  = 0;
    b[3]  = 64;                 // messageLength
    b[4]  = 0;                  // domain
    b[5]  = 0;
    b[6]  = 0;                  // flagField hi
    b[7]  = 0;                  // flagField lo
    // correctionField (8 bytes) = 0
    // reserved (4 bytes) = 0
    memcpy(b + 20, s_local_clock_id, 8);  // sourcePortIdentity.clockIdentity
    b[28] = 0;                  // portNumber hi
    b[29] = 1;                  // portNumber lo
    b[30] = (s_seq_id >> 8) & 0xFF;
    b[31] =  s_seq_id       & 0xFF;
    s_seq_id++;
    b[32] = 0x05;               // controlField for Announce
    b[33] = 0;                  // logMessageInterval = 0 (1 s)

    // originTimestamp = current PTP clock
    uint64_t sec = aes67_ptp_sec_read();
    uint32_t ns  = aes67_ptp_nsec_read();
    b[34] = (sec >> 40) & 0xFF;
    b[35] = (sec >> 32) & 0xFF;
    b[36] = (sec >> 24) & 0xFF;
    b[37] = (sec >> 16) & 0xFF;
    b[38] = (sec >>  8) & 0xFF;
    b[39] =  sec        & 0xFF;
    b[40] = (ns >> 24) & 0xFF;
    b[41] = (ns >> 16) & 0xFF;
    b[42] = (ns >>  8) & 0xFF;
    b[43] =  ns        & 0xFF;

    b[44] = 0;                  // currentUtcOffset hi
    b[45] = 37;                 // currentUtcOffset lo (TAI-UTC = 37 s in 2024)
    b[46] = 0;                  // reserved
    b[47] = s_local_priority1;
    b[48] = s_local_clock_class;
    b[49] = s_local_clock_accuracy;
    b[50] = (s_local_offset_var >> 8) & 0xFF;
    b[51] =  s_local_offset_var       & 0xFF;
    b[52] = s_local_priority2;
    memcpy(b + 53, s_local_clock_id, 8);  // grandmasterIdentity (= ours)
    b[61] = 0;                  // stepsRemoved hi
    b[62] = 0;                  // stepsRemoved lo
    b[63] = s_local_time_source;

    udp_sendto(s_pcb, p, &s_group, PTP_GENERAL_PORT);
    pbuf_free(p);
}

// ---- Announce RX handler -------------------------------------------------
static void announce_recv(const uint8_t *b, size_t len)
{
    if (len < 64) return;
    if ((b[0] & 0x0F) != MSG_ANNOUNCE) return;
    if ((b[1] & 0x0F) != 0x02) return;  // versionPTP must be 2

    uint8_t  src_clock_id[8];
    memcpy(src_clock_id, b + 20, 8);

    // Ignore our own announces (loopback or self via switch)
    if (memcmp(src_clock_id, s_local_clock_id, 8) == 0) return;

    ptp_foreign_master_t *fm = find_or_alloc_foreign(src_clock_id);
    bool was_valid = fm->valid;
    memcpy(fm->clock_id, src_clock_id, 8);
    fm->priority1                  = b[47];
    fm->clock_class                = b[48];
    fm->clock_accuracy             = b[49];
    fm->offset_scaled_log_variance = (b[50] << 8) | b[51];
    fm->priority2                  = b[52];
    fm->steps_removed              = (b[61] << 8) | b[62];
    fm->time_source                = b[63];
    fm->last_seen_ms               = sys_now();
    fm->valid                      = true;
    if (!was_valid)
        fm->qualified_announce_count = 1;
    else if (fm->qualified_announce_count < 4)
        fm->qualified_announce_count++;
}

// ---- UDP RX callback -----------------------------------------------------
static void ptp_udp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (!p) return;

    if (p->tot_len >= 64 && p->tot_len < 256) {
        uint8_t buf[256];
        pbuf_copy_partial(p, buf, p->tot_len, 0);
        if ((buf[0] & 0x0F) == MSG_ANNOUNCE)
            announce_recv(buf, p->tot_len);
    }

    pbuf_free(p);
}

// ---- Run BMC and update state ---------------------------------------------
static void run_bmc(void)
{
    uint32_t now = sys_now();

    // Age out foreign masters
    for (int i = 0; i < PTP_BMC_MAX_FOREIGN; i++) {
        if (s_foreign[i].valid &&
            (now - s_foreign[i].last_seen_ms) > PTP_ANNOUNCE_TIMEOUT_MS) {
            s_foreign[i].valid = false;
        }
    }

    // Find Erbest: best qualified foreign master
    int erbest = -1;
    bmc_dataset_t erbest_ds;
    memset(&erbest_ds, 0xFF, sizeof(erbest_ds));
    for (int i = 0; i < PTP_BMC_MAX_FOREIGN; i++) {
        if (!s_foreign[i].valid) continue;
        if (s_foreign[i].qualified_announce_count < 2) continue;  // §9.3.2.4.4
        bmc_dataset_t ds;
        memcpy(ds.clock_id, s_foreign[i].clock_id, 8);
        ds.priority1      = s_foreign[i].priority1;
        ds.clock_class    = s_foreign[i].clock_class;
        ds.clock_accuracy = s_foreign[i].clock_accuracy;
        ds.offset_var     = s_foreign[i].offset_scaled_log_variance;
        ds.priority2      = s_foreign[i].priority2;
        ds.steps_removed  = s_foreign[i].steps_removed;
        if (erbest < 0 || bmc_compare(&ds, &erbest_ds) < 0) {
            erbest = i;
            erbest_ds = ds;
        }
    }

    bmc_dataset_t local_ds;
    local_dataset(&local_ds);

    ptp_port_state_t new_state;
    if (erbest < 0) {
        // No foreign masters - we are the master (if our class qualifies)
        new_state = PTP_STATE_MASTER;
    } else {
        int cmp = bmc_compare(&local_ds, &erbest_ds);
        if (cmp < 0) {
            // Local wins
            new_state = PTP_STATE_MASTER;
        } else if (cmp > 0) {
            // Foreign wins
            new_state = PTP_STATE_SLAVE;
        } else {
            // Equal (shouldn't happen with unique clock IDs)
            new_state = PTP_STATE_PASSIVE;
        }
    }

    if (new_state != s_state) {
        printf("BMC: state %d -> %d (%s)\n", s_state, new_state,
               new_state == PTP_STATE_MASTER ? "MASTER" :
               new_state == PTP_STATE_SLAVE  ? "SLAVE"  : "OTHER");
        s_state = new_state;
        // Set hardware mode CSR
        aes67_mode_is_master_write(new_state == PTP_STATE_MASTER ? 1 : 0);
    }
}

// ---- Init / poll ----------------------------------------------------------
void ptp_bmc_init(void)
{
    IP4_ADDR(&s_group, 224, 0, 1, 129);

    s_pcb = udp_new();
    if (!s_pcb) return;
    udp_recv(s_pcb, ptp_udp_recv, NULL);
    udp_bind(s_pcb, IP_ADDR_ANY, PTP_GENERAL_PORT);

    if (netif_default)
        igmp_joingroup_netif(netif_default, ip_2_ip4(&s_group));

    memset(s_foreign, 0, sizeof(s_foreign));
    s_state = PTP_STATE_LISTENING;
    s_last_announce_tx_ms = 0;
    s_last_bmc_tick_ms    = 0;
    s_seq_id              = 0;

    // Push the local clock identity into the hardware engine
    uint64_t cid64 = 0;
    for (int i = 0; i < 8; i++)
        cid64 = (cid64 << 8) | s_local_clock_id[i];
    aes67_local_clock_id_write(cid64);

    // Default Sync interval = 125 ms (good for AES67)
    aes67_sync_interval_cycles_write(125000000 / 8);  // at 125 MHz, 1 cycle = 8 ns

    printf("BMC: bound port 320, group 224.0.1.129\n");
}

void ptp_bmc_poll(void)
{
    uint32_t now = sys_now();

    if ((now - s_last_bmc_tick_ms) >= PTP_BMC_TICK_MS) {
        s_last_bmc_tick_ms = now;
        run_bmc();
    }

    if (s_state == PTP_STATE_MASTER &&
        (now - s_last_announce_tx_ms) >= PTP_ANNOUNCE_INTERVAL_MS) {
        s_last_announce_tx_ms = now;
        send_announce();
    }
}
