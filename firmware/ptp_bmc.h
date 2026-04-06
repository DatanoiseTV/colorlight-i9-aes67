// SPDX-License-Identifier: MIT
// PTP Best Master Clock Algorithm (RFC 1588-2008 §9)
//
// Listens for Announce messages on UDP port 320, maintains a foreign-
// master cache, runs the data set comparison algorithm, decides whether
// to be MASTER or SLAVE, sets the hardware mode CSR, and (when MASTER)
// periodically sends Announce messages itself.
//
// Hardware handles all Sync/FollowUp/DelayReq/DelayResp generation and
// reception with hardware timestamps once the mode CSR is set.

#ifndef PTP_BMC_H
#define PTP_BMC_H

#include <stdint.h>
#include <stdbool.h>

#define PTP_BMC_MAX_FOREIGN 8

typedef struct {
    bool      valid;
    uint8_t   clock_id[8];
    uint8_t   priority1;
    uint8_t   clock_class;
    uint8_t   clock_accuracy;
    uint16_t  offset_scaled_log_variance;
    uint8_t   priority2;
    uint16_t  steps_removed;
    uint8_t   time_source;
    uint32_t  last_seen_ms;
    uint16_t  qualified_announce_count;  // for "qualification" per §9.3.2.4.4
} ptp_foreign_master_t;

typedef enum {
    PTP_STATE_INITIALIZING,
    PTP_STATE_LISTENING,
    PTP_STATE_PRE_MASTER,
    PTP_STATE_MASTER,
    PTP_STATE_PASSIVE,
    PTP_STATE_UNCALIBRATED,
    PTP_STATE_SLAVE,
} ptp_port_state_t;

void ptp_bmc_init(void);
void ptp_bmc_poll(void);
ptp_port_state_t ptp_bmc_state(void);

// Local clock data set fields (configured at boot, can be changed at runtime)
void ptp_bmc_set_priority1(uint8_t p);
void ptp_bmc_set_priority2(uint8_t p);
void ptp_bmc_set_clock_class(uint8_t c);
void ptp_bmc_set_clock_accuracy(uint8_t a);
void ptp_bmc_set_local_clock_id(const uint8_t id[8]);

#endif
