// SPDX-License-Identifier: MIT
// REST API handlers for the AES67 SoC.
//
// All endpoints return JSON. Routes:
//   GET  /api/info          unit identity (hostname, MAC, IP, build)
//   GET  /api/status        live stats (PTP, RTP, jitter buffer, etc.)
//   GET  /api/config        all configurable values
//   POST /api/config        update one or more values (JSON body)
//   GET  /api/sap           discovered SAP sessions
//   GET  /api/bmc           PTP foreign-master cache + role
//   GET  /api/dsp           per-channel gains/mutes/meters
//   POST /api/dsp           set per-channel gains/mutes (JSON body)

#ifndef API_H
#define API_H

#include <stdint.h>

void api_register_routes(void);

// Hostname helpers (used by mDNS, DHCP, SAP, BMC)
void        unit_hostname_init(void);
const char *unit_hostname(void);
void        unit_clock_id(uint8_t out[8]);

#endif
