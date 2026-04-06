// SPDX-License-Identifier: MIT
// lwIP NO_SYS time source: derived from the hardware PTP clock.
//
// `sys_now()` returns a free-running millisecond counter. lwIP uses it for
// retransmit timers, DHCP timeouts, mDNS scheduling, etc.

#include <stdint.h>
#include <generated/csr.h>
#include "arch/cc.h"

uint32_t sys_now(void)
{
    // ptp_sec * 1000 + ptp_nsec / 1_000_000
    uint64_t sec = aes67_ptp_sec_read();
    uint32_t ns  = aes67_ptp_nsec_read();
    return (uint32_t)(sec * 1000U + ns / 1000000U);
}

unsigned int lwip_rand(void)
{
    // Mix nanoseconds with a Linear Congruential Generator
    static uint32_t state = 0xCAFEBABE;
    uint32_t ns = aes67_ptp_nsec_read();
    state = state * 1664525U + 1013904223U + ns;
    return state;
}
