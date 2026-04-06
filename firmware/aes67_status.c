// SPDX-License-Identifier: MIT
// HTTPD CGI handler that exposes AES67 hardware status as JSON.
// Registered with lwIP's httpd as `/status.cgi`.

#include "lwip/apps/httpd.h"
#include <generated/csr.h>
#include <stdio.h>
#include <string.h>

static char s_json_buf[768];

static const char *aes67_status_cgi(int index, int num_params,
                                     char *param[], char *value[])
{
    (void)index; (void)num_params; (void)param; (void)value;

    static const char *lock_names[] = { "UNLOCKED", "LOCKING", "LOCKED" };
    uint32_t lock_state = aes67_ptp_lock_state_read();
    if (lock_state > 2) lock_state = 0;

    snprintf(s_json_buf, sizeof(s_json_buf),
        "{"
        "\"ptp\":{"
            "\"state\":\"%s\","
            "\"sec\":%llu,"
            "\"nsec\":%lu,"
            "\"offset_ns\":%ld,"
            "\"path_delay_ns\":%ld,"
            "\"sync_count\":%lu,"
            "\"last_seq\":%lu"
        "},"
        "\"rtp\":{"
            "\"rx\":%lu,"
            "\"tx\":%lu,"
            "\"seq_errors\":%lu,"
            "\"jbuf_depth\":%lu"
        "}"
        "}",
        lock_names[lock_state],
        (unsigned long long)aes67_ptp_sec_read(),
        (unsigned long)aes67_ptp_nsec_read(),
        (long)aes67_ptp_offset_filt_ns_read(),
        (long)aes67_ptp_path_delay_ns_read(),
        (unsigned long)aes67_ptp_sync_count_read(),
        (unsigned long)aes67_ptp_last_seq_read(),
        (unsigned long)aes67_rtp_packets_rx_read(),
        (unsigned long)aes67_rtp_packets_tx_read(),
        (unsigned long)aes67_rtp_seq_errors_read(),
        (unsigned long)aes67_jbuf_depth_read());

    // The httpd CGI mechanism returns a URI for the response body. We use a
    // SSI tag instead in real builds; for simplicity here we just return the
    // index page (which fetches /status.json via XHR).
    return "/status.json";
}

static const tCGI s_cgi_handlers[] = {
    { "/status.cgi", aes67_status_cgi },
};

void aes67_status_init(void)
{
    http_set_cgi_handlers(s_cgi_handlers, LWIP_ARRAYSIZE(s_cgi_handlers));
}

const char *aes67_status_get_json(void)
{
    aes67_status_cgi(0, 0, NULL, NULL);
    return s_json_buf;
}
