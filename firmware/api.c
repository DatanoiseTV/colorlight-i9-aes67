// SPDX-License-Identifier: MIT
// REST API handlers for the AES67 SoC.

#include "api.h"
#include "httpd_lite.h"
#include "ptp_bmc.h"
#include "sap.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include <generated/csr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern struct netif *netif_default;

// ---- Hostname & clock identity (derived from MAC lower 24 bits) ----------
static char    s_hostname[32];
static uint8_t s_clock_id[8];

void unit_hostname_init(void)
{
    // Read the locally-administered MAC from the hardware engine
    uint64_t mac = aes67_local_mac_read();
    uint32_t serial = (uint32_t)(mac & 0xFFFFFF);  // lower 24 bits

    // hostname: aes67br-XXXXXX (6 hex digits)
    snprintf(s_hostname, sizeof(s_hostname), "aes67br-%06lX",
             (unsigned long)serial);

    // EUI-64 clock identity from MAC: insert FFFE in the middle (RFC §7.5.2.2.2)
    s_clock_id[0] = (mac >> 40) & 0xFF;
    s_clock_id[1] = (mac >> 32) & 0xFF;
    s_clock_id[2] = (mac >> 24) & 0xFF;
    s_clock_id[3] = 0xFF;
    s_clock_id[4] = 0xFE;
    s_clock_id[5] = (mac >> 16) & 0xFF;
    s_clock_id[6] = (mac >>  8) & 0xFF;
    s_clock_id[7] =  mac        & 0xFF;
}

const char *unit_hostname(void) { return s_hostname; }

void unit_clock_id(uint8_t out[8]) { memcpy(out, s_clock_id, 8); }

// ---- Tiny JSON builders --------------------------------------------------
static char s_json[2048];

static int j_kv_u(char *p, int rem, const char *k, uint64_t v, int last)
{
    return snprintf(p, rem, "\"%s\":%llu%s", k,
                    (unsigned long long)v, last ? "" : ",");
}
static int j_kv_s(char *p, int rem, const char *k, int64_t v, int last)
{
    return snprintf(p, rem, "\"%s\":%lld%s", k,
                    (long long)v, last ? "" : ",");
}
static int j_kv_str(char *p, int rem, const char *k, const char *v, int last)
{
    return snprintf(p, rem, "\"%s\":\"%s\"%s", k, v, last ? "" : ",");
}

static const char *lock_str(uint32_t s)
{
    return s == 2 ? "LOCKED" : (s == 1 ? "LOCKING" : "UNLOCKED");
}

static const char *role_str(void)
{
    switch (ptp_bmc_state()) {
        case PTP_STATE_MASTER: return "MASTER";
        case PTP_STATE_SLAVE:  return "SLAVE";
        case PTP_STATE_PASSIVE:return "PASSIVE";
        case PTP_STATE_LISTENING: return "LISTENING";
        default: return "INIT";
    }
}

// ---- /api/info ------------------------------------------------------------
static void api_info(httpd_conn_t *c, http_method_t m, const char *p,
                     const char *q, const uint8_t *b, size_t bl)
{
    (void)m; (void)p; (void)q; (void)b; (void)bl;
    char *jp = s_json; int rem = sizeof(s_json);
    int n;
    n = snprintf(jp, rem, "{");                  jp += n; rem -= n;
    n = j_kv_str(jp, rem, "hostname", s_hostname, 0); jp += n; rem -= n;
    char macstr[24];
    uint64_t mac = aes67_local_mac_read();
    snprintf(macstr, sizeof(macstr), "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)((mac>>40)&0xFF), (unsigned)((mac>>32)&0xFF),
             (unsigned)((mac>>24)&0xFF), (unsigned)((mac>>16)&0xFF),
             (unsigned)((mac>>8)&0xFF),  (unsigned)( mac     &0xFF));
    n = j_kv_str(jp, rem, "mac", macstr, 0); jp += n; rem -= n;
    char ipstr[24];
    if (netif_default && !ip4_addr_isany(netif_ip4_addr(netif_default))) {
        snprintf(ipstr, sizeof(ipstr), "%s",
                 ip4addr_ntoa(netif_ip4_addr(netif_default)));
    } else {
        strcpy(ipstr, "0.0.0.0");
    }
    n = j_kv_str(jp, rem, "ip", ipstr, 0); jp += n; rem -= n;
    n = j_kv_str(jp, rem, "build", __DATE__ " " __TIME__, 1);
    jp += n; rem -= n;
    snprintf(jp, rem, "}");
    httpd_send_json(c, s_json);
}

// ---- /api/status ----------------------------------------------------------
static void api_status(httpd_conn_t *c, http_method_t m, const char *p,
                        const char *q, const uint8_t *b, size_t bl)
{
    (void)m; (void)p; (void)q; (void)b; (void)bl;
    char *jp = s_json; int rem = sizeof(s_json); int n;

    n = snprintf(jp, rem, "{\"ptp\":{"); jp += n; rem -= n;
    n = j_kv_str(jp, rem, "role", role_str(), 0); jp += n; rem -= n;
    n = j_kv_str(jp, rem, "state", lock_str(aes67_ptp_lock_state_read()), 0);
    jp += n; rem -= n;
    n = j_kv_u(jp, rem, "sec",  aes67_ptp_sec_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "nsec", aes67_ptp_nsec_read(), 0); jp += n; rem -= n;
    n = j_kv_s(jp, rem, "offset_ns",
               (int32_t)aes67_ptp_offset_filt_ns_read(), 0); jp += n; rem -= n;
    n = j_kv_s(jp, rem, "path_delay_ns",
               (int32_t)aes67_ptp_path_delay_ns_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "sync_count", aes67_ptp_sync_count_read(), 1);
    jp += n; rem -= n;

    n = snprintf(jp, rem, "},\"rtp\":["); jp += n; rem -= n;
    // Stream 0
    n = snprintf(jp, rem, "{");                                              jp += n; rem -= n;
    n = j_kv_u(jp, rem, "rx",         aes67_rtp_packets_rx_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "tx",         aes67_rtp_packets_tx_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "seq_errors", aes67_rtp_seq_errors_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "jbuf_depth", aes67_jbuf_depth_read(),     1); jp += n; rem -= n;
    n = snprintf(jp, rem, "},");                                             jp += n; rem -= n;
    // Stream 1
    n = snprintf(jp, rem, "{");                                              jp += n; rem -= n;
    n = j_kv_u(jp, rem, "rx",         aes67_rtp_packets_rx_1_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "tx",         aes67_rtp_packets_tx_1_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "seq_errors", aes67_rtp_seq_errors_1_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "jbuf_depth", aes67_jbuf_depth_1_read(),     1); jp += n; rem -= n;
    n = snprintf(jp, rem, "}]}");

    httpd_send_json(c, s_json);
}

// ---- /api/config ----------------------------------------------------------
static void api_config_get(httpd_conn_t *c)
{
    char *jp = s_json; int rem = sizeof(s_json); int n;
    n = snprintf(jp, rem, "{");                                      jp += n; rem -= n;
    n = j_kv_u(jp, rem, "tx_ssrc",            aes67_tx_ssrc_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "rx_expected_ssrc",   aes67_rx_expected_ssrc_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "payload_type",       aes67_payload_type_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "num_channels",       aes67_num_channels_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "samples_per_packet", aes67_samples_per_packet_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "jbuf_target_depth",  aes67_jbuf_target_depth_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "sample_rate",        aes67_sample_rate_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "rtp_mcast_ip",       aes67_rtp_mcast_ip_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "rtp_port",           aes67_rtp_port_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "kp",                 aes67_kp_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "ki",                 aes67_ki_read(), 0); jp += n; rem -= n;
    n = j_kv_s(jp, rem, "tx_delay_ns",        (int32_t)aes67_tx_delay_ns_read(), 0); jp += n; rem -= n;
    n = j_kv_s(jp, rem, "rx_delay_ns",        (int32_t)aes67_rx_delay_ns_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "filter_shift",       aes67_filter_shift_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "mode_is_master",     aes67_mode_is_master_read(), 0); jp += n; rem -= n;
    n = j_kv_u(jp, rem, "sync_interval_cycles", aes67_sync_interval_cycles_read(), 1);
    jp += n; rem -= n;
    snprintf(jp, rem, "}");
    httpd_send_json(c, s_json);
}

// Very small key=number JSON parser (one level, no strings)
static int json_get_uint(const uint8_t *body, size_t bl, const char *key,
                         uint64_t *out)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr((const char *)body, pat);
    if (!p) return 0;
    p += strlen(pat);
    while (*p && (*p == ' ' || *p == ':' || *p == '\t')) p++;
    if (*p == '"') p++;
    char *end;
    unsigned long long v = strtoull(p, &end, 0);
    if (end == p) return 0;
    *out = v;
    (void)bl;
    return 1;
}

static void api_config_post(httpd_conn_t *c, const uint8_t *body, size_t bl)
{
    uint64_t v;
    if (json_get_uint(body, bl, "tx_ssrc", &v))            aes67_tx_ssrc_write(v);
    if (json_get_uint(body, bl, "rx_expected_ssrc", &v))   aes67_rx_expected_ssrc_write(v);
    if (json_get_uint(body, bl, "payload_type", &v))       aes67_payload_type_write(v);
    if (json_get_uint(body, bl, "num_channels", &v))       aes67_num_channels_write(v);
    if (json_get_uint(body, bl, "samples_per_packet", &v)) aes67_samples_per_packet_write(v);
    if (json_get_uint(body, bl, "jbuf_target_depth", &v))  aes67_jbuf_target_depth_write(v);
    if (json_get_uint(body, bl, "sample_rate", &v))        aes67_sample_rate_write(v);
    if (json_get_uint(body, bl, "rtp_mcast_ip", &v))       aes67_rtp_mcast_ip_write(v);
    if (json_get_uint(body, bl, "rtp_port", &v))           aes67_rtp_port_write(v);
    if (json_get_uint(body, bl, "kp", &v))                 aes67_kp_write(v);
    if (json_get_uint(body, bl, "ki", &v))                 aes67_ki_write(v);
    if (json_get_uint(body, bl, "tx_delay_ns", &v))        aes67_tx_delay_ns_write(v);
    if (json_get_uint(body, bl, "rx_delay_ns", &v))        aes67_rx_delay_ns_write(v);
    if (json_get_uint(body, bl, "filter_shift", &v))       aes67_filter_shift_write(v);
    if (json_get_uint(body, bl, "mode_is_master", &v))     aes67_mode_is_master_write(v);
    if (json_get_uint(body, bl, "sync_interval_cycles", &v))
        aes67_sync_interval_cycles_write(v);
    httpd_send_json(c, "{\"ok\":true}");
}

static void api_config(httpd_conn_t *c, http_method_t m, const char *p,
                        const char *q, const uint8_t *b, size_t bl)
{
    (void)p; (void)q;
    if (m == HTTP_GET) api_config_get(c);
    else if (m == HTTP_POST) api_config_post(c, b, bl);
    else httpd_send_404(c);
}

// ---- /api/sap -------------------------------------------------------------
static void api_sap(httpd_conn_t *c, http_method_t m, const char *p,
                     const char *q, const uint8_t *b, size_t bl)
{
    (void)m; (void)p; (void)q; (void)b; (void)bl;
    char *jp = s_json; int rem = sizeof(s_json); int n;
    n = snprintf(jp, rem, "{\"count\":%d,\"sessions\":[", sap_remote_count());
    jp += n; rem -= n;
    int first = 1;
    for (int i = 0; i < 16; i++) {
        const sap_remote_t *r = sap_get_remote(i);
        if (!r) continue;
        n = snprintf(jp, rem, "%s{\"id\":%u,\"sdp_len\":%u}",
                     first ? "" : ",", r->msg_id_hash, r->sdp_len);
        jp += n; rem -= n;
        first = 0;
    }
    snprintf(jp, rem, "]}");
    httpd_send_json(c, s_json);
}

// ---- /api/bmc -------------------------------------------------------------
static void api_bmc(httpd_conn_t *c, http_method_t m, const char *p,
                     const char *q, const uint8_t *b, size_t bl)
{
    (void)m; (void)p; (void)q; (void)b; (void)bl;
    int n = snprintf(s_json, sizeof(s_json),
        "{\"role\":\"%s\",\"is_master\":%lu}",
        role_str(), (unsigned long)aes67_mode_is_master_read());
    (void)n;
    httpd_send_json(c, s_json);
}

// ---- /api/mdio - read or write a PHY register -----------------------------
// GET  /api/mdio?phy=N&reg=M           - read PHY N register M
// POST /api/mdio  {"phy":N,"reg":M,"data":V}
static uint16_t mdio_read(uint8_t phy, uint8_t reg)
{
    aes67_mdio_phy_addr_write(phy & 0x1F);
    aes67_mdio_reg_addr_write(reg & 0x1F);
    aes67_mdio_op_read_write(1);
    aes67_mdio_start_write(1);
    while (aes67_mdio_busy_read())
        ;
    return aes67_mdio_read_data_read();
}

static void mdio_write(uint8_t phy, uint8_t reg, uint16_t data)
{
    aes67_mdio_phy_addr_write(phy & 0x1F);
    aes67_mdio_reg_addr_write(reg & 0x1F);
    aes67_mdio_write_data_write(data);
    aes67_mdio_op_read_write(0);
    aes67_mdio_start_write(1);
    while (aes67_mdio_busy_read())
        ;
}

static void api_mdio(httpd_conn_t *c, http_method_t m, const char *p,
                      const char *q, const uint8_t *b, size_t bl)
{
    (void)p;
    if (m == HTTP_GET) {
        uint32_t phy = httpd_query_get_uint(q, "phy", 0);
        uint32_t reg = httpd_query_get_uint(q, "reg", 0);
        uint16_t v   = mdio_read(phy, reg);
        snprintf(s_json, sizeof(s_json),
                 "{\"phy\":%lu,\"reg\":%lu,\"data\":%u}",
                 (unsigned long)phy, (unsigned long)reg, v);
        httpd_send_json(c, s_json);
    } else if (m == HTTP_POST) {
        uint64_t phy, reg, data;
        if (json_get_uint(b, bl, "phy",  &phy) &&
            json_get_uint(b, bl, "reg",  &reg) &&
            json_get_uint(b, bl, "data", &data)) {
            mdio_write(phy, reg, data);
            httpd_send_json(c, "{\"ok\":true}");
        } else {
            httpd_send_json(c, "{\"ok\":false}");
        }
    } else {
        httpd_send_404(c);
    }
}

// ---- /api/dsp -------------------------------------------------------------
static void api_dsp_get(httpd_conn_t *c)
{
    char *jp = s_json; int rem = sizeof(s_json); int n;
    n = snprintf(jp, rem, "{\"channels\":["); jp += n; rem -= n;
    for (int i = 0; i < 16; i++) {
        aes67_dsp_meter_ch_sel_write(i);
        // Wait one bus cycle for register update (no-op on Wishbone, just sequence)
        uint32_t meter = aes67_dsp_meter_value_read();
        aes67_dsp_cfg_ch_sel_write(i);
        uint32_t gain  = aes67_dsp_gain_val_read();
        uint32_t mute  = aes67_dsp_mute_val_read();
        n = snprintf(jp, rem, "%s{\"ch\":%d,\"gain\":%lu,\"mute\":%lu,\"peak\":%lu}",
                     i ? "," : "", i,
                     (unsigned long)gain,
                     (unsigned long)mute,
                     (unsigned long)meter);
        jp += n; rem -= n;
    }
    snprintf(jp, rem, "]}");
    httpd_send_json(c, s_json);
}

static void api_dsp_post(httpd_conn_t *c, const uint8_t *body, size_t bl)
{
    uint64_t ch, v;
    if (json_get_uint(body, bl, "ch", &ch)) {
        aes67_dsp_cfg_ch_sel_write(ch & 0xF);
        if (json_get_uint(body, bl, "gain", &v)) {
            aes67_dsp_gain_val_write(v);
            aes67_dsp_gain_we_write(1);
        }
        if (json_get_uint(body, bl, "mute", &v)) {
            aes67_dsp_mute_val_write(v ? 1 : 0);
            aes67_dsp_mute_we_write(1);
        }
    }
    httpd_send_json(c, "{\"ok\":true}");
}

static void api_dsp(httpd_conn_t *c, http_method_t m, const char *p,
                     const char *q, const uint8_t *b, size_t bl)
{
    (void)p; (void)q;
    if (m == HTTP_GET) api_dsp_get(c);
    else if (m == HTTP_POST) api_dsp_post(c, b, bl);
    else httpd_send_404(c);
}

// ---- Registration --------------------------------------------------------
void api_register_routes(void)
{
    httpd_lite_route("/api/info",   api_info);
    httpd_lite_route("/api/status", api_status);
    httpd_lite_route("/api/config", api_config);
    httpd_lite_route("/api/sap",    api_sap);
    httpd_lite_route("/api/bmc",    api_bmc);
    httpd_lite_route("/api/dsp",    api_dsp);
    httpd_lite_route("/api/mdio",   api_mdio);
}
