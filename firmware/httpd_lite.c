// SPDX-License-Identifier: MIT
// httpd_lite: raw-TCP HTTP/1.0 server.
//
// Per-connection state struct holds the request buffer, parsed fields,
// and (for streaming responses) the generator callback. lwIP raw TCP
// callbacks (recv, sent, err, poll) drive everything in the lwIP main
// thread.

#include "httpd_lite.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_ROUTES 16
typedef struct {
    const char       *path;
    httpd_handler_fn  fn;
} route_t;
static route_t s_routes[MAX_ROUTES];
static int     s_num_routes;

#define MAX_CONNS 4

struct httpd_conn {
    struct tcp_pcb *pcb;
    bool            in_use;

    // Request parsing buffer
    uint8_t         req_buf[HTTPD_MAX_PATH + HTTPD_MAX_QUERY + HTTPD_MAX_BODY + 512];
    int             req_len;
    bool            headers_done;
    int             content_length;
    int             body_start;  // offset into req_buf where body starts

    // Parsed request
    http_method_t   method;
    char            path[HTTPD_MAX_PATH];
    char            query[HTTPD_MAX_QUERY];

    // Streaming response state
    httpd_stream_fn stream_fn;
    void           *stream_ctx;
    void          (*on_close)(void *);
    bool            stream_active;

    // Pending output buffer (for httpd_send_body chunks before TCP write)
    uint8_t         out_buf[1024];
    int             out_len;
};

static httpd_conn_t s_conns[MAX_CONNS];

static httpd_conn_t *alloc_conn(void)
{
    for (int i = 0; i < MAX_CONNS; i++)
        if (!s_conns[i].in_use) {
            memset(&s_conns[i], 0, sizeof(s_conns[i]));
            s_conns[i].in_use = true;
            return &s_conns[i];
        }
    return NULL;
}

static void free_conn(httpd_conn_t *c)
{
    if (c->on_close && c->stream_ctx)
        c->on_close(c->stream_ctx);
    c->in_use = false;
    c->pcb    = NULL;
    c->stream_active = false;
    c->stream_fn = NULL;
    c->on_close = NULL;
}

// ---- Route registration ----
void httpd_lite_route(const char *path, httpd_handler_fn fn)
{
    if (s_num_routes >= MAX_ROUTES) return;
    s_routes[s_num_routes].path = path;
    s_routes[s_num_routes].fn   = fn;
    s_num_routes++;
}

static httpd_handler_fn find_handler(const char *path)
{
    for (int i = 0; i < s_num_routes; i++) {
        if (strcmp(s_routes[i].path, path) == 0)
            return s_routes[i].fn;
    }
    return NULL;
}

// ---- HTTP request parsing ----
static int parse_request(httpd_conn_t *c)
{
    // Find end of headers
    char *hdr = (char *)c->req_buf;
    char *end = strstr(hdr, "\r\n\r\n");
    if (!end) return 0;  // need more data
    *end = 0;
    c->body_start = (end - hdr) + 4;

    // Parse request line
    char *line_end = strstr(hdr, "\r\n");
    if (!line_end) return -1;
    *line_end = 0;

    if (strncmp(hdr, "GET ", 4) == 0) {
        c->method = HTTP_GET;
        hdr += 4;
    } else if (strncmp(hdr, "POST ", 5) == 0) {
        c->method = HTTP_POST;
        hdr += 5;
    } else {
        c->method = HTTP_OTHER;
        return -1;
    }

    // Skip leading slash isn't required; just copy path
    char *space = strchr(hdr, ' ');
    if (!space) return -1;
    *space = 0;

    // Split path and query
    char *qmark = strchr(hdr, '?');
    if (qmark) {
        *qmark = 0;
        snprintf(c->query, sizeof(c->query), "%s", qmark + 1);
    } else {
        c->query[0] = 0;
    }
    snprintf(c->path, sizeof(c->path), "%s", hdr);

    // Find Content-Length
    char *p = line_end + 2;
    c->content_length = 0;
    while (*p) {
        char *next = strstr(p, "\r\n");
        if (!next) break;
        *next = 0;
        if (strncasecmp(p, "Content-Length:", 15) == 0) {
            c->content_length = atoi(p + 15);
        }
        p = next + 2;
    }

    c->headers_done = true;
    return 1;
}

// ---- Output helpers ----
static void send_pending(httpd_conn_t *c)
{
    if (!c->pcb || c->out_len == 0) return;
    err_t e = tcp_write(c->pcb, c->out_buf, c->out_len, TCP_WRITE_FLAG_COPY);
    if (e == ERR_OK) {
        tcp_output(c->pcb);
        c->out_len = 0;
    }
}

void httpd_send_header(httpd_conn_t *c, int status, const char *ctype)
{
    int n = snprintf((char *)c->out_buf, sizeof(c->out_buf),
        "HTTP/1.0 %d %s\r\nContent-Type: %s\r\nConnection: close\r\n\r\n",
        status, status == 200 ? "OK" : "ERR", ctype);
    c->out_len = n;
    send_pending(c);
}

void httpd_send_body(httpd_conn_t *c, const void *data, size_t len)
{
    if (!c->pcb) return;
    // For simple responses we just write straight to TCP
    tcp_write(c->pcb, data, len, TCP_WRITE_FLAG_COPY);
    tcp_output(c->pcb);
}

void httpd_send_text(httpd_conn_t *c, int status, const char *ctype,
                     const char *body)
{
    httpd_send_header(c, status, ctype);
    httpd_send_body(c, body, strlen(body));
}

void httpd_send_json(httpd_conn_t *c, const char *json)
{
    httpd_send_text(c, 200, "application/json", json);
}

void httpd_send_404(httpd_conn_t *c)
{
    httpd_send_text(c, 404, "text/plain", "Not Found");
}

// ---- Streaming response ----
void httpd_start_stream(httpd_conn_t *c, const char *ctype,
                         httpd_stream_fn gen, void *ctx,
                         void (*on_close)(void *))
{
    httpd_send_header(c, 200, ctype);
    c->stream_fn  = gen;
    c->stream_ctx = ctx;
    c->on_close   = on_close;
    c->stream_active = true;
}

static void pump_stream(httpd_conn_t *c)
{
    if (!c->stream_active || !c->pcb || !c->stream_fn) return;

    int avail = tcp_sndbuf(c->pcb);
    if (avail <= 0) return;

    while (avail >= 256 && c->stream_active) {
        uint8_t chunk[1024];
        int max = avail < (int)sizeof(chunk) ? avail : (int)sizeof(chunk);
        int n = c->stream_fn(c->stream_ctx, chunk, max);
        if (n <= 0) {
            c->stream_active = false;
            tcp_output(c->pcb);
            tcp_close(c->pcb);
            free_conn(c);
            return;
        }
        if (tcp_write(c->pcb, chunk, n, TCP_WRITE_FLAG_COPY) != ERR_OK)
            break;
        avail = tcp_sndbuf(c->pcb);
    }
    tcp_output(c->pcb);
}

// ---- TCP callbacks ----
static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)pcb; (void)len;
    httpd_conn_t *c = (httpd_conn_t *)arg;
    if (c->stream_active) pump_stream(c);
    else if (!c->stream_active && c->out_len == 0) {
        // Simple response done - close
        // Closing here is handled by recv -> close after response
    }
    return ERR_OK;
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    httpd_conn_t *c = (httpd_conn_t *)arg;
    (void)err;

    if (!p) {
        // Connection closed by peer
        tcp_close(pcb);
        free_conn(c);
        return ERR_OK;
    }

    // Append to request buffer
    if (c->req_len + p->tot_len < (int)sizeof(c->req_buf)) {
        pbuf_copy_partial(p, c->req_buf + c->req_len, p->tot_len, 0);
        c->req_len += p->tot_len;
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    if (!c->headers_done) {
        int r = parse_request(c);
        if (r < 0) {
            httpd_send_text(c, 400, "text/plain", "Bad Request");
            tcp_close(pcb);
            free_conn(c);
            return ERR_OK;
        }
    }

    if (c->headers_done) {
        // Wait until we have the full body for POSTs
        int needed = c->body_start + c->content_length;
        if (c->req_len < needed) return ERR_OK;

        httpd_handler_fn fn = find_handler(c->path);
        if (!fn) {
            httpd_send_404(c);
            tcp_close(pcb);
            free_conn(c);
            return ERR_OK;
        }

        const uint8_t *body = c->req_buf + c->body_start;
        fn(c, c->method, c->path, c->query, body, c->content_length);

        // If the handler set up streaming, pump it now
        if (c->stream_active) {
            pump_stream(c);
        } else {
            tcp_close(pcb);
            free_conn(c);
        }
    }

    return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
    (void)err;
    httpd_conn_t *c = (httpd_conn_t *)arg;
    if (c) {
        c->pcb = NULL;
        free_conn(c);
    }
}

static err_t on_poll(void *arg, struct tcp_pcb *pcb)
{
    (void)pcb;
    httpd_conn_t *c = (httpd_conn_t *)arg;
    if (c->stream_active) pump_stream(c);
    return ERR_OK;
}

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg; (void)err;

    httpd_conn_t *c = alloc_conn();
    if (!c) {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    c->pcb = pcb;

    tcp_arg (pcb, c);
    tcp_recv(pcb, on_recv);
    tcp_sent(pcb, on_sent);
    tcp_err (pcb, on_err);
    tcp_poll(pcb, on_poll, 4);  // every ~2 s

    return ERR_OK;
}

void httpd_lite_init(uint16_t port)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    tcp_bind(pcb, IP_ADDR_ANY, port);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, on_accept);
    printf("httpd_lite: listening on TCP port %u\n", port);
}

// ---- Query string helper ----
uint32_t httpd_query_get_uint(const char *query, const char *key, uint32_t def)
{
    if (!query || !*query) return def;
    int klen = strlen(key);
    const char *p = query;
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=')
            return (uint32_t)atoi(p + klen + 1);
        const char *amp = strchr(p, '&');
        if (!amp) break;
        p = amp + 1;
    }
    return def;
}
