// SPDX-License-Identifier: MIT
// httpd_lite: small raw-TCP HTTP/1.0 server for the AES67 SoC.
//
// Replaces lwIP's apps/httpd because we need:
//   - dynamic JSON responses (config/status)
//   - POST handling
//   - chunked binary streaming for the audio preview endpoint
//   - tight control over connection lifecycle
//
// Routes are registered with httpd_lite_route(). Each route handler
// receives the request context and returns either a complete response
// (the simple case) or installs streaming callbacks for ongoing data
// generation (the audio preview case).

#ifndef HTTPD_LITE_H
#define HTTPD_LITE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HTTPD_MAX_PATH    128
#define HTTPD_MAX_QUERY   128
#define HTTPD_MAX_BODY    2048

typedef enum { HTTP_GET, HTTP_POST, HTTP_OTHER } http_method_t;

typedef struct httpd_conn httpd_conn_t;

// Streaming generator: called repeatedly until it returns 0 bytes.
// `buf` is the buffer to fill, `max` its capacity. Return number of
// bytes written, or -1 to terminate the stream.
typedef int (*httpd_stream_fn)(void *ctx, uint8_t *buf, int max);

// Request handler. Receives the parsed request. To produce a simple
// response, call httpd_send_*(). To stream, call httpd_start_stream().
typedef void (*httpd_handler_fn)(httpd_conn_t *c, http_method_t m,
                                  const char *path, const char *query,
                                  const uint8_t *body, size_t body_len);

// Lifecycle
void httpd_lite_init(uint16_t port);

// Routing: an exact path match (no wildcards). Multiple routes possible.
void httpd_lite_route(const char *path, httpd_handler_fn handler);

// Inside a handler:
void httpd_send_header(httpd_conn_t *c, int status, const char *content_type);
void httpd_send_body(httpd_conn_t *c, const void *data, size_t len);
void httpd_send_text(httpd_conn_t *c, int status, const char *content_type,
                      const char *body);
void httpd_send_json(httpd_conn_t *c, const char *json);
void httpd_send_404(httpd_conn_t *c);

// Start a streaming response: sends headers, then calls `gen` repeatedly
// as TCP buffer space frees up. The handler returns immediately after
// calling this.
void httpd_start_stream(httpd_conn_t *c, const char *content_type,
                         httpd_stream_fn gen, void *gen_ctx,
                         void (*on_close)(void *ctx));

// Helper: parse a uint from a query string param. Returns def if not found.
uint32_t httpd_query_get_uint(const char *query, const char *key, uint32_t def);

#endif
