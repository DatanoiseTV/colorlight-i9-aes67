// SPDX-License-Identifier: MIT
// Audio preview streaming endpoint - serves /audio.wav as a chunked
// stream of int16 PCM samples from the hardware audio_capture FIFO.
//
// The browser plays it via a regular <audio> element. We send a fake
// WAV header (huge nominal length) so the browser doesn't try to seek.
// Sample rate and channel count are reported in the header.

#include "audio_stream.h"
#include "httpd_lite.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

#define PREVIEW_SAMPLE_RATE 48000
#define PREVIEW_CHANNELS    2
#define PREVIEW_BPS         16

typedef struct {
    int   ch_left;
    int   ch_right;
    int   started;
} stream_state_t;

static stream_state_t s_state;
static int            s_active;

// ---- WAV header (44 bytes, fake huge length) ----
static int build_wav_header(uint8_t *buf, int max)
{
    if (max < 44) return 0;
    const uint32_t fake_len = 0xFFFFFFFFu;
    const uint32_t byte_rate = PREVIEW_SAMPLE_RATE * PREVIEW_CHANNELS * PREVIEW_BPS / 8;
    const uint16_t block_align = PREVIEW_CHANNELS * PREVIEW_BPS / 8;

    memcpy(buf,      "RIFF", 4);
    buf[4]  = (fake_len >>  0) & 0xFF;
    buf[5]  = (fake_len >>  8) & 0xFF;
    buf[6]  = (fake_len >> 16) & 0xFF;
    buf[7]  = (fake_len >> 24) & 0xFF;
    memcpy(buf + 8,  "WAVE", 4);
    memcpy(buf + 12, "fmt ", 4);
    buf[16] = 16; buf[17] = 0; buf[18] = 0; buf[19] = 0;     // fmt chunk size
    buf[20] = 1;  buf[21] = 0;                                // PCM
    buf[22] = PREVIEW_CHANNELS; buf[23] = 0;
    buf[24] = (PREVIEW_SAMPLE_RATE >>  0) & 0xFF;
    buf[25] = (PREVIEW_SAMPLE_RATE >>  8) & 0xFF;
    buf[26] = (PREVIEW_SAMPLE_RATE >> 16) & 0xFF;
    buf[27] = (PREVIEW_SAMPLE_RATE >> 24) & 0xFF;
    buf[28] = (byte_rate >>  0) & 0xFF;
    buf[29] = (byte_rate >>  8) & 0xFF;
    buf[30] = (byte_rate >> 16) & 0xFF;
    buf[31] = (byte_rate >> 24) & 0xFF;
    buf[32] = block_align; buf[33] = 0;
    buf[34] = PREVIEW_BPS; buf[35] = 0;
    memcpy(buf + 36, "data", 4);
    buf[40] = (fake_len >>  0) & 0xFF;
    buf[41] = (fake_len >>  8) & 0xFF;
    buf[42] = (fake_len >> 16) & 0xFF;
    buf[43] = (fake_len >> 24) & 0xFF;
    return 44;
}

// ---- Stream generator: pulled by httpd_lite as TCP buffer drains ----
//
// Each entry from the capture FIFO is {4-bit channel, 16-bit sample}.
// We need to assemble interleaved L/R pairs in the order the WAV format
// expects. Strategy: maintain a one-sample latch for left and right;
// emit a stereo frame when both have been seen since the last emit.
static int16_t s_left, s_right;
static int     s_have_left, s_have_right;

static int audio_stream_gen(void *ctx, uint8_t *buf, int max)
{
    stream_state_t *st = (stream_state_t *)ctx;
    int written = 0;

    if (!st->started) {
        int n = build_wav_header(buf, max);
        st->started = 1;
        return n;
    }

    while (written + 4 <= max) {
        if (aes67_audio_cap_empty_read())
            break;

        aes67_audio_cap_rd_write(1);
        uint32_t entry = aes67_audio_cap_rd_data_read();
        uint32_t ch    = (entry >> 16) & 0xF;
        int16_t  samp  = (int16_t)(entry & 0xFFFF);

        if ((int)ch == st->ch_left) {
            s_left = samp;
            s_have_left = 1;
        } else if ((int)ch == st->ch_right) {
            s_right = samp;
            s_have_right = 1;
        }

        if (s_have_left && s_have_right) {
            buf[written++] = s_left & 0xFF;
            buf[written++] = (s_left >> 8) & 0xFF;
            buf[written++] = s_right & 0xFF;
            buf[written++] = (s_right >> 8) & 0xFF;
            s_have_left = s_have_right = 0;
        }
    }

    return written;
}

static void on_close(void *ctx)
{
    (void)ctx;
    aes67_audio_cap_enable_write(0);
    aes67_audio_cap_chan_mask_write(0);
    s_active = 0;
}

static void audio_stream_handler(httpd_conn_t *c, http_method_t m,
                                  const char *path, const char *query,
                                  const uint8_t *body, size_t body_len)
{
    (void)m; (void)path; (void)body; (void)body_len;

    int left  = (int)httpd_query_get_uint(query, "l", 0);
    int right = (int)httpd_query_get_uint(query, "r", 1);
    if (left  > 15) left  = 15;
    if (right > 15) right = 15;

    s_state.ch_left  = left;
    s_state.ch_right = right;
    s_state.started  = 0;
    s_have_left = s_have_right = 0;

    // Configure the capture mask: only the two requested channels
    uint16_t mask = (1u << left) | (1u << right);
    aes67_audio_cap_chan_mask_write(mask);
    aes67_audio_cap_enable_write(1);
    s_active = 1;

    httpd_start_stream(c, "audio/wav", audio_stream_gen, &s_state, on_close);
}

void audio_stream_register(void)
{
    httpd_lite_route("/audio.wav", audio_stream_handler);
}
