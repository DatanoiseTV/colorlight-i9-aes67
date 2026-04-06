// SPDX-License-Identifier: MIT
// HTTP audio preview stream.
// Endpoint: GET /audio.wav?ch=L,R
//   - Sets the audio_capture channel mask
//   - Returns a WAV header (44 bytes) followed by streaming int16 PCM
//   - Browser plays via <audio src="/audio.wav?...">
#ifndef AUDIO_STREAM_H
#define AUDIO_STREAM_H
void audio_stream_register(void);
#endif
