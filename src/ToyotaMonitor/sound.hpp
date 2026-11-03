#pragma once

// Speaker output: ES8311 codec + NS4150 amp on the Guition JC4880P443C.
// Sounds are 16 kHz mono 16-bit PCM compiled into flash (src/sounds).
// Also included by the generated C files, so keep it plain C.

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const int16_t *pcm;
    size_t samples;
} Sound;

extern const Sound snd_reactor_online;
extern const Sound snd_sensors_online;
extern const Sound snd_all_systems_nominal;

#ifdef __cplusplus
}

namespace sound {

static const uint32_t SAMPLE_RATE = 16000;
static const uint32_t GAP_MS = 250;   // silence between queued sounds

inline uint32_t duration_ms(const Sound &s) { return s.samples * 1000 / SAMPLE_RATE; }

// Sets up the codec on the already created I2C bus (I2C_NUM_1) and the I2S output
bool begin();

// Queues sounds to play one after another in the background; nullptr ends the list
void play(const Sound *const *sequence);

}  // namespace sound
#endif
