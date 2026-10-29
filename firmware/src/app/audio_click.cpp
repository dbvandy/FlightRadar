#include "audio_click.h"

#include <Arduino.h>

// Reconstructed no-op audio layer: tracks volume only. No I2S output yet.
static int s_volume = 60;
static bool s_inited = false;

void audio_init() {
    s_inited = true;
}

void audio_set_volume(int volume_pct) {
    if (volume_pct < 0) {
        volume_pct = 0;
    }
    if (volume_pct > 100) {
        volume_pct = 100;
    }
    s_volume = volume_pct;
}

void audio_click_play() {
    (void)s_inited;
    (void)s_volume;
}

void audio_shutter_play() {
    (void)s_volume;
}

void audio_projector_play(int duration_ms) {
    (void)duration_ms;
    (void)s_volume;
}
