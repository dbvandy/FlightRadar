#pragma once

/** Init I2S TX + PA pin. Call once after board begin. */
void audio_init();

/** Volume 0–100 (0 = silent, no I2S write). */
void audio_set_volume(int volume_pct);
int audio_get_volume();

/** Short UI click (~18 ms). Safe if volume==0 or not inited. */
void audio_click_play();

/** Camera shutter cue (synthesized double chirp). */
void audio_shutter_play();

/** Movie-projector gate clicks for duration_ms. */
void audio_projector_play(int duration_ms);
