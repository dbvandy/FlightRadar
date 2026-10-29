#pragma once

// Minimal audio stubs (reconstructed). The CrowPanel I2S/PA driver was never committed;
// these keep the UI + main linking. Replace with the I2S implementation to get sound back.

/** Init codec / I2S. Safe to call more than once. */
void audio_init();
/** 0-100. Stored for the real driver; 0 mutes. */
void audio_set_volume(int volume_pct);
/** Short UI click. */
void audio_click_play();
/** Camera shutter (Screen Shot). */
void audio_shutter_play();
/** Movie-projector gate clicks for duration_ms (timelapse start / email cue). */
void audio_projector_play(int duration_ms);
