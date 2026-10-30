#include "audio_click.h"

#include "board_config.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

// Match Elecrow CrowPanel Advanced V1.2 bsp_audio.c (I2S_NUM_1).
static constexpr int kSampleRate = 16000;
// Short mechanical-style click (not a sustained beep).
static constexpr int kClickMs = 9;
static constexpr int kDebounceMs = 18;
static constexpr int kPaSettleMs = 8;
static constexpr int kClickSamples = (kSampleRate * kClickMs) / 1000;
// Old 100% was ~28000; user wants old 20% as new 100% → 0.2×.
static constexpr float kMaxAmp = 5600.0f;

static i2s_chan_handle_t s_tx_chan = nullptr;
static bool s_ready = false;
static bool s_pa_on = false;
static int s_volume = 50;
static uint32_t s_last_play_ms = 0;
static bool s_playing = false;
static bool s_pa_active_high = false;  // Elecrow: active-LOW (CTRL=0 enables)

static void pa_enable(bool on) {
    const int level = s_pa_active_high ? (on ? 1 : 0) : (on ? 0 : 1);
    gpio_set_level((gpio_num_t)AUDIO_GPIO_CTRL, level);
    s_pa_on = on;
}

/** Sharp UI click: impulse + brief noise, exponential decay (no beep). */
static void fill_ui_click(int16_t *buf, int samples, float amp_scale) {
    const float amp = kMaxAmp * amp_scale;
    uint32_t rnd = 0xA5u ^ (uint32_t)micros();
    for (int i = 0; i < samples; i++) {
        const float t = (float)i / (float)kSampleRate;
        // Fast exponential decay (~3 ms time constant).
        const float env = expf(-t * 320.0f);
        // Tiny bipolar impulse on first few samples.
        float impulse = 0.0f;
        if (i == 0) {
            impulse = 1.0f;
        } else if (i == 1) {
            impulse = -0.55f;
        } else if (i == 2) {
            impulse = 0.22f;
        }
        // Cheap noise (xorshift).
        rnd ^= rnd << 13;
        rnd ^= rnd >> 17;
        rnd ^= rnd << 5;
        const float noise = ((float)((int32_t)(rnd & 0xFFFFu) - 32768) / 32768.0f);
        // Mostly transient + a little high-passed noise; no sustained sine.
        const float s = amp * env * (0.85f * impulse + 0.35f * noise);
        const int16_t v = (int16_t)s;
        buf[i * 2] = v;
        buf[i * 2 + 1] = v;
    }
}

/** Quiet helper tone for shutter/projector (still short). */
static void fill_soft_tick(int16_t *buf, int samples, float hz, float amp_scale) {
    const float amp = kMaxAmp * amp_scale * 0.7f;
    const float two_pi_f = 2.0f * 3.14159265f * hz / (float)kSampleRate;
    for (int i = 0; i < samples; i++) {
        const float t = (float)i / (float)kSampleRate;
        const float env = expf(-t * 280.0f);
        const int16_t s = (int16_t)(amp * env * sinf(two_pi_f * (float)i));
        buf[i * 2] = s;
        buf[i * 2 + 1] = s;
    }
}

static bool audio_write_stereo(const int16_t *buf, size_t byte_len) {
    if (!s_ready || !s_tx_chan || !buf || byte_len == 0) {
        return false;
    }
    if (!s_pa_on) {
        pa_enable(true);
        delay(kPaSettleMs);
    }

    size_t offset = 0;
    while (offset < byte_len) {
        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_tx_chan, (const uint8_t *)buf + offset, byte_len - offset,
                                          &written, pdMS_TO_TICKS(200));
        if (err != ESP_OK || written == 0) {
            return false;
        }
        offset += written;
    }
    return true;
}

void audio_init() {
    const gpio_config_t pa_cfg = {
        .pin_bit_mask = 1ULL << AUDIO_GPIO_CTRL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&pa_cfg);
    if (err != ESP_OK) {
        Serial.printf("audio: PA gpio_config failed: %s\n", esp_err_to_name(err));
        s_ready = false;
        return;
    }
    s_pa_active_high = false;
    pa_enable(true);
    delay(5);

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 256,
        .auto_clear = true,
        .intr_priority = 0,
    };
    err = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (err != ESP_OK) {
        Serial.printf("audio: i2s_new_channel failed: %s\n", esp_err_to_name(err));
        s_ready = false;
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg =
            {
                .sample_rate_hz = (uint32_t)kSampleRate,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            },
        .slot_cfg =
            {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_STEREO,
                .slot_mask = I2S_STD_SLOT_BOTH,
                .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
                .ws_pol = false,
                .bit_shift = true,
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false,
            },
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)AUDIO_GPIO_BCLK,
                .ws = (gpio_num_t)AUDIO_GPIO_LRCLK,
                .dout = (gpio_num_t)AUDIO_GPIO_SDATA,
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };

    err = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("audio: i2s_channel_init_std_mode failed: %s\n", esp_err_to_name(err));
        s_ready = false;
        return;
    }

    err = i2s_channel_enable(s_tx_chan);
    if (err != ESP_OK) {
        Serial.printf("audio: i2s_channel_enable failed: %s\n", esp_err_to_name(err));
        s_ready = false;
        return;
    }

    s_ready = true;
    Serial.printf("audio: ready (vol=%d, click amp capped ~old 20%%)\n", s_volume);
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

int audio_get_volume() {
    return s_volume;
}

void audio_click_play() {
    if (!s_ready || !s_tx_chan || s_volume <= 0 || s_playing) {
        return;
    }
    const uint32_t now = millis();
    if (now - s_last_play_ms < (uint32_t)kDebounceMs) {
        return;
    }
    s_playing = true;
    s_last_play_ms = now;

    int16_t buf[kClickSamples * 2];
    fill_ui_click(buf, kClickSamples, (float)s_volume / 100.0f);
    audio_write_stereo(buf, sizeof(buf));
    s_playing = false;
}

void audio_shutter_play() {
    if (!s_ready || s_volume <= 0 || s_playing) {
        return;
    }
    s_playing = true;
    s_last_play_ms = millis();

    constexpr int kBurst = (kSampleRate * 14) / 1000;
    constexpr int kGap = (kSampleRate * 12) / 1000;
    constexpr int kTotal = kBurst * 2 + kGap;
    int16_t buf[kTotal * 2];
    memset(buf, 0, sizeof(buf));
    fill_ui_click(buf, kBurst, (float)s_volume / 100.0f);
    fill_soft_tick(buf + (kBurst + kGap) * 2, kBurst, 900.0f, (float)s_volume / 100.0f * 0.6f);
    audio_write_stereo(buf, sizeof(buf));
    s_playing = false;
}

void audio_projector_play(int duration_ms) {
    if (!s_ready || s_volume <= 0 || s_playing) {
        return;
    }
    if (duration_ms < 40) {
        duration_ms = 40;
    }
    if (duration_ms > 3000) {
        duration_ms = 3000;
    }
    s_playing = true;
    s_last_play_ms = millis();

    constexpr int kGateMs = 8;
    constexpr int kGateSamples = (kSampleRate * kGateMs) / 1000;
    int16_t gate[kGateSamples * 2];
    fill_ui_click(gate, kGateSamples, (float)s_volume / 100.0f * 0.45f);

    const uint32_t end_ms = millis() + (uint32_t)duration_ms;
    while ((int32_t)(end_ms - millis()) > 0) {
        audio_write_stereo(gate, sizeof(gate));
        delay(55);
    }
    s_playing = false;
}
