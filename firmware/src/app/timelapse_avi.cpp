#include "timelapse_avi.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_jpeg_enc.h"

/** Per-frame metadata only — JPEG bytes live on SD. */
struct TlMeta {
    uint16_t id;   // tl_%03u.jpg
    uint32_t len;  // JPEG byte length
};

static TlMeta s_frames[FR_TL_MAX_FRAMES];
static int s_n = 0;
static uint16_t s_next_id = 0;
static char s_dir[160] = {0};

static void tl_frame_path(char *out, size_t out_len, uint16_t id) {
    if (!out || out_len == 0) {
        return;
    }
    if (!s_dir[0]) {
        out[0] = 0;
        return;
    }
    snprintf(out, out_len, "%s/tl_%03u.jpg", s_dir, (unsigned)id);
}

static void tl_unlink_id(uint16_t id) {
    char path[192];
    tl_frame_path(path, sizeof(path), id);
    if (path[0]) {
        unlink(path);
    }
}

void tl_set_storage_dir(const char *dir) {
    if (!dir || !dir[0]) {
        s_dir[0] = 0;
        return;
    }
    strncpy(s_dir, dir, sizeof(s_dir) - 1);
    s_dir[sizeof(s_dir) - 1] = 0;
}

void tl_clear(void) {
    for (int i = 0; i < s_n; i++) {
        tl_unlink_id(s_frames[i].id);
        s_frames[i].id = 0;
        s_frames[i].len = 0;
    }
    s_n = 0;
    s_next_id = 0;
}

int tl_frame_count(void) {
    return s_n;
}

static void put_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/** Box-filter downscale (2×2 / 3×3 average) — nearest-neighbor looked awful. */
static void downscale_rgb565(const uint16_t *src, int sw, int sh, uint16_t *dst, int dw, int dh) {
    for (int y = 0; y < dh; y++) {
        const int y0 = (y * sh) / dh;
        const int y1 = ((y + 1) * sh) / dh;
        const int yb = (y1 > y0 + 1) ? y1 : (y0 + 1);
        for (int x = 0; x < dw; x++) {
            const int x0 = (x * sw) / dw;
            const int x1 = ((x + 1) * sw) / dw;
            const int xb = (x1 > x0 + 1) ? x1 : (x0 + 1);
            uint32_t rsum = 0, gsum = 0, bsum = 0, n = 0;
            for (int sy = y0; sy < yb && sy < sh; sy++) {
                for (int sx = x0; sx < xb && sx < sw; sx++) {
                    const uint16_t p = src[sy * sw + sx];
                    rsum += (p >> 11) & 0x1F;
                    gsum += (p >> 5) & 0x3F;
                    bsum += p & 0x1F;
                    n++;
                }
            }
            if (n == 0) {
                dst[y * dw + x] = src[y0 * sw + x0];
            } else {
                const uint16_t r = (uint16_t)(rsum / n);
                const uint16_t g = (uint16_t)(gsum / n);
                const uint16_t b = (uint16_t)(bsum / n);
                dst[y * dw + x] = (uint16_t)((r << 11) | (g << 5) | b);
            }
        }
    }
}

static void tl_drop_oldest(void) {
    if (s_n <= 0) {
        return;
    }
    tl_unlink_id(s_frames[0].id);
    memmove(&s_frames[0], &s_frames[1], sizeof(TlMeta) * (size_t)(s_n - 1));
    s_n--;
    s_frames[s_n].id = 0;
    s_frames[s_n].len = 0;
}

static bool tl_write_jpeg_file(const char *path, const uint8_t *data, size_t len) {
    if (!path || !path[0] || !data || len == 0) {
        return false;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        Serial.printf("[tl] jpeg fopen failed %s\n", path);
        return false;
    }
    const size_t n = fwrite(data, 1, len, f);
    const int rc = fclose(f);
    return n == len && rc == 0;
}

bool tl_add_rgb565_frame(const uint16_t *snap_rgb565, int snap_w, int snap_h) {
    if (!snap_rgb565 || snap_w < 8 || snap_h < 8) {
        return false;
    }
    if (!s_dir[0]) {
        Serial.println("[tl] no storage dir");
        return false;
    }
    if (s_n >= FR_TL_MAX_FRAMES) {
        tl_drop_oldest();
    }

    const size_t rgb_bytes = (size_t)FR_TL_W * (size_t)FR_TL_H * 2u;
    uint8_t *rgb = (uint8_t *)jpeg_calloc_align(rgb_bytes, 16);
    if (!rgb) {
        Serial.println("[tl] rgb alloc failed");
        return false;
    }
    downscale_rgb565(snap_rgb565, snap_w, snap_h, (uint16_t *)rgb, FR_TL_W, FR_TL_H);

    jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
    cfg.width = FR_TL_W;
    cfg.height = FR_TL_H;
    cfg.src_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    cfg.subsampling = JPEG_SUBSAMPLE_420;
    cfg.quality = 85;
    cfg.rotate = JPEG_ROTATE_0D;
    cfg.task_enable = false;

    jpeg_enc_handle_t enc = nullptr;
    if (jpeg_enc_open(&cfg, &enc) != JPEG_ERR_OK || !enc) {
        jpeg_free_align(rgb);
        return false;
    }

    // Headroom for 200x200 @ q85 — freed after SD write.
    const int out_cap = 160 * 1024;
    uint8_t *jpg = (uint8_t *)heap_caps_malloc((size_t)out_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpg) {
        jpg = (uint8_t *)heap_caps_malloc((size_t)out_cap, MALLOC_CAP_8BIT);
    }
    if (!jpg) {
        jpeg_enc_close(enc);
        jpeg_free_align(rgb);
        Serial.println("[tl] jpg alloc failed");
        return false;
    }

    int out_size = 0;
    const jpeg_error_t jerr =
        jpeg_enc_process(enc, rgb, (int)rgb_bytes, jpg, out_cap, &out_size);
    jpeg_enc_close(enc);
    jpeg_free_align(rgb);
    if (jerr != JPEG_ERR_OK || out_size <= 0) {
        heap_caps_free(jpg);
        return false;
    }

    const uint16_t id = s_next_id;
    char path[192];
    tl_frame_path(path, sizeof(path), id);
    const bool wrote = tl_write_jpeg_file(path, jpg, (size_t)out_size);
    heap_caps_free(jpg);
    if (!wrote) {
        return false;
    }

    s_frames[s_n].id = id;
    s_frames[s_n].len = (uint32_t)out_size;
    s_n++;
    s_next_id = (uint16_t)(id + 1u);
    return true;
}

static bool tl_fwrite(FILE *f, const void *data, size_t len) {
    return f && data && len > 0 && fwrite(data, 1, len, f) == len;
}

static bool tl_fwrite_le32(FILE *f, uint32_t v) {
    uint8_t b[4];
    put_le32(b, v);
    return tl_fwrite(f, b, 4);
}

static bool tl_fwrite_le16(FILE *f, uint16_t v) {
    uint8_t b[2];
    put_le16(b, v);
    return tl_fwrite(f, b, 2);
}

/** Read JPEG from SD and append as one AVI 00dc chunk; frees buffer after write. */
static bool tl_copy_jpeg_to_avi(FILE *avi, uint16_t id, uint32_t len) {
    char path[192];
    tl_frame_path(path, sizeof(path), id);
    if (!path[0] || len == 0) {
        return false;
    }
    FILE *jf = fopen(path, "rb");
    if (!jf) {
        Serial.printf("[tl] jpeg read open failed %s\n", path);
        return false;
    }
    uint8_t *buf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_8BIT);
    }
    if (!buf) {
        fclose(jf);
        return false;
    }
    const size_t got = fread(buf, 1, len, jf);
    fclose(jf);
    bool ok = (got == len) && tl_fwrite(avi, buf, len);
    heap_caps_free(buf);
    return ok;
}

/** Stream MJPEG AVI to SD — one JPEG in RAM at a time (avoids PSRAM frame ring / OOM). */
bool tl_build_avi_to_file(const char *path, int fps, size_t *out_len) {
    if (!path || !path[0] || s_n <= 0 || !s_dir[0]) {
        return false;
    }
    if (out_len) {
        *out_len = 0;
    }
    if (fps < 1) {
        fps = 1;
    }
    if (fps > 30) {
        fps = 30;
    }

    const int n = s_n;
    uint32_t *lens = (uint32_t *)malloc((size_t)n * sizeof(uint32_t));
    uint32_t *idx_off = (uint32_t *)malloc((size_t)n * sizeof(uint32_t));
    uint16_t *ids = (uint16_t *)malloc((size_t)n * sizeof(uint16_t));
    if (!lens || !idx_off || !ids) {
        free(lens);
        free(idx_off);
        free(ids);
        return false;
    }
    size_t movi_payload = 0;
    for (int i = 0; i < n; i++) {
        lens[i] = s_frames[i].len;
        ids[i] = s_frames[i].id;
        const uint32_t pad = (lens[i] & 1u) ? 1u : 0u;
        movi_payload += 8u + lens[i] + pad;
    }

    const size_t strl_size = 4 + (8 + 56) + (8 + 40);
    const size_t hdrl_size = 4 + (8 + 56) + (8 + strl_size);
    const size_t hdrl_list = 8 + hdrl_size;
    const size_t movi_list = 8 + 4 + movi_payload;
    const size_t idx1_chunk = 8 + (size_t)n * 16u;
    const size_t riff_payload = 4 + hdrl_list + movi_list + idx1_chunk;

    FILE *f = fopen(path, "wb");
    if (!f) {
        free(lens);
        free(idx_off);
        free(ids);
        Serial.printf("[tl] AVI fopen failed %s\n", path);
        return false;
    }

    bool ok = true;
    ok = ok && tl_fwrite(f, "RIFF", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)riff_payload);
    ok = ok && tl_fwrite(f, "AVI ", 4);

    ok = ok && tl_fwrite(f, "LIST", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)hdrl_size);
    ok = ok && tl_fwrite(f, "hdrl", 4);

    ok = ok && tl_fwrite(f, "avih", 4);
    ok = ok && tl_fwrite_le32(f, 56);
    ok = ok && tl_fwrite_le32(f, (uint32_t)(1000000 / fps));
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0x10);
    ok = ok && tl_fwrite_le32(f, (uint32_t)n);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 1);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, (uint32_t)FR_TL_W);
    ok = ok && tl_fwrite_le32(f, (uint32_t)FR_TL_H);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);

    ok = ok && tl_fwrite(f, "LIST", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)strl_size);
    ok = ok && tl_fwrite(f, "strl", 4);

    ok = ok && tl_fwrite(f, "strh", 4);
    ok = ok && tl_fwrite_le32(f, 56);
    ok = ok && tl_fwrite(f, "vids", 4);
    ok = ok && tl_fwrite(f, "MJPG", 4);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le16(f, 0);
    ok = ok && tl_fwrite_le16(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 1);
    ok = ok && tl_fwrite_le32(f, (uint32_t)fps);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, (uint32_t)n);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 10000);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le16(f, 0);
    ok = ok && tl_fwrite_le16(f, 0);
    ok = ok && tl_fwrite_le16(f, (uint16_t)FR_TL_W);
    ok = ok && tl_fwrite_le16(f, (uint16_t)FR_TL_H);

    ok = ok && tl_fwrite(f, "strf", 4);
    ok = ok && tl_fwrite_le32(f, 40);
    ok = ok && tl_fwrite_le32(f, 40);
    ok = ok && tl_fwrite_le32(f, (uint32_t)FR_TL_W);
    ok = ok && tl_fwrite_le32(f, (uint32_t)FR_TL_H);
    ok = ok && tl_fwrite_le16(f, 1);
    ok = ok && tl_fwrite_le16(f, 24);
    ok = ok && tl_fwrite(f, "MJPG", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)FR_TL_W * (uint32_t)FR_TL_H * 3u);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);
    ok = ok && tl_fwrite_le32(f, 0);

    ok = ok && tl_fwrite(f, "LIST", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)(4 + movi_payload));
    ok = ok && tl_fwrite(f, "movi", 4);

    // Offsets relative to start of 'movi' fourcc (already written).
    uint32_t movi_rel = 4;
    for (int i = 0; i < n && ok; i++) {
        if ((i & 3) == 3) {
            vTaskDelay(1);
        }
        idx_off[i] = movi_rel;
        ok = ok && tl_fwrite(f, "00dc", 4);
        ok = ok && tl_fwrite_le32(f, lens[i]);
        ok = ok && tl_copy_jpeg_to_avi(f, ids[i], lens[i]);
        if (ok && (lens[i] & 1u)) {
            const uint8_t z = 0;
            ok = tl_fwrite(f, &z, 1);
        }
        movi_rel += 8u + lens[i] + ((lens[i] & 1u) ? 1u : 0u);
    }

    ok = ok && tl_fwrite(f, "idx1", 4);
    ok = ok && tl_fwrite_le32(f, (uint32_t)n * 16u);
    for (int i = 0; i < n && ok; i++) {
        ok = ok && tl_fwrite(f, "00dc", 4);
        ok = ok && tl_fwrite_le32(f, 0x10);
        ok = ok && tl_fwrite_le32(f, idx_off[i]);
        ok = ok && tl_fwrite_le32(f, lens[i]);
    }

    const long end_pos = ftell(f);
    fclose(f);
    free(lens);
    free(idx_off);
    free(ids);

    if (!ok || end_pos <= 0) {
        Serial.println("[tl] AVI write failed");
        remove(path);
        tl_clear();
        return false;
    }

    // Drop temp JPEGs after successful mux.
    tl_clear();

    Serial.printf("[tl] AVI ok %d frames %ld bytes → %s\n", n, end_pos, path);
    if (out_len) {
        *out_len = (size_t)end_pos;
    }
    return true;
}
