#include "faa_plate_db.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "board_config.h"
#include "lvgl_v8_port.h"
#include "sd_store.h"

#define FRPL_MAGIC "FRPL"
#define PLATE_DIR SD_MOUNT_POINT "/fr_data/plates"
#define FRPL_MAX_BYTES (512u * 1024u)
#define PLATE_CACHE_SLOTS 4
#define PLATE_NEG_MAX 24
#define PLATE_REQ_QUEUE 8

#pragma pack(push, 1)
typedef struct {
    char magic[4];
    uint16_t w;
    uint16_t h;
    uint16_t cf;
    uint16_t reserved;
    uint32_t data_size;
} FrplHeader;
#pragma pack(pop)

struct PlateSlot {
    lv_img_dsc_t dsc;
    uint8_t *pixels;
    size_t pixels_cap;
    char icao[8];
    uint32_t last_use;
};

static PlateSlot s_slots[PLATE_CACHE_SLOTS] {};
static uint32_t s_use_tick = 1;
static char s_neg[PLATE_NEG_MAX][8] {};
static int s_neg_n = 0;
static char s_req[PLATE_REQ_QUEUE][8] {};
static int s_req_n = 0;
static bool s_ready_checked = false;
static bool s_ready = false;
static SemaphoreHandle_t s_mu = nullptr;

static void plate_mu_init(void) {
    if (!s_mu) {
        s_mu = xSemaphoreCreateMutex();
    }
}

static void plate_lock(void) {
    plate_mu_init();
    if (s_mu) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
}

static void plate_unlock(void) {
    if (s_mu) {
        xSemaphoreGive(s_mu);
    }
}

static void normalize_icao(const char *in, char *out, size_t out_len) {
    if (!out || out_len < 5) {
        return;
    }
    out[0] = 0;
    if (!in || !in[0]) {
        return;
    }
    char tmp[8] = {0};
    size_t j = 0;
    for (size_t i = 0; in[i] && j + 1 < sizeof(tmp); i++) {
        const unsigned char c = (unsigned char)in[i];
        if (isalnum(c)) {
            tmp[j++] = (char)toupper(c);
        }
    }
    tmp[j] = 0;
    if (j == 3) {
        snprintf(out, out_len, "K%s", tmp);
    } else {
        strncpy(out, tmp, out_len - 1);
        out[out_len - 1] = 0;
    }
}

static bool icao_eq(const char *a, const char *b) {
    return a && b && strcasecmp(a, b) == 0;
}

static bool neg_has(const char *icao) {
    for (int i = 0; i < s_neg_n; i++) {
        if (icao_eq(s_neg[i], icao)) {
            return true;
        }
    }
    return false;
}

static void neg_add(const char *icao) {
    if (!icao || !icao[0] || neg_has(icao)) {
        return;
    }
    if (s_neg_n >= PLATE_NEG_MAX) {
        // Drop oldest.
        memmove(s_neg[0], s_neg[1], sizeof(s_neg[0]) * (size_t)(PLATE_NEG_MAX - 1));
        s_neg_n = PLATE_NEG_MAX - 1;
    }
    strncpy(s_neg[s_neg_n], icao, sizeof(s_neg[0]) - 1);
    s_neg[s_neg_n][sizeof(s_neg[0]) - 1] = 0;
    s_neg_n++;
}

static PlateSlot *slot_find(const char *icao) {
    for (int i = 0; i < PLATE_CACHE_SLOTS; i++) {
        if (s_slots[i].pixels && icao_eq(s_slots[i].icao, icao)) {
            return &s_slots[i];
        }
    }
    return nullptr;
}

static PlateSlot *slot_victim(void) {
    PlateSlot *empty = nullptr;
    PlateSlot *lru = &s_slots[0];
    for (int i = 0; i < PLATE_CACHE_SLOTS; i++) {
        PlateSlot *s = &s_slots[i];
        if (!s->pixels) {
            empty = s;
            break;
        }
        if (s->last_use < lru->last_use) {
            lru = s;
        }
    }
    return empty ? empty : lru;
}

static void slot_clear(PlateSlot *s) {
    if (!s) {
        return;
    }
    if (s->pixels) {
        heap_caps_free(s->pixels);
        s->pixels = nullptr;
    }
    s->pixels_cap = 0;
    s->icao[0] = 0;
    s->last_use = 0;
    memset(&s->dsc, 0, sizeof(s->dsc));
}

void faa_plates_invalidate(void) {
    // Swap under LVGL lock so draw cannot hold a freed pointer.
    lvgl_port_lock(-1);
    plate_lock();
    s_ready_checked = false;
    s_ready = false;
    s_neg_n = 0;
    s_req_n = 0;
    memset(s_neg, 0, sizeof(s_neg));
    memset(s_req, 0, sizeof(s_req));
    for (int i = 0; i < PLATE_CACHE_SLOTS; i++) {
        slot_clear(&s_slots[i]);
    }
    plate_unlock();
    lvgl_port_unlock();
}

bool faa_plates_sd_ready(void) {
    plate_lock();
    if (s_ready_checked) {
        const bool r = s_ready;
        plate_unlock();
        return r;
    }
    plate_unlock();

    bool ready = false;
    if (sd_mount()) {
        struct stat st {};
        if (stat(PLATE_DIR, &st) == 0 && S_ISDIR(st.st_mode)) {
            static const char *kProbe[] = {"KATL", "KMCO", "KJAX", "KX60"};
            for (size_t i = 0; i < sizeof(kProbe) / sizeof(kProbe[0]); i++) {
                char path[96];
                snprintf(path, sizeof(path), "%s/%s.frpl", PLATE_DIR, kProbe[i]);
                if (stat(path, &st) == 0 && st.st_size > (off_t)sizeof(FrplHeader)) {
                    ready = true;
                    break;
                }
            }
        }
    }

    plate_lock();
    s_ready_checked = true;
    s_ready = ready;
    plate_unlock();
    return ready;
}

const lv_img_dsc_t *faa_plate_peek(const char *icao) {
    char key[8];
    normalize_icao(icao, key, sizeof(key));
    if (!key[0]) {
        return nullptr;
    }
    plate_lock();
    PlateSlot *s = slot_find(key);
    const lv_img_dsc_t *out = nullptr;
    if (s && s->pixels && s->dsc.data) {
        s->last_use = ++s_use_tick;
        out = &s->dsc;
    }
    plate_unlock();
    return out;
}

void faa_plate_request(const char *icao) {
    char key[8];
    normalize_icao(icao, key, sizeof(key));
    if (!key[0]) {
        return;
    }
    plate_lock();
    if (slot_find(key) || neg_has(key)) {
        plate_unlock();
        return;
    }
    for (int i = 0; i < s_req_n; i++) {
        if (icao_eq(s_req[i], key)) {
            plate_unlock();
            return;
        }
    }
    if (s_req_n < PLATE_REQ_QUEUE) {
        strncpy(s_req[s_req_n], key, sizeof(s_req[0]) - 1);
        s_req[s_req_n][sizeof(s_req[0]) - 1] = 0;
        s_req_n++;
    }
    plate_unlock();
}

const lv_img_dsc_t *faa_plate_lookup(const char *icao) {
    const lv_img_dsc_t *img = faa_plate_peek(icao);
    if (!img) {
        faa_plate_request(icao);
    }
    return img;
}

static bool load_frpl_to_staging(const char *icao, FrplHeader *out_hdr, uint8_t **out_pixels) {
    *out_pixels = nullptr;
    if (!icao || !icao[0] || !out_hdr) {
        return false;
    }
    if (!sd_mount()) {
        return false;
    }
    char path[96];
    snprintf(path, sizeof(path), "%s/%s.frpl", PLATE_DIR, icao);
    sd_lock();
    FILE *f = fopen(path, "rb");
    if (!f) {
        sd_unlock();
        return false;
    }
    FrplHeader hdr {};
    if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) || memcmp(hdr.magic, FRPL_MAGIC, 4) != 0 ||
        hdr.w == 0 || hdr.h == 0 || hdr.data_size == 0 || hdr.data_size > FRPL_MAX_BYTES) {
        fclose(f);
        sd_unlock();
        return false;
    }
    uint8_t *buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_8BIT);
    }
    if (!buf) {
        fclose(f);
        sd_unlock();
        return false;
    }
    if (fread(buf, 1, hdr.data_size, f) != hdr.data_size) {
        heap_caps_free(buf);
        fclose(f);
        sd_unlock();
        return false;
    }
    fclose(f);
    sd_unlock();
    *out_hdr = hdr;
    *out_pixels = buf;
    return true;
}

bool faa_plates_service(void) {
    char key[8] = {0};
    plate_lock();
    if (s_req_n <= 0) {
        plate_unlock();
        return false;
    }
    strncpy(key, s_req[0], sizeof(key) - 1);
    // Shift queue.
    if (s_req_n > 1) {
        memmove(s_req[0], s_req[1], sizeof(s_req[0]) * (size_t)(s_req_n - 1));
    }
    s_req_n--;
    s_req[s_req_n][0] = 0;
    // Already cached while queued?
    if (slot_find(key) || neg_has(key)) {
        plate_unlock();
        return false;
    }
    plate_unlock();

    FrplHeader hdr {};
    uint8_t *pixels = nullptr;
    const bool ok = load_frpl_to_staging(key, &hdr, &pixels);
    if (!ok || !pixels) {
        plate_lock();
        neg_add(key);
        plate_unlock();
        Serial.printf("[plate] miss %s\n", key);
        return false;
    }

    // Install under LVGL lock so draw never sees a freed buffer mid-blit.
    lvgl_port_lock(-1);
    plate_lock();
    PlateSlot *slot = slot_find(key);
    if (!slot) {
        slot = slot_victim();
    }
    if (slot->pixels && slot->pixels != pixels) {
        heap_caps_free(slot->pixels);
    }
    slot->pixels = pixels;
    slot->pixels_cap = hdr.data_size;
    strncpy(slot->icao, key, sizeof(slot->icao) - 1);
    slot->icao[sizeof(slot->icao) - 1] = 0;
    slot->last_use = ++s_use_tick;
    memset(&slot->dsc, 0, sizeof(slot->dsc));
    slot->dsc.header.cf = (lv_img_cf_t)hdr.cf;
    slot->dsc.header.always_zero = 0;
    slot->dsc.header.reserved = 0;
    slot->dsc.header.w = hdr.w;
    slot->dsc.header.h = hdr.h;
    slot->dsc.data_size = hdr.data_size;
    slot->dsc.data = slot->pixels;
    s_ready = true;
    s_ready_checked = true;
    plate_unlock();
    lvgl_port_unlock();

    Serial.printf("[plate] loaded %s (%ux%u %u B)\n", key, (unsigned)hdr.w, (unsigned)hdr.h,
                  (unsigned)hdr.data_size);
    return true;
}
