#include "airport_plate_db.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "board_config.h"
#include "lvgl_v8_port.h"
#include "net_client.h"

#define FRPL_MAGIC "FRPL"
#define FRPL_MAX_BYTES (512u * 1024u)
// ~16 × 153 KB ≈ 2.4 MB PSRAM — covers the current GitHub pack with room to grow.
#ifndef PLATE_CACHE_SLOTS
#define PLATE_CACHE_SLOTS 16
#endif
#define PLATE_NEG_MAX 48
#define PLATE_REQ_QUEUE 48  // enough for a full GitHub pack refresh

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

static PlateSlot s_slots[PLATE_CACHE_SLOTS]{};
static uint32_t s_use_tick = 1;
static char s_neg[PLATE_NEG_MAX][8]{};
static int s_neg_n = 0;
static char s_req[PLATE_REQ_QUEUE][8]{};
static int s_req_n = 0;
static bool s_ready = false;
static bool s_bulk_update = false;
static bool s_fs_ready = false;
static SemaphoreHandle_t s_mu = nullptr;

/** Flat SPIFFS path — survives reboot; only written by Update Plates. */
static void plate_fs_path(const char *icao, char *out, size_t out_len) {
    snprintf(out, out_len, "/p_%s.frpl", icao);
}

static bool plate_fs_init(void) {
    if (s_fs_ready) {
        return true;
    }
    // Partition label "spiffs" in default_16MB.csv (~3.4 MB).
    if (!SPIFFS.begin(true, "/spiffs", 4, "spiffs")) {
        Serial.println("[plate] SPIFFS mount failed");
        return false;
    }
    s_fs_ready = true;
    Serial.println("[plate] SPIFFS ready");
    return true;
}

static void plate_fs_wipe_all(void) {
    if (!plate_fs_init()) {
        return;
    }
    File root = SPIFFS.open("/");
    if (!root || !root.isDirectory()) {
        return;
    }
    File f = root.openNextFile();
    while (f) {
        const char *name = f.name();
        char path[48];
        // SPIFFS may return "/p_KOCF.frpl" or "p_KOCF.frpl".
        if (name && strstr(name, ".frpl")) {
            if (name[0] == '/') {
                strncpy(path, name, sizeof(path) - 1);
            } else {
                snprintf(path, sizeof(path), "/%s", name);
            }
            path[sizeof(path) - 1] = 0;
            f.close();
            SPIFFS.remove(path);
        } else {
            f.close();
        }
        f = root.openNextFile();
    }
}

static bool plate_fs_save(const char *icao, const FrplHeader *hdr, const uint8_t *pixels) {
    if (!icao || !hdr || !pixels || !plate_fs_init()) {
        return false;
    }
    char path[32];
    plate_fs_path(icao, path, sizeof(path));
    SPIFFS.remove(path);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) {
        // Free one other plate file and retry once (partition ~3.4 MB).
        File root = SPIFFS.open("/");
        File o = root ? root.openNextFile() : File();
        while (o) {
            const char *name = o.name();
            if (name && strstr(name, ".frpl") && !strstr(name, icao)) {
                char doomed[48];
                if (name[0] == '/') {
                    strncpy(doomed, name, sizeof(doomed) - 1);
                } else {
                    snprintf(doomed, sizeof(doomed), "/%s", name);
                }
                doomed[sizeof(doomed) - 1] = 0;
                o.close();
                SPIFFS.remove(doomed);
                break;
            }
            o.close();
            o = root.openNextFile();
        }
        f = SPIFFS.open(path, FILE_WRITE);
        if (!f) {
            Serial.printf("[plate] SPIFFS write open fail %s\n", icao);
            return false;
        }
    }
    const size_t hdr_n = sizeof(FrplHeader);
    if (f.write((const uint8_t *)hdr, hdr_n) != hdr_n ||
        f.write(pixels, hdr->data_size) != hdr->data_size) {
        f.close();
        SPIFFS.remove(path);
        Serial.printf("[plate] SPIFFS write fail %s\n", icao);
        return false;
    }
    f.close();
    Serial.printf("[plate] SPIFFS saved %s (%u B)\n", icao, (unsigned)(hdr_n + hdr->data_size));
    return true;
}

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
    lvgl_port_lock(-1);
    plate_lock();
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
    // True once any plate is in PSRAM, or SPIFFS has been mounted with files.
    return s_ready || s_fs_ready;
}

/** Install decoded FRPL pixels into the PSRAM LRU (caller owns pixels on failure).
 *  Does NOT take the LVGL lock — callers already hold it (UI) or are on net task
 *  and must invalidate via ui_notify_plate_bitmap after. */
static bool slot_install(const char *key, const FrplHeader *hdr, uint8_t *pixels) {
    if (!key || !hdr || !pixels) {
        return false;
    }
    plate_lock();
    PlateSlot *slot = slot_find(key);
    if (!slot) {
        slot = slot_victim();
    }
    if (slot->pixels && slot->pixels != pixels) {
        heap_caps_free(slot->pixels);
    }
    slot->pixels = pixels;
    slot->pixels_cap = hdr->data_size;
    strncpy(slot->icao, key, sizeof(slot->icao) - 1);
    slot->icao[sizeof(slot->icao) - 1] = 0;
    slot->last_use = ++s_use_tick;
    memset(&slot->dsc, 0, sizeof(slot->dsc));
    slot->dsc.header.cf = (lv_img_cf_t)hdr->cf;
    slot->dsc.header.always_zero = 0;
    slot->dsc.header.reserved = 0;
    slot->dsc.header.w = hdr->w;
    slot->dsc.header.h = hdr->h;
    slot->dsc.data_size = hdr->data_size;
    slot->dsc.data = slot->pixels;
    s_ready = true;
    plate_unlock();
    return true;
}

/** Load FRPL from SPIFFS into PSRAM — no network. */
static bool load_frpl_fs(const char *icao, FrplHeader *out_hdr, uint8_t **out_pixels) {
    *out_pixels = nullptr;
    if (!icao || !icao[0] || !out_hdr || !plate_fs_init()) {
        return false;
    }
    char path[32];
    plate_fs_path(icao, path, sizeof(path));
    if (!SPIFFS.exists(path)) {
        return false;
    }
    File f = SPIFFS.open(path, FILE_READ);
    if (!f) {
        return false;
    }
    const size_t flen = f.size();
    if (flen < sizeof(FrplHeader)) {
        f.close();
        return false;
    }
    FrplHeader hdr{};
    if (f.read((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) {
        f.close();
        return false;
    }
    if (memcmp(hdr.magic, FRPL_MAGIC, 4) != 0 || hdr.w == 0 || hdr.h == 0 || hdr.data_size == 0 ||
        hdr.data_size > FRPL_MAX_BYTES || flen < sizeof(FrplHeader) + (size_t)hdr.data_size) {
        f.close();
        return false;
    }
    uint8_t *buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_8BIT);
    }
    if (!buf) {
        f.close();
        return false;
    }
    if (f.read(buf, hdr.data_size) != hdr.data_size) {
        heap_caps_free(buf);
        f.close();
        return false;
    }
    f.close();
    *out_hdr = hdr;
    *out_pixels = buf;
    return true;
}

bool faa_plates_pending(void) {
    plate_lock();
    const bool pending = s_req_n > 0;
    plate_unlock();
    return pending;
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

void faa_plate_forget(const char *icao) {
    char key[8];
    normalize_icao(icao, key, sizeof(key));
    if (!key[0]) {
        return;
    }
    lvgl_port_lock(-1);
    plate_lock();
    PlateSlot *s = slot_find(key);
    if (s) {
        slot_clear(s);
    }
    for (int i = 0; i < s_neg_n; i++) {
        if (icao_eq(s_neg[i], key)) {
            if (i < s_neg_n - 1) {
                memmove(s_neg[i], s_neg[i + 1], sizeof(s_neg[0]) * (size_t)(s_neg_n - 1 - i));
            }
            s_neg_n--;
            s_neg[s_neg_n][0] = 0;
            break;
        }
    }
    // Drop any pending queue entry so a fresh request can be enqueued.
    for (int i = 0; i < s_req_n; i++) {
        if (icao_eq(s_req[i], key)) {
            if (i < s_req_n - 1) {
                memmove(s_req[i], s_req[i + 1], sizeof(s_req[0]) * (size_t)(s_req_n - 1 - i));
            }
            s_req_n--;
            s_req[s_req_n][0] = 0;
            break;
        }
    }
    plate_unlock();
    lvgl_port_unlock();
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

/** Clear PSRAM + SPIFFS, then queue every ICAO for download (Update Plates only). */
void faa_plates_queue_refresh(const char icaos[][8], int n) {
    if (!icaos || n <= 0) {
        return;
    }
    plate_fs_wipe_all();
    lvgl_port_lock(-1);
    plate_lock();
    s_neg_n = 0;
    s_req_n = 0;
    memset(s_neg, 0, sizeof(s_neg));
    memset(s_req, 0, sizeof(s_req));
    for (int i = 0; i < PLATE_CACHE_SLOTS; i++) {
        slot_clear(&s_slots[i]);
    }
    s_ready = false;
    s_bulk_update = true;
    for (int i = 0; i < n && s_req_n < PLATE_REQ_QUEUE; i++) {
        char key[8];
        normalize_icao(icaos[i], key, sizeof(key));
        if (!key[0]) {
            continue;
        }
        bool dup = false;
        for (int j = 0; j < s_req_n; j++) {
            if (icao_eq(s_req[j], key)) {
                dup = true;
                break;
            }
        }
        if (dup) {
            continue;
        }
        strncpy(s_req[s_req_n], key, sizeof(s_req[0]) - 1);
        s_req[s_req_n][sizeof(s_req[0]) - 1] = 0;
        s_req_n++;
    }
    const int queued = s_req_n;
    plate_unlock();
    lvgl_port_unlock();
    Serial.printf("[plate] refresh queued %d (SPIFFS wiped)\n", queued);
}

bool faa_plates_bulk_active(void) {
    plate_lock();
    const bool active = s_bulk_update;
    plate_unlock();
    return active;
}

bool faa_plates_bulk_finished(void) {
    plate_lock();
    const bool done = s_bulk_update && s_req_n <= 0;
    if (done) {
        s_bulk_update = false;
    }
    plate_unlock();
    return done;
}

const lv_img_dsc_t *faa_plate_lookup(const char *icao) {
    // DRAW-SAFE: PSRAM peek only. Never SPIFFS/alloc/lock here — that white-flickered
    // the MIPI panel when map/preview called us from LV_EVENT_DRAW_MAIN.
    return faa_plate_peek(icao);
}

/** Load one plate from SPIFFS → PSRAM. Call from UI/net events — never from draw. */
bool faa_plate_load_local(const char *icao) {
    if (faa_plate_peek(icao)) {
        return true;
    }
    char key[8];
    normalize_icao(icao, key, sizeof(key));
    if (!key[0]) {
        return false;
    }
    FrplHeader hdr{};
    uint8_t *pixels = nullptr;
    if (!load_frpl_fs(key, &hdr, &pixels) || !pixels) {
        return false;
    }
    if (!slot_install(key, &hdr, pixels)) {
        heap_caps_free(pixels);
        return false;
    }
    Serial.printf("[plate] SPIFFS→PSRAM %s\n", key);
    return true;
}

/** Download ICAO.frpl from GitHub into a fresh PSRAM buffer. */
static bool load_frpl_http(const char *icao, FrplHeader *out_hdr, uint8_t **out_pixels) {
    *out_pixels = nullptr;
    if (!icao || !icao[0] || !out_hdr) {
        return false;
    }
    char url[220];
    snprintf(url, sizeof(url), "%s/plates/%s.frpl", FR_SD_DATA_BASE_URL, icao);
    // force_close: plate fetch is a one-shot binary; don't keep TLS sticky for ADS-B.
    if (!https_get(url, nullptr, nullptr, 8000, 25000, "application/octet-stream", true)) {
        Serial.printf("[plate] GET fail %s: %s\n", icao, net_last_error());
        return false;
    }
    size_t blen = 0;
    const uint8_t *body = net_http_body(&blen);
    if (!body || blen < sizeof(FrplHeader)) {
        return false;
    }
    FrplHeader hdr{};
    memcpy(&hdr, body, sizeof(hdr));
    if (memcmp(hdr.magic, FRPL_MAGIC, 4) != 0 || hdr.w == 0 || hdr.h == 0 || hdr.data_size == 0 ||
        hdr.data_size > FRPL_MAX_BYTES || blen < sizeof(FrplHeader) + (size_t)hdr.data_size) {
        Serial.printf("[plate] bad FRPL %s (len=%u)\n", icao, (unsigned)blen);
        return false;
    }
    uint8_t *buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)heap_caps_malloc(hdr.data_size, MALLOC_CAP_8BIT);
    }
    if (!buf) {
        Serial.printf("[plate] no PSRAM for %s (%u B)\n", icao, (unsigned)hdr.data_size);
        return false;
    }
    memcpy(buf, body + sizeof(FrplHeader), hdr.data_size);
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
    if (s_req_n > 1) {
        memmove(s_req[0], s_req[1], sizeof(s_req[0]) * (size_t)(s_req_n - 1));
    }
    s_req_n--;
    s_req[s_req_n][0] = 0;
    // During Update Plates, always re-fetch even if a slot somehow remains.
    if (!s_bulk_update && (slot_find(key) || neg_has(key))) {
        plate_unlock();
        return false;
    }
    plate_unlock();

    FrplHeader hdr{};
    uint8_t *pixels = nullptr;
    const bool ok = load_frpl_http(key, &hdr, &pixels);
    if (!ok || !pixels) {
        plate_lock();
        neg_add(key);
        plate_unlock();
        Serial.printf("[plate] miss %s\n", key);
        net_tls_reset();
        return false;
    }

    plate_fs_save(key, &hdr, pixels);
    if (!slot_install(key, &hdr, pixels)) {
        heap_caps_free(pixels);
        return false;
    }

    Serial.printf("[plate] stored %s (%ux%u %u B)\n", key, (unsigned)hdr.w, (unsigned)hdr.h,
                  (unsigned)hdr.data_size);
    return true;
}
