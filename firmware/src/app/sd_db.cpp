#include "sd_db.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include <esp_heap_caps.h>

#include "board_config.h"
#include "sd_store.h"

#define FR_DATA_DIR SD_MOUNT_POINT "/fr_data"

static char s_status[120] = "DB: not loaded";

const WptRec *kWptDb = nullptr;
size_t K_WPT_DB_COUNT = 0;

const AptRec *kAptDb = nullptr;
size_t K_APT_DB_COUNT = 0;

const RwyRec *kRwyDb = nullptr;
size_t K_RWY_DB_COUNT = 0;

const char *sd_db_status_text(void) {
    return s_status;
}

static void *alloc_psram(size_t n) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(n, MALLOC_CAP_8BIT);
    }
    return p;
}

static bool load_blob(const char *path, const char *magic4, size_t rec_size, void **out_ptr,
                      size_t *out_count) {
    if (!path || !magic4 || !out_ptr || !out_count || rec_size == 0) {
        return false;
    }
    if (*out_ptr && *out_count) {
        return true;  // already loaded
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }
    char magic[4];
    uint32_t count = 0;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, magic4, 4) != 0 ||
        fread(&count, 1, 4, f) != 4 || count == 0 || count > 500000u) {
        fclose(f);
        return false;
    }
    const size_t bytes = (size_t)count * rec_size;
    void *buf = alloc_psram(bytes);
    if (!buf) {
        fclose(f);
        return false;
    }
    if (fread(buf, 1, bytes, f) != bytes) {
        heap_caps_free(buf);
        fclose(f);
        return false;
    }
    fclose(f);
    *out_ptr = buf;
    *out_count = count;
    return true;
}

void sd_db_reload(void) {
    // Swap pointers out first so readers never see freed memory through the globals.
    const WptRec *w = kWptDb;
    const AptRec *a = kAptDb;
    const RwyRec *r = kRwyDb;
    kWptDb = nullptr;
    kAptDb = nullptr;
    kRwyDb = nullptr;
    K_WPT_DB_COUNT = 0;
    K_APT_DB_COUNT = 0;
    K_RWY_DB_COUNT = 0;
    if (w) {
        heap_caps_free((void *)w);
    }
    if (a) {
        heap_caps_free((void *)a);
    }
    if (r) {
        heap_caps_free((void *)r);
    }
    sd_db_load();
}

bool sd_db_load(void) {
    if (kWptDb && kAptDb && kRwyDb) {
        snprintf(s_status, sizeof(s_status), "DB: SD ready (wpt %u apt %u rwy %u)",
                 (unsigned)K_WPT_DB_COUNT, (unsigned)K_APT_DB_COUNT, (unsigned)K_RWY_DB_COUNT);
        return true;
    }
    if (!sd_mount()) {
        strncpy(s_status, "DB: SD mount failed", sizeof(s_status) - 1);
        return false;
    }

    void *wpt = (void *)kWptDb;
    void *apt = (void *)kAptDb;
    void *rwy = (void *)kRwyDb;
    size_t nw = K_WPT_DB_COUNT, na = K_APT_DB_COUNT, nr = K_RWY_DB_COUNT;

#pragma pack(push, 1)
    typedef struct {
        char name[6];
        int16_t lat_c;
        int16_t lon_c;
    } WptPacked;
    typedef struct {
        char icao[5];
        char iata[4];
        int16_t lat_c;
        int16_t lon_c;
        int16_t elev_ft;
        uint16_t rwy_ft;
        uint8_t size;
    } AptPacked;
    typedef struct {
        char icao[5];
        char le[4];
        char he[4];
        uint16_t len_ft;
        uint16_t hdg_deg;
        uint8_t width_ft;
        uint8_t is_turf;
        int16_t east_ft;
        int16_t north_ft;
    } RwyPacked;
#pragma pack(pop)

    const bool ok_w =
        load_blob(FR_DATA_DIR "/waypoints.bin", "FRWP", sizeof(WptPacked), &wpt, &nw);
    const bool ok_a =
        load_blob(FR_DATA_DIR "/airports.bin", "FRAP", sizeof(AptPacked), &apt, &na);
    const bool ok_r =
        load_blob(FR_DATA_DIR "/runways.bin", "FRRW", sizeof(RwyPacked), &rwy, &nr);

    if (ok_w) {
        kWptDb = (const WptRec *)wpt;
        K_WPT_DB_COUNT = nw;
    }
    if (ok_a) {
        kAptDb = (const AptRec *)apt;
        K_APT_DB_COUNT = na;
    }
    if (ok_r) {
        kRwyDb = (const RwyRec *)rwy;
        K_RWY_DB_COUNT = nr;
    }

    if (ok_w && ok_a && ok_r) {
        snprintf(s_status, sizeof(s_status), "DB: SD ready (wpt %u apt %u rwy %u)",
                 (unsigned)K_WPT_DB_COUNT, (unsigned)K_APT_DB_COUNT, (unsigned)K_RWY_DB_COUNT);
        Serial.printf("[sd_db] %s\n", s_status);
        return true;
    }
    snprintf(s_status, sizeof(s_status), "DB: copy fr_data to SD (w%d a%d r%d)", ok_w ? 1 : 0,
             ok_a ? 1 : 0, ok_r ? 1 : 0);
    Serial.printf("[sd_db] %s\n", s_status);
    return false;
}
