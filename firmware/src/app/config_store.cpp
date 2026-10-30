#include "config_store.h"

#include <Preferences.h>

static Preferences prefs;

// All 50 states on (matches kCountyMaskAllBits in ui.cpp).
static constexpr uint64_t kCountyMaskAll50 = (1ULL << 50) - 1;

static int clamp_max_alt(int alt_ft) {
    if (alt_ft < 5000) {
        return 5000;
    }
    if (alt_ft > 50000) {
        return 50000;
    }
    // Snap to 5000 ft steps.
    return ((alt_ft + 2500) / 5000) * 5000;
}

static int clamp_pct(int v) {
    if (v < 0) {
        return 0;
    }
    if (v > 100) {
        return 100;
    }
    return v;
}

void config_load(AppConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->range_nm = 50.0f;
    cfg->max_alt_ft = 45000;
    cfg->show_airports = true;
    cfg->county_mask = kCountyMaskAll50;
    cfg->brightness = 100;
    cfg->volume = 60;
    if (!prefs.begin("fradar", true)) {
        cfg->startup_range_nm = cfg->range_nm;
        cfg->startup_max_alt_ft = cfg->max_alt_ft;
        cfg->startup_show_airports = cfg->show_airports;
        return;
    }
    prefs.getString("ssid", cfg->wifi_ssid, sizeof(cfg->wifi_ssid));
    prefs.getString("pass", cfg->wifi_pass, sizeof(cfg->wifi_pass));
    prefs.getString("q", cfg->location.query, sizeof(cfg->location.query));
    prefs.getString("label", cfg->location.label, sizeof(cfg->location.label));
    prefs.getString("icao", cfg->location.icao, sizeof(cfg->location.icao));
    cfg->location.lat = prefs.getFloat("lat", 0);
    cfg->location.lon = prefs.getFloat("lon", 0);
    cfg->range_nm = prefs.getFloat("range", 50.0f);
    cfg->max_alt_ft = clamp_max_alt(prefs.getInt("maxalt", 45000));
    cfg->show_airports = prefs.getBool("showapt", true);
    cfg->show_counties = prefs.getBool("showcty", false);
    cfg->show_cities = prefs.getBool("showcity", false);
    cfg->county_mask = prefs.getULong64("ctymask", kCountyMaskAll50) & kCountyMaskAll50;
    cfg->brightness = clamp_pct(prefs.getInt("bright", 100));
    cfg->volume = clamp_pct(prefs.getInt("vol", 60));
    cfg->location.valid = (fabsf(cfg->location.lat) > 0.01f || fabsf(cfg->location.lon) > 0.01f);
    if (cfg->range_nm < 10.0f) {
        cfg->range_nm = 10.0f;
    }
    if (cfg->range_nm > 250.0f) {
        cfg->range_nm = 250.0f;
    }
    // Startup Settings are the persisted view; follow zoom only changes the live values.
    cfg->startup_range_nm = cfg->range_nm;
    cfg->startup_max_alt_ft = cfg->max_alt_ft;
    cfg->startup_show_airports = cfg->show_airports;
    cfg->startup_show_counties = cfg->show_counties;
    cfg->startup_show_cities = cfg->show_cities;
    prefs.end();
}

void config_save(const AppConfig *cfg) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putString("ssid", cfg->wifi_ssid);
    prefs.putString("pass", cfg->wifi_pass);
    prefs.putString("q", cfg->location.query);
    prefs.putString("label", cfg->location.label);
    prefs.putString("icao", cfg->location.icao);
    prefs.putFloat("lat", cfg->location.lat);
    prefs.putFloat("lon", cfg->location.lon);
    prefs.putFloat("range", cfg->range_nm);
    prefs.putInt("maxalt", clamp_max_alt(cfg->max_alt_ft));
    prefs.putBool("showapt", cfg->show_airports);
    prefs.putBool("showcty", cfg->show_counties);
    prefs.putBool("showcity", cfg->show_cities);
    prefs.putULong64("ctymask", cfg->county_mask);
    prefs.putInt("bright", clamp_pct(cfg->brightness));
    prefs.putInt("vol", clamp_pct(cfg->volume));
    prefs.end();
}

void config_save_wifi(const char *ssid, const char *pass) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}

void config_save_location(const GeoFix *fix) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putString("q", fix->query);
    prefs.putString("label", fix->label);
    prefs.putString("icao", fix->icao);
    prefs.putFloat("lat", fix->lat);
    prefs.putFloat("lon", fix->lon);
    prefs.end();
}

void config_save_range(float range_nm) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putFloat("range", range_nm);
    prefs.end();
}

void config_save_max_alt(int max_alt_ft) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putInt("maxalt", clamp_max_alt(max_alt_ft));
    prefs.end();
}

void config_save_show_airports(bool show) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putBool("showapt", show);
    prefs.end();
}

void config_save_show_counties(bool show) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putBool("showcty", show);
    prefs.end();
}

void config_save_show_cities(bool show) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putBool("showcity", show);
    prefs.end();
}

void config_save_county_mask(uint64_t mask) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putULong64("ctymask", mask & kCountyMaskAll50);
    prefs.end();
}

void config_save_brightness(int brightness) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putInt("bright", clamp_pct(brightness));
    prefs.end();
}

void config_save_volume(int volume) {
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putInt("vol", clamp_pct(volume));
    prefs.end();
}

// ---- Id lists stored as a single comma-separated NVS string ----------------

static void list_load(const char *key, char *ids, size_t id_len, size_t max_n, int *n_out) {
    *n_out = 0;
    if (!prefs.begin("fradar", true)) {
        return;
    }
    char buf[256] = {0};
    prefs.getString(key, buf, sizeof(buf));
    prefs.end();
    char *save = nullptr;
    for (char *tok = strtok_r(buf, ",", &save); tok && (size_t)*n_out < max_n; tok = strtok_r(nullptr, ",", &save)) {
        char *dst = ids + (size_t)(*n_out) * id_len;
        strncpy(dst, tok, id_len - 1);
        dst[id_len - 1] = 0;
        if (dst[0]) {
            (*n_out)++;
        }
    }
}

static void list_save(const char *key, const char *ids, size_t id_len, int n) {
    char buf[256] = {0};
    size_t pos = 0;
    for (int i = 0; i < n; i++) {
        const char *s = ids + (size_t)i * id_len;
        const size_t sl = strnlen(s, id_len);
        if (sl == 0 || pos + sl + 2 >= sizeof(buf)) {
            continue;
        }
        if (pos) {
            buf[pos++] = ',';
        }
        memcpy(buf + pos, s, sl);
        pos += sl;
        buf[pos] = 0;
    }
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putString(key, buf);
    prefs.end();
}

void config_load_track_history(char ids[][FR_TRACK_HIST_ID_LEN], int *n) {
    list_load("trkhist", &ids[0][0], FR_TRACK_HIST_ID_LEN, FR_TRACK_HIST_MAX, n);
}

void config_save_track_history(const char ids[][FR_TRACK_HIST_ID_LEN], int n) {
    list_save("trkhist", &ids[0][0], FR_TRACK_HIST_ID_LEN, n);
}

void config_load_board_history(char ids[][FR_BOARD_HIST_ID_LEN], int *n) {
    list_load("brdhist", &ids[0][0], FR_BOARD_HIST_ID_LEN, FR_BOARD_HIST_MAX, n);
}

void config_save_board_history(const char ids[][FR_BOARD_HIST_ID_LEN], int n) {
    list_save("brdhist", &ids[0][0], FR_BOARD_HIST_ID_LEN, n);
}

// Plate extras: "ICAO:City,ICAO:City" (city optional).
void config_load_plate_extras(char ids[][FR_PLATE_EXTRA_ID_LEN], char cities[][FR_PLATE_EXTRA_CITY_LEN], int *n) {
    *n = 0;
    if (!prefs.begin("fradar", true)) {
        return;
    }
    char buf[512] = {0};
    prefs.getString("plextra", buf, sizeof(buf));
    prefs.end();
    char *save = nullptr;
    for (char *tok = strtok_r(buf, ",", &save); tok && *n < FR_PLATE_EXTRA_MAX; tok = strtok_r(nullptr, ",", &save)) {
        char *colon = strchr(tok, ':');
        if (colon) {
            *colon = 0;
        }
        strncpy(ids[*n], tok, FR_PLATE_EXTRA_ID_LEN - 1);
        ids[*n][FR_PLATE_EXTRA_ID_LEN - 1] = 0;
        cities[*n][0] = 0;
        if (colon) {
            strncpy(cities[*n], colon + 1, FR_PLATE_EXTRA_CITY_LEN - 1);
            cities[*n][FR_PLATE_EXTRA_CITY_LEN - 1] = 0;
        }
        if (ids[*n][0]) {
            (*n)++;
        }
    }
}

void config_save_plate_extras(const char ids[][FR_PLATE_EXTRA_ID_LEN], const char cities[][FR_PLATE_EXTRA_CITY_LEN],
                              int n) {
    char buf[512] = {0};
    size_t pos = 0;
    for (int i = 0; i < n; i++) {
        char item[FR_PLATE_EXTRA_ID_LEN + FR_PLATE_EXTRA_CITY_LEN + 2];
        char city[FR_PLATE_EXTRA_CITY_LEN];
        strncpy(city, cities[i], sizeof(city) - 1);
        city[sizeof(city) - 1] = 0;
        for (char *p = city; *p; p++) {  // keep the separators unambiguous
            if (*p == ',' || *p == ':') {
                *p = ' ';
            }
        }
        snprintf(item, sizeof(item), city[0] ? "%s:%s" : "%s", ids[i], city);
        const size_t il = strlen(item);
        if (il == 0 || pos + il + 2 >= sizeof(buf)) {
            continue;
        }
        if (pos) {
            buf[pos++] = ',';
        }
        memcpy(buf + pos, item, il);
        pos += il;
        buf[pos] = 0;
    }
    if (!prefs.begin("fradar", false)) {
        return;
    }
    prefs.putString("plextra", buf);
    prefs.end();
}
