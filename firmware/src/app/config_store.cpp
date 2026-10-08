#include "config_store.h"

#include <Preferences.h>

static Preferences prefs;

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

void config_load(AppConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->range_nm = 50.0f;
    cfg->max_alt_ft = 45000;
    cfg->show_airports = true;
    if (!prefs.begin("fradar", true)) {
        return;
    }
    prefs.getString("ssid", cfg->wifi_ssid, sizeof(cfg->wifi_ssid));
    prefs.getString("pass", cfg->wifi_pass, sizeof(cfg->wifi_pass));
    prefs.getString("q", cfg->location.query, sizeof(cfg->location.query));
    prefs.getString("label", cfg->location.label, sizeof(cfg->location.label));
    cfg->location.lat = prefs.getFloat("lat", 0);
    cfg->location.lon = prefs.getFloat("lon", 0);
    cfg->range_nm = prefs.getFloat("range", 50.0f);
    cfg->max_alt_ft = clamp_max_alt(prefs.getInt("maxalt", 45000));
    cfg->show_airports = prefs.getBool("showapt", true);
    cfg->location.valid = (fabsf(cfg->location.lat) > 0.01f || fabsf(cfg->location.lon) > 0.01f);
    if (cfg->range_nm < 10.0f) {
        cfg->range_nm = 10.0f;
    }
    if (cfg->range_nm > 250.0f) {
        cfg->range_nm = 250.0f;
    }
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
    prefs.putFloat("lat", cfg->location.lat);
    prefs.putFloat("lon", cfg->location.lon);
    prefs.putFloat("range", cfg->range_nm);
    prefs.putInt("maxalt", clamp_max_alt(cfg->max_alt_ft));
    prefs.putBool("showapt", cfg->show_airports);
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
