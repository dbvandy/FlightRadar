#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <math.h>
#include <strings.h>
#include <time.h>

#include "lvgl_v8_port.h"
#include "board_ctrl.h"
#include "app/audio_click.h"
#include "app/config_store.h"
#include "app/airport_plate_db.h"
#include "app/faa_plate_db.h"
#include "app/net_client.h"
#include "app/ui.h"

#ifdef BOARD_CROWPANEL_P4
#include <esp_ldo_regulator.h>
#include "board_config.h"
#endif

using namespace esp_panel::drivers;
using namespace esp_panel::board;

static SharedState g_state;
static Board *g_board = nullptr;
static int g_brightness = 50;

void board_set_brightness(int percent) {
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }
    g_brightness = percent;
    if (g_board) {
        if (auto *bl = g_board->getBacklight()) {
            bl->setBrightness((uint8_t)percent);
        }
    }
}

int board_get_brightness() {
    return g_brightness;
}

static uint32_t take_requests() {
    return __sync_fetch_and_and((uint32_t *)&g_state.requests, 0u);
}

static void set_status_unlocked(const char *msg) {
    strncpy(g_state.status, msg, sizeof(g_state.status) - 1);
    g_state.status[sizeof(g_state.status) - 1] = 0;
}

// Forward — defined with PSRAM scratch helpers below.
static bool ensure_net_scratch();

static bool wifi_connect_sta(const char *ssid, const char *pass, uint32_t timeout_ms) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("flight-radar");
    // Drop existing STA first — Hosted DNS -54 often leaves WL_CONNECTED true;
    // WiFi.begin() alone then no-ops and never refreshes DNS (board/ADS-B stay dead).
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(false, false);
        delay(250);
    }
    WiFi.begin(ssid, pass);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        // Keep DHCP address; pin public DNS (Hosted stack sometimes wedges on ISP DNS).
        WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), IPAddress(8, 8, 8, 8),
                    IPAddress(1, 1, 1, 1));
        // After WiFi is up: only LARGE allocs prefer PSRAM. Small TLS/AES DMA
        // buffers must stay internal. enable(64) broke WiFi; enable(4096) is safe.
        heap_caps_malloc_extmem_enable(4096);
        // Multiple servers — clocks stay blank until time(nullptr) passes ~2023.
        configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
        ensure_net_scratch();
        net_arm_tls_dma_after_wifi();
    }
    return WiFi.status() == WL_CONNECTED;
}

static void handle_wifi_scan() {
    ui_set_status("Scanning Wi-Fi...");
    char restore_ssid[33] = {0};
    char restore_pass[65] = {0};
    const bool was_up = (WiFi.status() == WL_CONNECTED);
    if (was_up || g_state.cfg.wifi_ssid[0]) {
        strncpy(restore_ssid, g_state.cfg.wifi_ssid, sizeof(restore_ssid) - 1);
        strncpy(restore_pass, g_state.cfg.wifi_pass, sizeof(restore_pass) - 1);
    }
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    delay(80);
    const int n = WiFi.scanNetworks(false, false);
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.wifi_count = 0;
    for (int i = 0; i < n && g_state.wifi_count < FR_MAX_WIFI; i++) {
        strncpy(g_state.wifi_names[g_state.wifi_count], WiFi.SSID(i).c_str(), 32);
        g_state.wifi_names[g_state.wifi_count][32] = 0;
        g_state.wifi_rssi[g_state.wifi_count] = WiFi.RSSI(i);
        g_state.wifi_count++;
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_wifi_scan_done();
    // Scan drops STA — rejoin so ADS-B keeps working after the list fills.
    if (restore_ssid[0]) {
        const bool ok = wifi_connect_sta(restore_ssid, restore_pass, 15000);
        g_state.wifi_connected = ok;
        if (ok) {
            ui_set_status("Scan done — Wi-Fi restored");
        }
    }
}

static void handle_wifi_connect() {
    char ssid[33];
    char pass[65];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(ssid, g_state.pending_ssid, sizeof(ssid) - 1);
    strncpy(pass, g_state.pending_pass, sizeof(pass) - 1);
    ssid[32] = 0;
    pass[64] = 0;
    g_state.wifi_connecting = true;
    xSemaphoreGive(g_state.mutex);

    const bool ok = wifi_connect_sta(ssid, pass, 20000);
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.wifi_connecting = false;
    g_state.wifi_connected = ok;
    if (ok) {
        strncpy(g_state.cfg.wifi_ssid, ssid, sizeof(g_state.cfg.wifi_ssid) - 1);
        strncpy(g_state.cfg.wifi_pass, pass, sizeof(g_state.cfg.wifi_pass) - 1);
        lvgl_port_lock(-1);
        config_save_wifi(ssid, pass);
        lvgl_port_unlock();
        snprintf(g_state.status, sizeof(g_state.status), "Wi-Fi %s  %s", ssid, WiFi.localIP().toString().c_str());
    } else {
        snprintf(g_state.status, sizeof(g_state.status), "Could not join %s", ssid);
    }
    char msg[96];
    strncpy(msg, g_state.status, sizeof(msg) - 1);
    xSemaphoreGive(g_state.mutex);
    ui_notify_wifi_result(ok, msg);
}

static void handle_geocode() {
    char q[96];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(q, g_state.pending_query, sizeof(q) - 1);
    g_state.geo_searching = true;
    xSemaphoreGive(g_state.mutex);

    GeoFix results[8];
    const int n = net_geocode(q, results, 8);
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.geo_searching = false;
    g_state.geo_count = n > 0 ? n : 0;
    if (n > 0) {
        memcpy(g_state.geo_results, results, sizeof(GeoFix) * g_state.geo_count);
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_geo_done();
}

// Traffic/airport scratch in PSRAM — leave internal DMA free for WiFi + TLS AES.
static Aircraft *g_traffic_tmp = nullptr;
static Airport *g_airport_tmp = nullptr;

static bool ensure_net_scratch() {
    if (!g_traffic_tmp) {
        g_traffic_tmp = (Aircraft *)heap_caps_malloc(sizeof(Aircraft) * FR_MAX_AIRCRAFT,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_traffic_tmp) {
            g_traffic_tmp = (Aircraft *)heap_caps_malloc(sizeof(Aircraft) * FR_MAX_AIRCRAFT, MALLOC_CAP_8BIT);
        }
        if (g_traffic_tmp) {
            memset(g_traffic_tmp, 0, sizeof(Aircraft) * FR_MAX_AIRCRAFT);
        }
    }
    if (!g_airport_tmp) {
        g_airport_tmp = (Airport *)heap_caps_malloc(sizeof(Airport) * FR_MAX_AIRPORTS,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_airport_tmp) {
            g_airport_tmp = (Airport *)heap_caps_malloc(sizeof(Airport) * FR_MAX_AIRPORTS, MALLOC_CAP_8BIT);
        }
        if (g_airport_tmp) {
            memset(g_airport_tmp, 0, sizeof(Airport) * FR_MAX_AIRPORTS);
        }
    }
    return g_traffic_tmp && g_airport_tmp;
}

static void handle_save_range() {
    float range;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    range = g_state.cfg.range_nm;
    xSemaphoreGive(g_state.mutex);
    // NVS/flash stalls RGB DMA — pause LVGL while writing.
    lvgl_port_lock(-1);
    config_save_range(range);
    lvgl_port_unlock();
}

static void handle_save_max_alt() {
    int max_alt;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    max_alt = g_state.cfg.max_alt_ft;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_max_alt(max_alt);
    lvgl_port_unlock();
}

static void handle_save_show_airports() {
    bool show;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    show = g_state.cfg.show_airports;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_show_airports(show);
    lvgl_port_unlock();
}

static void handle_save_show_counties() {
    bool show;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    show = g_state.cfg.show_counties;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_show_counties(show);
    lvgl_port_unlock();
}

static void handle_save_show_cities() {
    bool show;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    show = g_state.cfg.show_cities;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_show_cities(show);
    lvgl_port_unlock();
}

static void handle_save_county_mask() {
    uint64_t mask;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    mask = g_state.cfg.county_mask;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_county_mask(mask);
    lvgl_port_unlock();
}

static void handle_save_brightness() {
    int bright;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    bright = g_state.cfg.brightness;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_brightness(bright);
    lvgl_port_unlock();
}

static void handle_save_volume() {
    int vol;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    vol = g_state.cfg.volume;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_volume(vol);
    lvgl_port_unlock();
}

static void handle_save_location() {
    GeoFix fix{};
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    fix = g_state.cfg.location;
    xSemaphoreGive(g_state.mutex);
    // Airport homes: persist/display ICAO only (no facility name or street address).
    const bool enriched = net_enrich_airport_home_label(&fix);
    if (enriched) {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.cfg.location = fix;
        xSemaphoreGive(g_state.mutex);
    }
    lvgl_port_lock(-1);
    config_save_location(&fix);
    if (enriched) {
        ui_notify_home_label(fix.label);
    }
    lvgl_port_unlock();
}

// Do not hold the LVGL lock across HTTPS — that freezes touch (zoom feels
// dead during ADS-B). Status spam is muted for the fetch duration only.
static void net_begin_psram_safe() {
    net_set_status_callback(nullptr);
}

static void net_end_psram_safe() {
    net_set_status_callback(ui_set_status);
}

static void ac_set_dst_from(Aircraft *ac, float olat, float olon) {
    if (!ac) {
        return;
    }
    float east = 0, north = 0;
    ll_to_nm(ac->lat, ac->lon, olat, olon, &east, &north);
    ac->dst_nm = sqrtf(east * east + north * north);
}

static int traffic_keep_closest(Aircraft *list, int n, const char *priority_hex, int keep_max) {
    if (n <= 0 || keep_max <= 0) {
        return 0;
    }
    if (n > keep_max) {
        // Simple selection: repeatedly pull next-nearest into place (n is small, <= 64).
        for (int i = 0; i < keep_max; i++) {
            int best = i;
            for (int j = i + 1; j < n; j++) {
                const bool pri_b =
                    priority_hex && priority_hex[0] && strcasecmp(list[best].hex, priority_hex) == 0;
                const bool pri_j =
                    priority_hex && priority_hex[0] && strcasecmp(list[j].hex, priority_hex) == 0;
                if (pri_j && !pri_b) {
                    best = j;
                } else if (pri_b == pri_j && list[j].dst_nm < list[best].dst_nm) {
                    best = j;
                }
            }
            if (best != i) {
                Aircraft swap = list[i];
                list[i] = list[best];
                list[best] = swap;
            }
        }
        n = keep_max;
    } else if (priority_hex && priority_hex[0]) {
        // Ensure tracked flight is first among the kept set.
        for (int i = 1; i < n; i++) {
            if (strcasecmp(list[i].hex, priority_hex) == 0) {
                Aircraft swap = list[0];
                list[0] = list[i];
                list[i] = swap;
                break;
            }
        }
    }
    return n;
}

static bool traffic_passes_alt_filter(const Aircraft *ac, int max_alt_ft) {
    if (!ac || !ac->valid) {
        return false;
    }
    // 50000+ ft = no ceiling (Home / boot unfiltered).
    if (max_alt_ft <= 0 || max_alt_ft >= 50000) {
        return true;
    }
    // Match UI: keep on-ground traffic when a ceiling is active.
    if (ac->on_ground) {
        return true;
    }
    return ac->alt_ft <= (float)max_alt_ft;
}

static void handle_traffic() {
    float lat, lon, range;
    int max_alt_ft = 5000;
    bool follow_active = false, follow_solo = false;
    char follow_hex[8] = {0};
    char follow_flight[12] = {0};
    Aircraft tracked{};
    bool tracked_ok = false;
    uint32_t epoch = 0;

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    lat = g_state.cfg.location.lat;
    lon = g_state.cfg.location.lon;
    range = g_state.cfg.range_nm;
    max_alt_ft = g_state.cfg.max_alt_ft;
    follow_active = g_state.follow_active;
    follow_solo = g_state.follow_solo;
    strncpy(follow_hex, g_state.follow_hex, sizeof(follow_hex) - 1);
    strncpy(follow_flight, g_state.follow_flight, sizeof(follow_flight) - 1);
    tracked_ok = g_state.tracked_ok || (g_state.tracked_ac.valid && g_state.tracked_ac.hex[0]);
    if (tracked_ok) {
        tracked = g_state.tracked_ac;
    }
    epoch = g_state.traffic_epoch;
    xSemaphoreGive(g_state.mutex);
    if (!g_state.cfg.location.valid && !(follow_active && follow_hex[0])) {
        return;
    }

    if (!ensure_net_scratch()) {
        ui_set_status("No traffic scratch RAM");
        return;
    }
    Aircraft *tmp = g_traffic_tmp;
    int n = -1;
    const bool want_solo = follow_active && follow_solo && (follow_hex[0] || follow_flight[0]);
    net_begin_psram_safe();

    // While tracking: always fetch the followed aircraft only (even after pan/route fit).
    if (want_solo) {
        ui_set_status("ADS-B tracked...");
        Aircraft one{};
        n = -1;
        if (follow_hex[0]) {
            n = net_fetch_aircraft_by_hex(follow_hex, &one);
        }
        if (n <= 0 && follow_flight[0]) {
            n = net_fetch_aircraft_by_callsign(follow_flight, &one);
        }
        if (n > 0) {
            ac_set_dst_from(&one, lat, lon);
            tmp[0] = one;
            n = 1;
        } else if (tracked_ok && tracked.valid) {
            // Keep last known / pre-departure pin so follow UI stays populated.
            ac_set_dst_from(&tracked, lat, lon);
            tmp[0] = tracked;
            n = 1;
        } else {
            n = (n < 0) ? -1 : 0;
        }
    } else {
        // Full local traffic around map center (home, pan, or unfollow).
        ui_set_status("Updating ADS-B...");
        float fetch_nm = range;
        if (fetch_nm > FR_ADS_B_MAX_NM) {
            fetch_nm = FR_ADS_B_MAX_NM;
        }
        if (fetch_nm < 10.0f) {
            fetch_nm = 10.0f;
        }
        n = net_fetch_aircraft(lat, lon, fetch_nm, tmp, FR_MAX_AIRCRAFT);
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                ac_set_dst_from(&tmp[i], lat, lon);
            }
        }
    }

    net_end_psram_safe();

    // Home/Unfollow during a solo HTTPS call: discard the 1-plane result and
    // fetch full local traffic with the current home settings.
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    const uint32_t epoch_now = g_state.traffic_epoch;
    const bool still_solo = g_state.follow_active && g_state.follow_solo;
    if (want_solo && (!still_solo || epoch_now != epoch)) {
        lat = g_state.cfg.location.lat;
        lon = g_state.cfg.location.lon;
        range = g_state.cfg.range_nm;
        max_alt_ft = g_state.cfg.max_alt_ft;
        epoch = epoch_now;
        xSemaphoreGive(g_state.mutex);

        net_begin_psram_safe();
        ui_set_status("Updating ADS-B...");
        float fetch_nm = range;
        if (fetch_nm > FR_ADS_B_MAX_NM) {
            fetch_nm = FR_ADS_B_MAX_NM;
        }
        if (fetch_nm < 10.0f) {
            fetch_nm = 10.0f;
        }
        n = net_fetch_aircraft(lat, lon, fetch_nm, tmp, FR_MAX_AIRCRAFT);
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                ac_set_dst_from(&tmp[i], lat, lon);
            }
        }
        net_end_psram_safe();
        follow_active = false;
        follow_solo = false;
    } else {
        xSemaphoreGive(g_state.mutex);
    }

    // Altitude ceiling for local radar only (solo track already has one aircraft).
    if (n > 0 && !(follow_active && follow_solo)) {
        int kept = 0;
        for (int i = 0; i < n; i++) {
            if (traffic_passes_alt_filter(&tmp[i], max_alt_ft)) {
                if (kept != i) {
                    tmp[kept] = tmp[i];
                }
                kept++;
            }
        }
        n = kept;
    }
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    // Another Home/Unfollow landed while we were fetching — don't publish stale data.
    if (g_state.traffic_epoch != epoch) {
        xSemaphoreGive(g_state.mutex);
        return;
    }
    if (n >= 0) {
        memcpy(g_state.aircraft, tmp, sizeof(Aircraft) * n);
        g_state.aircraft_count = n;
        g_state.traffic_ok = true;
        g_state.last_traffic_ms = millis();
        if (follow_active && follow_solo) {
            snprintf(g_state.status, sizeof(g_state.status), "ADS-B tracked  %d ac", n);
        } else {
            snprintf(g_state.status, sizeof(g_state.status), "ADS-B live  %d ac", n);
        }
        // Refresh tracked snapshot when we have a hex match.
        if (follow_active && follow_hex[0]) {
            for (int i = 0; i < n; i++) {
                if (strcasecmp(tmp[i].hex, follow_hex) == 0) {
                    g_state.tracked_ac = tmp[i];
                    g_state.tracked_ok = true;
                    break;
                }
            }
        }
    } else {
        g_state.traffic_ok = false;
        snprintf(g_state.status, sizeof(g_state.status), "ADS-B: %s", net_last_error());
    }
    xSemaphoreGive(g_state.mutex);
    // Coalesced labels+PPI after fetch; short settle so RGB DMA is idle.
    ui_schedule_radar_redraw(180);
}

static void handle_airports() {
    float lat, lon, range;
    bool class_b_only = false;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    lat = g_state.cfg.location.lat;
    lon = g_state.cfg.location.lon;
    range = g_state.cfg.range_nm;
    class_b_only = g_state.follow_active;
    xSemaphoreGive(g_state.mutex);
    if (!g_state.cfg.location.valid) {
        return;
    }
    ui_set_status(class_b_only ? "Loading Class B airports..." : "Loading airports...");
    if (!ensure_net_scratch()) {
        ui_set_status("No airport scratch RAM");
        return;
    }
    Airport *tmp = g_airport_tmp;
    net_begin_psram_safe();
    const int n = net_fetch_airports(lat, lon, range, tmp, FR_MAX_AIRPORTS, class_b_only);
    net_end_psram_safe();
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    if (n >= 0) {
        memcpy(g_state.airports, tmp, sizeof(Airport) * n);
        g_state.airport_count = n;
        snprintf(g_state.status, sizeof(g_state.status),
                 class_b_only ? "%d Class B airports in view" : "%d airports in view", n);
    } else {
        set_status_unlocked("Airport fetch failed");
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_airports();
}

static void handle_weather() {
    char icao[8];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(icao, g_state.pending_icao, sizeof(icao) - 1);
    icao[7] = 0;
    xSemaphoreGive(g_state.mutex);
    Weather w{};
    net_begin_psram_safe();
    const bool ok = net_fetch_weather(icao, &w);
    // AWC radios here (net task) — ui_notify_weather must not HTTPS under LVGL lock.
    AirportRadios radios{};
    if (icao[0] && net_lookup_airport_radios(icao, &radios)) {
        w.radios = radios;
    }
    net_end_psram_safe();
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.weather = w;
    if (!ok) {
        strncpy(g_state.weather.icao, icao, sizeof(g_state.weather.icao) - 1);
        strncpy(g_state.weather.raw_metar, "No METAR available", sizeof(g_state.weather.raw_metar) - 1);
        g_state.weather.ok = false;
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_weather();
}

static void handle_flight_info() {
    char flight[12];
    char reg[12];
    char hex[8];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(flight, g_state.pending_flight, sizeof(flight) - 1);
    flight[sizeof(flight) - 1] = 0;
    strncpy(reg, g_state.pending_reg, sizeof(reg) - 1);
    reg[sizeof(reg) - 1] = 0;
    strncpy(hex, g_state.pending_hex, sizeof(hex) - 1);
    hex[sizeof(hex) - 1] = 0;
    xSemaphoreGive(g_state.mutex);

    FlightInfo fi{};
    strncpy(fi.flight, flight, sizeof(fi.flight) - 1);
    strncpy(fi.reg, reg, sizeof(fi.reg) - 1);
    strncpy(fi.hex, hex, sizeof(fi.hex) - 1);

    // Prefer ICAO callsign for FlightStats / FlightAware (DAL3161 vs DL3161).
    char icao_cs[16] = {0}, iata_al[4] = {0}, iata_num[8] = {0};
    char sched_key[16] = {0};
    if (flight[0] &&
        net_parse_flight_number(flight, icao_cs, sizeof(icao_cs), iata_al, sizeof(iata_al), iata_num,
                                sizeof(iata_num))) {
        if (icao_cs[0]) {
            strncpy(fi.flight, icao_cs, sizeof(fi.flight) - 1);
        }
        if (iata_al[0] && iata_num[0]) {
            snprintf(sched_key, sizeof(sched_key), "%s%s", iata_al, iata_num);
        } else {
            strncpy(sched_key, icao_cs[0] ? icao_cs : flight, sizeof(sched_key) - 1);
        }
    } else if (flight[0]) {
        strncpy(sched_key, flight, sizeof(sched_key) - 1);
    }

    auto push_fi = [&]() {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.flight_info = fi;
        xSemaphoreGive(g_state.mutex);
        ui_notify_flight_info();
    };

    // Track / board often already filled OD / FlightStats — seed so a failed FS
    // retry cannot wipe dep/arr already on screen, then jump to FA for waypoints.
    bool have_sched = false;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    if (g_state.follow_active) {
        const char *ff = g_state.flight_info.flight;
        const char *fh = g_state.flight_info.hex;
        const bool hex_same = hex[0] && fh[0] && strcasecmp(hex, fh) == 0;
        const bool flt_same =
            flight[0] && ff[0] &&
            (strcasecmp(flight, ff) == 0 ||
             (icao_cs[0] && strcasecmp(icao_cs, ff) == 0) ||
             (sched_key[0] && strcasecmp(sched_key, ff) == 0));
        if ((hex_same || flt_same) &&
            (g_state.flight_info.schedule_ok || g_state.flight_info.route_ok ||
             g_state.flight_info.origin[0] || g_state.flight_info.dest[0] ||
             g_state.flight_info.route_wpt_n > 0)) {
            fi = g_state.flight_info;
            have_sched = fi.schedule_ok && (fi.origin[0] || fi.dest[0]);
        }
    }
    xSemaphoreGive(g_state.mutex);

    if (net_abort_requested()) {
        ui_set_status("Tracking");
        return;
    }

    ui_set_status(have_sched ? "Loading route…" : (flight[0] ? "Looking up flight..." : "Looking up aircraft..."));
    net_begin_psram_safe();
    net_tls_reset();
    delay(300);

    if (!have_sched && !net_abort_requested()) {
        // Resolve registration from hexdb when ADS-B did not provide `r`.
        if (hex[0]) {
            char meta_reg[12] = {0};
            fi.meta_ok = net_fetch_aircraft_meta(hex, fi.manufacturer, sizeof(fi.manufacturer), fi.type_name,
                                                 sizeof(fi.type_name), fi.owner, sizeof(fi.owner), meta_reg,
                                                 sizeof(meta_reg));
            if (!reg[0] && meta_reg[0]) {
                strncpy(reg, meta_reg, sizeof(reg) - 1);
                reg[sizeof(reg) - 1] = 0;
            }
        }
        if (reg[0]) {
            strncpy(fi.reg, reg, sizeof(fi.reg) - 1);
        }

        // FlightStats first — publish so dep/arr appear before the slow FA scrape.
        if (sched_key[0] && !net_abort_requested()) {
            ui_set_status("Flight stats...");
            net_tls_reset();
            delay(250);
            net_fetch_flight_schedule(sched_key, &fi);
        }
        if ((!fi.origin[0] || !fi.dest[0]) && (icao_cs[0] || flight[0]) && !net_abort_requested()) {
            const char *rk = icao_cs[0] ? icao_cs : flight;
            char o[8] = {0}, d[8] = {0};
            if (net_fetch_route(rk, o, sizeof(o), d, sizeof(d))) {
                if (!fi.origin[0] && o[0]) {
                    strncpy(fi.origin, o, sizeof(fi.origin) - 1);
                }
                if (!fi.dest[0] && d[0]) {
                    strncpy(fi.dest, d, sizeof(fi.dest) - 1);
                }
            }
        }
        if ((!fi.origin[0] || !fi.dest[0]) && reg[0] && !net_abort_requested()) {
            char o[8] = {0}, d[8] = {0};
            if (net_fetch_route(reg, o, sizeof(o), d, sizeof(d))) {
                if (!fi.origin[0] && o[0]) {
                    strncpy(fi.origin, o, sizeof(fi.origin) - 1);
                }
                if (!fi.dest[0] && d[0]) {
                    strncpy(fi.dest, d, sizeof(fi.dest) - 1);
                }
            }
        }
        if (fi.origin[0] && fi.dest[0]) {
            fi.route_ok = true;
        }
        if ((fi.origin[0] || fi.dest[0]) && (!fi.origin_name[0] || !fi.dest_name[0])) {
            net_fetch_airport_names(fi.origin, fi.dest, fi.origin_name, sizeof(fi.origin_name), fi.dest_name,
                                    sizeof(fi.dest_name));
        }
        push_fi();  // dep/arr on radar while FA loads

        // Live ADS-B between FlightStats and FA.
        net_end_psram_safe();
        handle_traffic();
        net_begin_psram_safe();
    }

    // Filed IFR waypoints + historical alt/gs track from FlightAware (TLS-heavy).
    static FlightTrackPt s_fa_track[FR_MAX_TRACK_PTS];
    int track_n = 0;
    const char *plan_key = icao_cs[0] ? icao_cs : (flight[0] ? flight : nullptr);
    if (plan_key && !net_abort_requested()) {
        ui_set_status("Loading route…");
        net_tls_reset();
        delay(400);
        if (!net_abort_requested()) {
            net_fetch_flight_plan(plan_key, &fi, s_fa_track, FR_MAX_TRACK_PTS, &track_n);
        }
    }
    net_end_psram_safe();

    // Merge into shared state — never blank out OD already published for this flight.
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    if (g_state.follow_active &&
        (g_state.flight_info.origin[0] || g_state.flight_info.dest[0]) &&
        (!fi.origin[0] || !fi.dest[0])) {
        if (!fi.origin[0] && g_state.flight_info.origin[0]) {
            strncpy(fi.origin, g_state.flight_info.origin, sizeof(fi.origin) - 1);
            strncpy(fi.origin_name, g_state.flight_info.origin_name, sizeof(fi.origin_name) - 1);
        }
        if (!fi.dest[0] && g_state.flight_info.dest[0]) {
            strncpy(fi.dest, g_state.flight_info.dest, sizeof(fi.dest) - 1);
            strncpy(fi.dest_name, g_state.flight_info.dest_name, sizeof(fi.dest_name) - 1);
        }
        if (fi.origin[0] && fi.dest[0]) {
            fi.route_ok = true;
        }
    }
    g_state.flight_info = fi;
    if (track_n > 0) {
        const int ncopy = (track_n > FR_MAX_TRACK_PTS) ? FR_MAX_TRACK_PTS : track_n;
        memcpy(g_state.flight_track, s_fa_track, sizeof(FlightTrackPt) * (size_t)ncopy);
        g_state.flight_track_n = ncopy;
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_flight_info();

    // ADS-B + chart samples right after FA (queued refresh alone often waits behind plates).
    if (!net_abort_requested()) {
        net_tls_reset();
        delay(350);
        handle_traffic();
        __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_REFRESH_TRAFFIC);
    }

    // Never leave the PPI stuck on "Loading route…" — Update ADS-B must be able to recover.
    if (net_abort_requested()) {
        ui_set_status("Tracking");
    } else if (track_n > 0 || fi.route_wpt_n > 0 || fi.schedule_ok || fi.route_ok) {
        ui_set_status("Tracking");
    } else if (fi.meta_ok) {
        ui_set_status("Aircraft info ready");
    } else {
        ui_set_status("Tracking");
    }
}

static void cpy(char *dst, size_t n, const char *src) {
    if (!dst || n == 0) {
        return;
    }
    strncpy(dst, src ? src : "", n - 1);
    dst[n - 1] = 0;
}

static void handle_search_airports() {
    char q[96];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(q, g_state.pending_query, sizeof(q) - 1);
    q[sizeof(q) - 1] = 0;
    g_state.apt_hit_n = 0;
    xSemaphoreGive(g_state.mutex);

    AptSearchHit hits[FR_MAX_APT_SEARCH];
    memset(hits, 0, sizeof(hits));
    net_begin_psram_safe();
    const int n = net_search_airports(q, hits, FR_MAX_APT_SEARCH);
    net_end_psram_safe();

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    memcpy(g_state.apt_hits, hits, sizeof(AptSearchHit) * (n > 0 ? n : 0));
    g_state.apt_hit_n = n > 0 ? n : 0;
    xSemaphoreGive(g_state.mutex);
    if (n > 0) {
        ui_notify_airport_search(true, "Select an airport");
    } else {
        ui_notify_airport_search(false, "No airports found");
    }
}

static void handle_refresh_plate_cities() {
    char icaos[24][8];
    char cities[24][28];
    int n = 0;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    n = g_state.plate_city_n;
    if (n > 24) {
        n = 24;
    }
    for (int i = 0; i < n; i++) {
        strncpy(icaos[i], g_state.plate_city_icao[i], sizeof(icaos[i]) - 1);
        icaos[i][sizeof(icaos[i]) - 1] = 0;
        cities[i][0] = 0;
    }
    xSemaphoreGive(g_state.mutex);
    if (n <= 0) {
        ui_notify_plate_cities(false);
        return;
    }
    ui_set_status("Refreshing plate cities...");
    net_begin_psram_safe();
    const int hits = net_fetch_airport_cities(icaos, cities, n);
    net_end_psram_safe();
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    for (int i = 0; i < n; i++) {
        strncpy(g_state.plate_city_name[i], cities[i], sizeof(g_state.plate_city_name[i]) - 1);
        g_state.plate_city_name[i][sizeof(g_state.plate_city_name[i]) - 1] = 0;
    }
    xSemaphoreGive(g_state.mutex);
    ui_notify_plate_cities(hits > 0);
}

static void handle_airport_board() {
    char icao[8];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(icao, g_state.pending_board_icao, sizeof(icao) - 1);
    icao[sizeof(icao) - 1] = 0;
    // Keep previous board rows until a successful fetch (reopen must not wipe UI).
    xSemaphoreGive(g_state.mutex);

    // Static — keep board rows off the net_task stack.
    static AirportBoardRow s_arr[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_dep[FR_MAX_BOARD_ROWS];
    char label[48] = {0};
    int an = 0, dn = 0;
    memset(s_arr, 0, sizeof(s_arr));
    memset(s_dep, 0, sizeof(s_dep));

    ui_set_status("Loading airport board...");
    ui_set_board_status("Loading board…");
    float board_lat = 0, board_lon = 0;

    // Clear sticky abort left by Update ADS-B / track FA thrash so board can run.
    net_abort_clear();

    // FlightAware live/airport/{ICAO} — Ident/Type/From|To/Depart/Arrive in one page.
    // Do not fall back to local ADS-B (wrong columns + misleading "No traffic near airport").
    const bool ok = net_fetch_airport_board(icao, s_arr, FR_MAX_BOARD_ROWS, &an, s_dep, FR_MAX_BOARD_ROWS, &dn,
                                            label, sizeof(label), &board_lat, &board_lon);

    if (ok) {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        memcpy(g_state.board_arr, s_arr, sizeof(s_arr));
        memcpy(g_state.board_dep, s_dep, sizeof(s_dep));
        g_state.board_arr_n = an;
        g_state.board_dep_n = dn;
        g_state.board_lat = board_lat;
        g_state.board_lon = board_lon;
        g_state.board_ll_ok = (fabsf(board_lat) + fabsf(board_lon) > 0.01f);
        strncpy(g_state.board_icao, icao, sizeof(g_state.board_icao) - 1);
        g_state.board_icao[sizeof(g_state.board_icao) - 1] = 0;
        strncpy(g_state.board_label, label[0] ? label : icao, sizeof(g_state.board_label) - 1);
        g_state.board_label[sizeof(g_state.board_label) - 1] = 0;
        xSemaphoreGive(g_state.mutex);
        char msg[64];
        snprintf(msg, sizeof(msg), "%d arrivals · %d departures", an, dn);
        ui_notify_airport_board(true, msg);
    } else {
        ui_notify_airport_board(false, net_last_error()[0] ? net_last_error() : "Board fetch failed");
    }
}

static void handle_route_search() {
    char dep_q[48], arr_q[48];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(dep_q, g_state.pending_route_dep, sizeof(dep_q) - 1);
    dep_q[sizeof(dep_q) - 1] = 0;
    strncpy(arr_q, g_state.pending_route_arr, sizeof(arr_q) - 1);
    arr_q[sizeof(arr_q) - 1] = 0;
    g_state.route_dep_hit_n = 0;
    g_state.route_arr_hit_n = 0;
    g_state.route_row_n = 0;
    g_state.route_dep_icao[0] = 0;
    g_state.route_arr_icao[0] = 0;
    xSemaphoreGive(g_state.mutex);

    // Keep large hit/row buffers off the net-task stack.
    static AptSearchHit s_dep_hits[FR_MAX_APT_SEARCH];
    static AptSearchHit s_arr_hits[FR_MAX_APT_SEARCH];
    static AirportBoardRow s_route_rows[FR_MAX_BOARD_ROWS];
    memset(s_dep_hits, 0, sizeof(s_dep_hits));
    memset(s_arr_hits, 0, sizeof(s_arr_hits));

    ui_set_status("Searching airports...");
    net_begin_psram_safe();
    const int dn = net_search_airports(dep_q, s_dep_hits, FR_MAX_APT_SEARCH);
    const int an = net_search_airports(arr_q, s_arr_hits, FR_MAX_APT_SEARCH);
    net_end_psram_safe();

    char dep_icao[8] = {0}, arr_icao[8] = {0};
    if (dn == 1) {
        strncpy(dep_icao, s_dep_hits[0].icao, sizeof(dep_icao) - 1);
    }
    if (an == 1) {
        strncpy(arr_icao, s_arr_hits[0].icao, sizeof(arr_icao) - 1);
    }

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    memcpy(g_state.route_dep_hits, s_dep_hits, sizeof(AptSearchHit) * (dn > 0 ? dn : 0));
    g_state.route_dep_hit_n = dn > 0 ? dn : 0;
    memcpy(g_state.route_arr_hits, s_arr_hits, sizeof(AptSearchHit) * (an > 0 ? an : 0));
    g_state.route_arr_hit_n = an > 0 ? an : 0;
    if (dep_icao[0]) {
        strncpy(g_state.route_dep_icao, dep_icao, sizeof(g_state.route_dep_icao) - 1);
    }
    if (arr_icao[0]) {
        strncpy(g_state.route_arr_icao, arr_icao, sizeof(g_state.route_arr_icao) - 1);
    }
    xSemaphoreGive(g_state.mutex);

    if (dn <= 0) {
        ui_notify_route_search(false, "No departure airport found");
        return;
    }
    if (an <= 0) {
        ui_notify_route_search(false, "No arrival airport found");
        return;
    }
    if (dn > 1 || an > 1) {
        ui_notify_route_search(true, "Select departure and arrival airports");
        return;
    }

    // Both resolved uniquely — fetch flights.
    char label[48] = {0};
    int rn = 0;
    memset(s_route_rows, 0, sizeof(s_route_rows));
    ui_set_status("Loading route flights...");
    net_begin_psram_safe();
    const bool ok =
        net_fetch_route_flights(dep_icao, arr_icao, s_route_rows, FR_MAX_BOARD_ROWS, &rn, label, sizeof(label));
    net_end_psram_safe();

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    memcpy(g_state.route_rows, s_route_rows, sizeof(s_route_rows));
    g_state.route_row_n = rn;
    strncpy(g_state.route_label, label[0] ? label : "", sizeof(g_state.route_label) - 1);
    g_state.route_dep_hit_n = 0;
    g_state.route_arr_hit_n = 0;
    xSemaphoreGive(g_state.mutex);

    if (ok) {
        char msg[48];
        snprintf(msg, sizeof(msg), "%d flights - select one to follow", rn);
        ui_notify_route_flights(true, msg);
    } else {
        ui_notify_route_flights(false, net_last_error()[0] ? net_last_error() : "No flights found");
    }
}

static void handle_fetch_route_flights() {
    char dep[8], arr[8];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(dep, g_state.route_dep_icao, sizeof(dep) - 1);
    dep[sizeof(dep) - 1] = 0;
    strncpy(arr, g_state.route_arr_icao, sizeof(arr) - 1);
    arr[sizeof(arr) - 1] = 0;
    g_state.route_row_n = 0;
    xSemaphoreGive(g_state.mutex);

    if (!dep[0] || !arr[0]) {
        ui_notify_route_flights(false, "Select departure and arrival airports");
        return;
    }

    static AirportBoardRow s_fetch_rows[FR_MAX_BOARD_ROWS];
    char label[48] = {0};
    int rn = 0;
    memset(s_fetch_rows, 0, sizeof(s_fetch_rows));
    ui_set_status("Loading route flights...");
    net_begin_psram_safe();
    const bool ok = net_fetch_route_flights(dep, arr, s_fetch_rows, FR_MAX_BOARD_ROWS, &rn, label, sizeof(label));
    net_end_psram_safe();

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    memcpy(g_state.route_rows, s_fetch_rows, sizeof(s_fetch_rows));
    g_state.route_row_n = rn;
    strncpy(g_state.route_label, label[0] ? label : "", sizeof(g_state.route_label) - 1);
    xSemaphoreGive(g_state.mutex);

    if (ok) {
        char msg[48];
        snprintf(msg, sizeof(msg), "%d flights - select one to follow", rn);
        ui_notify_route_flights(true, msg);
    } else {
        ui_notify_route_flights(false, net_last_error()[0] ? net_last_error() : "No flights found");
    }
}

static void handle_track_flight() {
    char query[96];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(query, g_state.pending_query, sizeof(query) - 1);
    query[sizeof(query) - 1] = 0;
    g_state.tracked_ok = false;
    memset(&g_state.tracked_ac, 0, sizeof(g_state.tracked_ac));
    xSemaphoreGive(g_state.mutex);

    char icao_cs[16] = {0}, iata_al[4] = {0}, iata_num[8] = {0};
    if (!net_parse_flight_number(query, icao_cs, sizeof(icao_cs), iata_al, sizeof(iata_al), iata_num,
                                 sizeof(iata_num))) {
        ui_notify_track_result(false, "Enter a flight like UA100 or UAL100");
        return;
    }

    ui_set_status(icao_cs[0] ? "Tracking flight..." : "Looking up flight...");
    Aircraft ac{};
    FlightInfo fi{};
    cpy(fi.flight, sizeof(fi.flight), icao_cs[0] ? icao_cs : query);

    char sched_key[16];
    if (iata_al[0] && iata_num[0]) {
        snprintf(sched_key, sizeof(sched_key), "%s%s", iata_al, iata_num);
    } else {
        cpy(sched_key, sizeof(sched_key), icao_cs);
    }

    net_begin_psram_safe();
    // After FA board / plan thrash, settle TLS before the first track HTTPS.
    net_tls_reset();
    delay(350);

    // Live ADS-B position FIRST — publish immediately so Track isn't stuck behind
    // FlightStats / FlightAware (common hang after airport board).
    int n = -1;
    if (icao_cs[0]) {
        n = net_fetch_aircraft_by_callsign(icao_cs, &ac);
    }
    if (n <= 0 && iata_al[0] && iata_num[0]) {
        char iata_cs[16];
        snprintf(iata_cs, sizeof(iata_cs), "%s%s", iata_al, iata_num);
        n = net_fetch_aircraft_by_callsign(iata_cs, &ac);
        if (n > 0 && !icao_cs[0]) {
            cpy(icao_cs, sizeof(icao_cs), iata_cs);
        }
    }

    bool published = false;
    auto publish_track = [&](const char *msg) {
        if (!ac.flight[0] && icao_cs[0]) {
            cpy(ac.flight, sizeof(ac.flight), icao_cs);
        }
        if (!fi.flight[0]) {
            cpy(fi.flight, sizeof(fi.flight), ac.flight[0] ? ac.flight : icao_cs);
        }
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.tracked_ac = ac;
        g_state.tracked_ok = true;
        g_state.flight_info = fi;
        if (!published) {
            g_state.flight_track_n = 0;
        }
        xSemaphoreGive(g_state.mutex);
        ui_notify_track_result(true, msg);
        ui_notify_flight_info();
        published = true;
    };

    if (n > 0) {
        // Airborne / ADS-B hit — open follow now; route + schedule fill in after.
        publish_track("Tracking");
    }

    auto push_fi = [&]() {
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.flight_info = fi;
        if (ac.hex[0]) {
            g_state.tracked_ac = ac;
        }
        xSemaphoreGive(g_state.mutex);
        ui_notify_flight_info();
    };

    // FlightStats next — gates/times used to be near-instant; do not wait on FA.
    if (!net_abort_requested()) {
        ui_set_status("Flight stats...");
        net_tls_reset();
        delay(300);
        net_fetch_flight_schedule(sched_key, &fi);
    }
    if (fi.origin[0] || fi.dest[0]) {
        fi.route_ok = fi.origin[0] && fi.dest[0];
        if (!fi.origin_name[0] || !fi.dest_name[0]) {
            net_fetch_airport_names(fi.origin, fi.dest, fi.origin_name, sizeof(fi.origin_name), fi.dest_name,
                                    sizeof(fi.dest_name));
        }
    }
    // HexDB OD when FS fails / leaves gaps — needed for dep–arr line + overview zoom.
    if ((!fi.origin[0] || !fi.dest[0]) && icao_cs[0] && !net_abort_requested()) {
        char o[8] = {0}, d[8] = {0};
        if (net_fetch_route(icao_cs, o, sizeof(o), d, sizeof(d))) {
            if (!fi.origin[0] && o[0]) {
                cpy(fi.origin, sizeof(fi.origin), o);
            }
            if (!fi.dest[0] && d[0]) {
                cpy(fi.dest, sizeof(fi.dest), d);
            }
        }
    }
    if ((!fi.origin[0] || !fi.dest[0]) && iata_al[0] && iata_num[0] && !net_abort_requested()) {
        char iata_cs[16];
        snprintf(iata_cs, sizeof(iata_cs), "%s%s", iata_al, iata_num);
        char o[8] = {0}, d[8] = {0};
        if (net_fetch_route(iata_cs, o, sizeof(o), d, sizeof(d))) {
            if (!fi.origin[0] && o[0]) {
                cpy(fi.origin, sizeof(fi.origin), o);
            }
            if (!fi.dest[0] && d[0]) {
                cpy(fi.dest, sizeof(fi.dest), d);
            }
        }
    }
    if (fi.origin[0] && fi.dest[0]) {
        fi.route_ok = true;
        if (!fi.origin_name[0] || !fi.dest_name[0]) {
            net_fetch_airport_names(fi.origin, fi.dest, fi.origin_name, sizeof(fi.origin_name), fi.dest_name,
                                    sizeof(fi.dest_name));
        }
    }
    if (published) {
        push_fi();  // dep/arr (or empty) on radar before FA — clears "Flight stats…"
        ui_set_status("Tracking");
    }

    if (n > 0 && ac.hex[0] && !fi.meta_ok) {
        char meta_reg[12] = {0};
        fi.meta_ok = net_fetch_aircraft_meta(ac.hex, fi.manufacturer, sizeof(fi.manufacturer), fi.type_name,
                                             sizeof(fi.type_name), fi.owner, sizeof(fi.owner), meta_reg,
                                             sizeof(meta_reg));
        cpy(fi.hex, sizeof(fi.hex), ac.hex);
        if (meta_reg[0] && !ac.reg[0]) {
            cpy(ac.reg, sizeof(ac.reg), meta_reg);
        }
        if (meta_reg[0]) {
            cpy(fi.reg, sizeof(fi.reg), meta_reg);
        }
    }

    // Not departed (or no ADS-B): pin to departing airport and keep FlightStats status.
    const bool pin_origin = fi.origin[0] && (n <= 0 || (fi.schedule_ok && !fi.departed));
    if (pin_origin) {
        float lat = 0, lon = 0;
        if (net_lookup_airport(fi.origin, &lat, &lon, nullptr) ||
            net_lookup_airport_iata(fi.origin, nullptr, 0, &lat, &lon)) {
            Aircraft ground = ac;
            if (n <= 0) {
                memset(&ground, 0, sizeof(ground));
            }
            if (!ground.flight[0]) {
                cpy(ground.flight, sizeof(ground.flight), icao_cs[0] ? icao_cs : sched_key);
            }
            ground.lat = lat;
            ground.lon = lon;
            ground.on_ground = true;
            ground.alt_ft = 0;
            ground.gs_kts = 0;
            ground.vrate_fpm = 0;
            ground.valid = true;
            if (n <= 0) {
                ground.hex[0] = 0;
            }
            ac = ground;
            n = 1;
        }
    }

    if (n <= 0) {
        net_end_psram_safe();
        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.flight_info = fi;
        g_state.tracked_ok = false;
        xSemaphoreGive(g_state.mutex);
        if (fi.schedule_ok) {
            ui_notify_flight_info();
            ui_notify_track_result(false, "Schedule found, but no airport position");
        } else {
            ui_notify_track_result(false, "Flight not found");
        }
        return;
    }

    if (!published) {
        if (fi.schedule_ok && !fi.departed) {
            char msg[64];
            if (fi.status[0]) {
                snprintf(msg, sizeof(msg), "%s @ %s", fi.status, fi.origin[0] ? fi.origin : "origin");
            } else {
                snprintf(msg, sizeof(msg), "At %s (not departed)", fi.origin[0] ? fi.origin : "origin");
            }
            publish_track(msg);
        } else {
            publish_track("Tracking");
        }
    } else {
        push_fi();
    }

    net_end_psram_safe();

    // Same as airport-board enrich: get onto radar with ADS-B + FlightStats first,
    // then defer FlightAware so the net task can keep polling ADS-B (FA used to
    // freeze the PPI on "Loading route…" / Collecting samples).
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    strncpy(g_state.pending_flight, icao_cs[0] ? icao_cs : sched_key, sizeof(g_state.pending_flight) - 1);
    g_state.pending_flight[sizeof(g_state.pending_flight) - 1] = 0;
    if (ac.reg[0]) {
        strncpy(g_state.pending_reg, ac.reg, sizeof(g_state.pending_reg) - 1);
        g_state.pending_reg[sizeof(g_state.pending_reg) - 1] = 0;
    }
    if (ac.hex[0]) {
        strncpy(g_state.pending_hex, ac.hex, sizeof(g_state.pending_hex) - 1);
        g_state.pending_hex[sizeof(g_state.pending_hex) - 1] = 0;
    }
    xSemaphoreGive(g_state.mutex);

    net_tls_reset();
    delay(350);
    handle_traffic();
    __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_REFRESH_TRAFFIC);
    // FA route + chart seed on the next net-loop turn (after ADS-B gets a slot).
    // Skip if Update ADS-B already asked to abort — keep the plane moving.
    if (!net_abort_requested() && (icao_cs[0] || (iata_al[0] && iata_num[0]))) {
        __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_FLIGHT_INFO);
    }
    ui_set_status("Tracking");
}

/** Update ADS-B companion: refresh FlightStats Sch/Act/Est into the followed flight.
 *  Non-empty incoming fields win; blanks leave whatever was already in flight_info. */
static void handle_refresh_schedule() {
    char flight[16] = {0};
    FlightInfo keep{};
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    const bool following = g_state.follow_active;
    if (following) {
        keep = g_state.flight_info;
        if (g_state.follow_flight[0]) {
            strncpy(flight, g_state.follow_flight, sizeof(flight) - 1);
        } else if (keep.flight[0]) {
            strncpy(flight, keep.flight, sizeof(flight) - 1);
        } else if (g_state.tracked_ok && g_state.tracked_ac.flight[0]) {
            strncpy(flight, g_state.tracked_ac.flight, sizeof(flight) - 1);
        }
    }
    xSemaphoreGive(g_state.mutex);
    if (!following || !flight[0]) {
        ui_set_status(following ? "No flight id for schedule" : "Tracking");
        return;
    }

    char icao_cs[16] = {0}, iata_al[4] = {0}, iata_num[8] = {0};
    char sched_key[16] = {0};
    if (net_parse_flight_number(flight, icao_cs, sizeof(icao_cs), iata_al, sizeof(iata_al), iata_num,
                                sizeof(iata_num))) {
        if (iata_al[0] && iata_num[0]) {
            snprintf(sched_key, sizeof(sched_key), "%s%s", iata_al, iata_num);
        } else {
            strncpy(sched_key, icao_cs[0] ? icao_cs : flight, sizeof(sched_key) - 1);
        }
    } else {
        strncpy(sched_key, flight, sizeof(sched_key) - 1);
    }
    if (!sched_key[0]) {
        ui_set_status("Bad flight id");
        return;
    }

    if (net_abort_requested()) {
        ui_set_status("Tracking");
        return;
    }
    ui_set_status("Updating schedule…");
    FlightInfo neu{};
    strncpy(neu.flight, icao_cs[0] ? icao_cs : flight, sizeof(neu.flight) - 1);
    if (keep.hex[0]) {
        strncpy(neu.hex, keep.hex, sizeof(neu.hex) - 1);
    }
    if (keep.reg[0]) {
        strncpy(neu.reg, keep.reg, sizeof(neu.reg) - 1);
    }

    net_begin_psram_safe();
    net_tls_reset();
    delay(200);
    bool ok = false;
    if (!net_abort_requested()) {
        // Fast path: one FS page only (no otherDays refetch) so Update can't hang.
        ok = net_fetch_flight_schedule(sched_key, &neu, false);
    }
    // OD fallback when FS fails — still get dep/arr airports for the line / panel.
    if ((!neu.origin[0] || !neu.dest[0]) && icao_cs[0] && !net_abort_requested()) {
        char o[8] = {0}, d[8] = {0};
        if (net_fetch_route(icao_cs, o, sizeof(o), d, sizeof(d))) {
            if (!neu.origin[0] && o[0]) {
                strncpy(neu.origin, o, sizeof(neu.origin) - 1);
            }
            if (!neu.dest[0] && d[0]) {
                strncpy(neu.dest, d, sizeof(neu.dest) - 1);
            }
        }
    }
    if ((neu.origin[0] || neu.dest[0]) && (!neu.origin_name[0] || !neu.dest_name[0]) &&
        !net_abort_requested()) {
        net_fetch_airport_names(neu.origin, neu.dest, neu.origin_name, sizeof(neu.origin_name), neu.dest_name,
                                sizeof(neu.dest_name));
    }
    net_end_psram_safe();
    if (net_abort_requested()) {
        ui_set_status("Tracking");
        return;
    }

    auto take_str = [](char *dst, size_t n, const char *src) {
        if (src && src[0]) {
            strncpy(dst, src, n - 1);
            dst[n - 1] = 0;
        }
    };
    // Merge: fill/replace only when FlightStats / hexdb returned a value.
    take_str(keep.origin, sizeof(keep.origin), neu.origin);
    take_str(keep.origin_name, sizeof(keep.origin_name), neu.origin_name);
    take_str(keep.dest, sizeof(keep.dest), neu.dest);
    take_str(keep.dest_name, sizeof(keep.dest_name), neu.dest_name);
    take_str(keep.dep_gate, sizeof(keep.dep_gate), neu.dep_gate);
    take_str(keep.dep_terminal, sizeof(keep.dep_terminal), neu.dep_terminal);
    take_str(keep.arr_gate, sizeof(keep.arr_gate), neu.arr_gate);
    take_str(keep.arr_terminal, sizeof(keep.arr_terminal), neu.arr_terminal);
    take_str(keep.dep_time, sizeof(keep.dep_time), neu.dep_time);
    take_str(keep.dep_act, sizeof(keep.dep_act), neu.dep_act);
    take_str(keep.dep_est, sizeof(keep.dep_est), neu.dep_est);
    take_str(keep.arr_time, sizeof(keep.arr_time), neu.arr_time);
    take_str(keep.arr_act, sizeof(keep.arr_act), neu.arr_act);
    take_str(keep.arr_est, sizeof(keep.arr_est), neu.arr_est);
    take_str(keep.status, sizeof(keep.status), neu.status);
    take_str(keep.flight_date, sizeof(keep.flight_date), neu.flight_date);
    if (neu.dep_act_epoch) {
        keep.dep_act_epoch = neu.dep_act_epoch;
    }
    if (neu.arr_act_epoch) {
        keep.arr_act_epoch = neu.arr_act_epoch;
    }
    if (neu.arr_est_epoch) {
        keep.arr_est_epoch = neu.arr_est_epoch;
    }
    if (neu.schedule_ok) {
        keep.schedule_ok = true;
    }
    if (neu.departed) {
        keep.departed = true;
    }
    if (keep.origin[0] && keep.dest[0]) {
        keep.route_ok = true;
    }
    if (neu.flight[0]) {
        take_str(keep.flight, sizeof(keep.flight), neu.flight);
    }
    if (!keep.flight[0]) {
        strncpy(keep.flight, icao_cs[0] ? icao_cs : flight, sizeof(keep.flight) - 1);
        keep.flight[sizeof(keep.flight) - 1] = 0;
    }

    const bool got_dep_arr = keep.origin[0] || keep.dest[0] || keep.dep_time[0] || keep.arr_time[0] ||
                             keep.dep_act[0] || keep.arr_est[0] || keep.dep_gate[0] || keep.arr_gate[0];
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.flight_info = keep;
    xSemaphoreGive(g_state.mutex);
    ui_notify_flight_info();
    delay(450);  // beat ui_set_status LVGL throttle so the result replaces "Updating schedule…"
    if (ok) {
        ui_set_status("Schedule updated");
    } else if (got_dep_arr) {
        ui_set_status("Airports updated");
    } else {
        ui_set_status("Schedule refresh failed");
    }
}

static void net_task(void *arg) {
    LV_UNUSED(arg);
    uint32_t last_traffic = millis();
    vTaskDelay(pdMS_TO_TICKS(2500));
    for (;;) {
        uint32_t req = take_requests();
        // Apply zoom/NVS before long HTTPS so range changes are not stuck behind ADS-B.
        if (req & UI_REQ_SAVE_RANGE) {
            handle_save_range();
        }
        if (req & UI_REQ_SAVE_MAX_ALT) {
            handle_save_max_alt();
        }
        if (req & UI_REQ_SAVE_SHOW_AIRPORTS) {
            handle_save_show_airports();
        }
        if (req & UI_REQ_SAVE_SHOW_COUNTIES) {
            handle_save_show_counties();
        }
        if (req & UI_REQ_SAVE_SHOW_CITIES) {
            handle_save_show_cities();
        }
        if (req & UI_REQ_SAVE_COUNTY_MASK) {
            handle_save_county_mask();
        }
        if (req & UI_REQ_SAVE_BRIGHTNESS) {
            handle_save_brightness();
        }
        if (req & UI_REQ_SAVE_VOLUME) {
            handle_save_volume();
        }
        if (req & UI_REQ_SAVE_LOCATION) {
            handle_save_location();
        }
        if (req & UI_REQ_WIFI_SCAN) {
            handle_wifi_scan();
        }
        if (req & UI_REQ_WIFI_CONNECT) {
            handle_wifi_connect();
        }
        if (req & UI_REQ_GEOCODE) {
            handle_geocode();
        }
        // Update ADS-B: clear wedged TLS/HTTP before the traffic poll.
        if (req & UI_REQ_NET_RECOVER) {
            ui_set_status("Resetting network…");
            net_stack_recover();  // also clears net_abort so ADS-B/schedule can run
            ui_set_status("Updating ADS-B…");
        }
        // Traffic before airports so the PPI paints flights as soon as ADS-B answers.
        if (req & UI_REQ_REFRESH_TRAFFIC) {
            handle_traffic();
            last_traffic = millis();
            if (req & UI_REQ_NET_RECOVER) {
                ui_set_status("ADS-B updated");
            }
        }
        if (req & UI_REQ_REFRESH_AIRPORTS) {
            handle_airports();
        }
        // After ADS-B recover/poll: refresh Sch/Act/Est for the followed flight.
        if (req & UI_REQ_REFRESH_SCHEDULE) {
            handle_refresh_schedule();
        }
        // Never leave FETCH_FLIGHT_INFO queued across a recover — it re-locks Loading route.
        if ((req & UI_REQ_NET_RECOVER) && (req & UI_REQ_FETCH_FLIGHT_INFO)) {
            req &= ~UI_REQ_FETCH_FLIGHT_INFO;
        }
        if (req & UI_REQ_FETCH_WEATHER) {
            handle_weather();
        }
        // Airport board BEFORE track FA enrich — combining them put FlightAware
        // flight-plan scrapes ahead of live/airport and wedged Hosted DNS (-54).
        if (req & UI_REQ_FETCH_AIRPORT_BOARD) {
            // New board tap wins over a queued track start this tick.
            if (req & UI_REQ_TRACK_FLIGHT) {
                __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_TRACK_FLIGHT);
            }
            // Drop pending FA route scrape; board needs a clean TLS session.
            if (req & UI_REQ_FETCH_FLIGHT_INFO) {
                req &= ~UI_REQ_FETCH_FLIGHT_INFO;
            }
            handle_airport_board();
        }
        if (req & UI_REQ_TRACK_FLIGHT) {
            // Defer track while board owns this tick (re-queued above if both set).
            if (!(req & UI_REQ_FETCH_AIRPORT_BOARD)) {
                handle_track_flight();
            }
        }
        if (req & UI_REQ_FETCH_FLIGHT_INFO) {
            // Never run heavy FA plan while a board fetch is outstanding.
            if (req & UI_REQ_FETCH_AIRPORT_BOARD) {
                __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_FLIGHT_INFO);
            } else {
                // Skip only when Track already loaded THIS flight's schedule+route.
                bool skip = false;
                xSemaphoreTake(g_state.mutex, portMAX_DELAY);
                if (g_state.follow_active && g_state.flight_info.schedule_ok &&
                    (g_state.flight_info.route_wpt_n > 0 || g_state.flight_track_n > 0)) {
                    const char *pf = g_state.pending_flight;
                    const char *ph = g_state.pending_hex;
                    const char *ff = g_state.flight_info.flight;
                    const char *fh = g_state.flight_info.hex;
                    const bool hex_same = ph[0] && fh[0] && strcasecmp(ph, fh) == 0;
                    const bool flt_same = pf[0] && ff[0] && strcasecmp(pf, ff) == 0;
                    skip = hex_same || flt_same;
                }
                xSemaphoreGive(g_state.mutex);
                if (!skip) {
                    handle_flight_info();
                }
            }
        }
        if (req & UI_REQ_SEARCH_AIRPORTS) {
            handle_search_airports();
        }
        if (req & UI_REQ_REFRESH_PLATE_CITIES) {
            handle_refresh_plate_cities();
        }
        if (req & UI_REQ_FETCH_PLATE_RADIOS) {
            // Don't steal TLS while FA board / track is in this tick or newly queued.
            const uint32_t busy = g_state.requests;
            if ((req | busy) & (UI_REQ_FETCH_AIRPORT_BOARD | UI_REQ_TRACK_FLIGHT |
                                UI_REQ_FETCH_FLIGHT_INFO | UI_REQ_FETCH_ROUTE_FLIGHTS)) {
                __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_PLATE_RADIOS);
            } else {
                char icao[8] = {0};
                xSemaphoreTake(g_state.mutex, portMAX_DELAY);
                strncpy(icao, g_state.pending_plate_radios_icao, sizeof(icao) - 1);
                xSemaphoreGive(g_state.mutex);
                AirportRadios radios{};
                const bool ok = icao[0] && net_lookup_airport_radios(icao, &radios);
                xSemaphoreTake(g_state.mutex, portMAX_DELAY);
                if (ok && strcasecmp(icao, g_state.pending_plate_radios_icao) == 0) {
                    g_state.plate_radios = radios;
                    g_state.plate_radios_ok = true;
                } else {
                    g_state.plate_radios_ok = false;
                }
                xSemaphoreGive(g_state.mutex);
                ui_notify_plate_radios(ok);
            }
        }
        if (req & UI_REQ_ROUTE_SEARCH) {
            handle_route_search();
        }
        if (req & UI_REQ_FETCH_ROUTE_FLIGHTS) {
            handle_fetch_route_flights();
        }
        if (req & UI_REQ_UPDATE_PLATES) {
            // Install from SPIFFS first; only missing ICAOs hit GitHub plates/*.frpl.
            ui_set_status("Updating plates...");
            net_abort_clear();
            net_host_gates_clear();
            net_tls_reset();
            delay(80);
            char icaos[48][8];
            int n = 0;
            for (int i = 0; i < AIRPORT_PLATE_COUNT && n < 48; i++) {
                strncpy(icaos[n], kAirportPlates[i].icao, 7);
                icaos[n][7] = 0;
                n++;
            }
            xSemaphoreTake(g_state.mutex, portMAX_DELAY);
            g_state.plate_pack_n = 0;
            if (n > 0) {
                memcpy(g_state.plate_pack_icao, icaos, sizeof(char[8]) * (size_t)n);
                g_state.plate_pack_n = n;
            }
            xSemaphoreGive(g_state.mutex);
            {
                char q[64];
                snprintf(q, sizeof(q), "Queue %d (local first)", n);
                ui_plates_log_line("", q);
            }
            faa_plates_queue_pack_dir();
            if (n > 0) {
                ui_notify_plate_download(icaos[0], 0, 1, n);
            }
            __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_PLATE);
        }
        // Plate downloads ONLY while Update Plates left FETCH_PLATE set — never auto-grab.
        if (req & UI_REQ_FETCH_PLATE) {
            if (net_abort_requested()) {
                faa_plates_cancel();
                net_tls_reset();
                net_abort_clear();
                ui_plates_log_line("", "Cancelled");
                ui_set_status("Plate downloads cancelled");
            } else {
                const uint32_t busy = g_state.requests;
                const uint32_t defer_mask = UI_REQ_FETCH_AIRPORT_BOARD | UI_REQ_TRACK_FLIGHT |
                                            UI_REQ_FETCH_FLIGHT_INFO | UI_REQ_FETCH_ROUTE_FLIGHTS |
                                            UI_REQ_NET_RECOVER;
                if ((req | busy) & defer_mask) {
                    __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_PLATE);
                } else if (faa_plates_pending()) {
                    net_begin_psram_safe();
                    const bool got = faa_plates_service();
                    net_end_psram_safe();
                    if (net_abort_requested()) {
                        faa_plates_cancel();
                        net_tls_reset();
                        net_abort_clear();
                        ui_plates_log_line("", "Cancelled");
                        ui_set_status("Plate downloads cancelled");
                    } else {
                        if (got) {
                            ui_notify_plate_bitmap();
                        }
                        if (faa_plates_pending()) {
                            __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_FETCH_PLATE);
                        } else if (faa_plates_bulk_finished()) {
                            int ok_n = 0, fail_n = 0, total_n = 0;
                            faa_plates_bulk_stats(&ok_n, &fail_n, &total_n);
                            char done[96];
                            if (fail_n <= 0) {
                                snprintf(done, sizeof(done), "Plates updated - %d/%d OK", ok_n, total_n);
                                ui_notify_plates_updated(true, done);
                            } else if (ok_n > 0) {
                                snprintf(done, sizeof(done), "Plates partial - %d OK, %d FAIL", ok_n,
                                         fail_n);
                                ui_notify_plates_updated(true, done);
                            } else {
                                snprintf(done, sizeof(done), "Plate update failed - 0/%d OK", total_n);
                                ui_notify_plates_updated(false, done);
                            }
                            ui_set_status(done);
                            last_traffic = millis();  // give ADS-B a beat before the next poll
                        }
                    }
                }
            }
        }
        // Wait until the radar screen is live so the first successful ADS-B
        // response is painted (fetching during the Wi-Fi splash skipped redraw).
        // Pause ADS-B while Update Plates owns TLS — concurrent polls caused plate TLS fails.
        if (g_state.radar_live && g_state.wifi_connected && g_state.cfg.location.valid &&
            !faa_plates_bulk_active() && !faa_plates_pending()) {
            // Retry quickly after a failed boot fetch; otherwise every 15s.
            const uint32_t interval_ms = g_state.traffic_ok ? 15000u : 3000u;
            if (millis() - last_traffic > interval_ms) {
                handle_traffic();
                last_traffic = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#ifdef BOARD_CROWPANEL_P4
static bool crowpanel_enable_panel_power() {
    // MIPI D-PHY needs LDO3 @ 2.5V; touch/IO pullups need LDO4 @ 3.3V (Elecrow docs).
    esp_ldo_channel_handle_t ldo3 = nullptr;
    esp_ldo_channel_handle_t ldo4 = nullptr;
    esp_ldo_channel_config_t ldo3_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    esp_ldo_channel_config_t ldo4_cfg = {
        .chan_id = 4,
        .voltage_mv = 3300,
    };
    if (esp_ldo_acquire_channel(&ldo3_cfg, &ldo3) != ESP_OK) {
        Serial.println("LDO3 2.5V failed");
        return false;
    }
    if (esp_ldo_acquire_channel(&ldo4_cfg, &ldo4) != ESP_OK) {
        Serial.println("LDO4 3.3V failed");
        return false;
    }
    // Active-low panel power rail (ESPHome CrowPanel V1.2).
    pinMode(LCD_GPIO_POWER, OUTPUT);
    digitalWrite(LCD_GPIO_POWER, LOW);
    delay(20);
    return true;
}
#endif

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Flight Radar starting");

    memset(&g_state, 0, sizeof(g_state));
    g_state.mutex = xSemaphoreCreateMutex();
    config_load(&g_state.cfg);
    // Defer HTTP buffer alloc until after WiFi — pre-boot DMA/PSRAM games broke
    // ESP-Hosted (Req_WifiInit timeout).

#ifdef BOARD_CROWPANEL_P4
    if (!crowpanel_enable_panel_power()) {
        Serial.println("Panel power init failed");
        while (true) {
            delay(1000);
        }
    }
#endif

    g_board = new Board();
    g_board->init();
    auto lcd = g_board->getLCD();
    if (lcd != nullptr) {
#if LVGL_PORT_AVOID_TEARING_MODE
        lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#endif
        auto lcd_bus = lcd->getBus();
        if (lcd_bus != nullptr && lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 20);
        }
    }
    if (!g_board->begin()) {
        Serial.println("Board init failed");
        while (true) {
            delay(1000);
        }
    }
    if (auto *bl = g_board->getBacklight()) {
        bl->on();
        board_set_brightness(g_state.cfg.brightness);
    }

    audio_set_volume(g_state.cfg.volume);  // default 50% on quieter scale
    audio_init();

#ifndef BOARD_CROWPANEL_P4
    if (auto *exp = g_board->getIO_Expander()) {
        auto *ch422 = static_cast<esp_expander::CH422G *>(exp->getBase());
        ch422->enableAllIO_Output();
        ch422->digitalWrite(2, 1);
    }
#endif

    lvgl_port_init(g_board->getLCD(), g_board->getTouch());
    lvgl_port_lock(-1);
    ui_init(&g_state);
    lvgl_port_unlock();

    // Splash first (display-only). Wi-Fi starts after so STA join is not disrupted.
    ui_wait_splash(10000);  // 10s, or tap splash to skip

    // Leave internal heap alone for ESP-Hosted WiFi + TLS. Do NOT call
    // heap_caps_malloc_extmem_enable() — that diverted small allocs to PSRAM and
    // WiFi init failed with Req_WifiInit timeout.
    WiFi.setSleep(false);
    net_init_buffers();
    net_set_status_callback(ui_set_status);
    // Hard recover / TLS path can rejoin STA without calling WiFi.config mid-flight.
    net_set_wifi_rejoin_callback([](uint32_t timeout_ms) -> bool {
        if (!g_state.cfg.wifi_ssid[0]) {
            return false;
        }
        const bool ok = wifi_connect_sta(g_state.cfg.wifi_ssid, g_state.cfg.wifi_pass, timeout_ms);
        g_state.wifi_connected = ok;
        return ok;
    });
    xTaskCreatePinnedToCore(net_task, "net", 32768, nullptr, 1, nullptr, 0);

    if (g_state.cfg.wifi_ssid[0]) {
        // Show Wi-Fi panel + joining popup, then unlock so LVGL can animate while STA joins.
        lvgl_port_lock(-1);
        ui_show_wifi();
        ui_show_wifi_joining(g_state.cfg.wifi_ssid);
        lvgl_port_unlock();
        {
            char st[96];
            snprintf(st, sizeof(st), "Joining \"%s\" network", g_state.cfg.wifi_ssid);
            ui_set_status(st);
        }
        const bool ok = wifi_connect_sta(g_state.cfg.wifi_ssid, g_state.cfg.wifi_pass, 15000);
        g_state.wifi_connected = ok;
        lvgl_port_lock(-1);
        ui_hide_wifi_joining();
        lvgl_port_unlock();
        if (ok) {
            char msg[96];
            snprintf(msg, sizeof(msg), "Joined %s", g_state.cfg.wifi_ssid);
            if (g_state.cfg.location.valid) {
                // Existing airport homes may still be a long address — collapse to ICAO.
                if (net_enrich_airport_home_label(&g_state.cfg.location)) {
                    config_save_location(&g_state.cfg.location);
                }
                // ui_show_radar marks radar_live and requests ADS-B + airports.
                lvgl_port_lock(-1);
                ui_notify_home_label(g_state.cfg.location.icao[0] ? g_state.cfg.location.icao
                                                                  : g_state.cfg.location.label);
                ui_show_radar();
                lvgl_port_unlock();
            } else {
                lvgl_port_lock(-1);
                ui_show_location();
                lvgl_port_unlock();
            }
            ui_set_status(msg);
        } else {
            lvgl_port_lock(-1);
            ui_show_wifi();
            lvgl_port_unlock();
            __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_WIFI_SCAN);
        }
    } else {
        lvgl_port_lock(-1);
        ui_show_wifi();
        lvgl_port_unlock();
        __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_WIFI_SCAN);
    }
}

void loop() {
    delay(1000);
}
