#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>

#include "lvgl_v8_port.h"
#include "app/config_store.h"
#include "app/net_client.h"
#include "app/ui.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

static SharedState g_state;
static Board *g_board = nullptr;

static uint32_t take_requests() {
    return __sync_fetch_and_and((uint32_t *)&g_state.requests, 0u);
}

static void set_status_unlocked(const char *msg) {
    strncpy(g_state.status, msg, sizeof(g_state.status) - 1);
    g_state.status[sizeof(g_state.status) - 1] = 0;
}

static bool wifi_connect_sta(const char *ssid, const char *pass, uint32_t timeout_ms) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("flight-radar");
    WiFi.begin(ssid, pass);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

static void handle_wifi_scan() {
    ui_set_status("Scanning Wi-Fi...");
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

static Aircraft g_traffic_tmp[FR_MAX_AIRCRAFT];
static Airport g_airport_tmp[FR_MAX_AIRPORTS];

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

static void handle_save_location() {
    GeoFix fix{};
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    fix = g_state.cfg.location;
    xSemaphoreGive(g_state.mutex);
    lvgl_port_lock(-1);
    config_save_location(&fix);
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

static void handle_traffic() {
    float lat, lon, range;
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    lat = g_state.cfg.location.lat;
    lon = g_state.cfg.location.lon;
    range = g_state.cfg.range_nm;
    xSemaphoreGive(g_state.mutex);
    if (!g_state.cfg.location.valid) {
        return;
    }
    ui_set_status("Updating ADS-B...");
    Aircraft *tmp = g_traffic_tmp;
    net_begin_psram_safe();
    const int n = net_fetch_aircraft(lat, lon, range, tmp, FR_MAX_AIRCRAFT);
    net_end_psram_safe();
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    if (n >= 0) {
        memcpy(g_state.aircraft, tmp, sizeof(Aircraft) * n);
        g_state.aircraft_count = n;
        g_state.traffic_ok = true;
        g_state.last_traffic_ms = millis();
        snprintf(g_state.status, sizeof(g_state.status), "ADS-B live  %d ac", n);
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
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    lat = g_state.cfg.location.lat;
    lon = g_state.cfg.location.lon;
    range = g_state.cfg.range_nm;
    xSemaphoreGive(g_state.mutex);
    if (!g_state.cfg.location.valid) {
        return;
    }
    ui_set_status("Loading airports...");
    Airport *tmp = g_airport_tmp;
    net_begin_psram_safe();
    const int n = net_fetch_airports(lat, lon, range, tmp, FR_MAX_AIRPORTS);
    net_end_psram_safe();
    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    if (n >= 0) {
        memcpy(g_state.airports, tmp, sizeof(Airport) * n);
        g_state.airport_count = n;
        snprintf(g_state.status, sizeof(g_state.status), "%d airports in view", n);
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
    ui_set_status(flight[0] ? "Looking up flight..." : "Looking up aircraft...");
    net_begin_psram_safe();
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
    strncpy(fi.reg, reg, sizeof(fi.reg) - 1);
    // Dep/Arr from flight number/callsign; fall back to registration/tail if needed.
    if (flight[0]) {
        fi.route_ok = net_fetch_route(flight, fi.origin, sizeof(fi.origin), fi.dest, sizeof(fi.dest));
    }
    if (!fi.route_ok && reg[0]) {
        fi.route_ok = net_fetch_route(reg, fi.origin, sizeof(fi.origin), fi.dest, sizeof(fi.dest));
    }
    if (fi.route_ok) {
        net_fetch_airport_names(fi.origin, fi.dest, fi.origin_name, sizeof(fi.origin_name), fi.dest_name,
                                sizeof(fi.dest_name));
    }
    net_end_psram_safe();

    xSemaphoreTake(g_state.mutex, portMAX_DELAY);
    g_state.flight_info = fi;
    xSemaphoreGive(g_state.mutex);
    ui_notify_flight_info();
    if (fi.route_ok) {
        ui_set_status("Route ready");
    } else if (fi.meta_ok) {
        ui_set_status("Aircraft info ready");
    } else {
        ui_set_status("Route unavailable");
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
        if (req & UI_REQ_REFRESH_AIRPORTS) {
            handle_airports();
        }
        if (req & UI_REQ_REFRESH_TRAFFIC) {
            handle_traffic();
            last_traffic = millis();
        }
        if (req & UI_REQ_FETCH_WEATHER) {
            handle_weather();
        }
        if (req & UI_REQ_FETCH_FLIGHT_INFO) {
            handle_flight_info();
        }
        if (g_state.wifi_connected && g_state.cfg.location.valid &&
            millis() - last_traffic > 15000) {
            handle_traffic();
            last_traffic = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Flight Radar starting");

    memset(&g_state, 0, sizeof(g_state));
    g_state.mutex = xSemaphoreCreateMutex();
    config_load(&g_state.cfg);
    // Grab DRAM for ADS-B before RGB bounce + LVGL eat internal heap.
    net_init_buffers();

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
    }
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

    WiFi.setSleep(false);
    heap_caps_malloc_extmem_enable(64);
    net_set_status_callback(ui_set_status);
    xTaskCreatePinnedToCore(net_task, "net", 20480, nullptr, 1, nullptr, 0);

    if (g_state.cfg.wifi_ssid[0]) {
        lvgl_port_lock(-1);
        ui_show_wifi();
        lvgl_port_unlock();
        ui_set_status("Connecting to saved Wi-Fi...");
        const bool ok = wifi_connect_sta(g_state.cfg.wifi_ssid, g_state.cfg.wifi_pass, 15000);
        g_state.wifi_connected = ok;
        if (ok) {
            char msg[96];
            snprintf(msg, sizeof(msg), "Joined %s", g_state.cfg.wifi_ssid);
            if (g_state.cfg.location.valid) {
                lvgl_port_lock(-1);
                ui_show_radar();
                lvgl_port_unlock();
                __sync_fetch_and_or((uint32_t *)&g_state.requests, UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
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
