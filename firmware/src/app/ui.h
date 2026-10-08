#pragma once

#include "types.h"

enum UiRequest : uint32_t {
    UI_REQ_WIFI_SCAN = 1u << 0,
    UI_REQ_WIFI_CONNECT = 1u << 1,
    UI_REQ_GEOCODE = 1u << 2,
    UI_REQ_REFRESH_TRAFFIC = 1u << 3,
    UI_REQ_REFRESH_AIRPORTS = 1u << 4,
    UI_REQ_FETCH_WEATHER = 1u << 5,
    UI_REQ_SAVE_RANGE = 1u << 6,
    UI_REQ_SAVE_LOCATION = 1u << 7,
    UI_REQ_SAVE_MAX_ALT = 1u << 8,
    UI_REQ_SAVE_SHOW_AIRPORTS = 1u << 9,
    UI_REQ_FETCH_FLIGHT_INFO = 1u << 10,
};

struct SharedState {
    SemaphoreHandle_t mutex;
    AppConfig cfg;
    Aircraft aircraft[FR_MAX_AIRCRAFT];
    int aircraft_count;
    Airport airports[FR_MAX_AIRPORTS];
    int airport_count;
    Weather weather;
    GeoFix geo_results[8];
    int geo_count;
    char wifi_names[FR_MAX_WIFI][33];
    int wifi_rssi[FR_MAX_WIFI];
    int wifi_count;
    char status[96];
    char pending_ssid[33];
    char pending_pass[65];
    char pending_query[96];
    char pending_icao[8];
    char pending_flight[12];
    char pending_reg[12];
    char pending_hex[8];
    FlightInfo flight_info;
    bool wifi_connected;
    bool wifi_connecting;
    bool geo_searching;
    bool traffic_ok;
    uint32_t last_traffic_ms;
    volatile uint32_t requests;
};

void ui_init(SharedState *state);
void ui_show_wifi();
void ui_show_location();
void ui_show_radar();
void ui_notify_wifi_scan_done();
void ui_notify_wifi_result(bool ok, const char *msg);
void ui_notify_geo_done();
void ui_notify_traffic();
void ui_notify_traffic_labels();
void ui_schedule_radar_redraw(uint32_t delay_ms);
void ui_notify_airports();
void ui_notify_weather();
void ui_notify_flight_info();
void ui_set_status(const char *msg);
