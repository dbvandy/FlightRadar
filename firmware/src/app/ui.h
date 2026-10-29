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
    UI_REQ_SAVE_SHOW_COUNTIES = 1u << 11,
    UI_REQ_SAVE_SHOW_CITIES = 1u << 12,
    UI_REQ_SAVE_COUNTY_MASK = 1u << 13,
    UI_REQ_SAVE_BRIGHTNESS = 1u << 14,
    UI_REQ_SAVE_VOLUME = 1u << 15,
    UI_REQ_SAVE_EMAIL = 1u << 16,
    UI_REQ_EMAIL_SCREENSHOT = 1u << 17,  // net task emails screenshot_bmp (BMP or AVI)
    UI_REQ_TRACK_FLIGHT = 1u << 18,
    UI_REQ_SEARCH_AIRPORTS = 1u << 19,
    UI_REQ_FETCH_AIRPORT_BOARD = 1u << 20,
    UI_REQ_REFRESH_PLATE_CITIES = 1u << 21,
    UI_REQ_ROUTE_SEARCH = 1u << 22,
    UI_REQ_FETCH_ROUTE_FLIGHTS = 1u << 23,
    UI_REQ_UPDATE_SD_DATA = 1u << 24,  // download plates+DBs from GitHub -> SD
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
    char pending_board_icao[8];
    // User plate city refresh (UI -> net -> ui_notify_plate_cities).
    char plate_city_icao[24][8];
    char plate_city_name[24][28];
    int plate_city_n;
    // Airport arrivals / departures board.
    char board_icao[8];
    char board_label[48];
    float board_lat;  // airport LL for local clock (0 if unknown)
    float board_lon;
    bool board_ll_ok;
    AirportBoardRow board_arr[FR_MAX_BOARD_ROWS];
    AirportBoardRow board_dep[FR_MAX_BOARD_ROWS];
    int board_arr_n;
    int board_dep_n;
    AptSearchHit apt_hits[FR_MAX_APT_SEARCH];
    int apt_hit_n;
    // Route-flights search.
    char pending_route_dep[48];
    char pending_route_arr[48];
    char route_dep_icao[8];
    char route_arr_icao[8];
    char route_label[48];
    AirportBoardRow route_rows[FR_MAX_BOARD_ROWS];
    int route_row_n;
    AptSearchHit route_dep_hits[FR_MAX_APT_SEARCH];
    int route_dep_hit_n;
    AptSearchHit route_arr_hits[FR_MAX_APT_SEARCH];
    int route_arr_hit_n;
    FlightInfo flight_info;
    FlightTrackPt flight_track[FR_MAX_TRACK_PTS];  // FA alt/gs history for follow chart
    int flight_track_n;
    // Screenshot / timelapse email: buffer owned by UI until net task frees via
    // ui_release_screenshot_bmp(). filename selects mime (radar.bmp / *.avi).
    uint8_t *screenshot_bmp;
    size_t screenshot_bmp_len;
    char screenshot_filename[40];
    bool screenshot_busy;
    Aircraft tracked_ac;
    bool tracked_ok;
    // Follow mode (UI -> net task): solo = ADS-B for tracked only; else nearby list.
    // hold_center = user panned/Home - fetch traffic around map center, not solo-only.
    bool follow_active;
    bool follow_solo;
    bool follow_hold_center;
    char follow_hex[8];
    char follow_flight[12];
    bool wifi_connected;
    bool wifi_connecting;
    bool geo_searching;
    bool traffic_ok;
    uint32_t last_traffic_ms;
    // Bumped when leaving follow (Home/Unfollow) so an in-flight solo ADS-B
    // response cannot overwrite the full local traffic list.
    volatile uint32_t traffic_epoch;
    // Set once the radar screen is shown - delays boot ADS-B until PPI can paint.
    bool radar_live;
    volatile uint32_t requests;
};

void ui_init(SharedState *state);
/** Full-screen splash (display-only). Call before Wi-Fi work. */
void ui_show_splash();
/** Show splash and hold for ms (LVGL keeps refreshing). */
void ui_wait_splash(uint32_t ms);
/** Mount SD and load fr_data DBs into PSRAM (call outside LVGL lock). */
void ui_load_sd_databases();
void ui_show_wifi();
void ui_show_wifi_joining(const char *ssid);
void ui_hide_wifi_joining();
void ui_show_location();
void ui_show_radar();
void ui_notify_wifi_scan_done();
void ui_notify_wifi_result(bool ok, const char *msg);
void ui_notify_geo_done();
void ui_notify_home_label(const char *label);
void ui_notify_traffic();
void ui_notify_traffic_labels();
void ui_schedule_radar_redraw(uint32_t delay_ms);
void ui_notify_airports();
void ui_notify_weather();
void ui_notify_flight_info();
void ui_notify_airport_search(bool ok, const char *msg);
void ui_notify_airport_board(bool ok, const char *msg);
/** Update Airport Board status label (safe from net task; takes LVGL lock). */
void ui_set_board_status(const char *msg);
void ui_notify_plate_cities(bool ok);
void ui_notify_route_search(bool ok, const char *msg);
void ui_notify_route_flights(bool ok, const char *msg);
void ui_notify_track_result(bool ok, const char *msg);
void ui_notify_email_result(bool ok, const char *msg);
void ui_notify_sd_data_result(bool ok, const char *msg);
/** Free screenshot BMP after net task finishes (call under LVGL lock or from UI notify). */
void ui_release_screenshot_bmp();
void ui_set_status(const char *msg);
