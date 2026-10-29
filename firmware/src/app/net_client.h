#pragma once

#include "types.h"

// Call once early in setup() before RGB/LVGL init so the body buffer lands in DRAM.
void net_init_buffers();
/** Drop any sticky TLS session (call before a heavy FA fetch / after abort). */
void net_tls_reset();

// body may be nullptr to keep the payload only in the internal fixed buffer.
// early_ok: optional stop-as-soon-as-usable predicate (airport board markers, etc.).
bool https_get(const char *url, String *body, const char *user_agent = nullptr,
               uint32_t connect_ms = 8000, uint32_t io_ms = 10000, const char *accept = nullptr,
               bool force_close = false, bool (*early_ok)(const char *buf, size_t len) = nullptr);
const char *net_last_error();
void net_set_status_callback(void (*cb)(const char *msg));

int net_geocode(const char *query, GeoFix *out, size_t out_max);
/** Airport homes: rewrite label to "CODE - City". Returns true if label changed. */
bool net_enrich_airport_home_label(GeoFix *fix);

int net_fetch_aircraft(float lat, float lon, float range_nm, Aircraft *out, size_t out_max);
/** Solo ADS-B lookups for the followed aircraft. Return count (0/1) or -1 on error. */
int net_fetch_aircraft_by_hex(const char *hex, Aircraft *out);
int net_fetch_aircraft_by_callsign(const char *callsign, Aircraft *out);
int net_fetch_airports(float lat, float lon, float range_nm, Airport *out, size_t out_max,
                       bool class_b_only = false);
int net_lookup_runways(const char *icao, Runway *out, size_t out_max);
bool net_lookup_airport(const char *icao, float *lat, float *lon, int *rwy_ft, int *elev_ft = nullptr);
/** 3-letter IATA (or ICAO) -> ICAO + position. icao_out may be null. */
bool net_lookup_airport_iata(const char *code, char *icao_out, size_t icao_len, float *lat, float *lon);
/** Tower / ground / ATIS frequencies (AWC airport feed). */
bool net_lookup_airport_radios(const char *icao, AirportRadios *out);
// Route lookup by flight/callsign via hexdb callsign-route (registration also works as a fallback key).
bool net_fetch_route(const char *callsign, char *origin, size_t origin_len, char *dest, size_t dest_len);
bool net_fetch_airport_names(const char *origin_icao, const char *dest_icao, char *origin_name,
                             size_t origin_name_len, char *dest_name, size_t dest_name_len);
bool net_fetch_aircraft_meta(const char *hex, char *manufacturer, size_t man_len, char *type_name,
                             size_t type_len, char *owner, size_t owner_len, char *registration = nullptr,
                             size_t reg_len = 0);
bool net_fetch_weather(const char *icao, Weather *out);

/** Split "UA100" / "UAL100" into ICAO callsign + IATA airline/number. */
bool net_parse_flight_number(const char *query, char *icao_cs, size_t icao_cs_len, char *iata_al,
                             size_t iata_al_len, char *iata_num, size_t iata_num_len);
/** Schedule / gate / times for a flight (FlightStats). Fills fi on success. */
bool net_fetch_flight_schedule(const char *flight_key, FlightInfo *fi);
/** Filed route waypoints + alt/groundspeed history (FlightAware). */
bool net_fetch_flight_plan(const char *flight_key, FlightInfo *fi, FlightTrackPt *track, int track_max,
                           int *track_n);

// Airport pickers / boards.
int net_search_airports(const char *query, AptSearchHit *out, size_t out_max);
/** Fill cities[i] for each ICAO (28-char buffers). Returns how many resolved. */
int net_fetch_airport_cities(const char icaos[][8], char cities[][28], int n);
bool net_fetch_airport_board(const char *icao, AirportBoardRow *arr, int arr_max, int *arr_n,
                             AirportBoardRow *dep, int dep_max, int *dep_n, char *label, size_t label_len);
bool net_fetch_route_flights(const char *dep_icao, const char *arr_icao, AirportBoardRow *rows, int rows_max,
                             int *rows_n, char *label, size_t label_len);

// Email (Screen Shot / Timelapse). SMTP over implicit TLS (port 465 style).
bool net_send_email_attachment(const EmailConfig *email, const uint8_t *data, size_t len, const char *filename,
                               const char *mime, const char *subject, const char *body_text);
bool net_send_screenshot_email(const EmailConfig *email, const uint8_t *bmp, size_t bmp_len);

/** Download the SD data pack (manifest.json + files) from FR_SD_DATA_BASE_URL to /sdcard/fr_data. */
bool net_install_fr_data();

void url_encode(const char *in, char *out, size_t out_len);
