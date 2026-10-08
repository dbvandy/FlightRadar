#pragma once

#include "types.h"

// Call once early in setup() before RGB/LVGL init so the body buffer lands in DRAM.
void net_init_buffers();

// body may be nullptr to keep the payload only in the internal fixed buffer.
bool https_get(const char *url, String *body, const char *user_agent = nullptr,
               uint32_t connect_ms = 8000, uint32_t io_ms = 10000);
const char *net_last_error();
void net_set_status_callback(void (*cb)(const char *msg));

int net_geocode(const char *query, GeoFix *out, size_t out_max);
int net_fetch_aircraft(float lat, float lon, float range_nm, Aircraft *out, size_t out_max);
int net_fetch_airports(float lat, float lon, float range_nm, Airport *out, size_t out_max);
int net_lookup_runways(const char *icao, Runway *out, size_t out_max);
bool net_lookup_airport(const char *icao, float *lat, float *lon, int *rwy_ft);
// Route lookup by flight/callsign via hexdb callsign-route (registration also works as a fallback key).
bool net_fetch_route(const char *callsign, char *origin, size_t origin_len, char *dest, size_t dest_len);
bool net_fetch_airport_names(const char *origin_icao, const char *dest_icao, char *origin_name,
                             size_t origin_name_len, char *dest_name, size_t dest_name_len);
bool net_fetch_aircraft_meta(const char *hex, char *manufacturer, size_t man_len, char *type_name,
                             size_t type_len, char *owner, size_t owner_len, char *registration = nullptr,
                             size_t reg_len = 0);
bool net_fetch_weather(const char *icao, Weather *out);

void url_encode(const char *in, char *out, size_t out_len);
