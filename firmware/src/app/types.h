#pragma once

#include <Arduino.h>
#include <math.h>

#define FR_MAX_AIRCRAFT 64
#define FR_MAX_AIRPORTS 60
#define FR_MAX_WIFI     24
#define FR_NM_TO_FT     6076.12f
#define FR_DEG2RAD      0.01745329252f
#define FR_RAD2DEG      57.295779513f

struct Aircraft {
    char hex[8];
    char flight[12];
    char reg[12];
    char type[8];
    char squawk[8];
    char category[4];  // ADS-B emitter category, e.g. A1/A3/A5
    float lat;
    float lon;
    float alt_ft;
    float gs_kts;
    float track_deg;
    float vrate_fpm;
    float dst_nm;
    bool on_ground;
    bool valid;
};

struct Airport {
    char icao[8];
    char iata[8];
    char name[72];
    char state[8];
    char country[8];
    float lat;
    float lon;
    int elev_ft;
    int longest_rwy_ft;
    char rwy_class;  // AWC rwyLength: 'L', 'M', 'S', or 0
    bool valid;
};

#define FR_MAX_RWYS_PER_APT 8
struct Runway {
    char le[4];
    char he[4];
    int len_ft;
    int hdg_deg;   // true heading of LE end
    int width_ft;
};

struct Weather {
    char icao[8];
    char flt_cat[8];
    char raw_metar[220];
    char raw_taf[320];
    char cover[16];
    float temp_c;
    float wind_dir;
    float wind_kt;
    float vis_sm;
    bool ok;
};

struct FlightInfo {
    char flight[12];
    char hex[8];
    char reg[12];
    char origin[8];
    char dest[8];
    char origin_name[48];
    char dest_name[48];
    char manufacturer[40];
    char type_name[48];
    char owner[48];
    bool route_ok;
    bool meta_ok;
};

struct GeoFix {
    char query[96];
    char label[96];
    float lat;
    float lon;
    bool valid;
};

struct AppConfig {
    char wifi_ssid[33];
    char wifi_pass[65];
    GeoFix location;
    float range_nm;
    int max_alt_ft;  // display ceiling: show aircraft with alt_ft < max_alt_ft
    bool show_airports;
};

inline void ll_to_nm(float lat, float lon, float olat, float olon, float *east_nm, float *north_nm) {
    *north_nm = (lat - olat) * 60.0f;
    *east_nm = (lon - olon) * 60.0f * cosf(olat * FR_DEG2RAD);
}

inline void bbox_from_range(float lat, float lon, float range_nm, float *lamin, float *lomin, float *lamax, float *lomax) {
    float dlat = range_nm / 60.0f;
    float clat = cosf(lat * FR_DEG2RAD);
    if (clat < 0.15f) {
        clat = 0.15f;
    }
    float dlon = range_nm / (60.0f * clat);
    *lamin = lat - dlat;
    *lamax = lat + dlat;
    *lomin = lon - dlon;
    *lomax = lon + dlon;
}

inline void trim_inplace(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == '\t')) {
        s[--n] = 0;
    }
    size_t i = 0;
    while (s[i] == ' ' || s[i] == '\t') {
        i++;
    }
    if (i) {
        memmove(s, s + i, n - i + 1);
    }
}
