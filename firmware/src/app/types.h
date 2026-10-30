#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#define FR_VERSION_STR "1.1.3"

#define FR_MAX_AIRCRAFT 64
#define FR_TRACK_NEARBY_MAX 40  // when following with Show Ac: closest N to tracked
#define FR_MAX_AIRPORTS 60
#define FR_MAX_WIFI     24
#define FR_ADS_B_MAX_NM 250.0f  // adsb.lol / adsb.fi radius cap
#define FR_FOLLOW_MAX_NM 1000.0f
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
    bool is_turf;  // grass / turf / sod strip
    int east_ft;   // midpoint east of airport centroid (plate layout)
    int north_ft;  // midpoint north of airport centroid
};

/** Tower / ground / ATIS etc. for the airport detail + plates pages. */
struct AirportRadios {
    char ctaf[12];
    char atc[12];   // tower / LCL, or ARTCC/CNTR when atc_is_center
    char gnd[12];
    char atis[12];  // ATIS, or AWOS/ASOS fallback
    bool atc_is_center;
    bool atis_is_awos;  // true when atis[] is AWOS/ASOS (no ATIS)
};

struct Weather {
    char icao[8];
    char flt_cat[8];
    char raw_metar[220];
    char raw_taf[320];
    char cover[16];
    char city[40];     // from AWC airport (or parsed from facility name)
    char freqs[96];    // formatted radio freqs (ATIS / TWR / ...)
    AirportRadios radios;  // structured AWC radio slots
    float temp_c;
    float wind_dir;
    float wind_kt;
    float vis_sm;
    bool ok;
};

#define FR_MAX_ROUTE_WPTS 32
struct RouteWaypoint {
    char name[14];  // e.g. NOBAR or 4500N/07300W
    float lat;
    float lon;
    bool ok;
};

// FlightAware track history for follow chart (1-minute samples, ~12 h).
#define FR_MAX_TRACK_PTS 720
struct FlightTrackPt {
    uint32_t t_unix;  // UTC epoch seconds
    float alt_ft;
    float gs_kts;
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
    char status[40];       // FlightStats status / phase (Boarding, At Gate, ...)
    char flight_date[28];  // e.g. "Sat Oct 18" (departure date)
    char dep_gate[12];
    char arr_gate[12];
    char dep_terminal[8];
    char arr_terminal[8];
    char dep_time[28];  // scheduled departure (local)
    char arr_time[28];  // scheduled arrival (local)
    char dep_est[28];   // estimated departure (local)
    char arr_est[28];   // estimated arrival (local)
    char dep_act[28];   // actual departure (local)
    char arr_act[28];   // actual arrival (local)
    uint32_t dep_act_epoch;  // actual departure UTC seconds (0 unknown)
    uint32_t arr_act_epoch;  // actual arrival UTC seconds (0 unknown)
    uint32_t arr_est_epoch;  // estimated arrival UTC seconds (0 unknown)
    RouteWaypoint route_wpts[FR_MAX_ROUTE_WPTS];  // filed IFR fixes from FlightAware
    int route_wpt_n;
    bool route_ok;
    bool meta_ok;
    bool schedule_ok;
    bool departed;  // left gate/runway per FlightStats (false = still at origin)
};

// Airport arrivals / departures board + route-flight search rows.
#define FR_MAX_BOARD_ROWS 20
struct AirportBoardRow {
    char flight[12];      // callsign / flight number
    char type[8];         // aircraft type (ICAO)
    char other_apt[8];    // opposite end ICAO/IATA
    char other_name[36];  // short city/airport name
    char dep_time[28];    // scheduled departure (local)
    char dep_est[28];     // estimated / actual departure (local)
    char arr_time[28];    // scheduled arrival (local)
    char arr_est[28];     // estimated / actual arrival (local)
    uint32_t dep_epoch;   // UTC seconds (0 unknown)
    uint32_t arr_epoch;   // UTC seconds (0 unknown)
    int progress_pct;     // 0..100 along route, -1 unknown
    bool valid;
};

// Airport search hit (board / plates / route-flights pickers).
#define FR_MAX_APT_SEARCH 8
struct AptSearchHit {
    char icao[8];
    char iata[4];
    char label[72];
    float lat;
    float lon;
    bool valid;
};

struct GeoFix {
    char query[96];
    char label[96];   // place name above clock (city, or ICAO when airport home)
    char icao[8];     // non-empty => home is this airport (label should match ICAO)
    float lat;
    float lon;
    bool valid;
    bool from_zip;    // home chosen via US ZIP -> label is city name
};

struct AppConfig {
    char wifi_ssid[33];
    char wifi_pass[65];
    GeoFix location;
    float range_nm;       // current PPI range
    int max_alt_ft;       // current altitude ceiling (50000 = off)
    bool show_airports;   // current overlay
    bool show_counties;   // current overlay
    bool show_cities;     // major cities (>=100k) overlay
    // Startup Settings - restored on boot / Home / Unfollow (not overwritten by follow zoom).
    float startup_range_nm;
    int startup_max_alt_ft;
    bool startup_show_airports;
    bool startup_show_counties;
    bool startup_show_cities;
    uint64_t county_mask;  // bit i = show counties for kCountyStateAbbr[i] (0=AL ... 49=WY)
    int brightness;  // 0-100 backlight
    int volume;      // 0-100 click / UI audio
};

// Persisted id lists (NVS) used by the plates / board / track pickers.
#define FR_PLATE_EXTRA_MAX 24
#define FR_PLATE_EXTRA_ID_LEN 8
#define FR_PLATE_EXTRA_CITY_LEN 28
#define FR_BOARD_HIST_MAX 8
#define FR_BOARD_HIST_ID_LEN 8
#define FR_TRACK_HIST_MAX 8
#define FR_TRACK_HIST_ID_LEN 12

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
