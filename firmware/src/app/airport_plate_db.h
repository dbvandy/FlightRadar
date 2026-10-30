#pragma once

#include <lvgl.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

// Built-in plate list. Bitmaps: Update Plates downloads FRPL → SPIFFS + PSRAM;
// browsing only reads local storage (no auto network). Procedural runway = fallback.
struct AirportPlateInfo {
    const char *icao;
    const char *iata;
    const char *name;
};

static const AirportPlateInfo kAirportPlates[] = {
    {"KATL", "ATL", "Hartsfield-Jackson Atlanta Intl"},
    {"KBNA", "BNA", "Nashville Intl"},
    {"KCLT", "CLT", "Charlotte Douglas Intl"},
    {"KDFW", "DFW", "Dallas/Fort Worth Intl"},
    {"KDTW", "DTW", "Detroit Metro Wayne County"},
    {"KGNV", "GNV", "Gainesville Regional"},
    {"KHOU", "HOU", "William P Hobby"},
    {"KINF", "INF", "Inverness"},
    {"KJAX", "JAX", "Jacksonville Intl"},
    {"KMCO", "MCO", "Orlando Intl"},
    {"KOCF", "OCF", "Ocala Intl"},
    {"KTPA", "TPA", "Tampa Intl"},
    {"KX60", "X60", "Williston Regional"},
};
#define AIRPORT_PLATE_COUNT ((int)(sizeof(kAirportPlates) / sizeof(kAirportPlates[0])))

static inline int airport_plate_index(const char *icao) {
    if (!icao || !icao[0]) {
        return -1;
    }
    for (int i = 0; i < AIRPORT_PLATE_COUNT; i++) {
        if (strcasecmp(kAirportPlates[i].icao, icao) == 0) {
            return i;
        }
    }
    return -1;
}

static inline const AirportPlateInfo *airport_plate_at(int idx) {
    return (idx >= 0 && idx < AIRPORT_PLATE_COUNT) ? &kAirportPlates[idx] : nullptr;
}

// FAA bitmap plates: SPIFFS (durable) + PSRAM LRU. Network only via Update Plates.
const lv_img_dsc_t *faa_plate_lookup(const char *icao);  // PSRAM peek only (draw-safe)
/** SPIFFS → PSRAM for one ICAO. Never call from LVGL draw callbacks. */
bool faa_plate_load_local(const char *icao);
const lv_img_dsc_t *faa_plate_peek(const char *icao);
void faa_plate_request(const char *icao);  // queue only; serviced during Update Plates
/** Drop one ICAO from PSRAM cache + miss list (does not erase SPIFFS). */
void faa_plate_forget(const char *icao);
bool faa_plates_service(void);
bool faa_plates_pending(void);
/** Wipe SPIFFS + PSRAM, then download ICAOs (Update Plates button). */
void faa_plates_queue_refresh(const char icaos[][8], int n);
bool faa_plates_bulk_active(void);
bool faa_plates_bulk_finished(void);
void faa_plates_invalidate(void);
bool faa_plates_sd_ready(void);
