#pragma once

#include <lvgl.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

// Plates list: airports that ship a plate in /sdcard/fr_data/plates/*.frpl (see sd_pack).
// RECOVERY NOTE: the original generated Class B list was never committed. This covers the
// plates present in firmware/sd_pack/fr_data/plates; extend as plates are added.
struct AirportPlateInfo {
    const char *icao;
    const char *iata;
    const char *name;
};

static const AirportPlateInfo kAirportPlates[] = {
    {"KATL", "ATL", "Hartsfield-Jackson Atlanta Intl"},
    {"KBNA", "BNA", "Nashville Intl"},
    {"KCLT", "CLT", "Charlotte Douglas Intl"},
    {"KGNV", "GNV", "Gainesville Regional"},
    {"KINF", "INF", "Inverness"},
    {"KJAX", "JAX", "Jacksonville Intl"},
    {"KMCO", "MCO", "Orlando Intl"},
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

// FAA plate bitmaps on SD (faa_plate_db.cpp).
const lv_img_dsc_t *faa_plate_lookup(const char *icao);
const lv_img_dsc_t *faa_plate_peek(const char *icao);
void faa_plate_request(const char *icao);
bool faa_plates_service(void);
void faa_plates_invalidate(void);
bool faa_plates_sd_ready(void);
