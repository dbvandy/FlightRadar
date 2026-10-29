#pragma once
#include <stddef.h>
#include <stdint.h>

// Airport DB loaded from SD (/sdcard/fr_data/airports.bin) into PSRAM by sd_db_load().
// lat_c/lon_c = degrees * 100. size: 0=large 1=medium 2=small.
// (The old embedded 11.7k-row table is in git history; the SD pack replaces it.)
struct AptRec { char icao[5]; char iata[4]; int16_t lat_c; int16_t lon_c; int16_t elev_ft; uint16_t rwy_ft; uint8_t size; };

extern const AptRec *kAptDb;
extern size_t K_APT_DB_COUNT;
