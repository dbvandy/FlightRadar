#pragma once
#include <stddef.h>
#include <stdint.h>

// Runway DB loaded from SD (/sdcard/fr_data/runways.bin) into PSRAM by sd_db_load().
// hdg_deg is true heading of the LE end; east_ft/north_ft = runway midpoint offset from
// the airport centroid (plate layout). (Old embedded table is in git history.)
struct RwyRec { char icao[5]; char le[4]; char he[4]; uint16_t len_ft; uint16_t hdg_deg; uint8_t width_ft; uint8_t is_turf; int16_t east_ft; int16_t north_ft; };

extern const RwyRec *kRwyDb;
extern size_t K_RWY_DB_COUNT;
