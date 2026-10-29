#pragma once
#include <stddef.h>
#include <stdint.h>

// FAA NASR FIX waypoints loaded from SD (/sdcard/fr_data/waypoints.bin) by sd_db_load().
// lat_c/lon_c = degrees * 100.
struct WptRec { char name[6]; int16_t lat_c; int16_t lon_c; };

extern const WptRec *kWptDb;
extern size_t K_WPT_DB_COUNT;
