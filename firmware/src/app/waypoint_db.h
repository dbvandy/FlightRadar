#pragma once
#include <stddef.h>
#include <stdint.h>

// Waypoint DB left empty without SD (full NASR FIX set was SD-only).
// Follow routes still use FlightAware waypoints.
struct WptRec { char name[6]; int16_t lat_c; int16_t lon_c; };

extern const WptRec *kWptDb;
extern size_t K_WPT_DB_COUNT;
