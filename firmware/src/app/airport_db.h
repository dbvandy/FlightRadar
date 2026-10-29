#pragma once
#include <stddef.h>
#include <stdint.h>

// Flash-embedded OurAirports DB (longest runway >= 3500 ft).
// lat_c/lon_c = degrees * 100. size: 0=large 1=medium 2=small.
struct AptRec { char icao[5]; char iata[4]; int16_t lat_c; int16_t lon_c; int16_t elev_ft; uint16_t rwy_ft; uint8_t size; };

extern const AptRec *kAptDb;
extern size_t K_APT_DB_COUNT;
