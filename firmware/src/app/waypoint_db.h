#pragma once
#include <stddef.h>
#include <stdint.h>

// Flash-embedded FAA NASR FIX / intersection DB.
// Names FlightAware "Flight Data" route tokens (filed IFR string).
struct WptRec { char name[6]; int16_t lat_c; int16_t lon_c; };

extern const WptRec *kWptDb;
extern size_t K_WPT_DB_COUNT;
