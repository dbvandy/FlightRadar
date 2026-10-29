#pragma once
#include <stddef.h>
#include <stdint.h>

// Flash-embedded runway DB for airports with longest rwy >= 3500 ft.
// hdg_deg is true heading of the LE end.
struct RwyRec { char icao[5]; char le[4]; char he[4]; uint16_t len_ft; uint16_t hdg_deg; uint8_t width_ft; };

extern const RwyRec *kRwyDb;
extern size_t K_RWY_DB_COUNT;
