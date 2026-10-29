#pragma once

#include <stddef.h>
#include <stdint.h>

#include "airport_db.h"
#include "runway_db.h"
#include "waypoint_db.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mount SD and load /sdcard/fr_data/{waypoints,airports,runways}.bin into PSRAM.
 * Safe to call more than once. Returns true if all three loaded (or already loaded).
 */
bool sd_db_load(void);

/** Drop PSRAM copies and reload from SD (after Update Data wrote new .bin files). */
void sd_db_reload(void);

/** Status line for UI / serial. */
const char *sd_db_status_text(void);

#ifdef __cplusplus
}
#endif
