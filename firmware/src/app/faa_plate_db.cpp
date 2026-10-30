#include "airport_plate_db.h"

// SD plate bitmaps removed — Class B plates still draw procedurally via airport_plate_db.

const lv_img_dsc_t *faa_plate_lookup(const char *icao) {
    (void)icao;
    return nullptr;
}

const lv_img_dsc_t *faa_plate_peek(const char *icao) {
    (void)icao;
    return nullptr;
}

void faa_plate_request(const char *icao) {
    (void)icao;
}

bool faa_plates_service(void) {
    return false;
}

void faa_plates_invalidate(void) {}

bool faa_plates_sd_ready(void) {
    return false;
}
