#pragma once

#include "types.h"

void config_load(AppConfig *cfg);
void config_save(const AppConfig *cfg);
void config_save_wifi(const char *ssid, const char *pass);
void config_save_location(const GeoFix *fix);
void config_save_range(float range_nm);
void config_save_max_alt(int max_alt_ft);
void config_save_show_airports(bool show);
void config_save_show_counties(bool show);
void config_save_show_cities(bool show);
void config_save_county_mask(uint64_t mask);
void config_save_brightness(int brightness);
void config_save_volume(int volume);
void config_save_email(const EmailConfig *email);

// Plates list: user-added airports (ICAO + optional city) beyond the built-in list.
void config_load_plate_extras(char ids[][FR_PLATE_EXTRA_ID_LEN], char cities[][FR_PLATE_EXTRA_CITY_LEN], int *n);
void config_save_plate_extras(const char ids[][FR_PLATE_EXTRA_ID_LEN], const char cities[][FR_PLATE_EXTRA_CITY_LEN],
                              int n);

// Recent flights tracked / airports opened on the arrivals-departures board.
void config_load_track_history(char ids[][FR_TRACK_HIST_ID_LEN], int *n);
void config_save_track_history(const char ids[][FR_TRACK_HIST_ID_LEN], int n);
void config_load_board_history(char ids[][FR_BOARD_HIST_ID_LEN], int *n);
void config_save_board_history(const char ids[][FR_BOARD_HIST_ID_LEN], int n);
