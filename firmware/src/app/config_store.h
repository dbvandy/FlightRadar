#pragma once

#include "types.h"

void config_load(AppConfig *cfg);
void config_save(const AppConfig *cfg);
void config_save_wifi(const char *ssid, const char *pass);
void config_save_location(const GeoFix *fix);
void config_save_range(float range_nm);
void config_save_max_alt(int max_alt_ft);
void config_save_show_airports(bool show);
