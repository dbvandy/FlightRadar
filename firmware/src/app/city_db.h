#pragma once
#include <stddef.h>
#include <stdint.h>

// RECOVERY SEED: the original generated city table (all US cities >= 100k) was never committed.
// This is a hand-entered list of the largest US cities (approx. centers, 2020-ish population)
// so the Cities overlay works. lat_c/lon_c = degrees * 100.
struct CityRec { const char *name; int16_t lat_c; int16_t lon_c; uint32_t pop; };
static const CityRec kCityDb[] = {
    {"New York", 4071, -7401, 8804000},   {"Los Angeles", 3405, -11824, 3898000},
    {"Chicago", 4188, -8762, 2746000},    {"Houston", 2976, -9537, 2304000},
    {"Phoenix", 3345, -11207, 1608000},   {"Philadelphia", 3995, -7516, 1603000},
    {"San Antonio", 2943, -9849, 1434000}, {"San Diego", 3272, -11716, 1386000},
    {"Dallas", 3278, -9680, 1304000},     {"San Jose", 3734, -12189, 1013000},
    {"Austin", 3027, -9774, 961000},      {"Jacksonville", 3033, -8166, 949000},
    {"Fort Worth", 3275, -9732, 918000},  {"Columbus", 3996, -8300, 905000},
    {"Charlotte", 3523, -8083, 874000},   {"San Francisco", 3778, -12242, 873000},
    {"Indianapolis", 3977, -8616, 887000}, {"Seattle", 4761, -12233, 737000},
    {"Denver", 3974, -10499, 715000},     {"Washington", 3891, -7704, 689000},
    {"Boston", 4236, -7106, 675000},      {"El Paso", 3176, -10650, 678000},
    {"Nashville", 3616, -8678, 689000},   {"Detroit", 4233, -8305, 639000},
    {"Oklahoma City", 3547, -9752, 681000}, {"Portland", 4552, -12268, 652000},
    {"Las Vegas", 3617, -11514, 641000},  {"Memphis", 3515, -9001, 633000},
    {"Louisville", 3825, -8576, 633000},  {"Baltimore", 3929, -7661, 585000},
    {"Milwaukee", 4304, -8791, 577000},   {"Albuquerque", 3508, -10665, 564000},
    {"Tucson", 3222, -11097, 542000},     {"Fresno", 3674, -11979, 542000},
    {"Sacramento", 3858, -12149, 524000}, {"Kansas City", 3910, -9458, 508000},
    {"Atlanta", 3375, -8439, 498000},     {"Omaha", 4126, -9594, 486000},
    {"Colorado Springs", 3883, -10482, 478000}, {"Raleigh", 3578, -7864, 467000},
    {"Miami", 2578, -8019, 442000},       {"Virginia Beach", 3685, -7598, 459000},
    {"Oakland", 3780, -12227, 440000},    {"Minneapolis", 4498, -9327, 429000},
    {"Tulsa", 3615, -9599, 413000},       {"Tampa", 2795, -8246, 384000},
    {"Arlington", 3270, -9711, 394000},   {"New Orleans", 2995, -9007, 384000},
    {"Wichita", 3769, -9734, 397000},     {"Cleveland", 4150, -8168, 372000},
    {"Bakersfield", 3537, -11902, 403000}, {"Honolulu", 2131, -15786, 345000},
    {"Anaheim", 3384, -11791, 346000},    {"Orlando", 2854, -8138, 307000},
    {"Lexington", 3804, -8450, 322000},   {"Anchorage", 6122, -14990, 291000},
    {"Pittsburgh", 4044, -8000, 303000},  {"Cincinnati", 3910, -8451, 309000},
    {"St. Louis", 3863, -9020, 301000},   {"Salt Lake City", 4076, -11189, 200000},
    {"Birmingham", 3352, -8681, 200000},  {"Buffalo", 4288, -7885, 278000},
    {"Richmond", 3754, -7744, 226000},    {"Boise", 4362, -11620, 235000},
    {"Des Moines", 4160, -9362, 214000},  {"Providence", 4182, -7142, 190000},
    {"Hartford", 4176, -7268, 121000},    {"Jackson", 3230, -9019, 153000},
    {"Little Rock", 3475, -9227, 202000}, {"Charleston", 3278, -7993, 150000},
    {"Savannah", 3208, -8110, 147000},    {"Tallahassee", 3044, -8428, 196000},
    {"Gainesville", 2965, -8233, 141000}, {"Knoxville", 3596, -8392, 190000},
};
#define K_CITY_DB_COUNT (sizeof(kCityDb) / sizeof(kCityDb[0]))
