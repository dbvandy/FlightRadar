#include "ui.h"

#include "audio_click.h"
#include "board_ctrl.h"
#include "config_store.h"
#include "lvgl_v8_port.h"
#include "city_db.h"
#include "county_db.h"
#include "airport_plate_db.h"
#include "faa_plate_db.h"
#include "map_db.h"
#include "net_client.h"
#include "splash_img.h"
#include "board_config.h"
#include <WiFi.h>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

static SharedState *g_state = nullptr;
#ifdef BOARD_CROWPANEL_P4
// 1024x600: square scope sized to the panel height, side rail stays on the right.
static constexpr lv_coord_t kRadarPlot = 580;
static constexpr lv_coord_t kSideW = 400;
static constexpr lv_coord_t kSideH = 580;
static constexpr lv_coord_t kClockSize = 66;  // midway between 80 and 52
#else
static constexpr lv_coord_t kRadarPlot = 470;
static constexpr lv_coord_t kSideW = 318;
static constexpr lv_coord_t kSideH = 468;
static constexpr lv_coord_t kClockSize = 56;
#endif

static lv_obj_t *scr_radar = nullptr;
static lv_obj_t *scr_splash = nullptr;
static volatile bool s_splash_tap = false;
static lv_obj_t *wifi_panel = nullptr;
static lv_obj_t *wifi_show_pw_cb = nullptr;
static lv_obj_t *wifi_joining_overlay = nullptr;
static lv_obj_t *wifi_joining_lbl = nullptr;
static lv_obj_t *loc_panel = nullptr;
static lv_obj_t *wifi_list = nullptr;
static lv_obj_t *wifi_status = nullptr;
static lv_obj_t *wifi_connected_lbl = nullptr;
static lv_obj_t *wifi_ta = nullptr;
static lv_obj_t *wifi_kb = nullptr;
static lv_obj_t *loc_ta = nullptr;
static lv_obj_t *loc_list = nullptr;
static lv_obj_t *loc_status = nullptr;
static lv_obj_t *loc_kb = nullptr;
static lv_obj_t *loc_zoom_dd = nullptr;
static lv_obj_t *loc_alt_dd = nullptr;
static lv_obj_t *loc_cty_cb = nullptr;
static lv_obj_t *loc_apt_cb = nullptr;
static lv_obj_t *loc_city_cb = nullptr;
static lv_obj_t *track_panel = nullptr;
static lv_obj_t *track_ta = nullptr;
static lv_obj_t *track_status = nullptr;
static lv_obj_t *track_kb = nullptr;
static lv_obj_t *board_panel = nullptr;
static lv_obj_t *plates_panel = nullptr;
static lv_obj_t *plates_list = nullptr;
static lv_obj_t *plates_preview = nullptr;
static lv_obj_t *plates_status = nullptr;
static lv_obj_t *plates_ta = nullptr;
static lv_obj_t *plates_add_ta = nullptr;
static lv_obj_t *plates_add_hits = nullptr;
static lv_obj_t *plates_title_lbl = nullptr;
static lv_obj_t *plates_kb = nullptr;
static lv_obj_t *plates_kb_target = nullptr;
static int s_plates_sel = 0;
#define FR_PLATES_LIST_MAX (AIRPORT_PLATE_COUNT + FR_PLATE_EXTRA_MAX)
static int s_plates_filt[FR_PLATES_LIST_MAX];
static int s_plates_filt_n = 0;
static Runway s_plates_prev_rwys[FR_MAX_RWYS_PER_APT];
static int s_plates_prev_rwy_n = 0;
static char s_plates_prev_icao[8] = {0};
static char s_plate_extra[FR_PLATE_EXTRA_MAX][FR_PLATE_EXTRA_ID_LEN];
static char s_plate_extra_city[FR_PLATE_EXTRA_MAX][FR_PLATE_EXTRA_CITY_LEN];
static int s_plate_extra_n = 0;
static bool s_plates_from_settings = true;
static char s_detail_city[40] = {0};
static AirportRadios s_detail_radios{};
static lv_obj_t *plates_info_tbl = nullptr;
static lv_obj_t *plates_city_lbl = nullptr;
static AirportRadios s_plates_radios{};
static AptSearchHit s_plate_hits[FR_MAX_APT_SEARCH];
static int s_plate_hit_n = 0;
static bool s_plates_search_pending = false;
static lv_obj_t *board_ta = nullptr;
static lv_obj_t *board_status = nullptr;
static lv_obj_t *board_title_lbl = nullptr;
static lv_obj_t *board_hits = nullptr;
static lv_obj_t *board_hist_cont = nullptr;
static lv_obj_t *board_arr_hdr = nullptr;   // fixed header (does not scroll)
static lv_obj_t *board_dep_hdr = nullptr;
static lv_obj_t *board_arr_table = nullptr; // scrollable body rows only
static lv_obj_t *board_dep_table = nullptr;
static lv_obj_t *board_kb = nullptr;
static AirportBoardRow s_board_arr[FR_MAX_BOARD_ROWS];
static AirportBoardRow s_board_dep[FR_MAX_BOARD_ROWS];
static int s_board_arr_n = 0;
static int s_board_dep_n = 0;
static lv_obj_t *s_board_sel_table = nullptr;  // arrivals or departures body with highlight
static int s_board_sel_row = -1;               // body row index (>=0 = flight)
static constexpr lv_coord_t kBoardColW[6] = {98, 56, 94, 72, 72, 78};  // FLIGHT -1ch; EQP gap before FROM/TO
static constexpr lv_coord_t kBoardHdrH = 44;
static constexpr lv_coord_t kBoardBodyH = 282;  // 326 total - header
// Per-table sort: 0=FLIGHT 1=EQP 2=FROM/TO 3=DEP 4=ARR 5=AIR
static int s_board_arr_sort_col = 3;
static bool s_board_arr_sort_asc = true;
static int s_board_dep_sort_col = 3;
static bool s_board_dep_sort_asc = true;
static AptSearchHit s_apt_hits[FR_MAX_APT_SEARCH];
static int s_apt_hit_n = 0;
// Dep/Arr city-pair flight search (mirrors Airport Board).
static lv_obj_t *route_panel = nullptr;
static lv_obj_t *route_dep_ta = nullptr;
static lv_obj_t *route_arr_ta = nullptr;
static lv_obj_t *route_status = nullptr;
static lv_obj_t *route_title_lbl = nullptr;
static lv_obj_t *route_dep_hits = nullptr;
static lv_obj_t *route_arr_hits = nullptr;
static lv_obj_t *route_table = nullptr;
static lv_obj_t *route_kb = nullptr;
static lv_obj_t *route_kb_target = nullptr;
static AirportBoardRow s_route_rows[FR_MAX_BOARD_ROWS];
static int s_route_row_n = 0;
static int s_route_sel_row = -1;
static AptSearchHit s_route_dep_hits[FR_MAX_APT_SEARCH];
static int s_route_dep_hit_n = 0;
static AptSearchHit s_route_arr_hits[FR_MAX_APT_SEARCH];
static int s_route_arr_hit_n = 0;
static char s_route_dep_icao[8] = {0};
static char s_route_arr_icao[8] = {0};
static char s_board_hist[FR_BOARD_HIST_MAX][FR_BOARD_HIST_ID_LEN];
static int s_board_hist_n = 0;
static lv_obj_t *track_hist_cont = nullptr;
static char s_track_hist[FR_TRACK_HIST_MAX][FR_TRACK_HIST_ID_LEN];
static int s_track_hist_n = 0;
static lv_obj_t *radar_plot = nullptr;
static lv_obj_t *radar_status = nullptr;
static lv_obj_t *radar_range_lbl = nullptr;
static lv_obj_t *radar_alt_lbl = nullptr;
static lv_obj_t *radar_place_lbl = nullptr;   // city / airport / Following …
static lv_obj_t *radar_clock = nullptr;       // minimal analog clock (UR corner)
static lv_obj_t *board_clock = nullptr;       // same analog clock on Airport Board
static lv_obj_t *board_clock_lbl = nullptr;   // "Local Time for Airport"
static int s_clk_h = 0, s_clk_m = 0, s_clk_s = 0;
static bool s_clk_valid = false;
static int s_board_clk_h = 0, s_board_clk_m = 0, s_board_clk_s = 0;
static bool s_board_clk_valid = false;
static char s_board_icao[8] = {0};
static float s_board_lat = 0.0f;
static float s_board_lon = 0.0f;
static bool s_board_ll_ok = false;
static lv_obj_t *radar_count_lbl = nullptr;
static lv_obj_t *traffic_table = nullptr;
static lv_obj_t *follow_panel = nullptr;
static lv_obj_t *follow_info_lbl = nullptr;
static lv_obj_t *follow_list_btn = nullptr;
static lv_obj_t *follow_center_btn = nullptr;
static lv_obj_t *follow_sched_lbl = nullptr;
static lv_obj_t *follow_chart = nullptr;
static lv_obj_t *follow_btn_row = nullptr;
static lv_obj_t *side_controls = nullptr;
static lv_coord_t s_controls_h_list = 178;
static lv_coord_t s_controls_h_follow = 178;
static lv_coord_t s_follow_table_h = 0;
static lv_obj_t *detail = nullptr;
static lv_obj_t *detail_title = nullptr;
static lv_obj_t *detail_body = nullptr;
static lv_obj_t *detail_plate = nullptr;
static lv_obj_t *detail_center_btn = nullptr;
static lv_obj_t *detail_follow_btn = nullptr;
static lv_obj_t *detail_add_plate_btn = nullptr;
static Runway s_plate_rwys[FR_MAX_RWYS_PER_APT];
static int s_plate_rwy_n = 0;
static char s_plate_icao[8] = {0};
static Airport s_detail_apt{};
static bool s_detail_apt_ok = false;
static Aircraft s_detail_ac{};
static bool s_detail_is_aircraft = false;
static FlightInfo s_flight_info{};
static Aircraft s_table_ac[FR_MAX_AIRCRAFT];
static int s_table_n = 0;
static int s_traffic_sel_row = -1;  // table row (0 = header; >=1 = flight)
static lv_timer_t *s_traffic_ui_timer = nullptr;
static lv_timer_t *s_clock_timer = nullptr;
static lv_timer_t *s_follow_state_timer = nullptr;  // periodic aircraft-state → county layer
static lv_timer_t *s_follow_path_center_timer = nullptr;  // path overview → Center after 15 s
static uint32_t s_status_lv_ms = 0;
static int s_tz_off_sec = 0;  // rough civil TZ from home longitude

// Follow-flight: center radar on a hex and record a path trail.
static bool s_follow = false;
static int s_follow_county_state = -2;  // aircraft state while counties on (-2 unset, -1 unknown, 0..49)
static bool s_follow_solo = false;  // when following: hide all other aircraft (fixed on; no UI toggle)
// Follow route overlay: 0=show all, 1=hide names, 2=hide names+line (+marks).
static int s_wpt_view_mode = 0;
static bool s_follow_hold_center = false;  // user panned the map — don't snap back to aircraft
static bool s_follow_approach_zoom_off = false;  // user zoom in/out — stop landing auto-zoom
static char s_follow_hex[8] = {0};
static char s_follow_label[16] = {0};
static GeoFix s_follow_saved_loc{};
static bool s_follow_saved_ok = false;
static float s_follow_saved_range = 250.0f;
static int s_follow_saved_max_alt = 45000;
static bool s_follow_saved_view_ok = false;
static bool s_follow_route_fitted = false;  // zoom/center fitted to dep+arr for this track
static bool s_follow_spd_view_done = false;  // applied once per track from groundspeed
static float s_hist_alt_floor = -1.0f;  // rolling low alt while locking chart climb-start
static GeoFix s_home_loc{};  // base/home from boot cfg or Change location (not pan center)
static bool s_home_ok = false;
static Aircraft s_follow_ac{};
static FlightInfo s_follow_fi{};
static bool s_follow_ac_ok = false;
// Airborne flight timer: latch first wheels-up sample (excludes taxi / gate / first-seen).
static uint32_t s_follow_takeoff_ms = 0;  // millis at takeoff; 0 = not yet airborne
static uint32_t s_follow_land_ms = 0;     // millis at landing after takeoff; 0 = still flying
// Persistent dep/arr while following — survive later ADS-B / route refreshes.
struct FollowApt {
    char icao[8];
    char name[48];
    float lat;
    float lon;
    int elev_ft;
    bool ok;
};
static FollowApt s_follow_dep{};
static FollowApt s_follow_arr{};
static Runway s_follow_dep_rwys[FR_MAX_RWYS_PER_APT];
static int s_follow_dep_rwy_n = 0;
static Runway s_follow_arr_rwys[FR_MAX_RWYS_PER_APT];
static int s_follow_arr_rwy_n = 0;
static bool s_follow_county_user_hid = false;  // user hid counties while tracking — don't auto-show
static constexpr int kFollowWptMax = FR_MAX_ROUTE_WPTS;
struct FollowWpt {
    float lat;
    float lon;
    char label[14];
    bool ok;
};
static FollowWpt s_follow_wpts[kFollowWptMax];
static int s_follow_wpt_n = 0;
static const lv_color_t COL_WPT = lv_color_hex(0x7EC8FF);
static constexpr int kTrailMax = 160;
struct TrailPt {
    float lat;
    float lon;
};
static TrailPt s_trail[kTrailMax];
static int s_trail_head = 0;
static int s_trail_count = 0;
static constexpr int kHistMax = 720;  // 1-min samples, ~12 h takeoff→now
static constexpr uint32_t kHistMinWindowMs = 20u * 60u * 1000u;  // X-axis defaults to 20 min
struct HistPt {
    float alt_ft;
    float gs_kts;
    uint32_t ms;
    uint32_t t_unix;  // UTC epoch when known (FA seed / timed live)
};
static HistPt s_hist[kHistMax];
static int s_hist_head = 0;
static int s_hist_count = 0;
static bool s_hist_from_fa = false;  // seeded from FlightAware track history
static uint32_t s_hist_fa_t0_unix = 0;  // takeoff / first chart sample (unix)
static uint32_t s_hist_fa_t_end_unix = 0;  // last FA sample used in seed
static uint32_t s_hist_last_minute = 0;    // live append throttle (unix / 60)

// Near zoom (always). While following, zoom-out continues in 250 nm steps to 1000.
static const float kZoomNear[] = {4, 5, 15, 25, 40, 60, 100, 160, 250};
static const int kZoomNearN = 9;
static const float kZoomFarStep = 250.0f;
static const float kZoomFarMax = FR_FOLLOW_MAX_NM;
static const lv_coord_t kRadarPanClickThresh = 10;  // px: drag beyond this skips aircraft/airport pick
static const int kAltMinFt = 5000;
static const int kAltMaxFt = 50000;
static const int kAltStepFt = 5000;

static const lv_color_t COL_BG = lv_color_hex(0x050805);
static const lv_color_t COL_PANEL = lv_color_hex(0x0C140E);
static const lv_color_t COL_GREEN = lv_color_hex(0x27E36A);
static const lv_color_t COL_DIM = lv_color_hex(0x1A5A32);
static const lv_color_t COL_TEXT = lv_color_hex(0xD8FFE4);
static const lv_color_t COL_AC = lv_color_hex(0xFFE066);
static const lv_color_t COL_PATH = lv_color_hex(0xFFF5A8);  // light yellow flight path
static const lv_color_t COL_SPD = lv_color_hex(0x7EC8FF);   // speed series on follow chart
static const lv_color_t COL_APT_LG = lv_color_hex(0x4DA6FF);
static const lv_color_t COL_APT_MD = lv_color_hex(0xD45CFF);
static const lv_color_t COL_APT_SM = lv_color_hex(0x8A9A84);
static const lv_color_t COL_MAP = lv_color_hex(0x4A554E);  // state/coast lines + city labels
static const lv_color_t COL_HOME = lv_color_hex(0x7EC8FF);  // home city / airport highlight
// Control buttons: darker blue face + border with a raised 3D bevel.
static const lv_color_t COL_BTN = lv_color_hex(0x0E3A78);
static const lv_color_t COL_BTN_HI = lv_color_hex(0x1E5A9C);  // top of vertical gradient
static const lv_color_t COL_BTN_BORDER = lv_color_hex(0x2E6AB0);
static const lv_color_t COL_BTN_PRESS = lv_color_hex(0x081F48);
static const lv_color_t COL_BTN_SHADOW = lv_color_hex(0x01040C);

static void style_screen(lv_obj_t *scr) {
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
}

static void ui_btn_click_sound(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    if (!obj || !lv_obj_has_flag(obj, LV_OBJ_FLAG_CLICKABLE)) {
        return;
    }
    audio_click_play();
}

static void ui_kb_click_sound(lv_event_t *e) {
    LV_UNUSED(e);
    audio_click_play();
}

static lv_obj_t *make_btn(lv_obj_t *parent, const char *txt, lv_event_cb_t cb, void *ud) {
    lv_obj_t *btn = lv_btn_create(parent);
    // Raised blue face (light→dark vertical gradient).
    lv_obj_set_style_bg_color(btn, COL_BTN_HI, 0);
    lv_obj_set_style_bg_grad_color(btn, COL_BTN, 0);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_color(btn, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_outline_width(btn, 1, 0);
    lv_obj_set_style_outline_color(btn, lv_color_hex(0x4A80C0), 0);
    lv_obj_set_style_outline_opa(btn, LV_OPA_40, 0);
    lv_obj_set_style_outline_pad(btn, 0, 0);
    lv_obj_set_style_shadow_width(btn, 8, 0);
    lv_obj_set_style_shadow_ofs_x(btn, 2, 0);
    lv_obj_set_style_shadow_ofs_y(btn, 3, 0);
    lv_obj_set_style_shadow_color(btn, COL_BTN_SHADOW, 0);
    lv_obj_set_style_shadow_opa(btn, LV_OPA_70, 0);
    lv_obj_set_style_radius(btn, 8, 0);
    // Pressed = inset: darker face, thinner shadow, slight sink.
    lv_obj_set_style_bg_color(btn, COL_BTN_PRESS, LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(btn, COL_BTN, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x1A4A88), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(btn, 2, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_x(btn, 0, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(btn, 1, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(btn, LV_OPA_50, LV_STATE_PRESSED);
    lv_obj_set_style_translate_y(btn, 1, LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, ui_btn_click_sound, LV_EVENT_PRESSED, nullptr);
    if (cb) {
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
    }
    lv_obj_t *l = lv_label_create(btn);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, COL_TEXT, 0);
    lv_obj_center(l);
    return btn;
}

static void request(uint32_t r) {
    if (g_state) {
        __sync_fetch_and_or((uint32_t *)&g_state->requests, r);
    }
}

static float snap_zoom_level(float range, bool following) {
    if (range <= 250.0f + 0.5f) {
        int best = 0;
        float err = 1e9f;
        for (int i = 0; i < kZoomNearN; i++) {
            const float e = fabsf(kZoomNear[i] - range);
            if (e < err) {
                err = e;
                best = i;
            }
        }
        return kZoomNear[best];
    }
    if (!following) {
        return 250.0f;
    }
    // 500, 750, ... 1000
    float stepped = 250.0f + kZoomFarStep * roundf((range - 250.0f) / kZoomFarStep);
    if (stepped < 500.0f) {
        stepped = 500.0f;
    }
    if (stepped > kZoomFarMax) {
        stepped = kZoomFarMax;
    }
    return stepped;
}

static float zoom_out_level(float range, bool following) {
    if (range < 250.0f - 0.5f) {
        for (int i = 0; i < kZoomNearN; i++) {
            if (kZoomNear[i] > range + 0.5f) {
                return kZoomNear[i];
            }
        }
        return 250.0f;
    }
    if (!following) {
        return 250.0f;
    }
    float next = snap_zoom_level(range, true) + kZoomFarStep;
    if (next > kZoomFarMax) {
        next = kZoomFarMax;
    }
    return next;
}

static float zoom_in_level(float range, bool following) {
    if (range > 250.0f + 0.5f) {
        if (!following) {
            return 250.0f;
        }
        float prev = snap_zoom_level(range, true) - kZoomFarStep;
        if (prev < 250.0f) {
            return 250.0f;
        }
        return prev;
    }
    for (int i = kZoomNearN - 1; i >= 0; i--) {
        if (kZoomNear[i] < range - 0.5f) {
            return kZoomNear[i];
        }
    }
    return kZoomNear[0];
}

static void set_range(float nm);
static void set_range_ex(float nm, bool request_refresh);
static void set_max_alt(int alt_ft);
static void set_max_alt_ex(int alt_ft, bool persist);
static void capture_home_from_cfg();
static void restore_startup_view();
static void leave_follow_for_home();
static const char *home_place_label();
static void apply_traffic_ui(bool redraw_radar);
static constexpr uint64_t kCountyMaskAllBits = (1ULL << 50) - 1;  // bits 0..49

static void sync_follow_to_state() {
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->follow_active = s_follow;
    g_state->follow_solo = s_follow_solo;
    g_state->follow_hold_center = s_follow_hold_center;
    if (!s_follow) {
        g_state->follow_hex[0] = 0;
        g_state->follow_flight[0] = 0;
        g_state->follow_hold_center = false;
        g_state->tracked_ok = false;
        memset(&g_state->tracked_ac, 0, sizeof(g_state->tracked_ac));
    } else {
        strncpy(g_state->follow_hex, s_follow_hex, sizeof(g_state->follow_hex) - 1);
        g_state->follow_hex[sizeof(g_state->follow_hex) - 1] = 0;
        strncpy(g_state->follow_flight, s_follow_label, sizeof(g_state->follow_flight) - 1);
        g_state->follow_flight[sizeof(g_state->follow_flight) - 1] = 0;
        if (s_follow_ac_ok) {
            g_state->tracked_ac = s_follow_ac;
            g_state->tracked_ok = true;
        }
    }
    xSemaphoreGive(g_state->mutex);
}

static lv_color_t airport_color(const Airport *a) {
    if (a->rwy_class == 'L' || a->longest_rwy_ft >= 8000 ||
        (a->iata[0] && a->longest_rwy_ft >= 5000)) {
        return COL_APT_LG;
    }
    if (a->rwy_class == 'M' || a->iata[0] || a->longest_rwy_ft >= 4000) {
        return COL_APT_MD;
    }
    return COL_APT_SM;
}

static Aircraft s_ac[FR_MAX_AIRCRAFT];
static Airport s_apt[FR_MAX_AIRPORTS];
static int s_ac_n = 0;
static int s_apt_n = 0;
static float s_olat = 0;
static float s_olon = 0;
static float s_range = 40;
static int s_max_alt = 45000;
static bool s_show_airports = true;
static bool s_show_counties = true;
static bool s_show_cities = true;
static uint64_t s_county_mask = (1ULL << 50) - 1;  // all 50 states until config loads
static lv_obj_t *apt_toggle_btn = nullptr;
static lv_obj_t *cty_toggle_btn = nullptr;
static lv_obj_t *city_toggle_btn = nullptr;
static lv_obj_t *wpt_toggle_btn = nullptr;

static bool aircraft_below_ceiling(const Aircraft *ac, int max_alt_ft) {
    // 50000 = no ceiling (Home / boot).
    if (max_alt_ft <= 0 || max_alt_ft >= kAltMaxFt) {
        return true;
    }
    if (ac->on_ground) {
        return true;
    }
    return ac->alt_ft <= (float)max_alt_ft;
}

static bool aircraft_shown(const Aircraft *ac) {
    if (!ac || !ac->valid) {
        return false;
    }
    // While tracking: only the followed aircraft (hide everyone else on the PPI).
    if (s_follow) {
        if (s_follow_hex[0] && ac->hex[0] && strcasecmp(ac->hex, s_follow_hex) == 0) {
            return true;
        }
        if (s_follow_label[0] && ac->flight[0] && strcasecmp(ac->flight, s_follow_label) == 0) {
            return true;
        }
        return false;
    }
    return aircraft_below_ceiling(ac, s_max_alt);
}

static void refresh_wifi_connected_lbl() {
    if (!wifi_connected_lbl) {
        return;
    }
    char buf[72];
    if (g_state && g_state->wifi_connected && g_state->cfg.wifi_ssid[0]) {
        snprintf(buf, sizeof(buf), "Connected to: %s", g_state->cfg.wifi_ssid);
    } else if (g_state && g_state->cfg.wifi_ssid[0]) {
        snprintf(buf, sizeof(buf), "Connected to: (not joined)  saved %s", g_state->cfg.wifi_ssid);
    } else {
        snprintf(buf, sizeof(buf), "Connected to: —");
    }
    lv_label_set_text(wifi_connected_lbl, buf);
}

static void update_wpt_toggle_btn() {
    if (!wpt_toggle_btn) {
        return;
    }
    if (s_follow) {
        lv_obj_clear_flag(wpt_toggle_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_t *lbl = lv_obj_get_child(wpt_toggle_btn, 0);
        if (lbl) {
            // Label = next action: Show all → Hide Wpts → Hide Path → Show Path.
            const char *txt = "Hide Wpts";
            if (s_wpt_view_mode == 1) {
                txt = "Hide Path";
            } else if (s_wpt_view_mode == 2) {
                txt = "Show Path";
            }
            lv_label_set_text(lbl, txt);
        }
    } else {
        lv_obj_add_flag(wpt_toggle_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void format_alt_ceiling(int alt_ft, char *buf, size_t n) {
    snprintf(buf, n, "FL%03d", alt_ft / 100);
}

static bool map_point_ll(float lat, float lon, float olat, float olon, float range, lv_coord_t w, lv_coord_t h, lv_coord_t *x, lv_coord_t *y) {
    float east, north;
    ll_to_nm(lat, lon, olat, olon, &east, &north);
    const float R = (LV_MIN(w, h) / 2.0f) - 8.0f;
    *x = (lv_coord_t)lroundf(w / 2.0f + (east / range) * R);
    *y = (lv_coord_t)lroundf(h / 2.0f - (north / range) * R);
    const float dist = hypotf(east, north);
    return dist <= range * 1.02f;
}

static void map_cache_mark_dirty();
static void map_plates_mark_dirty();
static void plates_city_from_name(const char *name, char *out, size_t out_len);
static int state_index_from_ll(float lat, float lon);
static void follow_refresh_county_state(bool force_redraw);
static void follow_maybe_approach_zoom(const Aircraft *ac);
static void follow_state_timer_cb(lv_timer_t *t);
static void follow_state_timer_ensure(bool on);
static void refresh_traffic_table();
static void refresh_follow_panel();
static void set_follow_side_mode(bool following);
static void open_airport_board(lv_event_t *e);
static void build_airport_board();
static void hide_airport_board(lv_event_t *e);
static void open_route_search(lv_event_t *e);
static void build_route_search();
static void hide_route_search(lv_event_t *e);
static void track_start_query(const char *q, bool show_kb);
static void merge_follow_fi(const FlightInfo &fi);
static void layout_follow_info_boxes();
static void refresh_follow_sched_col();
static void follow_live_status(const Aircraft *ac, const FlightInfo *fi, char *out, size_t n);
static void update_cty_toggle_label();
static void update_apt_toggle_label();
static void update_city_toggle_label();
static void sync_county_checkboxes();
static uint32_t follow_dep_epoch_utc();
static void hist_maybe_lock_climb_t0(float alt_ft, uint32_t t_unix);
static void center_on_flight(lv_event_t *e);
static void follow_path_center_timer_cancel();

static void trail_clear() {
    s_trail_head = 0;
    s_trail_count = 0;
}

static void hist_clear() {
    s_hist_head = 0;
    s_hist_count = 0;
    s_hist_from_fa = false;
    s_hist_fa_t0_unix = 0;
    s_hist_fa_t_end_unix = 0;
    s_hist_last_minute = 0;
    s_hist_alt_floor = -1.0f;
}

/** True when ADS-B reports airborne with AGL above taxi/ground clutter (takeoff latch). */
static bool follow_ac_is_airborne(const Aircraft *ac) {
    if (!ac || ac->on_ground) {
        return false;
    }
    int elev = 0;
    if (s_follow_dep.ok) {
        elev = s_follow_dep.elev_ft;
    } else if (s_follow_arr.ok) {
        elev = s_follow_arr.elev_ft;
    }
    return (ac->alt_ft - (float)elev) >= 100.0f;
}

/** Altitude gate for "landed" (below 600 ft pressure/MSL). */
static constexpr float kFollowLandAltFt = 600.0f;

/** True when we should treat the flight as landed (freezes follow duration). */
static bool follow_ac_is_landed(const Aircraft *ac) {
    if (!ac) {
        return true;
    }
    if (ac->on_ground) {
        return true;
    }
    // ADS-B often clears on_ground late; alt < 600 ft ⇒ assume down / short final done.
    if (ac->alt_ft > 0.0f && ac->alt_ft < kFollowLandAltFt) {
        return true;
    }
    return false;
}

/** Latch takeoff on first airborne sample; freeze duration when landed. */
static void follow_note_airborne_sample(const Aircraft *ac) {
    if (!ac) {
        return;
    }
    // Clearly airborne again (go-around / bounce) — clear landed latch.
    if (follow_ac_is_airborne(ac) && ac->alt_ft >= kFollowLandAltFt) {
        if (s_follow_takeoff_ms == 0) {
            s_follow_takeoff_ms = millis();
        }
        s_follow_land_ms = 0;
        return;
    }
    // First airborne sample can be below 600 ft (initial climb) — still latch takeoff.
    if (follow_ac_is_airborne(ac)) {
        if (s_follow_takeoff_ms == 0) {
            s_follow_takeoff_ms = millis();
        }
        return;
    }
    if (s_follow_takeoff_ms != 0 && s_follow_land_ms == 0 && follow_ac_is_landed(ac)) {
        s_follow_land_ms = millis();
    }
}

static void hist_push_at(float alt_ft, float gs_kts, uint32_t ms, uint32_t t_unix) {
    if (alt_ft < 0.0f) {
        alt_ft = 0.0f;
    }
    if (gs_kts < 0.0f) {
        gs_kts = 0.0f;
    }
    s_hist[s_hist_head].alt_ft = alt_ft;
    s_hist[s_hist_head].gs_kts = gs_kts;
    s_hist[s_hist_head].ms = ms;
    s_hist[s_hist_head].t_unix = t_unix;
    s_hist_head = (s_hist_head + 1) % kHistMax;
    if (s_hist_count < kHistMax) {
        s_hist_count++;
    }
}

/** Live ADS-B sample: at most one new chart point per UTC minute (update in-place). */
static void hist_push(float alt_ft, float gs_kts) {
    if (alt_ft < 0.0f) {
        alt_ft = 0.0f;
    }
    if (gs_kts < 0.0f) {
        gs_kts = 0.0f;
    }
    const uint32_t now_ms = millis();
    const time_t now_u = time(nullptr);
    uint32_t t_unix = 0;
    uint32_t minute = 0;
    if (now_u >= (time_t)1700000000L) {
        t_unix = (uint32_t)now_u;
        minute = t_unix / 60u;
        if (s_hist_count > 0 && s_hist_last_minute == minute) {
            const int last = (s_hist_head - 1 + kHistMax) % kHistMax;
            s_hist[last].alt_ft = alt_ft;
            s_hist[last].gs_kts = gs_kts;
            s_hist[last].ms = now_ms;
            s_hist[last].t_unix = t_unix;
            hist_maybe_lock_climb_t0(alt_ft, t_unix);
            return;
        }
        s_hist_last_minute = minute;
    } else if (s_hist_count > 0) {
        const int last = (s_hist_head - 1 + kHistMax) % kHistMax;
        const float da = alt_ft - s_hist[last].alt_ft;
        const float ds = gs_kts - s_hist[last].gs_kts;
        if (fabsf(da) < 5.0f && fabsf(ds) < 1.0f && s_hist_count > 2) {
            s_hist[last].alt_ft = alt_ft;
            s_hist[last].gs_kts = gs_kts;
            s_hist[last].ms = now_ms;
            return;
        }
    }
    hist_push_at(alt_ft, gs_kts, now_ms, t_unix);
    if (t_unix) {
        hist_maybe_lock_climb_t0(alt_ft, t_unix);
    }
}

/**
 * Chart left edge = first climb off the taxi/ground plateau (not gate-out / dep Act).
 * Returns unix of the first sample where altitude clearly starts increasing.
 */
static uint32_t hist_climb_start_unix(const FlightTrackPt *pts, int n) {
    if (!pts || n < 2) {
        return (pts && n == 1) ? pts[0].t_unix : 0;
    }
    float ground = pts[0].alt_ft;
    const int scan = (n < 45) ? n : 45;  // early flight / taxi window
    for (int i = 0; i < scan; i++) {
        if (pts[i].alt_ft < ground) {
            ground = pts[i].alt_ft;
        }
    }
    if (ground < 0.0f) {
        ground = 0.0f;
    }
    for (int i = 1; i < n; i++) {
        const float prev = pts[i - 1].alt_ft;
        const float cur = pts[i].alt_ft;
        // Still on the plateau — keep watching.
        if (cur <= ground + 80.0f) {
            if (cur < ground) {
                ground = cur;
            }
            continue;
        }
        // Altitude starting up from near-ground: lock this sample to chart left.
        if (prev <= ground + 150.0f && cur >= prev + 50.0f) {
            return pts[i].t_unix;
        }
        if (prev < 200.0f && cur >= 200.0f && cur > prev) {
            return pts[i].t_unix;
        }
    }
    for (int i = 0; i < n; i++) {
        if (pts[i].alt_ft >= 200.0f) {
            return pts[i].t_unix;
        }
    }
    return pts[0].t_unix;
}

/** Drop taxi / pre-climb samples so chart left is clean at climb lock. */
static void hist_trim_before_unix(uint32_t t0) {
    if (!t0 || s_hist_count <= 0) {
        return;
    }
    while (s_hist_count > 0) {
        const int oldest = (s_hist_head - s_hist_count + kHistMax) % kHistMax;
        const uint32_t u = s_hist[oldest].t_unix;
        if (u >= 1700000000u && u + 30u < t0) {
            s_hist_count--;
            continue;
        }
        break;
    }
}

/** Lock chart t0 once live altitude begins climbing off the floor. */
static void hist_maybe_lock_climb_t0(float alt_ft, uint32_t t_unix) {
    if (s_hist_fa_t0_unix || !t_unix) {
        return;
    }
    if (s_hist_alt_floor < 0.0f) {
        s_hist_alt_floor = alt_ft;
        return;
    }
    if (alt_ft < s_hist_alt_floor) {
        s_hist_alt_floor = alt_ft;
        return;
    }
    if (alt_ft >= s_hist_alt_floor + 100.0f) {
        s_hist_fa_t0_unix = t_unix;
        hist_trim_before_unix(t_unix);
        if (follow_chart) {
            lv_obj_invalidate(follow_chart);
        }
    }
}

/** Seed chart from FA track: 1-minute alt/gs from climb-start through current time. */
static void hist_seed_from_fa_track(const FlightTrackPt *pts, int n) {
    if (!pts || n < 2) {
        return;
    }
    const time_t now_u = time(nullptr);
    const uint32_t now_ms = millis();
    if (now_u < (time_t)1700000000L) {
        return;
    }

    const uint32_t t_takeoff = hist_climb_start_unix(pts, n);
    const uint32_t t_end = pts[n - 1].t_unix;
    // Already seeded through this FA end — keep live 1-min appends.
    if (s_hist_from_fa && t_end <= s_hist_fa_t_end_unix + 30u && s_hist_count >= 2) {
        return;
    }

    hist_clear();
    s_hist_from_fa = true;
    s_hist_fa_t0_unix = t_takeoff ? t_takeoff : pts[0].t_unix;
    s_hist_fa_t_end_unix = t_end;
    s_hist_alt_floor = -1.0f;  // climb already locked from FA

    uint32_t last_minute = 0;
    bool have_min = false;
    for (int i = 0; i < n; i++) {
        const FlightTrackPt &pt = pts[i];
        if (t_takeoff && pt.t_unix + 30u < t_takeoff) {
            continue;  // taxi / before climb — omit from chart
        }
        if (pt.t_unix > (uint32_t)now_u + 60u) {
            break;  // future / clock skew
        }
        const uint32_t minute = pt.t_unix / 60u;
        if (have_min && minute == last_minute) {
            const int last = (s_hist_head - 1 + kHistMax) % kHistMax;
            long age_ms = (long)(now_u - (time_t)pt.t_unix) * 1000L;
            if (age_ms < 0) {
                age_ms = 0;
            }
            uint32_t ms = now_ms;
            if ((uint32_t)age_ms < now_ms) {
                ms = now_ms - (uint32_t)age_ms;
            } else {
                ms = 0;
            }
            s_hist[last].alt_ft = pt.alt_ft < 0.0f ? 0.0f : pt.alt_ft;
            s_hist[last].gs_kts = pt.gs_kts < 0.0f ? 0.0f : pt.gs_kts;
            s_hist[last].ms = ms;
            s_hist[last].t_unix = pt.t_unix;
            continue;
        }
        last_minute = minute;
        have_min = true;
        long age_ms = (long)(now_u - (time_t)pt.t_unix) * 1000L;
        if (age_ms < 0) {
            age_ms = 0;
        }
        uint32_t ms = now_ms;
        if ((uint32_t)age_ms < now_ms) {
            ms = now_ms - (uint32_t)age_ms;
        } else {
            ms = 0;
        }
        hist_push_at(pt.alt_ft, pt.gs_kts, ms, pt.t_unix);
    }
    if (s_hist_count > 0) {
        const int newest = (s_hist_head - 1 + kHistMax) % kHistMax;
        if (s_hist[newest].t_unix) {
            s_hist_last_minute = s_hist[newest].t_unix / 60u;
        }
    }
    if (follow_chart) {
        lv_obj_invalidate(follow_chart);
    }
}

static void follow_try_seed_hist_from_state() {
    if (!g_state || !s_follow) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const int n = (g_state->flight_track_n > FR_MAX_TRACK_PTS) ? FR_MAX_TRACK_PTS : g_state->flight_track_n;
    if (n >= 2) {
        hist_seed_from_fa_track(g_state->flight_track, n);
    }
    xSemaphoreGive(g_state->mutex);
}

static uint32_t hist_span_ms() {
    if (s_hist_count < 1) {
        return 0;
    }
    const int oldest = (s_hist_head - s_hist_count + kHistMax) % kHistMax;
    const int newest = (s_hist_head - 1 + kHistMax) % kHistMax;
    const uint32_t a = s_hist[oldest].ms;
    const uint32_t b = s_hist[newest].ms;
    return (b >= a) ? (b - a) : 0;
}

static uint32_t hist_window_ms() {
    const uint32_t span = hist_span_ms();
    return (span > kHistMinWindowMs) ? span : kHistMinWindowMs;
}

static void format_hist_window_label(char *out, size_t n, uint32_t window_ms) {
    if (!out || n == 0) {
        return;
    }
    const uint32_t total_sec = (window_ms + 500u) / 1000u;
    const uint32_t mins = total_sec / 60u;
    const uint32_t secs = total_sec % 60u;
    if (mins >= 60u) {
        const uint32_t hrs = mins / 60u;
        const uint32_t rem = mins % 60u;
        snprintf(out, n, "%lu h %02lu m", (unsigned long)hrs, (unsigned long)rem);
    } else if (secs == 0u || mins >= 5u) {
        snprintf(out, n, "%lu min", (unsigned long)mins);
    } else {
        snprintf(out, n, "%lu:%02lu", (unsigned long)mins, (unsigned long)secs);
    }
}

static void trail_push(float lat, float lon) {
    if (s_trail_count > 0) {
        const int last = (s_trail_head - 1 + kTrailMax) % kTrailMax;
        const float dlat = lat - s_trail[last].lat;
        const float dlon = lon - s_trail[last].lon;
        // Skip tiny moves (~0.03 nm) so the polyline stays readable.
        if ((dlat * dlat + dlon * dlon) < 2.5e-7f) {
            return;
        }
    }
    s_trail[s_trail_head].lat = lat;
    s_trail[s_trail_head].lon = lon;
    s_trail_head = (s_trail_head + 1) % kTrailMax;
    if (s_trail_count < kTrailMax) {
        s_trail_count++;
    }
}

/** Civil TZ offset from lat/lon with US/EU daylight saving (shared with net_client). */
static int tz_offset_for_ll(float lat, float lon) {
    return net_tz_offset_for_ll(lat, lon, 0);
}

static void recalc_tz_offset() {
    float lon = 0.0f;
    float lat = 0.0f;
    if (s_home_ok) {
        lon = s_home_loc.lon;
        lat = s_home_loc.lat;
    } else if (g_state && g_state->cfg.location.valid) {
        lon = g_state->cfg.location.lon;
        lat = g_state->cfg.location.lat;
    } else {
        lon = s_olon;
        lat = s_olat;
    }
    s_tz_off_sec = tz_offset_for_ll(lat, lon);
}

static void realign_radar_ur_labels() {
    if (!radar_plot) {
        return;
    }
    // Place / track name and clock both flush to the radar's right edge.
    if (radar_place_lbl) {
        lv_obj_set_style_text_align(radar_place_lbl, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align_to(radar_place_lbl, radar_plot, LV_ALIGN_TOP_RIGHT, -8, 8);
    }
    if (radar_clock) {
        if (radar_place_lbl) {
            lv_obj_align_to(radar_clock, radar_place_lbl, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 4);
        } else {
            lv_obj_align_to(radar_clock, radar_plot, LV_ALIGN_TOP_RIGHT, -8, 28);
        }
    }
}

/** Minimal line-drawn analog clock: outer ring, 12 hour ticks, H/M/S hands. */
static void analog_clock_paint(lv_draw_ctx_t *ctx, const lv_area_t *coords, int h, int m, int s,
                               bool valid) {
    if (!ctx || !coords) {
        return;
    }
    const lv_coord_t cx = (coords->x1 + coords->x2) / 2;
    const lv_coord_t cy = (coords->y1 + coords->y2) / 2;
    const lv_coord_t R = (LV_MIN(lv_area_get_width(coords), lv_area_get_height(coords)) / 2) - 1;

    lv_draw_rect_dsc_t ring;
    lv_draw_rect_dsc_init(&ring);
    ring.bg_opa = LV_OPA_TRANSP;
    ring.border_color = COL_DIM;
    ring.border_width = 1;
    ring.border_opa = LV_OPA_COVER;
    ring.radius = LV_RADIUS_CIRCLE;
    lv_area_t ra = {(lv_coord_t)(cx - R), (lv_coord_t)(cy - R), (lv_coord_t)(cx + R), (lv_coord_t)(cy + R)};
    lv_draw_rect(ctx, &ring, &ra);

    lv_draw_line_dsc_t ln;
    lv_draw_line_dsc_init(&ln);
    ln.color = COL_TEXT;
    ln.opa = LV_OPA_COVER;
    ln.width = 1;
    ln.round_start = 1;
    ln.round_end = 1;
    for (int i = 0; i < 12; i++) {
        const float ang = (float)i * 30.0f * FR_DEG2RAD;
        const float fx = sinf(ang);
        const float fy = -cosf(ang);
        const float outer = (float)R - 1.0f;
        const float inner = (float)R - ((i % 3 == 0) ? 9.0f : 6.0f);
        lv_point_t a = {(lv_coord_t)lroundf((float)cx + fx * outer),
                        (lv_coord_t)lroundf((float)cy + fy * outer)};
        lv_point_t b = {(lv_coord_t)lroundf((float)cx + fx * inner),
                        (lv_coord_t)lroundf((float)cy + fy * inner)};
        lv_draw_line(ctx, &ln, &a, &b);
    }

    if (!valid) {
        return;
    }

    auto hand = [&](float deg, float len, lv_coord_t width, lv_color_t col) {
        const float ang = deg * FR_DEG2RAD;
        const float fx = sinf(ang);
        const float fy = -cosf(ang);
        lv_point_t a = {cx, cy};
        lv_point_t b = {(lv_coord_t)lroundf((float)cx + fx * len),
                        (lv_coord_t)lroundf((float)cy + fy * len)};
        ln.width = width;
        ln.color = col;
        lv_draw_line(ctx, &ln, &a, &b);
    };
    const float hour_deg = 30.0f * (float)(h % 12) + 0.5f * (float)m;
    const float min_deg = 6.0f * (float)m + 0.1f * (float)s;
    const float sec_deg = 6.0f * (float)s;
    hand(hour_deg, (float)R * 0.48f, 2, COL_TEXT);
    hand(min_deg, (float)R * 0.70f, 2, COL_GREEN);
    hand(sec_deg, (float)R * 0.82f, 1, COL_DIM);

    lv_draw_rect_dsc_t hub;
    lv_draw_rect_dsc_init(&hub);
    hub.bg_color = COL_GREEN;
    hub.bg_opa = LV_OPA_COVER;
    hub.radius = LV_RADIUS_CIRCLE;
    lv_area_t ha = {(lv_coord_t)(cx - 2), (lv_coord_t)(cy - 2), (lv_coord_t)(cx + 2), (lv_coord_t)(cy + 2)};
    lv_draw_rect(ctx, &hub, &ha);
}

static void radar_clock_draw(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    analog_clock_paint(ctx, &coords, s_clk_h, s_clk_m, s_clk_s, s_clk_valid);
}

static void board_clock_draw(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    analog_clock_paint(ctx, &coords, s_board_clk_h, s_board_clk_m, s_board_clk_s, s_board_clk_valid);
}

static void refresh_board_clock_time() {
    time_t now = time(nullptr);
    if (now < 1700000000 || !s_board_ll_ok) {
        s_board_clk_valid = false;
        if (board_clock) {
            lv_obj_invalidate(board_clock);
        }
        return;
    }
    now += (time_t)tz_offset_for_ll(s_board_lat, s_board_lon);
    struct tm t;
    gmtime_r(&now, &t);
    s_board_clk_h = t.tm_hour;
    s_board_clk_m = t.tm_min;
    s_board_clk_s = t.tm_sec;
    s_board_clk_valid = true;
    if (board_clock) {
        lv_obj_invalidate(board_clock);
    }
}

static void board_set_airport_ll(const char *icao) {
    s_board_ll_ok = false;
    s_board_icao[0] = 0;
    s_board_lat = 0.0f;
    s_board_lon = 0.0f;
    if (!icao || !icao[0]) {
        refresh_board_clock_time();
        return;
    }
    strncpy(s_board_icao, icao, sizeof(s_board_icao) - 1);
    s_board_icao[sizeof(s_board_icao) - 1] = 0;
    if (net_lookup_airport(icao, &s_board_lat, &s_board_lon, nullptr, nullptr)) {
        s_board_ll_ok = true;
    } else {
        char resolved[8] = {0};
        if (net_lookup_airport_iata(icao, resolved, sizeof(resolved), &s_board_lat, &s_board_lon)) {
            s_board_ll_ok = true;
            if (resolved[0]) {
                strncpy(s_board_icao, resolved, sizeof(s_board_icao) - 1);
                s_board_icao[sizeof(s_board_icao) - 1] = 0;
            }
        }
    }
    // Fallback: last airport-search hits (have lat/lon even if DB missed).
    if (!s_board_ll_ok) {
        for (int i = 0; i < s_apt_hit_n; i++) {
            if (!s_apt_hits[i].valid) {
                continue;
            }
            if (strcasecmp(s_apt_hits[i].icao, icao) != 0 &&
                strcasecmp(s_apt_hits[i].iata, icao) != 0) {
                continue;
            }
            if (fabsf(s_apt_hits[i].lat) + fabsf(s_apt_hits[i].lon) > 0.01f) {
                s_board_lat = s_apt_hits[i].lat;
                s_board_lon = s_apt_hits[i].lon;
                s_board_ll_ok = true;
                if (s_apt_hits[i].icao[0]) {
                    strncpy(s_board_icao, s_apt_hits[i].icao, sizeof(s_board_icao) - 1);
                    s_board_icao[sizeof(s_board_icao) - 1] = 0;
                }
            }
            break;
        }
    }
    refresh_board_clock_time();
}

static void refresh_local_time_lbl() {
    time_t now = time(nullptr);
    // Unix ~2023-11: below this, NTP has not synced yet.
    if (now < 1700000000) {
        s_clk_valid = false;
        if (radar_clock) {
            lv_obj_invalidate(radar_clock);
        }
        refresh_board_clock_time();
        realign_radar_ur_labels();
        return;
    }
    // Recompute TZ periodically so DST transitions apply without reboot.
    static uint32_t s_tz_recalc_ms = 0;
    const uint32_t ms = millis();
    if (s_tz_recalc_ms == 0 || (ms - s_tz_recalc_ms) > 600000u) {
        recalc_tz_offset();
        s_tz_recalc_ms = ms;
    }
    now += (time_t)s_tz_off_sec;
    struct tm t;
    gmtime_r(&now, &t);
    s_clk_h = t.tm_hour;
    s_clk_m = t.tm_min;
    s_clk_s = t.tm_sec;
    s_clk_valid = true;
    if (radar_clock) {
        lv_obj_invalidate(radar_clock);
    }
    refresh_board_clock_time();
}

static void clock_timer_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    // Retry NTP in the background if boot sync missed (clocks stay frozen otherwise).
    static uint32_t s_ntp_retry_ms = 0;
    const time_t now = time(nullptr);
    if (now < 1700000000) {
        const uint32_t ms = millis();
        if (s_ntp_retry_ms == 0 || (ms - s_ntp_retry_ms) > 15000u) {
            s_ntp_retry_ms = ms;
            if (WiFi.status() == WL_CONNECTED) {
                configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
            }
        }
    }
    refresh_local_time_lbl();
}

/** Resolve aircraft lat/lon → US state and refresh county overlay when it changes.
 *  Over water / no state hit: keep the last known state's counties visible. */
static void follow_refresh_county_state(bool force_redraw) {
    if (!s_follow) {
        return;
    }
    float lat = 0, lon = 0;
    bool have = false;
    if (s_follow_ac_ok) {
        lat = s_follow_ac.lat;
        lon = s_follow_ac.lon;
        have = true;
    } else if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        if (g_state->tracked_ok) {
            lat = g_state->tracked_ac.lat;
            lon = g_state->tracked_ac.lon;
            have = true;
        }
        xSemaphoreGive(g_state->mutex);
    }
    if (!have) {
        return;
    }
    const int st = state_index_from_ll(lat, lon);
    // Over water / outside a US state — do not clear the last state's counties.
    if (st < 0) {
        if (s_follow_county_state >= 0) {
            if (force_redraw && s_show_counties) {
                map_cache_mark_dirty();
                if (radar_plot) {
                    lv_obj_invalidate(radar_plot);
                }
            }
            return;
        }
        // Never had a state yet — stay at unknown.
        if (s_follow_county_state == st && !force_redraw) {
            return;
        }
        s_follow_county_state = st;
        if (force_redraw) {
            refresh_follow_sched_col();
            layout_follow_info_boxes();
        }
        return;
    }
    if (st == s_follow_county_state && !force_redraw) {
        return;
    }
    const bool changed = (st != s_follow_county_state);
    s_follow_county_state = st;
    if (s_show_counties && (changed || force_redraw)) {
        map_cache_mark_dirty();
        if (radar_plot) {
            lv_obj_invalidate(radar_plot);
        }
    }
    // Keep "State: …" on the schedule card in sync.
    if (changed || force_redraw) {
        refresh_follow_sched_col();
        layout_follow_info_boxes();
    }
}

static void follow_state_timer_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    if (!s_follow) {
        return;
    }
    follow_refresh_county_state(false);
    if (s_follow_ac_ok) {
        follow_maybe_approach_zoom(&s_follow_ac);
    }
}

static void follow_state_timer_ensure(bool on) {
    if (on) {
        if (!s_follow_state_timer) {
            // Every 5s re-check aircraft position → state for the county layer.
            s_follow_state_timer = lv_timer_create(follow_state_timer_cb, 5000, nullptr);
        }
        follow_refresh_county_state(true);
    } else if (s_follow_state_timer) {
        lv_timer_del(s_follow_state_timer);
        s_follow_state_timer = nullptr;
    }
}

// Place/track name in the upper-right (no coordinates).
static void update_loc_label(const char *label, float lat, float lon) {
    LV_UNUSED(lat);
    LV_UNUSED(lon);
    if (!radar_place_lbl) {
        return;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "%s", label && label[0] ? label : "Local Radar");
    lv_label_set_text(radar_place_lbl, buf);
    realign_radar_ur_labels();
}

static void set_radar_center(float lat, float lon, const char *label, uint32_t extra_req) {
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.location.lat = lat;
    g_state->cfg.location.lon = lon;
    g_state->cfg.location.valid = true;
    if (label && label[0]) {
        strncpy(g_state->cfg.location.label, label, sizeof(g_state->cfg.location.label) - 1);
        g_state->cfg.location.label[sizeof(g_state->cfg.location.label) - 1] = 0;
    }
    xSemaphoreGive(g_state->mutex);

    s_olat = lat;
    s_olon = lon;
    map_cache_mark_dirty();
    update_loc_label(label, lat, lon);

    if (extra_req) {
        request(extra_req);
    }
}

static void follow_waypoints_clear() {
    memset(s_follow_wpts, 0, sizeof(s_follow_wpts));
    s_follow_wpt_n = 0;
}

static float wpt_dist_nm(float lat1, float lon1, float lat2, float lon2) {
    const float mid_lat = (lat1 + lat2) * 0.5f;
    float cos_lat = cosf(mid_lat * FR_DEG2RAD);
    if (cos_lat < 0.15f) {
        cos_lat = 0.15f;
    }
    const float dlat = (lat2 - lat1) * 60.0f;
    const float dlon = (lon2 - lon1) * 60.0f * cos_lat;
    return sqrtf(dlat * dlat + dlon * dlon);
}

/** Perpendicular distance (nm) of C from the A→B chord (flat-earth approx). */
static float wpt_cross_track_nm(float alat, float alon, float blat, float blon, float clat, float clon) {
    const float mid_lat = (alat + blat) * 0.5f;
    float cos_lat = cosf(mid_lat * FR_DEG2RAD);
    if (cos_lat < 0.15f) {
        cos_lat = 0.15f;
    }
    const float abx = (blon - alon) * 60.0f * cos_lat;
    const float aby = (blat - alat) * 60.0f;
    const float acx = (clon - alon) * 60.0f * cos_lat;
    const float acy = (clat - alat) * 60.0f;
    const float ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-4f) {
        return wpt_dist_nm(alat, alon, clat, clon);
    }
    return fabsf(acx * aby - acy * abx) / sqrtf(ab2);
}

/**
 * Drop intermediate fixes that sit way off the path of their neighbors
 * (bad duplicate-name lookups, bogus lat/lon tokens). Endpoints are kept.
 */
static void follow_waypoints_prune_outliers() {
    constexpr float kXtNm = 45.0f;       // cross-track vs neighbor chord
    constexpr float kDetourNm = 70.0f;   // extra path length vs skipping the point
    constexpr float kXtHardNm = 100.0f;  // extreme deviation alone is enough

    bool changed = true;
    while (changed && s_follow_wpt_n > 2) {
        changed = false;
        int worst = -1;
        float worst_score = 0.0f;
        for (int i = 1; i < s_follow_wpt_n - 1; i++) {
            const FollowWpt &prev = s_follow_wpts[i - 1];
            const FollowWpt &cur = s_follow_wpts[i];
            const FollowWpt &next = s_follow_wpts[i + 1];
            if (!prev.ok || !cur.ok || !next.ok) {
                continue;
            }
            const float xt =
                wpt_cross_track_nm(prev.lat, prev.lon, next.lat, next.lon, cur.lat, cur.lon);
            const float d_skip = wpt_dist_nm(prev.lat, prev.lon, next.lat, next.lon);
            const float d_via = wpt_dist_nm(prev.lat, prev.lon, cur.lat, cur.lon) +
                                wpt_dist_nm(cur.lat, cur.lon, next.lat, next.lon);
            const float detour = d_via - d_skip;
            const bool bad = (xt >= kXtHardNm) || (xt >= kXtNm && detour >= kDetourNm) ||
                             (d_skip > 1.0f && d_via > 3.0f * d_skip && xt >= 25.0f);
            if (bad && xt >= worst_score) {
                worst_score = xt;
                worst = i;
            }
        }
        if (worst < 0) {
            break;
        }
        for (int j = worst; j < s_follow_wpt_n - 1; j++) {
            s_follow_wpts[j] = s_follow_wpts[j + 1];
        }
        s_follow_wpt_n--;
        changed = true;
    }
}

static void follow_waypoints_rebuild() {
    follow_waypoints_clear();
    auto push_wpt = [&](float lat, float lon, const char *label) {
        if (s_follow_wpt_n >= kFollowWptMax || !label || !label[0]) {
            return;
        }
        FollowWpt &w = s_follow_wpts[s_follow_wpt_n++];
        w.lat = lat;
        w.lon = lon;
        strncpy(w.label, label, sizeof(w.label) - 1);
        w.label[sizeof(w.label) - 1] = 0;
        w.ok = true;
    };

    // Prefer filed IFR waypoints from FlightAware flight data.
    if (s_follow_fi.route_wpt_n > 0) {
        for (int i = 0; i < s_follow_fi.route_wpt_n; i++) {
            const RouteWaypoint &rw = s_follow_fi.route_wpts[i];
            if (!rw.ok) {
                continue;
            }
            push_wpt(rw.lat, rw.lon, rw.name[0] ? rw.name : "WPT");
        }
        follow_waypoints_prune_outliers();
        return;
    }

    // Fallback: dep / arr only (no synthetic GC stubs).
    if (s_follow_dep.ok) {
        push_wpt(s_follow_dep.lat, s_follow_dep.lon, s_follow_dep.icao[0] ? s_follow_dep.icao : "DEP");
    }
    if (s_follow_arr.ok) {
        push_wpt(s_follow_arr.lat, s_follow_arr.lon, s_follow_arr.icao[0] ? s_follow_arr.icao : "ARR");
    }
}

static void follow_apt_clear() {
    memset(&s_follow_dep, 0, sizeof(s_follow_dep));
    memset(&s_follow_arr, 0, sizeof(s_follow_arr));
    s_follow_dep_rwy_n = 0;
    s_follow_arr_rwy_n = 0;
    follow_waypoints_clear();
    s_follow_route_fitted = false;
}

static bool aircraft_is_helicopter(const Aircraft *ac);

/** Once per track: heli or <100 kt → 40 nm / FL100 / counties on; else keep default follow view. */
static void follow_maybe_apply_speed_view(const Aircraft *ac) {
    if (!s_follow || s_follow_spd_view_done || !ac) {
        return;
    }
    const bool heli = aircraft_is_helicopter(ac);
    float gs = ac->gs_kts;
    if (ac->on_ground) {
        gs = 0.0f;
    }
    // Helis always use the close-in view; planes wait for a real GS sample.
    if (!heli && isnan(gs)) {
        return;
    }
    s_follow_spd_view_done = true;
    if (!heli && gs >= 100.0f) {
        return;  // leave range / ceiling / overlays as already set
    }
    set_range(40.0f);
    set_max_alt(10000);
    if (!s_follow_county_user_hid) {
        s_show_counties = true;
        if (g_state) {
            xSemaphoreTake(g_state->mutex, portMAX_DELAY);
            g_state->cfg.show_counties = true;
            xSemaphoreGive(g_state->mutex);
        }
        update_cty_toggle_label();
        map_cache_mark_dirty();
    }
    // Keep close-in view; don't zoom out to full dep–arr span.
    s_follow_route_fitted = true;
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

/** Approach auto-zoom: as aircraft nears arrival, step PPI in to 60 → 40 → 25 → 15 → 5 → 4 nm. */
static void follow_maybe_approach_zoom(const Aircraft *ac) {
    if (!s_follow || !ac || !s_follow_arr.ok || s_follow_approach_zoom_off) {
        return;
    }
    float east = 0, north = 0;
    ll_to_nm(s_follow_arr.lat, s_follow_arr.lon, ac->lat, ac->lon, &east, &north);
    const float nm = hypotf(east, north);
    float want = 0.0f;
    if (nm <= 4.0f) {
        want = 4.0f;
    } else if (nm <= 5.0f) {
        want = 5.0f;
    } else if (nm <= 15.0f) {
        want = 15.0f;
    } else if (nm <= 25.0f) {
        want = 25.0f;
    } else if (nm <= 40.0f) {
        want = 40.0f;
    } else if (nm <= 60.0f) {
        want = 60.0f;
    } else {
        return;
    }
    // Only zoom in — never widen the view for approach.
    if (s_range <= want + 0.5f) {
        return;
    }
    // Snap PPI onto the aircraft so the approach stays framed.
    s_follow_hold_center = false;
    char label[48];
    snprintf(label, sizeof(label), "Following %s",
             s_follow_label[0] ? s_follow_label : (s_follow_hex[0] ? s_follow_hex : "flight"));
    set_radar_center(ac->lat, ac->lon, label, 0);
    set_range(want);
}

static void follow_path_center_timer_cancel() {
    if (s_follow_path_center_timer) {
        lv_timer_del(s_follow_path_center_timer);
        s_follow_path_center_timer = nullptr;
    }
}

/** After path overview, switch to Center (aircraft-locked) view. */
static void follow_path_center_timer_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    s_follow_path_center_timer = nullptr;
    if (!s_follow) {
        return;
    }
    // Still on the fitted path overview (hold_center). Skip if user already centered.
    if (!s_follow_hold_center || !s_follow_route_fitted) {
        return;
    }
    center_on_flight(nullptr);
}

static void follow_path_center_timer_arm() {
    follow_path_center_timer_cancel();
    s_follow_path_center_timer = lv_timer_create(follow_path_center_timer_cb, 15000, nullptr);
    lv_timer_set_repeat_count(s_follow_path_center_timer, 1);
}

/** Center + zoom so dep and arr airports (and aircraft if known) fit in the PPI. */
static void follow_fit_dep_arr_view() {
    if (!s_follow || !s_follow_dep.ok || !s_follow_arr.ok) {
        return;
    }
    const float clat = (s_follow_dep.lat + s_follow_arr.lat) * 0.5f;
    const float clon = (s_follow_dep.lon + s_follow_arr.lon) * 0.5f;

    auto dist_from_center = [&](float lat, float lon) -> float {
        float east = 0, north = 0;
        ll_to_nm(lat, lon, clat, clon, &east, &north);
        return hypotf(east, north);
    };

    float need = dist_from_center(s_follow_dep.lat, s_follow_dep.lon);
    const float d_arr = dist_from_center(s_follow_arr.lat, s_follow_arr.lon);
    if (d_arr > need) {
        need = d_arr;
    }
    if (s_follow_ac_ok) {
        const float d_ac = dist_from_center(s_follow_ac.lat, s_follow_ac.lon);
        if (d_ac > need) {
            need = d_ac;
        }
    }
    need *= 1.12f;  // keep marks inside the rim
    if (need < 15.0f) {
        need = 15.0f;
    }

    float zoom = kZoomFarMax;
    bool found = false;
    for (int i = 0; i < kZoomNearN; i++) {
        if (kZoomNear[i] >= need) {
            zoom = kZoomNear[i];
            found = true;
            break;
        }
    }
    if (!found) {
        for (float r = 500.0f; r <= kZoomFarMax + 0.5f; r += kZoomFarStep) {
            if (r >= need) {
                zoom = r;
                found = true;
                break;
            }
        }
        if (!found) {
            zoom = kZoomFarMax;
        }
    }

    char label[48];
    snprintf(label, sizeof(label), "Following %s",
             s_follow_label[0] ? s_follow_label : (s_follow_hex[0] ? s_follow_hex : "flight"));
    // Hold center so traffic updates don't snap back onto the aircraft.
    s_follow_hold_center = true;
    sync_follow_to_state();
    // Traffic only — airports/counties stay hidden while tracking.
    set_radar_center(clat, clon, label, UI_REQ_REFRESH_TRAFFIC);
    set_range(zoom);
    s_follow_route_fitted = true;
    // Show full flight path briefly, then auto-Center on the aircraft.
    follow_path_center_timer_arm();
}

static void follow_apt_load_rwys(FollowApt *apt, Runway *rwys, int *rwy_n) {
    if (!apt || !rwys || !rwy_n) {
        return;
    }
    *rwy_n = 0;
    if (apt->ok && apt->icao[0]) {
        *rwy_n = net_lookup_runways(apt->icao, rwys, FR_MAX_RWYS_PER_APT);
    }
}

static void follow_apt_resolve(FollowApt *apt, const char *icao, const char *name) {
    if (!apt || !icao || !icao[0]) {
        return;
    }
    // Already locked in for this follow session.
    if (apt->ok && strcasecmp(apt->icao, icao) == 0) {
        if (name && name[0] && !apt->name[0]) {
            strncpy(apt->name, name, sizeof(apt->name) - 1);
        }
        return;
    }
    strncpy(apt->icao, icao, sizeof(apt->icao) - 1);
    apt->icao[sizeof(apt->icao) - 1] = 0;
    apt->elev_ft = 0;
    if (name && name[0]) {
        strncpy(apt->name, name, sizeof(apt->name) - 1);
        apt->name[sizeof(apt->name) - 1] = 0;
    }
    float lat = 0, lon = 0;
    int elev = 0;
    char resolved[8] = {0};
    if (net_lookup_airport(icao, &lat, &lon, nullptr, &elev)) {
        apt->lat = lat;
        apt->lon = lon;
        apt->elev_ft = elev;
        apt->ok = true;
    } else if (net_lookup_airport_iata(icao, resolved, sizeof(resolved), &lat, &lon)) {
        if (resolved[0]) {
            strncpy(apt->icao, resolved, sizeof(apt->icao) - 1);
            net_lookup_airport(apt->icao, nullptr, nullptr, nullptr, &elev);
            apt->elev_ft = elev;
        }
        apt->lat = lat;
        apt->lon = lon;
        apt->ok = true;
    } else {
        apt->ok = false;
    }
    if (apt == &s_follow_dep) {
        follow_apt_load_rwys(apt, s_follow_dep_rwys, &s_follow_dep_rwy_n);
        map_plates_mark_dirty();
    } else if (apt == &s_follow_arr) {
        follow_apt_load_rwys(apt, s_follow_arr_rwys, &s_follow_arr_rwy_n);
        map_plates_mark_dirty();
    }
}

static void follow_apt_update_from_fi(const FlightInfo *fi) {
    if (!fi) {
        return;
    }
    if (fi->origin[0]) {
        follow_apt_resolve(&s_follow_dep, fi->origin, fi->origin_name);
    }
    if (fi->dest[0]) {
        follow_apt_resolve(&s_follow_arr, fi->dest, fi->dest_name);
    }
    // Mirror persistent codes back into follow FlightInfo so panel/sched never lose them.
    if (s_follow_dep.icao[0] && !s_follow_fi.origin[0]) {
        strncpy(s_follow_fi.origin, s_follow_dep.icao, sizeof(s_follow_fi.origin) - 1);
        if (s_follow_dep.name[0]) {
            strncpy(s_follow_fi.origin_name, s_follow_dep.name, sizeof(s_follow_fi.origin_name) - 1);
        }
    }
    if (s_follow_arr.icao[0] && !s_follow_fi.dest[0]) {
        strncpy(s_follow_fi.dest, s_follow_arr.icao, sizeof(s_follow_fi.dest) - 1);
        if (s_follow_arr.name[0]) {
            strncpy(s_follow_fi.dest_name, s_follow_arr.name, sizeof(s_follow_fi.dest_name) - 1);
        }
    }
    if (s_follow_fi.origin[0] && s_follow_fi.dest[0]) {
        s_follow_fi.route_ok = true;
    }
    follow_waypoints_rebuild();
    if (s_follow && s_follow_dep.ok && s_follow_arr.ok && !s_follow_route_fitted) {
        follow_fit_dep_arr_view();
    }
    if (radar_plot && (s_follow_dep.ok || s_follow_arr.ok || s_follow_wpt_n > 0)) {
        lv_obj_invalidate(radar_plot);
    }
}

static void stop_follow(bool restore_saved) {
    follow_state_timer_ensure(false);
    follow_path_center_timer_cancel();
    s_follow = false;
    s_follow_solo = false;
    s_follow_county_state = -2;
    s_follow_county_user_hid = false;
    s_wpt_view_mode = 0;
    s_follow_hold_center = false;
    s_follow_approach_zoom_off = false;
    s_follow_route_fitted = false;
    s_follow_spd_view_done = false;
    s_traffic_sel_row = -1;
    s_follow_hex[0] = 0;
    s_follow_label[0] = 0;
    s_follow_ac_ok = false;
    s_follow_takeoff_ms = 0;
    s_follow_land_ms = 0;
    memset(&s_follow_ac, 0, sizeof(s_follow_ac));
    memset(&s_follow_fi, 0, sizeof(s_follow_fi));
    follow_apt_clear();
    trail_clear();
    hist_clear();
    update_wpt_toggle_btn();
    set_follow_side_mode(false);
    // Drop ultra-zoom when leaving follow (before restore; restore overrides if saved).
    if (s_range > 250.0f + 0.5f) {
        set_range_ex(250.0f, false);
    }
    // Only restore the pre-follow view when explicitly requested (not on Home).
    if (restore_saved && s_follow_saved_view_ok) {
        set_range_ex(s_follow_saved_range, false);
        set_max_alt_ex(s_follow_saved_max_alt, true);
    }
    s_follow_saved_view_ok = false;
    sync_follow_to_state();
    if (restore_saved && s_follow_saved_ok && g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->cfg.location = s_follow_saved_loc;
        xSemaphoreGive(g_state->mutex);
        s_olat = s_follow_saved_loc.lat;
        s_olon = s_follow_saved_loc.lon;
        map_cache_mark_dirty();
        update_loc_label(s_follow_saved_loc.label, s_follow_saved_loc.lat, s_follow_saved_loc.lon);
        request(UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
    }
    s_follow_saved_ok = false;
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

/** Home / Unfollow: clear solo track, apply Startup Settings, recenter, full traffic. */
static void leave_follow_for_home() {
    follow_state_timer_ensure(false);
    follow_path_center_timer_cancel();
    s_follow = false;
    s_follow_solo = false;
    s_follow_county_state = -2;
    s_follow_county_user_hid = false;
    s_wpt_view_mode = 0;
    s_follow_hold_center = false;
    s_follow_approach_zoom_off = false;
    s_follow_route_fitted = false;
    s_follow_spd_view_done = false;
    s_traffic_sel_row = -1;
    s_follow_hex[0] = 0;
    s_follow_label[0] = 0;
    s_follow_ac_ok = false;
    s_follow_takeoff_ms = 0;
    s_follow_land_ms = 0;
    s_follow_saved_ok = false;
    s_follow_saved_view_ok = false;
    memset(&s_follow_ac, 0, sizeof(s_follow_ac));
    memset(&s_follow_fi, 0, sizeof(s_follow_fi));
    follow_apt_clear();
    trail_clear();
    hist_clear();
    update_wpt_toggle_btn();
    set_follow_side_mode(false);

    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->follow_active = false;
        g_state->follow_solo = false;
        g_state->follow_hold_center = false;
        g_state->follow_hex[0] = 0;
        g_state->follow_flight[0] = 0;
        g_state->tracked_ok = false;
        memset(&g_state->tracked_ac, 0, sizeof(g_state->tracked_ac));
        // Invalidate any in-flight solo ADS-B response and drop the 1-plane list.
        g_state->traffic_epoch++;
        g_state->aircraft_count = 0;
        xSemaphoreGive(g_state->mutex);
    }

    if (!s_home_ok) {
        capture_home_from_cfg();
    }
    restore_startup_view();

    if (s_home_ok) {
        // Restore full Settings home (lat/lon/icao/label), not a pan-ephemeral center.
        if (g_state) {
            xSemaphoreTake(g_state->mutex, portMAX_DELAY);
            g_state->cfg.location = s_home_loc;
            xSemaphoreGive(g_state->mutex);
        }
        set_radar_center(s_home_loc.lat, s_home_loc.lon, home_place_label(),
                         UI_REQ_SAVE_LOCATION | UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
    } else {
        request(UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
    }

    if (radar_status) {
        lv_label_set_text(radar_status, "Updating ADS-B...");
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void update_adsb_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (radar_status) {
        lv_label_set_text(radar_status, "Updating ADS-B…");
    }
    request(UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
}

static void capture_home_from_cfg() {
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    if (g_state->cfg.location.valid) {
        s_home_loc = g_state->cfg.location;
        s_home_ok = true;
    }
    xSemaphoreGive(g_state->mutex);
    if (s_home_ok) {
        recalc_tz_offset();
    }
}

static const char *home_place_label() {
    // Airport homes show ICAO only (never facility name / street address).
    if (s_home_ok && s_home_loc.icao[0]) {
        return s_home_loc.icao;
    }
    if (s_home_ok && s_home_loc.label[0]) {
        return s_home_loc.label;
    }
    if (g_state && g_state->cfg.location.icao[0]) {
        return g_state->cfg.location.icao;
    }
    if (g_state && g_state->cfg.location.label[0]) {
        return g_state->cfg.location.label;
    }
    return "Local Radar";
}

/** Apply Startup Settings (zoom / alt / overlays) — used on boot and Home.
 *  Does not request ADS-B; caller refreshes after recentering on home. */
static void restore_startup_view() {
    if (!g_state) {
        return;
    }
    float nm = 40.0f;
    int alt = 45000;
    bool apt = true, cty = true, cities = true;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    nm = g_state->cfg.startup_range_nm;
    alt = g_state->cfg.startup_max_alt_ft;
    apt = g_state->cfg.startup_show_airports;
    cty = g_state->cfg.startup_show_counties;
    cities = g_state->cfg.startup_show_cities;
    g_state->cfg.show_airports = apt;
    g_state->cfg.show_counties = cty;
    g_state->cfg.show_cities = cities;
    xSemaphoreGive(g_state->mutex);
    set_range_ex(nm, false);
    set_max_alt_ex(alt, false);
    s_show_airports = apt;
    s_show_counties = cty;
    s_show_cities = cities;
    update_apt_toggle_label();
    update_cty_toggle_label();
    update_city_toggle_label();
    map_cache_mark_dirty();
}

static void home_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_home_ok) {
        capture_home_from_cfg();
    }
    if (!s_home_ok) {
        if (radar_status) {
            lv_label_set_text(radar_status, "No home location");
        }
        return;
    }
    // Same path as Unfollow: Startup Settings + full local traffic at home.
    leave_follow_for_home();
}

static void center_on_flight(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_follow) {
        return;
    }
    follow_path_center_timer_cancel();
    s_follow_hold_center = false;
    s_follow_solo = true;  // lock back onto the tracked aircraft only
    sync_follow_to_state();
    float lat = 0, lon = 0;
    char label[48];
    if (s_follow_ac_ok) {
        lat = s_follow_ac.lat;
        lon = s_follow_ac.lon;
        snprintf(label, sizeof(label), "Following %s", s_follow_label[0] ? s_follow_label : s_follow_hex);
    } else if (s_follow_dep.ok) {
        lat = s_follow_dep.lat;
        lon = s_follow_dep.lon;
        snprintf(label, sizeof(label), "Following %s", s_follow_label[0] ? s_follow_label : "flight");
    } else {
        return;
    }
    set_radar_center(lat, lon, label, UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
    set_range(100.0f);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void update_follow_from_traffic() {
    if (!s_follow || (!s_follow_hex[0] && !s_follow_label[0])) {
        return;
    }
    const Aircraft *hit = nullptr;
    for (int i = 0; i < s_ac_n; i++) {
        if (s_follow_hex[0] && s_ac[i].hex[0] && strcasecmp(s_ac[i].hex, s_follow_hex) == 0) {
            hit = &s_ac[i];
            break;
        }
        if (!hit && s_follow_label[0] && s_ac[i].flight[0] &&
            strcasecmp(s_ac[i].flight, s_follow_label) == 0) {
            hit = &s_ac[i];
            // keep scanning in case a hex match appears later in the list
        }
    }
    if (!hit) {
        refresh_follow_panel();
        return;
    }
    s_follow_ac = *hit;
    s_follow_ac_ok = true;
    if (hit->hex[0] && !s_follow_hex[0]) {
        strncpy(s_follow_hex, hit->hex, sizeof(s_follow_hex) - 1);
        s_follow_hex[sizeof(s_follow_hex) - 1] = 0;
    }
    if (hit->flight[0]) {
        strncpy(s_follow_label, hit->flight, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
        trim_inplace(s_follow_label);
    }
    // Once FlightStats says departed (or ADS-B shows airborne), clear pre-dep pin flag locally.
    if (s_follow_fi.schedule_ok && !s_follow_fi.departed && !hit->on_ground && hit->alt_ft > 500.0f) {
        s_follow_fi.departed = true;
    }
    // Keep county-state in sync with the latest ADS-B fix (timer also polls every 5s).
    follow_refresh_county_state(false);
    sync_follow_to_state();
    trail_push(hit->lat, hit->lon);
    hist_push(hit->on_ground ? 0.0f : hit->alt_ft, hit->gs_kts);
    follow_note_airborne_sample(hit);
    follow_maybe_apply_speed_view(hit);
    follow_maybe_approach_zoom(hit);

    float east = 0, north = 0;
    ll_to_nm(hit->lat, hit->lon, s_olat, s_olon, &east, &north);
    const float moved_nm = hypotf(east, north);
    char label[48];
    snprintf(label, sizeof(label), "Following %s", s_follow_label[0] ? s_follow_label : s_follow_hex);
    // Recenter unless user panned with N/S/E/W; still refresh airports if it drifted far.
    const uint32_t req = (moved_nm > 8.0f) ? UI_REQ_REFRESH_AIRPORTS : 0;
    if (!s_follow_hold_center && (moved_nm > 0.05f || req)) {
        set_radar_center(hit->lat, hit->lon, label, req);
    } else if (s_follow_hold_center) {
        if (req) {
            request(req);
        }
        update_loc_label(label, s_olat, s_olon);
    } else {
        update_loc_label(label, hit->lat, hit->lon);
    }
    refresh_follow_panel();
}

/** True for ADS-B rotorcraft (A7) or common ICAO helicopter type codes. */
static bool aircraft_is_helicopter(const Aircraft *ac) {
    if (!ac) {
        return false;
    }
    const char *c = ac->category;
    if ((c[0] == 'A' || c[0] == 'a') && c[1] == '7') {
        return true;  // ADS-B emitter category A7 = rotorcraft
    }
    const char *t = ac->type;
    if (!t[0]) {
        return false;
    }
    char u[8];
    int n = 0;
    for (; n < 7 && t[n]; n++) {
        u[n] = (char)toupper((unsigned char)t[n]);
    }
    u[n] = 0;
    // Robinson
    if (!strncmp(u, "R22", 3) || !strncmp(u, "R44", 3) || !strncmp(u, "R66", 3)) {
        return true;
    }
    // Bell
    if (!strncmp(u, "B06", 3) || !strncmp(u, "B47", 3) || !strncmp(u, "B212", 4) || !strncmp(u, "B222", 4) ||
        !strncmp(u, "B230", 4) || !strncmp(u, "B407", 4) || !strncmp(u, "B412", 4) || !strncmp(u, "B427", 4) ||
        !strncmp(u, "B429", 4) || !strncmp(u, "B430", 4)) {
        return true;
    }
    // Airbus / Eurocopter / Aerospatiale
    if ((!strncmp(u, "EC", 2) && n > 2 && isdigit((unsigned char)u[2])) || !strncmp(u, "AS3", 3) ||
        !strncmp(u, "AS5", 3) || !strncmp(u, "AS6", 3) || !strncmp(u, "H125", 4) || !strncmp(u, "H130", 4) ||
        !strncmp(u, "H135", 4) || !strncmp(u, "H145", 4) || !strncmp(u, "H155", 4) || !strncmp(u, "H175", 4) ||
        !strncmp(u, "H225", 4)) {
        return true;
    }
    // Sikorsky / military
    if (!strncmp(u, "S61", 3) || !strncmp(u, "S64", 3) || !strncmp(u, "S70", 3) || !strncmp(u, "S76", 3) ||
        !strncmp(u, "S92", 3) || !strncmp(u, "H60", 3) || !strncmp(u, "MH60", 4) || !strncmp(u, "UH1", 3) ||
        !strncmp(u, "CH46", 4) || !strncmp(u, "CH47", 4) || !strncmp(u, "AH64", 4) || !strncmp(u, "H64", 3) ||
        !strncmp(u, "H53", 3)) {
        return true;
    }
    // AgustaWestland / MD / Mil / misc
    if (!strncmp(u, "A109", 4) || !strncmp(u, "A119", 4) || !strncmp(u, "A139", 4) || !strncmp(u, "A169", 4) ||
        !strncmp(u, "A189", 4) || !strncmp(u, "MD5", 3) || !strncmp(u, "MD6", 3) || !strncmp(u, "MD9", 3) ||
        !strncmp(u, "MI8", 3) || !strncmp(u, "MI17", 4) || !strncmp(u, "MI24", 4) || !strncmp(u, "KA32", 4) ||
        !strncmp(u, "H500", 4) || !strncmp(u, "H369", 4) || !strncmp(u, "NH90", 4) || !strncmp(u, "EN48", 4) ||
        !strncmp(u, "GAZL", 4) || !strncmp(u, "PUMA", 4) || !strncmp(u, "LYNX", 4) || !strncmp(u, "HELI", 4)) {
        return true;
    }
    return false;
}

// 0=light/GA, 1=medium airliner, 2=heavy
static int aircraft_size_class(const Aircraft *ac) {
    if (aircraft_is_helicopter(ac)) {
        return 0;
    }
    const char *c = ac->category;
    if (c[0] == 'A' || c[0] == 'a') {
        if (c[1] == '5' || c[1] == '6') {
            return 2;  // heavy / high vortex
        }
        if (c[1] == '1' || c[1] == '7') {
            return 0;  // light / rotorcraft
        }
        if (c[1] == '2' || c[1] == '3' || c[1] == '4') {
            return 1;
        }
    }
    const char *t = ac->type;
    if (!t[0]) {
        return 1;
    }
    if (!strncmp(t, "A38", 3) || !strncmp(t, "A34", 3) || !strncmp(t, "A33", 3) ||
        !strncmp(t, "A35", 3) || !strncmp(t, "A30", 3) || !strncmp(t, "B74", 3) ||
        !strncmp(t, "B77", 3) || !strncmp(t, "B78", 3) || !strncmp(t, "B76", 3) ||
        !strncmp(t, "MD1", 3) || !strncmp(t, "A12", 3)) {
        return 2;
    }
    if ((t[0] == 'C' && isdigit((unsigned char)t[1])) || !strncmp(t, "P28", 3) ||
        !strncmp(t, "PA2", 3) || !strncmp(t, "PA3", 3) || !strncmp(t, "PA4", 3) ||
        !strncmp(t, "BE3", 3) || !strncmp(t, "BE9", 3) || !strncmp(t, "SR2", 3) ||
        !strncmp(t, "DA4", 3) || !strncmp(t, "M20", 3) || !strncmp(t, "RV", 2) ||
        !strncmp(t, "GLID", 4) || !strncmp(t, "ULAC", 4) || !strncmp(t, "GYRO", 4)) {
        return 0;
    }
    return 1;
}

static void plane_xf(lv_coord_t x, lv_coord_t y, float rx, float ry, float fx, float fy, float lx, float ly,
                     lv_point_t *out) {
    out->x = (lv_coord_t)lroundf(x + rx * lx + fx * ly);
    out->y = (lv_coord_t)lroundf(y + ry * lx + fy * ly);
}

static void draw_airplane_icon(lv_draw_ctx_t *ctx, lv_coord_t x, lv_coord_t y, float track_deg, int size_class,
                               lv_color_t color) {
    // Icon scale by ADS-B size class: light / medium / heavy.
    const float s = (size_class >= 2) ? 15.0f : (size_class == 0) ? 7.0f : 11.0f;
    const float hdg = track_deg * FR_DEG2RAD;
    // Screen: +Y down. Nose along track: forward = (sin, -cos).
    const float fx = sinf(hdg);
    const float fy = -cosf(hdg);
    const float rx = cosf(hdg);
    const float ry = sinf(hdg);

    lv_point_t nose, tail, lwing, rwing, lstab, rstab, body;
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, 1.00f * s, &nose);
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, -0.85f * s, &tail);
    plane_xf(x, y, rx, ry, fx, fy, -0.95f * s, 0.05f * s, &lwing);
    plane_xf(x, y, rx, ry, fx, fy, 0.95f * s, 0.05f * s, &rwing);
    plane_xf(x, y, rx, ry, fx, fy, -0.40f * s, -0.55f * s, &lstab);
    plane_xf(x, y, rx, ry, fx, fy, 0.40f * s, -0.55f * s, &rstab);
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, 0.05f * s, &body);

    lv_draw_line_dsc_t ln;
    lv_draw_line_dsc_init(&ln);
    ln.color = color;
    ln.opa = LV_OPA_COVER;
    ln.round_start = 1;
    ln.round_end = 1;

    // Fuselage
    ln.width = (size_class >= 2) ? 3 : 2;
    lv_draw_line(ctx, &ln, &nose, &tail);
    // Main wing
    ln.width = (size_class >= 2) ? 3 : (size_class == 0) ? 2 : 3;
    lv_draw_line(ctx, &ln, &lwing, &rwing);
    // Horizontal stabilizer
    ln.width = 2;
    lv_draw_line(ctx, &ln, &lstab, &rstab);

    // Small filled body so the icon reads at a glance.
    lv_draw_rect_dsc_t fill;
    lv_draw_rect_dsc_init(&fill);
    fill.bg_color = color;
    fill.bg_opa = LV_OPA_COVER;
    fill.radius = LV_RADIUS_CIRCLE;
    const lv_coord_t r = (size_class >= 2) ? 3 : 2;
    lv_area_t dot = {(lv_coord_t)(body.x - r), (lv_coord_t)(body.y - r), (lv_coord_t)(body.x + r),
                     (lv_coord_t)(body.y + r)};
    lv_draw_rect(ctx, &fill, &dot);
}

/** Top-down helicopter: fuselage + main rotor cross + tail boom / rotor. */
static void draw_helicopter_icon(lv_draw_ctx_t *ctx, lv_coord_t x, lv_coord_t y, float track_deg,
                                 lv_color_t color) {
    const float s = 8.0f;  // slightly under medium jet (11); above light GA (7)
    const float hdg = track_deg * FR_DEG2RAD;
    const float fx = sinf(hdg);
    const float fy = -cosf(hdg);
    const float rx = cosf(hdg);
    const float ry = sinf(hdg);

    lv_point_t nose, tail, hub, blade_a0, blade_a1, blade_b0, blade_b1, trotor0, trotor1;
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, 0.55f * s, &nose);
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, -1.05f * s, &tail);
    plane_xf(x, y, rx, ry, fx, fy, 0.00f * s, 0.05f * s, &hub);
    // Main rotor as an X (two blades) centered on hub.
    plane_xf(x, y, rx, ry, fx, fy, -0.95f * s, 0.55f * s, &blade_a0);
    plane_xf(x, y, rx, ry, fx, fy, 0.95f * s, -0.45f * s, &blade_a1);
    plane_xf(x, y, rx, ry, fx, fy, -0.95f * s, -0.45f * s, &blade_b0);
    plane_xf(x, y, rx, ry, fx, fy, 0.95f * s, 0.55f * s, &blade_b1);
    // Tail rotor (short lateral stroke at boom tip).
    plane_xf(x, y, rx, ry, fx, fy, -0.35f * s, -1.05f * s, &trotor0);
    plane_xf(x, y, rx, ry, fx, fy, 0.35f * s, -1.05f * s, &trotor1);

    lv_draw_line_dsc_t ln;
    lv_draw_line_dsc_init(&ln);
    ln.color = color;
    ln.opa = LV_OPA_COVER;
    ln.round_start = 1;
    ln.round_end = 1;

    ln.width = 2;
    lv_draw_line(ctx, &ln, &nose, &tail);       // fuselage / boom
    ln.width = 2;
    lv_draw_line(ctx, &ln, &blade_a0, &blade_a1);  // main rotor
    lv_draw_line(ctx, &ln, &blade_b0, &blade_b1);
    ln.width = 2;
    lv_draw_line(ctx, &ln, &trotor0, &trotor1);  // tail rotor

    lv_draw_rect_dsc_t fill;
    lv_draw_rect_dsc_init(&fill);
    fill.bg_color = color;
    fill.bg_opa = LV_OPA_COVER;
    fill.radius = LV_RADIUS_CIRCLE;
    const lv_coord_t r = 2;
    lv_area_t dot = {(lv_coord_t)(hub.x - r), (lv_coord_t)(hub.y - r), (lv_coord_t)(hub.x + r),
                     (lv_coord_t)(hub.y + r)};
    lv_draw_rect(ctx, &fill, &dot);
}

// Geographic sketch is rasterized into PSRAM off the RGB framebuffers so heavy
// coastline drawing does not contend with panel DMA (flicker/hop on redraw).
static lv_color_t *s_map_cache = nullptr;
static lv_coord_t s_map_cw = 0;
static lv_coord_t s_map_ch = 0;
static float s_map_clat = 1e9f;
static float s_map_clon = 1e9f;
static float s_map_crange = -1.0f;
static uint64_t s_map_cmask = 0;
static bool s_map_cshow = false;
static int s_map_cfollow_state = -2;  // -2 = unused; -1 = unknown; 0..49 = state
static bool s_map_ready = false;
static lv_img_dsc_t s_map_img{};

// Per-state bbox of county points (lat_q/lon_q = deg*50), built once.
static int16_t s_st_lat_min[50], s_st_lat_max[50], s_st_lon_min[50], s_st_lon_max[50];
static bool s_st_bbox_ok = false;

static void ensure_state_bboxes() {
    if (s_st_bbox_ok) {
        return;
    }
    for (int si = 0; si < 50; si++) {
        s_st_lat_min[si] = 32767;
        s_st_lat_max[si] = -32768;
        s_st_lon_min[si] = 32767;
        s_st_lon_max[si] = -32768;
        const CountyStateRange &sr = kCountyStateRanges[si];
        for (uint16_t p = 0; p < sr.poly_count; p++) {
            const CountyPoly &poly = kCountyPolys[sr.poly_start + p];
            for (uint16_t k = 0; k < poly.count; k++) {
                const CountyPt &pt = kCountyPts[poly.start + k];
                if (pt.lat_q < s_st_lat_min[si]) s_st_lat_min[si] = pt.lat_q;
                if (pt.lat_q > s_st_lat_max[si]) s_st_lat_max[si] = pt.lat_q;
                if (pt.lon_q < s_st_lon_min[si]) s_st_lon_min[si] = pt.lon_q;
                if (pt.lon_q > s_st_lon_max[si]) s_st_lon_max[si] = pt.lon_q;
            }
        }
    }
    s_st_bbox_ok = true;
}

static bool county_point_in_poly(int qlat, int qlon, uint32_t start, uint16_t count) {
    if (count < 3) {
        return false;
    }
    bool inside = false;
    for (uint16_t i = 0, j = (uint16_t)(count - 1); i < count; j = i++) {
        const int yi = kCountyPts[start + i].lat_q;
        const int xi = kCountyPts[start + i].lon_q;
        const int yj = kCountyPts[start + j].lat_q;
        const int xj = kCountyPts[start + j].lon_q;
        if ((yi > qlat) == (yj > qlat)) {
            continue;
        }
        const int dy = yj - yi;
        if (dy == 0) {
            continue;
        }
        const int64_t cross = (int64_t)(xj - xi) * (qlat - yi);
        const int xint = (int)(xi + cross / dy);
        if (qlon < xint) {
            inside = !inside;
        }
    }
    return inside;
}

/** US state index 0..49 for lat/lon, or -1 if not over a state. */
static int state_index_from_ll(float lat, float lon) {
    ensure_state_bboxes();
    const int qlat = (int)lroundf(lat * 50.0f);
    const int qlon = (int)lroundf(lon * 50.0f);
    for (int si = 0; si < 50; si++) {
        if (qlat < s_st_lat_min[si] - 2 || qlat > s_st_lat_max[si] + 2) {
            continue;
        }
        if (qlon < s_st_lon_min[si] - 2 || qlon > s_st_lon_max[si] + 2) {
            continue;
        }
        const CountyStateRange &sr = kCountyStateRanges[si];
        for (uint16_t p = 0; p < sr.poly_count; p++) {
            const CountyPoly &poly = kCountyPolys[sr.poly_start + p];
            if (county_point_in_poly(qlat, qlon, poly.start, poly.count)) {
                return si;
            }
        }
    }
    // Coastal / gap fallback: nearest state centroid within ~80 nm.
    float best = 80.0f * 80.0f;
    int best_si = -1;
    for (int si = 0; si < 50; si++) {
        const float clat = (s_st_lat_min[si] + s_st_lat_max[si]) * 0.5f / 50.0f;
        const float clon = (s_st_lon_min[si] + s_st_lon_max[si]) * 0.5f / 50.0f;
        float east = 0, north = 0;
        ll_to_nm(clat, clon, lat, lon, &east, &north);
        const float d2 = east * east + north * north;
        if (d2 < best) {
            best = d2;
            best_si = si;
        }
    }
    return best_si;
}

static inline void cache_put(lv_color_t *buf, int w, int h, int x, int y, lv_color_t c) {
    if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) {
        return;
    }
    buf[y * w + x] = c;
}

static void cache_line(lv_color_t *buf, int w, int h, int x0, int y0, int x1, int y1, int cx, int cy, int64_t R2,
                       lv_color_t c, int thickness = 1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    const int thick = thickness < 1 ? 1 : thickness;
    for (;;) {
        const int64_t ddx = (int64_t)x0 - cx;
        const int64_t ddy = (int64_t)y0 - cy;
        if (ddx * ddx + ddy * ddy <= R2) {
            cache_put(buf, w, h, x0, y0, c);
            if (thick >= 2) {
                // ~2px stroke: paint orthogonal neighbors.
                cache_put(buf, w, h, x0 + 1, y0, c);
                cache_put(buf, w, h, x0, y0 + 1, c);
                cache_put(buf, w, h, x0 - 1, y0, c);
                cache_put(buf, w, h, x0, y0 - 1, c);
            }
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void map_cache_mark_dirty() {
    s_map_ready = false;
}

template <typename PtT, typename PolyT>
static void cache_draw_polys(lv_color_t *buf, int w, int h, float olat, float olon, float range, int cx, int cy,
                             int64_t R2, int pad, int lat0, int lon0, const PtT *pts, const PolyT *polys,
                             uint16_t poly_count, lv_color_t ink, int thickness = 1) {
    // Soft margin so long borders (state outlines) still draw when zoomed in —
    // 3-point sampling used to skip polys whose start/mid/end fell outside the pad.
    const int cull = pad + 120;
    for (uint16_t pi = 0; pi < poly_count; pi++) {
        const PolyT &poly = polys[pi];
        if (poly.count < 2) {
            continue;
        }
        int min_lat = pts[poly.start].lat_q;
        int max_lat = min_lat;
        int min_lon = pts[poly.start].lon_q;
        int max_lon = min_lon;
        for (uint16_t k = 1; k < poly.count; k++) {
            const PtT &pt = pts[poly.start + k];
            if (pt.lat_q < min_lat) {
                min_lat = pt.lat_q;
            }
            if (pt.lat_q > max_lat) {
                max_lat = pt.lat_q;
            }
            if (pt.lon_q < min_lon) {
                min_lon = pt.lon_q;
            }
            if (pt.lon_q > max_lon) {
                max_lon = pt.lon_q;
            }
        }
        if (max_lat < lat0 - cull || min_lat > lat0 + cull || max_lon < lon0 - cull || min_lon > lon0 + cull) {
            continue;
        }

        bool have_prev = false;
        lv_coord_t px = 0, py = 0;
        bool prev_in = false;
        for (uint16_t k = 0; k < poly.count; k++) {
            const PtT &pt = pts[poly.start + k];
            if (abs((int)pt.lat_q - lat0) > cull || abs((int)pt.lon_q - lon0) > cull) {
                have_prev = false;
                continue;
            }
            lv_coord_t x, y;
            map_point_ll(pt.lat_q / 50.0f, pt.lon_q / 50.0f, olat, olon, range, w, h, &x, &y);
            const int64_t dx = (int64_t)x - cx;
            const int64_t dy = (int64_t)y - cy;
            const bool inside = (dx * dx + dy * dy) <= R2;
            if (have_prev && (inside || prev_in)) {
                cache_line(buf, w, h, px, py, x, y, cx, cy, R2, ink, thickness);
            }
            px = x;
            py = y;
            prev_in = inside;
            have_prev = true;
        }
    }
}

static bool map_cache_ensure(lv_coord_t w, lv_coord_t h, float olat, float olon, float range) {
    if (w < 8 || h < 8) {
        return false;
    }
    // County toggle works even while tracking (enter-follow starts with counties off).
    const bool draw_counties = s_show_counties;
    // While tracking, only the state under the aircraft (maintained by follow_refresh_county_state).
    int follow_st = -2;
    if (s_follow && draw_counties) {
        if (s_follow_county_state >= -1) {
            follow_st = s_follow_county_state;
        } else {
            follow_refresh_county_state(false);
            follow_st = s_follow_county_state;
        }
    }
    if (s_map_ready && s_map_cache && s_map_cw == w && s_map_ch == h && fabsf(s_map_clat - olat) < 1e-5f &&
        fabsf(s_map_clon - olon) < 1e-5f && fabsf(s_map_crange - range) < 0.05f && s_map_cmask == s_county_mask &&
        s_map_cshow == draw_counties && s_map_cfollow_state == follow_st) {
        return true;
    }

    const size_t bytes = (size_t)w * (size_t)h * sizeof(lv_color_t);
    if (!s_map_cache || s_map_cw != w || s_map_ch != h) {
        if (s_map_cache) {
            heap_caps_free(s_map_cache);
            s_map_cache = nullptr;
        }
        s_map_cache = (lv_color_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_map_cache) {
            s_map_ready = false;
            return false;
        }
        s_map_cw = w;
        s_map_ch = h;
    }

    const lv_color_t bg = COL_BG;
    const lv_color_t disk = lv_color_hex(0x031208);
    const int n = (int)w * (int)h;
    for (int i = 0; i < n; i++) {
        s_map_cache[i] = bg;
    }

    const int cx = w / 2;
    const int cy = h / 2;
    const int R = (LV_MIN(w, h) / 2) - 6;
    const int64_t R2 = (int64_t)R * (int64_t)R;
    for (int y = cy - R; y <= cy + R; y++) {
        for (int x = cx - R; x <= cx + R; x++) {
            const int64_t dx = (int64_t)x - cx;
            const int64_t dy = (int64_t)y - cy;
            if (dx * dx + dy * dy <= R2) {
                cache_put(s_map_cache, w, h, x, y, disk);
            }
        }
    }

    const int pad = (int)lroundf(range / 1.1f) + 3;
    const int lat0 = (int)lroundf(olat * 50.0f);
    const int lon0 = (int)lroundf(olon * 50.0f);
    // Coast / lakes / country borders — always thin.
    const uint16_t non_state = (kMapStatePolyStart < kMapPolyCount) ? kMapStatePolyStart : kMapPolyCount;
    if (non_state > 0) {
        cache_draw_polys(s_map_cache, w, h, olat, olon, range, cx, cy, R2, pad, lat0, lon0, kMapPts, kMapPolys,
                         non_state, COL_MAP, 1);
    }
    // US state outlines — thick only while county layer is on for selected states.
    if (kMapStatePolyCount > 0 && kMapStatePolyStart < kMapPolyCount) {
        const bool any_cty = draw_counties && (s_follow ? (follow_st >= 0) : (s_county_mask != 0));
        const int state_thick = any_cty ? 2 : 1;
        cache_draw_polys(s_map_cache, w, h, olat, olon, range, cx, cy, R2, pad, lat0, lon0, kMapPts,
                         &kMapPolys[kMapStatePolyStart], kMapStatePolyCount, COL_MAP, state_thick);
    }
    if (draw_counties) {
        if (s_follow) {
            // Tracking: only counties for the aircraft's current state.
            if (follow_st >= 0 && follow_st < 50) {
                const CountyStateRange &sr = kCountyStateRanges[follow_st];
                if (sr.poly_count > 0) {
                    cache_draw_polys(s_map_cache, w, h, olat, olon, range, cx, cy, R2, pad, lat0, lon0,
                                     kCountyPts, &kCountyPolys[sr.poly_start], sr.poly_count, COL_MAP, 1);
                }
            }
        } else if (s_county_mask) {
            for (int si = 0; si < 50; si++) {
                if ((s_county_mask & (1ULL << si)) == 0) {
                    continue;
                }
                const CountyStateRange &sr = kCountyStateRanges[si];
                if (sr.poly_count == 0) {
                    continue;
                }
                cache_draw_polys(s_map_cache, w, h, olat, olon, range, cx, cy, R2, pad, lat0, lon0, kCountyPts,
                                 &kCountyPolys[sr.poly_start], sr.poly_count, COL_MAP, 1);
            }
        }
    }

    memset(&s_map_img, 0, sizeof(s_map_img));
    s_map_img.header.always_zero = 0;
    s_map_img.header.w = w;
    s_map_img.header.h = h;
    s_map_img.header.cf = LV_IMG_CF_TRUE_COLOR;
    s_map_img.data_size = bytes;
    s_map_img.data = (const uint8_t *)s_map_cache;
    s_map_clat = olat;
    s_map_clon = olon;
    s_map_crange = range;
    s_map_cmask = s_county_mask;
    s_map_cshow = draw_counties;
    s_map_cfollow_state = follow_st;
    s_map_ready = true;
    return true;
}

static uint32_t s_snap_ms = 0;

static void snapshot_radar() {
    // Brief wait — a zero timeout was skipping the first boot traffic paint.
    if (!g_state || xSemaphoreTake(g_state->mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    s_ac_n = g_state->aircraft_count;
    s_apt_n = g_state->airport_count;
    if (s_ac_n > 0) {
        memcpy(s_ac, g_state->aircraft, sizeof(Aircraft) * s_ac_n);
    }
    if (s_apt_n > 0) {
        memcpy(s_apt, g_state->airports, sizeof(Airport) * s_apt_n);
    }
    s_olat = g_state->cfg.location.lat;
    s_olon = g_state->cfg.location.lon;
    s_range = g_state->cfg.range_nm;
    s_max_alt = g_state->cfg.max_alt_ft;
    s_show_airports = g_state->cfg.show_airports;
    s_show_counties = g_state->cfg.show_counties;
    s_show_cities = g_state->cfg.show_cities;
    s_county_mask = g_state->cfg.county_mask;
    xSemaphoreGive(g_state->mutex);
    s_snap_ms = millis();
}

/** Sweep redraws ~18×/s; reuse the last traffic snapshot so draw skips mutex + memcpy. */
static void snapshot_radar_for_draw() {
    const uint32_t now = millis();
    if (s_snap_ms != 0 && (now - s_snap_ms) < 200u) {
        return;
    }
    snapshot_radar();
}

/** Class B built-ins + user-added plate ICAOs (persisted in NVS). */
static bool plate_known(const char *icao) {
    if (!icao || !icao[0]) {
        return false;
    }
    if (airport_plate_index(icao) >= 0) {
        return true;
    }
    for (int i = 0; i < s_plate_extra_n; i++) {
        if (strcasecmp(s_plate_extra[i], icao) == 0) {
            return true;
        }
    }
    return false;
}

/** Cached ≤15 nm map plates — avoids scanning the full runway DB on every PPI redraw. */
#ifndef FR_MAP_PLATE_MAX
#define FR_MAP_PLATE_MAX 12
#endif
struct MapPlateEntry {
    char icao[8];
    float lat;
    float lon;
    Runway rwys[FR_MAX_RWYS_PER_APT];
    int rwy_n;
};
static MapPlateEntry s_map_plates[FR_MAP_PLATE_MAX];
static int s_map_plate_n = 0;
static bool s_map_plates_ready = false;
static float s_map_plates_olat = 1e9f;
static float s_map_plates_olon = 1e9f;
static float s_map_plates_range = -1.0f;
static int s_map_plates_apt_n = -1;
static int s_map_plates_extra_n = -1;
static char s_map_plates_dep[8] = {0};
static char s_map_plates_arr[8] = {0};

static void map_plates_mark_dirty() {
    s_map_plates_ready = false;
}

static bool map_plates_push(const char *icao, float lat, float lon, const Runway *rwys, int rwy_n) {
    if (!icao || !icao[0] || rwy_n <= 0 || !rwys || s_map_plate_n >= FR_MAP_PLATE_MAX) {
        return false;
    }
    for (int i = 0; i < s_map_plate_n; i++) {
        if (strcasecmp(s_map_plates[i].icao, icao) == 0) {
            return false;
        }
    }
    MapPlateEntry &e = s_map_plates[s_map_plate_n];
    memset(&e, 0, sizeof(e));
    strncpy(e.icao, icao, sizeof(e.icao) - 1);
    e.lat = lat;
    e.lon = lon;
    e.rwy_n = rwy_n > FR_MAX_RWYS_PER_APT ? FR_MAX_RWYS_PER_APT : rwy_n;
    memcpy(e.rwys, rwys, sizeof(Runway) * (size_t)e.rwy_n);
    s_map_plate_n++;
    return true;
}

static void map_plates_ensure() {
    if (s_range > 15.5f) {
        s_map_plate_n = 0;
        s_map_plates_ready = true;
        return;
    }
    const char *dep = (s_follow && s_follow_dep.ok) ? s_follow_dep.icao : "";
    const char *arr = (s_follow && s_follow_arr.ok) ? s_follow_arr.icao : "";
    if (s_map_plates_ready && fabsf(s_map_plates_olat - s_olat) < 1e-5f &&
        fabsf(s_map_plates_olon - s_olon) < 1e-5f && fabsf(s_map_plates_range - s_range) < 0.05f &&
        s_map_plates_apt_n == s_apt_n && s_map_plates_extra_n == s_plate_extra_n &&
        strcasecmp(s_map_plates_dep, dep) == 0 && strcasecmp(s_map_plates_arr, arr) == 0) {
        return;
    }

    s_map_plate_n = 0;
    if (s_follow) {
        if (s_follow_dep.ok && s_follow_dep_rwy_n > 0) {
            map_plates_push(s_follow_dep.icao, s_follow_dep.lat, s_follow_dep.lon, s_follow_dep_rwys,
                            s_follow_dep_rwy_n);
        }
        if (s_follow_arr.ok && s_follow_arr_rwy_n > 0) {
            map_plates_push(s_follow_arr.icao, s_follow_arr.lat, s_follow_arr.lon, s_follow_arr_rwys,
                            s_follow_arr_rwy_n);
        }
    }
    Runway cat_rwys[FR_MAX_RWYS_PER_APT];
    for (int i = 0; i < s_apt_n && s_map_plate_n < FR_MAP_PLATE_MAX; i++) {
        const Airport *a = &s_apt[i];
        if (!a->valid || !a->icao[0] || !plate_known(a->icao)) {
            continue;
        }
        const int rn = net_lookup_runways(a->icao, cat_rwys, FR_MAX_RWYS_PER_APT);
        if (rn <= 0) {
            continue;
        }
        map_plates_push(a->icao, a->lat, a->lon, cat_rwys, rn);
    }

    s_map_plates_olat = s_olat;
    s_map_plates_olon = s_olon;
    s_map_plates_range = s_range;
    s_map_plates_apt_n = s_apt_n;
    s_map_plates_extra_n = s_plate_extra_n;
    strncpy(s_map_plates_dep, dep, sizeof(s_map_plates_dep) - 1);
    s_map_plates_dep[sizeof(s_map_plates_dep) - 1] = 0;
    strncpy(s_map_plates_arr, arr, sizeof(s_map_plates_arr) - 1);
    s_map_plates_arr[sizeof(s_map_plates_arr) - 1] = 0;
    s_map_plates_ready = true;
}

static int plates_total_count() {
    return AIRPORT_PLATE_COUNT + s_plate_extra_n;
}

static const char *plates_entry_icao(int idx) {
    if (idx >= 0 && idx < AIRPORT_PLATE_COUNT) {
        return kAirportPlates[idx].icao;
    }
    const int e = idx - AIRPORT_PLATE_COUNT;
    if (e >= 0 && e < s_plate_extra_n) {
        return s_plate_extra[e];
    }
    return nullptr;
}

/** Heading family [0,180) so reciprocal ends count as the same alignment. */
static int rwy_hdg_family(int hdg_deg) {
    int h = hdg_deg % 180;
    if (h < 0) {
        h += 180;
    }
    return h;
}

static bool rwy_is_parallel(int hdg_a, int hdg_b) {
    int d = abs(rwy_hdg_family(hdg_a) - rwy_hdg_family(hdg_b));
    if (d > 90) {
        d = 180 - d;
    }
    return d <= 12;
}

/** Collect CTAF/ATC|CTR/GND/ATIS|AWOS lines that have frequencies. */
static int plate_freq_lines(const AirportRadios *r, const char *lbls[4], const char *freqs[4]) {
    int n = 0;
    if (!r) {
        return 0;
    }
    auto push = [&](const char *lbl, const char *freq) {
        if (freq && freq[0] && n < 4) {
            lbls[n] = lbl;
            freqs[n] = freq;
            n++;
        }
    };
    push("CTAF", r->ctaf);
    push((r->atc_is_center && r->atc[0]) ? "CTR" : "ATC", r->atc);
    push("GND", r->gnd);
    push((r->atis_is_awos && r->atis[0]) ? "AWOS" : "ATIS", r->atis);
    return n;
}

/** Airport diagram plate: beige card, black/grass strips, runway numbers at ends.
 *  Drawn from runway_db / airport_plate_db (procedural LVGL geometry).
 *  Lengths and frequencies live in the plates-page table below the preview. */
/** Draw embedded FAA/custom plate bitmap (INDEXED_4BIT) scaled to fit coords. */
static bool draw_faa_plate_bitmap(lv_draw_ctx_t *ctx, const lv_area_t *coords, const char *icao) {
    if (!ctx || !coords || !icao || !icao[0]) {
        return false;
    }
    const lv_img_dsc_t *img = faa_plate_lookup(icao);
    if (!img || !img->data || img->header.w == 0 || img->header.h == 0) {
        return false;
    }
    const lv_coord_t aw = lv_area_get_width(coords);
    const lv_coord_t ah = lv_area_get_height(coords);
    if (aw < 8 || ah < 8) {
        return false;
    }
    const lv_coord_t side = LV_MIN(aw, ah);
    uint16_t zoom = 256;
    if ((lv_coord_t)img->header.w != side) {
        zoom = (uint16_t)((256u * (uint32_t)side) / (uint32_t)img->header.w);
        if (zoom < 1) {
            zoom = 1;
        }
    }
    // LVGL applies zoom around pivot; coords are the un-zoomed image box.
    const lv_coord_t cx = (lv_coord_t)((coords->x1 + coords->x2) / 2);
    const lv_coord_t cy = (lv_coord_t)((coords->y1 + coords->y2) / 2);
    const lv_coord_t x1 = (lv_coord_t)(cx - (lv_coord_t)img->header.w / 2);
    const lv_coord_t y1 = (lv_coord_t)(cy - (lv_coord_t)img->header.h / 2);
    lv_area_t dest = {x1, y1, (lv_coord_t)(x1 + (lv_coord_t)img->header.w - 1),
                      (lv_coord_t)(y1 + (lv_coord_t)img->header.h - 1)};
    lv_draw_img_dsc_t id;
    lv_draw_img_dsc_init(&id);
    id.zoom = zoom;
    id.pivot.x = (lv_coord_t)(img->header.w / 2);
    id.pivot.y = (lv_coord_t)(img->header.h / 2);
    lv_draw_img(ctx, &id, &dest, img);
    return true;
}

static void draw_runway_plate(lv_draw_ctx_t *ctx, lv_coord_t cx, lv_coord_t cy, lv_coord_t size,
                              const char *icao, const Runway *rwys, int rwy_n) {
    if (!ctx || size < 24) {
        return;
    }
    if (!rwys || rwy_n <= 0) {
        return;
    }
    if (rwy_n > FR_MAX_RWYS_PER_APT) {
        rwy_n = FR_MAX_RWYS_PER_APT;
    }
    const lv_coord_t half = size / 2;
    lv_area_t box = {(lv_coord_t)(cx - half), (lv_coord_t)(cy - half), (lv_coord_t)(cx + half),
                     (lv_coord_t)(cy + half)};
    lv_draw_rect_dsc_t bg;
    lv_draw_rect_dsc_init(&bg);
    bg.bg_color = lv_color_hex(0xF5F5F0);
    bg.bg_opa = LV_OPA_COVER;
    bg.border_color = lv_color_hex(0x222222);
    bg.border_width = 1;
    bg.border_opa = LV_OPA_COVER;
    bg.radius = 3;
    lv_draw_rect(ctx, &bg, &box);

    const lv_coord_t title_h = (size >= 80) ? 16 : 12;

    lv_draw_label_dsc_t lab;
    lv_draw_label_dsc_init(&lab);
    lab.font = (size >= 120) ? &lv_font_montserrat_14 : (size >= 80 ? &lv_font_montserrat_12 : &lv_font_montserrat_10);
    lab.color = lv_color_hex(0x111111);
    if (icao && icao[0]) {
        lv_area_t ta = {(lv_coord_t)(box.x1 + 3), (lv_coord_t)(box.y1 + 2), (lv_coord_t)(box.x2 - 3),
                        (lv_coord_t)(box.y1 + title_h)};
        lv_draw_label(ctx, &lab, &ta, icao, nullptr);
    }

    const float diag_left = (float)box.x1 + 2.0f;
    const float diag_right = (float)box.x2 - 2.0f;
    const float diag_top = (float)box.y1 + (float)title_h + 2.0f;
    const float diag_bot = (float)box.y2 - 2.0f;
    const float diag_w = diag_right - diag_left;
    const float diag_h = diag_bot - diag_top;
    const float pad = (size >= 120) ? 28.0f : (size >= 80 ? 22.0f : 16.0f);
    const float num_gap = (size >= 120) ? 14.0f : (size >= 80 ? 11.0f : 9.0f);

    // Fit all runway endpoints (geo mid ± half length along heading) into the diagram.
    float min_e = 1e9f, max_e = -1e9f, min_n = 1e9f, max_n = -1e9f;
    bool have_geo = false;
    for (int i = 0; i < rwy_n; i++) {
        if (rwys[i].east_ft != 0 || rwys[i].north_ft != 0) {
            have_geo = true;
        }
    }
    for (int i = 0; i < rwy_n; i++) {
        const Runway *r = &rwys[i];
        const float rad = r->hdg_deg * FR_DEG2RAD;
        const float fx = sinf(rad);   // east component of LE→HE
        const float fy = cosf(rad);   // north component of LE→HE
        const float half_l = 0.5f * (float)r->len_ft;
        const float me = (float)r->east_ft;
        const float mn = (float)r->north_ft;
        const float e0 = me - fx * half_l;
        const float e1 = me + fx * half_l;
        const float n0 = mn - fy * half_l;
        const float n1 = mn + fy * half_l;
        min_e = fminf(min_e, fminf(e0, e1));
        max_e = fmaxf(max_e, fmaxf(e0, e1));
        min_n = fminf(min_n, fminf(n0, n1));
        max_n = fmaxf(max_n, fmaxf(n0, n1));
    }
    if (!have_geo) {
        // Single-strip / no coords: center at origin with length span only.
        min_e = max_e = 0.0f;
        min_n = max_n = 0.0f;
        for (int i = 0; i < rwy_n; i++) {
            const float half_l = 0.5f * (float)rwys[i].len_ft;
            min_e = fminf(min_e, -half_l);
            max_e = fmaxf(max_e, half_l);
            min_n = fminf(min_n, -half_l);
            max_n = fmaxf(max_n, half_l);
        }
    }
    float span_e = max_e - min_e;
    float span_n = max_n - min_n;
    if (span_e < 500.0f) {
        span_e = 500.0f;
    }
    if (span_n < 500.0f) {
        span_n = 500.0f;
    }
    const float usable_w = (diag_w > pad + 8.0f) ? (diag_w - pad) : ((float)size * 0.45f);
    const float usable_h = (diag_h > pad + 8.0f) ? (diag_h - pad) : ((float)size * 0.45f);
    float scale = usable_w / span_e;
    if (usable_h / span_n < scale) {
        scale = usable_h / span_n;
    }
    const float cen_e = 0.5f * (min_e + max_e);
    const float cen_n = 0.5f * (min_n + max_n);
    const float diag_cx = 0.5f * (diag_left + diag_right);
    const float mid_y = 0.5f * (diag_top + diag_bot);

    lab.font = (size >= 100) ? &lv_font_montserrat_12 : &lv_font_montserrat_10;

    for (int i = 0; i < rwy_n; i++) {
        const Runway *r = &rwys[i];
        const float rad = r->hdg_deg * FR_DEG2RAD;
        const float fx = sinf(rad);
        const float fy = -cosf(rad);  // screen Y grows downward
        const float rx = cosf(rad);
        const float ry = sinf(rad);
        // Geographic midpoint → screen (north = up).
        const float rcx = diag_cx + ((float)r->east_ft - cen_e) * scale;
        const float rcy = mid_y - ((float)r->north_ft - cen_n) * scale;
        const float half_l = 0.5f * r->len_ft * scale;
        float half_w = 0.5f * (float)LV_MAX(r->width_ft, 60) * scale;
        if (half_w < 1.5f) {
            half_w = 1.5f;
        }
        if (half_w > (size >= 100 ? 12.0f : 6.0f)) {
            half_w = (size >= 100) ? 12.0f : 6.0f;
        }
        // Grass strips: slightly wider so they read clearly next to pavement.
        if (r->is_turf && half_w < (size >= 100 ? 4.0f : 2.5f)) {
            half_w = (size >= 100) ? 4.0f : 2.5f;
        }

        lv_point_t pts[4];
        const float ox[4] = {-half_l, half_l, half_l, -half_l};
        const float oy[4] = {-half_w, -half_w, half_w, half_w};
        for (int k = 0; k < 4; k++) {
            pts[k].x = (lv_coord_t)lroundf(rcx + fx * ox[k] + rx * oy[k]);
            pts[k].y = (lv_coord_t)lroundf(rcy + fy * ox[k] + ry * oy[k]);
        }
        lv_draw_rect_dsc_t rwy_fill;
        lv_draw_rect_dsc_init(&rwy_fill);
        rwy_fill.bg_opa = LV_OPA_COVER;
        if (r->is_turf) {
            rwy_fill.bg_color = lv_color_hex(0x5B8F4A);  // grass
        } else {
            rwy_fill.bg_color = lv_color_hex(0x1A1A1A);
        }
        lv_draw_polygon(ctx, &rwy_fill, pts, 4);

        lv_draw_line_dsc_t cl;
        lv_draw_line_dsc_init(&cl);
        cl.width = 1;
        cl.opa = LV_OPA_COVER;
        if (r->is_turf) {
            // Dashed centerline on grass (draw short segments).
            cl.color = lv_color_hex(0xE8F5E0);
            const int segs = (size >= 100) ? 7 : 5;
            for (int s = 0; s < segs; s++) {
                if (s & 1) {
                    continue;
                }
                const float t0 = -0.85f + 1.7f * ((float)s / (float)segs);
                const float t1 = -0.85f + 1.7f * ((float)(s + 1) / (float)segs);
                lv_point_t p0 = {(lv_coord_t)lroundf(rcx + fx * half_l * t0),
                                 (lv_coord_t)lroundf(rcy + fy * half_l * t0)};
                lv_point_t p1 = {(lv_coord_t)lroundf(rcx + fx * half_l * t1),
                                 (lv_coord_t)lroundf(rcy + fy * half_l * t1)};
                lv_draw_line(ctx, &cl, &p0, &p1);
            }
        } else {
            cl.color = lv_color_hex(0xFFFFFF);
            lv_point_t p0 = {(lv_coord_t)lroundf(rcx - fx * half_l * 0.85f),
                             (lv_coord_t)lroundf(rcy - fy * half_l * 0.85f)};
            lv_point_t p1 = {(lv_coord_t)lroundf(rcx + fx * half_l * 0.85f),
                             (lv_coord_t)lroundf(rcy + fy * half_l * 0.85f)};
            lv_draw_line(ctx, &cl, &p0, &p1);
        }

        // Runway numbers at each end.
        lv_point_t le = {(lv_coord_t)lroundf(rcx - fx * (half_l + num_gap)),
                         (lv_coord_t)lroundf(rcy - fy * (half_l + num_gap))};
        lv_point_t he = {(lv_coord_t)lroundf(rcx + fx * (half_l + num_gap)),
                         (lv_coord_t)lroundf(rcy + fy * (half_l + num_gap))};
        const lv_coord_t tw = (size >= 100) ? 14 : 11;
        const lv_coord_t th = (size >= 100) ? 9 : 7;
        lv_area_t lea = {(lv_coord_t)(le.x - tw), (lv_coord_t)(le.y - th), (lv_coord_t)(le.x + tw),
                         (lv_coord_t)(le.y + th)};
        lv_area_t hea = {(lv_coord_t)(he.x - tw), (lv_coord_t)(he.y - th), (lv_coord_t)(he.x + tw),
                         (lv_coord_t)(he.y + th)};
        if (r->le[0]) {
            lv_draw_label(ctx, &lab, &lea, r->le, nullptr);
        }
        if (r->he[0]) {
            lv_draw_label(ctx, &lab, &hea, r->he, nullptr);
        }
    }
}

/** Mini airport diagram centered on (cx,cy) — used on the PPI at ≤15 nm. */
static void draw_map_airport_plate(lv_draw_ctx_t *ctx, lv_coord_t cx, lv_coord_t cy, lv_coord_t size,
                                   const char *icao, const Runway *rwys, int rwy_n) {
    draw_runway_plate(ctx, cx, cy, size, icao, rwys, rwy_n);
}

static void plate_draw(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const lv_coord_t w = lv_area_get_width(&coords);
    const lv_coord_t h = lv_area_get_height(&coords);
    const lv_coord_t cx = (coords.x1 + coords.x2) / 2;
    const lv_coord_t cy = (coords.y1 + coords.y2) / 2 + 8;

    lv_draw_rect_dsc_t bg;
    lv_draw_rect_dsc_init(&bg);
    bg.bg_color = lv_color_hex(0xF5F5F0);
    bg.bg_opa = LV_OPA_COVER;
    bg.border_color = lv_color_hex(0x222222);
    bg.border_width = 2;
    bg.border_opa = LV_OPA_COVER;
    lv_draw_rect(ctx, &bg, &coords);

    // Prefer custom/FAA bitmap (KATL, KBNA, …) over synthetic runway sketch.
    if (draw_faa_plate_bitmap(ctx, &coords, s_plate_icao)) {
        return;
    }

    if (s_plate_rwy_n > 0) {
        const lv_coord_t side = LV_MIN(w, h) - 8;
        draw_runway_plate(ctx, cx, cy, side, s_plate_icao, s_plate_rwys, s_plate_rwy_n);
        return;
    }

    lv_draw_label_dsc_t lab;
    lv_draw_label_dsc_init(&lab);
    lab.font = &lv_font_montserrat_12;
    lab.color = lv_color_hex(0x111111);
    {
        char hdr[40];
        snprintf(hdr, sizeof(hdr), "%s  AIRPORT DIAGRAM", s_plate_icao[0] ? s_plate_icao : "----");
        lv_area_t ta = {(lv_coord_t)(coords.x1 + 8), (lv_coord_t)(coords.y1 + 6),
                        (lv_coord_t)(coords.x2 - 8), (lv_coord_t)(coords.y1 + 24)};
        lv_draw_label(ctx, &lab, &ta, hdr, nullptr);
    }

    // North arrow (top-right).
    {
        lv_draw_line_dsc_t ln;
        lv_draw_line_dsc_init(&ln);
        ln.color = lv_color_hex(0x111111);
        ln.width = 2;
        ln.opa = LV_OPA_COVER;
        const lv_coord_t nx = coords.x2 - 28;
        const lv_coord_t ny = coords.y1 + 40;
        lv_point_t a = {nx, (lv_coord_t)(ny - 14)};
        lv_point_t b = {nx, (lv_coord_t)(ny + 10)};
        lv_draw_line(ctx, &ln, &a, &b);
        lv_point_t l = {(lv_coord_t)(nx - 5), (lv_coord_t)(ny - 6)};
        lv_point_t r = {(lv_coord_t)(nx + 5), (lv_coord_t)(ny - 6)};
        lv_draw_line(ctx, &ln, &a, &l);
        lv_draw_line(ctx, &ln, &a, &r);
        lv_area_t ta = {(lv_coord_t)(nx - 6), (lv_coord_t)(ny + 10), (lv_coord_t)(nx + 10), (lv_coord_t)(ny + 24)};
        lv_draw_label(ctx, &lab, &ta, "N", nullptr);
    }

    if (s_plate_rwy_n <= 0) {
        lv_area_t ta = {(lv_coord_t)(coords.x1 + 20), (lv_coord_t)(cy - 8), (lv_coord_t)(coords.x2 - 20),
                        (lv_coord_t)(cy + 16)};
        lv_draw_label(ctx, &lab, &ta, "No runway diagram", nullptr);
        return;
    }

    int max_len = 1;
    for (int i = 0; i < s_plate_rwy_n; i++) {
        if (s_plate_rwys[i].len_ft > max_len) {
            max_len = s_plate_rwys[i].len_ft;
        }
    }
    const float scale = (float)(LV_MIN(w, h) - 70) / (float)max_len;

    lv_draw_rect_dsc_t rwy_fill;
    lv_draw_rect_dsc_init(&rwy_fill);
    rwy_fill.bg_color = lv_color_hex(0x1A1A1A);
    rwy_fill.bg_opa = LV_OPA_COVER;

    for (int i = 0; i < s_plate_rwy_n; i++) {
        const Runway *r = &s_plate_rwys[i];
        const float rad = r->hdg_deg * FR_DEG2RAD;
        const float fx = sinf(rad);
        const float fy = -cosf(rad);
        const float rx = cosf(rad);
        const float ry = sinf(rad);
        const float half_l = 0.5f * r->len_ft * scale;
        float half_w = 0.5f * (float)LV_MAX(r->width_ft, 60) * scale;
        if (half_w < 3.0f) {
            half_w = 3.0f;
        }
        if (half_w > 10.0f) {
            half_w = 10.0f;
        }

        lv_point_t pts[4];
        const float ox[4] = {-half_l, half_l, half_l, -half_l};
        const float oy[4] = {-half_w, -half_w, half_w, half_w};
        for (int k = 0; k < 4; k++) {
            pts[k].x = (lv_coord_t)lroundf(cx + fx * ox[k] + rx * oy[k]);
            pts[k].y = (lv_coord_t)lroundf(cy + fy * ox[k] + ry * oy[k]);
        }
        lv_draw_polygon(ctx, &rwy_fill, pts, 4);

        // Centerline dashes.
        lv_draw_line_dsc_t cl;
        lv_draw_line_dsc_init(&cl);
        cl.color = lv_color_hex(0xFFFFFF);
        cl.width = 1;
        cl.opa = LV_OPA_COVER;
        lv_point_t p0 = {(lv_coord_t)lroundf(cx - fx * half_l * 0.85f), (lv_coord_t)lroundf(cy - fy * half_l * 0.85f)};
        lv_point_t p1 = {(lv_coord_t)lroundf(cx + fx * half_l * 0.85f), (lv_coord_t)lroundf(cy + fy * half_l * 0.85f)};
        lv_draw_line(ctx, &cl, &p0, &p1);

        // Runway end labels.
        lv_point_t le = {(lv_coord_t)lroundf(cx - fx * (half_l + 12)), (lv_coord_t)lroundf(cy - fy * (half_l + 12))};
        lv_point_t he = {(lv_coord_t)lroundf(cx + fx * (half_l + 12)), (lv_coord_t)lroundf(cy + fy * (half_l + 12))};
        lv_area_t lea = {(lv_coord_t)(le.x - 14), (lv_coord_t)(le.y - 8), (lv_coord_t)(le.x + 14), (lv_coord_t)(le.y + 10)};
        lv_area_t hea = {(lv_coord_t)(he.x - 14), (lv_coord_t)(he.y - 8), (lv_coord_t)(he.x + 14), (lv_coord_t)(he.y + 10)};
        lv_draw_label(ctx, &lab, &lea, r->le, nullptr);
        lv_draw_label(ctx, &lab, &hea, r->he, nullptr);
    }
}

// Airport / aircraft detail; right column holds stacked action buttons.
static constexpr lv_coord_t kDetailW = 540;
static constexpr lv_coord_t kDetailH = 360;
static constexpr lv_coord_t kDetailBtnW = 108;
static constexpr lv_coord_t kDetailBtnCol = 120;  // right stack reserve
static constexpr lv_coord_t kDetailPlateW = 180;
static constexpr lv_coord_t kDetailPlateH = 210;

static void format_radios_block(const AirportRadios *r, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (!r) {
        return;
    }
    // Only list radios that exist (e.g. KINF has CTAF+CTR, no ATIS/GND).
    char *p = out;
    size_t left = out_len;
    auto append_line = [&](const char *lbl, const char *freq) {
        if (!freq || !freq[0] || left < 8) {
            return;
        }
        const int n = snprintf(p, left, "%s%s  %s", (p == out) ? "" : "\n", lbl, freq);
        if (n > 0 && (size_t)n < left) {
            p += n;
            left -= (size_t)n;
        }
    };
    append_line("CTAF", r->ctaf);
    append_line((r->atc_is_center && r->atc[0]) ? "CTR" : "ATC", r->atc);
    append_line("GND", r->gnd);
    append_line((r->atis_is_awos && r->atis[0]) ? "AWOS" : "ATIS", r->atis);
}

static void layout_detail_body(bool with_side_panel) {
    if (!detail_body) {
        return;
    }
    if (with_side_panel) {
        // Airport plate on left; body between plate and button column.
        lv_obj_set_width(detail_body, kDetailW - kDetailBtnCol - (kDetailPlateW + 16));
        lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, (lv_coord_t)(kDetailPlateW + 8), 40);
    } else {
        lv_obj_set_width(detail_body, kDetailW - kDetailBtnCol - 16);
        lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, 8, 40);
    }
}

static void set_detail_plate(const char *icao) {
    s_plate_rwy_n = 0;
    s_plate_icao[0] = 0;
    s_detail_is_aircraft = false;
    if (icao && icao[0]) {
        strncpy(s_plate_icao, icao, sizeof(s_plate_icao) - 1);
        s_plate_icao[sizeof(s_plate_icao) - 1] = 0;
        s_plate_rwy_n = net_lookup_runways(s_plate_icao, s_plate_rwys, FR_MAX_RWYS_PER_APT);
    }
    if (!detail_plate || !detail_body) {
        return;
    }
    const bool have_bitmap = s_plate_icao[0] && faa_plate_lookup(s_plate_icao) != nullptr;
    if (s_plate_rwy_n > 0 || have_bitmap) {
        lv_obj_clear_flag(detail_plate, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(detail_plate);
        layout_detail_body(true);
    } else {
        lv_obj_add_flag(detail_plate, LV_OBJ_FLAG_HIDDEN);
        layout_detail_body(false);
    }
}

static void set_detail_aircraft_view() {
    s_detail_is_aircraft = true;
    if (detail_plate) {
        lv_obj_add_flag(detail_plate, LV_OBJ_FLAG_HIDDEN);
    }
    layout_detail_body(false);
}

static void update_detail_center_btn() {
    if (!detail_center_btn) {
        return;
    }
    if (s_detail_apt_ok && !s_detail_is_aircraft) {
        lv_obj_clear_flag(detail_center_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(detail_center_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_detail_follow_btn() {
    if (!detail_follow_btn) {
        return;
    }
    if (s_detail_is_aircraft && s_detail_ac.hex[0]) {
        lv_obj_clear_flag(detail_follow_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_t *lab = lv_obj_get_child(detail_follow_btn, 0);
        if (lab) {
            const bool same = s_follow && strcasecmp(s_follow_hex, s_detail_ac.hex) == 0;
            lv_label_set_text(lab, same ? "Unfollow" : "Follow");
        }
    } else {
        lv_obj_add_flag(detail_follow_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_detail_add_plate_btn() {
    if (!detail_add_plate_btn) {
        return;
    }
    if (s_detail_apt_ok && !s_detail_is_aircraft && s_detail_apt.icao[0]) {
        lv_obj_clear_flag(detail_add_plate_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_t *lab = lv_obj_get_child(detail_add_plate_btn, 0);
        if (lab) {
            lv_label_set_text(lab, plate_known(s_detail_apt.icao) ? "On List" : "Add Plate");
        }
    } else {
        lv_obj_add_flag(detail_add_plate_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_detail(const char *title, const char *body, const char *plate_icao) {
    if (plate_icao && plate_icao[0]) {
        set_detail_plate(plate_icao);
    } else if (s_detail_is_aircraft) {
        set_detail_aircraft_view();
    } else {
        set_detail_plate(nullptr);
    }
    update_detail_center_btn();
    update_detail_follow_btn();
    update_detail_add_plate_btn();
    lv_obj_clear_flag(detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(detail);
    lv_label_set_text(detail_title, title);
    lv_label_set_text(detail_body, body);
}

static void hide_detail(lv_event_t *e) {
    LV_UNUSED(e);
    lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    s_detail_is_aircraft = false;
    set_detail_plate(nullptr);
    s_detail_apt_ok = false;
    s_detail_city[0] = 0;
    memset(&s_detail_radios, 0, sizeof(s_detail_radios));
    update_detail_center_btn();
    update_detail_follow_btn();
    update_detail_add_plate_btn();
    memset(&s_flight_info, 0, sizeof(s_flight_info));
}

static void center_on_airport(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_detail_apt_ok || !g_state) {
        return;
    }

    stop_follow(false);

    GeoFix fix{};
    // Home airport: ICAO only — never facility name / address on the map or place label.
    strncpy(fix.icao, s_detail_apt.icao, sizeof(fix.icao) - 1);
    fix.icao[sizeof(fix.icao) - 1] = 0;
    strncpy(fix.label, s_detail_apt.icao, sizeof(fix.label) - 1);
    fix.label[sizeof(fix.label) - 1] = 0;
    strncpy(fix.query, s_detail_apt.icao, sizeof(fix.query) - 1);
    fix.query[sizeof(fix.query) - 1] = 0;
    fix.lat = s_detail_apt.lat;
    fix.lon = s_detail_apt.lon;
    fix.valid = true;

    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.location = fix;
    xSemaphoreGive(g_state->mutex);

    s_home_loc = fix;
    s_home_ok = true;
    s_olat = fix.lat;
    s_olon = fix.lon;
    map_cache_mark_dirty();
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);
    update_loc_label(fix.icao, fix.lat, fix.lon);

    hide_detail(nullptr);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    request(UI_REQ_SAVE_LOCATION | UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
}

/**
 * Begin (or switch) follow on an aircraft.
 * toggle_if_same: true = Follow/Unfollow button (tap same plane ends track);
 *                 false = radar/table/Track (tap same plane is ignored — keeps tracking).
 */
static void start_follow_aircraft(const Aircraft *ac, bool toggle_if_same) {
    if (!g_state || !ac || (!ac->hex[0] && !ac->flight[0]) || !ac->valid) {
        return;
    }

    const bool same_hex = ac->hex[0] && s_follow_hex[0] && strcasecmp(s_follow_hex, ac->hex) == 0;
    const bool same_flight =
        ac->flight[0] && s_follow_label[0] && strcasecmp(s_follow_label, ac->flight) == 0 && !ac->hex[0];
    if (s_follow && (same_hex || same_flight)) {
        if (!toggle_if_same) {
            return;  // already tracking this flight
        }
        stop_follow(true);
        hide_detail(nullptr);
        if (radar_plot) {
            lv_obj_invalidate(radar_plot);
        }
        return;
    }

    bool entering = false;
    if (!s_follow) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        s_follow_saved_loc = g_state->cfg.location;
        s_follow_saved_range = g_state->cfg.range_nm;
        s_follow_saved_max_alt = g_state->cfg.max_alt_ft;
        xSemaphoreGive(g_state->mutex);
        s_follow_saved_ok = s_follow_saved_loc.valid;
        s_follow_saved_view_ok = true;
        entering = true;
    }

    s_follow = true;
    s_follow_solo = true;
    s_follow_hold_center = false;
    s_follow_approach_zoom_off = false;
    s_follow_county_state = -2;
    s_follow_county_user_hid = false;
    s_wpt_view_mode = 0;  // show route names + line
    // Airports/cities off; counties ON for the aircraft's state (unless user later hides).
    // Helis / <100 kt tracks also force counties via follow_maybe_apply_speed_view.
    s_show_airports = false;
    s_show_counties = true;
    s_show_cities = false;
    s_follow_spd_view_done = false;
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->cfg.show_airports = false;
        g_state->cfg.show_counties = true;
        g_state->cfg.show_cities = false;
        xSemaphoreGive(g_state->mutex);
    }
    update_apt_toggle_label();
    update_cty_toggle_label();
    update_city_toggle_label();
    map_cache_mark_dirty();
    if (entering) {
        set_range(250.0f);
        set_max_alt(45000);
    }
    if (ac->hex[0]) {
        strncpy(s_follow_hex, ac->hex, sizeof(s_follow_hex) - 1);
        s_follow_hex[sizeof(s_follow_hex) - 1] = 0;
    } else {
        s_follow_hex[0] = 0;  // pre-departure: track by callsign until ADS-B hex appears
    }
    if (ac->flight[0]) {
        strncpy(s_follow_label, ac->flight, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
        trim_inplace(s_follow_label);
    } else {
        strncpy(s_follow_label, s_follow_hex, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
    }
    s_follow_ac = *ac;
    s_follow_ac_ok = true;
    sync_follow_to_state();
    follow_apt_clear();
    // Prefer detail flight-info when following from the aircraft panel.
    if (s_detail_is_aircraft && ac->hex[0] && strcasecmp(s_detail_ac.hex, ac->hex) == 0) {
        s_follow_fi = s_flight_info;
        follow_apt_update_from_fi(&s_follow_fi);
    } else if (s_detail_is_aircraft && ac->flight[0] && strcasecmp(s_detail_ac.flight, ac->flight) == 0 &&
               s_flight_info.origin[0]) {
        s_follow_fi = s_flight_info;
        follow_apt_update_from_fi(&s_follow_fi);
    } else {
        memset(&s_follow_fi, 0, sizeof(s_follow_fi));
    }

    trail_clear();
    hist_clear();
    s_follow_takeoff_ms = 0;
    s_follow_land_ms = 0;
    trail_push(ac->lat, ac->lon);
    hist_push(ac->on_ground ? 0.0f : ac->alt_ft, ac->gs_kts);
    follow_note_airborne_sample(ac);
    update_wpt_toggle_btn();
    set_follow_side_mode(true);
    refresh_follow_panel();
    follow_maybe_apply_speed_view(ac);

    // Until dep/arr are known, frame the aircraft; fit_dep_arr overrides when both resolve.
    if (!s_follow_route_fitted) {
        char label[48];
        snprintf(label, sizeof(label), "Following %s", s_follow_label);
        set_radar_center(ac->lat, ac->lon, label, UI_REQ_REFRESH_TRAFFIC);
    }
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);

    if (ac->flight[0] || ac->reg[0] || ac->hex[0]) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        strncpy(g_state->pending_flight, ac->flight, sizeof(g_state->pending_flight) - 1);
        g_state->pending_flight[sizeof(g_state->pending_flight) - 1] = 0;
        strncpy(g_state->pending_reg, ac->reg, sizeof(g_state->pending_reg) - 1);
        g_state->pending_reg[sizeof(g_state->pending_reg) - 1] = 0;
        strncpy(g_state->pending_hex, ac->hex, sizeof(g_state->pending_hex) - 1);
        g_state->pending_hex[sizeof(g_state->pending_hex) - 1] = 0;
        xSemaphoreGive(g_state->mutex);
        request(UI_REQ_FETCH_FLIGHT_INFO);
    }

    // Poll aircraft position every 5s so county layer tracks the current state.
    follow_state_timer_ensure(true);
    if (s_show_counties) {
        follow_refresh_county_state(true);
    }

    hide_detail(nullptr);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void follow_flight(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_detail_is_aircraft) {
        return;
    }
    start_follow_aircraft(&s_detail_ac, true);  // Unfollow if already on this aircraft
}

static void radar_draw(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const lv_coord_t w = lv_area_get_width(&coords);
    const lv_coord_t h = lv_area_get_height(&coords);
    const lv_coord_t cx = (coords.x1 + coords.x2) / 2;
    const lv_coord_t cy = (coords.y1 + coords.y2) / 2;
    const lv_coord_t R = (LV_MIN(w, h) / 2) - 6;

    if (!g_state) {
        return;
    }

    snapshot_radar_for_draw();
    // Prefer a pre-rasterized map blit (built off-FB). Fall back to a solid disk.
    if (map_cache_ensure(w, h, s_olat, s_olon, s_range) && s_map_ready) {
        lv_draw_img_dsc_t id;
        lv_draw_img_dsc_init(&id);
        lv_draw_img(ctx, &id, &coords, &s_map_img);
    } else {
        lv_draw_rect_dsc_t fill;
        lv_draw_rect_dsc_init(&fill);
        fill.bg_color = lv_color_hex(0x031208);
        fill.bg_opa = LV_OPA_COVER;
        fill.radius = LV_RADIUS_CIRCLE;
        lv_area_t circle = {(lv_coord_t)(cx - R), (lv_coord_t)(cy - R), (lv_coord_t)(cx + R), (lv_coord_t)(cy + R)};
        lv_draw_rect(ctx, &fill, &circle);
    }

    lv_draw_line_dsc_t ring;
    lv_draw_line_dsc_init(&ring);
    ring.color = COL_DIM;
    ring.width = 1;
    ring.opa = LV_OPA_70;
    lv_draw_rect_dsc_t ringr;
    lv_draw_rect_dsc_init(&ringr);
    ringr.bg_opa = LV_OPA_TRANSP;
    ringr.border_color = COL_DIM;
    ringr.border_width = 1;
    ringr.border_opa = LV_OPA_70;
    ringr.radius = LV_RADIUS_CIRCLE;
    for (int i = 1; i <= 4; i++) {
        const lv_coord_t rr = (lv_coord_t)(R * i / 4);
        lv_area_t ra = {(lv_coord_t)(cx - rr), (lv_coord_t)(cy - rr), (lv_coord_t)(cx + rr), (lv_coord_t)(cy + rr)};
        lv_draw_rect(ctx, &ringr, &ra);
    }
    lv_point_t n1 = {cx, (lv_coord_t)(cy - R)};
    lv_point_t n2 = {cx, (lv_coord_t)(cy + R)};
    lv_point_t e1 = {(lv_coord_t)(cx - R), cy};
    lv_point_t e2 = {(lv_coord_t)(cx + R), cy};
    lv_draw_line(ctx, &ring, &n1, &n2);
    lv_draw_line(ctx, &ring, &e1, &e2);

    if (s_follow && s_trail_count >= 2) {
        lv_draw_line_dsc_t path;
        lv_draw_line_dsc_init(&path);
        path.color = COL_PATH;
        path.width = 2;
        path.opa = LV_OPA_COVER;
        path.round_start = 1;
        path.round_end = 1;
        const int oldest = (s_trail_head - s_trail_count + kTrailMax) % kTrailMax;
        lv_coord_t px = 0, py = 0;
        bool have = false;
        for (int i = 0; i < s_trail_count; i++) {
            const TrailPt &pt = s_trail[(oldest + i) % kTrailMax];
            lv_coord_t x, y;
            if (!map_point_ll(pt.lat, pt.lon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                have = false;
                continue;
            }
            x += coords.x1;
            y += coords.y1;
            if (have) {
                lv_point_t a = {px, py};
                lv_point_t b = {x, y};
                lv_draw_line(ctx, &path, &a, &b);
            }
            px = x;
            py = y;
            have = true;
        }
    }

    lv_draw_label_dsc_t lab;
    lv_draw_label_dsc_init(&lab);
    lab.font = &lv_font_montserrat_12;
    lab.color = COL_TEXT;

    // Airport toggle works even while tracking (enter-follow starts with airports off).
    // Solo home mark uses locked s_home_loc (not pan center) so dragging away moves it off-center.
    {
        const char *home_icao = (!s_follow && s_home_ok && s_home_loc.icao[0]) ? s_home_loc.icao : nullptr;
        auto is_home_apt = [&](const Airport *a) -> bool {
            if (!home_icao || !a) {
                return false;
            }
            if (strcasecmp(a->icao, home_icao) == 0) {
                return true;
            }
            if (a->iata[0] && strcasecmp(a->iata, home_icao) == 0) {
                return true;
            }
            if (strlen(home_icao) == 4 && (home_icao[0] == 'K' || home_icao[0] == 'k') && a->iata[0] &&
                strcasecmp(a->iata, home_icao + 1) == 0) {
                return true;
            }
            return false;
        };
        auto draw_apt_mark = [&](float alat, float alon, const char *code, lv_color_t col, lv_coord_t sz,
                                 const lv_font_t *font, lv_color_t text_col) {
            lv_coord_t x, y;
            if (!map_point_ll(alat, alon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                return;
            }
            x += coords.x1;
            y += coords.y1;
            lv_draw_rect_dsc_t mark;
            lv_draw_rect_dsc_init(&mark);
            mark.bg_color = col;
            mark.bg_opa = LV_OPA_COVER;
            mark.radius = 2;
            lv_area_t m = {(lv_coord_t)(x - sz / 2), (lv_coord_t)(y - sz / 2), (lv_coord_t)(x + sz / 2),
                           (lv_coord_t)(y + sz / 2)};
            lv_draw_rect(ctx, &mark, &m);
            lv_draw_label_dsc_t apt_lab;
            lv_draw_label_dsc_init(&apt_lab);
            apt_lab.font = font;
            apt_lab.color = text_col;
            const lv_coord_t th = font ? font->line_height : 12;
            lv_area_t ta = {(lv_coord_t)(x + 5), (lv_coord_t)(y - th / 2), (lv_coord_t)(x + 120),
                            (lv_coord_t)(y + th / 2 + 2)};
            lv_draw_label(ctx, &apt_lab, &ta, code, nullptr);
        };
        // Home airport: light blue + larger than normal airport labels (10 → 14).
        auto draw_home_apt = [&](float alat, float alon, const char *code) {
            draw_apt_mark(alat, alon, code, COL_HOME, 6, &lv_font_montserrat_14, COL_HOME);
        };
        // City/ZIP home: light blue name, larger than airport codes.
        auto draw_home_city = [&]() {
            if (s_follow || !s_home_ok || s_home_loc.icao[0] || !s_home_loc.label[0]) {
                return;
            }
            lv_coord_t x, y;
            if (!map_point_ll(s_home_loc.lat, s_home_loc.lon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                return;
            }
            x += coords.x1;
            y += coords.y1;
            lv_draw_label_dsc_t home_lab;
            lv_draw_label_dsc_init(&home_lab);
            home_lab.font = &lv_font_montserrat_14;
            home_lab.color = COL_HOME;
            lv_point_t sz;
            lv_txt_get_size(&sz, s_home_loc.label, home_lab.font, home_lab.letter_space, home_lab.line_space,
                            LV_COORD_MAX, home_lab.flag);
            lv_area_t ta = {(lv_coord_t)(x - sz.x / 2), (lv_coord_t)(y - sz.y / 2 - 2),
                            (lv_coord_t)(x + sz.x / 2 + 1), (lv_coord_t)(y + sz.y / 2 + 2)};
            lv_draw_label(ctx, &home_lab, &ta, s_home_loc.label, nullptr);
        };
        auto draw_home_airport_mark = [&]() {
            if (!home_icao) {
                return;
            }
            const char *mark_txt = home_icao;
            bool drawn = false;
            for (int i = 0; i < s_apt_n; i++) {
                const Airport *a = &s_apt[i];
                if (!is_home_apt(a)) {
                    continue;
                }
                mark_txt = a->iata[0] ? a->iata : a->icao;
                draw_home_apt(a->lat, a->lon, mark_txt);
                drawn = true;
                break;
            }
            if (!drawn) {
                // US ICAO Kxxx → show xxx when IATA list isn't loaded yet.
                if (strlen(home_icao) == 4 && (home_icao[0] == 'K' || home_icao[0] == 'k')) {
                    mark_txt = home_icao + 1;
                }
                draw_home_apt(s_home_loc.lat, s_home_loc.lon, mark_txt);
            }
        };
        if (s_show_airports) {
            for (int i = 0; i < s_apt_n; i++) {
                const Airport *a = &s_apt[i];
                if (is_home_apt(a)) {
                    continue;  // drawn highlighted below
                }
                const char *code = a->iata[0] ? a->iata : a->icao;
                draw_apt_mark(a->lat, a->lon, code, COL_MAP, a->iata[0] ? 4 : 3, &lv_font_montserrat_10, COL_MAP);
            }
            draw_home_airport_mark();
            draw_home_city();
        } else if (home_icao) {
            draw_home_airport_mark();
        } else {
            draw_home_city();
        }
    }

    // Major cities — pop floor rises with zoom-out so labels stay readable.
    // <500 nm: >=100k; >=500 nm: >=250k; >=1000 nm: >=500k.
    if (s_show_cities) {
        uint32_t min_pop = 100000;
        if (s_range >= 1000.0f) {
            min_pop = 500000;
        } else if (s_range >= 500.0f) {
            min_pop = 250000;
        }
        lv_draw_label_dsc_t city_lab;
        lv_draw_label_dsc_init(&city_lab);
        city_lab.font = &lv_font_montserrat_10;
        city_lab.color = COL_MAP;
        for (size_t i = 0; i < K_CITY_DB_COUNT; i++) {
            if (kCityDb[i].pop < min_pop) {
                continue;
            }
            const float clat = kCityDb[i].lat_c / 100.0f;
            const float clon = kCityDb[i].lon_c / 100.0f;
            lv_coord_t x, y;
            if (!map_point_ll(clat, clon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                continue;
            }
            x += coords.x1;
            y += coords.y1;
            lv_draw_rect_dsc_t mark;
            lv_draw_rect_dsc_init(&mark);
            mark.bg_color = COL_MAP;
            mark.bg_opa = LV_OPA_COVER;
            mark.radius = 1;
            const lv_coord_t sz = 3;
            lv_area_t m = {(lv_coord_t)(x - sz / 2), (lv_coord_t)(y - sz / 2), (lv_coord_t)(x + sz / 2),
                           (lv_coord_t)(y + sz / 2)};
            lv_draw_rect(ctx, &mark, &m);
            lv_area_t ta = {(lv_coord_t)(x + 4), (lv_coord_t)(y - 6), (lv_coord_t)(x + 100), (lv_coord_t)(y + 8)};
            lv_draw_label(ctx, &city_lab, &ta, kCityDb[i].name, nullptr);
        }
    }

    // At ≤15 nm: overlay mini airport plates (cached; no per-frame runway DB scan).
    if (s_range <= 15.5f) {
        lv_coord_t plate_sz = 56;
        if (s_range <= 4.5f) {
            plate_sz = 220;
        } else if (s_range <= 5.5f) {
            plate_sz = 160;
        } else if (s_range <= 15.5f) {
            plate_sz = 90;
        }
        map_plates_ensure();
        for (int i = 0; i < s_map_plate_n; i++) {
            const MapPlateEntry &e = s_map_plates[i];
            lv_coord_t x, y;
            if (!map_point_ll(e.lat, e.lon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                continue;
            }
            draw_map_airport_plate(ctx, (lv_coord_t)(coords.x1 + x), (lv_coord_t)(coords.y1 + y), plate_sz,
                                   e.icao, e.rwys, e.rwy_n);
        }
    }

    // Persistent route waypoints + dep/arr while following (clamp to rim if off-scale).
    // Mode 0 = all, 1 = hide names, 2 = hide line + mid waypoints (dep/arr stay visible).
    // On final (≤15 nm): hide route lines and waypoint labels — plates own the view.
    if (s_follow && s_follow_wpt_n > 0 && s_range > 15.5f) {
        auto project_wpt = [&](float lat, float lon, lv_coord_t *ox, lv_coord_t *oy) {
            float east = 0, north = 0;
            ll_to_nm(lat, lon, s_olat, s_olon, &east, &north);
            float ex = (s_range > 0.1f) ? (east / s_range) : 0.0f;
            float ny = (s_range > 0.1f) ? (-north / s_range) : 0.0f;
            const float len = hypotf(ex, ny);
            if (len > 1.02f && len > 1e-4f) {
                ex /= len;
                ny /= len;
            }
            *ox = (lv_coord_t)lroundf((float)cx + ex * (float)R);
            *oy = (lv_coord_t)lroundf((float)cy + ny * (float)R);
        };

        // Route polyline through waypoints (hidden in mode 2).
        if (s_wpt_view_mode != 2) {
            lv_draw_line_dsc_t route_ln;
            lv_draw_line_dsc_init(&route_ln);
            route_ln.color = COL_WPT;
            route_ln.width = 1;  // thin light-blue filed route
            route_ln.opa = LV_OPA_COVER;
            route_ln.round_start = 1;
            route_ln.round_end = 1;
            lv_coord_t prev_x = 0, prev_y = 0;
            bool have_prev = false;
            for (int i = 0; i < s_follow_wpt_n; i++) {
                if (!s_follow_wpts[i].ok) {
                    have_prev = false;
                    continue;
                }
                lv_coord_t x, y;
                project_wpt(s_follow_wpts[i].lat, s_follow_wpts[i].lon, &x, &y);
                if (have_prev) {
                    lv_point_t a = {prev_x, prev_y};
                    lv_point_t b = {x, y};
                    lv_draw_line(ctx, &route_ln, &a, &b);
                }
                prev_x = x;
                prev_y = y;
                have_prev = true;
            }
        }

        for (int i = 0; i < s_follow_wpt_n; i++) {
            const FollowWpt &w = s_follow_wpts[i];
            if (!w.ok) {
                continue;
            }
            const bool is_end = (i == 0 || i == s_follow_wpt_n - 1);
            // Mode 2: keep dep/arr airports only.
            if (s_wpt_view_mode == 2 && !is_end) {
                continue;
            }
            lv_coord_t x, y;
            project_wpt(w.lat, w.lon, &x, &y);
            const lv_color_t col = (i == 0) ? COL_APT_LG : ((i == s_follow_wpt_n - 1) ? COL_APT_MD : COL_WPT);
            lv_draw_rect_dsc_t mark;
            lv_draw_rect_dsc_init(&mark);
            mark.bg_color = col;
            mark.bg_opa = LV_OPA_COVER;
            mark.radius = is_end ? LV_RADIUS_CIRCLE : 2;
            mark.border_color = lv_color_hex(0xFFFFFF);
            mark.border_width = 1;
            mark.border_opa = LV_OPA_COVER;
            const lv_coord_t hs = is_end ? 5 : 3;
            lv_area_t m = {(lv_coord_t)(x - hs), (lv_coord_t)(y - hs), (lv_coord_t)(x + hs), (lv_coord_t)(y + hs)};
            lv_draw_rect(ctx, &mark, &m);

            // Mode 1: hide mid-waypoint names only; dep/arr labels stay (same as mode 2).
            if ((s_wpt_view_mode == 1 && !is_end) || !w.label[0]) {
                continue;
            }
            const char *name = w.label;
            // Montserrat 12 caps need ~8–9 px; keep one line (no wrap on 5-letter fixes).
            lv_point_t tsz = {0, 0};
            lv_txt_get_size(&tsz, name, &lv_font_montserrat_12, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            const lv_coord_t pad = 4;
            const lv_coord_t tw = (lv_coord_t)(tsz.x + pad * 2);
            const lv_coord_t th = (lv_coord_t)(tsz.y + 2);

            // Local route tangent so we can park the label above/below the track when needed.
            float tdx = 0.0f, tdy = 0.0f;
            if (i > 0 && s_follow_wpts[i - 1].ok) {
                lv_coord_t px, py;
                project_wpt(s_follow_wpts[i - 1].lat, s_follow_wpts[i - 1].lon, &px, &py);
                tdx += (float)(x - px);
                tdy += (float)(y - py);
            }
            if (i + 1 < s_follow_wpt_n && s_follow_wpts[i + 1].ok) {
                lv_coord_t nx, ny;
                project_wpt(s_follow_wpts[i + 1].lat, s_follow_wpts[i + 1].lon, &nx, &ny);
                tdx += (float)(nx - x);
                tdy += (float)(ny - y);
            }
            float tlen = hypotf(tdx, tdy);
            if (tlen < 1e-3f) {
                tdx = 1.0f;
                tdy = 0.0f;
                tlen = 1.0f;
            }
            tdx /= tlen;
            tdy /= tlen;
            // Unit normals (perpendicular to track).
            const float n1x = -tdy, n1y = tdx;
            const float n2x = tdy, n2y = -tdx;

            auto label_hits_track = [&](lv_coord_t lx, lv_coord_t ly) -> bool {
                // Corner/center samples vs nearby route segments.
                const float samples[5][2] = {
                    {(float)(lx + tw / 2), (float)(ly + th / 2)},
                    {(float)lx, (float)(ly + th / 2)},
                    {(float)(lx + tw), (float)(ly + th / 2)},
                    {(float)(lx + tw / 2), (float)ly},
                    {(float)(lx + tw / 2), (float)(ly + th)},
                };
                const float thresh2 = 7.0f * 7.0f;  // ~line width + padding
                auto dist2_seg = [](float px, float py, float ax, float ay, float bx, float by) -> float {
                    const float abx = bx - ax, aby = by - ay;
                    const float apx = px - ax, apy = py - ay;
                    const float ab2 = abx * abx + aby * aby;
                    float t = (ab2 > 1e-6f) ? (apx * abx + apy * aby) / ab2 : 0.0f;
                    if (t < 0.0f) {
                        t = 0.0f;
                    } else if (t > 1.0f) {
                        t = 1.0f;
                    }
                    const float qx = ax + t * abx, qy = ay + t * aby;
                    const float dx = px - qx, dy = py - qy;
                    return dx * dx + dy * dy;
                };
                for (int s = 0; s < 5; s++) {
                    const float px = samples[s][0], py = samples[s][1];
                    if (i > 0 && s_follow_wpts[i - 1].ok) {
                        lv_coord_t ax, ay;
                        project_wpt(s_follow_wpts[i - 1].lat, s_follow_wpts[i - 1].lon, &ax, &ay);
                        if (dist2_seg(px, py, (float)ax, (float)ay, (float)x, (float)y) < thresh2) {
                            return true;
                        }
                    }
                    if (i + 1 < s_follow_wpt_n && s_follow_wpts[i + 1].ok) {
                        lv_coord_t bx, by;
                        project_wpt(s_follow_wpts[i + 1].lat, s_follow_wpts[i + 1].lon, &bx, &by);
                        if (dist2_seg(px, py, (float)x, (float)y, (float)bx, (float)by) < thresh2) {
                            return true;
                        }
                    }
                }
                return false;
            };

            // Prefer right of mark; if that sits on the track, park above/below the line.
            lv_coord_t tx = (lv_coord_t)(x + hs + 3);
            lv_coord_t ty = (lv_coord_t)(y - th / 2);
            if (label_hits_track(tx, ty)) {
                const float gap = (float)(hs + 4) + (float)th * 0.5f;
                // Prefer the normal pointing more "up" on screen (negative y).
                float ux = n1x, uy = n1y;
                if (uy > n2y) {
                    ux = n2x;
                    uy = n2y;
                }
                tx = (lv_coord_t)lroundf((float)x + ux * gap - (float)tw * 0.5f);
                ty = (lv_coord_t)lroundf((float)y + uy * gap - (float)th * 0.5f);
                if (label_hits_track(tx, ty)) {
                    // Flip to the other side of the track.
                    tx = (lv_coord_t)lroundf((float)x - ux * gap - (float)tw * 0.5f);
                    ty = (lv_coord_t)lroundf((float)y - uy * gap - (float)th * 0.5f);
                }
            }

            // Rim marks (dep/arr often clamped to edge): park label toward center.
            {
                const float rdx = (float)x - (float)cx;
                const float rdy = (float)y - (float)cy;
                const float rlen = hypotf(rdx, rdy);
                if (rlen > (float)R * 0.82f && rlen > 1e-3f) {
                    const float ix = -rdx / rlen;
                    const float iy = -rdy / rlen;
                    const float gap = (float)(hs + 4) + 0.5f * (float)LV_MAX(tw, th);
                    tx = (lv_coord_t)lroundf((float)x + ix * gap - (float)tw * 0.5f);
                    ty = (lv_coord_t)lroundf((float)y + iy * gap - (float)th * 0.5f);
                }
            }
            // Hard-clamp so the whole label box stays inside the PPI circle.
            {
                float max_r = (float)R - 0.5f * hypotf((float)tw, (float)th) - 2.0f;
                if (max_r < 4.0f) {
                    max_r = 4.0f;
                }
                float mx = (float)tx + 0.5f * (float)tw;
                float my = (float)ty + 0.5f * (float)th;
                const float dx = mx - (float)cx;
                const float dy = my - (float)cy;
                const float dist = hypotf(dx, dy);
                if (dist > max_r && dist > 1e-3f) {
                    const float s = max_r / dist;
                    mx = (float)cx + dx * s;
                    my = (float)cy + dy * s;
                    tx = (lv_coord_t)lroundf(mx - 0.5f * (float)tw);
                    ty = (lv_coord_t)lroundf(my - 0.5f * (float)th);
                }
            }

            lv_draw_rect_dsc_t name_bg;
            lv_draw_rect_dsc_init(&name_bg);
            name_bg.bg_color = lv_color_hex(0x020806);
            name_bg.bg_opa = LV_OPA_70;
            name_bg.radius = 2;
            lv_area_t nba = {tx, ty, (lv_coord_t)(tx + tw), (lv_coord_t)(ty + th)};
            lv_draw_rect(ctx, &name_bg, &nba);
            lab.color = col;
            lab.font = &lv_font_montserrat_12;
            lab.opa = LV_OPA_COVER;
            lab.flag = LV_TEXT_FLAG_EXPAND;  // never wrap waypoint id across lines
            lv_area_t ta = {(lv_coord_t)(tx + pad), ty, (lv_coord_t)(tx + tw), (lv_coord_t)(ty + th)};
            lv_draw_label(ctx, &lab, &ta, name, nullptr);
            lab.flag = LV_TEXT_FLAG_NONE;
        }
    }
    for (int i = 0; i < s_ac_n; i++) {
        const Aircraft *ac = &s_ac[i];
        if (!aircraft_shown(ac)) {
            continue;
        }
        lv_coord_t x, y;
        if (!map_point_ll(ac->lat, ac->lon, s_olat, s_olon, s_range, w, h, &x, &y)) {
            continue;
        }
        x += coords.x1;
        y += coords.y1;
        if (aircraft_is_helicopter(ac)) {
            draw_helicopter_icon(ctx, x, y, ac->track_deg, COL_AC);
        } else {
            draw_airplane_icon(ctx, x, y, ac->track_deg, aircraft_size_class(ac), COL_AC);
        }
        const char *id = ac->flight[0] ? ac->flight : ac->hex;
        lv_area_t ta = {(lv_coord_t)(x + 10), (lv_coord_t)(y - 6), (lv_coord_t)(x + 90), (lv_coord_t)(y + 12)};
        lab.color = COL_AC;
        lv_draw_label(ctx, &lab, &ta, id, nullptr);
    }
}

static void format_term_gate_line(char *out, size_t n, const char *term, const char *gate) {
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    if (term && term[0] && gate && gate[0]) {
        snprintf(out, n, "Term-%s Gate-%s", term, gate);
    } else if (gate && gate[0]) {
        snprintf(out, n, "Gate-%s", gate);
    } else if (term && term[0]) {
        snprintf(out, n, "Term-%s", term);
    } else {
        snprintf(out, n, "Term-- Gate--");
    }
}

static void format_aircraft(const Aircraft *ac, char *title, size_t tlen, char *body, size_t blen,
                            const FlightInfo *fi, bool route_pending) {
    snprintf(title, tlen, "%s", ac->flight[0] ? ac->flight : ac->hex);
    char alt[24];
    if (ac->on_ground) {
        strncpy(alt, "GND", sizeof(alt) - 1);
    } else {
        snprintf(alt, sizeof(alt), "%.0f ft", ac->alt_ft);
    }
    char route[64];
    if (fi && fi->route_ok && fi->origin[0] && fi->dest[0]) {
        snprintf(route, sizeof(route), "%s  →  %s", fi->origin, fi->dest);
    } else if (route_pending && (ac->flight[0] || ac->reg[0] || ac->hex[0])) {
        strncpy(route, ac->flight[0] ? "looking up by flight..." : "looking up...", sizeof(route) - 1);
        route[sizeof(route) - 1] = 0;
    } else {
        strncpy(route, "unavailable", sizeof(route) - 1);
        route[sizeof(route) - 1] = 0;
    }
    const char *reg_disp = "--";
    if (fi && fi->reg[0]) {
        reg_disp = fi->reg;
    } else if (ac->reg[0]) {
        reg_disp = ac->reg;
    }
    const char *type_line = "--";
    char type_buf[56];
    if (fi && fi->type_name[0]) {
        type_line = fi->type_name;
    } else if (ac->type[0]) {
        type_line = ac->type;
    }
    if (fi && fi->manufacturer[0] && ac->type[0]) {
        snprintf(type_buf, sizeof(type_buf), "%s (%s)", fi->type_name[0] ? fi->type_name : ac->type, fi->manufacturer);
        type_line = type_buf;
    }
    const char *owner = (fi && fi->owner[0]) ? fi->owner : "--";
    char dep_line[96] = "--";
    char arr_line[96] = "--";
    char dep_times[72] = "";
    char arr_times[72] = "";
    char gate_line[48] = "";
    if (fi && fi->origin[0]) {
        if (fi->origin_name[0]) {
            snprintf(dep_line, sizeof(dep_line), "%s (%s)", fi->origin_name, fi->origin);
        } else {
            snprintf(dep_line, sizeof(dep_line), "%s", fi->origin);
        }
    }
    if (fi && fi->dest[0]) {
        if (fi->dest_name[0]) {
            snprintf(arr_line, sizeof(arr_line), "%s (%s)", fi->dest_name, fi->dest);
        } else {
            snprintf(arr_line, sizeof(arr_line), "%s", fi->dest);
        }
    }
    if (fi && (fi->dep_time[0] || fi->dep_act[0])) {
        snprintf(dep_times, sizeof(dep_times), "Dep Sch  %s   Act %s", fi->dep_time[0] ? fi->dep_time : "--",
                 fi->dep_act[0] ? fi->dep_act : "--");
    }
    if (fi && (fi->arr_time[0] || fi->arr_est[0])) {
        snprintf(arr_times, sizeof(arr_times), "Arr Sch  %s   Est %s", fi->arr_time[0] ? fi->arr_time : "--",
                 fi->arr_est[0] ? fi->arr_est : "--");
    }
    if (fi && (fi->dep_gate[0] || fi->arr_gate[0] || fi->dep_terminal[0] || fi->arr_terminal[0] ||
               ac->on_ground || (fi->status[0]))) {
        char dep_g[28] = "", arr_g[28] = "";
        format_term_gate_line(dep_g, sizeof(dep_g), fi->dep_terminal, fi->dep_gate);
        format_term_gate_line(arr_g, sizeof(arr_g), fi->arr_terminal, fi->arr_gate);
        char live_st[40] = {0};
        follow_live_status(ac, fi, live_st, sizeof(live_st));
        if ((fi->dep_gate[0] || fi->dep_terminal[0] || fi->arr_gate[0] || fi->arr_terminal[0])) {
            snprintf(gate_line, sizeof(gate_line), "Gate     Dep %s  Arr %s%s", dep_g, arr_g,
                     ac->on_ground ? "  (at airport)" : "");
        } else if (live_st[0]) {
            snprintf(gate_line, sizeof(gate_line), "Status   %s", live_st);
        } else if (ac->on_ground) {
            strncpy(gate_line, "Status   On ground at airport", sizeof(gate_line) - 1);
        }
    }
    snprintf(body, blen,
             "Flight  %s\n"
             "Tail    %s\n"
             "Route   %s\n"
             "Dep     %s\n"
             "%s%s"
             "Arr     %s\n"
             "%s%s"
             "%s%s"
             "ICAO24  %s\n"
             "Type    %s\n"
             "Owner   %s\n"
             "Alt     %s\n"
             "Speed   %.0f kt\n"
             "Track   %.0f deg\n"
             "V/S     %+.0f fpm\n"
             "Squawk  %s\n"
             "Pos     %.4f, %.4f\n"
             "Range   %.1f nm",
             ac->flight[0] ? ac->flight : "--",
             reg_disp,
             route,
             dep_line,
             dep_times[0] ? dep_times : "",
             dep_times[0] ? "\n" : "",
             arr_line,
             arr_times[0] ? arr_times : "",
             arr_times[0] ? "\n" : "",
             gate_line[0] ? gate_line : "",
             gate_line[0] ? "\n" : "",
             ac->hex[0] ? ac->hex : "--",
             type_line,
             owner,
             alt,
             ac->gs_kts, ac->track_deg, ac->vrate_fpm,
             ac->squawk[0] ? ac->squawk : "--",
             ac->lat, ac->lon, ac->dst_nm);
}

static void open_aircraft_detail(const Aircraft *ac) {
    if (!ac) {
        return;
    }
    s_detail_ac = *ac;
    s_detail_is_aircraft = true;
    s_detail_apt_ok = false;
    memset(&s_flight_info, 0, sizeof(s_flight_info));
    char title[32], body[768];
    format_aircraft(&s_detail_ac, title, sizeof(title), body, sizeof(body), nullptr, true);
    show_detail(title, body, nullptr);
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        strncpy(g_state->pending_flight, s_detail_ac.flight, sizeof(g_state->pending_flight) - 1);
        g_state->pending_flight[sizeof(g_state->pending_flight) - 1] = 0;
        strncpy(g_state->pending_reg, s_detail_ac.reg, sizeof(g_state->pending_reg) - 1);
        g_state->pending_reg[sizeof(g_state->pending_reg) - 1] = 0;
        strncpy(g_state->pending_hex, s_detail_ac.hex, sizeof(g_state->pending_hex) - 1);
        g_state->pending_hex[sizeof(g_state->pending_hex) - 1] = 0;
        memset(&g_state->flight_info, 0, sizeof(g_state->flight_info));
        xSemaphoreGive(g_state->mutex);
        if (s_detail_ac.flight[0] || s_detail_ac.reg[0] || s_detail_ac.hex[0]) {
            request(UI_REQ_FETCH_FLIGHT_INFO);
        } else {
            format_aircraft(&s_detail_ac, title, sizeof(title), body, sizeof(body), nullptr, false);
            lv_label_set_text(detail_body, body);
        }
    }
}

// Drag-to-pan state for radar_plot (finger swipe moves the map center).
static bool s_radar_drag_active = false;
static bool s_radar_did_pan = false;
static lv_coord_t s_radar_drag_last_x = 0;
static lv_coord_t s_radar_drag_last_y = 0;
static float s_radar_drag_dist = 0.0f;

static void pan_radar_by_nm(float east_nm, float north_nm) {
    if (east_nm == 0.0f && north_nm == 0.0f) {
        return;
    }
    float lat = s_olat + north_nm / 60.0f;
    float clat = cosf(s_olat * FR_DEG2RAD);
    if (clat < 0.15f) {
        clat = 0.15f;
    }
    float lon = s_olon + east_nm / (60.0f * clat);
    if (lat > 85.0f) {
        lat = 85.0f;
    }
    if (lat < -85.0f) {
        lat = -85.0f;
    }
    while (lon > 180.0f) {
        lon -= 360.0f;
    }
    while (lon < -180.0f) {
        lon += 360.0f;
    }
    char label[64];
    if (s_follow) {
        s_follow_hold_center = true;
        sync_follow_to_state();
        snprintf(label, sizeof(label), "Following %s", s_follow_label[0] ? s_follow_label : s_follow_hex);
    } else {
        // Pan is a temporary view — do not overwrite saved home / ICAO in NVS.
        snprintf(label, sizeof(label), "Local Radar");
    }
    // Refresh traffic for the new center; never persist pan as home location.
    set_radar_center(lat, lon, label, UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void radar_press(lv_event_t *e) {
    if (lvgl_port_pinch_active()) {
        s_radar_drag_active = false;
        s_radar_did_pan = true;
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_point_t p;
    lv_indev_get_point(lv_indev_get_act(), &p);
    s_radar_drag_last_x = p.x - obj->coords.x1;
    s_radar_drag_last_y = p.y - obj->coords.y1;
    s_radar_drag_dist = 0.0f;
    s_radar_did_pan = false;
    s_radar_drag_active = true;
}

static void radar_pressing(lv_event_t *e) {
    if (lvgl_port_pinch_active()) {
        s_radar_drag_active = false;
        s_radar_did_pan = true;
        return;
    }
    if (!s_radar_drag_active) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_point_t p;
    lv_indev_get_point(lv_indev_get_act(), &p);
    const lv_coord_t x = p.x - obj->coords.x1;
    const lv_coord_t y = p.y - obj->coords.y1;
    const lv_coord_t dx = x - s_radar_drag_last_x;
    const lv_coord_t dy = y - s_radar_drag_last_y;
    if (dx == 0 && dy == 0) {
        return;
    }
    s_radar_drag_dist += hypotf((float)dx, (float)dy);
    s_radar_drag_last_x = x;
    s_radar_drag_last_y = y;

    const lv_coord_t w = lv_obj_get_width(obj);
    const lv_coord_t h = lv_obj_get_height(obj);
    const float R = (float)((LV_MIN(w, h) / 2) - 6);
    if (R < 1.0f) {
        return;
    }
    const float nm_per_px = s_range / R;
    // Finger right → map follows → center moves west; finger down → center moves north.
    const float east_nm = -(float)dx * nm_per_px;
    const float north_nm = (float)dy * nm_per_px;
    pan_radar_by_nm(east_nm, north_nm);
    if (s_radar_drag_dist > (float)kRadarPanClickThresh) {
        s_radar_did_pan = true;
    }
}

static void radar_clicked(lv_event_t *e) {
    // LVGL still fires CLICKED after a drag — skip pick if we panned.
    const bool was_pan = s_radar_did_pan || s_radar_drag_dist > (float)kRadarPanClickThresh;
    s_radar_drag_active = false;
    s_radar_did_pan = false;
    s_radar_drag_dist = 0.0f;
    if (was_pan) {
        return;
    }

    lv_obj_t *obj = lv_event_get_target(e);
    lv_point_t p;
    lv_indev_get_point(lv_indev_get_act(), &p);
    lv_coord_t x = p.x - obj->coords.x1;
    lv_coord_t y = p.y - obj->coords.y1;
    const lv_coord_t w = lv_obj_get_width(obj);
    const lv_coord_t h = lv_obj_get_height(obj);

    int best_ac = -1, best_apt = -1;
    float best_ac_d = 22.0f, best_apt_d = 26.0f;
    snapshot_radar();
    for (int i = 0; i < s_ac_n; i++) {
        if (!aircraft_shown(&s_ac[i])) {
            continue;
        }
        lv_coord_t ax, ay;
        if (!map_point_ll(s_ac[i].lat, s_ac[i].lon, s_olat, s_olon, s_range, w, h, &ax, &ay)) {
            continue;
        }
        float d = hypotf((float)(ax - x), (float)(ay - y));
        if (d < best_ac_d) {
            best_ac_d = d;
            best_ac = i;
        }
    }
    if (s_show_airports) {
        for (int i = 0; i < s_apt_n; i++) {
            lv_coord_t ax, ay;
            if (!map_point_ll(s_apt[i].lat, s_apt[i].lon, s_olat, s_olon, s_range, w, h, &ax, &ay)) {
                continue;
            }
            float d = hypotf((float)(ax - x), (float)(ay - y));
            if (d < best_apt_d) {
                best_apt_d = d;
                best_apt = i;
            }
        }
    }
    if (best_ac >= 0 && (best_apt < 0 || best_ac_d <= best_apt_d)) {
        // Tap aircraft on map → track immediately (no detail popup).
        // Do not toggle off if already following this one (accidental tap ≠ Unfollow).
        start_follow_aircraft(&s_ac[best_ac], false);
        return;
    }
    if (best_apt >= 0) {
        const Airport *a = &s_apt[best_apt];
        s_detail_apt = *a;
        s_detail_apt_ok = true;
        strncpy(g_state->pending_icao, a->icao, sizeof(g_state->pending_icao) - 1);
        char title[40], body[420];
        s_detail_city[0] = 0;
        memset(&s_detail_radios, 0, sizeof(s_detail_radios));
        plates_city_from_name(a->name, s_detail_city, sizeof(s_detail_city));
        net_lookup_airport_radios(a->icao, &s_detail_radios);
        char radios[96];
        format_radios_block(&s_detail_radios, radios, sizeof(radios));
        snprintf(title, sizeof(title), "%s  %s", a->icao, a->iata);
        snprintf(body, sizeof(body),
                 "%s\n%s %s\nElev %d ft\nRwy  %d ft\nPos  %.4f, %.4f\n"
                 "%s\n\nLoading METAR / TAF...",
                 a->name, a->state, a->country, a->elev_ft, a->longest_rwy_ft, a->lat, a->lon, radios);
        show_detail(title, body, a->icao);
        request(UI_REQ_FETCH_WEATHER);
        return;
    }
}

static void set_range_ex(float nm, bool request_refresh) {
    // While tracking, allow approach zooms down to the nearest kZoomNear step (4 nm).
    const float min_nm = s_follow ? kZoomNear[0] : 10.0f;
    if (nm < min_nm) {
        nm = min_nm;
    }
    const float max_nm = s_follow ? kZoomFarMax : 250.0f;
    if (nm > max_nm) {
        nm = max_nm;
    }
    nm = snap_zoom_level(nm, s_follow);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const float old = g_state->cfg.range_nm;
    if (fabsf(old - nm) < 0.5f) {
        xSemaphoreGive(g_state->mutex);
        s_range = nm;
        return;
    }
    g_state->cfg.range_nm = nm;
    s_olat = g_state->cfg.location.lat;
    s_olon = g_state->cfg.location.lon;
    xSemaphoreGive(g_state->mutex);
    s_range = nm;
    map_cache_mark_dirty();
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);
    char buf[32];
    snprintf(buf, sizeof(buf), "%.0f nm", nm);
    if (radar_range_lbl) {
        lv_label_set_text(radar_range_lbl, buf);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    if (!request_refresh) {
        return;
    }
    // Zoom buttons change the live view only — Startup Settings persist via Location panel.
    // While tracking with airports on, re-query Class B when zooming out so hubs fill the PPI.
    uint32_t req = UI_REQ_REFRESH_TRAFFIC;
    if (nm > old + 0.5f && (!s_follow || s_show_airports)) {
        req |= UI_REQ_REFRESH_AIRPORTS;
    }
    request(req);
}

static void set_range(float nm) {
    set_range_ex(nm, true);
}

/** dir +1 zoom in, -1 zoom out — shared by Zoom buttons and pinch. */
static void radar_zoom_step(int dir) {
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const float range = g_state->cfg.range_nm;
    xSemaphoreGive(g_state->mutex);
    bool changed = false;
    if (dir > 0) {
        const float next = zoom_in_level(range, s_follow);
        if (next < range - 0.5f) {
            set_range(next);
            changed = true;
        }
    } else if (dir < 0) {
        const float next = zoom_out_level(range, s_follow);
        if (next > range + 0.5f) {
            set_range(next);
            changed = true;
        }
    }
    // Manual zoom while tracking cancels landing auto-zoom for this follow.
    if (changed && s_follow) {
        s_follow_approach_zoom_off = true;
    }
}

static void zoom_out(lv_event_t *e) {
    LV_UNUSED(e);
    radar_zoom_step(-1);
}

static void zoom_in(lv_event_t *e) {
    LV_UNUSED(e);
    radar_zoom_step(+1);
}

static void on_pinch_zoom(int dir) {
    // Cancel any in-progress pan and suppress a bogus click after pinch.
    s_radar_drag_active = false;
    s_radar_did_pan = true;
    s_radar_drag_dist = (float)kRadarPanClickThresh + 1.0f;
    // Invert vs port: pinch-in = zoom in, spread = zoom out.
    radar_zoom_step(-dir);
}

static void set_max_alt_ex(int alt_ft, bool persist) {
    if (alt_ft < kAltMinFt) {
        alt_ft = kAltMinFt;
    }
    if (alt_ft > kAltMaxFt) {
        alt_ft = kAltMaxFt;
    }
    alt_ft = (alt_ft / kAltStepFt) * kAltStepFt;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    if (g_state->cfg.max_alt_ft == alt_ft) {
        xSemaphoreGive(g_state->mutex);
        s_max_alt = alt_ft;
        return;
    }
    g_state->cfg.max_alt_ft = alt_ft;
    xSemaphoreGive(g_state->mutex);
    s_max_alt = alt_ft;
    char buf[24];
    format_alt_ceiling(alt_ft, buf, sizeof(buf));
    if (radar_alt_lbl) {
        lv_label_set_text(radar_alt_lbl, buf);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    // ALT +/- changes the live ceiling only — Startup Settings persist via Location panel.
    LV_UNUSED(persist);
}

static void set_max_alt(int alt_ft) {
    set_max_alt_ex(alt_ft, false);
}

static void alt_up(lv_event_t *e) {
    LV_UNUSED(e);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const int cur = g_state->cfg.max_alt_ft;
    xSemaphoreGive(g_state->mutex);
    if (cur < kAltMaxFt) {
        set_max_alt(cur + kAltStepFt);
    }
}

static void alt_down(lv_event_t *e) {
    LV_UNUSED(e);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const int cur = g_state->cfg.max_alt_ft;
    xSemaphoreGive(g_state->mutex);
    if (cur > kAltMinFt) {
        set_max_alt(cur - kAltStepFt);
    }
}

static lv_obj_t *settings_panel = nullptr;
static lv_obj_t *sound_panel = nullptr;
static lv_obj_t *county_panel = nullptr;
static lv_obj_t *county_cbs[50] = {};
static lv_obj_t *county_select_all_cb = nullptr;
static bool s_county_bulk = false;
static constexpr uint64_t kCountyMaskAll = kCountyMaskAllBits;
static lv_obj_t *bright_slider = nullptr;
static lv_obj_t *bright_pct_lbl = nullptr;
static lv_obj_t *vol_slider = nullptr;
static lv_obj_t *vol_pct_lbl = nullptr;

static void hide_sound_panel(lv_event_t *e) {
    LV_UNUSED(e);
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    // Leave Settings closed after a selection.
}

static void sync_county_select_all() {
    if (!county_select_all_cb) {
        return;
    }
    s_county_bulk = true;
    if ((s_county_mask & kCountyMaskAll) == kCountyMaskAll) {
        lv_obj_add_state(county_select_all_cb, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(county_select_all_cb, LV_STATE_CHECKED);
    }
    s_county_bulk = false;
}

static void sync_county_checkboxes() {
    s_county_bulk = true;
    for (int i = 0; i < 50; i++) {
        if (!county_cbs[i]) {
            continue;
        }
        if (s_county_mask & (1ULL << i)) {
            lv_obj_add_state(county_cbs[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(county_cbs[i], LV_STATE_CHECKED);
        }
    }
    s_county_bulk = false;
    sync_county_select_all();
}

static void hide_county_panel(lv_event_t *e) {
    LV_UNUSED(e);
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    // Leave Settings closed after a selection.
}

static void apply_county_mask(uint64_t mask) {
    s_county_mask = mask & kCountyMaskAll;
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->cfg.county_mask = s_county_mask;
        xSemaphoreGive(g_state->mutex);
    }
    map_cache_mark_dirty();
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    request(UI_REQ_SAVE_COUNTY_MASK);
}

static void county_cb_changed(lv_event_t *e) {
    if (s_county_bulk) {
        return;
    }
    lv_obj_t *cb = (lv_obj_t *)lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= 50) {
        return;
    }
    const bool on = lv_obj_has_state(cb, LV_STATE_CHECKED);
    uint64_t mask = s_county_mask;
    if (on) {
        mask |= (1ULL << idx);
    } else {
        mask &= ~(1ULL << idx);
    }
    apply_county_mask(mask);
    sync_county_select_all();
}

static void county_select_all_changed(lv_event_t *e) {
    if (s_county_bulk) {
        return;
    }
    lv_obj_t *cb = (lv_obj_t *)lv_event_get_target(e);
    const bool on = lv_obj_has_state(cb, LV_STATE_CHECKED);
    apply_county_mask(on ? kCountyMaskAll : 0);
    sync_county_checkboxes();
}

static void goto_counties(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        sync_county_checkboxes();
        lv_obj_clear_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(county_panel);
    }
}

static void hide_wifi(lv_event_t *e) {
    LV_UNUSED(e);
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void sync_startup_dropdowns();
static void apply_startup_dropdowns(bool persist);

static void hide_location(lv_event_t *e) {
    LV_UNUSED(e);
    apply_startup_dropdowns(true);
    if (loc_kb) {
        lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_panel) {
        lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_location_keyboard() {
    if (!loc_kb || !loc_ta) {
        return;
    }
    lv_keyboard_set_textarea(loc_kb, loc_ta);
    lv_obj_clear_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(loc_kb);
}

static void loc_ta_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    show_location_keyboard();
}

static void open_wifi_panel() {
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (detail) {
        lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_kb) {
        lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_panel) {
        lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_clear_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(wifi_panel);
    }
    // Always mask password before any text is set (avoids a plaintext flash).
    if (wifi_show_pw_cb) {
        lv_obj_clear_state(wifi_show_pw_cb, LV_STATE_CHECKED);
    }
    if (wifi_ta) {
        lv_textarea_set_password_mode(wifi_ta, true);
        lv_textarea_set_password_show_time(wifi_ta, 0);  // never flash typed chars
    }
    // Prefill saved credentials so reconnect works without retyping.
    if (g_state && g_state->cfg.wifi_ssid[0]) {
        strncpy(g_state->pending_ssid, g_state->cfg.wifi_ssid, sizeof(g_state->pending_ssid) - 1);
        g_state->pending_ssid[sizeof(g_state->pending_ssid) - 1] = 0;
        if (wifi_ta) {
            lv_textarea_set_text(wifi_ta, g_state->cfg.wifi_pass);
        }
        if (wifi_status) {
            char msg[80];
            snprintf(msg, sizeof(msg), "Saved: %s — Scan or Connect", g_state->cfg.wifi_ssid);
            lv_label_set_text(wifi_status, msg);
        }
    } else if (wifi_status) {
        lv_label_set_text(wifi_status, "Scan networks, then Connect");
    }
    refresh_wifi_connected_lbl();
}

static void hide_settings(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void sync_sound_panel_controls() {
    int bright = 100;
    int vol = 60;
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        bright = g_state->cfg.brightness;
        vol = g_state->cfg.volume;
        xSemaphoreGive(g_state->mutex);
    }
    if (bright < 0) {
        bright = 0;
    }
    if (bright > 100) {
        bright = 100;
    }
    if (vol < 0) {
        vol = 0;
    }
    if (vol > 100) {
        vol = 100;
    }
    char buf[16];
    if (bright_slider) {
        lv_slider_set_value(bright_slider, bright, LV_ANIM_OFF);
    }
    if (bright_pct_lbl) {
        snprintf(buf, sizeof(buf), "%d%%", bright);
        lv_label_set_text(bright_pct_lbl, buf);
    }
    if (vol_slider) {
        lv_slider_set_value(vol_slider, vol, LV_ANIM_OFF);
    }
    if (vol_pct_lbl) {
        snprintf(buf, sizeof(buf), "%d%%", vol);
        lv_label_set_text(vol_pct_lbl, buf);
    }
}

static void goto_sound(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        sync_sound_panel_controls();
        lv_obj_clear_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(sound_panel);
    }
}

static void bright_slider_changed(lv_event_t *e) {
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int v = (int)lv_slider_get_value(slider);
    if (v < 0) {
        v = 0;
    }
    if (v > 100) {
        v = 100;
    }
    board_set_brightness(v);
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->cfg.brightness = v;
        xSemaphoreGive(g_state->mutex);
    }
    if (bright_pct_lbl) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", v);
        lv_label_set_text(bright_pct_lbl, buf);
    }
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        request(UI_REQ_SAVE_BRIGHTNESS);
    }
}

static void vol_slider_changed(lv_event_t *e) {
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int v = (int)lv_slider_get_value(slider);
    if (v < 0) {
        v = 0;
    }
    if (v > 100) {
        v = 100;
    }
    audio_set_volume(v);
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->cfg.volume = v;
        xSemaphoreGive(g_state->mutex);
    }
    if (vol_pct_lbl) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", v);
        lv_label_set_text(vol_pct_lbl, buf);
    }
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        audio_click_play();
        request(UI_REQ_SAVE_VOLUME);
    }
}

static void open_settings(lv_event_t *e) {
    LV_UNUSED(e);
    if (detail) {
        lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_kb) {
        lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_panel) {
        lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (settings_panel) {
        lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(settings_panel);
    }
}

static void style_two_line_btn_label(lv_obj_t *btn, lv_coord_t btn_w) {
    lv_obj_t *lab = lv_obj_get_child(btn, 0);
    if (!lab) {
        return;
    }
    lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(lab, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_line_space(lab, -2, 0);
    lv_label_set_long_mode(lab, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lab, btn_w - 6);
    lv_obj_center(lab);
}

static void update_apt_toggle_label() {
    if (!apt_toggle_btn) {
        return;
    }
    lv_obj_t *lbl = lv_obj_get_child(apt_toggle_btn, 0);
    if (lbl) {
        lv_label_set_text(lbl, s_show_airports ? "Hide\nAirport" : "Show\nAirport");
    }
}

static void update_cty_toggle_label() {
    if (!cty_toggle_btn) {
        return;
    }
    lv_obj_t *lbl = lv_obj_get_child(cty_toggle_btn, 0);
    if (lbl) {
        lv_label_set_text(lbl, s_show_counties ? "Hide\nCounty" : "Show\nCounty");
    }
}

static void update_city_toggle_label() {
    if (!city_toggle_btn) {
        return;
    }
    lv_obj_t *lbl = lv_obj_get_child(city_toggle_btn, 0);
    if (lbl) {
        lv_label_set_text(lbl, s_show_cities ? "Hide\nCity" : "Show\nCity");
    }
}

static void toggle_airports(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.show_airports = !g_state->cfg.show_airports;
    s_show_airports = g_state->cfg.show_airports;
    xSemaphoreGive(g_state->mutex);
    update_apt_toggle_label();
    // While tracking, re-query so the DB search only returns Class B (not a full list).
    if (s_follow && s_show_airports) {
        request(UI_REQ_REFRESH_AIRPORTS);
    }
    // Airports are drawn live; map disk cache unchanged.
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    // Live toggle only — Startup Settings (boot/Home) persist via Location panel.
}

static void toggle_counties(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.show_counties = !g_state->cfg.show_counties;
    s_show_counties = g_state->cfg.show_counties;
    xSemaphoreGive(g_state->mutex);
    // Remember intentional hide while tracking so we don't force counties back on.
    if (s_follow) {
        s_follow_county_user_hid = !s_show_counties;
    }
    update_cty_toggle_label();
    // While tracking, resolve the aircraft's state immediately when counties turn on.
    if (s_follow && s_show_counties) {
        follow_refresh_county_state(true);
    } else {
        map_cache_mark_dirty();
        if (radar_plot) {
            lv_obj_invalidate(radar_plot);
        }
    }
    // Live toggle only — Startup Settings (boot/Home) persist via Location panel.
}

static void toggle_cities(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.show_cities = !g_state->cfg.show_cities;
    s_show_cities = g_state->cfg.show_cities;
    xSemaphoreGive(g_state->mutex);
    update_city_toggle_label();
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void toggle_wpt_labels(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_follow) {
        return;
    }
    // Triple: show all → hide names → hide path → show all.
    s_wpt_view_mode = (s_wpt_view_mode + 1) % 3;
    update_wpt_toggle_btn();
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void plates_hide_keyboard() {
    if (plates_kb) {
        lv_obj_add_flag(plates_kb, LV_OBJ_FLAG_HIDDEN);
    }
    plates_kb_target = nullptr;
}

static void plates_show_keyboard(lv_obj_t *ta) {
    if (!plates_kb || !ta) {
        return;
    }
    const bool already =
        plates_kb_target == ta && !lv_obj_has_flag(plates_kb, LV_OBJ_FLAG_HIDDEN);
    plates_kb_target = ta;
    lv_keyboard_set_textarea(plates_kb, ta);
    lv_obj_clear_flag(plates_kb, LV_OBJ_FLAG_HIDDEN);
    // Avoid move_foreground thrash on repeated FOCUSED/CLICKED (causes panel flicker).
    if (!already) {
        lv_obj_move_foreground(plates_panel);
        lv_obj_move_foreground(plates_kb);
    }
}

static void plates_fill_info_table() {
    if (!plates_info_tbl) {
        return;
    }
    const char *flbl[4] = {};
    const char *ffreq[4] = {};
    const int freq_n = plate_freq_lines(&s_plates_radios, flbl, ffreq);

    int order[FR_MAX_RWYS_PER_APT];
    for (int i = 0; i < s_plates_prev_rwy_n; i++) {
        order[i] = i;
    }
    for (int a = 0; a < s_plates_prev_rwy_n; a++) {
        for (int b = a + 1; b < s_plates_prev_rwy_n; b++) {
            if (strcmp(s_plates_prev_rwys[order[a]].le, s_plates_prev_rwys[order[b]].le) > 0) {
                const int tmp = order[a];
                order[a] = order[b];
                order[b] = tmp;
            }
        }
    }

    const int data_n = s_plates_prev_rwy_n + freq_n;
    const uint16_t rows = (uint16_t)(1 + (data_n > 0 ? data_n : 1));
    lv_table_set_col_cnt(plates_info_tbl, 2);
    lv_table_set_row_cnt(plates_info_tbl, rows);
    lv_table_set_cell_value(plates_info_tbl, 0, 0, "RWY / FREQ");
    lv_table_set_cell_value(plates_info_tbl, 0, 1, "VALUE");

    if (data_n <= 0) {
        lv_table_set_cell_value(plates_info_tbl, 1, 0, "--");
        lv_table_set_cell_value(plates_info_tbl, 1, 1, "--");
        return;
    }

    uint16_t row = 1;
    for (int k = 0; k < s_plates_prev_rwy_n; k++) {
        const Runway *r = &s_plates_prev_rwys[order[k]];
        char id[16];
        char val[24];
        if (r->he[0]) {
            snprintf(id, sizeof(id), "%s/%s", r->le, r->he);
        } else {
            snprintf(id, sizeof(id), "%s", r->le);
        }
        snprintf(val, sizeof(val), "%d ft", r->len_ft);
        lv_table_set_cell_value(plates_info_tbl, row, 0, id);
        lv_table_set_cell_value(plates_info_tbl, row, 1, val);
        row++;
    }
    for (int k = 0; k < freq_n; k++) {
        lv_table_set_cell_value(plates_info_tbl, row, 0, flbl[k]);
        lv_table_set_cell_value(plates_info_tbl, row, 1, ffreq[k]);
        row++;
    }
}

static void plates_load_preview(const char *icao) {
    s_plates_prev_rwy_n = 0;
    s_plates_prev_icao[0] = 0;
    memset(&s_plates_radios, 0, sizeof(s_plates_radios));
    if (icao && icao[0]) {
        strncpy(s_plates_prev_icao, icao, sizeof(s_plates_prev_icao) - 1);
        s_plates_prev_icao[sizeof(s_plates_prev_icao) - 1] = 0;
        s_plates_prev_rwy_n =
            net_lookup_runways(s_plates_prev_icao, s_plates_prev_rwys, FR_MAX_RWYS_PER_APT);
        net_lookup_airport_radios(s_plates_prev_icao, &s_plates_radios);
    }
    if (plates_preview) {
        lv_obj_invalidate(plates_preview);
    }
    if (plates_title_lbl) {
        const AirportPlateInfo *info = airport_plate_at(airport_plate_index(icao));
        if (info) {
            char t[96];
            snprintf(t, sizeof(t), "%s  %s — %s", info->icao, info->iata, info->name);
            lv_label_set_text(plates_title_lbl, t);
        } else if (icao && icao[0]) {
            char t[48];
            snprintf(t, sizeof(t), "%s  (added)", icao);
            lv_label_set_text(plates_title_lbl, t);
        } else {
            lv_label_set_text(plates_title_lbl, "Airport Plates");
        }
    }
    plates_fill_info_table();
    if (plates_city_lbl) {
        // Keep last city-refresh result unless starting a fresh empty preview.
        if (!icao || !icao[0]) {
            lv_label_set_text(plates_city_lbl, "");
        }
    }
}

static void plates_preview_draw(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const lv_coord_t w = lv_area_get_width(&coords);
    const lv_coord_t h = lv_area_get_height(&coords);
    const lv_coord_t cx = (coords.x1 + coords.x2) / 2;
    const lv_coord_t cy = (coords.y1 + coords.y2) / 2;
    const lv_coord_t side = LV_MIN(w, h) - 8;
    if (draw_faa_plate_bitmap(ctx, &coords, s_plates_prev_icao)) {
        return;
    }
    if (s_plates_prev_rwy_n > 0) {
        draw_runway_plate(ctx, cx, cy, side, s_plates_prev_icao, s_plates_prev_rwys,
                          s_plates_prev_rwy_n);
    } else {
        lv_draw_label_dsc_t lab;
        lv_draw_label_dsc_init(&lab);
        lab.font = &lv_font_montserrat_14;
        lab.color = COL_DIM;
        lv_area_t ta = {(lv_coord_t)(coords.x1 + 12), (lv_coord_t)(cy - 8), (lv_coord_t)(coords.x2 - 12),
                        (lv_coord_t)(cy + 16)};
        lv_draw_label(ctx, &lab, &ta, "No runway data", nullptr);
    }
}

static void plates_list_item_clicked(lv_event_t *e) {
    const int filt_i = (int)(intptr_t)lv_event_get_user_data(e);
    if (filt_i < 0 || filt_i >= s_plates_filt_n) {
        return;
    }
    s_plates_sel = s_plates_filt[filt_i];
    const char *icao = plates_entry_icao(s_plates_sel);
    if (!icao) {
        return;
    }
    plates_load_preview(icao);
}

static int plates_filt_icao_cmp(const void *a, const void *b) {
    const int ia = *(const int *)a;
    const int ib = *(const int *)b;
    // Custom (NVS) plates first, then built-ins; A–Z by ICAO within each group.
    const bool a_custom = ia >= AIRPORT_PLATE_COUNT;
    const bool b_custom = ib >= AIRPORT_PLATE_COUNT;
    if (a_custom != b_custom) {
        return a_custom ? -1 : 1;
    }
    const char *ca = plates_entry_icao(ia);
    const char *cb = plates_entry_icao(ib);
    if (!ca) {
        ca = "";
    }
    if (!cb) {
        cb = "";
    }
    return strcasecmp(ca, cb);
}

static void plates_refresh_list() {
    if (!plates_list) {
        return;
    }
    lv_obj_clean(plates_list);
    s_plates_filt_n = 0;
    char q[40] = {0};
    if (plates_ta) {
        const char *t = lv_textarea_get_text(plates_ta);
        if (t) {
            size_t n = 0;
            for (; t[n] && n + 1 < sizeof(q); n++) {
                q[n] = (char)tolower((unsigned char)t[n]);
            }
            q[n] = 0;
        }
    }
    const int total = plates_total_count();
    for (int i = 0; i < total && s_plates_filt_n < FR_PLATES_LIST_MAX; i++) {
        char hay[128];
        if (i < AIRPORT_PLATE_COUNT) {
            const AirportPlateInfo *info = &kAirportPlates[i];
            snprintf(hay, sizeof(hay), "%s %s %s", info->icao, info->iata, info->name);
        } else {
            const int e = i - AIRPORT_PLATE_COUNT;
            const char *icao = s_plate_extra[e];
            snprintf(hay, sizeof(hay), "%s %s", icao, s_plate_extra_city[e]);
        }
        if (q[0]) {
            for (char *p = hay; *p; p++) {
                *p = (char)tolower((unsigned char)*p);
            }
            if (!strstr(hay, q)) {
                continue;
            }
        }
        s_plates_filt[s_plates_filt_n++] = i;
    }
    // Custom plates on top (A–Z), then built-ins (A–Z).
    if (s_plates_filt_n > 1) {
        qsort(s_plates_filt, (size_t)s_plates_filt_n, sizeof(s_plates_filt[0]), plates_filt_icao_cmp);
    }
    for (int fi = 0; fi < s_plates_filt_n; fi++) {
        const int i = s_plates_filt[fi];
        char label[96];
        if (i < AIRPORT_PLATE_COUNT) {
            const AirportPlateInfo *info = &kAirportPlates[i];
            char city[40];
            plates_city_from_name(info->name, city, sizeof(city));
            if (city[0]) {
                snprintf(label, sizeof(label), "%s  %s", info->icao, city);
            } else {
                snprintf(label, sizeof(label), "%s  %s", info->icao, info->iata);
            }
        } else {
            const int e = i - AIRPORT_PLATE_COUNT;
            const char *icao = s_plate_extra[e];
            if (s_plate_extra_city[e][0]) {
                snprintf(label, sizeof(label), "%s  %s", icao, s_plate_extra_city[e]);
            } else {
                snprintf(label, sizeof(label), "%s  +", icao);
            }
        }
        lv_obj_t *btn = lv_list_add_btn(plates_list, nullptr, label);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0A120C), 0);
        lv_obj_set_style_text_color(btn, COL_TEXT, 0);
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_14, 0);
        // Chop long city names — no horizontal scroll on list labels.
        const uint32_t nch = lv_obj_get_child_cnt(btn);
        for (uint32_t ci = 0; ci < nch; ci++) {
            lv_obj_t *ch = lv_obj_get_child(btn, ci);
            if (lv_obj_check_type(ch, &lv_label_class)) {
                lv_label_set_long_mode(ch, LV_LABEL_LONG_CLIP);
                lv_obj_set_width(ch, 320);
                break;
            }
        }
        lv_obj_add_event_cb(btn, plates_list_item_clicked, LV_EVENT_CLICKED, (void *)(intptr_t)fi);
    }
    if (s_plates_filt_n == 0) {
        if (plates_status) {
            lv_label_set_text(plates_status, "No matching airports");
        }
        plates_load_preview(nullptr);
        return;
    }
    bool sel_ok = false;
    for (int i = 0; i < s_plates_filt_n; i++) {
        if (s_plates_filt[i] == s_plates_sel) {
            sel_ok = true;
            break;
        }
    }
    if (!sel_ok) {
        s_plates_sel = s_plates_filt[0];
    }
    const char *icao = plates_entry_icao(s_plates_sel);
    if (icao) {
        plates_load_preview(icao);
    }
}

static void plates_search_changed(lv_event_t *e) {
    LV_UNUSED(e);
    plates_refresh_list();
}

static void plates_normalize_code(const char *raw, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (!raw) {
        return;
    }
    size_t n = 0;
    for (const char *p = raw; *p && n + 1 < out_len; p++) {
        if (*p == ' ' || *p == '\t') {
            continue;
        }
        out[n++] = (char)toupper((unsigned char)*p);
    }
    out[n] = 0;
}

static void plates_add_hit_clicked(lv_event_t *e);
static void plates_add_icao(const char *icao_raw, const char *city_opt);
static void update_detail_add_plate_btn();
static void open_plates_for_icao(const char *icao);
static void plates_request_city_refresh();

static void plates_request_city_refresh() {
    if (!g_state || s_plate_extra_n <= 0) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    int n = s_plate_extra_n;
    if (n > (int)(sizeof(g_state->plate_city_icao) / sizeof(g_state->plate_city_icao[0]))) {
        n = (int)(sizeof(g_state->plate_city_icao) / sizeof(g_state->plate_city_icao[0]));
    }
    g_state->plate_city_n = n;
    for (int i = 0; i < n; i++) {
        strncpy(g_state->plate_city_icao[i], s_plate_extra[i], sizeof(g_state->plate_city_icao[i]) - 1);
        g_state->plate_city_icao[i][sizeof(g_state->plate_city_icao[i]) - 1] = 0;
        g_state->plate_city_name[i][0] = 0;
    }
    xSemaphoreGive(g_state->mutex);
    if (plates_city_lbl) {
        lv_label_set_text(plates_city_lbl, "Refreshing city names...");
    }
    request(UI_REQ_REFRESH_PLATE_CITIES);
}

/** Short place name for plates list (CITY/FACILITY → CITY, else truncate name). */
static void plates_city_from_name(const char *name, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (!name || !name[0]) {
        return;
    }
    const char *slash = strchr(name, '/');
    if (slash && slash > name) {
        size_t len = (size_t)(slash - name);
        while (len > 0 && name[len - 1] == ' ') {
            len--;
        }
        if (len >= out_len) {
            len = out_len - 1;
        }
        memcpy(out, name, len);
        out[len] = 0;
        return;
    }
    strncpy(out, name, out_len - 1);
    out[out_len - 1] = 0;
}

static void plates_refresh_add_hits_ui() {
    if (!plates_add_hits) {
        return;
    }
    lv_obj_clean(plates_add_hits);
    if (s_plate_hit_n <= 0) {
        lv_obj_add_flag(plates_add_hits, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(plates_add_hits, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < s_plate_hit_n; i++) {
        if (!s_plate_hits[i].valid || !s_plate_hits[i].icao[0]) {
            continue;
        }
        const char *lbl = s_plate_hits[i].label[0] ? s_plate_hits[i].label : s_plate_hits[i].icao;
        lv_obj_t *btn = lv_list_add_btn(plates_add_hits, nullptr, lbl);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0A120C), 0);
        lv_obj_set_style_text_color(btn, COL_TEXT, 0);
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_12, 0);
        lv_obj_add_event_cb(btn, plates_add_hit_clicked, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

static void plates_add_icao(const char *icao_raw, const char *city_opt) {
    char icao[8] = {0};
    plates_normalize_code(icao_raw, icao, sizeof(icao));
    if (!icao[0]) {
        if (plates_status) {
            lv_label_set_text(plates_status, "Enter an airport code");
        }
        return;
    }
    // Resolve 3-letter IATA → ICAO when possible.
    if (strlen(icao) == 3) {
        char icao4[8] = {0};
        float lat = 0, lon = 0;
        if (net_lookup_airport_iata(icao, icao4, sizeof(icao4), &lat, &lon) && icao4[0]) {
            strncpy(icao, icao4, sizeof(icao) - 1);
            icao[sizeof(icao) - 1] = 0;
        }
    }
    if (plate_known(icao)) {
        if (plates_status) {
            lv_label_set_text(plates_status, "Already on plates list");
        }
        int idx = airport_plate_index(icao);
        if (idx < 0) {
            for (int i = 0; i < s_plate_extra_n; i++) {
                if (strcasecmp(s_plate_extra[i], icao) == 0) {
                    idx = AIRPORT_PLATE_COUNT + i;
                    break;
                }
            }
        }
        if (idx >= 0) {
            s_plates_sel = idx;
            plates_refresh_list();
        }
        plates_hide_keyboard();
        return;
    }
    Runway tmp[FR_MAX_RWYS_PER_APT];
    const int rn = net_lookup_runways(icao, tmp, FR_MAX_RWYS_PER_APT);
    if (rn <= 0) {
        if (plates_status) {
            lv_label_set_text(plates_status, "No runway data for that airport");
        }
        return;
    }
    if (s_plate_extra_n >= FR_PLATE_EXTRA_MAX) {
        if (plates_status) {
            lv_label_set_text(plates_status, "Custom plate list full");
        }
        return;
    }
    strncpy(s_plate_extra[s_plate_extra_n], icao, FR_PLATE_EXTRA_ID_LEN - 1);
    s_plate_extra[s_plate_extra_n][FR_PLATE_EXTRA_ID_LEN - 1] = 0;
    s_plate_extra_city[s_plate_extra_n][0] = 0;
    if (city_opt && city_opt[0]) {
        strncpy(s_plate_extra_city[s_plate_extra_n], city_opt, FR_PLATE_EXTRA_CITY_LEN - 1);
        s_plate_extra_city[s_plate_extra_n][FR_PLATE_EXTRA_CITY_LEN - 1] = 0;
    } else if (s_detail_apt_ok && strcasecmp(s_detail_apt.icao, icao) == 0 && s_detail_city[0]) {
        strncpy(s_plate_extra_city[s_plate_extra_n], s_detail_city, FR_PLATE_EXTRA_CITY_LEN - 1);
        s_plate_extra_city[s_plate_extra_n][FR_PLATE_EXTRA_CITY_LEN - 1] = 0;
    }
    s_plates_sel = AIRPORT_PLATE_COUNT + s_plate_extra_n;
    s_plate_extra_n++;
    config_save_plate_extras(s_plate_extra, s_plate_extra_city, s_plate_extra_n);
    map_plates_mark_dirty();
    s_plate_hit_n = 0;
    plates_refresh_add_hits_ui();
    if (plates_add_ta) {
        lv_textarea_set_text(plates_add_ta, "");
    }
    plates_refresh_list();
    plates_hide_keyboard();
    if (plates_status) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Added %s (%d rwys)", icao, rn);
        lv_label_set_text(plates_status, msg);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    // Always refresh city names for custom plates (fills blanks + corrects stale).
    plates_request_city_refresh();
}

static void open_plates_for_icao(const char *icao) {
    if (!icao || !icao[0] || !plates_panel) {
        return;
    }
    hide_detail(nullptr);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    s_plates_from_settings = false;
    config_load_plate_extras(s_plate_extra, s_plate_extra_city, &s_plate_extra_n);
    int idx = airport_plate_index(icao);
    if (idx < 0) {
        for (int i = 0; i < s_plate_extra_n; i++) {
            if (strcasecmp(s_plate_extra[i], icao) == 0) {
                idx = AIRPORT_PLATE_COUNT + i;
                break;
            }
        }
    }
    if (idx >= 0) {
        s_plates_sel = idx;
    }
    if (plates_ta) {
        lv_textarea_set_text(plates_ta, "");
    }
    if (plates_add_ta) {
        lv_textarea_set_text(plates_add_ta, "");
    }
    s_plate_hit_n = 0;
    plates_refresh_add_hits_ui();
    plates_refresh_list();
    lv_obj_clear_flag(plates_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(plates_panel);
    plates_hide_keyboard();
    plates_request_city_refresh();
}

static void detail_add_plate_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_detail_apt_ok || !s_detail_apt.icao[0]) {
        return;
    }
    if (plate_known(s_detail_apt.icao)) {
        open_plates_for_icao(s_detail_apt.icao);
        return;
    }
    char city[FR_PLATE_EXTRA_CITY_LEN] = {0};
    if (s_detail_city[0]) {
        strncpy(city, s_detail_city, sizeof(city) - 1);
    } else {
        plates_city_from_name(s_detail_apt.name, city, sizeof(city));
    }
    plates_add_icao(s_detail_apt.icao, city[0] ? city : nullptr);
    update_detail_add_plate_btn();
}

static void plates_add_hit_clicked(lv_event_t *e) {
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_plate_hit_n) {
        return;
    }
    char city[FR_PLATE_EXTRA_CITY_LEN] = {0};
    plates_city_from_name(s_plate_hits[idx].label, city, sizeof(city));
    plates_add_icao(s_plate_hits[idx].icao, city[0] ? city : nullptr);
}

static void plates_add_search_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!plates_add_ta || !g_state) {
        return;
    }
    const char *q = lv_textarea_get_text(plates_add_ta);
    if (!q || !q[0]) {
        plates_show_keyboard(plates_add_ta);
        return;
    }
    char code[8];
    plates_normalize_code(q, code, sizeof(code));
    const size_t n = strlen(code);
    bool code_like = (n == 3 || n == 4);
    for (size_t i = 0; code_like && i < n; i++) {
        if (!isalnum((unsigned char)code[i])) {
            code_like = false;
        }
    }
    if (code_like) {
        plates_add_icao(code, nullptr);
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->pending_query, q, sizeof(g_state->pending_query) - 1);
    g_state->pending_query[sizeof(g_state->pending_query) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    s_plates_search_pending = true;
    if (plates_status) {
        lv_label_set_text(plates_status, "Searching airports...");
    }
    request(UI_REQ_SEARCH_AIRPORTS);
}

static void plates_ta_clicked(lv_event_t *e) {
    lv_obj_t *ta = lv_event_get_target(e);
    plates_show_keyboard(ta);
}

static void plates_kb_event(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        if (plates_kb_target == plates_add_ta) {
            plates_add_search_clicked(e);
        } else {
            plates_hide_keyboard();
        }
    } else if (code == LV_EVENT_CANCEL) {
        plates_hide_keyboard();
    }
}

static void hide_plates_panel(lv_event_t *e) {
    LV_UNUSED(e);
    plates_hide_keyboard();
    s_plates_search_pending = false;
    if (plates_panel) {
        lv_obj_add_flag(plates_panel, LV_OBJ_FLAG_HIDDEN);
    }
    // Leave Settings closed after a selection.
}

static void goto_plates(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (!plates_panel) {
        return;
    }
    s_plates_from_settings = true;
    config_load_plate_extras(s_plate_extra, s_plate_extra_city, &s_plate_extra_n);
    if (plates_ta) {
        lv_textarea_set_text(plates_ta, "");
    }
    if (plates_add_ta) {
        lv_textarea_set_text(plates_add_ta, "");
    }
    s_plate_hit_n = 0;
    plates_refresh_add_hits_ui();
    plates_refresh_list();
    lv_obj_clear_flag(plates_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(plates_panel);
    plates_hide_keyboard();
    plates_request_city_refresh();
}

static void plates_remove_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (s_plates_sel < AIRPORT_PLATE_COUNT) {
        if (plates_status) {
            lv_label_set_text(plates_status, "Built-in plates can't be removed");
        }
        return;
    }
    const int eidx = s_plates_sel - AIRPORT_PLATE_COUNT;
    if (eidx < 0 || eidx >= s_plate_extra_n) {
        return;
    }
    char removed[8];
    strncpy(removed, s_plate_extra[eidx], sizeof(removed) - 1);
    removed[sizeof(removed) - 1] = 0;
    for (int i = eidx; i < s_plate_extra_n - 1; i++) {
        memcpy(s_plate_extra[i], s_plate_extra[i + 1], FR_PLATE_EXTRA_ID_LEN);
        memcpy(s_plate_extra_city[i], s_plate_extra_city[i + 1], FR_PLATE_EXTRA_CITY_LEN);
    }
    s_plate_extra_n--;
    if (s_plate_extra_n < 0) {
        s_plate_extra_n = 0;
    }
    if (s_plate_extra_n > 0) {
        s_plate_extra[s_plate_extra_n][0] = 0;
        s_plate_extra_city[s_plate_extra_n][0] = 0;
    }
    config_save_plate_extras(s_plate_extra, s_plate_extra_city, s_plate_extra_n);
    map_plates_mark_dirty();
    s_plates_sel = 0;
    plates_refresh_list();
    if (plates_status) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Removed %s", removed[0] ? removed : "plate");
        lv_label_set_text(plates_status, msg);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void goto_wifi(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    ui_show_wifi();
    // Scan when opened from Settings (not during boot connect — scan disconnects STA).
    if (wifi_status) {
        lv_label_set_text(wifi_status, "Scanning...");
    }
    request(UI_REQ_WIFI_SCAN);
}

static void goto_loc(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    ui_show_location();
}

static void wifi_show_pass_changed(lv_event_t *e) {
    lv_obj_t *cb = (lv_obj_t *)lv_event_get_target(e);
    if (!wifi_ta || !cb) {
        return;
    }
    const bool show = lv_obj_has_state(cb, LV_STATE_CHECKED);
    lv_textarea_set_password_mode(wifi_ta, !show);
    if (!show) {
        lv_textarea_set_password_show_time(wifi_ta, 0);
    }
}

static void wifi_scan_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    lv_label_set_text(wifi_status, "Scanning...");
    request(UI_REQ_WIFI_SCAN);
}

static void wifi_item_clicked(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    const char *ssid = (const char *)lv_event_get_user_data(e);
    if (!ssid) {
        ssid = lv_list_get_btn_text(wifi_list, btn);
    }
    strncpy(g_state->pending_ssid, ssid, sizeof(g_state->pending_ssid) - 1);
    // Keep password masked when picking a new network.
    if (wifi_show_pw_cb) {
        lv_obj_clear_state(wifi_show_pw_cb, LV_STATE_CHECKED);
    }
    if (wifi_ta) {
        lv_textarea_set_password_mode(wifi_ta, true);
        lv_textarea_set_password_show_time(wifi_ta, 0);
        lv_textarea_set_text(wifi_ta, "");
    }
    lv_obj_clear_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifi_kb);
    lv_keyboard_set_textarea(wifi_kb, wifi_ta);
    char msg[80];
    snprintf(msg, sizeof(msg), "Password for %s", ssid);
    lv_label_set_text(wifi_status, msg);
}

static void wifi_connect_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    const char *pw = lv_textarea_get_text(wifi_ta);
    strncpy(g_state->pending_pass, pw ? pw : "", sizeof(g_state->pending_pass) - 1);
    if (g_state->pending_ssid[0] == 0) {
        lv_label_set_text(wifi_status, "Tap a network first");
        return;
    }
    lv_label_set_text(wifi_status, "Connecting...");
    lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    request(UI_REQ_WIFI_CONNECT);
}

static void loc_search_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    const char *q = lv_textarea_get_text(loc_ta);
    strncpy(g_state->pending_query, q ? q : "", sizeof(g_state->pending_query) - 1);
    g_state->pending_query[sizeof(g_state->pending_query) - 1] = 0;
    trim_inplace(g_state->pending_query);
    if (g_state->pending_query[0] == 0) {
        lv_label_set_text(loc_status, "Enter ICAO/IATA (KOCF), zip, or city");
        return;
    }
    lv_label_set_text(loc_status, "Looking up location...");
    lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    request(UI_REQ_GEOCODE);
}

static void loc_pick_clicked(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    if (idx < 0 || idx >= g_state->geo_count) {
        xSemaphoreGive(g_state->mutex);
        return;
    }
    GeoFix pick = g_state->geo_results[idx];
    xSemaphoreGive(g_state->mutex);
    stop_follow(false);
    // Airport picks already have icao+ICAO label from geocode; keep city/ZIP labels as-is.
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.location = pick;
    xSemaphoreGive(g_state->mutex);
    s_home_loc = pick;
    s_home_ok = pick.valid;
    // Must recenter PPI — cfg.location alone does not move s_olat/s_olon.
    s_olat = pick.lat;
    s_olon = pick.lon;
    map_cache_mark_dirty();
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);
    recalc_tz_offset();
    hide_location(nullptr);
    ui_show_radar();
    if (pick.valid) {
        update_loc_label(pick.icao[0] ? pick.icao : pick.label, pick.lat, pick.lon);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    request(UI_REQ_SAVE_LOCATION | UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
}

static int startup_zoom_index(float nm) {
    int best = 2;  // 40 nm
    float best_e = 1e9f;
    for (int i = 0; i < kZoomNearN; i++) {
        const float e = fabsf(kZoomNear[i] - nm);
        if (e < best_e) {
            best_e = e;
            best = i;
        }
    }
    return best;
}

static int startup_alt_index(int alt_ft) {
    int idx = (alt_ft - kAltMinFt) / kAltStepFt;
    if (idx < 0) {
        idx = 0;
    }
    const int max_i = (kAltMaxFt - kAltMinFt) / kAltStepFt;
    if (idx > max_i) {
        idx = max_i;
    }
    return idx;
}

static void style_startup_checkbox(lv_obj_t *cb) {
    lv_obj_set_style_text_color(cb, COL_TEXT, 0);
    lv_obj_set_style_text_font(cb, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(cb, COL_GREEN, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(cb, lv_color_hex(0x1A2A1A), LV_PART_INDICATOR | LV_STATE_CHECKED);
}

static void sync_startup_dropdowns() {
    if (!g_state) {
        return;
    }
    // Show saved Startup Settings — not the live zoom/alt (those are temporary).
    float range = 40.0f;
    int alt = 45000;
    bool cty = true;
    bool apt = true;
    bool cities = true;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    range = g_state->cfg.startup_range_nm;
    alt = g_state->cfg.startup_max_alt_ft;
    cty = g_state->cfg.startup_show_counties;
    apt = g_state->cfg.startup_show_airports;
    cities = g_state->cfg.startup_show_cities;
    xSemaphoreGive(g_state->mutex);
    if (loc_zoom_dd) {
        lv_dropdown_set_selected(loc_zoom_dd, (uint16_t)startup_zoom_index(range));
    }
    if (loc_alt_dd) {
        lv_dropdown_set_selected(loc_alt_dd, (uint16_t)startup_alt_index(alt));
    }
    if (loc_cty_cb) {
        if (cty) {
            lv_obj_add_state(loc_cty_cb, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(loc_cty_cb, LV_STATE_CHECKED);
        }
    }
    if (loc_apt_cb) {
        if (apt) {
            lv_obj_add_state(loc_apt_cb, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(loc_apt_cb, LV_STATE_CHECKED);
        }
    }
    if (loc_city_cb) {
        if (cities) {
            lv_obj_add_state(loc_city_cb, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(loc_city_cb, LV_STATE_CHECKED);
        }
    }
}

static void apply_startup_dropdowns(bool persist) {
    if (!g_state) {
        return;
    }
    float nm = 40.0f;
    int alt = 45000;
    bool cty = true;
    bool apt = true;
    bool cities = true;
    if (loc_zoom_dd) {
        uint16_t zi = lv_dropdown_get_selected(loc_zoom_dd);
        if (zi >= (uint16_t)kZoomNearN) {
            zi = 2;
        }
        nm = kZoomNear[zi];
    }
    if (loc_alt_dd) {
        uint16_t ai = lv_dropdown_get_selected(loc_alt_dd);
        alt = kAltMinFt + (int)ai * kAltStepFt;
        if (alt > kAltMaxFt) {
            alt = kAltMaxFt;
        }
    }
    if (loc_cty_cb) {
        cty = lv_obj_has_state(loc_cty_cb, LV_STATE_CHECKED);
    }
    if (loc_apt_cb) {
        apt = lv_obj_has_state(loc_apt_cb, LV_STATE_CHECKED);
    }
    if (loc_city_cb) {
        cities = lv_obj_has_state(loc_city_cb, LV_STATE_CHECKED);
    }

    set_range(nm);
    set_max_alt(alt);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const bool cty_changed = (g_state->cfg.show_counties != cty);
    const bool apt_changed = (g_state->cfg.show_airports != apt);
    const bool city_changed = (g_state->cfg.show_cities != cities);
    g_state->cfg.show_counties = cty;
    g_state->cfg.show_airports = apt;
    g_state->cfg.show_cities = cities;
    g_state->cfg.startup_range_nm = nm;
    g_state->cfg.startup_max_alt_ft = alt;
    g_state->cfg.startup_show_counties = cty;
    g_state->cfg.startup_show_airports = apt;
    g_state->cfg.startup_show_cities = cities;
    xSemaphoreGive(g_state->mutex);
    s_show_counties = cty;
    s_show_airports = apt;
    s_show_cities = cities;
    update_cty_toggle_label();
    update_apt_toggle_label();
    update_city_toggle_label();
    if (cty_changed) {
        map_cache_mark_dirty();
    }
    if (cty_changed || apt_changed || city_changed) {
        if (radar_plot) {
            lv_obj_invalidate(radar_plot);
        }
    }
    if (persist) {
        uint32_t req = UI_REQ_SAVE_RANGE | UI_REQ_SAVE_MAX_ALT | UI_REQ_SAVE_SHOW_COUNTIES |
                       UI_REQ_SAVE_SHOW_AIRPORTS | UI_REQ_SAVE_SHOW_CITIES;
        request(req);
    }
    if (loc_status) {
        char msg[112];
        snprintf(msg, sizeof(msg), "Boot: %.0f nm, FL%03d, counties %s, airports %s, cities %s", nm,
                 alt / 100, cty ? "on" : "off", apt ? "on" : "off", cities ? "on" : "off");
        lv_label_set_text(loc_status, msg);
    }
}

static void startup_dd_changed(lv_event_t *e) {
    LV_UNUSED(e);
    apply_startup_dropdowns(true);
}

static void build_location() {
    loc_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(loc_panel, 720, 340);
    lv_obj_center(loc_panel);
    lv_obj_set_style_bg_color(loc_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(loc_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(loc_panel, 12, 0);
    lv_obj_set_style_pad_all(loc_panel, 8, 0);
    lv_obj_clear_flag(loc_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(loc_panel);
    lv_label_set_text(title, "Startup Settings");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, COL_GREEN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *lclose = make_btn(loc_panel, "Close", hide_location, nullptr);
    lv_obj_set_size(lclose, 90, 36);
    lv_obj_align(lclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *hint = lv_label_create(loc_panel);
    lv_label_set_text(hint, "Home: ICAO/IATA (KOCF/OCF), zip, or city. Airport homes show code only.");
    lv_obj_set_style_text_color(hint, COL_TEXT, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 8, 40);

    loc_ta = lv_textarea_create(loc_panel);
    lv_obj_set_size(loc_ta, 500, 36);
    lv_obj_align(loc_ta, LV_ALIGN_TOP_LEFT, 8, 62);
    lv_textarea_set_placeholder_text(loc_ta, "e.g. KOCF  or  37203  or  Nashville, TN");
    lv_textarea_set_one_line(loc_ta, true);
    lv_obj_add_event_cb(loc_ta, loc_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(loc_ta, loc_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *search = make_btn(loc_panel, "Search", loc_search_clicked, nullptr);
    lv_obj_set_size(search, 140, 36);
    lv_obj_align(search, LV_ALIGN_TOP_RIGHT, -8, 62);

    loc_list = lv_list_create(loc_panel);
    lv_obj_set_size(loc_list, 688, 96);
    lv_obj_align(loc_list, LV_ALIGN_TOP_MID, 0, 104);
    lv_obj_set_style_bg_color(loc_list, COL_PANEL, 0);
    lv_obj_set_style_border_color(loc_list, COL_DIM, 0);

    loc_status = lv_label_create(loc_panel);
    lv_label_set_text(loc_status, "");
    lv_obj_set_style_text_color(loc_status, COL_GREEN, 0);
    lv_obj_set_style_text_font(loc_status, &lv_font_montserrat_12, 0);
    lv_obj_align(loc_status, LV_ALIGN_TOP_LEFT, 8, 206);

    // Boot defaults — dropdowns (applied on Close and on change; saved to NVS).
    lv_obj_t *zl = lv_label_create(loc_panel);
    lv_label_set_text(zl, "Zoom");
    lv_obj_set_style_text_color(zl, COL_TEXT, 0);
    lv_obj_set_style_text_font(zl, &lv_font_montserrat_12, 0);
    lv_obj_align(zl, LV_ALIGN_TOP_LEFT, 8, 232);

    loc_zoom_dd = lv_dropdown_create(loc_panel);
    lv_dropdown_set_options(loc_zoom_dd, "15 nm\n25 nm\n40 nm\n60 nm\n100 nm\n160 nm\n250 nm");
    lv_obj_set_width(loc_zoom_dd, 130);
    lv_obj_align(loc_zoom_dd, LV_ALIGN_TOP_LEFT, 8, 252);
    lv_obj_add_event_cb(loc_zoom_dd, startup_dd_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *al = lv_label_create(loc_panel);
    lv_label_set_text(al, "Altitude");
    lv_obj_set_style_text_color(al, COL_TEXT, 0);
    lv_obj_set_style_text_font(al, &lv_font_montserrat_12, 0);
    lv_obj_align(al, LV_ALIGN_TOP_LEFT, 160, 232);

    loc_alt_dd = lv_dropdown_create(loc_panel);
    {
        char opts[160];
        opts[0] = 0;
        for (int a = kAltMinFt; a <= kAltMaxFt; a += kAltStepFt) {
            char one[16];
            snprintf(one, sizeof(one), opts[0] ? "\nFL%03d" : "FL%03d", a / 100);
            strncat(opts, one, sizeof(opts) - strlen(opts) - 1);
        }
        lv_dropdown_set_options(loc_alt_dd, opts);
    }
    lv_obj_set_width(loc_alt_dd, 120);
    lv_obj_align(loc_alt_dd, LV_ALIGN_TOP_LEFT, 160, 252);
    lv_obj_add_event_cb(loc_alt_dd, startup_dd_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    loc_cty_cb = lv_checkbox_create(loc_panel);
    lv_checkbox_set_text(loc_cty_cb, "Show counties");
    style_startup_checkbox(loc_cty_cb);
    lv_obj_align(loc_cty_cb, LV_ALIGN_TOP_LEFT, 300, 236);
    lv_obj_add_event_cb(loc_cty_cb, startup_dd_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    loc_apt_cb = lv_checkbox_create(loc_panel);
    lv_checkbox_set_text(loc_apt_cb, "Show airports");
    style_startup_checkbox(loc_apt_cb);
    lv_obj_align(loc_apt_cb, LV_ALIGN_TOP_LEFT, 300, 272);
    lv_obj_add_event_cb(loc_apt_cb, startup_dd_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    loc_city_cb = lv_checkbox_create(loc_panel);
    lv_checkbox_set_text(loc_city_cb, "Show cities");
    style_startup_checkbox(loc_city_cb);
    lv_obj_align(loc_city_cb, LV_ALIGN_TOP_LEFT, 480, 236);
    lv_obj_add_event_cb(loc_city_cb, startup_dd_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    // Screen-level keyboard (same pattern as Wi-Fi) so it is not clipped by the panel.
    loc_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(loc_kb, 800, 190);
    lv_obj_align(loc_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(loc_kb, loc_ta);
    lv_obj_add_event_cb(loc_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
}

static void show_track_keyboard() {
    if (!track_kb || !track_ta) {
        return;
    }
    lv_keyboard_set_textarea(track_kb, track_ta);
    lv_obj_clear_flag(track_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(track_panel);
    lv_obj_move_foreground(track_kb);
}

static void hide_track(lv_event_t *e) {
    LV_UNUSED(e);
    if (track_kb) {
        lv_obj_add_flag(track_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (track_panel) {
        lv_obj_add_flag(track_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void track_ta_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    show_track_keyboard();
}

static void refresh_track_history_ui();
static void track_hist_clicked(lv_event_t *e);

static void normalize_track_id(const char *in, char *out, size_t out_sz) {
    if (!out || out_sz == 0) {
        return;
    }
    out[0] = 0;
    if (!in) {
        return;
    }
    size_t j = 0;
    for (const char *p = in; *p && j + 1 < out_sz; p++) {
        char c = *p;
        if (c == ' ' || c == '\t') {
            continue;
        }
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }
        out[j++] = c;
    }
    out[j] = 0;
}

static void track_history_push(const char *raw) {
    char id[FR_TRACK_HIST_ID_LEN];
    normalize_track_id(raw, id, sizeof(id));
    if (!id[0]) {
        return;
    }
    int found = -1;
    for (int i = 0; i < s_track_hist_n; i++) {
        if (strcmp(s_track_hist[i], id) == 0) {
            found = i;
            break;
        }
    }
    if (found >= 0) {
        for (int i = found; i > 0; i--) {
            strncpy(s_track_hist[i], s_track_hist[i - 1], FR_TRACK_HIST_ID_LEN - 1);
            s_track_hist[i][FR_TRACK_HIST_ID_LEN - 1] = 0;
        }
    } else {
        if (s_track_hist_n < FR_TRACK_HIST_MAX) {
            s_track_hist_n++;
        }
        for (int i = s_track_hist_n - 1; i > 0; i--) {
            strncpy(s_track_hist[i], s_track_hist[i - 1], FR_TRACK_HIST_ID_LEN - 1);
            s_track_hist[i][FR_TRACK_HIST_ID_LEN - 1] = 0;
        }
    }
    strncpy(s_track_hist[0], id, FR_TRACK_HIST_ID_LEN - 1);
    s_track_hist[0][FR_TRACK_HIST_ID_LEN - 1] = 0;
    config_save_track_history(s_track_hist, s_track_hist_n);
    refresh_track_history_ui();
}

static void refresh_track_history_ui() {
    if (!track_hist_cont) {
        return;
    }
    lv_obj_clean(track_hist_cont);
    for (int i = 0; i < s_track_hist_n; i++) {
        if (!s_track_hist[i][0]) {
            continue;
        }
        lv_obj_t *btn = make_btn(track_hist_cont, s_track_hist[i], track_hist_clicked, (void *)s_track_hist[i]);
        lv_obj_set_size(btn, LV_SIZE_CONTENT, 32);
        lv_obj_set_style_pad_left(btn, 10, 0);
        lv_obj_set_style_pad_right(btn, 10, 0);
        lv_obj_set_style_pad_top(btn, 4, 0);
        lv_obj_set_style_pad_bottom(btn, 4, 0);
        lv_obj_set_style_radius(btn, 6, 0);
    }
}

static void track_start_query(const char *q, bool show_kb) {
    if (!g_state) {
        return;
    }
    char id[FR_TRACK_HIST_ID_LEN];
    normalize_track_id(q, id, sizeof(id));
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->pending_query, id[0] ? id : (q ? q : ""), sizeof(g_state->pending_query) - 1);
    g_state->pending_query[sizeof(g_state->pending_query) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    if (!g_state->pending_query[0]) {
        if (track_status) {
            lv_label_set_text(track_status, "Enter a flight number (e.g. UA100)");
        }
        if (show_kb) {
            show_track_keyboard();
        }
        return;
    }
    if (track_ta) {
        lv_textarea_set_text(track_ta, g_state->pending_query);
    }
    if (track_status) {
        lv_label_set_text(track_status, "Looking up flight...");
    }
    if (show_kb) {
        show_track_keyboard();
    }
    request(UI_REQ_TRACK_FLIGHT);
}

static void track_go_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!track_ta) {
        return;
    }
    track_start_query(lv_textarea_get_text(track_ta), true);
}

static void track_hist_clicked(lv_event_t *e) {
    const char *id = (const char *)lv_event_get_user_data(e);
    if (!id || !id[0]) {
        return;
    }
    track_start_query(id, true);
}

static void track_kb_event(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        track_go_clicked(e);
    } else if (code == LV_EVENT_CANCEL) {
        hide_track(e);
    }
}

static void open_track_flight(lv_event_t *e) {
    LV_UNUSED(e);
    if (!track_panel) {
        return;
    }
    config_load_track_history(s_track_hist, &s_track_hist_n);
    refresh_track_history_ui();
    if (track_status) {
        lv_label_set_text(track_status, "");
    }
    if (track_ta) {
        lv_textarea_set_text(track_ta, "");
        lv_obj_add_state(track_ta, LV_STATE_FOCUSED);
    }
    lv_obj_clear_flag(track_panel, LV_OBJ_FLAG_HIDDEN);
    show_track_keyboard();
}

static void build_track_flight() {
    // Letters + number pad (phone-style digits on the right).
    static const char *kTrackKbMap[] = {
        "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "7", "8", "9", "\n",
        "A", "S", "D", "F", "G", "H", "J", "K", "L", LV_SYMBOL_BACKSPACE, "4", "5", "6", "\n",
        "Z", "X", "C", "V", "B", "N", "M", "0", "1", "2", "3", "\n",
        LV_SYMBOL_CLOSE, " ", LV_SYMBOL_OK, "",
    };
    static const lv_btnmatrix_ctrl_t kTrackKbCtrl[] = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 5, 4, 4, 4,
        3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
        LV_KEYBOARD_CTRL_BTN_FLAGS | 5, 12, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    };

    config_load_track_history(s_track_hist, &s_track_hist_n);

    track_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(track_panel, 560, 300);
    lv_obj_align(track_panel, LV_ALIGN_TOP_MID, 0, 16);  // sit above the keyboard
    lv_obj_set_style_bg_color(track_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(track_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(track_panel, 12, 0);
    lv_obj_set_style_pad_all(track_panel, 8, 0);
    lv_obj_clear_flag(track_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(track_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(track_panel);
    lv_label_set_text(title, "Track Flight");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, COL_GREEN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *tclose = make_btn(track_panel, "Close", hide_track, nullptr);
    lv_obj_set_size(tclose, 90, 36);
    lv_obj_align(tclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *hint = lv_label_create(track_panel);
    lv_label_set_text(hint, "Enter a flight number (UA100 or UAL100). Centers on live position.");
    lv_obj_set_style_text_color(hint, COL_TEXT, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_width(hint, 520);
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 8, 42);

    track_ta = lv_textarea_create(track_panel);
    lv_obj_set_size(track_ta, 340, 40);
    lv_obj_align(track_ta, LV_ALIGN_TOP_LEFT, 8, 78);
    lv_textarea_set_placeholder_text(track_ta, "e.g. UA100");
    lv_textarea_set_one_line(track_ta, true);
    lv_textarea_set_max_length(track_ta, 10);
    lv_textarea_set_text(track_ta, "");
    lv_obj_add_event_cb(track_ta, track_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(track_ta, track_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *go = make_btn(track_panel, "Track", track_go_clicked, nullptr);
    lv_obj_set_size(go, 160, 40);
    lv_obj_align(go, LV_ALIGN_TOP_RIGHT, -8, 78);

    track_status = lv_label_create(track_panel);
    lv_label_set_text(track_status, "");
    lv_obj_set_style_text_color(track_status, COL_GREEN, 0);
    lv_obj_set_style_text_font(track_status, &lv_font_montserrat_14, 0);
    lv_obj_set_width(track_status, 520);
    lv_label_set_long_mode(track_status, LV_LABEL_LONG_WRAP);
    lv_obj_align(track_status, LV_ALIGN_TOP_LEFT, 8, 126);

    lv_obj_t *recent_lbl = lv_label_create(track_panel);
    lv_label_set_text(recent_lbl, "Recent");
    lv_obj_set_style_text_color(recent_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(recent_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(recent_lbl, LV_ALIGN_TOP_LEFT, 8, 152);

    track_hist_cont = lv_obj_create(track_panel);
    lv_obj_set_size(track_hist_cont, 536, 110);
    lv_obj_align(track_hist_cont, LV_ALIGN_TOP_LEFT, 4, 174);
    lv_obj_set_style_bg_opa(track_hist_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(track_hist_cont, 0, 0);
    lv_obj_set_style_pad_all(track_hist_cont, 2, 0);
    lv_obj_set_style_pad_row(track_hist_cont, 6, 0);
    lv_obj_set_style_pad_column(track_hist_cont, 6, 0);
    lv_obj_set_flex_flow(track_hist_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(track_hist_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(track_hist_cont, LV_OBJ_FLAG_SCROLLABLE);

    refresh_track_history_ui();

    track_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(track_kb, 800, 240);
    lv_obj_align(track_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_map(track_kb, LV_KEYBOARD_MODE_USER_1, kTrackKbMap, kTrackKbCtrl);
    lv_keyboard_set_mode(track_kb, LV_KEYBOARD_MODE_USER_1);
    lv_keyboard_set_textarea(track_kb, track_ta);
    lv_obj_add_event_cb(track_kb, track_kb_event, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(track_kb, track_kb_event, LV_EVENT_CANCEL, nullptr);
    lv_obj_add_event_cb(track_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(track_kb, LV_OBJ_FLAG_HIDDEN);
}

static void format_air_bar(char *out, size_t n, uint8_t pct) {
    if (!out || n < 12) {
        if (out && n) {
            out[0] = 0;
        }
        return;
    }
    const int w = 8;
    int filled = ((int)pct * w + 50) / 100;
    if (filled < 0) {
        filled = 0;
    }
    if (filled > w) {
        filled = w;
    }
    out[0] = '[';
    for (int i = 0; i < w; i++) {
        out[1 + i] = (i < filled) ? '#' : '-';
    }
    out[1 + w] = ']';
    out[2 + w] = 0;
}

static void board_hide_keyboard() {
    if (board_kb) {
        lv_obj_add_flag(board_kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void board_show_keyboard() {
    if (!board_kb || !board_ta) {
        return;
    }
    lv_keyboard_set_textarea(board_kb, board_ta);
    lv_obj_clear_flag(board_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(board_panel);
    lv_obj_move_foreground(board_kb);
}

static void hide_airport_board(lv_event_t *e) {
    LV_UNUSED(e);
    board_hide_keyboard();
    if (board_panel) {
        lv_obj_add_flag(board_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static int board_sort_cmp(const AirportBoardRow *a, const AirportBoardRow *b, int col) {
    if (!a || !b) {
        return 0;
    }
    int cmp = 0;
    switch (col) {
        case 0:  // FLIGHT
            cmp = strcasecmp(a->flight[0] ? a->flight : "", b->flight[0] ? b->flight : "");
            break;
        case 1:  // EQP
            cmp = strcasecmp(a->type[0] ? a->type : "", b->type[0] ? b->type : "");
            break;
        case 2: {  // FROM / TO
            const char *ao = a->other_apt[0] ? a->other_apt : (a->other_name[0] ? a->other_name : "");
            const char *bo = b->other_apt[0] ? b->other_apt : (b->other_name[0] ? b->other_name : "");
            cmp = strcasecmp(ao, bo);
            break;
        }
        case 3:  // DEP
            if (a->dep_epoch && b->dep_epoch) {
                cmp = (a->dep_epoch < b->dep_epoch) ? -1 : (a->dep_epoch > b->dep_epoch) ? 1 : 0;
            } else {
                cmp = strcmp(a->dep_time[0] ? a->dep_time : "", b->dep_time[0] ? b->dep_time : "");
            }
            break;
        case 4:  // ARR
            if (a->arr_epoch && b->arr_epoch) {
                cmp = (a->arr_epoch < b->arr_epoch) ? -1 : (a->arr_epoch > b->arr_epoch) ? 1 : 0;
            } else {
                cmp = strcmp(a->arr_time[0] ? a->arr_time : "", b->arr_time[0] ? b->arr_time : "");
            }
            break;
        case 5:  // AIR progress
            cmp = (a->progress_pct < b->progress_pct) ? -1 : (a->progress_pct > b->progress_pct) ? 1 : 0;
            break;
        default:
            break;
    }
    if (cmp == 0) {
        cmp = strcasecmp(a->flight[0] ? a->flight : "", b->flight[0] ? b->flight : "");
    }
    return cmp;
}

static void board_sort_rows(AirportBoardRow *rows, int n, int col, bool asc) {
    if (!rows || n < 2) {
        return;
    }
    for (int a = 0; a < n - 1; a++) {
        for (int b = a + 1; b < n; b++) {
            int cmp = board_sort_cmp(&rows[a], &rows[b], col);
            if (!asc) {
                cmp = -cmp;
            }
            if (cmp > 0) {
                AirportBoardRow tmp = rows[a];
                rows[a] = rows[b];
                rows[b] = tmp;
            }
        }
    }
}

static void board_apply_col_widths(lv_obj_t *table) {
    if (!table) {
        return;
    }
    lv_table_set_col_cnt(table, 6);
    for (int c = 0; c < 6; c++) {
        lv_table_set_col_width(table, c, kBoardColW[c]);
    }
}

static void board_set_headers(lv_obj_t *hdr, bool arrivals, int sort_col, bool sort_asc) {
    if (!hdr) {
        return;
    }
    board_apply_col_widths(hdr);
    lv_table_set_row_cnt(hdr, 1);
    const char *arrow = sort_asc ? LV_SYMBOL_UP : LV_SYMBOL_DOWN;
    char h[20];
    auto set_hdr = [&](uint16_t col, const char *base) {
        if ((int)col == sort_col) {
            snprintf(h, sizeof(h), "%s%s", base, arrow);
            lv_table_set_cell_value(hdr, 0, col, h);
        } else {
            lv_table_set_cell_value(hdr, 0, col, base);
        }
    };
    set_hdr(0, "FLIGHT");
    set_hdr(1, "EQP");
    set_hdr(2, arrivals ? "FROM" : "TO");
    set_hdr(3, "DEP");
    set_hdr(4, "ARR");
    set_hdr(5, "AIR");
}

static void board_fill_table(lv_obj_t *table, AirportBoardRow *rows, int n, bool arrivals) {
    if (!table) {
        return;
    }
    const int sort_col = arrivals ? s_board_arr_sort_col : s_board_dep_sort_col;
    const bool sort_asc = arrivals ? s_board_arr_sort_asc : s_board_dep_sort_asc;
    if (rows && n > 0) {
        board_sort_rows(rows, n, sort_col, sort_asc);
    }
    board_apply_col_widths(table);
    // LVGL: never set row_cnt to 0 (realloc(0) / empty-cell free path panics).
    if (n <= 0) {
        lv_table_set_row_cnt(table, 1);
        for (int c = 0; c < 6; c++) {
            lv_table_set_cell_value(table, 0, c, "");
        }
        return;
    }
    lv_table_set_row_cnt(table, (uint16_t)n);
    for (int i = 0; i < n; i++) {
        const AirportBoardRow &r = rows[i];
        // FROM/TO: city name; fall back to airport code.
        char dest[48];
        if (r.other_name[0]) {
            snprintf(dest, sizeof(dest), "%s", r.other_name);
        } else if (r.other_apt[0]) {
            snprintf(dest, sizeof(dest), "%s", r.other_apt);
        } else {
            dest[0] = '-';
            dest[1] = 0;
        }
        // DEP/ARR: prefer estimated/actual over scheduled (one time per column).
        const char *dep = r.dep_est[0] ? r.dep_est : (r.dep_time[0] ? r.dep_time : "--");
        const char *arr = r.arr_est[0] ? r.arr_est : (r.arr_time[0] ? r.arr_time : "--");
        char bar[16];
        format_air_bar(bar, sizeof(bar), r.progress_pct);
        lv_table_set_cell_value(table, i, 0, r.flight[0] ? r.flight : "--");
        lv_table_set_cell_value(table, i, 1, r.type[0] ? r.type : "--");
        lv_table_set_cell_value(table, i, 2, dest);
        lv_table_set_cell_value(table, i, 3, dep);
        lv_table_set_cell_value(table, i, 4, arr);
        lv_table_set_cell_value(table, i, 5, bar);
    }
}

static void refresh_airport_board_tables() {
    board_set_headers(board_arr_hdr, true, s_board_arr_sort_col, s_board_arr_sort_asc);
    board_fill_table(board_arr_table, s_board_arr, s_board_arr_n, true);
    board_set_headers(board_dep_hdr, false, s_board_dep_sort_col, s_board_dep_sort_asc);
    board_fill_table(board_dep_table, s_board_dep, s_board_dep_n, false);
}

static void board_load_icao(const char *icao);
static void refresh_board_history_ui();
static void board_hist_clicked(lv_event_t *e);

static void board_history_push(const char *icao_raw) {
    char id[FR_BOARD_HIST_ID_LEN];
    normalize_track_id(icao_raw, id, sizeof(id));
    if (!id[0]) {
        return;
    }
    int found = -1;
    for (int i = 0; i < s_board_hist_n; i++) {
        if (strcmp(s_board_hist[i], id) == 0) {
            found = i;
            break;
        }
    }
    if (found >= 0) {
        for (int i = found; i > 0; i--) {
            strncpy(s_board_hist[i], s_board_hist[i - 1], FR_BOARD_HIST_ID_LEN - 1);
            s_board_hist[i][FR_BOARD_HIST_ID_LEN - 1] = 0;
        }
    } else {
        if (s_board_hist_n < FR_BOARD_HIST_MAX) {
            s_board_hist_n++;
        }
        for (int i = s_board_hist_n - 1; i > 0; i--) {
            strncpy(s_board_hist[i], s_board_hist[i - 1], FR_BOARD_HIST_ID_LEN - 1);
            s_board_hist[i][FR_BOARD_HIST_ID_LEN - 1] = 0;
        }
    }
    strncpy(s_board_hist[0], id, FR_BOARD_HIST_ID_LEN - 1);
    s_board_hist[0][FR_BOARD_HIST_ID_LEN - 1] = 0;
    config_save_board_history(s_board_hist, s_board_hist_n);
    refresh_board_history_ui();
}

static void refresh_board_history_ui() {
    if (!board_hist_cont) {
        return;
    }
    lv_obj_clean(board_hist_cont);
    for (int i = 0; i < s_board_hist_n; i++) {
        if (!s_board_hist[i][0]) {
            continue;
        }
        lv_obj_t *btn = make_btn(board_hist_cont, s_board_hist[i], board_hist_clicked, (void *)s_board_hist[i]);
        lv_obj_set_size(btn, LV_SIZE_CONTENT, 30);
        lv_obj_set_style_pad_left(btn, 8, 0);
        lv_obj_set_style_pad_right(btn, 8, 0);
        lv_obj_set_style_pad_top(btn, 4, 0);
        lv_obj_set_style_pad_bottom(btn, 4, 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_t *lab = lv_obj_get_child(btn, 0);
        if (lab) {
            lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
        }
    }
}

static void board_hist_clicked(lv_event_t *e) {
    const char *icao = (const char *)lv_event_get_user_data(e);
    if (!icao || !icao[0]) {
        return;
    }
    board_hide_keyboard();
    if (board_ta) {
        lv_textarea_set_text(board_ta, icao);
    }
    board_load_icao(icao);
}

static void board_hit_clicked(lv_event_t *e) {
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_apt_hit_n) {
        return;
    }
    board_hide_keyboard();
    board_load_icao(s_apt_hits[idx].icao);
}

static void refresh_airport_hits_ui() {
    if (!board_hits) {
        return;
    }
    lv_obj_clean(board_hits);
    if (s_apt_hit_n <= 0) {
        lv_obj_add_flag(board_hits, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(board_hits, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < s_apt_hit_n; i++) {
        if (!s_apt_hits[i].valid || !s_apt_hits[i].icao[0]) {
            continue;
        }
        lv_obj_t *btn =
            make_btn(board_hits, s_apt_hits[i].label[0] ? s_apt_hits[i].label : s_apt_hits[i].icao,
                     board_hit_clicked, (void *)(intptr_t)i);
        lv_obj_set_size(btn, LV_SIZE_CONTENT, 30);
        lv_obj_set_style_pad_left(btn, 8, 0);
        lv_obj_set_style_pad_right(btn, 8, 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_t *lab = lv_obj_get_child(btn, 0);
        if (lab) {
            lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
        }
    }
}

static void board_load_icao(const char *icao) {
    if (!g_state || !icao || !icao[0]) {
        return;
    }
    board_history_push(icao);
    board_set_airport_ll(icao);
    // Kick NTP if clock still cold — board clock needs a valid epoch.
    if (WiFi.status() == WL_CONNECTED && time(nullptr) < 1700000000) {
        configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->pending_board_icao, icao, sizeof(g_state->pending_board_icao) - 1);
    g_state->pending_board_icao[sizeof(g_state->pending_board_icao) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    if (board_status) {
        lv_label_set_text(board_status, "Loading FlightAware board...");
    }
    if (board_title_lbl) {
        char t[64];
        snprintf(t, sizeof(t), "Airport Board - %s", icao);
        lv_label_set_text(board_title_lbl, t);
    }
    request(UI_REQ_FETCH_AIRPORT_BOARD);
}

static void board_search_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state || !board_ta) {
        return;
    }
    const char *q = lv_textarea_get_text(board_ta);
    if (!q || !q[0]) {
        if (board_status) {
            lv_label_set_text(board_status, "Enter ICAO (KATL) or city name");
        }
        board_show_keyboard();
        return;
    }
    // Fast path: 3–4 letter code → load board directly.
    char code[8];
    size_t n = 0;
    for (const char *p = q; *p && n + 1 < sizeof(code); p++) {
        if (*p == ' ' || *p == '\t') {
            continue;
        }
        code[n++] = (char)toupper((unsigned char)*p);
    }
    code[n] = 0;
    bool code_like = (n == 3 || n == 4);
    for (size_t i = 0; code_like && i < n; i++) {
        if (!isalnum((unsigned char)code[i])) {
            code_like = false;
        }
    }
    if (code_like) {
        board_hide_keyboard();
        s_apt_hit_n = 0;
        refresh_airport_hits_ui();
        board_load_icao(code);
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->pending_query, q, sizeof(g_state->pending_query) - 1);
    g_state->pending_query[sizeof(g_state->pending_query) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    if (board_status) {
        lv_label_set_text(board_status, "Searching airports...");
    }
    request(UI_REQ_SEARCH_AIRPORTS);
}

static void board_ta_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    board_show_keyboard();
}

static void board_kb_event(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        board_search_clicked(e);
    } else if (code == LV_EVENT_CANCEL) {
        hide_airport_board(e);
    }
}

static void board_clear_selection() {
    lv_obj_t *prev = s_board_sel_table;
    s_board_sel_table = nullptr;
    s_board_sel_row = -1;
    if (prev) {
        lv_obj_invalidate(prev);
    }
}

static void board_hdr_draw_part(lv_event_t *e) {
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (!dsc || dsc->part != LV_PART_ITEMS || dsc->type != LV_TABLE_DRAW_PART_CELL) {
        return;
    }
    if (dsc->rect_dsc) {
        dsc->rect_dsc->bg_color = lv_color_hex(0x1A2A1C);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
    }
    if (dsc->label_dsc) {
        dsc->label_dsc->color = COL_GREEN;
        dsc->label_dsc->font = &lv_font_montserrat_16;
    }
}

static void board_table_draw_part(lv_event_t *e) {
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (!dsc || dsc->part != LV_PART_ITEMS || dsc->type != LV_TABLE_DRAW_PART_CELL) {
        return;
    }
    if (table != s_board_sel_table || s_board_sel_row < 0) {
        return;
    }
    const uint16_t col_cnt = lv_table_get_col_cnt(table);
    if (col_cnt == 0) {
        return;
    }
    const uint16_t row = (uint16_t)(dsc->id / col_cnt);
    if ((int)row != s_board_sel_row) {
        return;
    }
    if (dsc->rect_dsc) {
        dsc->rect_dsc->bg_color = lv_color_hex(0xB0B0B0);  // light grey highlight
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
    }
    if (dsc->label_dsc) {
        dsc->label_dsc->color = lv_color_hex(0x101010);
    }
}

static void board_hdr_clicked(lv_event_t *e) {
    lv_obj_t *hdr = (lv_obj_t *)lv_event_get_target(e);
    uint16_t row = 0, col = 0;
    lv_table_get_selected_cell(hdr, &row, &col);
    if (row == LV_TABLE_CELL_NONE || col == LV_TABLE_CELL_NONE || col > 5) {
        return;
    }
    const bool arrivals = (hdr == board_arr_hdr);
    int *sort_col = arrivals ? &s_board_arr_sort_col : &s_board_dep_sort_col;
    bool *sort_asc = arrivals ? &s_board_arr_sort_asc : &s_board_dep_sort_asc;
    if (*sort_col == (int)col) {
        *sort_asc = !*sort_asc;
    } else {
        *sort_col = (int)col;
        *sort_asc = true;
    }
    board_clear_selection();
    board_set_headers(hdr, arrivals, *sort_col, *sort_asc);
    board_fill_table(arrivals ? board_arr_table : board_dep_table,
                     arrivals ? s_board_arr : s_board_dep,
                     arrivals ? s_board_arr_n : s_board_dep_n, arrivals);
}

static void board_table_clicked(lv_event_t *e) {
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    uint16_t row = 0, col = 0;
    lv_table_get_selected_cell(table, &row, &col);
    if (row == LV_TABLE_CELL_NONE || col == LV_TABLE_CELL_NONE) {
        return;
    }
    const bool arrivals = (table == board_arr_table);
    const int idx = (int)row;
    const AirportBoardRow *rows = arrivals ? s_board_arr : s_board_dep;
    const int n = arrivals ? s_board_arr_n : s_board_dep_n;
    if (idx < 0 || idx >= n || !rows[idx].flight[0]) {
        return;
    }
    // Highlight selected row (light grey); clear the other table's selection.
    if (s_board_sel_table && s_board_sel_table != table) {
        lv_obj_invalidate(s_board_sel_table);
    }
    s_board_sel_table = table;
    s_board_sel_row = idx;
    lv_obj_invalidate(table);

    board_hide_keyboard();
    const AirportBoardRow &br = rows[idx];

    // Start follow immediately from the board row. Waiting on ADS-B/FlightStats/hexdb
    // after a big FA board download often hangs on TLS/DNS while the UI sat on
    // "Tracking…" forever.
    Aircraft ac{};
    FlightInfo fi{};
    auto cpy = [](char *dst, size_t n, const char *src) {
        if (!dst || n == 0) {
            return;
        }
        strncpy(dst, src ? src : "", n - 1);
        dst[n - 1] = 0;
    };
    cpy(ac.flight, sizeof(ac.flight), br.flight);
    cpy(ac.type, sizeof(ac.type), br.type);
    ac.valid = true;
    ac.on_ground = false;
    ac.alt_ft = 5000.0f;
    // Pin near the board airport until ADS-B finds the live position.
    if (s_board_ll_ok) {
        ac.lat = s_board_lat;
        ac.lon = s_board_lon;
    } else if (br.other_apt[0]) {
        float lat = 0, lon = 0;
        if (net_lookup_airport(br.other_apt, &lat, &lon, nullptr) ||
            net_lookup_airport_iata(br.other_apt, nullptr, 0, &lat, &lon)) {
            ac.lat = lat;
            ac.lon = lon;
        }
    }
    cpy(fi.flight, sizeof(fi.flight), br.flight);
    if (arrivals) {
        cpy(fi.origin, sizeof(fi.origin), br.other_apt);
        cpy(fi.origin_name, sizeof(fi.origin_name), br.other_name);
        cpy(fi.dest, sizeof(fi.dest), s_board_icao);
    } else {
        cpy(fi.origin, sizeof(fi.origin), s_board_icao);
        cpy(fi.dest, sizeof(fi.dest), br.other_apt);
        cpy(fi.dest_name, sizeof(fi.dest_name), br.other_name);
    }
    cpy(fi.dep_time, sizeof(fi.dep_time), br.dep_time[0] ? br.dep_time : br.dep_est);
    cpy(fi.arr_time, sizeof(fi.arr_time), br.arr_time[0] ? br.arr_time : br.arr_est);
    cpy(fi.dep_est, sizeof(fi.dep_est), br.dep_est);
    cpy(fi.arr_est, sizeof(fi.arr_est), br.arr_est);
    if (br.dep_epoch) {
        fi.dep_act_epoch = br.dep_epoch;
    }
    if (br.arr_epoch) {
        fi.arr_est_epoch = br.arr_epoch;
    }
    fi.departed = true;
    fi.schedule_ok = fi.dep_time[0] || fi.arr_time[0] || fi.dep_est[0] || fi.arr_est[0];
    fi.route_ok = fi.origin[0] && fi.dest[0];

    track_history_push(br.flight);
    hide_airport_board(nullptr);
    hide_track(nullptr);
    hide_route_search(nullptr);
    s_detail_ac = ac;
    s_detail_is_aircraft = true;
    s_flight_info = fi;
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        g_state->tracked_ac = ac;
        g_state->tracked_ok = true;
        g_state->flight_info = fi;
        xSemaphoreGive(g_state->mutex);
    }
    start_follow_aircraft(&ac, false);
    merge_follow_fi(fi);
    refresh_follow_panel();

    // Background enrich: live ADS-B fix + full schedule/route (may be slow after FA).
    track_start_query(br.flight, false);
}

static void open_airport_board(lv_event_t *e) {
    LV_UNUSED(e);
    if (!board_panel) {
        return;
    }
    hide_track(nullptr);
    hide_route_search(nullptr);
    config_load_board_history(s_board_hist, &s_board_hist_n);
    refresh_board_history_ui();
    if (board_title_lbl) {
        lv_label_set_text(board_title_lbl, "Airport Board");
    }
    s_apt_hit_n = 0;
    board_clear_selection();
    refresh_airport_hits_ui();
    // Keep last board tables visible while a refresh runs (avoids empty "stuck" feel).
    const bool have_cache = (s_board_arr_n > 0 || s_board_dep_n > 0);
    if (!have_cache) {
        refresh_airport_board_tables();
    }
    lv_obj_clear_flag(board_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(board_panel);

    // Default to the last selected airport (most recent in history).
    if (s_board_hist_n > 0 && s_board_hist[0][0]) {
        if (board_ta) {
            lv_textarea_set_text(board_ta, s_board_hist[0]);
        }
        board_hide_keyboard();
        if (have_cache && board_status) {
            lv_label_set_text(board_status, "Refreshing FlightAware…");
        }
        board_load_icao(s_board_hist[0]);
    } else {
        if (board_status) {
            lv_label_set_text(board_status, "Search an airport, then select a flight to follow");
        }
        if (board_ta) {
            lv_textarea_set_text(board_ta, "");
            lv_obj_add_state(board_ta, LV_STATE_FOCUSED);
        }
        board_show_keyboard();
    }
}

static void build_airport_board() {
    static const char *kBoardKbMap[] = {
        "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "\n",
        "A", "S", "D", "F", "G", "H", "J", "K", "L", LV_SYMBOL_BACKSPACE, "\n",
        "Z", "X", "C", "V", "B", "N", "M", " ", "\n",
        LV_SYMBOL_CLOSE, LV_SYMBOL_OK, "",
    };
    static const lv_btnmatrix_ctrl_t kBoardKbCtrl[] = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 5,
        3, 3, 3, 3, 3, 3, 3, 8,
        LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    };

    config_load_board_history(s_board_hist, &s_board_hist_n);

    board_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(board_panel, 1000, 560);
    lv_obj_center(board_panel);
    lv_obj_set_style_bg_color(board_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(board_panel, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(board_panel, 2, 0);
    lv_obj_set_style_radius(board_panel, 12, 0);
    lv_obj_set_style_pad_all(board_panel, 8, 0);
    lv_obj_clear_flag(board_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(board_panel, LV_OBJ_FLAG_HIDDEN);

    board_title_lbl = lv_label_create(board_panel);
    lv_label_set_text(board_title_lbl, "Airport Board");
    lv_obj_set_style_text_font(board_title_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(board_title_lbl, COL_GREEN, 0);
    lv_obj_align(board_title_lbl, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *bclose = make_btn(board_panel, "Close", hide_airport_board, nullptr);
    lv_obj_set_size(bclose, 90, 36);
    lv_obj_align(bclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    // Same analog clock as radar, showing selected airport local time.
    board_clock = lv_obj_create(board_panel);
    lv_obj_set_size(board_clock, kClockSize, kClockSize);
    lv_obj_set_style_bg_opa(board_clock, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(board_clock, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(board_clock, 0, 0);
    lv_obj_clear_flag(board_clock, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(board_clock, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(board_clock, board_clock_draw, LV_EVENT_DRAW_MAIN, nullptr);
    lv_obj_align_to(board_clock, bclose, LV_ALIGN_OUT_LEFT_TOP, -10, 0);

    board_clock_lbl = lv_label_create(board_panel);
    lv_label_set_text(board_clock_lbl, "Local Time for Airport");
    lv_obj_set_style_text_font(board_clock_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(board_clock_lbl, COL_TEXT, 0);
    lv_obj_set_style_text_align(board_clock_lbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align_to(board_clock_lbl, board_clock, LV_ALIGN_OUT_LEFT_MID, -8, 0);

    lv_obj_t *hint = lv_label_create(board_panel);
    lv_label_set_text(hint, "Search an airport, then select a flight to follow");
    lv_obj_set_style_text_color(hint, COL_DIM, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 8, 36);

    board_ta = lv_textarea_create(board_panel);
    lv_obj_set_size(board_ta, 420, 40);
    lv_obj_align(board_ta, LV_ALIGN_TOP_LEFT, 8, 58);
    lv_textarea_set_placeholder_text(board_ta, "KATL or Atlanta");
    lv_textarea_set_one_line(board_ta, true);
    lv_textarea_set_max_length(board_ta, 40);
    lv_obj_add_event_cb(board_ta, board_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(board_ta, board_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *search = make_btn(board_panel, "Search", board_search_clicked, nullptr);
    lv_obj_set_size(search, 120, 40);
    lv_obj_align(search, LV_ALIGN_TOP_LEFT, 440, 58);

    board_status = lv_label_create(board_panel);
    lv_label_set_text(board_status, "");
    lv_obj_set_style_text_color(board_status, COL_GREEN, 0);
    lv_obj_set_style_text_font(board_status, &lv_font_montserrat_14, 0);
    lv_obj_set_width(board_status, 400);
    lv_label_set_long_mode(board_status, LV_LABEL_LONG_CLIP);
    lv_obj_align(board_status, LV_ALIGN_TOP_LEFT, 580, 66);

    lv_obj_t *recent_lbl = lv_label_create(board_panel);
    lv_label_set_text(recent_lbl, "Recent");
    lv_obj_set_style_text_color(recent_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(recent_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(recent_lbl, LV_ALIGN_TOP_LEFT, 8, 116);

    board_hist_cont = lv_obj_create(board_panel);
    lv_obj_set_size(board_hist_cont, 900, 36);
    lv_obj_align(board_hist_cont, LV_ALIGN_TOP_LEFT, 72, 112);
    lv_obj_set_style_bg_opa(board_hist_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(board_hist_cont, 0, 0);
    lv_obj_set_style_pad_all(board_hist_cont, 2, 0);
    lv_obj_set_style_pad_column(board_hist_cont, 6, 0);
    lv_obj_set_flex_flow(board_hist_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_clear_flag(board_hist_cont, LV_OBJ_FLAG_SCROLLABLE);

    board_hits = lv_obj_create(board_panel);
    lv_obj_set_size(board_hits, 980, 36);
    lv_obj_align(board_hits, LV_ALIGN_TOP_LEFT, 4, 150);
    lv_obj_set_style_bg_opa(board_hits, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(board_hits, 0, 0);
    lv_obj_set_style_pad_all(board_hits, 2, 0);
    lv_obj_set_style_pad_column(board_hits, 6, 0);
    lv_obj_set_flex_flow(board_hits, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_add_flag(board_hits, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *arr_lbl = lv_label_create(board_panel);
    lv_label_set_text(arr_lbl, "Arrivals");
    lv_obj_set_style_text_color(arr_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(arr_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(arr_lbl, LV_ALIGN_TOP_LEFT, 8, 188);

    lv_obj_t *dep_lbl = lv_label_create(board_panel);
    lv_label_set_text(dep_lbl, "Departures");
    lv_obj_set_style_text_color(dep_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(dep_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(dep_lbl, LV_ALIGN_TOP_LEFT, 508, 188);

    auto style_board_table = [](lv_obj_t *t, bool header) {
        lv_obj_set_style_bg_color(t, lv_color_hex(0x0A120C), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(t, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(t, COL_BTN_BORDER, LV_PART_MAIN);
        lv_obj_set_style_border_width(t, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(t, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(t, 2, LV_PART_MAIN);
        // Tall cells so header row is easy to tap for sorting.
        lv_obj_set_style_pad_top(t, 10, LV_PART_ITEMS);
        lv_obj_set_style_pad_bottom(t, 10, LV_PART_ITEMS);
        lv_obj_set_style_pad_left(t, 4, LV_PART_ITEMS);
        lv_obj_set_style_pad_right(t, 4, LV_PART_ITEMS);
        lv_obj_set_style_text_font(t, header ? &lv_font_montserrat_16 : &lv_font_montserrat_14, LV_PART_ITEMS);
        lv_obj_set_style_text_color(t, header ? COL_GREEN : COL_TEXT, LV_PART_ITEMS);
        lv_obj_set_style_bg_color(t, header ? lv_color_hex(0x1A2A1C) : lv_color_hex(0x0A120C), LV_PART_ITEMS);
        lv_obj_set_style_bg_opa(t, LV_OPA_COVER, LV_PART_ITEMS);
        if (header) {
            lv_obj_clear_flag(t, LV_OBJ_FLAG_SCROLLABLE);
        } else {
            lv_obj_set_scroll_dir(t, LV_DIR_VER);
        }
    };

    // Fixed header + scrollable body so column titles stay visible while scrolling.
    board_arr_hdr = lv_table_create(board_panel);
    lv_obj_set_size(board_arr_hdr, 480, kBoardHdrH);
    lv_obj_align(board_arr_hdr, LV_ALIGN_TOP_LEFT, 4, 214);
    style_board_table(board_arr_hdr, true);
    lv_obj_add_event_cb(board_arr_hdr, board_hdr_clicked, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(board_arr_hdr, board_hdr_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    board_arr_table = lv_table_create(board_panel);
    lv_obj_set_size(board_arr_table, 480, kBoardBodyH);
    lv_obj_align(board_arr_table, LV_ALIGN_TOP_LEFT, 4, 214 + kBoardHdrH);
    style_board_table(board_arr_table, false);
    lv_obj_add_event_cb(board_arr_table, board_table_clicked, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(board_arr_table, board_table_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    board_dep_hdr = lv_table_create(board_panel);
    lv_obj_set_size(board_dep_hdr, 480, kBoardHdrH);
    lv_obj_align(board_dep_hdr, LV_ALIGN_TOP_LEFT, 504, 214);
    style_board_table(board_dep_hdr, true);
    lv_obj_add_event_cb(board_dep_hdr, board_hdr_clicked, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(board_dep_hdr, board_hdr_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    board_dep_table = lv_table_create(board_panel);
    lv_obj_set_size(board_dep_table, 480, kBoardBodyH);
    lv_obj_align(board_dep_table, LV_ALIGN_TOP_LEFT, 504, 214 + kBoardHdrH);
    style_board_table(board_dep_table, false);
    lv_obj_add_event_cb(board_dep_table, board_table_clicked, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(board_dep_table, board_table_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    refresh_board_history_ui();
    refresh_airport_board_tables();

    board_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(board_kb, 900, 220);
    lv_obj_align(board_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_map(board_kb, LV_KEYBOARD_MODE_USER_1, kBoardKbMap, kBoardKbCtrl);
    lv_keyboard_set_mode(board_kb, LV_KEYBOARD_MODE_USER_1);
    lv_keyboard_set_textarea(board_kb, board_ta);
    lv_obj_add_event_cb(board_kb, board_kb_event, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(board_kb, board_kb_event, LV_EVENT_CANCEL, nullptr);
    lv_obj_add_event_cb(board_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(board_kb, LV_OBJ_FLAG_HIDDEN);
}

static void route_hide_keyboard() {
    if (route_kb) {
        lv_obj_add_flag(route_kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void route_show_keyboard(lv_obj_t *ta) {
    if (!route_kb || !ta) {
        return;
    }
    route_kb_target = ta;
    lv_keyboard_set_textarea(route_kb, ta);
    lv_obj_clear_flag(route_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(route_panel);
    lv_obj_move_foreground(route_kb);
}

static void hide_route_search(lv_event_t *e) {
    LV_UNUSED(e);
    route_hide_keyboard();
    if (route_panel) {
        lv_obj_add_flag(route_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void route_try_fetch_flights() {
    if (!g_state || !s_route_dep_icao[0] || !s_route_arr_icao[0]) {
        return;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->route_dep_icao, s_route_dep_icao, sizeof(g_state->route_dep_icao) - 1);
    g_state->route_dep_icao[sizeof(g_state->route_dep_icao) - 1] = 0;
    strncpy(g_state->route_arr_icao, s_route_arr_icao, sizeof(g_state->route_arr_icao) - 1);
    g_state->route_arr_icao[sizeof(g_state->route_arr_icao) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    if (route_status) {
        lv_label_set_text(route_status, "Loading flights...");
    }
    if (route_title_lbl) {
        char t[64];
        snprintf(t, sizeof(t), "%s to %s", s_route_dep_icao, s_route_arr_icao);
        lv_label_set_text(route_title_lbl, t);
    }
    request(UI_REQ_FETCH_ROUTE_FLIGHTS);
}

static void route_fmt_sch_est(char *out, size_t out_len, const char *sch, const char *est) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (sch && sch[0] && est && est[0] && strcasecmp(sch, est) != 0) {
        snprintf(out, out_len, "S %s  E %s", sch, est);
    } else if (sch && sch[0]) {
        snprintf(out, out_len, "%s", sch);
    } else if (est && est[0]) {
        snprintf(out, out_len, "E %s", est);
    } else {
        snprintf(out, out_len, "--");
    }
}

static void route_refresh_table() {
    if (!route_table) {
        return;
    }
    lv_table_set_col_cnt(route_table, 5);
    lv_table_set_row_cnt(route_table, (s_route_row_n > 0 ? s_route_row_n : 0) + 1);
    lv_table_set_col_width(route_table, 0, 90);
    lv_table_set_col_width(route_table, 1, 60);
    lv_table_set_col_width(route_table, 2, 160);
    lv_table_set_col_width(route_table, 3, 160);
    lv_table_set_col_width(route_table, 4, 160);
    lv_table_set_cell_value(route_table, 0, 0, "FLIGHT");
    lv_table_set_cell_value(route_table, 0, 1, "TYPE");
    lv_table_set_cell_value(route_table, 0, 2, "VIA / CITY");
    lv_table_set_cell_value(route_table, 0, 3, "DEP Sch/Est");
    lv_table_set_cell_value(route_table, 0, 4, "ARR Sch/Est");
    for (int i = 0; i < s_route_row_n; i++) {
        const AirportBoardRow &r = s_route_rows[i];
        char via[48];
        if (r.other_apt[0] && r.other_name[0]) {
            snprintf(via, sizeof(via), "%s  %s", r.other_apt, r.other_name);
        } else if (r.other_apt[0]) {
            snprintf(via, sizeof(via), "%s", r.other_apt);
        } else if (r.other_name[0]) {
            snprintf(via, sizeof(via), "%s", r.other_name);
        } else {
            via[0] = '-';
            via[1] = 0;
        }
        char dep_cell[40], arr_cell[40];
        route_fmt_sch_est(dep_cell, sizeof(dep_cell), r.dep_time, r.dep_est);
        route_fmt_sch_est(arr_cell, sizeof(arr_cell), r.arr_time, r.arr_est);
        lv_table_set_cell_value(route_table, i + 1, 0, r.flight[0] ? r.flight : "--");
        lv_table_set_cell_value(route_table, i + 1, 1, r.type[0] ? r.type : "--");
        lv_table_set_cell_value(route_table, i + 1, 2, via);
        lv_table_set_cell_value(route_table, i + 1, 3, dep_cell);
        lv_table_set_cell_value(route_table, i + 1, 4, arr_cell);
    }
}

static void route_dep_hit_clicked(lv_event_t *e) {
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_route_dep_hit_n) {
        return;
    }
    strncpy(s_route_dep_icao, s_route_dep_hits[idx].icao, sizeof(s_route_dep_icao) - 1);
    s_route_dep_icao[sizeof(s_route_dep_icao) - 1] = 0;
    if (route_dep_ta) {
        lv_textarea_set_text(route_dep_ta, s_route_dep_icao);
    }
    s_route_dep_hit_n = 0;
    if (route_dep_hits) {
        lv_obj_clean(route_dep_hits);
        lv_obj_add_flag(route_dep_hits, LV_OBJ_FLAG_HIDDEN);
    }
    route_hide_keyboard();
    route_try_fetch_flights();
}

static void route_arr_hit_clicked(lv_event_t *e) {
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_route_arr_hit_n) {
        return;
    }
    strncpy(s_route_arr_icao, s_route_arr_hits[idx].icao, sizeof(s_route_arr_icao) - 1);
    s_route_arr_icao[sizeof(s_route_arr_icao) - 1] = 0;
    if (route_arr_ta) {
        lv_textarea_set_text(route_arr_ta, s_route_arr_icao);
    }
    s_route_arr_hit_n = 0;
    if (route_arr_hits) {
        lv_obj_clean(route_arr_hits);
        lv_obj_add_flag(route_arr_hits, LV_OBJ_FLAG_HIDDEN);
    }
    route_hide_keyboard();
    route_try_fetch_flights();
}

static void route_refresh_hits_ui() {
    auto fill = [](lv_obj_t *cont, AptSearchHit *hits, int n, void (*cb)(lv_event_t *)) {
        if (!cont) {
            return;
        }
        lv_obj_clean(cont);
        if (n <= 0) {
            lv_obj_add_flag(cont, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        lv_obj_clear_flag(cont, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < n; i++) {
            if (!hits[i].valid || !hits[i].icao[0]) {
                continue;
            }
            lv_obj_t *btn = make_btn(cont, hits[i].label[0] ? hits[i].label : hits[i].icao, cb, (void *)(intptr_t)i);
            lv_obj_set_size(btn, LV_SIZE_CONTENT, 30);
            lv_obj_set_style_pad_left(btn, 8, 0);
            lv_obj_set_style_pad_right(btn, 8, 0);
            lv_obj_set_style_radius(btn, 6, 0);
            lv_obj_t *lab = lv_obj_get_child(btn, 0);
            if (lab) {
                lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
            }
        }
    };
    fill(route_dep_hits, s_route_dep_hits, s_route_dep_hit_n, route_dep_hit_clicked);
    fill(route_arr_hits, s_route_arr_hits, s_route_arr_hit_n, route_arr_hit_clicked);
}

static void route_search_clicked(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state || !route_dep_ta || !route_arr_ta) {
        return;
    }
    const char *dq = lv_textarea_get_text(route_dep_ta);
    const char *aq = lv_textarea_get_text(route_arr_ta);
    if (!dq || !dq[0] || !aq || !aq[0]) {
        if (route_status) {
            lv_label_set_text(route_status, "Enter departure and arrival cities or ICAOs");
        }
        route_show_keyboard(!dq || !dq[0] ? route_dep_ta : route_arr_ta);
        return;
    }
    s_route_dep_icao[0] = 0;
    s_route_arr_icao[0] = 0;
    s_route_row_n = 0;
    s_route_sel_row = -1;
    route_refresh_table();
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    strncpy(g_state->pending_route_dep, dq, sizeof(g_state->pending_route_dep) - 1);
    g_state->pending_route_dep[sizeof(g_state->pending_route_dep) - 1] = 0;
    strncpy(g_state->pending_route_arr, aq, sizeof(g_state->pending_route_arr) - 1);
    g_state->pending_route_arr[sizeof(g_state->pending_route_arr) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    if (route_status) {
        lv_label_set_text(route_status, "Searching airports...");
    }
    route_hide_keyboard();
    request(UI_REQ_ROUTE_SEARCH);
}

static void route_ta_clicked(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    route_show_keyboard(ta);
}

static void route_kb_event(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        route_search_clicked(e);
    } else if (code == LV_EVENT_CANCEL) {
        hide_route_search(e);
    }
}

static void route_table_draw_part(lv_event_t *e) {
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (!dsc || dsc->part != LV_PART_ITEMS || dsc->type != LV_TABLE_DRAW_PART_CELL || !route_table) {
        return;
    }
    if (s_route_sel_row < 1) {
        return;
    }
    const uint16_t col_cnt = lv_table_get_col_cnt(route_table);
    if (col_cnt == 0) {
        return;
    }
    const uint16_t row = (uint16_t)(dsc->id / col_cnt);
    if ((int)row != s_route_sel_row) {
        return;
    }
    if (dsc->rect_dsc) {
        dsc->rect_dsc->bg_color = lv_color_hex(0xB0B0B0);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
    }
    if (dsc->label_dsc) {
        dsc->label_dsc->color = lv_color_hex(0x101010);
    }
}

static void route_table_clicked(lv_event_t *e) {
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    uint16_t row = 0, col = 0;
    lv_table_get_selected_cell(table, &row, &col);
    if (row == 0 || row == LV_TABLE_CELL_NONE) {
        return;
    }
    const int idx = (int)row - 1;
    if (idx < 0 || idx >= s_route_row_n || !s_route_rows[idx].flight[0]) {
        return;
    }
    s_route_sel_row = (int)row;
    lv_obj_invalidate(table);
    route_hide_keyboard();
    if (route_status) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Tracking %s...", s_route_rows[idx].flight);
        lv_label_set_text(route_status, msg);
    }
    track_start_query(s_route_rows[idx].flight, false);
}

static void open_route_search(lv_event_t *e) {
    LV_UNUSED(e);
    if (!route_panel) {
        return;
    }
    hide_track(nullptr);
    hide_airport_board(nullptr);
    s_route_dep_hit_n = 0;
    s_route_arr_hit_n = 0;
    s_route_row_n = 0;
    s_route_sel_row = -1;
    s_route_dep_icao[0] = 0;
    s_route_arr_icao[0] = 0;
    route_refresh_hits_ui();
    route_refresh_table();
    if (route_title_lbl) {
        lv_label_set_text(route_title_lbl, "Departures and Arrivals Search");
    }
    if (route_status) {
        lv_label_set_text(route_status, "Enter cities or ICAOs, then Search - select a flight to follow");
    }
    if (route_dep_ta) {
        lv_textarea_set_text(route_dep_ta, "");
    }
    if (route_arr_ta) {
        lv_textarea_set_text(route_arr_ta, "");
        lv_obj_add_state(route_dep_ta, LV_STATE_FOCUSED);
    }
    lv_obj_clear_flag(route_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(route_panel);
    route_show_keyboard(route_dep_ta);
}

static void build_route_search() {
    static const char *kRouteKbMap[] = {
        "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "\n",
        "A", "S", "D", "F", "G", "H", "J", "K", "L", LV_SYMBOL_BACKSPACE, "\n",
        "Z", "X", "C", "V", "B", "N", "M", " ", "\n",
        LV_SYMBOL_CLOSE, LV_SYMBOL_OK, "",
    };
    static const lv_btnmatrix_ctrl_t kRouteKbCtrl[] = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 5,
        3, 3, 3, 3, 3, 3, 3, 8,
        LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    };

    route_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(route_panel, 1000, 560);
    lv_obj_center(route_panel);
    lv_obj_set_style_bg_color(route_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(route_panel, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(route_panel, 2, 0);
    lv_obj_set_style_radius(route_panel, 12, 0);
    lv_obj_set_style_pad_all(route_panel, 8, 0);
    lv_obj_clear_flag(route_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(route_panel, LV_OBJ_FLAG_HIDDEN);

    route_title_lbl = lv_label_create(route_panel);
    lv_label_set_text(route_title_lbl, "Departures and Arrivals Search");
    lv_obj_set_style_text_font(route_title_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(route_title_lbl, COL_GREEN, 0);
    lv_obj_align(route_title_lbl, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *rclose = make_btn(route_panel, "Close", hide_route_search, nullptr);
    lv_obj_set_size(rclose, 90, 36);
    lv_obj_align(rclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *hint = lv_label_create(route_panel);
    lv_label_set_text(hint, "Search by departure and arrival city or ICAO - select a flight to follow");
    lv_obj_set_style_text_color(hint, COL_DIM, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 8, 36);

    lv_obj_t *dep_lbl = lv_label_create(route_panel);
    lv_label_set_text(dep_lbl, "Dep");
    lv_obj_set_style_text_color(dep_lbl, COL_GREEN, 0);
    lv_obj_align(dep_lbl, LV_ALIGN_TOP_LEFT, 8, 66);

    route_dep_ta = lv_textarea_create(route_panel);
    lv_obj_set_size(route_dep_ta, 280, 40);
    lv_obj_align(route_dep_ta, LV_ALIGN_TOP_LEFT, 48, 58);
    lv_textarea_set_placeholder_text(route_dep_ta, "KATL or Atlanta");
    lv_textarea_set_one_line(route_dep_ta, true);
    lv_textarea_set_max_length(route_dep_ta, 40);
    lv_obj_add_event_cb(route_dep_ta, route_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(route_dep_ta, route_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *arr_lbl = lv_label_create(route_panel);
    lv_label_set_text(arr_lbl, "Arr");
    lv_obj_set_style_text_color(arr_lbl, COL_GREEN, 0);
    lv_obj_align(arr_lbl, LV_ALIGN_TOP_LEFT, 350, 66);

    route_arr_ta = lv_textarea_create(route_panel);
    lv_obj_set_size(route_arr_ta, 280, 40);
    lv_obj_align(route_arr_ta, LV_ALIGN_TOP_LEFT, 390, 58);
    lv_textarea_set_placeholder_text(route_arr_ta, "KJFK or New York");
    lv_textarea_set_one_line(route_arr_ta, true);
    lv_textarea_set_max_length(route_arr_ta, 40);
    lv_obj_add_event_cb(route_arr_ta, route_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(route_arr_ta, route_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *search = make_btn(route_panel, "Search", route_search_clicked, nullptr);
    lv_obj_set_size(search, 120, 40);
    lv_obj_align(search, LV_ALIGN_TOP_LEFT, 690, 58);

    route_status = lv_label_create(route_panel);
    lv_label_set_text(route_status, "");
    lv_obj_set_style_text_color(route_status, COL_GREEN, 0);
    lv_obj_set_style_text_font(route_status, &lv_font_montserrat_14, 0);
    lv_obj_set_width(route_status, 160);
    lv_label_set_long_mode(route_status, LV_LABEL_LONG_WRAP);
    lv_obj_align(route_status, LV_ALIGN_TOP_LEFT, 830, 58);

    route_dep_hits = lv_obj_create(route_panel);
    lv_obj_set_size(route_dep_hits, 480, 40);
    lv_obj_align(route_dep_hits, LV_ALIGN_TOP_LEFT, 4, 108);
    lv_obj_set_style_bg_opa(route_dep_hits, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(route_dep_hits, 0, 0);
    lv_obj_set_style_pad_all(route_dep_hits, 2, 0);
    lv_obj_set_style_pad_column(route_dep_hits, 6, 0);
    lv_obj_set_flex_flow(route_dep_hits, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_add_flag(route_dep_hits, LV_OBJ_FLAG_HIDDEN);

    route_arr_hits = lv_obj_create(route_panel);
    lv_obj_set_size(route_arr_hits, 480, 40);
    lv_obj_align(route_arr_hits, LV_ALIGN_TOP_LEFT, 504, 108);
    lv_obj_set_style_bg_opa(route_arr_hits, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(route_arr_hits, 0, 0);
    lv_obj_set_style_pad_all(route_arr_hits, 2, 0);
    lv_obj_set_style_pad_column(route_arr_hits, 6, 0);
    lv_obj_set_flex_flow(route_arr_hits, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_add_flag(route_arr_hits, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *res_lbl = lv_label_create(route_panel);
    lv_label_set_text(res_lbl, "Flights");
    lv_obj_set_style_text_color(res_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(res_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(res_lbl, LV_ALIGN_TOP_LEFT, 8, 156);

    route_table = lv_table_create(route_panel);
    lv_obj_set_size(route_table, 980, 360);
    lv_obj_align(route_table, LV_ALIGN_TOP_LEFT, 4, 182);
    lv_obj_set_style_bg_color(route_table, lv_color_hex(0x0A120C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(route_table, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(route_table, COL_BTN_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(route_table, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(route_table, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_all(route_table, 2, LV_PART_ITEMS);
    lv_obj_set_style_text_font(route_table, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_set_style_text_color(route_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(route_table, lv_color_hex(0x0A120C), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(route_table, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_scroll_dir(route_table, LV_DIR_VER);
    lv_obj_add_event_cb(route_table, route_table_clicked, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(route_table, route_table_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);
    route_refresh_table();

    route_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(route_kb, 900, 220);
    lv_obj_align(route_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_map(route_kb, LV_KEYBOARD_MODE_USER_1, kRouteKbMap, kRouteKbCtrl);
    lv_keyboard_set_mode(route_kb, LV_KEYBOARD_MODE_USER_1);
    lv_keyboard_set_textarea(route_kb, route_dep_ta);
    lv_obj_add_event_cb(route_kb, route_kb_event, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(route_kb, route_kb_event, LV_EVENT_CANCEL, nullptr);
    lv_obj_add_event_cb(route_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(route_kb, LV_OBJ_FLAG_HIDDEN);
}

static void traffic_table_selected(lv_event_t *e);
static void traffic_table_draw_part(lv_event_t *e);
static void refresh_traffic_table();
static void update_traffic_headers();
static bool radar_overlay_open();
static bool follow_chart_timeline(uint32_t *dep_unix_out, uint32_t *arr_unix_out);
static uint32_t hist_pt_unix(const HistPt &pt);

static void follow_chart_draw(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const lv_coord_t pad_l = 8;
    const lv_coord_t pad_r = 8;
    // Half-line gap under MAX ALT / MAX SPD header before the plot grid.
    const lv_coord_t pad_t = 22 + (lv_font_montserrat_12.line_height / 2);
    const lv_coord_t pad_b = 22;  // room for time scale under plot
    const lv_coord_t gx1 = coords.x1 + pad_l;
    const lv_coord_t gy1 = coords.y1 + pad_t;
    const lv_coord_t gx2 = coords.x2 - pad_r;
    const lv_coord_t gy2 = coords.y2 - pad_b;
    const lv_coord_t gw = gx2 - gx1;
    const lv_coord_t gh = gy2 - gy1;
    if (gw < 20 || gh < 20) {
        return;
    }

    lv_draw_rect_dsc_t bg;
    lv_draw_rect_dsc_init(&bg);
    bg.bg_color = lv_color_hex(0x0A120C);
    bg.bg_opa = LV_OPA_COVER;
    bg.radius = 6;
    lv_draw_rect(ctx, &bg, &coords);

    // —— Time / value scale (before series lines) ——
    float amax_rec = 0.0f, smax_rec = 0.0f;
    const int oldest = (s_hist_head - s_hist_count + kHistMax) % kHistMax;
    uint32_t dep_u = 0, arr_u = 0;
    const bool have_tl = follow_chart_timeline(&dep_u, &arr_u);
    const uint32_t t0_ms = (s_hist_count > 0) ? s_hist[oldest].ms : millis();
    const uint32_t window_ms =
        have_tl ? (arr_u - dep_u) * 1000u : hist_window_ms();
    for (int i = 0; i < s_hist_count; i++) {
        const HistPt &pt = s_hist[(oldest + i) % kHistMax];
        if (pt.alt_ft > amax_rec) {
            amax_rec = pt.alt_ft;
        }
        if (pt.gs_kts > smax_rec) {
            smax_rec = pt.gs_kts;
        }
    }
    // Bottom = 0 AGL / 0 kt. Alt top = max+10%; speed top = max/0.7.
    float a_top = amax_rec * 1.1f;
    float s_top = smax_rec / 0.7f;
    a_top = roundf(a_top / 100.0f) * 100.0f;
    s_top = roundf(s_top / 10.0f) * 10.0f;
    if (a_top < 100.0f) {
        a_top = 100.0f;
    }
    if (s_top < 10.0f) {
        s_top = 10.0f;
    }

    auto y_of = [&](float v, float vmax) -> lv_coord_t {
        float t = v / vmax;
        if (t < 0.0f) {
            t = 0.0f;
        }
        if (t > 1.0f) {
            t = 1.0f;
        }
        return (lv_coord_t)lroundf((float)gy2 - t * (float)gh);
    };
    auto x_of = [&](const HistPt &pt) -> lv_coord_t {
        float xf = 0.0f;
        if (have_tl && arr_u > dep_u) {
            const uint32_t u = hist_pt_unix(pt);
            if (u >= dep_u) {
                xf = (float)(u - dep_u) / (float)(arr_u - dep_u);
            }
        } else if (window_ms > 0) {
            const uint32_t dt = (pt.ms >= t0_ms) ? (pt.ms - t0_ms) : 0;
            xf = (float)dt / (float)window_ms;
        }
        if (xf < 0.0f) {
            xf = 0.0f;
        }
        if (xf > 1.0f) {
            xf = 1.0f;
        }
        return gx1 + (lv_coord_t)lroundf(xf * (float)gw);
    };

    lv_draw_line_dsc_t grid;
    lv_draw_line_dsc_init(&grid);
    grid.color = COL_DIM;
    grid.width = 1;
    grid.opa = LV_OPA_40;
    for (int i = 0; i <= 4; i++) {
        const lv_coord_t y = gy1 + (gh * i) / 4;
        lv_point_t a = {gx1, y};
        lv_point_t b = {gx2, y};
        lv_draw_line(ctx, &grid, &a, &b);
    }

    lv_draw_label_dsc_t lab;
    lv_draw_label_dsc_init(&lab);
    lab.font = &lv_font_montserrat_12;
    char alt_lbl[40], spd_lbl[40];
    if (s_hist_count >= 2) {
        snprintf(alt_lbl, sizeof(alt_lbl), "MAX ALT: %.0f", amax_rec);
        snprintf(spd_lbl, sizeof(spd_lbl), "MAX SPD: %.0f", smax_rec);
    } else {
        strncpy(alt_lbl, "MAX ALT:", sizeof(alt_lbl) - 1);
        alt_lbl[sizeof(alt_lbl) - 1] = 0;
        strncpy(spd_lbl, "MAX SPD:", sizeof(spd_lbl) - 1);
        spd_lbl[sizeof(spd_lbl) - 1] = 0;
    }
    lab.color = COL_GREEN;
    lab.align = LV_TEXT_ALIGN_LEFT;
    lv_area_t ta = {(lv_coord_t)(coords.x1 + 8), (lv_coord_t)(coords.y1 + 4),
                    (lv_coord_t)(coords.x1 + 180), (lv_coord_t)(coords.y1 + 20)};
    lv_draw_label(ctx, &lab, &ta, alt_lbl, nullptr);
    lab.color = COL_SPD;
    lab.align = LV_TEXT_ALIGN_RIGHT;
    ta.x1 = coords.x2 - 140;
    ta.x2 = coords.x2 - 8;
    lv_draw_label(ctx, &lab, &ta, spd_lbl, nullptr);

    // Time origin "0" flush left (takeoff / 0 AGL).
    char win_lbl[24];
    format_hist_window_label(win_lbl, sizeof(win_lbl), window_ms);
    lab.color = COL_TEXT;
    lab.align = LV_TEXT_ALIGN_LEFT;
    ta.x1 = gx1;
    ta.y1 = gy2 + 2;
    ta.x2 = gx1 + 28;
    ta.y2 = coords.y2 - 1;
    lv_draw_label(ctx, &lab, &ta, "0", nullptr);

    if (s_hist_count < 2) {
        lab.color = COL_TEXT;
        lab.align = LV_TEXT_ALIGN_CENTER;
        ta.x1 = gx1;
        ta.y1 = gy1 + gh / 2 - 8;
        ta.x2 = gx2;
        ta.y2 = ta.y1 + 16;
        lv_draw_label(ctx, &lab, &ta, "Collecting samples...", nullptr);
        // Still show expected total on the right while collecting.
        lab.align = LV_TEXT_ALIGN_RIGHT;
        ta.x1 = gx2 - 80;
        ta.y1 = gy2 + 2;
        ta.x2 = gx2;
        ta.y2 = coords.y2 - 1;
        lv_draw_label(ctx, &lab, &ta, win_lbl, nullptr);
        return;
    }

    lv_draw_line_dsc_t alt_ln;
    lv_draw_line_dsc_init(&alt_ln);
    alt_ln.color = COL_GREEN;
    alt_ln.width = 2;
    alt_ln.opa = LV_OPA_COVER;
    alt_ln.round_start = 1;
    alt_ln.round_end = 1;

    lv_draw_line_dsc_t spd_ln;
    lv_draw_line_dsc_init(&spd_ln);
    spd_ln.color = COL_SPD;
    spd_ln.width = 2;
    spd_ln.opa = LV_OPA_COVER;
    spd_ln.round_start = 1;
    spd_ln.round_end = 1;

    // Start at takeoff: 0 AGL / 0 kt flush left, then series.
    lv_coord_t prev_ax = gx1;
    lv_coord_t prev_ay = gy2;
    lv_coord_t prev_sx = gx1;
    lv_coord_t prev_sy = gy2;
    bool have_prev = have_tl;  // timeline → origin is takeoff at left
    float last_alt = 0.0f, last_spd = 0.0f;
    uint32_t tip_unix = dep_u;
    for (int i = 0; i < s_hist_count; i++) {
        const HistPt &pt = s_hist[(oldest + i) % kHistMax];
        const lv_coord_t x = x_of(pt);
        const lv_coord_t ay = y_of(pt.alt_ft, a_top);
        const lv_coord_t sy = y_of(pt.gs_kts, s_top);
        if (have_prev) {
            lv_point_t a0 = {prev_ax, prev_ay};
            lv_point_t a1 = {x, ay};
            lv_draw_line(ctx, &alt_ln, &a0, &a1);
            lv_point_t s0 = {prev_sx, prev_sy};
            lv_point_t s1 = {x, sy};
            lv_draw_line(ctx, &spd_ln, &s0, &s1);
        }
        have_prev = true;
        prev_ax = x;
        prev_ay = ay;
        prev_sx = x;
        prev_sy = sy;
        last_alt = pt.alt_ft;
        last_spd = pt.gs_kts;
        tip_unix = hist_pt_unix(pt);
    }

    // Latest values near the current tip. Alt above the line; speed below when space allows.
    const lv_coord_t tip_w = 56;
    const lv_coord_t tip_h = 14;
    auto tip_above = [&](lv_coord_t y) -> lv_coord_t {
        lv_coord_t ly = y - tip_h - 4;
        if (ly < coords.y1 + 2) {
            ly = coords.y1 + 2;
        }
        if (ly + tip_h > gy2) {
            ly = gy2 - tip_h;
        }
        return ly;
    };
    auto tip_below = [&](lv_coord_t y) -> lv_coord_t {
        lv_coord_t ly = y + 4;
        if (ly + tip_h <= gy2) {
            return ly;
        }
        return tip_above(y);
    };
    lv_coord_t ay_l = tip_above(prev_ay);
    lv_coord_t sy_l = tip_below(prev_sy);
    const lv_coord_t tip_dy = (ay_l > sy_l) ? (ay_l - sy_l) : (sy_l - ay_l);
    if (tip_dy < tip_h + 2) {
        if (sy_l >= prev_sy) {
            lv_coord_t down = ay_l + tip_h + 2;
            if (down + tip_h <= gy2) {
                sy_l = down;
            } else {
                ay_l = sy_l - tip_h - 2;
                if (ay_l < coords.y1 + 2) {
                    ay_l = coords.y1 + 2;
                }
            }
        } else {
            sy_l = ay_l + tip_h + 2;
            if (sy_l + tip_h > gy2) {
                sy_l = tip_above(prev_sy);
            }
        }
    }
    lv_coord_t tip_x2 = prev_ax + 4;
    if (tip_x2 < gx1 + tip_w) {
        tip_x2 = gx1 + tip_w;
    }
    if (tip_x2 > gx2) {
        tip_x2 = gx2;
    }
    const lv_coord_t tip_x1 = tip_x2 - tip_w;
    char val[24];
    lab.font = &lv_font_montserrat_12;
    lab.align = LV_TEXT_ALIGN_RIGHT;
    lab.color = COL_GREEN;
    snprintf(val, sizeof(val), "%.0f", last_alt);
    ta.x1 = tip_x1;
    ta.y1 = ay_l;
    ta.x2 = tip_x2;
    ta.y2 = ay_l + tip_h;
    lv_draw_label(ctx, &lab, &ta, val, nullptr);
    lab.color = COL_SPD;
    snprintf(val, sizeof(val), "%.0f", last_spd);
    ta.y1 = sy_l;
    ta.y2 = sy_l + tip_h;
    lv_draw_label(ctx, &lab, &ta, val, nullptr);

    // Bottom: actual flight time tracks the data tip; hide ETA total when they overlap.
    uint32_t elapsed_ms = 0;
    if (have_tl && tip_unix >= dep_u) {
        elapsed_ms = (tip_unix - dep_u) * 1000u;
    } else if (s_hist_count > 0) {
        const HistPt &newest = s_hist[(s_hist_head - 1 + kHistMax) % kHistMax];
        elapsed_ms = (newest.ms >= t0_ms) ? (newest.ms - t0_ms) : 0;
    }
    char flt_lbl[24];
    format_hist_window_label(flt_lbl, sizeof(flt_lbl), elapsed_ms);

    const lv_coord_t flt_w = 72;
    lv_coord_t flt_x1 = prev_ax - flt_w / 2;
    if (flt_x1 < gx1 + 20) {
        flt_x1 = gx1 + 20;  // keep clear of the "0"
    }
    if (flt_x1 + flt_w > gx2) {
        flt_x1 = gx2 - flt_w;
    }
    lab.color = COL_TEXT;
    lab.align = LV_TEXT_ALIGN_CENTER;
    ta.x1 = flt_x1;
    ta.y1 = gy2 + 2;
    ta.x2 = flt_x1 + flt_w;
    ta.y2 = coords.y2 - 1;
    lv_draw_label(ctx, &lab, &ta, flt_lbl, nullptr);

    // Show expected total on the right until the moving flight time gets close.
    const lv_coord_t tot_w = 80;
    const lv_coord_t clear_pad = 12;
    if (flt_x1 + flt_w + clear_pad < gx2 - tot_w) {
        lab.align = LV_TEXT_ALIGN_RIGHT;
        ta.x1 = gx2 - tot_w;
        ta.y1 = gy2 + 2;
        ta.x2 = gx2;
        ta.y2 = coords.y2 - 1;
        lv_draw_label(ctx, &lab, &ta, win_lbl, nullptr);
    }
}

static void set_follow_side_mode(bool following) {
    if (traffic_table) {
        if (following) {
            lv_obj_add_flag(traffic_table, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(traffic_table, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (follow_panel) {
        if (following) {
            lv_obj_clear_flag(follow_panel, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(follow_panel, LV_OBJ_FLAG_HIDDEN);
        }
    }
    // Lower row always visible (Track | Hide Wpts | Center). Update ADS-B lives next to Home.
    if (follow_btn_row) {
        lv_obj_clear_flag(follow_btn_row, LV_OBJ_FLAG_HIDDEN);
    }
    auto set_follow_btn_vis = [](lv_obj_t *btn, bool show) {
        if (!btn) {
            return;
        }
        if (show) {
            lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);
        }
    };
    set_follow_btn_vis(follow_center_btn, following);
    // Always use follow height so layout does not jump when entering/leaving follow.
    const lv_coord_t ch = s_controls_h_follow;
    const lv_coord_t th = kSideH - 24 - ch - 8;
    s_follow_table_h = th;
    if (side_controls) {
        lv_obj_set_height(side_controls, ch);
    }
    if (traffic_table) {
        lv_obj_set_height(traffic_table, th);
    }
    if (follow_panel) {
        lv_obj_set_height(follow_panel, th);
    }
    if (following) {
        layout_follow_info_boxes();
    }
}

static void layout_follow_info_boxes() {
    if (!follow_info_lbl || !follow_sched_lbl || !follow_panel) {
        return;
    }
    const lv_coord_t info_w = lv_obj_get_width(follow_info_lbl);
    const lv_coord_t sched_w = lv_obj_get_width(follow_sched_lbl);
    const lv_coord_t info_pad = 6;
    const lv_coord_t sched_pad_h = 6 + 4;
    const lv_coord_t sched_pad_v = 2 + 2;

    lv_point_t info_sz = {0, 0};
    const char *info_txt = lv_label_get_text(follow_info_lbl);
    if (info_txt && info_txt[0]) {
        const lv_coord_t max_w = LV_MAX(info_w - 2 * info_pad, 20);
        lv_txt_get_size(&info_sz, info_txt, &lv_font_montserrat_14, 0, 0, max_w, LV_TEXT_FLAG_NONE);
    }
    lv_coord_t info_h = info_sz.y + 2 * info_pad;

    lv_point_t sched_sz = {0, 0};
    const char *sched_txt = lv_label_get_text(follow_sched_lbl);
    if (sched_txt && sched_txt[0]) {
        const lv_coord_t max_w = LV_MAX(sched_w - sched_pad_h, 20);
        lv_txt_get_size(&sched_sz, sched_txt, &lv_font_montserrat_12, 0, -2, max_w, LV_TEXT_FLAG_NONE);
    }
    lv_coord_t sched_h = sched_sz.y + sched_pad_v;

    // Keep info + DEP/ARR boxes the same height (taller of the two).
    lv_coord_t box_h = LV_MAX(info_h, sched_h);
    if (box_h < 40) {
        box_h = 40;
    }

    const lv_coord_t panel_h = lv_obj_get_height(follow_panel);
    const lv_coord_t gap = 4;
    lv_coord_t chart_h = panel_h - box_h - gap;
    if (chart_h < 60) {
        box_h = panel_h - gap - 60;
        if (box_h < 40) {
            box_h = 40;
        }
        chart_h = panel_h - box_h - gap;
        if (chart_h < 60) {
            chart_h = 60;
        }
    }

    lv_obj_set_style_pad_all(follow_info_lbl, info_pad, 0);
    lv_obj_set_style_pad_top(follow_sched_lbl, 2, 0);
    lv_obj_set_style_pad_bottom(follow_sched_lbl, 2, 0);
    lv_obj_set_height(follow_info_lbl, box_h);
    lv_obj_set_height(follow_sched_lbl, box_h);
    lv_obj_align(follow_info_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_align(follow_sched_lbl, LV_ALIGN_TOP_RIGHT, 0, 0);
    if (follow_chart) {
        lv_obj_set_height(follow_chart, chart_h);
        lv_obj_align(follow_chart, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_invalidate(follow_chart);
    }
}

static bool follow_parse_act_hm(const char *act, int *hh, int *mm);
static bool follow_parse_flight_md(const char *fd, int *mo, int *d);
static time_t follow_timegm(int y, int mo, int d, int h, int mi, int s);
static uint32_t follow_dep_epoch_utc();

/** Arrival airport lat/lon for TZ (resolved dest — never home / map center). */
static bool follow_arr_ll(float *lat, float *lon) {
    if (!lat || !lon) {
        return false;
    }
    if (s_follow_arr.ok) {
        *lat = s_follow_arr.lat;
        *lon = s_follow_arr.lon;
        return true;
    }
    const char *code = s_follow_fi.dest[0] ? s_follow_fi.dest : nullptr;
    if (!code || !code[0]) {
        return false;
    }
    float la = 0, lo = 0;
    if (net_lookup_airport(code, &la, &lo, nullptr)) {
        *lat = la;
        *lon = lo;
        return true;
    }
    if (net_lookup_airport_iata(code, nullptr, 0, &la, &lo)) {
        *lat = la;
        *lon = lo;
        return true;
    }
    return false;
}

/** Whole-token timezone abbreviation match (avoids false hits inside longer words). */
static bool tz_abbr_token(const char *act, const char *ab) {
    if (!act || !ab || !ab[0]) {
        return false;
    }
    const size_t n = strlen(ab);
    for (const char *p = act; *p; p++) {
        if (strncasecmp(p, ab, n) != 0) {
            continue;
        }
        const bool left_ok = (p == act) || !isalpha((unsigned char)p[-1]);
        const bool right_ok = !isalpha((unsigned char)p[n]);
        if (left_ok && right_ok) {
            return true;
        }
    }
    return false;
}

/** Offset seconds east of UTC from Act/Est suffix (e.g. "14:22 CDT"), or false if none. */
static bool tz_offset_from_act_suffix(const char *act, int *off_out) {
    if (!act || !act[0] || !off_out) {
        return false;
    }
    // Longer abbreviations first. Generic CT/ET/MT/PT refined via airport DST by caller.
    static const struct {
        const char *ab;
        int off;
    } kTz[] = {
        {"AKDT", -8 * 3600}, {"AKST", -9 * 3600}, {"HADT", -9 * 3600}, {"HAST", -10 * 3600},
        {"EDT", -4 * 3600},  {"EST", -5 * 3600},  {"CDT", -5 * 3600},  {"CST", -6 * 3600},
        {"MDT", -6 * 3600},  {"MST", -7 * 3600},  {"PDT", -7 * 3600},  {"PST", -8 * 3600},
        {"ADT", -3 * 3600},  {"AST", -4 * 3600},  {"HST", -10 * 3600}, {"UTC", 0},
        {"GMT", 0},          {"Z", 0},            {"CT", -6 * 3600},   {"ET", -5 * 3600},
        {"MT", -7 * 3600},   {"PT", -8 * 3600},
    };
    for (size_t i = 0; i < sizeof(kTz) / sizeof(kTz[0]); i++) {
        if (tz_abbr_token(act, kTz[i].ab)) {
            *off_out = kTz[i].off;
            return true;
        }
    }
    return false;
}

static bool tz_suffix_is_generic_us(const char *act) {
    return tz_abbr_token(act, "CT") || tz_abbr_token(act, "ET") || tz_abbr_token(act, "MT") ||
           tz_abbr_token(act, "PT");
}

/** Civil TZ at arrival airport (Central for DAL, etc.) — never home PPI center. */
static int follow_arrival_tz_off(const char *hm_local) {
    float lat = 0, lon = 0;
    const bool have_ll = follow_arr_ll(&lat, &lon);
    int off = 0;
    if (hm_local && tz_offset_from_act_suffix(hm_local, &off)) {
        // "CT"/"ET"/… or missing DST letter → use airport lon + DST so Central is correct.
        if (tz_suffix_is_generic_us(hm_local) && have_ll) {
            return tz_offset_for_ll(lat, lon);
        }
        return off;
    }
    if (have_ll) {
        return tz_offset_for_ll(lat, lon);
    }
    return s_tz_off_sec;
}

/** Best landing UTC: FlightStats dateUtc, else Est/Sch local wall @ arrival TZ. */
static uint32_t follow_arrival_epoch_utc() {
    const time_t now = time(nullptr);
    if (now < (time_t)1700000000L) {
        return 0;
    }
    uint32_t arr = s_follow_fi.arr_est_epoch;
    if (!arr) {
        arr = s_follow_fi.arr_act_epoch;
    }
    if (arr) {
        const long el = (long)arr - (long)now;
        // Accept ETA up to 20 h ahead or a few minutes past (clock skew / just landed).
        if (el >= -30L * 60L && el <= 20L * 3600L) {
            return arr;
        }
    }

    const char *hm = s_follow_fi.arr_est[0] ? s_follow_fi.arr_est
                                            : (s_follow_fi.arr_time[0] ? s_follow_fi.arr_time : "");
    int hh = 0, mm = 0;
    if (!hm[0] || !follow_parse_act_hm(hm, &hh, &mm)) {
        return 0;
    }
    const int off = follow_arrival_tz_off(hm);

    // Civil day in *arrival* timezone (so Central mid-evening is not forced to Eastern date).
    time_t local_now = now + (time_t)off;
    struct tm tl{};
    gmtime_r(&local_now, &tl);
    int y = tl.tm_year + 1900, mo = tl.tm_mon + 1, d = tl.tm_mday;
    int fmo = 0, fd = 0;
    if (follow_parse_flight_md(s_follow_fi.flight_date, &fmo, &fd)) {
        // flight_date is usually departure-local; keep month/day but re-anchor if needed below.
        mo = fmo;
        d = fd;
    }
    time_t arr_t = follow_timegm(y, mo, d, hh, mm, 0);
    if (arr_t <= 0) {
        return 0;
    }
    arr_t -= (time_t)off;

    const uint32_t dep = follow_dep_epoch_utc();
    if (dep) {
        while (arr_t + 120 < (time_t)dep) {
            arr_t += 86400;
        }
        // Arrival more than ~18 h after takeoff → likely wrong day from flight_date.
        while (arr_t > (time_t)dep + 18L * 3600L) {
            arr_t -= 86400;
        }
    } else {
        // No takeoff: pick nearest civil day so Est is not 20 h in the past.
        long el = (long)arr_t - (long)now;
        if (el < -2L * 3600L) {
            arr_t += 86400;
        } else if (el > 20L * 3600L) {
            arr_t -= 86400;
        }
    }
    return (uint32_t)arr_t;
}

/** Enroute time remaining: prefer Est landing (arrival TZ) − now; else ADS-B nm/GS. */
static void format_follow_remaining(char *out, size_t n) {
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    if (s_follow_ac_ok && s_follow_ac.on_ground) {
        snprintf(out, n, s_follow_fi.departed ? "Arrived" : "--");
        return;
    }

    auto emit_mins = [&](int mins) {
        if (mins < 1) {
            snprintf(out, n, "<1 min");
        } else if (mins >= 60) {
            snprintf(out, n, "%d h %02d m", mins / 60, mins % 60);
        } else {
            snprintf(out, n, "%d min", mins);
        }
    };

    const time_t now = time(nullptr);
    const uint32_t arr = follow_arrival_epoch_utc();
    if (arr && now >= (time_t)1700000000L) {
        if ((uint32_t)now >= arr) {
            snprintf(out, n, s_follow_ac_ok && s_follow_ac.on_ground ? "Arrived" : "<1 min");
            return;
        }
        emit_mins((int)((arr - (uint32_t)now) / 60u));
        return;
    }

    if (!s_follow_ac_ok || !s_follow_arr.ok) {
        snprintf(out, n, "--");
        return;
    }
    const Aircraft *ac = &s_follow_ac;
    float east = 0, north = 0;
    ll_to_nm(s_follow_arr.lat, s_follow_arr.lon, ac->lat, ac->lon, &east, &north);
    const float nm = hypotf(east, north);
    const float gs = ac->gs_kts;
    if (isnan(gs) || gs < 40.0f || nm < 0.5f) {
        snprintf(out, n, nm < 0.5f ? "<1 min" : "--");
        return;
    }
    int mins = (int)lroundf((nm / gs) * 60.0f);
    if (mins < 1) {
        mins = 1;
    }
    emit_mins(mins);
}

/** Civil date → days since 1970-01-01 (Howard Hinnant). */
static long follow_civil_days(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return (long)era * 146097L + (long)doe - 719468L;
}

static time_t follow_timegm(int y, int mo, int d, int h, int mi, int s) {
    if (y < 2020 || mo < 1 || mo > 12 || d < 1 || d > 31) {
        return 0;
    }
    return (time_t)(follow_civil_days(y, (unsigned)mo, (unsigned)d) * 86400L + (long)h * 3600L +
                    (long)mi * 60L + (long)s);
}

/** Parse "14:22 EDT" / "14:22" from FlightStats Act display. */
static bool follow_parse_act_hm(const char *act, int *hh, int *mm) {
    if (!act || !hh || !mm) {
        return false;
    }
    int h = 0, m = 0;
    if (sscanf(act, "%d:%d", &h, &m) != 2 || h < 0 || h > 23 || m < 0 || m > 59) {
        return false;
    }
    *hh = h;
    *mm = m;
    return true;
}

/** Parse "Wed Oct 21" → month/day. */
static bool follow_parse_flight_md(const char *fd, int *mo, int *d) {
    if (!fd || !fd[0] || !mo || !d) {
        return false;
    }
    static const char *const kMon[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    // Skip weekday token.
    const char *p = fd;
    while (*p && *p != ' ') {
        p++;
    }
    while (*p == ' ') {
        p++;
    }
    if (strlen(p) < 5) {
        return false;
    }
    char mon[4] = {p[0], p[1], p[2], 0};
    int day = 0;
    if (sscanf(p + 3, "%d", &day) != 1 || day < 1 || day > 31) {
        return false;
    }
    for (int i = 0; i < 12; i++) {
        if (strcasecmp(mon, kMon[i]) == 0) {
            *mo = i + 1;
            *d = day;
            return true;
        }
    }
    return false;
}

/**
 * Takeoff UTC from displayed Act local time + TZ suffix (or dep-airport TZ).
 * Fallback only — prefer FlightStats dep_act_epoch when present.
 */
static uint32_t follow_takeoff_epoch_from_act() {
    const FlightInfo *fi = &s_follow_fi;
    int hh = 0, mm = 0;
    if (!follow_parse_act_hm(fi->dep_act, &hh, &mm)) {
        return 0;
    }
    const time_t now = time(nullptr);
    if (now < (time_t)1700000000L) {
        return 0;
    }
    int off = 0;
    if (!tz_offset_from_act_suffix(fi->dep_act, &off)) {
        float lat = s_olat;
        float lon = s_olon;
        if (s_follow_dep.ok) {
            lat = s_follow_dep.lat;
            lon = s_follow_dep.lon;
        }
        off = tz_offset_for_ll(lat, lon);
    }

    time_t local_now = now + (time_t)off;
    struct tm tl{};
    gmtime_r(&local_now, &tl);
    int y = tl.tm_year + 1900;
    int mo = tl.tm_mon + 1;
    int d = tl.tm_mday;
    int fmo = 0, fd = 0;
    if (follow_parse_flight_md(fi->flight_date, &fmo, &fd)) {
        mo = fmo;
        d = fd;
    }

    // local wall → UTC: utc = timegm(wall) - tz_off
    time_t takeoff = follow_timegm(y, mo, d, hh, mm, 0);
    if (takeoff <= 0) {
        return 0;
    }
    takeoff -= (time_t)off;

    // Nudge ±1 day when flight_date / midnight crosses make Act land wrong day.
    long el = (long)now - (long)takeoff;
    if (el < -2L * 60L) {
        // Parsed takeoff still in the future → try previous civil day.
        takeoff -= 86400;
        el = (long)now - (long)takeoff;
    } else if (el > 20L * 3600L) {
        takeoff += 86400;
        el = (long)now - (long)takeoff;
    }
    if (el < -15L * 60L || el > 20L * 3600L) {
        return 0;
    }
    return (uint32_t)takeoff;
}

/** Best takeoff UTC: FlightStats dateUtc epoch first, then Act local parse. */
static uint32_t follow_dep_epoch_utc() {
    const time_t now = time(nullptr);
    if (now < (time_t)1700000000L) {
        return 0;
    }
    uint32_t ep = s_follow_fi.dep_act_epoch;
    if (ep) {
        const long el = (long)now - (long)ep;
        // Allow a few minutes of clock skew / just-departed.
        if (el >= -15L * 60L && el <= 20L * 3600L) {
            return ep;
        }
    }
    return follow_takeoff_epoch_from_act();
}

/** UTC epoch for a hist sample (prefer stored FA/live unix; else millis age). */
static uint32_t hist_pt_unix(const HistPt &pt) {
    if (pt.t_unix >= 1700000000u) {
        return pt.t_unix;
    }
    const time_t now_u = time(nullptr);
    const uint32_t now_ms = millis();
    if (now_u < (time_t)1700000000L) {
        return 0;
    }
    long age_ms = (long)now_ms - (long)pt.ms;
    if (age_ms < 0) {
        age_ms = 0;
    }
    return (uint32_t)(now_u - (time_t)(age_ms / 1000L));
}

/** Remaining enroute seconds: Est landing − now (arrival TZ), else ADS-B nm/GS. */
static uint32_t follow_remaining_sec() {
    const time_t now = time(nullptr);
    const uint32_t arr = follow_arrival_epoch_utc();
    if (arr && now >= (time_t)1700000000L && (uint32_t)now < arr) {
        uint32_t sec = arr - (uint32_t)now;
        if (sec < 60u) {
            return 60;
        }
        if (sec > 20u * 3600u) {
            return 20u * 3600u;
        }
        return sec;
    }
    if (!s_follow_ac_ok || !s_follow_arr.ok) {
        return 0;
    }
    const Aircraft *ac = &s_follow_ac;
    if (ac->on_ground) {
        return 0;
    }
    float east = 0, north = 0;
    ll_to_nm(s_follow_arr.lat, s_follow_arr.lon, ac->lat, ac->lon, &east, &north);
    const float nm = hypotf(east, north);
    const float gs = ac->gs_kts;
    if (isnan(gs) || gs < 40.0f || nm < 0.5f) {
        return 0;
    }
    const float sec_f = (nm / gs) * 3600.0f;
    if (sec_f < 60.0f) {
        return 60;
    }
    if (sec_f > 20.0f * 3600.0f) {
        return (uint32_t)(20.0f * 3600.0f);
    }
    return (uint32_t)lroundf(sec_f);
}

/**
 * Chart X-axis: takeoff (0) → expected landing.
 * Returns false if timeline cannot be formed (caller falls back to sample span).
 */
static bool follow_chart_timeline(uint32_t *dep_unix_out, uint32_t *arr_unix_out) {
    if (!dep_unix_out || !arr_unix_out) {
        return false;
    }
    *dep_unix_out = 0;
    *arr_unix_out = 0;
    const time_t now = time(nullptr);
    if (now < (time_t)1700000000L) {
        return false;
    }

    // Left edge = climb start (wheels-up), never gate-out / taxi departure.
    uint32_t dep = s_hist_fa_t0_unix;
    if (!dep && s_hist_count > 0) {
        const int oldest = (s_hist_head - s_hist_count + kHistMax) % kHistMax;
        dep = hist_pt_unix(s_hist[oldest]);
    }
    if (!dep) {
        return false;
    }

    // Prefer estimated landing (arrival-airport TZ / dateUtc) for the right edge.
    uint32_t arr = follow_arrival_epoch_utc();
    if (!arr) {
        const uint32_t rem = follow_remaining_sec();
        if (rem > 0) {
            arr = (uint32_t)now + rem;
        }
    }
    if (!arr) {
        const uint32_t span_ms = hist_span_ms();
        uint32_t pad_sec = span_ms / 1000u;
        if (pad_sec < 30u * 60u) {
            pad_sec = 30u * 60u;
        }
        arr = dep + pad_sec;
    }

    // Keep right edge at expected landing; stretch only if still flying past ETA.
    uint32_t latest = (uint32_t)now;
    if (s_hist_count > 0) {
        const int newest = (s_hist_head - 1 + kHistMax) % kHistMax;
        const uint32_t pu = hist_pt_unix(s_hist[newest]);
        if (pu > latest) {
            latest = pu;
        }
    }
    if (latest > arr) {
        arr = latest;
    }
    if (arr <= dep + 60u) {
        arr = dep + 60u;
    }
    if (arr - dep > 20u * 3600u) {
        arr = dep + 20u * 3600u;
    }

    *dep_unix_out = dep;
    *arr_unix_out = arr;
    return true;
}

/**
 * Flight time in info box: airborne only (climb start → now / touchdown).
 * Excludes taxi-out and taxi-in; shows "--" until altitude starts climbing.
 */
static void format_follow_flight_time(char *out, size_t n) {
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    const time_t now = time(nullptr);
    if (now < (time_t)1700000000L) {
        snprintf(out, n, "--");
        return;
    }

    auto emit_mins = [&](int mins) {
        if (mins < 1) {
            snprintf(out, n, "<1 min");
        } else if (mins >= 60) {
            snprintf(out, n, "%d h %02d m", mins / 60, mins % 60);
        } else {
            snprintf(out, n, "%d min", mins);
        }
    };

    // Prefer chart climb-start lock (FA or live alt rise) — not gate-out Act.
    uint32_t start_ep = s_hist_fa_t0_unix;
    if (start_ep) {
        const long el = (long)now - (long)start_ep;
        if (el < -15L * 60L || el > 20L * 3600L) {
            start_ep = 0;
        }
    }

    if (start_ep) {
        uint32_t end_ep = (uint32_t)now;
        if (s_follow_land_ms != 0) {
            const uint32_t now_ms = millis();
            long age_ms = (long)now_ms - (long)s_follow_land_ms;
            if (age_ms < 0) {
                age_ms = 0;
            }
            end_ep = (uint32_t)now - (uint32_t)(age_ms / 1000L);
        } else if (s_follow_fi.arr_act_epoch > start_ep &&
                   (!s_follow_ac_ok || !follow_ac_is_airborne(&s_follow_ac))) {
            // Touchdown from schedule when we did not latch land locally.
            end_ep = s_follow_fi.arr_act_epoch;
        }
        if (end_ep < start_ep) {
            snprintf(out, n, "--");
            return;
        }
        emit_mins((int)((end_ep - start_ep) / 60u));
        return;
    }

    // Fallback: local wheels-up latch (AGL ≥ 100) until climb unix is known.
    if (s_follow_takeoff_ms == 0) {
        snprintf(out, n, "--");  // still taxiing / not yet airborne
        return;
    }
    const uint32_t end_ms = s_follow_land_ms ? s_follow_land_ms : millis();
    if (end_ms < s_follow_takeoff_ms) {
        snprintf(out, n, "--");
        return;
    }
    emit_mins((int)((end_ms - s_follow_takeoff_ms) / 60000u));
}

static const char *const kUsStateNames[50] = {
    "Alabama",        "Alaska",       "Arizona",      "Arkansas",     "California",
    "Colorado",       "Connecticut",  "Delaware",     "Florida",      "Georgia",
    "Hawaii",         "Idaho",        "Illinois",     "Indiana",      "Iowa",
    "Kansas",         "Kentucky",     "Louisiana",    "Maine",        "Maryland",
    "Massachusetts",  "Michigan",     "Minnesota",    "Mississippi",  "Missouri",
    "Montana",        "Nebraska",     "Nevada",       "New Hampshire","New Jersey",
    "New Mexico",     "New York",     "North Carolina","North Dakota", "Ohio",
    "Oklahoma",       "Oregon",       "Pennsylvania", "Rhode Island", "South Carolina",
    "South Dakota",   "Tennessee",    "Texas",        "Utah",         "Vermont",
    "Virginia",       "Washington",   "West Virginia","Wisconsin",    "Wyoming",
};

static const char *follow_state_display_name() {
    if (s_follow_county_state >= 0 && s_follow_county_state < 50) {
        return kUsStateNames[s_follow_county_state];
    }
    return "--";
}

static void refresh_follow_sched_col() {
    if (!follow_sched_lbl) {
        return;
    }
    if (!s_follow) {
        lv_label_set_text(follow_sched_lbl, "");
        return;
    }
    const FlightInfo *fi = &s_follow_fi;
    char dep_tg[28], arr_tg[28];
    format_term_gate_line(dep_tg, sizeof(dep_tg), fi->dep_terminal, fi->dep_gate);
    format_term_gate_line(arr_tg, sizeof(arr_tg), fi->arr_terminal, fi->arr_gate);
    const char *dep_icao = s_follow_dep.icao[0] ? s_follow_dep.icao : (fi->origin[0] ? fi->origin : "--");
    const char *arr_icao = s_follow_arr.icao[0] ? s_follow_arr.icao : (fi->dest[0] ? fi->dest : "--");
    const bool have = fi->schedule_ok || fi->origin[0] || fi->dest[0] || s_follow_dep.icao[0] ||
                      s_follow_arr.icao[0] || fi->dep_est[0] || fi->arr_est[0] || fi->dep_act[0] ||
                      fi->arr_act[0] || fi->dep_gate[0] || fi->arr_gate[0] || fi->dep_terminal[0] ||
                      fi->arr_terminal[0];
    if (!have) {
        lv_label_set_recolor(follow_sched_lbl, false);
        lv_label_set_text(follow_sched_lbl, "FlightStats\npending...");
        return;
    }
    char rem[28];
    format_follow_remaining(rem, sizeof(rem));
    const char *st_name = follow_state_display_name();
    // DEP/ARR lines use LVGL recolor (bright green) so they read as bold/emphasis.
    // Flight time lives in the left info box.
    char body[520];
    if (fi->flight_date[0]) {
        snprintf(body, sizeof(body),
                 "%s\n"
                 "#27E36A DEP %s#\n"
                 "%s\n"
                 "Sch %s\n"
                 "Act %s\n"
                 "State: %s\n"
                 "\n"
                 "#27E36A ARR %s#\n"
                 "%s\n"
                 "Sch %s\n"
                 "Est %s\n"
                 "Rem %s",
                 fi->flight_date, dep_icao, dep_tg, fi->dep_time[0] ? fi->dep_time : "--",
                 fi->dep_act[0] ? fi->dep_act : "--", st_name, arr_icao, arr_tg,
                 fi->arr_time[0] ? fi->arr_time : "--", fi->arr_est[0] ? fi->arr_est : "--", rem);
    } else {
        snprintf(body, sizeof(body),
                 "#27E36A DEP %s#\n"
                 "%s\n"
                 "Sch %s\n"
                 "Act %s\n"
                 "State: %s\n"
                 "\n"
                 "#27E36A ARR %s#\n"
                 "%s\n"
                 "Sch %s\n"
                 "Est %s\n"
                 "Rem %s",
                 dep_icao, dep_tg, fi->dep_time[0] ? fi->dep_time : "--", fi->dep_act[0] ? fi->dep_act : "--",
                 st_name, arr_icao, arr_tg, fi->arr_time[0] ? fi->arr_time : "--",
                 fi->arr_est[0] ? fi->arr_est : "--", rem);
    }
    lv_label_set_recolor(follow_sched_lbl, true);
    lv_label_set_text(follow_sched_lbl, body);
}

static void format_follow_airport(char *out, size_t n, const char *icao, const char *name) {
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    // Prefer short ICAO so Dep/Arr always fit; append a short name when short enough.
    if (icao && icao[0] && name && name[0] && strlen(name) <= 18) {
        snprintf(out, n, "%s · %s", icao, name);
    } else if (icao && icao[0]) {
        snprintf(out, n, "%s", icao);
    } else if (name && name[0]) {
        snprintf(out, n, "%s", name);
    } else {
        snprintf(out, n, "--");
    }
}

// Merge incoming flight-info without wiping airports / schedule already on screen.
static void merge_follow_fi(const FlightInfo &fi) {
    FlightInfo keep = s_follow_fi;
    s_follow_fi = fi;
    if (!s_follow_fi.origin[0] && keep.origin[0]) {
        strncpy(s_follow_fi.origin, keep.origin, sizeof(s_follow_fi.origin) - 1);
        strncpy(s_follow_fi.origin_name, keep.origin_name, sizeof(s_follow_fi.origin_name) - 1);
        if (!s_follow_fi.dep_gate[0]) {
            strncpy(s_follow_fi.dep_gate, keep.dep_gate, sizeof(s_follow_fi.dep_gate) - 1);
        }
        if (!s_follow_fi.dep_terminal[0]) {
            strncpy(s_follow_fi.dep_terminal, keep.dep_terminal, sizeof(s_follow_fi.dep_terminal) - 1);
        }
        if (!s_follow_fi.dep_time[0]) {
            strncpy(s_follow_fi.dep_time, keep.dep_time, sizeof(s_follow_fi.dep_time) - 1);
        }
        if (!s_follow_fi.dep_act[0]) {
            strncpy(s_follow_fi.dep_act, keep.dep_act, sizeof(s_follow_fi.dep_act) - 1);
        }
        if (!s_follow_fi.dep_est[0]) {
            strncpy(s_follow_fi.dep_est, keep.dep_est, sizeof(s_follow_fi.dep_est) - 1);
        }
        if (!s_follow_fi.dep_act_epoch && keep.dep_act_epoch) {
            s_follow_fi.dep_act_epoch = keep.dep_act_epoch;
        }
    }
    if (!s_follow_fi.dest[0] && keep.dest[0]) {
        strncpy(s_follow_fi.dest, keep.dest, sizeof(s_follow_fi.dest) - 1);
        strncpy(s_follow_fi.dest_name, keep.dest_name, sizeof(s_follow_fi.dest_name) - 1);
        if (!s_follow_fi.arr_gate[0]) {
            strncpy(s_follow_fi.arr_gate, keep.arr_gate, sizeof(s_follow_fi.arr_gate) - 1);
        }
        if (!s_follow_fi.arr_terminal[0]) {
            strncpy(s_follow_fi.arr_terminal, keep.arr_terminal, sizeof(s_follow_fi.arr_terminal) - 1);
        }
        if (!s_follow_fi.arr_time[0]) {
            strncpy(s_follow_fi.arr_time, keep.arr_time, sizeof(s_follow_fi.arr_time) - 1);
        }
        if (!s_follow_fi.arr_act[0]) {
            strncpy(s_follow_fi.arr_act, keep.arr_act, sizeof(s_follow_fi.arr_act) - 1);
        }
        if (!s_follow_fi.arr_est[0]) {
            strncpy(s_follow_fi.arr_est, keep.arr_est, sizeof(s_follow_fi.arr_est) - 1);
        }
        if (!s_follow_fi.arr_act_epoch && keep.arr_act_epoch) {
            s_follow_fi.arr_act_epoch = keep.arr_act_epoch;
        }
    }
    if (!s_follow_fi.dep_act_epoch && keep.dep_act_epoch) {
        s_follow_fi.dep_act_epoch = keep.dep_act_epoch;
    }
    if (!s_follow_fi.arr_act_epoch && keep.arr_act_epoch) {
        s_follow_fi.arr_act_epoch = keep.arr_act_epoch;
    }
    if (!s_follow_fi.arr_est_epoch && keep.arr_est_epoch) {
        s_follow_fi.arr_est_epoch = keep.arr_est_epoch;
    }
    if (s_follow_fi.origin[0] && s_follow_fi.dest[0]) {
        s_follow_fi.route_ok = true;
    }
    if (!s_follow_fi.flight_date[0] && keep.flight_date[0]) {
        strncpy(s_follow_fi.flight_date, keep.flight_date, sizeof(s_follow_fi.flight_date) - 1);
    }
    if (!s_follow_fi.schedule_ok && keep.schedule_ok) {
        s_follow_fi.schedule_ok = true;
    }
    if (!s_follow_fi.status[0] && keep.status[0]) {
        strncpy(s_follow_fi.status, keep.status, sizeof(s_follow_fi.status) - 1);
    }
    if (!fi.departed && keep.departed) {
        s_follow_fi.departed = true;
    }
    // Keep previously fetched filed waypoints if this update has none.
    if (s_follow_fi.route_wpt_n <= 0 && keep.route_wpt_n > 0) {
        memcpy(s_follow_fi.route_wpts, keep.route_wpts, sizeof(s_follow_fi.route_wpts));
        s_follow_fi.route_wpt_n = keep.route_wpt_n;
    }
    follow_apt_update_from_fi(&s_follow_fi);
}

/** Live phase for a tracked flight from ADS-B (climb / cruise / descend / land). */
static void follow_live_status(const Aircraft *ac, const FlightInfo *fi, char *out, size_t n) {
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    if (!ac) {
        return;
    }
    // Field elevation for AGL: prefer arrival, else departure.
    int elev = 0;
    if (s_follow_arr.ok) {
        elev = s_follow_arr.elev_ft;
    } else if (s_follow_dep.ok) {
        elev = s_follow_dep.elev_ft;
    }
    const float agl = ac->alt_ft - (float)elev;
    if (ac->on_ground || agl < 100.0f) {
        if (fi && !fi->departed && ac->on_ground) {
            // Still at origin — keep FA ground status when useful.
            if (fi->status[0]) {
                const char *src = fi->status;
                size_t j = 0;
                for (size_t i = 0; src[i] && j + 1 < n; i++) {
                    const unsigned char c = (unsigned char)src[i];
                    if (c == 0xC2 && (unsigned char)src[i + 1] == 0xB7) {
                        break;
                    }
                    if (c == 0xB7 || c == '|') {
                        break;
                    }
                    out[j++] = (char)c;
                }
                while (j > 0 && (out[j - 1] == ' ' || out[j - 1] == '-')) {
                    j--;
                }
                out[j] = 0;
            }
            if (!out[0]) {
                snprintf(out, n, "At airport");
            }
        } else {
            snprintf(out, n, "Landed");
        }
        return;
    }
    if (agl < 500.0f) {
        snprintf(out, n, "Landing");
        return;
    }
    const float vs = isnan(ac->vrate_fpm) ? 0.0f : ac->vrate_fpm;
    if (vs > 100.0f) {
        snprintf(out, n, "Climbing");
    } else if (vs < -100.0f) {
        snprintf(out, n, "Descending");
    } else {
        snprintf(out, n, "Cruising");
    }
}

static void refresh_follow_panel() {
    if (!follow_info_lbl) {
        return;
    }
    if (!s_follow) {
        return;
    }
    const FlightInfo *fi = &s_follow_fi;
    follow_apt_update_from_fi(fi);
    char dep_apt[40], arr_apt[40];
    format_follow_airport(dep_apt, sizeof(dep_apt),
                          s_follow_dep.icao[0] ? s_follow_dep.icao : fi->origin,
                          s_follow_dep.name[0] ? s_follow_dep.name : fi->origin_name);
    format_follow_airport(arr_apt, sizeof(arr_apt),
                          s_follow_arr.icao[0] ? s_follow_arr.icao : fi->dest,
                          s_follow_arr.name[0] ? s_follow_arr.name : fi->dest_name);

    char flt[28];
    format_follow_flight_time(flt, sizeof(flt));
    char body[560];
    if (!s_follow_ac_ok) {
        const char *wait_st = fi->status[0] ? fi->status : "";
        if (wait_st[0]) {
            snprintf(body, sizeof(body),
                     "Flight %s\n"
                     "Tail   --\n"
                     "Dep  %s\n"
                     "Arr  %s\n"
                     "Status %s\n"
                     "Waiting for ADS-B update...\n"
                     "Flight time %s",
                     s_follow_label[0] ? s_follow_label : s_follow_hex, dep_apt, arr_apt, wait_st, flt);
        } else {
            snprintf(body, sizeof(body),
                     "Flight %s\n"
                     "Tail   --\n"
                     "Dep  %s\n"
                     "Arr  %s\n"
                     "Waiting for ADS-B update...\n"
                     "Flight time %s",
                     s_follow_label[0] ? s_follow_label : s_follow_hex, dep_apt, arr_apt, flt);
        }
        lv_label_set_text(follow_info_lbl, body);
        refresh_follow_sched_col();
        layout_follow_info_boxes();
        return;
    }

    const Aircraft *ac = &s_follow_ac;
    char alt[24];
    if (ac->on_ground) {
        strncpy(alt, "GND", sizeof(alt) - 1);
        alt[sizeof(alt) - 1] = 0;
    } else {
        snprintf(alt, sizeof(alt), "%.0f ft", ac->alt_ft);
    }
    const char *reg = (fi->reg[0] ? fi->reg : (ac->reg[0] ? ac->reg : "--"));
    const char *typ = "--";
    char type_buf[56];
    if (fi->type_name[0]) {
        typ = fi->type_name;
    } else if (ac->type[0]) {
        typ = ac->type;
    }
    if (fi->manufacturer[0] && (fi->type_name[0] || ac->type[0])) {
        snprintf(type_buf, sizeof(type_buf), "%s (%s)", fi->type_name[0] ? fi->type_name : ac->type,
                 fi->manufacturer);
        typ = type_buf;
    }
    const char *flight = ac->flight[0] ? ac->flight : (s_follow_label[0] ? s_follow_label : ac->hex);
    char status_buf[40];
    follow_live_status(ac, fi, status_buf, sizeof(status_buf));
    const char *status = status_buf;
    // Flight / Tail / Dep / Arr first so airports stay visible even if lower lines clip.
    if (!fi->departed && fi->schedule_ok && status[0] && ac->on_ground) {
        snprintf(body, sizeof(body),
                 "Flight %s\n"
                 "Tail   %s\n"
                 "Dep  %s\n"
                 "Arr  %s\n"
                 "%s\n"
                 "Status %s\n"
                 "Alt    %s     Spd  %.0f kt\n"
                 "Pos    %.4f, %.4f\n"
                 "Flight time %s",
                 flight, reg, dep_apt, arr_apt, typ, status, alt, ac->gs_kts, ac->lat, ac->lon, flt);
    } else {
        snprintf(body, sizeof(body),
                 "Flight %s\n"
                 "Tail   %s\n"
                 "Dep  %s\n"
                 "Arr  %s\n"
                 "%s\n"
                 "%s%s%s"
                 "Alt    %s     Spd  %.0f kt\n"
                 "Track  %.0f°     V/S  %+.0f fpm\n"
                 "Squawk %s\n"
                 "Flight time %s",
                 flight, reg, dep_apt, arr_apt, typ, status[0] ? "Status " : "", status,
                 status[0] ? "\n" : "", alt, ac->gs_kts, ac->track_deg, ac->vrate_fpm,
                 ac->squawk[0] ? ac->squawk : "--", flt);
    }
    lv_label_set_text(follow_info_lbl, body);
    refresh_follow_sched_col();
    layout_follow_info_boxes();
}

static void build_radar() {
    scr_radar = lv_obj_create(nullptr);
    style_screen(scr_radar);

    radar_plot = lv_obj_create(scr_radar);
    lv_obj_set_size(radar_plot, kRadarPlot, kRadarPlot);
    lv_obj_align(radar_plot, LV_ALIGN_LEFT_MID, 4, 0);
    // Opaque fill avoids parent punch-through during PPI redraw.
    lv_obj_set_style_bg_color(radar_plot, COL_BG, 0);
    lv_obj_set_style_bg_opa(radar_plot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_opa(radar_plot, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(radar_plot, 0, 0);
    lv_obj_set_style_pad_all(radar_plot, 0, 0);
    lv_obj_clear_flag(radar_plot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(radar_plot, radar_draw, LV_EVENT_DRAW_MAIN, nullptr);
    lv_obj_add_event_cb(radar_plot, radar_press, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(radar_plot, radar_pressing, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(radar_plot, radar_clicked, LV_EVENT_CLICKED, nullptr);

    // Title / location / ADS-B status sit in the square corners (outside the circular PPI).
    lv_obj_t *hdr = lv_label_create(scr_radar);
    lv_label_set_text(hdr, "FLIGHT RADAR");
    lv_label_set_long_mode(hdr, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hdr, 100);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(hdr, COL_GREEN, 0);
    lv_obj_align_to(hdr, radar_plot, LV_ALIGN_TOP_LEFT, 10, 8);

    lv_obj_t *ver_lbl = lv_label_create(scr_radar);
    lv_label_set_text(ver_lbl, FR_VERSION_STR);
    lv_obj_set_style_text_font(ver_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(ver_lbl, COL_DIM, 0);
    lv_obj_align_to(ver_lbl, hdr, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    // Upper-right: minimal analog clock in the square corner (outside PPI), place name to its left.
    radar_clock = lv_obj_create(scr_radar);
    lv_obj_set_size(radar_clock, kClockSize, kClockSize);
    lv_obj_set_style_bg_opa(radar_clock, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(radar_clock, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(radar_clock, 0, 0);
    lv_obj_clear_flag(radar_clock, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(radar_clock, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(radar_clock, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(radar_clock, radar_clock_draw, LV_EVENT_DRAW_MAIN, nullptr);

    radar_place_lbl = lv_label_create(scr_radar);
    lv_label_set_text(radar_place_lbl, "Local Radar");
    lv_obj_set_style_text_align(radar_place_lbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(radar_place_lbl, COL_TEXT, 0);
    lv_obj_set_style_text_font(radar_place_lbl, &lv_font_montserrat_16, 0);
    realign_radar_ur_labels();
    refresh_local_time_lbl();

    // Count/status at bottom corners; zoom/FL boxes sit just above them.
    const lv_coord_t radar_val_w = 96;
    const lv_coord_t radar_val_h = 30;

    radar_count_lbl = lv_label_create(scr_radar);
    lv_label_set_long_mode(radar_count_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radar_count_lbl, 200);
    lv_obj_set_style_text_color(radar_count_lbl, COL_AC, 0);
    lv_obj_set_style_text_font(radar_count_lbl, &lv_font_montserrat_12, 0);
    lv_label_set_text(radar_count_lbl, "0 aircraft");
    lv_obj_align_to(radar_count_lbl, radar_plot, LV_ALIGN_BOTTOM_LEFT, 8, -8);

    radar_status = lv_label_create(scr_radar);
    lv_label_set_long_mode(radar_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radar_status, 240);
    lv_obj_set_style_text_align(radar_status, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(radar_status, COL_TEXT, 0);
    lv_obj_set_style_text_font(radar_status, &lv_font_montserrat_12, 0);
    lv_label_set_text(radar_status, "ADS-B");
    lv_obj_align_to(radar_status, radar_plot, LV_ALIGN_BOTTOM_RIGHT, -8, -8);

    auto make_radar_value_box = [&]() {
        lv_obj_t *box = lv_obj_create(scr_radar);
        lv_obj_set_size(box, radar_val_w, radar_val_h);
        lv_obj_set_style_bg_color(box, lv_color_hex(0x0A120C), 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, COL_DIM, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 6, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);
        return box;
    };

    // Zoom (nm) upper-right above ADS-B; altitude upper-left above aircraft count.
    lv_obj_t *range_box = make_radar_value_box();
    radar_range_lbl = lv_label_create(range_box);
    lv_obj_set_style_text_font(radar_range_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radar_range_lbl, COL_GREEN, 0);
    lv_obj_clear_flag(radar_range_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(radar_range_lbl);
    lv_obj_align_to(range_box, radar_status, LV_ALIGN_OUT_TOP_RIGHT, 0, -4);
    lv_obj_move_foreground(range_box);

    lv_obj_t *alt_box = make_radar_value_box();
    radar_alt_lbl = lv_label_create(alt_box);
    lv_obj_set_style_text_font(radar_alt_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radar_alt_lbl, COL_GREEN, 0);
    lv_obj_clear_flag(radar_alt_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(radar_alt_lbl);
    lv_obj_align_to(alt_box, radar_count_lbl, LV_ALIGN_OUT_TOP_LEFT, 0, -4);
    lv_obj_move_foreground(alt_box);

    lv_obj_t *side = lv_obj_create(scr_radar);
    lv_obj_set_size(side, kSideW, kSideH);
    lv_obj_align(side, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(side, COL_PANEL, 0);
    lv_obj_set_style_border_color(side, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(side, 2, 0);
    lv_obj_set_style_radius(side, 10, 0);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(side, 12, 0);
    lv_obj_set_style_pad_row(side, 8, 0);
    // Scrolling steals taps from ZOOM +/- ("sometimes" no click).
    lv_obj_clear_flag(side, LV_OBJ_FLAG_SCROLLABLE);

    // Compact zoom / altitude + apt / settings stack pinned to bottom of side rail.
    const lv_coord_t ctrl_w = 360;
    const lv_coord_t btn_w = 112;  // follow Dep/Arr column width reference
    const lv_coord_t btn_h = 36;
    const lv_coord_t row_h = 40;
    const lv_coord_t row_btn_w = 85;
    const lv_coord_t pad_row = 6;  // half of prior 12 — more room for follow chart
    // 4 control rows (zoom / alt / board / track); gaps between rows only.
    s_controls_h_follow = 4 * row_h + 3 * pad_row;
    s_controls_h_list = s_controls_h_follow;
    // Always reserve follow-row height so Track stays visible without layout jump.
    const lv_coord_t controls_h = s_controls_h_follow;
    const lv_coord_t table_h = kSideH - 24 - controls_h - 8;
    s_follow_table_h = table_h;

    traffic_table = lv_table_create(side);
    lv_obj_set_size(traffic_table, ctrl_w, table_h);
    lv_obj_align(traffic_table, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(traffic_table, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_add_flag(traffic_table, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(traffic_table, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(traffic_table, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_bg_color(traffic_table, lv_color_hex(0x0A120C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(traffic_table, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(traffic_table, COL_DIM, LV_PART_MAIN);
    lv_obj_set_style_border_width(traffic_table, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(traffic_table, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(traffic_table, 0, LV_PART_MAIN);
    lv_obj_set_style_text_font(traffic_table, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_set_style_text_color(traffic_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(traffic_table, lv_color_hex(0x0A120C), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(traffic_table, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_border_color(traffic_table, COL_DIM, LV_PART_ITEMS);
    lv_obj_set_style_border_width(traffic_table, 1, LV_PART_ITEMS);
    lv_obj_set_style_pad_top(traffic_table, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(traffic_table, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(traffic_table, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(traffic_table, 2, LV_PART_ITEMS);
    lv_table_set_col_cnt(traffic_table, 6);
    lv_table_set_row_cnt(traffic_table, 1);
    // Tail / Flight / Dist / Alt / Spd / Type — ALT wider; SPD shifted right (narrower).
    lv_table_set_col_width(traffic_table, 0, 76);
    lv_table_set_col_width(traffic_table, 1, 76);
    lv_table_set_col_width(traffic_table, 2, 52);
    lv_table_set_col_width(traffic_table, 3, 72);
    lv_table_set_col_width(traffic_table, 4, 42);
    lv_table_set_col_width(traffic_table, 5, 50);
    update_traffic_headers();
    lv_obj_add_event_cb(traffic_table, traffic_table_selected, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(traffic_table, traffic_table_draw_part, LV_EVENT_DRAW_PART_BEGIN, nullptr);

    // Follow mode: info + Dep/Arr side-by-side (content height) | chart below.
    // Hide Wpts / Center live in follow_btn_row; Update ADS-B next to Home; Settings in btn_row.
    const lv_coord_t follow_info_h = 120;  // initial; refresh_follow_panel sizes to content
    const lv_coord_t follow_col_w = btn_w + 16;
    const lv_coord_t follow_info_w = ctrl_w - follow_col_w - 6;
    const lv_coord_t follow_chart_h = table_h - follow_info_h - 8;
    follow_panel = lv_obj_create(side);
    lv_obj_set_size(follow_panel, ctrl_w, table_h);
    lv_obj_align(follow_panel, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(follow_panel, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_add_flag(follow_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(follow_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(follow_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(follow_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(follow_panel, 0, 0);

    follow_info_lbl = lv_label_create(follow_panel);
    lv_obj_set_size(follow_info_lbl, follow_info_w, follow_info_h);
    lv_obj_align(follow_info_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(follow_info_lbl, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_bg_opa(follow_info_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(follow_info_lbl, COL_DIM, 0);
    lv_obj_set_style_border_width(follow_info_lbl, 1, 0);
    lv_obj_set_style_radius(follow_info_lbl, 6, 0);
    lv_obj_set_style_pad_all(follow_info_lbl, 8, 0);
    lv_obj_set_style_text_font(follow_info_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(follow_info_lbl, COL_TEXT, 0);
    lv_label_set_long_mode(follow_info_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(follow_info_lbl, "");

    follow_sched_lbl = lv_label_create(follow_panel);
    lv_obj_set_size(follow_sched_lbl, follow_col_w, follow_info_h);
    lv_obj_align(follow_sched_lbl, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_style_bg_color(follow_sched_lbl, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_bg_opa(follow_sched_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(follow_sched_lbl, COL_DIM, 0);
    lv_obj_set_style_border_width(follow_sched_lbl, 1, 0);
    lv_obj_set_style_radius(follow_sched_lbl, 6, 0);
    lv_obj_set_style_pad_left(follow_sched_lbl, 6, 0);
    lv_obj_set_style_pad_right(follow_sched_lbl, 4, 0);
    lv_obj_set_style_pad_top(follow_sched_lbl, 4, 0);
    lv_obj_set_style_pad_bottom(follow_sched_lbl, 4, 0);
    lv_obj_set_style_text_font(follow_sched_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(follow_sched_lbl, COL_TEXT, 0);
    lv_obj_set_style_text_line_space(follow_sched_lbl, -2, 0);
    lv_label_set_long_mode(follow_sched_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(follow_sched_lbl, "");

    follow_chart = lv_obj_create(follow_panel);
    lv_obj_set_size(follow_chart, ctrl_w, follow_chart_h);
    lv_obj_align(follow_chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(follow_chart, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_bg_opa(follow_chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(follow_chart, COL_DIM, 0);
    lv_obj_set_style_border_width(follow_chart, 1, 0);
    lv_obj_set_style_radius(follow_chart, 6, 0);
    lv_obj_set_style_pad_all(follow_chart, 0, 0);
    lv_obj_clear_flag(follow_chart, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(follow_chart, follow_chart_draw, LV_EVENT_DRAW_MAIN, nullptr);

    side_controls = lv_obj_create(side);
    lv_obj_t *controls = side_controls;
    lv_obj_set_size(controls, ctrl_w, controls_h);
    lv_obj_set_style_bg_opa(controls, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(controls, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(controls, 0, 0);
    lv_obj_set_style_pad_row(controls, pad_row, 0);
    lv_obj_set_flex_flow(controls, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(controls, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(controls, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(controls, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_align(controls, LV_ALIGN_BOTTOM_MID, 0, -4);

    auto make_ctrl_row = [&](lv_obj_t *parent) {
        lv_obj_t *row = lv_obj_create(parent);
        lv_obj_set_size(row, ctrl_w, row_h);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        return row;
    };
    auto shrink_btn_label = [](lv_obj_t *btn) {
        lv_obj_t *lab = lv_obj_get_child(btn, 0);
        if (lab) {
            lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
        }
    };
    auto make_press_ctrl = [&](lv_obj_t *parent, const char *txt, lv_event_cb_t cb) {
        lv_obj_t *btn = make_btn(parent, txt, cb, nullptr);
        lv_obj_set_size(btn, row_btn_w, btn_h);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_remove_event_cb(btn, cb);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, nullptr);
        shrink_btn_label(btn);
        return btn;
    };
    // Zoom In | Zoom Out | Home | Hide Airport
    lv_obj_t *zoom_row = make_ctrl_row(controls);
    lv_obj_t *zin = make_press_ctrl(zoom_row, "Zoom In", zoom_in);
    lv_obj_align(zin, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *zout = make_press_ctrl(zoom_row, "Zoom Out", zoom_out);
    lv_obj_align(zout, LV_ALIGN_LEFT_MID, +(row_btn_w + 6), 0);
    lv_obj_t *home_btn = make_btn(zoom_row, "Home", home_clicked, nullptr);
    lv_obj_set_size(home_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(home_btn, 6, 0);
    lv_obj_clear_flag(home_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(home_btn);
    lv_obj_align(home_btn, LV_ALIGN_LEFT_MID, +2 * (row_btn_w + 6), 0);
    apt_toggle_btn = make_btn(zoom_row, "Hide\nAirport", toggle_airports, nullptr);
    lv_obj_set_size(apt_toggle_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(apt_toggle_btn, 6, 0);
    lv_obj_clear_flag(apt_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(apt_toggle_btn, row_btn_w);
    lv_obj_align(apt_toggle_btn, LV_ALIGN_LEFT_MID, +3 * (row_btn_w + 6), 0);
    update_apt_toggle_label();

    // ALT + | ALT − | Update ADS-B | Show City
    lv_obj_t *alt_row = make_ctrl_row(controls);
    lv_obj_t *aup = make_press_ctrl(alt_row, "ALT +", alt_up);
    lv_obj_align(aup, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *adn = make_press_ctrl(alt_row, "ALT -", alt_down);
    lv_obj_align(adn, LV_ALIGN_LEFT_MID, +(row_btn_w + 6), 0);
    follow_list_btn = make_btn(alt_row, "Update\nADS-B", update_adsb_clicked, nullptr);
    lv_obj_set_size(follow_list_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(follow_list_btn, 6, 0);
    lv_obj_clear_flag(follow_list_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(follow_list_btn, row_btn_w);
    lv_obj_align(follow_list_btn, LV_ALIGN_LEFT_MID, +2 * (row_btn_w + 6), 0);
    city_toggle_btn = make_btn(alt_row, "Hide\nCity", toggle_cities, nullptr);
    lv_obj_set_size(city_toggle_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(city_toggle_btn, 6, 0);
    lv_obj_clear_flag(city_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(city_toggle_btn, row_btn_w);
    lv_obj_align(city_toggle_btn, LV_ALIGN_LEFT_MID, +3 * (row_btn_w + 6), 0);
    update_city_toggle_label();

    // Board | Dep/Arr | (gap) | Hide County
    lv_obj_t *btn_row = make_ctrl_row(controls);
    lv_obj_t *board_btn = make_btn(btn_row, "Airport\nBoard", open_airport_board, nullptr);
    lv_obj_set_size(board_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(board_btn, 6, 0);
    lv_obj_clear_flag(board_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(board_btn, row_btn_w);
    lv_obj_align(board_btn, LV_ALIGN_RIGHT_MID, -3 * (row_btn_w + 6), 0);
    lv_obj_t *route_btn = make_btn(btn_row, "Dept/Arrive\nSearch", open_route_search, nullptr);
    lv_obj_set_size(route_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(route_btn, 6, 0);
    lv_obj_clear_flag(route_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(route_btn, row_btn_w);
    lv_obj_align(route_btn, LV_ALIGN_RIGHT_MID, -2 * (row_btn_w + 6), 0);
    cty_toggle_btn = make_btn(btn_row, "Hide\nCounty", toggle_counties, nullptr);
    lv_obj_set_size(cty_toggle_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(cty_toggle_btn, 6, 0);
    lv_obj_clear_flag(cty_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    style_two_line_btn_label(cty_toggle_btn, row_btn_w);
    lv_obj_align(cty_toggle_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    update_cty_toggle_label();

    // Track | Wpt/Path | Center | Settings (Settings moved down one)
    follow_btn_row = make_ctrl_row(controls);
    lv_obj_t *track_btn = make_btn(follow_btn_row, "Track Flight", open_track_flight, nullptr);
    lv_obj_set_size(track_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(track_btn, 6, 0);
    lv_obj_clear_flag(track_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(track_btn);
    lv_obj_align(track_btn, LV_ALIGN_RIGHT_MID, -3 * (row_btn_w + 6), 0);

    wpt_toggle_btn = make_btn(follow_btn_row, "Hide Wpts", toggle_wpt_labels, nullptr);
    lv_obj_set_size(wpt_toggle_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(wpt_toggle_btn, 6, 0);
    lv_obj_clear_flag(wpt_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(wpt_toggle_btn);
    lv_obj_align(wpt_toggle_btn, LV_ALIGN_RIGHT_MID, -2 * (row_btn_w + 6), 0);
    lv_obj_add_flag(wpt_toggle_btn, LV_OBJ_FLAG_HIDDEN);

    follow_center_btn = make_btn(follow_btn_row, "Center", center_on_flight, nullptr);
    lv_obj_set_size(follow_center_btn, row_btn_w, btn_h);
    lv_obj_set_style_radius(follow_center_btn, 6, 0);
    lv_obj_clear_flag(follow_center_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(follow_center_btn);
    lv_obj_align(follow_center_btn, LV_ALIGN_RIGHT_MID, -(row_btn_w + 6), 0);
    lv_obj_add_flag(follow_center_btn, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *settings = make_btn(follow_btn_row, "Settings", open_settings, nullptr);
    lv_obj_set_size(settings, row_btn_w, btn_h);
    lv_obj_set_style_radius(settings, 6, 0);
    lv_obj_clear_flag(settings, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(settings);
    lv_obj_align(settings, LV_ALIGN_RIGHT_MID, 0, 0);

    settings_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(settings_panel, 360, 340);
    lv_obj_center(settings_panel);
    lv_obj_set_style_bg_color(settings_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(settings_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(settings_panel, 12, 0);
    lv_obj_set_style_pad_all(settings_panel, 8, 0);
    lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *stitle = lv_label_create(settings_panel);
    lv_label_set_text(stitle, "Settings");
    lv_obj_set_style_text_font(stitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(stitle, COL_GREEN, 0);
    lv_obj_align(stitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *sclose = make_btn(settings_panel, "Close", hide_settings, nullptr);
    lv_obj_set_size(sclose, 90, 36);
    lv_obj_align(sclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *sloc = make_btn(settings_panel, "Startup Settings", goto_loc, nullptr);
    lv_obj_set_size(sloc, 320, 44);
    lv_obj_align(sloc, LV_ALIGN_TOP_MID, 0, 52);

    lv_obj_t *swifi = make_btn(settings_panel, "Wi-Fi Setup", goto_wifi, nullptr);
    lv_obj_set_size(swifi, 320, 44);
    lv_obj_align(swifi, LV_ALIGN_TOP_MID, 0, 104);

    lv_obj_t *ssound = make_btn(settings_panel, "Display & Sound", goto_sound, nullptr);
    lv_obj_set_size(ssound, 320, 44);
    lv_obj_align(ssound, LV_ALIGN_TOP_MID, 0, 156);

    lv_obj_t *scounty = make_btn(settings_panel, "County Borders", goto_counties, nullptr);
    lv_obj_set_size(scounty, 320, 44);
    lv_obj_align(scounty, LV_ALIGN_TOP_MID, 0, 208);

    lv_obj_t *splates = make_btn(settings_panel, "Airport Plates", goto_plates, nullptr);
    lv_obj_set_size(splates, 320, 44);
    lv_obj_align(splates, LV_ALIGN_TOP_MID, 0, 260);

    // Airport-plate browser: add/filter list + live preview + update.
    plates_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(plates_panel, 1000, 560);
    lv_obj_center(plates_panel);
    lv_obj_set_style_bg_color(plates_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(plates_panel, COL_GREEN, 0);
    lv_obj_set_style_border_width(plates_panel, 2, 0);
    lv_obj_set_style_radius(plates_panel, 12, 0);
    lv_obj_set_style_pad_all(plates_panel, 8, 0);
    lv_obj_clear_flag(plates_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(plates_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *ptitle = lv_label_create(plates_panel);
    lv_label_set_text(ptitle, "Airport Plates");
    lv_obj_set_style_text_font(ptitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ptitle, COL_GREEN, 0);
    lv_obj_align(ptitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *pclose = make_btn(plates_panel, "Close", hide_plates_panel, nullptr);
    lv_obj_set_size(pclose, 90, 36);
    lv_obj_align(pclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    // Left: filter above plates list. Center: chart. Right: Remove / Add.
    constexpr lv_coord_t kPlatesListW = 340;
    constexpr lv_coord_t kPlatesListX = 8;
    constexpr lv_coord_t kPlatesFiltY = 40;
    constexpr lv_coord_t kPlatesFiltH = 36;
    constexpr lv_coord_t kPlatesListY = kPlatesFiltY + kPlatesFiltH + 8;
    constexpr lv_coord_t kPlatesListH = 448;
    // Preview diagram; runway lengths + freqs in table below; city status under that.
    constexpr lv_coord_t kPlatesPrevW = 280;
    constexpr lv_coord_t kPlatesPrevX = kPlatesListX + kPlatesListW + 10; // 358
    constexpr lv_coord_t kPlatesPrevY = 72;
    constexpr lv_coord_t kPlatesInfoH = 130;
    constexpr lv_coord_t kPlatesInfoY = kPlatesPrevY + kPlatesPrevW + 6;
    constexpr lv_coord_t kPlatesCityY = kPlatesInfoY + kPlatesInfoH + 4;
    constexpr lv_coord_t kPlatesUpdX = kPlatesPrevX + kPlatesPrevW + 10; // 648
    constexpr lv_coord_t kPlatesUpdW = 270;
    constexpr lv_coord_t kPlatesUpdH = 40;
    constexpr lv_coord_t kPlatesRemY = kPlatesPrevY;
    constexpr lv_coord_t kPlatesAddH = 36;
    constexpr lv_coord_t kPlatesAddTaY = kPlatesRemY + kPlatesUpdH + 8;
    constexpr lv_coord_t kPlatesAddBtnY = kPlatesAddTaY + kPlatesAddH + 8;
    constexpr lv_coord_t kPlatesAddHitsY = kPlatesAddBtnY + kPlatesAddH + 8;
    constexpr lv_coord_t kPlatesAddHitsH = 100;
    constexpr lv_coord_t kPlatesStatusY = kPlatesAddHitsY + kPlatesAddHitsH + 8;

    // Filter search above the left plates list.
    plates_ta = lv_textarea_create(plates_panel);
    lv_obj_set_size(plates_ta, kPlatesListW, kPlatesFiltH);
    lv_obj_align(plates_ta, LV_ALIGN_TOP_LEFT, kPlatesListX, kPlatesFiltY);
    lv_textarea_set_one_line(plates_ta, true);
    lv_textarea_set_placeholder_text(plates_ta, "Filter list");
    lv_textarea_set_max_length(plates_ta, 40);
    lv_obj_add_event_cb(plates_ta, plates_search_changed, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(plates_ta, plates_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(plates_ta, plates_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    plates_list = lv_list_create(plates_panel);
    lv_obj_set_size(plates_list, kPlatesListW, kPlatesListH);
    lv_obj_align(plates_list, LV_ALIGN_TOP_LEFT, kPlatesListX, kPlatesListY);
    lv_obj_set_style_bg_color(plates_list, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_border_color(plates_list, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(plates_list, 1, 0);
    lv_obj_set_style_pad_row(plates_list, 2, 0);

    plates_title_lbl = lv_label_create(plates_panel);
    lv_label_set_text(plates_title_lbl, "Select an airport");
    lv_obj_set_style_text_font(plates_title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(plates_title_lbl, COL_TEXT, 0);
    lv_obj_set_width(plates_title_lbl, kPlatesPrevW);
    lv_label_set_long_mode(plates_title_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_align(plates_title_lbl, LV_ALIGN_TOP_LEFT, kPlatesPrevX, kPlatesFiltY);

    plates_preview = lv_obj_create(plates_panel);
    lv_obj_set_size(plates_preview, kPlatesPrevW, kPlatesPrevW);
    lv_obj_align(plates_preview, LV_ALIGN_TOP_LEFT, kPlatesPrevX, kPlatesPrevY);
    lv_obj_set_style_bg_color(plates_preview, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_border_color(plates_preview, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(plates_preview, 1, 0);
    lv_obj_set_style_pad_all(plates_preview, 0, 0);
    lv_obj_clear_flag(plates_preview, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(plates_preview, plates_preview_draw, LV_EVENT_DRAW_MAIN, nullptr);

    plates_info_tbl = lv_table_create(plates_panel);
    lv_obj_set_size(plates_info_tbl, kPlatesPrevW, kPlatesInfoH);
    lv_obj_align(plates_info_tbl, LV_ALIGN_TOP_LEFT, kPlatesPrevX, kPlatesInfoY);
    lv_obj_add_flag(plates_info_tbl, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(plates_info_tbl, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(plates_info_tbl, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_bg_color(plates_info_tbl, lv_color_hex(0x0A120C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(plates_info_tbl, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(plates_info_tbl, COL_BTN_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(plates_info_tbl, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(plates_info_tbl, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(plates_info_tbl, 0, LV_PART_MAIN);
    lv_obj_set_style_text_font(plates_info_tbl, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_set_style_text_color(plates_info_tbl, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(plates_info_tbl, lv_color_hex(0x0A120C), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(plates_info_tbl, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_border_color(plates_info_tbl, COL_DIM, LV_PART_ITEMS);
    lv_obj_set_style_border_width(plates_info_tbl, 1, LV_PART_ITEMS);
    lv_obj_set_style_pad_top(plates_info_tbl, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(plates_info_tbl, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(plates_info_tbl, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(plates_info_tbl, 4, LV_PART_ITEMS);
    lv_table_set_col_cnt(plates_info_tbl, 2);
    lv_table_set_col_width(plates_info_tbl, 0, 150);
    lv_table_set_col_width(plates_info_tbl, 1, 120);
    lv_table_set_row_cnt(plates_info_tbl, 1);
    lv_table_set_cell_value(plates_info_tbl, 0, 0, "RWY / FREQ");
    lv_table_set_cell_value(plates_info_tbl, 0, 1, "VALUE");

    plates_city_lbl = lv_label_create(plates_panel);
    lv_label_set_text(plates_city_lbl, "");
    lv_obj_set_style_text_font(plates_city_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(plates_city_lbl, COL_GREEN, 0);
    lv_obj_set_width(plates_city_lbl, kPlatesPrevW);
    lv_label_set_long_mode(plates_city_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_align(plates_city_lbl, LV_ALIGN_TOP_LEFT, kPlatesPrevX, kPlatesCityY);

    lv_obj_t *prem = make_btn(plates_panel, "Remove Plate", plates_remove_clicked, nullptr);
    lv_obj_set_size(prem, kPlatesUpdW, kPlatesUpdH);
    lv_obj_align(prem, LV_ALIGN_TOP_LEFT, kPlatesUpdX, kPlatesRemY);

    // Add airport search below Remove Plate (right column).
    plates_add_ta = lv_textarea_create(plates_panel);
    lv_obj_set_size(plates_add_ta, kPlatesUpdW, kPlatesAddH);
    lv_obj_align(plates_add_ta, LV_ALIGN_TOP_LEFT, kPlatesUpdX, kPlatesAddTaY);
    lv_textarea_set_one_line(plates_add_ta, true);
    lv_textarea_set_placeholder_text(plates_add_ta, "Add ICAO/IATA");
    lv_textarea_set_max_length(plates_add_ta, 40);
    lv_obj_add_event_cb(plates_add_ta, plates_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(plates_add_ta, plates_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *padd = make_btn(plates_panel, "Add", plates_add_search_clicked, nullptr);
    lv_obj_set_size(padd, kPlatesUpdW, kPlatesAddH);
    lv_obj_align(padd, LV_ALIGN_TOP_LEFT, kPlatesUpdX, kPlatesAddBtnY);

    plates_add_hits = lv_list_create(plates_panel);
    lv_obj_set_size(plates_add_hits, kPlatesUpdW, kPlatesAddHitsH);
    lv_obj_align(plates_add_hits, LV_ALIGN_TOP_LEFT, kPlatesUpdX, kPlatesAddHitsY);
    lv_obj_set_style_bg_color(plates_add_hits, lv_color_hex(0x0A120C), 0);
    lv_obj_set_style_border_color(plates_add_hits, COL_BTN_BORDER, 0);
    lv_obj_set_style_border_width(plates_add_hits, 1, 0);
    lv_obj_add_flag(plates_add_hits, LV_OBJ_FLAG_HIDDEN);

    plates_status = lv_label_create(plates_panel);
    lv_label_set_text(plates_status, "");
    lv_obj_set_style_text_color(plates_status, COL_GREEN, 0);
    lv_obj_set_style_text_font(plates_status, &lv_font_montserrat_12, 0);
    lv_obj_set_width(plates_status, kPlatesUpdW);
    lv_label_set_long_mode(plates_status, LV_LABEL_LONG_WRAP);
    lv_obj_align(plates_status, LV_ALIGN_TOP_LEFT, kPlatesUpdX, kPlatesStatusY);

    {
        // Same letter + number pad layout as Track Flight.
        static const char *kPlatesKbMap[] = {
            "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "7", "8", "9", "\n",
            "A", "S", "D", "F", "G", "H", "J", "K", "L", LV_SYMBOL_BACKSPACE, "4", "5", "6", "\n",
            "Z", "X", "C", "V", "B", "N", "M", "0", "1", "2", "3", "\n",
            LV_SYMBOL_CLOSE, " ", LV_SYMBOL_OK, "",
        };
        static const lv_btnmatrix_ctrl_t kPlatesKbCtrl[] = {
            3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4,
            3, 3, 3, 3, 3, 3, 3, 3, 3, 5, 4, 4, 4,
            3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
            LV_KEYBOARD_CTRL_BTN_FLAGS | 5, 12, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
        };
        plates_kb = lv_keyboard_create(scr_radar);
        lv_obj_set_size(plates_kb, 800, 240);
        lv_obj_align(plates_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_keyboard_set_map(plates_kb, LV_KEYBOARD_MODE_USER_1, kPlatesKbMap, kPlatesKbCtrl);
        lv_keyboard_set_mode(plates_kb, LV_KEYBOARD_MODE_USER_1);
        lv_keyboard_set_textarea(plates_kb, plates_ta);
        lv_obj_add_event_cb(plates_kb, plates_kb_event, LV_EVENT_READY, nullptr);
        lv_obj_add_event_cb(plates_kb, plates_kb_event, LV_EVENT_CANCEL, nullptr);
        lv_obj_add_event_cb(plates_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_flag(plates_kb, LV_OBJ_FLAG_HIDDEN);
    }

    config_load_plate_extras(s_plate_extra, s_plate_extra_city, &s_plate_extra_n);

    // 5 rows × 10 columns of state checkboxes (alphabetical abbr).
    county_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(county_panel, 940, 420);
    lv_obj_center(county_panel);
    lv_obj_set_style_bg_color(county_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(county_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(county_panel, 12, 0);
    lv_obj_set_style_pad_all(county_panel, 8, 0);
    lv_obj_clear_flag(county_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *ctitle = lv_label_create(county_panel);
    lv_label_set_text(ctitle, "County Borders");
    lv_obj_set_style_text_font(ctitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ctitle, COL_GREEN, 0);
    lv_obj_align(ctitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *cclose = make_btn(county_panel, "Close", hide_county_panel, nullptr);
    lv_obj_set_size(cclose, 90, 36);
    lv_obj_align(cclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *chint = lv_label_create(county_panel);
    lv_label_set_text(chint, "Select states to outline counties on the map");
    lv_obj_set_style_text_color(chint, COL_DIM, 0);
    lv_obj_set_style_text_font(chint, &lv_font_montserrat_14, 0);
    lv_obj_align(chint, LV_ALIGN_TOP_LEFT, 8, 40);

    county_select_all_cb = lv_checkbox_create(county_panel);
    lv_checkbox_set_text(county_select_all_cb, "Select All");
    lv_obj_set_style_text_color(county_select_all_cb, COL_TEXT, 0);
    lv_obj_set_style_text_font(county_select_all_cb, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(county_select_all_cb, COL_GREEN, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(county_select_all_cb, COL_DIM, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(county_select_all_cb, lv_color_hex(0x1A2A1A), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_align(county_select_all_cb, LV_ALIGN_TOP_RIGHT, -110, 40);
    lv_obj_add_event_cb(county_select_all_cb, county_select_all_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    constexpr lv_coord_t kCbCellW = 90;
    constexpr lv_coord_t kCbCellH = 58;
    constexpr lv_coord_t kCbGridX = 12;
    constexpr lv_coord_t kCbGridY = 72;
    for (int i = 0; i < 50; i++) {
        const int row = i / 10;
        const int col = i % 10;
        lv_obj_t *cb = lv_checkbox_create(county_panel);
        lv_checkbox_set_text(cb, kCountyStateAbbr[i]);
        lv_obj_set_style_text_color(cb, COL_TEXT, 0);
        lv_obj_set_style_text_font(cb, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(cb, COL_GREEN, LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_border_color(cb, COL_DIM, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(cb, lv_color_hex(0x1A2A1A), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_align(cb, LV_ALIGN_TOP_LEFT, kCbGridX + col * kCbCellW, kCbGridY + row * kCbCellH);
        lv_obj_add_event_cb(cb, county_cb_changed, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
        county_cbs[i] = cb;
    }

    sound_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(sound_panel, 420, 280);
    lv_obj_center(sound_panel);
    lv_obj_set_style_bg_color(sound_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(sound_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(sound_panel, 12, 0);
    lv_obj_set_style_pad_all(sound_panel, 8, 0);
    lv_obj_clear_flag(sound_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *dtitle = lv_label_create(sound_panel);
    lv_label_set_text(dtitle, "Display & Sound");
    lv_obj_set_style_text_font(dtitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(dtitle, COL_GREEN, 0);
    lv_obj_align(dtitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *dclose = make_btn(sound_panel, "Close", hide_sound_panel, nullptr);
    lv_obj_set_size(dclose, 90, 36);
    lv_obj_align(dclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *blbl = lv_label_create(sound_panel);
    lv_label_set_text(blbl, "Brightness");
    lv_obj_set_style_text_color(blbl, COL_TEXT, 0);
    lv_obj_set_style_text_font(blbl, &lv_font_montserrat_14, 0);
    lv_obj_align(blbl, LV_ALIGN_TOP_LEFT, 16, 56);

    bright_pct_lbl = lv_label_create(sound_panel);
    lv_label_set_text(bright_pct_lbl, "100%");
    lv_obj_set_style_text_color(bright_pct_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(bright_pct_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(bright_pct_lbl, LV_ALIGN_TOP_RIGHT, -16, 56);

    bright_slider = lv_slider_create(sound_panel);
    lv_obj_set_width(bright_slider, 360);
    lv_slider_set_range(bright_slider, 0, 100);
    lv_slider_set_value(bright_slider, 100, LV_ANIM_OFF);
    lv_obj_align(bright_slider, LV_ALIGN_TOP_MID, 0, 88);
    lv_obj_set_style_bg_color(bright_slider, lv_color_hex(0x2A3A2A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bright_slider, COL_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bright_slider, COL_GREEN, LV_PART_KNOB);
    lv_obj_add_event_cb(bright_slider, bright_slider_changed, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(bright_slider, bright_slider_changed, LV_EVENT_RELEASED, nullptr);

    lv_obj_t *vlbl = lv_label_create(sound_panel);
    lv_label_set_text(vlbl, "Volume");
    lv_obj_set_style_text_color(vlbl, COL_TEXT, 0);
    lv_obj_set_style_text_font(vlbl, &lv_font_montserrat_14, 0);
    lv_obj_align(vlbl, LV_ALIGN_TOP_LEFT, 16, 150);

    vol_pct_lbl = lv_label_create(sound_panel);
    lv_label_set_text(vol_pct_lbl, "60%");
    lv_obj_set_style_text_color(vol_pct_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(vol_pct_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(vol_pct_lbl, LV_ALIGN_TOP_RIGHT, -16, 150);

    vol_slider = lv_slider_create(sound_panel);
    lv_obj_set_width(vol_slider, 360);
    lv_slider_set_range(vol_slider, 0, 100);
    lv_slider_set_value(vol_slider, 60, LV_ANIM_OFF);
    lv_obj_align(vol_slider, LV_ALIGN_TOP_MID, 0, 182);
    lv_obj_set_style_bg_color(vol_slider, lv_color_hex(0x2A3A2A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(vol_slider, COL_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(vol_slider, COL_GREEN, LV_PART_KNOB);
    lv_obj_add_event_cb(vol_slider, vol_slider_changed, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(vol_slider, vol_slider_changed, LV_EVENT_RELEASED, nullptr);

    wifi_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(wifi_panel, 760, 440);
    lv_obj_center(wifi_panel);
    lv_obj_set_style_bg_color(wifi_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(wifi_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(wifi_panel, 12, 0);
    lv_obj_set_style_pad_all(wifi_panel, 8, 0);
    lv_obj_clear_flag(wifi_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *wtitle = lv_label_create(wifi_panel);
    lv_label_set_text(wtitle, "Wi-Fi Setup");
    lv_obj_set_style_text_font(wtitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(wtitle, COL_GREEN, 0);
    lv_obj_align(wtitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *wclose = make_btn(wifi_panel, "Close", hide_wifi, nullptr);
    lv_obj_set_size(wclose, 90, 36);
    lv_obj_align(wclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    wifi_connected_lbl = lv_label_create(wifi_panel);
    lv_label_set_text(wifi_connected_lbl, "Connected to: —");
    lv_obj_set_style_text_color(wifi_connected_lbl, COL_GREEN, 0);
    lv_obj_set_style_text_font(wifi_connected_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(wifi_connected_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(wifi_connected_lbl, 520);
    lv_obj_align(wifi_connected_lbl, LV_ALIGN_TOP_LEFT, 12, 40);

    lv_obj_t *whint = lv_label_create(wifi_panel);
    lv_label_set_text(whint, "Tap a network, enter the password, then Connect.");
    lv_obj_set_style_text_color(whint, COL_TEXT, 0);
    lv_obj_set_style_text_font(whint, &lv_font_montserrat_12, 0);
    lv_obj_align(whint, LV_ALIGN_TOP_LEFT, 12, 62);

    wifi_list = lv_list_create(wifi_panel);
    lv_obj_set_size(wifi_list, 480, 280);
    lv_obj_align(wifi_list, LV_ALIGN_TOP_LEFT, 12, 84);
    lv_obj_set_style_bg_color(wifi_list, COL_PANEL, 0);
    lv_obj_set_style_border_color(wifi_list, COL_DIM, 0);

    lv_obj_t *scan = make_btn(wifi_panel, "Scan", wifi_scan_clicked, nullptr);
    lv_obj_set_size(scan, 220, 48);
    lv_obj_align(scan, LV_ALIGN_TOP_RIGHT, -12, 72);

    lv_obj_t *conn = make_btn(wifi_panel, "Connect", wifi_connect_clicked, nullptr);
    lv_obj_set_size(conn, 220, 48);
    lv_obj_align(conn, LV_ALIGN_TOP_RIGHT, -12, 132);

    wifi_ta = lv_textarea_create(wifi_panel);
    lv_obj_set_size(wifi_ta, 220, 48);
    lv_obj_align(wifi_ta, LV_ALIGN_TOP_RIGHT, -12, 192);
    lv_textarea_set_placeholder_text(wifi_ta, "Password");
    // Default: password hidden (dots). Check "Show password" to reveal.
    lv_textarea_set_password_mode(wifi_ta, true);
    lv_textarea_set_password_show_time(wifi_ta, 0);  // no plaintext flash while typing
    lv_textarea_set_one_line(wifi_ta, true);

    wifi_show_pw_cb = lv_checkbox_create(wifi_panel);
    lv_checkbox_set_text(wifi_show_pw_cb, "Show password");
    lv_obj_clear_state(wifi_show_pw_cb, LV_STATE_CHECKED);
    lv_obj_set_style_text_color(wifi_show_pw_cb, COL_TEXT, 0);
    lv_obj_set_style_text_font(wifi_show_pw_cb, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(wifi_show_pw_cb, COL_GREEN, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(wifi_show_pw_cb, lv_color_hex(0x1A2A1A), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_align(wifi_show_pw_cb, LV_ALIGN_TOP_RIGHT, -12, 244);
    lv_obj_add_event_cb(wifi_show_pw_cb, wifi_show_pass_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    wifi_status = lv_label_create(wifi_panel);
    lv_label_set_text(wifi_status, "Not connected");
    lv_obj_set_style_text_color(wifi_status, COL_GREEN, 0);
    lv_label_set_long_mode(wifi_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(wifi_status, 220);
    lv_obj_align(wifi_status, LV_ALIGN_TOP_RIGHT, -12, 280);

    wifi_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(wifi_kb, 800, 190);
    lv_obj_align(wifi_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(wifi_kb, wifi_ta);
    lv_obj_add_event_cb(wifi_kb, ui_kb_click_sound, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);

    detail = lv_obj_create(scr_radar);
    lv_obj_set_size(detail, kDetailW, kDetailH);
    lv_obj_center(detail);
    lv_obj_set_style_bg_color(detail, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(detail, COL_GREEN, 0);
    lv_obj_set_style_radius(detail, 12, 0);
    lv_obj_set_style_pad_all(detail, 6, 0);
    lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(detail, LV_OBJ_FLAG_SCROLLABLE);

    detail_title = lv_label_create(detail);
    lv_obj_set_style_text_font(detail_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(detail_title, COL_GREEN, 0);
    lv_obj_set_width(detail_title, kDetailW - kDetailBtnCol - 12);
    lv_label_set_long_mode(detail_title, LV_LABEL_LONG_CLIP);
    lv_obj_align(detail_title, LV_ALIGN_TOP_LEFT, 6, 4);

    // Buttons stacked on the right: Close, Follow, Center.
    lv_obj_t *closeb = make_btn(detail, "Close", hide_detail, nullptr);
    lv_obj_set_size(closeb, kDetailBtnW, 32);
    lv_obj_align(closeb, LV_ALIGN_TOP_RIGHT, -4, 4);
    shrink_btn_label(closeb);

    detail_follow_btn = make_btn(detail, "Follow", follow_flight, nullptr);
    lv_obj_set_size(detail_follow_btn, kDetailBtnW, 32);
    lv_obj_align(detail_follow_btn, LV_ALIGN_TOP_RIGHT, -4, 40);
    shrink_btn_label(detail_follow_btn);
    lv_obj_add_flag(detail_follow_btn, LV_OBJ_FLAG_HIDDEN);

    detail_center_btn = make_btn(detail, "Center", center_on_airport, nullptr);
    lv_obj_set_size(detail_center_btn, kDetailBtnW, 32);
    lv_obj_align(detail_center_btn, LV_ALIGN_TOP_RIGHT, -4, 76);
    shrink_btn_label(detail_center_btn);
    lv_obj_add_flag(detail_center_btn, LV_OBJ_FLAG_HIDDEN);

    detail_add_plate_btn = make_btn(detail, "Add Plate", detail_add_plate_clicked, nullptr);
    lv_obj_set_size(detail_add_plate_btn, kDetailBtnW, 32);
    lv_obj_align(detail_add_plate_btn, LV_ALIGN_TOP_RIGHT, -4, 112);
    shrink_btn_label(detail_add_plate_btn);
    lv_obj_add_flag(detail_add_plate_btn, LV_OBJ_FLAG_HIDDEN);

    detail_plate = lv_obj_create(detail);
    lv_obj_set_size(detail_plate, kDetailPlateW, kDetailPlateH);
    lv_obj_align(detail_plate, LV_ALIGN_BOTTOM_LEFT, 2, -2);
    lv_obj_set_style_bg_opa(detail_plate, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(detail_plate, 0, 0);
    lv_obj_set_style_pad_all(detail_plate, 0, 0);
    lv_obj_clear_flag(detail_plate, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(detail_plate, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(detail_plate, plate_draw, LV_EVENT_DRAW_MAIN, nullptr);

    detail_body = lv_label_create(detail);
    lv_obj_set_style_text_font(detail_body, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(detail_body, COL_TEXT, 0);
    lv_label_set_long_mode(detail_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(detail_body, kDetailW - kDetailBtnCol - 16);
    lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, 8, 40);

    build_location();
    build_track_flight();
    build_airport_board();
    build_route_search();

    if (!s_clock_timer) {
        s_clock_timer = lv_timer_create(clock_timer_cb, 1000, nullptr);
    }
}

static void splash_tapped(lv_event_t *e) {
    LV_UNUSED(e);
    s_splash_tap = true;
}

static void build_splash() {
    if (scr_splash) {
        return;
    }
    scr_splash = lv_obj_create(nullptr);
    style_screen(scr_splash);
    lv_obj_clear_flag(scr_splash, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(scr_splash, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr_splash, splash_tapped, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *img = lv_img_create(scr_splash);
    lv_img_set_src(img, &img_splash);
    // Source is 480x270; scale up to fill the 1024-wide CrowPanel (zoom 256 = 1x).
    const uint16_t zoom = (uint16_t)((256u * 1024u) / 480u);
    lv_img_set_zoom(img, zoom);
    lv_obj_center(img);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);  // taps go to screen

    // Title + version overlaid like the previous splash.
    lv_obj_t *title = lv_label_create(scr_splash);
    lv_label_set_text(title, "FLIGHT RADAR");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(title, COL_GREEN, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 28);
    lv_obj_clear_flag(title, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *ver = lv_label_create(scr_splash);
    lv_label_set_text(ver, FR_VERSION_STR);  // "0.11 Beta"
    lv_obj_set_style_text_font(ver, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ver, COL_TEXT, 0);
    lv_obj_set_style_text_align(ver, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(ver, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_clear_flag(ver, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *hint = lv_label_create(scr_splash);
    lv_label_set_text(hint, "Tap to continue");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, COL_DIM, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -24);
    lv_obj_clear_flag(hint, LV_OBJ_FLAG_CLICKABLE);
}

void ui_show_splash() {
    build_splash();
    s_splash_tap = false;
    if (scr_splash) {
        lv_scr_load(scr_splash);
    }
}

void ui_wait_splash(uint32_t ms) {
    lvgl_port_lock(-1);
    ui_show_splash();
    lvgl_port_unlock();
    // Display-only hold — do not touch Wi-Fi here (that previously broke STA join).
    // Tap anywhere on the splash to skip early.
    const uint32_t start = millis();
    while (millis() - start < ms && !s_splash_tap) {
        delay(50);
    }
}

void ui_init(SharedState *state) {
    g_state = state;
    capture_home_from_cfg();  // lock home before any pan overwrites cfg.location
    if (g_state) {
        s_max_alt = g_state->cfg.max_alt_ft;
        s_range = g_state->cfg.range_nm;
    }
    lv_disp_t *disp = lv_disp_get_default();
    lv_theme_t *th = lv_theme_default_init(disp, COL_GREEN, COL_DIM, true, &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);
    build_splash();
    build_radar();
    lv_scr_load(scr_radar);
    lvgl_port_set_pinch_zoom_cb(on_pinch_zoom);
}

void ui_show_wifi() {
    lv_scr_load(scr_radar);
    open_wifi_panel();
}

void ui_show_wifi_joining(const char *ssid) {
    if (!scr_radar) {
        return;
    }
    if (!wifi_joining_overlay) {
        wifi_joining_overlay = lv_obj_create(scr_radar);
        lv_obj_set_size(wifi_joining_overlay, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(wifi_joining_overlay, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(wifi_joining_overlay, LV_OPA_60, 0);
        lv_obj_set_style_border_width(wifi_joining_overlay, 0, 0);
        lv_obj_set_style_pad_all(wifi_joining_overlay, 0, 0);
        lv_obj_clear_flag(wifi_joining_overlay, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(wifi_joining_overlay, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *card = lv_obj_create(wifi_joining_overlay);
        lv_obj_set_size(card, 440, 140);
        lv_obj_center(card);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x101810), 0);
        lv_obj_set_style_border_color(card, COL_GREEN, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 16, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *spin = lv_spinner_create(card, 1000, 60);
        lv_obj_set_size(spin, 48, 48);
        lv_obj_align(spin, LV_ALIGN_LEFT_MID, 4, 0);
        lv_obj_set_style_arc_color(spin, COL_DIM, LV_PART_MAIN);
        lv_obj_set_style_arc_width(spin, 6, LV_PART_MAIN);
        lv_obj_set_style_arc_color(spin, COL_GREEN, LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(spin, 6, LV_PART_INDICATOR);

        wifi_joining_lbl = lv_label_create(card);
        lv_obj_set_style_text_color(wifi_joining_lbl, COL_GREEN, 0);
        lv_obj_set_style_text_font(wifi_joining_lbl, &lv_font_montserrat_14, 0);
        lv_label_set_long_mode(wifi_joining_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(wifi_joining_lbl, 320);
        lv_obj_align(wifi_joining_lbl, LV_ALIGN_RIGHT_MID, -4, 0);
    }

    char msg[96];
    const char *name = (ssid && ssid[0]) ? ssid : "Wi-Fi";
    snprintf(msg, sizeof(msg), "Joining \"%s\" network", name);
    if (wifi_joining_lbl) {
        lv_label_set_text(wifi_joining_lbl, msg);
    }
    if (wifi_status) {
        lv_label_set_text(wifi_status, msg);
    }
    lv_obj_clear_flag(wifi_joining_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifi_joining_overlay);
}

void ui_hide_wifi_joining() {
    if (wifi_joining_overlay) {
        lv_obj_add_flag(wifi_joining_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

void ui_show_location() {
    if (detail) {
        lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    }
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (sound_panel) {
        lv_obj_add_flag(sound_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (county_panel) {
        lv_obj_add_flag(county_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    char buf[128];
    update_loc_label(g_state->cfg.location.label, g_state->cfg.location.lat, g_state->cfg.location.lon);
    refresh_local_time_lbl();
    snprintf(buf, sizeof(buf), "%.0f nm", g_state->cfg.range_nm);
    lv_label_set_text(radar_range_lbl, buf);
    format_alt_ceiling(g_state->cfg.max_alt_ft, buf, sizeof(buf));
    if (radar_alt_lbl) {
        lv_label_set_text(radar_alt_lbl, buf);
    }
    lv_scr_load(scr_radar);
    sync_startup_dropdowns();
    if (loc_panel) {
        lv_obj_clear_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(loc_panel);
    }
    // Keyboard only when the location field is tapped — keep dropdowns usable.
}

void ui_show_radar() {
    char buf[128];
    if (s_follow) {
        char fl[48];
        snprintf(fl, sizeof(fl), "Following %s", s_follow_label[0] ? s_follow_label : s_follow_hex);
        update_loc_label(fl, g_state->cfg.location.lat, g_state->cfg.location.lon);
    } else {
        // Prefer ICAO for airport homes (never a facility name / street address).
        update_loc_label(home_place_label(), g_state->cfg.location.lat, g_state->cfg.location.lon);
    }
    refresh_local_time_lbl();
    snprintf(buf, sizeof(buf), "%.0f nm", g_state->cfg.range_nm);
    lv_label_set_text(radar_range_lbl, buf);
    format_alt_ceiling(g_state->cfg.max_alt_ft, buf, sizeof(buf));
    if (radar_alt_lbl) {
        lv_label_set_text(radar_alt_lbl, buf);
    }
    s_max_alt = g_state->cfg.max_alt_ft;
    s_range = g_state->cfg.range_nm;
    s_show_airports = g_state->cfg.show_airports;
    s_show_counties = g_state->cfg.show_counties;
    s_show_cities = g_state->cfg.show_cities;
    s_county_mask = g_state->cfg.county_mask;
    update_apt_toggle_label();
    update_cty_toggle_label();
    update_city_toggle_label();
    sync_county_checkboxes();
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_kb) {
        lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (loc_panel) {
        lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
    }
    lv_scr_load(scr_radar);
    if (g_state) {
        g_state->radar_live = true;
    }
    // Paint any traffic fetched while the Wi-Fi screen was up, then refresh.
    apply_traffic_ui(true);
    request(UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
}

void ui_notify_wifi_scan_done() {
    char names[FR_MAX_WIFI][33];
    int rssi[FR_MAX_WIFI];
    int n = 0;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    n = g_state->wifi_count;
    memcpy(names, g_state->wifi_names, sizeof(names));
    memcpy(rssi, g_state->wifi_rssi, sizeof(rssi));
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    lv_obj_clean(wifi_list);
    for (int i = 0; i < n; i++) {
        char row[48];
        snprintf(row, sizeof(row), "%s   %d dBm", names[i], rssi[i]);
        lv_obj_t *btn = lv_list_add_btn(wifi_list, LV_SYMBOL_WIFI, row);
        strncpy(g_state->wifi_names[i], names[i], 32);
        lv_obj_add_event_cb(btn, ui_btn_click_sound, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(btn, wifi_item_clicked, LV_EVENT_CLICKED, g_state->wifi_names[i]);
    }
    char msg[48];
    snprintf(msg, sizeof(msg), "%d networks", n);
    lv_label_set_text(wifi_status, msg);
    refresh_wifi_connected_lbl();
    lvgl_port_unlock();
}

void ui_notify_wifi_result(bool ok, const char *msg) {
    lvgl_port_lock(-1);
    lv_label_set_text(wifi_status, msg);
    refresh_wifi_connected_lbl();
    if (ok) {
        if (g_state->cfg.location.valid) {
            ui_show_radar();
            request(UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
        } else {
            ui_show_location();
        }
    }
    lvgl_port_unlock();
}

void ui_notify_geo_done() {
    GeoFix results[8];
    int n = 0;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    n = g_state->geo_count;
    if (n > 0) {
        memcpy(results, g_state->geo_results, sizeof(GeoFix) * n);
    }
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    lv_obj_clean(loc_list);
    if (n == 0) {
        lv_label_set_text(loc_status, "No matches. Try a city, ZIP, or full address.");
    } else {
        lv_label_set_text(loc_status, "Tap a result");
        for (int i = 0; i < n; i++) {
            lv_obj_t *btn = lv_list_add_btn(loc_list, LV_SYMBOL_GPS, results[i].label);
            lv_obj_add_event_cb(btn, ui_btn_click_sound, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(btn, loc_pick_clicked, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        }
    }
    lvgl_port_unlock();
}

void ui_notify_home_label(const char *label) {
    if (!label || !label[0] || !s_home_ok) {
        return;
    }
    strncpy(s_home_loc.label, label, sizeof(s_home_loc.label) - 1);
    s_home_loc.label[sizeof(s_home_loc.label) - 1] = 0;
    // Keep icao in sync when enrich rewrote label to the ICAO code.
    if (strlen(label) <= 4) {
        bool code = true;
        for (const char *p = label; *p; p++) {
            if (!isalnum((unsigned char)*p)) {
                code = false;
                break;
            }
        }
        if (code) {
            strncpy(s_home_loc.icao, label, sizeof(s_home_loc.icao) - 1);
            s_home_loc.icao[sizeof(s_home_loc.icao) - 1] = 0;
        }
    }
    if (!s_follow) {
        update_loc_label(s_home_loc.icao[0] ? s_home_loc.icao : s_home_loc.label, s_home_loc.lat,
                         s_home_loc.lon);
    }
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static bool radar_overlay_open() {
    auto hidden = [](lv_obj_t *o) -> bool {
        return !o || lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);
    };
    return !(hidden(detail) && hidden(wifi_panel) && hidden(loc_panel) && hidden(settings_panel) &&
             hidden(sound_panel) && hidden(track_panel) && hidden(board_panel) &&
             hidden(route_panel) && hidden(plates_panel) && hidden(wifi_kb) && hidden(loc_kb) &&
             hidden(track_kb) && hidden(board_kb) && hidden(route_kb) &&
             hidden(plates_kb));
}

static constexpr int kTrafficTableMax = FR_MAX_AIRCRAFT;

// Sortable columns: 2=DIST, 3=ALT, 4=SPD, 5=TYPE
enum TrafficSortCol : int { kSortDist = 2, kSortAlt = 3, kSortSpd = 4, kSortType = 5 };
static int s_traffic_sort_col = kSortDist;
static bool s_traffic_sort_asc = true;  // true = up arrow (low→high / A→Z)

static bool traffic_col_sortable(uint16_t col) {
    return col == kSortDist || col == kSortAlt || col == kSortSpd || col == kSortType;
}

static void update_traffic_headers() {
    if (!traffic_table) {
        return;
    }
    const char *arrow = s_traffic_sort_asc ? LV_SYMBOL_UP : LV_SYMBOL_DOWN;
    char h[24];
    auto set_hdr = [&](uint16_t col, const char *base) {
        if ((int)col == s_traffic_sort_col) {
            snprintf(h, sizeof(h), "%s%s", base, arrow);
            lv_table_set_cell_value(traffic_table, 0, col, h);
        } else {
            lv_table_set_cell_value(traffic_table, 0, col, base);
        }
    };
    lv_table_set_cell_value(traffic_table, 0, 0, "TAIL");
    lv_table_set_cell_value(traffic_table, 0, 1, "FLIGHT");
    set_hdr(2, "DIST");
    set_hdr(3, "ALT");
    set_hdr(4, "SPD");
    set_hdr(5, "TYPE");
}

static int traffic_sort_cmp(const Aircraft *a, const Aircraft *b) {
    int cmp = 0;
    switch (s_traffic_sort_col) {
        case kSortAlt:
            cmp = (a->alt_ft < b->alt_ft) ? -1 : (a->alt_ft > b->alt_ft) ? 1 : 0;
            break;
        case kSortSpd: {
            const bool an = isnan(a->gs_kts);
            const bool bn = isnan(b->gs_kts);
            if (an && bn) {
                cmp = 0;
            } else if (an) {
                cmp = 1;  // unknowns last
            } else if (bn) {
                cmp = -1;
            } else {
                cmp = (a->gs_kts < b->gs_kts) ? -1 : (a->gs_kts > b->gs_kts) ? 1 : 0;
            }
            break;
        }
        case kSortType: {
            const char *at = a->type[0] ? a->type : "";
            const char *bt = b->type[0] ? b->type : "";
            cmp = strcasecmp(at, bt);
            break;
        }
        case kSortDist:
        default:
            cmp = (a->dst_nm < b->dst_nm) ? -1 : (a->dst_nm > b->dst_nm) ? 1 : 0;
            break;
    }
    if (cmp == 0) {
        // Stable-ish tiebreak: nearest first.
        cmp = (a->dst_nm < b->dst_nm) ? -1 : (a->dst_nm > b->dst_nm) ? 1 : 0;
    }
    return s_traffic_sort_asc ? cmp : -cmp;
}

static void traffic_table_draw_part(lv_event_t *e) {
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (!dsc || dsc->part != LV_PART_ITEMS || dsc->type != LV_TABLE_DRAW_PART_CELL) {
        return;
    }
    if (s_traffic_sel_row < 1) {
        return;
    }
    const uint16_t col_cnt = lv_table_get_col_cnt(table);
    if (col_cnt == 0) {
        return;
    }
    const uint16_t row = (uint16_t)(dsc->id / col_cnt);
    if ((int)row != s_traffic_sel_row) {
        return;
    }
    if (dsc->rect_dsc) {
        dsc->rect_dsc->bg_color = lv_color_hex(0xB0B0B0);  // light grey highlight
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
    }
    if (dsc->label_dsc) {
        dsc->label_dsc->color = lv_color_hex(0x101010);
    }
}

static void traffic_table_selected(lv_event_t *e) {
    LV_UNUSED(e);
    if (!traffic_table) {
        return;
    }
    uint16_t row = 0, col = 0;
    lv_table_get_selected_cell(traffic_table, &row, &col);
    if (row == LV_TABLE_CELL_NONE || col == LV_TABLE_CELL_NONE) {
        return;
    }
    // Header tap: sort by DIST / ALT / SPD / TYPE (toggle asc/desc on same column).
    if (row == 0) {
        if (!traffic_col_sortable(col)) {
            return;
        }
        if (s_traffic_sort_col == (int)col) {
            s_traffic_sort_asc = !s_traffic_sort_asc;
        } else {
            s_traffic_sort_col = (int)col;
            s_traffic_sort_asc = true;
        }
        update_traffic_headers();
        refresh_traffic_table();
        return;
    }
    const int idx = (int)row - 1;
    if (idx < 0 || idx >= s_table_n) {
        return;
    }
    s_traffic_sel_row = (int)row;
    lv_obj_invalidate(traffic_table);
    // Highlight + track immediately — no aircraft detail popup.
    start_follow_aircraft(&s_table_ac[idx], false);
}

static void refresh_traffic_table() {
    if (!traffic_table) {
        return;
    }

    int idxs[FR_MAX_AIRCRAFT];
    int n = 0;
    for (int i = 0; i < s_ac_n; i++) {
        if (!s_ac[i].valid || s_ac[i].on_ground) {
            continue;
        }
        if (!aircraft_shown(&s_ac[i])) {
            continue;
        }
        if (s_ac[i].dst_nm > s_range * 1.02f) {
            continue;
        }
        idxs[n++] = i;
    }
    for (int a = 0; a < n; a++) {
        for (int b = a + 1; b < n; b++) {
            if (traffic_sort_cmp(&s_ac[idxs[b]], &s_ac[idxs[a]]) < 0) {
                const int tmp = idxs[a];
                idxs[a] = idxs[b];
                idxs[b] = tmp;
            }
        }
    }

    const int show = (n > kTrafficTableMax) ? kTrafficTableMax : n;
    s_table_n = show;
    lv_table_set_row_cnt(traffic_table, (uint16_t)(show + 1));
    update_traffic_headers();

    for (int r = 0; r < show; r++) {
        const Aircraft *ac = &s_ac[idxs[r]];
        s_table_ac[r] = *ac;
        const char *tail = ac->reg[0] ? ac->reg : (ac->hex[0] ? ac->hex : "--");
        const char *flight = ac->flight[0] ? ac->flight : "--";
        char dist[16], alt[24], spd[16];
        snprintf(dist, sizeof(dist), "%.1f", ac->dst_nm);
        if (!isnan(ac->vrate_fpm) && ac->vrate_fpm > 128.0f) {
            snprintf(alt, sizeof(alt), LV_SYMBOL_UP " %.0f", ac->alt_ft);
        } else if (!isnan(ac->vrate_fpm) && ac->vrate_fpm < -128.0f) {
            snprintf(alt, sizeof(alt), LV_SYMBOL_DOWN " %.0f", ac->alt_ft);
        } else {
            snprintf(alt, sizeof(alt), "%.0f", ac->alt_ft);
        }
        if (isnan(ac->gs_kts)) {
            strncpy(spd, "--", sizeof(spd) - 1);
            spd[sizeof(spd) - 1] = 0;
        } else {
            snprintf(spd, sizeof(spd), "%.0f", ac->gs_kts);
        }
        const char *typ = ac->type[0] ? ac->type : "--";
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 0, tail);
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 1, flight);
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 2, dist);
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 3, alt);
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 4, spd);
        lv_table_set_cell_value(traffic_table, (uint16_t)(r + 1), 5, typ);
    }
    // Keep highlight on the followed aircraft across refreshes / re-sorts.
    s_traffic_sel_row = -1;
    if (s_follow && (s_follow_hex[0] || s_follow_label[0])) {
        for (int r = 0; r < show; r++) {
            const Aircraft *ac = &s_table_ac[r];
            if (s_follow_hex[0] && ac->hex[0] && strcasecmp(ac->hex, s_follow_hex) == 0) {
                s_traffic_sel_row = r + 1;
                break;
            }
            if (s_follow_label[0] && ac->flight[0] && strcasecmp(ac->flight, s_follow_label) == 0) {
                s_traffic_sel_row = r + 1;
                break;
            }
        }
    }
    lv_obj_invalidate(traffic_table);
}

static void apply_traffic_ui(bool redraw_radar) {
    if (!g_state) {
        return;
    }
    char buf[64];
    char status[96];
    int acn = 0, apn = 0;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    acn = g_state->aircraft_count;
    apn = g_state->airport_count;
    strncpy(status, g_state->status, sizeof(status) - 1);
    status[sizeof(status) - 1] = 0;
    xSemaphoreGive(g_state->mutex);
    snprintf(buf, sizeof(buf), "%d aircraft   %d airports", acn, apn);

    snapshot_radar();
    update_follow_from_traffic();
    if (s_follow) {
        // Follow panel owns the right rail; skip bulk traffic table rebuild.
    } else {
        refresh_traffic_table();
    }

    if (radar_count_lbl) {
        lv_label_set_text(radar_count_lbl, buf);
    }
    if (radar_status) {
        lv_label_set_text(radar_status, status);
        s_status_lv_ms = millis();
    }
    if (redraw_radar && radar_plot && lv_scr_act() == scr_radar && !radar_overlay_open()) {
        // Rebuild map in PSRAM before the FB refresh so DRAW_MAIN stays a fast blit.
        map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);
        lv_obj_invalidate(radar_plot);
    }
}

static void traffic_ui_timer_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    s_traffic_ui_timer = nullptr;
    apply_traffic_ui(true);
}

void ui_notify_traffic_labels() {
    lvgl_port_lock(-1);
    apply_traffic_ui(false);
    lvgl_port_unlock();
}

void ui_notify_traffic() {
    // One deferred paint: labels + PPI together (avoids double-refresh flicker).
    ui_schedule_radar_redraw(120);
}

void ui_schedule_radar_redraw(uint32_t delay_ms) {
    lvgl_port_lock(-1);
    if (s_traffic_ui_timer) {
        lv_timer_del(s_traffic_ui_timer);
        s_traffic_ui_timer = nullptr;
    }
    s_traffic_ui_timer = lv_timer_create(traffic_ui_timer_cb, delay_ms ? delay_ms : 1, nullptr);
    lv_timer_set_repeat_count(s_traffic_ui_timer, 1);
    lvgl_port_unlock();
}

void ui_notify_airports() {
    ui_notify_traffic();
}

void ui_notify_flight_info() {
    FlightInfo fi{};
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    fi = g_state->flight_info;
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    if (s_follow && ((fi.hex[0] && strcasecmp(fi.hex, s_follow_hex) == 0) ||
                     (fi.flight[0] && strcasecmp(fi.flight, s_follow_label) == 0) ||
                     (fi.flight[0] && s_follow_ac_ok && strcasecmp(fi.flight, s_follow_ac.flight) == 0))) {
        merge_follow_fi(fi);
        if (fi.reg[0] && s_follow_ac_ok && !s_follow_ac.reg[0]) {
            strncpy(s_follow_ac.reg, fi.reg, sizeof(s_follow_ac.reg) - 1);
            s_follow_ac.reg[sizeof(s_follow_ac.reg) - 1] = 0;
        }
        follow_try_seed_hist_from_state();
        refresh_follow_panel();
    }
    if (!s_detail_is_aircraft || !detail || lv_obj_has_flag(detail, LV_OBJ_FLAG_HIDDEN)) {
        lvgl_port_unlock();
        return;
    }
    s_flight_info = fi;
    if (fi.reg[0] && !s_detail_ac.reg[0]) {
        strncpy(s_detail_ac.reg, fi.reg, sizeof(s_detail_ac.reg) - 1);
        s_detail_ac.reg[sizeof(s_detail_ac.reg) - 1] = 0;
    }
    char title[32], body[768];
    format_aircraft(&s_detail_ac, title, sizeof(title), body, sizeof(body), &s_flight_info, false);
    lv_label_set_text(detail_title, title);
    lv_label_set_text(detail_body, body);
    set_detail_aircraft_view();
    lvgl_port_unlock();
}

void ui_notify_airport_search(bool ok, const char *msg) {
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const int hit_n = (g_state->apt_hit_n > FR_MAX_APT_SEARCH) ? FR_MAX_APT_SEARCH : g_state->apt_hit_n;
    AptSearchHit hits[FR_MAX_APT_SEARCH];
    memcpy(hits, g_state->apt_hits, sizeof(AptSearchHit) * (hit_n > 0 ? hit_n : 0));
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    const bool plates_open =
        plates_panel && !lv_obj_has_flag(plates_panel, LV_OBJ_FLAG_HIDDEN) && s_plates_search_pending;
    if (plates_open) {
        s_plate_hit_n = hit_n;
        memcpy(s_plate_hits, hits, sizeof(AptSearchHit) * (s_plate_hit_n > 0 ? s_plate_hit_n : 0));
        s_plates_search_pending = false;
        if (plates_status) {
            lv_label_set_text(plates_status, msg ? msg : (ok ? "Tap a result to add" : "No airports found"));
        }
        plates_refresh_add_hits_ui();
        if (ok) {
            plates_hide_keyboard();
        }
    } else {
        s_apt_hit_n = hit_n;
        memcpy(s_apt_hits, hits, sizeof(AptSearchHit) * (s_apt_hit_n > 0 ? s_apt_hit_n : 0));
        if (board_status) {
            lv_label_set_text(board_status, msg ? msg : (ok ? "Select an airport" : "No airports found"));
        }
        refresh_airport_hits_ui();
        if (ok) {
            board_hide_keyboard();
        }
    }
    lvgl_port_unlock();
}

void ui_set_board_status(const char *msg) {
    if (!msg) {
        return;
    }
    // Throttle — net-task progress spam + LVGL lock was a reopen hang risk.
    static uint32_t s_board_st_ms = 0;
    const uint32_t now = millis();
    if (s_board_st_ms != 0 && (now - s_board_st_ms) < 400) {
        return;
    }
    s_board_st_ms = now;
    lvgl_port_lock(-1);
    if (board_status) {
        lv_label_set_text(board_status, msg);
    }
    lvgl_port_unlock();
}

void ui_notify_airport_board(bool ok, const char *msg) {
    char label[48] = {0};
    char icao[8] = {0};
    if (ok) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        s_board_arr_n = g_state->board_arr_n;
        s_board_dep_n = g_state->board_dep_n;
        if (s_board_arr_n > FR_MAX_BOARD_ROWS) {
            s_board_arr_n = FR_MAX_BOARD_ROWS;
        }
        if (s_board_dep_n > FR_MAX_BOARD_ROWS) {
            s_board_dep_n = FR_MAX_BOARD_ROWS;
        }
        memcpy(s_board_arr, g_state->board_arr, sizeof(AirportBoardRow) * s_board_arr_n);
        memcpy(s_board_dep, g_state->board_dep, sizeof(AirportBoardRow) * s_board_dep_n);
        strncpy(label, g_state->board_label, sizeof(label) - 1);
        strncpy(icao, g_state->board_icao[0] ? g_state->board_icao : g_state->pending_board_icao,
                sizeof(icao) - 1);
        xSemaphoreGive(g_state->mutex);
    } else {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        strncpy(label, g_state->board_label, sizeof(label) - 1);
        strncpy(icao, g_state->board_icao[0] ? g_state->board_icao : g_state->pending_board_icao,
                sizeof(icao) - 1);
        xSemaphoreGive(g_state->mutex);
    }

    lvgl_port_lock(-1);
    // Prefer lat/lon resolved during the board fetch (AWC); fall back to local DB.
    if (ok) {
        bool have_ll = false;
        float lat = 0, lon = 0;
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        if (g_state->board_ll_ok) {
            lat = g_state->board_lat;
            lon = g_state->board_lon;
            have_ll = true;
        }
        xSemaphoreGive(g_state->mutex);
        if (have_ll) {
            s_board_lat = lat;
            s_board_lon = lon;
            s_board_ll_ok = true;
            if (icao[0]) {
                strncpy(s_board_icao, icao, sizeof(s_board_icao) - 1);
                s_board_icao[sizeof(s_board_icao) - 1] = 0;
            }
        } else {
            const char *ll_code = icao[0] ? icao : label;
            if (ll_code && ll_code[0]) {
                board_set_airport_ll(ll_code);
            }
        }
        if (WiFi.status() == WL_CONNECTED && time(nullptr) < 1700000000) {
            configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
        }
        refresh_board_clock_time();
    }
    if (board_status) {
        lv_label_set_text(board_status, msg ? msg : (ok ? "Select a flight to follow" : "Board failed"));
    }
    if (board_title_lbl && label[0]) {
        char t[72];
        snprintf(t, sizeof(t), "Airport Board - %s", label);
        lv_label_set_text(board_title_lbl, t);
    }
    s_apt_hit_n = 0;
    board_clear_selection();
    refresh_airport_hits_ui();
    refresh_airport_board_tables();
    board_hide_keyboard();
    if (board_panel) {
        lv_obj_clear_flag(board_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(board_panel);
    }
    lvgl_port_unlock();
}

void ui_notify_plate_cities(bool ok) {
    lvgl_port_lock(-1);
    int n = 0;
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    n = g_state->plate_city_n;
    if (n > s_plate_extra_n) {
        n = s_plate_extra_n;
    }
    if (n > FR_PLATE_EXTRA_MAX) {
        n = FR_PLATE_EXTRA_MAX;
    }
    bool changed = false;
    for (int i = 0; i < n; i++) {
        if (!g_state->plate_city_icao[i][0] || !g_state->plate_city_name[i][0]) {
            continue;
        }
        if (strcasecmp(s_plate_extra[i], g_state->plate_city_icao[i]) != 0) {
            // Match by ICAO in case order shifted.
            bool found = false;
            for (int j = 0; j < s_plate_extra_n; j++) {
                if (strcasecmp(s_plate_extra[j], g_state->plate_city_icao[i]) == 0) {
                    if (strcmp(s_plate_extra_city[j], g_state->plate_city_name[i]) != 0) {
                        strncpy(s_plate_extra_city[j], g_state->plate_city_name[i],
                                FR_PLATE_EXTRA_CITY_LEN - 1);
                        s_plate_extra_city[j][FR_PLATE_EXTRA_CITY_LEN - 1] = 0;
                        changed = true;
                    }
                    found = true;
                    break;
                }
            }
            (void)found;
            continue;
        }
        if (strcmp(s_plate_extra_city[i], g_state->plate_city_name[i]) != 0) {
            strncpy(s_plate_extra_city[i], g_state->plate_city_name[i], FR_PLATE_EXTRA_CITY_LEN - 1);
            s_plate_extra_city[i][FR_PLATE_EXTRA_CITY_LEN - 1] = 0;
            changed = true;
        }
    }
    xSemaphoreGive(g_state->mutex);
    if (changed) {
        config_save_plate_extras(s_plate_extra, s_plate_extra_city, s_plate_extra_n);
        plates_refresh_list();
    }
    if (plates_city_lbl) {
        lv_label_set_text(plates_city_lbl, ok ? "City names updated" : "City refresh failed");
    }
    lvgl_port_unlock();
}

void ui_notify_route_search(bool ok, const char *msg) {
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    s_route_dep_hit_n = g_state->route_dep_hit_n;
    s_route_arr_hit_n = g_state->route_arr_hit_n;
    if (s_route_dep_hit_n > FR_MAX_APT_SEARCH) {
        s_route_dep_hit_n = FR_MAX_APT_SEARCH;
    }
    if (s_route_arr_hit_n > FR_MAX_APT_SEARCH) {
        s_route_arr_hit_n = FR_MAX_APT_SEARCH;
    }
    memcpy(s_route_dep_hits, g_state->route_dep_hits, sizeof(AptSearchHit) * s_route_dep_hit_n);
    memcpy(s_route_arr_hits, g_state->route_arr_hits, sizeof(AptSearchHit) * s_route_arr_hit_n);
    if (g_state->route_dep_icao[0]) {
        strncpy(s_route_dep_icao, g_state->route_dep_icao, sizeof(s_route_dep_icao) - 1);
        s_route_dep_icao[sizeof(s_route_dep_icao) - 1] = 0;
    }
    if (g_state->route_arr_icao[0]) {
        strncpy(s_route_arr_icao, g_state->route_arr_icao, sizeof(s_route_arr_icao) - 1);
        s_route_arr_icao[sizeof(s_route_arr_icao) - 1] = 0;
    }
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    if (route_status) {
        lv_label_set_text(route_status, msg ? msg : (ok ? "Select airports" : "Search failed"));
    }
    if (s_route_dep_icao[0] && route_dep_ta) {
        lv_textarea_set_text(route_dep_ta, s_route_dep_icao);
    }
    if (s_route_arr_icao[0] && route_arr_ta) {
        lv_textarea_set_text(route_arr_ta, s_route_arr_icao);
    }
    route_refresh_hits_ui();
    route_hide_keyboard();
    if (route_panel) {
        lv_obj_clear_flag(route_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(route_panel);
    }
    lvgl_port_unlock();
}

void ui_notify_route_flights(bool ok, const char *msg) {
    char label[48] = {0};
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    s_route_row_n = g_state->route_row_n;
    if (s_route_row_n > FR_MAX_BOARD_ROWS) {
        s_route_row_n = FR_MAX_BOARD_ROWS;
    }
    memcpy(s_route_rows, g_state->route_rows, sizeof(AirportBoardRow) * s_route_row_n);
    strncpy(label, g_state->route_label, sizeof(label) - 1);
    if (g_state->route_dep_icao[0]) {
        strncpy(s_route_dep_icao, g_state->route_dep_icao, sizeof(s_route_dep_icao) - 1);
    }
    if (g_state->route_arr_icao[0]) {
        strncpy(s_route_arr_icao, g_state->route_arr_icao, sizeof(s_route_arr_icao) - 1);
    }
    s_route_dep_hit_n = 0;
    s_route_arr_hit_n = 0;
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    if (route_status) {
        lv_label_set_text(route_status, msg ? msg : (ok ? "Select a flight to follow" : "No flights"));
    }
    if (route_title_lbl) {
        if (s_route_dep_icao[0] && s_route_arr_icao[0]) {
            char t[48];
            snprintf(t, sizeof(t), "%s to %s", s_route_dep_icao, s_route_arr_icao);
            lv_label_set_text(route_title_lbl, t);
        } else if (label[0]) {
            lv_label_set_text(route_title_lbl, label);
        } else {
            lv_label_set_text(route_title_lbl, "Departures and Arrivals Search");
        }
    }
    s_route_sel_row = -1;
    route_refresh_hits_ui();
    route_refresh_table();
    route_hide_keyboard();
    if (route_panel) {
        lv_obj_clear_flag(route_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(route_panel);
    }
    lvgl_port_unlock();
}

void ui_notify_track_result(bool ok, const char *msg) {
    Aircraft ac{};
    FlightInfo fi{};
    bool have_ac = false;
    char tracked_q[96] = {0};
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    have_ac = g_state->tracked_ok;
    if (have_ac) {
        ac = g_state->tracked_ac;
    }
    fi = g_state->flight_info;
    strncpy(tracked_q, g_state->pending_query, sizeof(tracked_q) - 1);
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    if (track_status) {
        lv_label_set_text(track_status, msg ? msg : (ok ? "Tracking" : "Not found"));
    }
    if (board_status && board_panel && !lv_obj_has_flag(board_panel, LV_OBJ_FLAG_HIDDEN)) {
        lv_label_set_text(board_status, msg ? msg : (ok ? "Tracking" : "Track failed"));
    }
    // Already following from an airport-board tap — keep that; only upgrade position/info.
    if ((!ok || !have_ac) && s_follow) {
        if (ok && fi.flight[0]) {
            merge_follow_fi(fi);
            refresh_follow_panel();
        }
        lvgl_port_unlock();
        return;
    }
    if (!ok || !have_ac) {
        lvgl_port_unlock();
        return;
    }
    // Prefer the flight label / query the user entered for history.
    const char *hist_id = tracked_q[0] ? tracked_q : (ac.flight[0] ? ac.flight : fi.flight);
    track_history_push(hist_id);
    hide_track(nullptr);
    hide_airport_board(nullptr);
    hide_route_search(nullptr);
    // Seed detail/flight-info so follow keeps schedule/gate fields.
    s_detail_ac = ac;
    s_detail_is_aircraft = true;
    s_flight_info = fi;
    start_follow_aircraft(&ac, false);
    // start_follow may clear fi; restore tracked schedule (airports/gates/times).
    merge_follow_fi(fi);
    follow_try_seed_hist_from_state();
    refresh_follow_panel();
    lvgl_port_unlock();
}

void ui_notify_weather() {
    Weather w{};
    Airport apt{};
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    w = g_state->weather;
    for (int i = 0; i < g_state->airport_count; i++) {
        if (strcmp(g_state->airports[i].icao, w.icao) == 0) {
            apt = g_state->airports[i];
            break;
        }
    }
    xSemaphoreGive(g_state->mutex);

    lvgl_port_lock(-1);
    // Don't reopen after Close / Center on airport while METAR was still in flight.
    if (!detail || lv_obj_has_flag(detail, LV_OBJ_FLAG_HIDDEN) || !s_detail_apt_ok ||
        strcmp(s_detail_apt.icao, w.icao) != 0) {
        lvgl_port_unlock();
        return;
    }
    // Prefer the airport snapshot from the tap so elev/rwy never vanish after METAR reload.
    apt = s_detail_apt;
    s_detail_is_aircraft = false;
    const Weather *wp = &w;
    char title[40];
    snprintf(title, sizeof(title), "%s  %s  %s", wp->icao, apt.iata, wp->flt_cat);

    char wind[48] = "--";
    char temp[24] = "--";
    char vis[24] = "--";
    if (!isnan(wp->wind_kt)) {
        if (isnan(wp->wind_dir)) {
            snprintf(wind, sizeof(wind), "VRB %.0f kt", wp->wind_kt);
        } else {
            snprintf(wind, sizeof(wind), "%.0f at %.0f kt", wp->wind_dir, wp->wind_kt);
        }
    }
    if (!isnan(wp->temp_c)) {
        snprintf(temp, sizeof(temp), "%.0f C", wp->temp_c);
    }
    if (!isnan(wp->vis_sm)) {
        snprintf(vis, sizeof(vis), "%.0f", wp->vis_sm);
    }

    if (wp->city[0]) {
        strncpy(s_detail_city, wp->city, sizeof(s_detail_city) - 1);
        s_detail_city[sizeof(s_detail_city) - 1] = 0;
    } else if (!s_detail_city[0]) {
        plates_city_from_name(apt.name, s_detail_city, sizeof(s_detail_city));
    }
    s_detail_radios = wp->radios;
    // Keep onboard values if AWC left slots empty.
    AirportRadios local{};
    if (net_lookup_airport_radios(wp->icao, &local)) {
        if (!s_detail_radios.ctaf[0] && local.ctaf[0]) {
            strncpy(s_detail_radios.ctaf, local.ctaf, sizeof(s_detail_radios.ctaf) - 1);
        }
        if (!s_detail_radios.atc[0] && local.atc[0]) {
            strncpy(s_detail_radios.atc, local.atc, sizeof(s_detail_radios.atc) - 1);
            s_detail_radios.atc_is_center = local.atc_is_center;
        }
        if (!s_detail_radios.gnd[0] && local.gnd[0]) {
            strncpy(s_detail_radios.gnd, local.gnd, sizeof(s_detail_radios.gnd) - 1);
        }
        if (!s_detail_radios.atis[0] && local.atis[0]) {
            strncpy(s_detail_radios.atis, local.atis, sizeof(s_detail_radios.atis) - 1);
            s_detail_radios.atis_is_awos = local.atis_is_awos;
        }
    }
    char radios[96];
    format_radios_block(&s_detail_radios, radios, sizeof(radios));

    char body[960];
    snprintf(body, sizeof(body),
             "%s\n%s %s\n"
             "Elev %d ft\n"
             "Rwy  %d ft\n"
             "Pos  %.4f, %.4f\n"
             "%s\n"
             "Wind  %s\n"
             "Temp  %s\n"
             "Vis   %s SM   Cover %s\n\n"
             "METAR\n%s\n\nTAF\n%s",
             apt.name[0] ? apt.name : wp->icao,
             apt.state, apt.country,
             apt.elev_ft,
             apt.longest_rwy_ft,
             apt.lat, apt.lon,
             radios,
             wind, temp, vis,
             wp->cover[0] ? wp->cover : "--",
             wp->raw_metar[0] ? wp->raw_metar : "unavailable",
             wp->raw_taf[0] ? wp->raw_taf : "unavailable");
    show_detail(title, body, wp->icao);
    lvgl_port_unlock();
}

void ui_set_status(const char *msg) {
    if (!msg) {
        return;
    }
    if (g_state) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        strncpy(g_state->status, msg, sizeof(g_state->status) - 1);
        g_state->status[sizeof(g_state->status) - 1] = 0;
        xSemaphoreGive(g_state->mutex);
    }
    // Throttle LVGL status paints during HTTPS so mid-fetch updates do not flicker.
    const uint32_t now = millis();
    if (s_status_lv_ms != 0 && (now - s_status_lv_ms) < 400) {
        return;
    }
    s_status_lv_ms = now;
    lvgl_port_lock(-1);
    if (radar_status) {
        lv_label_set_text(radar_status, msg);
    }
    lvgl_port_unlock();
}
