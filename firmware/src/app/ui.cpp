#include "ui.h"

#include "config_store.h"
#include "lvgl_v8_port.h"
#include "map_db.h"
#include "net_client.h"

#include <ctype.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static SharedState *g_state = nullptr;

#ifdef BOARD_CROWPANEL_P4
// 1024x600: square scope sized to the panel height, side rail stays on the right.
static constexpr lv_coord_t kRadarPlot = 580;
static constexpr lv_coord_t kSideW = 400;
static constexpr lv_coord_t kSideH = 580;
#else
static constexpr lv_coord_t kRadarPlot = 470;
static constexpr lv_coord_t kSideW = 318;
static constexpr lv_coord_t kSideH = 468;
#endif

static lv_obj_t *scr_radar = nullptr;
static lv_obj_t *wifi_panel = nullptr;
static lv_obj_t *loc_panel = nullptr;
static lv_obj_t *wifi_list = nullptr;
static lv_obj_t *wifi_status = nullptr;
static lv_obj_t *wifi_ta = nullptr;
static lv_obj_t *wifi_kb = nullptr;
static lv_obj_t *loc_ta = nullptr;
static lv_obj_t *loc_list = nullptr;
static lv_obj_t *loc_status = nullptr;
static lv_obj_t *loc_kb = nullptr;
static lv_obj_t *radar_plot = nullptr;
static lv_obj_t *radar_status = nullptr;
static lv_obj_t *radar_range_lbl = nullptr;
static lv_obj_t *radar_alt_lbl = nullptr;
static lv_obj_t *radar_loc_lbl = nullptr;
static lv_obj_t *radar_count_lbl = nullptr;
static lv_obj_t *traffic_table = nullptr;
static lv_obj_t *follow_panel = nullptr;
static lv_obj_t *follow_info_lbl = nullptr;
static lv_obj_t *follow_chart = nullptr;
static lv_obj_t *detail = nullptr;
static lv_obj_t *detail_title = nullptr;
static lv_obj_t *detail_body = nullptr;
static lv_obj_t *detail_plate = nullptr;
static lv_obj_t *detail_center_btn = nullptr;
static lv_obj_t *detail_follow_btn = nullptr;
static Runway s_plate_rwys[FR_MAX_RWYS_PER_APT];
static int s_plate_rwy_n = 0;
static char s_plate_icao[8] = {0};
static Airport s_detail_apt{};
static bool s_detail_apt_ok = false;
static Aircraft s_detail_ac{};
static bool s_detail_is_aircraft = false;
static FlightInfo s_flight_info{};
static Aircraft s_table_ac[15];
static int s_table_n = 0;
static lv_timer_t *s_traffic_ui_timer = nullptr;
static lv_timer_t *s_sweep_timer = nullptr;
static float s_sweep_deg = 0.0f;  // PPI sweep angle, clockwise from north
static uint32_t s_status_lv_ms = 0;

// Follow-flight: center radar on a hex and record a path trail.
static bool s_follow = false;
static bool s_follow_solo = false;  // when following: hide all other aircraft
static char s_follow_hex[8] = {0};
static char s_follow_label[16] = {0};
static GeoFix s_follow_saved_loc{};
static bool s_follow_saved_ok = false;
static Aircraft s_follow_ac{};
static FlightInfo s_follow_fi{};
static bool s_follow_ac_ok = false;
static constexpr int kTrailMax = 160;
struct TrailPt {
    float lat;
    float lon;
};
static TrailPt s_trail[kTrailMax];
static int s_trail_head = 0;
static int s_trail_count = 0;
static constexpr int kHistMax = 96;
struct HistPt {
    float alt_ft;
    float gs_kts;
};
static HistPt s_hist[kHistMax];
static int s_hist_head = 0;
static int s_hist_count = 0;

static const float kZoom[] = {15, 25, 40, 60, 100, 160, 250};
static const int kZoomN = 7;
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
static const lv_color_t COL_MAP = lv_color_hex(0x4A554E);  // dark gray area sketch (behind traffic)

static void style_screen(lv_obj_t *scr) {
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
}

static lv_obj_t *make_btn(lv_obj_t *parent, const char *txt, lv_event_cb_t cb, void *ud) {
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x145C2C), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1E8A42), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
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

static int zoom_index(float range) {
    int best = 3;
    float err = 1e9;
    for (int i = 0; i < kZoomN; i++) {
        float e = fabsf(kZoom[i] - range);
        if (e < err) {
            err = e;
            best = i;
        }
    }
    return best;
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
static float s_range = 50;
static int s_max_alt = 45000;
static bool s_show_airports = true;
static lv_obj_t *apt_toggle_btn = nullptr;
static lv_obj_t *solo_toggle_btn = nullptr;

static bool aircraft_below_ceiling(const Aircraft *ac, int max_alt_ft) {
    if (ac->on_ground && max_alt_ft > 0) {
        return true;
    }
    return ac->alt_ft < (float)max_alt_ft;
}

static bool aircraft_shown(const Aircraft *ac) {
    if (!aircraft_below_ceiling(ac, s_max_alt)) {
        return false;
    }
    if (s_follow && s_follow_solo && s_follow_hex[0]) {
        return strcasecmp(ac->hex, s_follow_hex) == 0;
    }
    return true;
}

static void update_solo_toggle_btn() {
    if (!solo_toggle_btn) {
        return;
    }
    if (s_follow) {
        lv_obj_clear_flag(solo_toggle_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_t *lbl = lv_obj_get_child(solo_toggle_btn, 0);
        if (lbl) {
            lv_label_set_text(lbl, s_follow_solo ? "Show Ac" : "Hide Ac");
        }
    } else {
        lv_obj_add_flag(solo_toggle_btn, LV_OBJ_FLAG_HIDDEN);
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
static void refresh_traffic_table();
static void refresh_follow_panel();
static void set_follow_side_mode(bool following);

static void trail_clear() {
    s_trail_head = 0;
    s_trail_count = 0;
}

static void hist_clear() {
    s_hist_head = 0;
    s_hist_count = 0;
}

static void hist_push(float alt_ft, float gs_kts) {
    if (alt_ft < 0.0f) {
        alt_ft = 0.0f;
    }
    if (gs_kts < 0.0f) {
        gs_kts = 0.0f;
    }
    if (s_hist_count > 0) {
        const int last = (s_hist_head - 1 + kHistMax) % kHistMax;
        const float da = alt_ft - s_hist[last].alt_ft;
        const float ds = gs_kts - s_hist[last].gs_kts;
        // Keep near-duplicate samples so the chart advances with time.
        if (fabsf(da) < 5.0f && fabsf(ds) < 1.0f && s_hist_count > 2) {
            s_hist[last].alt_ft = alt_ft;
            s_hist[last].gs_kts = gs_kts;
            return;
        }
    }
    s_hist[s_hist_head].alt_ft = alt_ft;
    s_hist[s_hist_head].gs_kts = gs_kts;
    s_hist_head = (s_hist_head + 1) % kHistMax;
    if (s_hist_count < kHistMax) {
        s_hist_count++;
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

static void update_loc_label(const char *label, float lat, float lon) {
    if (!radar_loc_lbl) {
        return;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\n%.4f, %.4f", label && label[0] ? label : "Radar", lat, lon);
    lv_label_set_text(radar_loc_lbl, buf);
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

static void stop_follow(bool restore_saved) {
    s_follow = false;
    s_follow_solo = false;
    s_follow_hex[0] = 0;
    s_follow_label[0] = 0;
    s_follow_ac_ok = false;
    memset(&s_follow_ac, 0, sizeof(s_follow_ac));
    memset(&s_follow_fi, 0, sizeof(s_follow_fi));
    trail_clear();
    hist_clear();
    update_solo_toggle_btn();
    set_follow_side_mode(false);
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
}

static void update_follow_from_traffic() {
    if (!s_follow || !s_follow_hex[0]) {
        return;
    }
    const Aircraft *hit = nullptr;
    for (int i = 0; i < s_ac_n; i++) {
        if (strcasecmp(s_ac[i].hex, s_follow_hex) == 0) {
            hit = &s_ac[i];
            break;
        }
    }
    if (!hit) {
        if (radar_loc_lbl && s_follow_label[0]) {
            char buf[128];
            snprintf(buf, sizeof(buf), "Following %s (lost)\n%.4f, %.4f", s_follow_label, s_olat, s_olon);
            lv_label_set_text(radar_loc_lbl, buf);
        }
        refresh_follow_panel();
        return;
    }
    s_follow_ac = *hit;
    s_follow_ac_ok = true;
    if (hit->flight[0]) {
        strncpy(s_follow_label, hit->flight, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
        trim_inplace(s_follow_label);
    }
    trail_push(hit->lat, hit->lon);
    hist_push(hit->on_ground ? 0.0f : hit->alt_ft, hit->gs_kts);

    float east = 0, north = 0;
    ll_to_nm(hit->lat, hit->lon, s_olat, s_olon, &east, &north);
    const float moved_nm = hypotf(east, north);
    char label[48];
    snprintf(label, sizeof(label), "Following %s", s_follow_label[0] ? s_follow_label : s_follow_hex);
    // Recenter in memory so the next poll tracks the plane; refresh airports if it drifted.
    const uint32_t req = (moved_nm > 8.0f) ? UI_REQ_REFRESH_AIRPORTS : 0;
    if (moved_nm > 0.05f || req) {
        set_radar_center(hit->lat, hit->lon, label, req);
    } else {
        update_loc_label(label, hit->lat, hit->lon);
    }
    refresh_follow_panel();
}

// 0=light/GA, 1=medium airliner, 2=heavy
static int aircraft_size_class(const Aircraft *ac) {
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

// Geographic sketch is rasterized into PSRAM off the RGB framebuffers so heavy
// coastline drawing does not contend with panel DMA (flicker/hop on redraw).
static lv_color_t *s_map_cache = nullptr;
static lv_coord_t s_map_cw = 0;
static lv_coord_t s_map_ch = 0;
static float s_map_clat = 1e9f;
static float s_map_clon = 1e9f;
static float s_map_crange = -1.0f;
static bool s_map_ready = false;
static lv_img_dsc_t s_map_img{};

static inline void cache_put(lv_color_t *buf, int w, int h, int x, int y, lv_color_t c) {
    if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) {
        return;
    }
    buf[y * w + x] = c;
}

static void cache_line(lv_color_t *buf, int w, int h, int x0, int y0, int x1, int y1, int cx, int cy, int64_t R2,
                       lv_color_t c) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        const int64_t ddx = (int64_t)x0 - cx;
        const int64_t ddy = (int64_t)y0 - cy;
        if (ddx * ddx + ddy * ddy <= R2) {
            cache_put(buf, w, h, x0, y0, c);
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
                             uint16_t poly_count, lv_color_t ink) {
    for (uint16_t pi = 0; pi < poly_count; pi++) {
        const PolyT &poly = polys[pi];
        if (poly.count < 2) {
            continue;
        }
        const PtT &a0 = pts[poly.start];
        const PtT &a1 = pts[poly.start + poly.count / 2];
        const PtT &a2 = pts[poly.start + poly.count - 1];
        const int near0 = (abs((int)a0.lat_q - lat0) <= pad + 20 && abs((int)a0.lon_q - lon0) <= pad + 20);
        const int near1 = (abs((int)a1.lat_q - lat0) <= pad + 20 && abs((int)a1.lon_q - lon0) <= pad + 20);
        const int near2 = (abs((int)a2.lat_q - lat0) <= pad + 20 && abs((int)a2.lon_q - lon0) <= pad + 20);
        if (!near0 && !near1 && !near2) {
            continue;
        }

        bool have_prev = false;
        lv_coord_t px = 0, py = 0;
        bool prev_in = false;
        for (uint16_t k = 0; k < poly.count; k++) {
            const PtT &pt = pts[poly.start + k];
            if (abs((int)pt.lat_q - lat0) > pad + 8 || abs((int)pt.lon_q - lon0) > pad + 8) {
                have_prev = false;
                continue;
            }
            lv_coord_t x, y;
            map_point_ll(pt.lat_q / 50.0f, pt.lon_q / 50.0f, olat, olon, range, w, h, &x, &y);
            const int64_t dx = (int64_t)x - cx;
            const int64_t dy = (int64_t)y - cy;
            const bool inside = (dx * dx + dy * dy) <= R2;
            if (have_prev && (inside || prev_in)) {
                cache_line(buf, w, h, px, py, x, y, cx, cy, R2, ink);
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
    if (s_map_ready && s_map_cache && s_map_cw == w && s_map_ch == h && fabsf(s_map_clat - olat) < 1e-5f &&
        fabsf(s_map_clon - olon) < 1e-5f && fabsf(s_map_crange - range) < 0.05f) {
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
    cache_draw_polys(s_map_cache, w, h, olat, olon, range, cx, cy, R2, pad, lat0, lon0, kMapPts, kMapPolys,
                     kMapPolyCount, COL_MAP);

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
    s_map_ready = true;
    return true;
}

static void snapshot_radar() {
    if (!g_state || xSemaphoreTake(g_state->mutex, 0) != pdTRUE) {
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
    xSemaphoreGive(g_state->mutex);
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

static void layout_detail_body(bool with_side_panel) {
    if (!detail_body) {
        return;
    }
    if (with_side_panel) {
        lv_obj_set_width(detail_body, 400);
        lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, 300, 48);
    } else {
        lv_obj_set_width(detail_body, 680);
        lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, 12, 48);
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
    if (s_plate_rwy_n > 0) {
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
            lv_label_set_text(lab, same ? "Stop follow" : "Follow Flight");
        }
    } else {
        lv_obj_add_flag(detail_follow_btn, LV_OBJ_FLAG_HIDDEN);
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
    update_detail_center_btn();
    update_detail_follow_btn();
    memset(&s_flight_info, 0, sizeof(s_flight_info));
}

static void center_on_airport(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_detail_apt_ok || !g_state) {
        return;
    }

    stop_follow(false);

    GeoFix fix{};
    if (s_detail_apt.name[0]) {
        strncpy(fix.label, s_detail_apt.name, sizeof(fix.label) - 1);
    } else {
        strncpy(fix.label, s_detail_apt.icao, sizeof(fix.label) - 1);
    }
    fix.label[sizeof(fix.label) - 1] = 0;
    strncpy(fix.query, s_detail_apt.icao, sizeof(fix.query) - 1);
    fix.query[sizeof(fix.query) - 1] = 0;
    fix.lat = s_detail_apt.lat;
    fix.lon = s_detail_apt.lon;
    fix.valid = true;

    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.location = fix;
    xSemaphoreGive(g_state->mutex);

    s_olat = fix.lat;
    s_olon = fix.lon;
    map_cache_mark_dirty();
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);
    update_loc_label(fix.label, fix.lat, fix.lon);

    hide_detail(nullptr);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    request(UI_REQ_SAVE_LOCATION | UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
}

static void follow_flight(lv_event_t *e) {
    LV_UNUSED(e);
    if (!g_state || !s_detail_is_aircraft || !s_detail_ac.hex[0]) {
        return;
    }

    if (s_follow && strcasecmp(s_follow_hex, s_detail_ac.hex) == 0) {
        stop_follow(true);
        hide_detail(nullptr);
        if (radar_plot) {
            lv_obj_invalidate(radar_plot);
        }
        return;
    }

    if (!s_follow) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        s_follow_saved_loc = g_state->cfg.location;
        xSemaphoreGive(g_state->mutex);
        s_follow_saved_ok = s_follow_saved_loc.valid;
    }

    s_follow = true;
    s_follow_solo = true;
    strncpy(s_follow_hex, s_detail_ac.hex, sizeof(s_follow_hex) - 1);
    s_follow_hex[sizeof(s_follow_hex) - 1] = 0;
    if (s_detail_ac.flight[0]) {
        strncpy(s_follow_label, s_detail_ac.flight, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
        trim_inplace(s_follow_label);
    } else {
        strncpy(s_follow_label, s_follow_hex, sizeof(s_follow_label) - 1);
        s_follow_label[sizeof(s_follow_label) - 1] = 0;
    }
    s_follow_ac = s_detail_ac;
    s_follow_ac_ok = true;
    s_follow_fi = s_flight_info;

    trail_clear();
    hist_clear();
    trail_push(s_detail_ac.lat, s_detail_ac.lon);
    hist_push(s_detail_ac.on_ground ? 0.0f : s_detail_ac.alt_ft, s_detail_ac.gs_kts);
    update_solo_toggle_btn();
    set_follow_side_mode(true);
    refresh_follow_panel();

    char label[48];
    snprintf(label, sizeof(label), "Following %s", s_follow_label);
    set_radar_center(s_detail_ac.lat, s_detail_ac.lon, label,
                     UI_REQ_REFRESH_TRAFFIC | UI_REQ_REFRESH_AIRPORTS);
    map_cache_ensure(kRadarPlot, kRadarPlot, s_olat, s_olon, s_range);

    // Keep route/type metadata fresh for the follow side panel.
    if (g_state && (s_detail_ac.flight[0] || s_detail_ac.reg[0] || s_detail_ac.hex[0])) {
        xSemaphoreTake(g_state->mutex, portMAX_DELAY);
        strncpy(g_state->pending_flight, s_detail_ac.flight, sizeof(g_state->pending_flight) - 1);
        g_state->pending_flight[sizeof(g_state->pending_flight) - 1] = 0;
        strncpy(g_state->pending_reg, s_detail_ac.reg, sizeof(g_state->pending_reg) - 1);
        g_state->pending_reg[sizeof(g_state->pending_reg) - 1] = 0;
        strncpy(g_state->pending_hex, s_detail_ac.hex, sizeof(g_state->pending_hex) - 1);
        g_state->pending_hex[sizeof(g_state->pending_hex) - 1] = 0;
        xSemaphoreGive(g_state->mutex);
        request(UI_REQ_FETCH_FLIGHT_INFO);
    }

    hide_detail(nullptr);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
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

    snapshot_radar();
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

    // Simulated PPI sweep: bright leading edge with a short, quickly fading trail.
    {
        lv_draw_line_dsc_t sw;
        lv_draw_line_dsc_init(&sw);
        sw.color = COL_GREEN;
        sw.round_start = 1;
        sw.round_end = 1;
        constexpr int kTrail = 16;
        constexpr float kSpanDeg = 26.0f;
        for (int i = kTrail; i >= 0; i--) {
            const float t = (float)i / (float)kTrail;  // 1 = oldest, 0 = tip
            const float ang = (s_sweep_deg - t * kSpanDeg) * FR_DEG2RAD;
            const float sn = sinf(ang);
            const float cs = cosf(ang);
            lv_point_t a = {cx, cy};
            lv_point_t b = {(lv_coord_t)lroundf((float)cx + (float)R * sn),
                            (lv_coord_t)lroundf((float)cy - (float)R * cs)};
            const float fade = (1.0f - t) * (1.0f - t);  // quadratic → dies off fast
            sw.opa = (lv_opa_t)lroundf(16.0f + 170.0f * fade);
            sw.width = (i == 0) ? 2 : 1;
            lv_draw_line(ctx, &sw, &a, &b);
        }
    }

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

    if (s_show_airports) {
        for (int i = 0; i < s_apt_n; i++) {
            const Airport *a = &s_apt[i];
            lv_coord_t x, y;
            if (!map_point_ll(a->lat, a->lon, s_olat, s_olon, s_range, w, h, &x, &y)) {
                continue;
            }
            x += coords.x1;
            y += coords.y1;
            lv_draw_rect_dsc_t mark;
            lv_draw_rect_dsc_init(&mark);
            mark.bg_color = airport_color(a);
            mark.bg_opa = LV_OPA_COVER;
            mark.radius = 2;
            const lv_coord_t s = a->iata[0] ? 7 : 5;
            lv_area_t m = {(lv_coord_t)(x - s / 2), (lv_coord_t)(y - s / 2), (lv_coord_t)(x + s / 2),
                           (lv_coord_t)(y + s / 2)};
            lv_draw_rect(ctx, &mark, &m);
            const char *code = a->iata[0] ? a->iata : a->icao;
            lv_area_t ta = {(lv_coord_t)(x + 6), (lv_coord_t)(y - 8), (lv_coord_t)(x + 70), (lv_coord_t)(y + 10)};
            lab.color = airport_color(a);
            lv_draw_label(ctx, &lab, &ta, code, nullptr);
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
        draw_airplane_icon(ctx, x, y, ac->track_deg, aircraft_size_class(ac), COL_AC);
        const char *id = ac->flight[0] ? ac->flight : ac->hex;
        lv_area_t ta = {(lv_coord_t)(x + 10), (lv_coord_t)(y - 6), (lv_coord_t)(x + 90), (lv_coord_t)(y + 12)};
        lab.color = COL_AC;
        lv_draw_label(ctx, &lab, &ta, id, nullptr);
    }

    static const char *rose[] = {"N", "E", "S", "W"};
    lv_point_t rp[4] = {{cx, (lv_coord_t)(cy - R + 4)}, {(lv_coord_t)(cx + R - 14), cy}, {cx, (lv_coord_t)(cy + R - 16)}, {(lv_coord_t)(cx - R + 4), cy}};
    lab.color = COL_GREEN;
    lab.font = &lv_font_montserrat_14;
    for (int i = 0; i < 4; i++) {
        lv_area_t ta = {(lv_coord_t)(rp[i].x - 8), (lv_coord_t)(rp[i].y - 8), (lv_coord_t)(rp[i].x + 16), (lv_coord_t)(rp[i].y + 12)};
        lv_draw_label(ctx, &lab, &ta, rose[i], nullptr);
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
    char dep_line[72] = "--";
    char arr_line[72] = "--";
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
    snprintf(body, blen,
             "Flight  %s\n"
             "Tail    %s\n"
             "Route   %s\n"
             "Dep     %s\n"
             "Arr     %s\n"
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
             arr_line,
             ac->hex,
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
    char title[32], body[640];
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

static void radar_clicked(lv_event_t *e) {
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
        open_aircraft_detail(&s_ac[best_ac]);
        return;
    }
    if (best_apt >= 0) {
        const Airport *a = &s_apt[best_apt];
        s_detail_apt = *a;
        s_detail_apt_ok = true;
        strncpy(g_state->pending_icao, a->icao, sizeof(g_state->pending_icao) - 1);
        char title[40], body[420];
        snprintf(title, sizeof(title), "%s  %s", a->icao, a->iata);
        snprintf(body, sizeof(body),
                 "%s\n%s %s\nElev %d ft\nRwy  %d ft\nPos  %.4f, %.4f\n\nLoading METAR / TAF...",
                 a->name, a->state, a->country, a->elev_ft, a->longest_rwy_ft, a->lat, a->lon);
        show_detail(title, body, a->icao);
        request(UI_REQ_FETCH_WEATHER);
        return;
    }
}

static void set_range(float nm) {
    if (nm < 10) {
        nm = 10;
    }
    if (nm > 250) {
        nm = 250;
    }
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const float old = g_state->cfg.range_nm;
    if (fabsf(old - nm) < 0.5f) {
        xSemaphoreGive(g_state->mutex);
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
    lv_label_set_text(radar_range_lbl, buf);
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    // NVS from the LVGL thread can stall/WDT; save on the net task.
    // Zooming in keeps the current airport set (draw already clips by range).
    uint32_t req = UI_REQ_REFRESH_TRAFFIC | UI_REQ_SAVE_RANGE;
    if (nm > old + 0.5f) {
        req |= UI_REQ_REFRESH_AIRPORTS;
    }
    request(req);
}

static void zoom_out(lv_event_t *e) {
    LV_UNUSED(e);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const float range = g_state->cfg.range_nm;
    xSemaphoreGive(g_state->mutex);
    int i = zoom_index(range);
    if (i < kZoomN - 1) {
        set_range(kZoom[i + 1]);
    }
}

static void zoom_in(lv_event_t *e) {
    LV_UNUSED(e);
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    const float range = g_state->cfg.range_nm;
    xSemaphoreGive(g_state->mutex);
    int i = zoom_index(range);
    if (i > 0) {
        set_range(kZoom[i - 1]);
    }
}

static void set_max_alt(int alt_ft) {
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
    request(UI_REQ_SAVE_MAX_ALT);
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

static void hide_wifi(lv_event_t *e) {
    LV_UNUSED(e);
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void hide_location(lv_event_t *e) {
    LV_UNUSED(e);
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
}

static void hide_settings(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void open_settings(lv_event_t *e) {
    LV_UNUSED(e);
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
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (settings_panel) {
        lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(settings_panel);
    }
}

static void update_apt_toggle_label() {
    if (!apt_toggle_btn) {
        return;
    }
    lv_obj_t *lbl = lv_obj_get_child(apt_toggle_btn, 0);
    if (lbl) {
        lv_label_set_text(lbl, s_show_airports ? "Hide Apt" : "Show Apt");
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
    // Airports are drawn live; map disk cache unchanged.
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
    request(UI_REQ_SAVE_SHOW_AIRPORTS);
}

static void toggle_follow_solo(lv_event_t *e) {
    LV_UNUSED(e);
    if (!s_follow) {
        return;
    }
    s_follow_solo = !s_follow_solo;
    update_solo_toggle_btn();
    refresh_traffic_table();
    if (radar_plot) {
        lv_obj_invalidate(radar_plot);
    }
}

static void goto_wifi(lv_event_t *e) {
    LV_UNUSED(e);
    ui_show_wifi();
    request(UI_REQ_WIFI_SCAN);
}

static void goto_loc(lv_event_t *e) {
    LV_UNUSED(e);
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    ui_show_location();
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
    lv_textarea_set_text(wifi_ta, "");
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
    if (g_state->pending_query[0] == 0) {
        lv_label_set_text(loc_status, "Enter a zip, city, or address");
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
    xSemaphoreTake(g_state->mutex, portMAX_DELAY);
    g_state->cfg.location = pick;
    xSemaphoreGive(g_state->mutex);
    hide_location(nullptr);
    ui_show_radar();
    request(UI_REQ_SAVE_LOCATION | UI_REQ_REFRESH_AIRPORTS | UI_REQ_REFRESH_TRAFFIC);
}

static void build_location() {
    loc_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(loc_panel, 760, 460);
    lv_obj_center(loc_panel);
    lv_obj_set_style_bg_color(loc_panel, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(loc_panel, COL_GREEN, 0);
    lv_obj_set_style_radius(loc_panel, 12, 0);
    lv_obj_set_style_pad_all(loc_panel, 8, 0);
    lv_obj_clear_flag(loc_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(loc_panel);
    lv_label_set_text(title, "Watch location");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, COL_GREEN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *lclose = make_btn(loc_panel, "Close", hide_location, nullptr);
    lv_obj_set_size(lclose, 90, 36);
    lv_obj_align(lclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *hint = lv_label_create(loc_panel);
    lv_label_set_text(hint, "Zip code, city, or street address. Tap a result to center the radar.");
    lv_obj_set_style_text_color(hint, COL_TEXT, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 8, 44);

    loc_ta = lv_textarea_create(loc_panel);
    lv_obj_set_size(loc_ta, 520, 44);
    lv_obj_align(loc_ta, LV_ALIGN_TOP_LEFT, 8, 72);
    lv_textarea_set_placeholder_text(loc_ta, "e.g. 37203  or  Nashville, TN  or  1 World Way, Los Angeles");
    lv_textarea_set_one_line(loc_ta, true);
    lv_obj_add_event_cb(loc_ta, loc_ta_clicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(loc_ta, loc_ta_clicked, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *search = make_btn(loc_panel, "Search", loc_search_clicked, nullptr);
    lv_obj_set_size(search, 180, 44);
    lv_obj_align(search, LV_ALIGN_TOP_RIGHT, -8, 72);

    loc_list = lv_list_create(loc_panel);
    lv_obj_set_size(loc_list, 728, 140);
    lv_obj_align(loc_list, LV_ALIGN_TOP_MID, 0, 124);
    lv_obj_set_style_bg_color(loc_list, COL_PANEL, 0);
    lv_obj_set_style_border_color(loc_list, COL_DIM, 0);

    loc_status = lv_label_create(loc_panel);
    lv_label_set_text(loc_status, "");
    lv_obj_set_style_text_color(loc_status, COL_GREEN, 0);
    lv_obj_align(loc_status, LV_ALIGN_TOP_LEFT, 8, 270);

    // Screen-level keyboard (same pattern as Wi-Fi) so it is not clipped by the panel.
    loc_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(loc_kb, 800, 190);
    lv_obj_align(loc_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(loc_kb, loc_ta);
    lv_obj_add_flag(loc_kb, LV_OBJ_FLAG_HIDDEN);
}

static void traffic_table_selected(lv_event_t *e);
static void refresh_traffic_table();
static bool radar_overlay_open();

static void sweep_timer_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    // ~4 s per revolution; fade trail is angular, not time-based.
    s_sweep_deg += 5.0f;
    if (s_sweep_deg >= 360.0f) {
        s_sweep_deg -= 360.0f;
    }
    if (!radar_plot || lv_scr_act() != scr_radar) {
        return;
    }
    if (radar_overlay_open()) {
        return;
    }
    lv_obj_invalidate(radar_plot);
}

static void follow_chart_draw(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const lv_coord_t w = lv_area_get_width(&coords);
    const lv_coord_t h = lv_area_get_height(&coords);
    const lv_coord_t pad_l = 8;
    const lv_coord_t pad_r = 8;
    const lv_coord_t pad_t = 22;
    const lv_coord_t pad_b = 18;
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

    // Bottom = 0 AGL / 0 kt.
    // Alt top = max + 10% (~90% height). Speed top = max / 0.7 (peak at 70% height).
    float amax_rec = 0.0f, smax_rec = 0.0f;
    const int oldest = (s_hist_head - s_hist_count + kHistMax) % kHistMax;
    for (int i = 0; i < s_hist_count; i++) {
        const HistPt &pt = s_hist[(oldest + i) % kHistMax];
        if (pt.alt_ft > amax_rec) {
            amax_rec = pt.alt_ft;
        }
        if (pt.gs_kts > smax_rec) {
            smax_rec = pt.gs_kts;
        }
    }
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

    lv_draw_label_dsc_t lab;
    lv_draw_label_dsc_init(&lab);
    lab.font = &lv_font_montserrat_12;
    char alt_lbl[40], spd_lbl[40];
    if (s_hist_count >= 2) {
        snprintf(alt_lbl, sizeof(alt_lbl), "ALT %.0f", a_top);
        snprintf(spd_lbl, sizeof(spd_lbl), "SPD %.0f", s_top);
    } else {
        strncpy(alt_lbl, "ALT ft", sizeof(alt_lbl) - 1);
        alt_lbl[sizeof(alt_lbl) - 1] = 0;
        strncpy(spd_lbl, "SPD kt", sizeof(spd_lbl) - 1);
        spd_lbl[sizeof(spd_lbl) - 1] = 0;
    }
    lab.color = COL_GREEN;
    lv_area_t ta = {(lv_coord_t)(coords.x1 + 8), (lv_coord_t)(coords.y1 + 4),
                    (lv_coord_t)(coords.x1 + 160), (lv_coord_t)(coords.y1 + 20)};
    lv_draw_label(ctx, &lab, &ta, alt_lbl, nullptr);
    lab.color = COL_SPD;
    ta.x1 = coords.x1 + 150;
    ta.x2 = coords.x1 + 300;
    lv_draw_label(ctx, &lab, &ta, spd_lbl, nullptr);

    if (s_hist_count < 2) {
        lab.color = COL_TEXT;
        ta.x1 = gx1;
        ta.y1 = gy1 + gh / 2 - 8;
        ta.x2 = gx2;
        ta.y2 = ta.y1 + 16;
        lv_draw_label(ctx, &lab, &ta, "Collecting samples...", nullptr);
        return;
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

    lv_coord_t prev_ax = 0, prev_ay = 0, prev_sx = 0, prev_sy = 0;
    for (int i = 0; i < s_hist_count; i++) {
        const HistPt &pt = s_hist[(oldest + i) % kHistMax];
        const float xf = (s_hist_count == 1) ? 0.0f : (float)i / (float)(s_hist_count - 1);
        const lv_coord_t x = gx1 + (lv_coord_t)lroundf(xf * (float)gw);
        const lv_coord_t ay = y_of(pt.alt_ft, a_top);
        const lv_coord_t sy = y_of(pt.gs_kts, s_top);
        if (i > 0) {
            lv_point_t a0 = {prev_ax, prev_ay};
            lv_point_t a1 = {x, ay};
            lv_draw_line(ctx, &alt_ln, &a0, &a1);
            lv_point_t s0 = {prev_sx, prev_sy};
            lv_point_t s1 = {x, sy};
            lv_draw_line(ctx, &spd_ln, &s0, &s1);
        }
        prev_ax = x;
        prev_ay = ay;
        prev_sx = x;
        prev_sy = sy;
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
}

static void refresh_follow_panel() {
    if (!follow_info_lbl) {
        return;
    }
    if (!s_follow) {
        return;
    }
    char body[420];
    if (!s_follow_ac_ok) {
        snprintf(body, sizeof(body), "Following %s\n\nWaiting for ADS-B update...",
                 s_follow_label[0] ? s_follow_label : s_follow_hex);
        lv_label_set_text(follow_info_lbl, body);
        if (follow_chart) {
            lv_obj_invalidate(follow_chart);
        }
        return;
    }

    const Aircraft *ac = &s_follow_ac;
    const FlightInfo *fi = &s_follow_fi;
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
    char route[48] = "--";
    if (fi->route_ok && fi->origin[0] && fi->dest[0]) {
        snprintf(route, sizeof(route), "%s → %s", fi->origin, fi->dest);
    }
    const char *flight = ac->flight[0] ? ac->flight : (s_follow_label[0] ? s_follow_label : ac->hex);
    snprintf(body, sizeof(body),
             "%s   %s\n"
             "%s\n"
             "Route  %s\n"
             "Alt    %s     Spd  %.0f kt\n"
             "Track  %.0f°     V/S  %+.0f fpm\n"
             "Squawk %s\n"
             "Pos    %.4f, %.4f",
             flight, reg, typ, route, alt, ac->gs_kts, ac->track_deg, ac->vrate_fpm,
             ac->squawk[0] ? ac->squawk : "--", ac->lat, ac->lon);
    lv_label_set_text(follow_info_lbl, body);
    if (follow_chart) {
        lv_obj_invalidate(follow_chart);
    }
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
    lv_obj_add_event_cb(radar_plot, radar_clicked, LV_EVENT_CLICKED, nullptr);

    // Title / location / ADS-B status sit in the square corners (outside the circular PPI).
    lv_obj_t *hdr = lv_label_create(scr_radar);
    lv_label_set_text(hdr, "FLIGHT RADAR");
    lv_label_set_long_mode(hdr, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hdr, 100);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(hdr, COL_GREEN, 0);
    lv_obj_align_to(hdr, radar_plot, LV_ALIGN_TOP_LEFT, 10, 8);

    radar_loc_lbl = lv_label_create(scr_radar);
    lv_label_set_long_mode(radar_loc_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radar_loc_lbl, 220);
    lv_obj_set_style_text_align(radar_loc_lbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(radar_loc_lbl, COL_TEXT, 0);
    lv_obj_set_style_text_font(radar_loc_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align_to(radar_loc_lbl, radar_plot, LV_ALIGN_TOP_RIGHT, -10, 8);

    radar_status = lv_label_create(scr_radar);
    lv_label_set_long_mode(radar_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radar_status, 240);
    lv_obj_set_style_text_align(radar_status, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(radar_status, COL_TEXT, 0);
    lv_obj_set_style_text_font(radar_status, &lv_font_montserrat_12, 0);
    lv_obj_align_to(radar_status, radar_plot, LV_ALIGN_BOTTOM_RIGHT, -10, -8);

    radar_count_lbl = lv_label_create(scr_radar);
    lv_label_set_long_mode(radar_count_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radar_count_lbl, 220);
    lv_obj_set_style_text_color(radar_count_lbl, COL_AC, 0);
    lv_obj_set_style_text_font(radar_count_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align_to(radar_count_lbl, radar_plot, LV_ALIGN_BOTTOM_LEFT, 10, -8);

    lv_obj_t *side = lv_obj_create(scr_radar);
    lv_obj_set_size(side, kSideW, kSideH);
    lv_obj_align(side, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(side, COL_PANEL, 0);
    lv_obj_set_style_border_color(side, COL_DIM, 0);
    lv_obj_set_style_radius(side, 10, 0);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(side, 12, 0);
    lv_obj_set_style_pad_row(side, 8, 0);
    // Scrolling steals taps from ZOOM +/- ("sometimes" no click).
    lv_obj_clear_flag(side, LV_OBJ_FLAG_SCROLLABLE);

    // Compact zoom / altitude + apt / settings stack pinned to bottom of side rail.
    // Leave a clear gap between value boxes (left) and button pairs (right).
    const lv_coord_t ctrl_w = 360;
    const lv_coord_t btn_w = 112;
    const lv_coord_t btn_h = 36;
    const lv_coord_t row_h = 40;
    const lv_coord_t val_w = btn_w;
    const lv_coord_t val_h = btn_h;
    const lv_coord_t controls_h = 180;
    const lv_coord_t table_h = kSideH - 24 - controls_h - 8;

    traffic_table = lv_table_create(side);
    lv_obj_set_size(traffic_table, ctrl_w, table_h);
    lv_obj_align(traffic_table, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(traffic_table, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_clear_flag(traffic_table, LV_OBJ_FLAG_SCROLLABLE);
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
    lv_obj_set_style_pad_top(traffic_table, 3, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(traffic_table, 3, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(traffic_table, 3, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(traffic_table, 3, LV_PART_ITEMS);
    lv_table_set_col_cnt(traffic_table, 6);
    lv_table_set_row_cnt(traffic_table, 1);
    // Tail / Flight / Dist / Alt / Spd / Type — fill ctrl_w.
    lv_table_set_col_width(traffic_table, 0, 66);
    lv_table_set_col_width(traffic_table, 1, 66);
    lv_table_set_col_width(traffic_table, 2, 46);
    lv_table_set_col_width(traffic_table, 3, 78);
    lv_table_set_col_width(traffic_table, 4, 46);
    lv_table_set_col_width(traffic_table, 5, 58);
    lv_table_set_cell_value(traffic_table, 0, 0, "TAIL");
    lv_table_set_cell_value(traffic_table, 0, 1, "FLIGHT");
    lv_table_set_cell_value(traffic_table, 0, 2, "DIST");
    lv_table_set_cell_value(traffic_table, 0, 3, "ALT");
    lv_table_set_cell_value(traffic_table, 0, 4, "SPD");
    lv_table_set_cell_value(traffic_table, 0, 5, "TYPE");
    lv_obj_add_event_cb(traffic_table, traffic_table_selected, LV_EVENT_VALUE_CHANGED, nullptr);

    // Follow mode: single-flight info + alt/speed chart (replaces traffic table).
    const lv_coord_t follow_info_h = 168;
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
    lv_obj_set_size(follow_info_lbl, ctrl_w, follow_info_h);
    lv_obj_align(follow_info_lbl, LV_ALIGN_TOP_MID, 0, 0);
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

    lv_obj_t *controls = lv_obj_create(side);
    lv_obj_set_size(controls, ctrl_w, controls_h);
    lv_obj_set_style_bg_opa(controls, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(controls, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(controls, 0, 0);
    lv_obj_set_style_pad_row(controls, 12, 0);
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
    auto make_value_box = [&](lv_obj_t *parent) {
        lv_obj_t *box = lv_obj_create(parent);
        lv_obj_set_size(box, val_w, val_h);
        lv_obj_set_style_bg_color(box, lv_color_hex(0x0A120C), 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, COL_DIM, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 6, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_align(box, LV_ALIGN_LEFT_MID, 0, 0);
        return box;
    };
    auto shrink_btn_label = [](lv_obj_t *btn) {
        lv_obj_t *lab = lv_obj_get_child(btn, 0);
        if (lab) {
            lv_obj_set_style_text_font(lab, &lv_font_montserrat_12, 0);
        }
    };
    auto make_press_ctrl = [&](lv_obj_t *parent, const char *txt, lv_event_cb_t cb) {
        lv_obj_t *btn = make_btn(parent, txt, cb, nullptr);
        lv_obj_set_size(btn, btn_w, btn_h);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_remove_event_cb(btn, cb);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, nullptr);
        shrink_btn_label(btn);
        return btn;
    };

    lv_obj_t *zoom_row = make_ctrl_row(controls);
    lv_obj_t *range_box = make_value_box(zoom_row);
    radar_range_lbl = lv_label_create(range_box);
    lv_obj_set_style_text_font(radar_range_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radar_range_lbl, COL_GREEN, 0);
    lv_obj_center(radar_range_lbl);
    lv_obj_t *zin = make_press_ctrl(zoom_row, "ZOOM +", zoom_in);
    lv_obj_align(zin, LV_ALIGN_RIGHT_MID, -(btn_w + 6), 0);
    lv_obj_t *zout = make_press_ctrl(zoom_row, "ZOOM -", zoom_out);
    lv_obj_align(zout, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *alt_row = make_ctrl_row(controls);
    lv_obj_t *alt_box = make_value_box(alt_row);
    radar_alt_lbl = lv_label_create(alt_box);
    lv_obj_set_style_text_font(radar_alt_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radar_alt_lbl, COL_GREEN, 0);
    lv_obj_center(radar_alt_lbl);
    lv_obj_t *aup = make_press_ctrl(alt_row, "ALT +", alt_up);
    lv_obj_align(aup, LV_ALIGN_RIGHT_MID, -(btn_w + 6), 0);
    lv_obj_t *adn = make_press_ctrl(alt_row, "ALT -", alt_down);
    lv_obj_align(adn, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *btn_row = make_ctrl_row(controls);
    solo_toggle_btn = make_btn(btn_row, "Hide Ac", toggle_follow_solo, nullptr);
    lv_obj_set_size(solo_toggle_btn, btn_w, btn_h);
    lv_obj_set_style_radius(solo_toggle_btn, 6, 0);
    lv_obj_clear_flag(solo_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(solo_toggle_btn);
    lv_obj_align(solo_toggle_btn, LV_ALIGN_RIGHT_MID, -2 * (btn_w + 6), 0);
    lv_obj_add_flag(solo_toggle_btn, LV_OBJ_FLAG_HIDDEN);

    apt_toggle_btn = make_btn(btn_row, "Hide Apt", toggle_airports, nullptr);
    lv_obj_set_size(apt_toggle_btn, btn_w, btn_h);
    lv_obj_set_style_radius(apt_toggle_btn, 6, 0);
    lv_obj_clear_flag(apt_toggle_btn, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(apt_toggle_btn);
    lv_obj_align(apt_toggle_btn, LV_ALIGN_RIGHT_MID, -(btn_w + 6), 0);

    lv_obj_t *settings = make_btn(btn_row, "Settings", open_settings, nullptr);
    lv_obj_set_size(settings, btn_w, btn_h);
    lv_obj_set_style_radius(settings, 6, 0);
    lv_obj_clear_flag(settings, LV_OBJ_FLAG_SCROLL_CHAIN);
    shrink_btn_label(settings);
    lv_obj_align(settings, LV_ALIGN_RIGHT_MID, 0, 0);

    settings_panel = lv_obj_create(scr_radar);
    lv_obj_set_size(settings_panel, 360, 200);
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

    lv_obj_t *sloc = make_btn(settings_panel, "Change location", goto_loc, nullptr);
    lv_obj_set_size(sloc, 320, 44);
    lv_obj_align(sloc, LV_ALIGN_TOP_MID, 0, 56);

    lv_obj_t *swifi = make_btn(settings_panel, "Wi-Fi setup", goto_wifi, nullptr);
    lv_obj_set_size(swifi, 320, 44);
    lv_obj_align(swifi, LV_ALIGN_TOP_MID, 0, 112);

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
    lv_label_set_text(wtitle, "Wi-Fi setup");
    lv_obj_set_style_text_font(wtitle, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(wtitle, COL_GREEN, 0);
    lv_obj_align(wtitle, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *wclose = make_btn(wifi_panel, "Close", hide_wifi, nullptr);
    lv_obj_set_size(wclose, 90, 36);
    lv_obj_align(wclose, LV_ALIGN_TOP_RIGHT, -4, 4);

    lv_obj_t *whint = lv_label_create(wifi_panel);
    lv_label_set_text(whint, "Tap a network, enter the password, then Connect.");
    lv_obj_set_style_text_color(whint, COL_TEXT, 0);
    lv_obj_set_style_text_font(whint, &lv_font_montserrat_14, 0);
    lv_obj_align(whint, LV_ALIGN_TOP_LEFT, 12, 44);

    wifi_list = lv_list_create(wifi_panel);
    lv_obj_set_size(wifi_list, 480, 300);
    lv_obj_align(wifi_list, LV_ALIGN_TOP_LEFT, 12, 72);
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
    lv_textarea_set_password_mode(wifi_ta, true);
    lv_textarea_set_one_line(wifi_ta, true);

    wifi_status = lv_label_create(wifi_panel);
    lv_label_set_text(wifi_status, "Not connected");
    lv_obj_set_style_text_color(wifi_status, COL_GREEN, 0);
    lv_label_set_long_mode(wifi_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(wifi_status, 220);
    lv_obj_align(wifi_status, LV_ALIGN_TOP_RIGHT, -12, 252);

    wifi_kb = lv_keyboard_create(scr_radar);
    lv_obj_set_size(wifi_kb, 800, 190);
    lv_obj_align(wifi_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(wifi_kb, wifi_ta);
    lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);

    detail = lv_obj_create(scr_radar);
    lv_obj_set_size(detail, 740, 430);
    lv_obj_center(detail);
    lv_obj_set_style_bg_color(detail, lv_color_hex(0x101810), 0);
    lv_obj_set_style_border_color(detail, COL_GREEN, 0);
    lv_obj_set_style_radius(detail, 12, 0);
    lv_obj_set_style_pad_all(detail, 8, 0);
    lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(detail, LV_OBJ_FLAG_SCROLLABLE);

    detail_title = lv_label_create(detail);
    lv_obj_set_style_text_font(detail_title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(detail_title, COL_GREEN, 0);
    lv_obj_align(detail_title, LV_ALIGN_TOP_LEFT, 8, 4);

    lv_obj_t *closeb = make_btn(detail, "Close", hide_detail, nullptr);
    lv_obj_set_size(closeb, 90, 36);
    lv_obj_align(closeb, LV_ALIGN_TOP_RIGHT, -4, 4);

    detail_center_btn = make_btn(detail, "Center on airport", center_on_airport, nullptr);
    lv_obj_set_size(detail_center_btn, 220, 36);
    lv_obj_align(detail_center_btn, LV_ALIGN_TOP_RIGHT, -100, 4);
    lv_obj_add_flag(detail_center_btn, LV_OBJ_FLAG_HIDDEN);

    detail_follow_btn = make_btn(detail, "Follow Flight", follow_flight, nullptr);
    lv_obj_set_size(detail_follow_btn, 160, 36);
    lv_obj_align(detail_follow_btn, LV_ALIGN_TOP_RIGHT, -100, 4);
    lv_obj_add_flag(detail_follow_btn, LV_OBJ_FLAG_HIDDEN);

    detail_plate = lv_obj_create(detail);
    lv_obj_set_size(detail_plate, 280, 330);
    lv_obj_align(detail_plate, LV_ALIGN_BOTTOM_LEFT, 4, -4);
    lv_obj_set_style_bg_opa(detail_plate, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(detail_plate, 0, 0);
    lv_obj_set_style_pad_all(detail_plate, 0, 0);
    lv_obj_clear_flag(detail_plate, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(detail_plate, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(detail_plate, plate_draw, LV_EVENT_DRAW_MAIN, nullptr);

    detail_body = lv_label_create(detail);
    lv_obj_set_style_text_font(detail_body, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(detail_body, COL_TEXT, 0);
    lv_label_set_long_mode(detail_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(detail_body, 680);
    lv_obj_align(detail_body, LV_ALIGN_TOP_LEFT, 12, 48);

    build_location();

    if (!s_sweep_timer) {
        s_sweep_timer = lv_timer_create(sweep_timer_cb, 55, nullptr);
    }
}

void ui_init(SharedState *state) {
    g_state = state;
    lv_disp_t *disp = lv_disp_get_default();
    lv_theme_t *th = lv_theme_default_init(disp, COL_GREEN, COL_DIM, true, &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);
    build_radar();
}

void ui_show_wifi() {
    lv_scr_load(scr_radar);
    open_wifi_panel();
}

void ui_show_location() {
    if (detail) {
        lv_obj_add_flag(detail, LV_OBJ_FLAG_HIDDEN);
    }
    if (settings_panel) {
        lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_kb) {
        lv_obj_add_flag(wifi_kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifi_panel) {
        lv_obj_add_flag(wifi_panel, LV_OBJ_FLAG_HIDDEN);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\n%.4f, %.4f",
             g_state->cfg.location.label[0] ? g_state->cfg.location.label : "Custom",
             g_state->cfg.location.lat, g_state->cfg.location.lon);
    lv_label_set_text(radar_loc_lbl, buf);
    snprintf(buf, sizeof(buf), "%.0f nm", g_state->cfg.range_nm);
    lv_label_set_text(radar_range_lbl, buf);
    format_alt_ceiling(g_state->cfg.max_alt_ft, buf, sizeof(buf));
    if (radar_alt_lbl) {
        lv_label_set_text(radar_alt_lbl, buf);
    }
    lv_scr_load(scr_radar);
    if (loc_panel) {
        lv_obj_clear_flag(loc_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(loc_panel);
    }
    show_location_keyboard();
}

void ui_show_radar() {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\n%.4f, %.4f",
             g_state->cfg.location.label[0] ? g_state->cfg.location.label : "Custom",
             g_state->cfg.location.lat, g_state->cfg.location.lon);
    lv_label_set_text(radar_loc_lbl, buf);
    snprintf(buf, sizeof(buf), "%.0f nm", g_state->cfg.range_nm);
    lv_label_set_text(radar_range_lbl, buf);
    format_alt_ceiling(g_state->cfg.max_alt_ft, buf, sizeof(buf));
    if (radar_alt_lbl) {
        lv_label_set_text(radar_alt_lbl, buf);
    }
    s_show_airports = g_state->cfg.show_airports;
    update_apt_toggle_label();
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
        lv_obj_add_event_cb(btn, wifi_item_clicked, LV_EVENT_CLICKED, g_state->wifi_names[i]);
    }
    char msg[48];
    snprintf(msg, sizeof(msg), "%d networks", n);
    lv_label_set_text(wifi_status, msg);
    lvgl_port_unlock();
}

void ui_notify_wifi_result(bool ok, const char *msg) {
    lvgl_port_lock(-1);
    lv_label_set_text(wifi_status, msg);
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
            lv_obj_add_event_cb(btn, loc_pick_clicked, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        }
    }
    lvgl_port_unlock();
}

static bool radar_overlay_open() {
    auto hidden = [](lv_obj_t *o) -> bool {
        return !o || lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);
    };
    return !(hidden(detail) && hidden(wifi_panel) && hidden(loc_panel) && hidden(settings_panel) &&
             hidden(wifi_kb) && hidden(loc_kb));
}

static constexpr int kTrafficTableMax = 15;

static void traffic_table_selected(lv_event_t *e) {
    LV_UNUSED(e);
    if (!traffic_table) {
        return;
    }
    uint16_t row = 0, col = 0;
    lv_table_get_selected_cell(traffic_table, &row, &col);
    if (row == 0 || row == LV_TABLE_CELL_NONE || col == LV_TABLE_CELL_NONE) {
        return;
    }
    const int idx = (int)row - 1;
    if (idx < 0 || idx >= s_table_n) {
        return;
    }
    open_aircraft_detail(&s_table_ac[idx]);
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
            if (s_ac[idxs[b]].dst_nm < s_ac[idxs[a]].dst_nm) {
                const int tmp = idxs[a];
                idxs[a] = idxs[b];
                idxs[b] = tmp;
            }
        }
    }

    const int show = (n > kTrafficTableMax) ? kTrafficTableMax : n;
    s_table_n = show;
    lv_table_set_row_cnt(traffic_table, (uint16_t)(show + 1));

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
    if (s_follow && fi.hex[0] && strcasecmp(fi.hex, s_follow_hex) == 0) {
        s_follow_fi = fi;
        if (fi.reg[0] && s_follow_ac_ok && !s_follow_ac.reg[0]) {
            strncpy(s_follow_ac.reg, fi.reg, sizeof(s_follow_ac.reg) - 1);
            s_follow_ac.reg[sizeof(s_follow_ac.reg) - 1] = 0;
        }
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
    char title[32], body[640];
    format_aircraft(&s_detail_ac, title, sizeof(title), body, sizeof(body), &s_flight_info, false);
    lv_label_set_text(detail_title, title);
    lv_label_set_text(detail_body, body);
    set_detail_aircraft_view();
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

    char body[700];
    snprintf(body, sizeof(body),
             "%s\n%s %s\n"
             "Elev %d ft\n"
             "Rwy  %d ft\n"
             "Pos  %.4f, %.4f\n"
             "Wind  %s\n"
             "Temp  %s\n"
             "Vis   %s SM   Cover %s\n\n"
             "METAR\n%s\n\nTAF\n%s",
             apt.name[0] ? apt.name : wp->icao,
             apt.state, apt.country,
             apt.elev_ft,
             apt.longest_rwy_ft,
             apt.lat, apt.lon,
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
