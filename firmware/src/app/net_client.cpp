#include "net_client.h"
#include "airport_db.h"
#include "runway_db.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <NetworkClientSecure.h>
#include <esp_heap_caps.h>
#include <ArduinoJson.h>
#include <cctype>
#include <cstring>
#include <strings.h>

static const char *kUa = "Mozilla/5.0 (FlightRadar-ESP32)";
static char g_last_err[96] = "";
static void (*g_status_cb)(const char *msg) = nullptr;
static int g_adsb_pref = 0;  // sticky working ADS-B source index

const char *net_last_error() {
    return g_last_err;
}

void net_set_status_callback(void (*cb)(const char *msg)) {
    g_status_cb = cb;
}

static void set_err(const char *msg) {
    strncpy(g_last_err, msg, sizeof(g_last_err) - 1);
    g_last_err[sizeof(g_last_err) - 1] = 0;
    Serial.println(g_last_err);
}

static void status_msg(const char *msg) {
    if (g_status_cb) {
        g_status_cb(msg);
    }
}

void url_encode(const char *in, char *out, size_t out_len) {
    static const char *hex = "0123456789ABCDEF";
    size_t o = 0;
    for (size_t i = 0; in[i] && o + 4 < out_len; i++) {
        unsigned char c = (unsigned char)in[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out[o++] = (char)c;
        } else if (c == ' ') {
            out[o++] = '%';
            out[o++] = '2';
            out[o++] = '0';
        } else {
            out[o++] = '%';
            out[o++] = hex[c >> 4];
            out[o++] = hex[c & 0x0F];
        }
    }
    out[o] = 0;
}

static void copy_trunc(char *dst, size_t dst_len, const char *src) {
    if (!src) {
        dst[0] = 0;
        return;
    }
    strncpy(dst, src, dst_len - 1);
    dst[dst_len - 1] = 0;
    trim_inplace(dst);
}

static bool parse_url(const char *url, bool *tls, char *host, size_t host_len, uint16_t *port, char *path, size_t path_len) {
    const char *p = url;
    *tls = false;
    *port = 80;
    if (strncmp(p, "https://", 8) == 0) {
        *tls = true;
        *port = 443;
        p += 8;
    } else if (strncmp(p, "http://", 7) == 0) {
        p += 7;
    } else {
        return false;
    }
    const char *slash = strchr(p, '/');
    const char *colon = strchr(p, ':');
    size_t host_n;
    if (colon && (!slash || colon < slash)) {
        host_n = (size_t)(colon - p);
        *port = (uint16_t)atoi(colon + 1);
    } else if (slash) {
        host_n = (size_t)(slash - p);
    } else {
        host_n = strlen(p);
    }
    if (host_n == 0 || host_n >= host_len) {
        return false;
    }
    memcpy(host, p, host_n);
    host[host_n] = 0;
    if (slash) {
        strncpy(path, slash, path_len - 1);
        path[path_len - 1] = 0;
    } else {
        strncpy(path, "/", path_len - 1);
    }
    return true;
}

// Fixed PSRAM body buffer (no String realloc thrash). Keep it out of internal
// DRAM — mbedTLS needs that for the SSL handshake.
static char *g_http_buf = nullptr;
static size_t g_http_cap = 0;
static size_t g_http_len = 0;
static constexpr size_t kHttpBufCap = 120 * 1024;

static bool ensure_http_buf() {
    if (g_http_buf) {
        return true;
    }
    g_http_buf = (char *)heap_caps_malloc(kHttpBufCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_http_buf) {
        g_http_buf = (char *)heap_caps_malloc(kHttpBufCap, MALLOC_CAP_8BIT);
    }
    if (!g_http_buf) {
        set_err("No HTTP body RAM");
        return false;
    }
    g_http_cap = kHttpBufCap;
    Serial.printf("HTTP body buf: %u KB\n", (unsigned)(g_http_cap / 1024));
    return true;
}

void net_init_buffers() {
    ensure_http_buf();
}

// Sticky TLS session — full handshakes every ADS-B poll are what thrash flash/PSRAM
// against RGB bounce. Keep-alive reuses one session on the preferred host.
static NetworkClientSecure *g_tls = nullptr;
static char g_tls_host[80] = "";
static uint16_t g_tls_port = 0;

static void tls_close() {
    if (g_tls) {
        g_tls->stop();
        delete g_tls;
        g_tls = nullptr;
    }
    g_tls_host[0] = 0;
    g_tls_port = 0;
}

static NetworkClientSecure *tls_acquire(const char *host, uint16_t port, uint32_t connect_ms, uint32_t io_ms) {
    if (g_tls && g_tls->connected() && g_tls_port == port && strcmp(g_tls_host, host) == 0) {
        return g_tls;
    }
    tls_close();
    NetworkClientSecure *client = new NetworkClientSecure();
    if (!client) {
        set_err("No TLS client RAM");
        return nullptr;
    }
    client->setInsecure();
    client->setHandshakeTimeout((connect_ms + 999) / 1000);
    client->setTimeout((io_ms + 999) / 1000);
    if (!client->connect(host, port, (int32_t)connect_ms)) {
        char tlserr[72] = {0};
        client->lastError(tlserr, sizeof(tlserr));
        char msg[96];
        snprintf(msg, sizeof(msg), "TLS %s", tlserr[0] ? tlserr : "connect failed");
        set_err(msg);
        client->stop();
        delete client;
        return nullptr;
    }
    g_tls = client;
    strncpy(g_tls_host, host, sizeof(g_tls_host) - 1);
    g_tls_host[sizeof(g_tls_host) - 1] = 0;
    g_tls_port = port;
    Serial.printf("TLS session open %s:%u\n", host, port);
    return g_tls;
}

static int header_content_length(const char *headers, int header_end) {
    const char *p = headers;
    const char *end = headers + header_end;
    while (p < end) {
        const char *nl = strstr(p, "\r\n");
        if (!nl || nl > end) {
            break;
        }
        if (strncasecmp(p, "Content-Length:", 15) == 0) {
            return atoi(p + 15);
        }
        p = nl + 2;
    }
    return -1;
}

static bool read_http_body(Client &client, String *body, uint32_t io_ms, bool keep_alive) {
    if (!ensure_http_buf()) {
        return false;
    }
    unsigned long start = millis();
    size_t len = 0;
    int header_end = -1;
    uint8_t chunk[512];
    const uint32_t hdr_ms = io_ms > 4000 ? 4000 : io_ms;
    while (millis() - start < hdr_ms) {
        int avail = client.available();
        if (avail > 0) {
            int n = client.read(chunk, min(avail, (int)sizeof(chunk)));
            if (n > 0) {
                if (len + (size_t)n >= g_http_cap) {
                    set_err("HTTP headers too large");
                    return false;
                }
                memcpy(g_http_buf + len, chunk, (size_t)n);
                len += (size_t)n;
                g_http_buf[len] = 0;
                char *he = strstr(g_http_buf, "\r\n\r\n");
                if (he) {
                    header_end = (int)(he - g_http_buf);
                    break;
                }
                if (len > 4096) {
                    set_err("HTTP headers too large");
                    return false;
                }
            }
        } else if (!client.connected()) {
            set_err("HTTP header closed");
            return false;
        } else {
            delay(2);
        }
    }
    if (header_end < 0) {
        set_err("HTTP header timeout");
        return false;
    }
    int code = 0;
    if (strncmp(g_http_buf, "HTTP/", 5) == 0) {
        const char *sp = strchr(g_http_buf, ' ');
        if (sp && sp < g_http_buf + header_end) {
            code = atoi(sp + 1);
        }
    }
    if (code != 200) {
        char msg[48];
        snprintf(msg, sizeof(msg), "HTTP status %d", code);
        set_err(msg);
        return false;
    }
    const int content_len = header_content_length(g_http_buf, header_end);
    size_t body_off = (size_t)header_end + 4;
    size_t body_len = len > body_off ? len - body_off : 0;
    if (body_len > 0) {
        memmove(g_http_buf, g_http_buf + body_off, body_len);
    }
    len = body_len;
    g_http_buf[len] = 0;
    start = millis();
    while (millis() - start < io_ms) {
        if (content_len >= 0 && (int)len >= content_len) {
            len = (size_t)content_len;
            g_http_buf[len] = 0;
            break;
        }
        int avail = client.available();
        if (avail > 0) {
            int space = (int)(g_http_cap - 1 - len);
            if (content_len >= 0) {
                int remain = content_len - (int)len;
                if (remain < space) {
                    space = remain;
                }
            }
            if (space <= 0) {
                set_err("HTTP body too large");
                return false;
            }
            int n = client.read(chunk, min(avail, min((int)sizeof(chunk), space)));
            if (n > 0) {
                memcpy(g_http_buf + len, chunk, (size_t)n);
                len += (size_t)n;
                g_http_buf[len] = 0;
            }
            start = millis();
            delay(0);
        } else if (!client.connected()) {
            break;
        } else if (content_len >= 0 && (int)len >= content_len) {
            break;
        } else {
            delay(2);
        }
    }
    if (keep_alive && content_len < 0) {
        // No length + keep-alive is unsafe; caller should close.
        keep_alive = false;
    }
    (void)keep_alive;
    g_http_len = len;
    if (body) {
        body->remove(0);
        if (!body->concat(g_http_buf, len)) {
            set_err("HTTP body copy failed");
            return false;
        }
    }
    return len > 0;
}

static const char *http_body_buf(size_t *len_out) {
    if (len_out) {
        *len_out = g_http_len;
    }
    return g_http_buf;
}

bool https_get(const char *url, String *body, const char *user_agent, uint32_t connect_ms, uint32_t io_ms) {
    bool tls = false;
    uint16_t port = 80;
    char host[80];
    char path[384];
    if (!parse_url(url, &tls, host, sizeof(host), &port, path, sizeof(path))) {
        set_err("Bad URL");
        return false;
    }
    Serial.printf("GET %s://%s:%u%s\n", tls ? "https" : "http", host, port, path);

    const char *ua = user_agent ? user_agent : kUa;
    bool ok = false;
    if (tls) {
        NetworkClientSecure *client = tls_acquire(host, port, connect_ms, io_ms);
        if (!client) {
            return false;
        }
        client->printf("GET %s HTTP/1.1\r\n", path);
        client->printf("Host: %s\r\n", host);
        client->printf("User-Agent: %s\r\n", ua);
        client->print("Accept: application/json\r\n");
        client->print("Accept-Encoding: identity\r\n");
        client->print("Connection: keep-alive\r\n\r\n");
        ok = read_http_body(*client, body, io_ms, true);
        if (!ok) {
            tls_close();
        }
    } else {
        WiFiClient client;
        client.setTimeout((io_ms + 999) / 1000);
        if (!client.connect(host, port, (int32_t)connect_ms)) {
            set_err("TCP connect failed");
            return false;
        }
        client.printf("GET %s HTTP/1.0\r\n", path);
        client.printf("Host: %s\r\n", host);
        client.printf("User-Agent: %s\r\n", ua);
        client.print("Accept: application/json\r\n");
        client.print("Connection: close\r\n\r\n");
        ok = read_http_body(client, body, io_ms, false);
        client.stop();
    }
    if (ok) {
        g_last_err[0] = 0;
    }
    return ok;
}

static bool looks_like_us_zip(const char *q) {
    int digits = 0;
    for (const char *p = q; *p; p++) {
        if (*p == ' ' || *p == '-') {
            continue;
        }
        if (!isdigit((unsigned char)*p)) {
            return false;
        }
        digits++;
        if (digits > 5) {
            return false;
        }
    }
    return digits == 5;
}

static int geocode_zippopotam(const char *zip, GeoFix *out, size_t out_max) {
    char url[96];
    snprintf(url, sizeof(url), "http://api.zippopotam.us/us/%s", zip);
    String body;
    if (!https_get(url, &body, kUa)) {
        return 0;
    }
    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        return 0;
    }
    JsonArray places = doc["places"].as<JsonArray>();
    if (places.isNull() || places.size() == 0 || out_max == 0) {
        return 0;
    }
    JsonObject p = places[0];
    copy_trunc(out[0].query, sizeof(out[0].query), zip);
    snprintf(out[0].label, sizeof(out[0].label), "%s, %s %s",
             p["place name"] | zip,
             p["state abbreviation"] | "",
             zip);
    out[0].lat = p["latitude"].as<float>();
    out[0].lon = p["longitude"].as<float>();
    out[0].valid = true;
    return 1;
}

static int geocode_nominatim(const char *query, GeoFix *out, size_t out_max) {
    char encoded[192];
    url_encode(query, encoded, sizeof(encoded));
    char url[256];
    snprintf(url, sizeof(url),
             "https://nominatim.openstreetmap.org/search?format=json&limit=5&addressdetails=0&q=%s",
             encoded);
    String body;
    if (!https_get(url, &body, kUa)) {
        return 0;
    }
    JsonDocument doc;
    if (deserializeJson(doc, body) || !doc.is<JsonArray>()) {
        return 0;
    }
    int n = 0;
    for (JsonObject item : doc.as<JsonArray>()) {
        if ((size_t)n >= out_max) {
            break;
        }
        copy_trunc(out[n].query, sizeof(out[n].query), query);
        copy_trunc(out[n].label, sizeof(out[n].label), item["display_name"] | query);
        out[n].lat = item["lat"].as<float>();
        out[n].lon = item["lon"].as<float>();
        out[n].valid = true;
        n++;
    }
    return n;
}

int net_geocode(const char *query, GeoFix *out, size_t out_max) {
    char q[96];
    copy_trunc(q, sizeof(q), query);
    if (q[0] == 0 || out_max == 0) {
        return 0;
    }
    int n = 0;
    if (looks_like_us_zip(q)) {
        n = geocode_zippopotam(q, out, out_max);
    }
    if (n == 0) {
        n = geocode_nominatim(q, out, out_max);
    }
    return n;
}

static int parse_adsblol(const char *body, size_t body_len, Aircraft *out, size_t out_max) {
    // No Json filter — filtered parses were returning empty "ac" on some hosts.
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body, body_len);
    if (err) {
        Serial.printf("ADS-B JSON: %s len=%u\n", err.c_str(), (unsigned)body_len);
        return -1;
    }
    JsonArray ac = doc["ac"].as<JsonArray>();
    if (ac.isNull()) {
        // Wrong payload (HTML/error) — not a valid empty traffic list.
        Serial.printf("ADS-B missing ac, body=%u head=%.40s\n",
                      (unsigned)body_len, body);
        return -1;
    }
    const int total = doc["total"] | (int)ac.size();
    Serial.printf("ADS-B ac=%u total=%d\n", (unsigned)ac.size(), total);
    int n = 0;
    int skipped = 0;
    for (JsonObject a : ac) {
        if ((size_t)n >= out_max) {
            break;
        }
        if (a["lat"].isNull() || a["lon"].isNull()) {
            skipped++;
            continue;
        }
        const float lat = a["lat"].as<float>();
        const float lon = a["lon"].as<float>();
        if (lat == 0.0f && lon == 0.0f) {
            skipped++;
            continue;
        }
        Aircraft &dst = out[n];
        memset(&dst, 0, sizeof(dst));
        copy_trunc(dst.hex, sizeof(dst.hex), a["hex"] | "");
        copy_trunc(dst.flight, sizeof(dst.flight), a["flight"] | "");
        copy_trunc(dst.reg, sizeof(dst.reg), a["r"] | "");
        copy_trunc(dst.type, sizeof(dst.type), a["t"] | "");
        copy_trunc(dst.squawk, sizeof(dst.squawk), a["squawk"] | "");
        copy_trunc(dst.category, sizeof(dst.category), a["category"] | "");
        dst.lat = lat;
        dst.lon = lon;
        dst.gs_kts = a["gs"] | 0.0f;
        dst.track_deg = a["track"] | 0.0f;
        dst.vrate_fpm = a["baro_rate"] | 0.0f;
        dst.dst_nm = a["dst"] | 0.0f;
        if (a["alt_baro"].is<const char *>()) {
            dst.on_ground = true;
            dst.alt_ft = 0;
        } else {
            dst.alt_ft = a["alt_baro"] | 0.0f;
            dst.on_ground = dst.alt_ft <= 0;
        }
        dst.valid = true;
        n++;
    }
    if (n == 0 && ac.size() > 0) {
        Serial.printf("ADS-B all %u rows lacked lat/lon (skipped=%d)\n",
                      (unsigned)ac.size(), skipped);
    }
    return n;
}

int net_fetch_aircraft(float lat, float lon, float range_nm, Aircraft *out, size_t out_max) {
    if (range_nm > 250.0f) {
        range_nm = 250.0f;
    }
    if (range_nm < 10.0f) {
        range_nm = 10.0f;
    }
    const int dist = (int)lroundf(range_nm);
    char url[192];
    int best_empty = 0;
    bool any_ok = false;

    // Community readsb/tar1090 aggregators (real ADS-B). No OpenSky — anonymous
    // OpenSky returns HTTP 429 under credit limits. airplanes.live requires prior approval (403).
    struct AdsbSrc {
        const char *name;
        const char *fmt;
    };
    static const AdsbSrc kSrc[] = {
        {"adsb.lol", "https://api.adsb.lol/v2/lat/%.4f/lon/%.4f/dist/%d"},
        {"adsb.fi", "https://opendata.adsb.fi/api/v2/lat/%.4f/lon/%.4f/dist/%d"},
    };
    const int nsrc = (int)(sizeof(kSrc) / sizeof(kSrc[0]));
    if (g_adsb_pref < 0 || g_adsb_pref >= nsrc) {
        g_adsb_pref = 0;
    }

    // Try preferred host first, then the rest. Empty lists are not sticky successes.
    for (int attempt = 0; attempt < nsrc; attempt++) {
        const int idx = (g_adsb_pref + attempt) % nsrc;
        char msg[48];
        snprintf(msg, sizeof(msg), "ADS-B %s...", kSrc[idx].name);
        status_msg(msg);
        snprintf(url, sizeof(url), kSrc[idx].fmt, lat, lon, dist);
        // nullptr body → fill g_http_buf only (no second PSRAM String copy).
        if (!https_get(url, nullptr, kUa, 5000, 10000)) {
            continue;
        }
        size_t blen = 0;
        const char *buf = http_body_buf(&blen);
        const int n = parse_adsblol(buf, blen, out, out_max);
        if (n > 0) {
            g_adsb_pref = idx;
            return n;
        }
        if (n == 0) {
            any_ok = true;
            best_empty = 0;
        }
    }

    if (any_ok) {
        return best_empty;
    }
    return -1;
}

int net_fetch_airports(float lat, float lon, float range_nm, Airport *out, size_t out_max) {
    // Onboard OurAirports DB: airports with longest runway >= 3500 ft.
    static const int kMinRwyFt = 3500;
    if (range_nm > 250.0f) {
        range_nm = 250.0f;
    }
    if (range_nm < 10.0f) {
        range_nm = 10.0f;
    }
    const float range2 = range_nm * range_nm;
    // Coarse lat/lon prefilter in centi-degrees (~0.6 nm per unit at mid lats).
    const int pad = (int)lroundf(range_nm / 0.6f) + 2;
    const int lat0 = (int)lroundf(lat * 100.0f);
    const int lon0 = (int)lroundf(lon * 100.0f);

    static Airport tmp[160];
    static uint8_t tier_of[160];
    static float dist2_of[160];
    int n = 0;
    int n_l = 0, n_m = 0, n_s = 0;

    for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
        const AptRec &rec = kAptDb[i];
        if ((int)rec.rwy_ft < kMinRwyFt) {
            continue;
        }
        if (abs((int)rec.lat_c - lat0) > pad || abs((int)rec.lon_c - lon0) > pad) {
            continue;
        }
        const float alat = rec.lat_c / 100.0f;
        const float alon = rec.lon_c / 100.0f;
        float east, north;
        ll_to_nm(alat, alon, lat, lon, &east, &north);
        const float d2 = east * east + north * north;
        if (d2 > range2) {
            continue;
        }

        int slot = n;
        if (n >= 160) {
            int worst = 0;
            for (int j = 1; j < n; j++) {
                if (dist2_of[j] > dist2_of[worst]) {
                    worst = j;
                }
            }
            if (d2 >= dist2_of[worst]) {
                continue;
            }
            if (tier_of[worst] == 0) {
                n_l--;
            } else if (tier_of[worst] == 1) {
                n_m--;
            } else {
                n_s--;
            }
            slot = worst;
        } else {
            n++;
        }

        Airport &dst = tmp[slot];
        memset(&dst, 0, sizeof(dst));
        memcpy(dst.icao, rec.icao, 4);
        dst.icao[4] = 0;
        if (rec.iata[0]) {
            memcpy(dst.iata, rec.iata, 3);
            dst.iata[3] = 0;
        }
        copy_trunc(dst.name, sizeof(dst.name), dst.icao);
        dst.lat = alat;
        dst.lon = alon;
        dst.elev_ft = (int)rec.elev_ft;
        dst.rwy_class = (rec.size == 0) ? 'L' : (rec.size == 1) ? 'M' : 'S';
        dst.longest_rwy_ft = (int)rec.rwy_ft;
        dst.valid = true;
        tier_of[slot] = rec.size;
        dist2_of[slot] = d2;
        if (rec.size == 0) {
            n_l++;
        } else if (rec.size == 1) {
            n_m++;
        } else {
            n_s++;
        }
    }

    // Prefer nearer airports; keep some medium/large mix so hubs aren't crowded out.
    static uint8_t picked[160];
    memset(picked, 0, n);
    int keep = 0;
    const int quotas[3] = {
        (int)(out_max * 2 / 5),
        (int)(out_max * 2 / 5),
        (int)(out_max - (out_max * 2 / 5) - (out_max * 2 / 5)),
    };
    for (int tier = 0; tier < 3 && keep < (int)out_max; tier++) {
        int taken = 0;
        while (taken < quotas[tier] && keep < (int)out_max) {
            int best = -1;
            float best_d = 1e30f;
            for (int i = 0; i < n; i++) {
                if (picked[i] || tier_of[i] != (uint8_t)tier) {
                    continue;
                }
                if (dist2_of[i] < best_d) {
                    best_d = dist2_of[i];
                    best = i;
                }
            }
            if (best < 0) {
                break;
            }
            out[keep++] = tmp[best];
            picked[best] = 1;
            taken++;
        }
    }
    while (keep < (int)out_max) {
        int best = -1;
        float best_d = 1e30f;
        for (int i = 0; i < n; i++) {
            if (picked[i]) {
                continue;
            }
            if (dist2_of[i] < best_d) {
                best_d = dist2_of[i];
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        out[keep++] = tmp[best];
        picked[best] = 1;
    }
    Serial.printf("Airports DB: rwy>=%d L/M/S=%d/%d/%d kept=%d\n", kMinRwyFt, n_l, n_m, n_s, keep);
    return keep;
}

int net_lookup_runways(const char *icao, Runway *out, size_t out_max) {
    if (!icao || !icao[0] || !out || out_max == 0) {
        return 0;
    }
    int n = 0;
    for (size_t i = 0; i < K_RWY_DB_COUNT; i++) {
        const RwyRec &r = kRwyDb[i];
        if (strcmp(r.icao, icao) != 0) {
            continue;
        }
        Runway &dst = out[n];
        memset(&dst, 0, sizeof(dst));
        copy_trunc(dst.le, sizeof(dst.le), r.le);
        copy_trunc(dst.he, sizeof(dst.he), r.he);
        dst.len_ft = (int)r.len_ft;
        dst.hdg_deg = (int)r.hdg_deg;
        dst.width_ft = (int)r.width_ft;
        n++;
        if ((size_t)n >= out_max) {
            break;
        }
    }
    return n;
}

bool net_lookup_airport(const char *icao, float *lat, float *lon, int *rwy_ft) {
    if (!icao || !icao[0]) {
        return false;
    }
    for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
        const AptRec &rec = kAptDb[i];
        if (strcmp(rec.icao, icao) != 0) {
            continue;
        }
        if (lat) {
            *lat = rec.lat_c / 100.0f;
        }
        if (lon) {
            *lon = rec.lon_c / 100.0f;
        }
        if (rwy_ft) {
            *rwy_ft = (int)rec.rwy_ft;
        }
        return true;
    }
    return false;
}

bool net_fetch_route(const char *callsign, char *origin, size_t origin_len, char *dest, size_t dest_len) {
    if (origin && origin_len) {
        origin[0] = 0;
    }
    if (dest && dest_len) {
        dest[0] = 0;
    }
    if (!callsign || !callsign[0]) {
        return false;
    }
    char cs[16];
    copy_trunc(cs, sizeof(cs), callsign);
    for (char *p = cs; *p; p++) {
        if (*p == ' ') {
            *p = 0;
            break;
        }
        *p = (char)toupper((unsigned char)*p);
    }
    if (!cs[0]) {
        return false;
    }
    char enc[32];
    url_encode(cs, enc, sizeof(enc));
    char url[128];
    // Prefer flight/callsign; registration/tail is also accepted by hexdb as a fallback key.
    snprintf(url, sizeof(url), "https://hexdb.io/callsign-route?callsign=%s", enc);
    String body;
    if (!https_get(url, &body, kUa, 5000, 8000)) {
        return false;
    }
    char buf[48];
    copy_trunc(buf, sizeof(buf), body.c_str());
    for (char *p = buf; *p; p++) {
        if (*p == '"' || *p == '\'') {
            *p = ' ';
        }
    }
    trim_inplace(buf);
    char *dash = strchr(buf, '-');
    if (!dash || dash == buf || !dash[1]) {
        return false;
    }
    *dash = 0;
    const char *o = buf;
    const char *d = dash + 1;
    auto ok_code = [](const char *s) -> bool {
        size_t n = strlen(s);
        if (n < 3 || n > 4) {
            return false;
        }
        for (size_t i = 0; i < n; i++) {
            if (!isalnum((unsigned char)s[i])) {
                return false;
            }
        }
        return true;
    };
    if (!ok_code(o) || !ok_code(d)) {
        return false;
    }
    if (origin && origin_len) {
        copy_trunc(origin, origin_len, o);
        for (char *p = origin; *p; p++) {
            *p = (char)toupper((unsigned char)*p);
        }
    }
    if (dest && dest_len) {
        copy_trunc(dest, dest_len, d);
        for (char *p = dest; *p; p++) {
            *p = (char)toupper((unsigned char)*p);
        }
    }
    return true;
}

bool net_fetch_airport_names(const char *origin_icao, const char *dest_icao, char *origin_name,
                             size_t origin_name_len, char *dest_name, size_t dest_name_len) {
    if (origin_name && origin_name_len) {
        origin_name[0] = 0;
    }
    if (dest_name && dest_name_len) {
        dest_name[0] = 0;
    }
    char ids[24] = {0};
    if (origin_icao && origin_icao[0] && dest_icao && dest_icao[0]) {
        snprintf(ids, sizeof(ids), "%s,%s", origin_icao, dest_icao);
    } else if (origin_icao && origin_icao[0]) {
        copy_trunc(ids, sizeof(ids), origin_icao);
    } else if (dest_icao && dest_icao[0]) {
        copy_trunc(ids, sizeof(ids), dest_icao);
    } else {
        return false;
    }
    // Prefer onboard DB IATA as a cheap label when AWC is slow/unavailable.
    auto fill_local = [](const char *icao, char *out, size_t out_len) {
        if (!icao || !icao[0] || !out || !out_len) {
            return;
        }
        for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
            if (strcmp(kAptDb[i].icao, icao) != 0) {
                continue;
            }
            if (kAptDb[i].iata[0]) {
                snprintf(out, out_len, "%s", kAptDb[i].iata);
            }
            return;
        }
    };
    fill_local(origin_icao, origin_name, origin_name_len);
    fill_local(dest_icao, dest_name, dest_name_len);

    char url[160];
    snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/airport?ids=%s&format=json", ids);
    String body;
    if (!https_get(url, &body, kUa, 5000, 8000)) {
        return (origin_name && origin_name[0]) || (dest_name && dest_name[0]);
    }
    JsonDocument doc;
    if (deserializeJson(doc, body) || !doc.is<JsonArray>()) {
        return (origin_name && origin_name[0]) || (dest_name && dest_name[0]);
    }
    for (JsonObject a : doc.as<JsonArray>()) {
        const char *icao = a["icaoId"] | "";
        const char *name = a["name"] | "";
        if (!icao[0] || !name[0]) {
            continue;
        }
        if (origin_icao && origin_icao[0] && strcasecmp(icao, origin_icao) == 0 && origin_name && origin_name_len) {
            copy_trunc(origin_name, origin_name_len, name);
        }
        if (dest_icao && dest_icao[0] && strcasecmp(icao, dest_icao) == 0 && dest_name && dest_name_len) {
            copy_trunc(dest_name, dest_name_len, name);
        }
    }
    return (origin_name && origin_name[0]) || (dest_name && dest_name[0]);
}

bool net_fetch_aircraft_meta(const char *hex, char *manufacturer, size_t man_len, char *type_name,
                             size_t type_len, char *owner, size_t owner_len, char *registration,
                             size_t reg_len) {
    if (manufacturer && man_len) {
        manufacturer[0] = 0;
    }
    if (type_name && type_len) {
        type_name[0] = 0;
    }
    if (owner && owner_len) {
        owner[0] = 0;
    }
    if (registration && reg_len) {
        registration[0] = 0;
    }
    if (!hex || !hex[0]) {
        return false;
    }
    char h[16];
    copy_trunc(h, sizeof(h), hex);
    for (char *p = h; *p; p++) {
        *p = (char)tolower((unsigned char)*p);
    }
    char url[96];
    snprintf(url, sizeof(url), "https://hexdb.io/api/v1/aircraft/%s", h);
    String body;
    if (!https_get(url, &body, kUa, 5000, 8000)) {
        return false;
    }
    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        return false;
    }
    if (manufacturer && man_len) {
        copy_trunc(manufacturer, man_len, doc["Manufacturer"] | "");
    }
    if (type_name && type_len) {
        const char *tn = doc["Type"] | "";
        if (!tn[0]) {
            tn = doc["ICAOTypeCode"] | "";
        }
        copy_trunc(type_name, type_len, tn);
    }
    if (owner && owner_len) {
        copy_trunc(owner, owner_len, doc["RegisteredOwners"] | "");
    }
    if (registration && reg_len) {
        copy_trunc(registration, reg_len, doc["Registration"] | "");
    }
    return (manufacturer && manufacturer[0]) || (type_name && type_name[0]) || (owner && owner[0]) ||
           (registration && registration[0]);
}

bool net_fetch_weather(const char *icao, Weather *out) {
    memset(out, 0, sizeof(*out));
    copy_trunc(out->icao, sizeof(out->icao), icao);
    char url[160];
    snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/metar?ids=%s&format=json", icao);
    String body;
    if (!https_get(url, &body, kUa)) {
        return false;
    }
    JsonDocument doc;
    if (deserializeJson(doc, body) || !doc.is<JsonArray>() || doc.as<JsonArray>().size() == 0) {
        return false;
    }
    JsonObject m = doc[0];
    copy_trunc(out->raw_metar, sizeof(out->raw_metar), m["rawOb"] | "");
    copy_trunc(out->flt_cat, sizeof(out->flt_cat), m["fltCat"] | "");
    copy_trunc(out->cover, sizeof(out->cover), m["cover"] | "");
    out->temp_c = m["temp"] | NAN;
    out->wind_dir = m["wdir"] | NAN;
    out->wind_kt = m["wspd"] | NAN;
    if (m["visib"].is<float>() || m["visib"].is<int>()) {
        out->vis_sm = m["visib"].as<float>();
    } else {
        out->vis_sm = atof(m["visib"] | "0");
    }
    snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/taf?ids=%s&format=json", icao);
    body = "";
    if (https_get(url, &body, kUa)) {
        JsonDocument taf;
        if (!deserializeJson(taf, body) && taf.is<JsonArray>() && taf.as<JsonArray>().size() > 0) {
            copy_trunc(out->raw_taf, sizeof(out->raw_taf), taf[0]["rawTAF"] | "");
        }
    }
    out->ok = out->raw_metar[0] != 0;
    return out->ok;
}
