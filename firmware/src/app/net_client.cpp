#include "net_client.h"
#include "airport_db.h"
#include "runway_db.h"
#include "sd_store.h"
#include "waypoint_db.h"
#include "board_config.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <NetworkClientSecure.h>
#include <esp_heap_caps.h>
#include <ArduinoJson.h>
#include <cctype>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <strings.h>
#include <time.h>

static const char *kUa = "Mozilla/5.0 (FlightRadar-ESP32)";
// FlightAware serves a fuller live page to a normal browser UA.
static const char *kUaBrowser =
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) "
    "Chrome/120.0.0.0 Safari/537.36";
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
// FlightStats ~110-170 KB; FlightAware live pages with flightPlan are ~500-600 KB.
static constexpr size_t kHttpBufCap = 768 * 1024;

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
// Set on every tls_close so the next handshake can wait for stack teardown.
static uint32_t s_tls_last_close_ms = 0;

static void tls_close() {
    if (g_tls) {
        g_tls->stop();
        delete g_tls;
        g_tls = nullptr;
        s_tls_last_close_ms = millis();
    }
    g_tls_host[0] = 0;
    g_tls_port = 0;
}

void net_tls_reset() {
    tls_close();
}

// Simple handshake (GitHub style) + one slow retry. Rapid reconnect to FlightAware
// after a large HTML download often fails once with "TLS generic error".

static NetworkClientSecure *tls_acquire(const char *host, uint16_t port, uint32_t connect_ms, uint32_t io_ms) {
    if (g_tls && g_tls->connected() && g_tls_port == port && strcmp(g_tls_host, host) == 0) {
        return g_tls;
    }
    tls_close();
    if (WiFi.status() != WL_CONNECTED) {
        set_err("TLS Wi-Fi down");
        return nullptr;
    }
    // Let lwIP/mbedTLS finish teardown after a prior FA page abort/close.
    if (s_tls_last_close_ms) {
        const uint32_t since = millis() - s_tls_last_close_ms;
        if (since < 1500u) {
            delay(1500u - since);
        }
    }

    char last_tlserr[72] = {0};
    for (int attempt = 1; attempt <= 2; attempt++) {
        NetworkClientSecure *client = new NetworkClientSecure();
        if (!client) {
            set_err("No TLS client RAM");
            return nullptr;
        }
        client->setInsecure();
        client->setHandshakeTimeout((connect_ms + 999) / 1000);
        client->setTimeout((io_ms + 999) / 1000);
        Serial.printf("TLS connect %s:%u try %d (free=%u)\n", host, port, attempt,
                      (unsigned)ESP.getFreeHeap());
        if (client->connect(host, port, (int32_t)connect_ms)) {
            g_tls = client;
            strncpy(g_tls_host, host, sizeof(g_tls_host) - 1);
            g_tls_host[sizeof(g_tls_host) - 1] = 0;
            g_tls_port = port;
            Serial.printf("TLS session open %s:%u\n", host, port);
            return g_tls;
        }
        client->lastError(last_tlserr, sizeof(last_tlserr));
        Serial.printf("TLS fail try %d: %s\n", attempt, last_tlserr[0] ? last_tlserr : "(none)");
        client->stop();
        delete client;
        s_tls_last_close_ms = millis();
        if (attempt == 1) {
            delay(2000);
            yield();
        }
    }
    char msg[96];
    snprintf(msg, sizeof(msg), "TLS %s", last_tlserr[0] ? last_tlserr : "connect failed");
    set_err(msg);
    return nullptr;
}

// Filled when read_http_body sees 301/302/307/308 (absolute or relative Location).
static char g_redirect_loc[512] = {0};

static bool header_find_value(const char *headers, int header_end, const char *key, char *out,
                              size_t out_len) {
    if (!headers || header_end <= 0 || !key || !out || out_len == 0) {
        return false;
    }
    out[0] = 0;
    const size_t key_len = strlen(key);
    const char *p = headers;
    const char *end = headers + header_end;
    while (p < end) {
        const char *line_end = (const char *)memchr(p, '\n', (size_t)(end - p));
        if (!line_end) {
            line_end = end;
        }
        if ((size_t)(line_end - p) > key_len + 1 && strncasecmp(p, key, key_len) == 0 &&
            p[key_len] == ':') {
            const char *v = p + key_len + 1;
            while (v < line_end && (*v == ' ' || *v == '\t')) {
                v++;
            }
            size_t n = 0;
            while (v + n < line_end && v[n] != '\r' && v[n] != '\n' && n + 1 < out_len) {
                n++;
            }
            memcpy(out, v, n);
            out[n] = 0;
            return n > 0;
        }
        p = line_end + 1;
    }
    return false;
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

static bool header_is_chunked(const char *headers, int header_end) {
    const char *p = headers;
    const char *end = headers + header_end;
    while (p < end) {
        const char *nl = strstr(p, "\r\n");
        if (!nl || nl > end) {
            break;
        }
        if (strncasecmp(p, "Transfer-Encoding:", 18) == 0) {
            const char *v = p + 18;
            while (v < nl && (*v == ' ' || *v == '\t')) {
                v++;
            }
            if (nl - v >= 7 && strncasecmp(v, "chunked", 7) == 0) {
                return true;
            }
        }
        p = nl + 2;
    }
    return false;
}

// Read exactly n bytes (or until idle / absolute deadline). Returns bytes read.
// NetworkClientSecure often reports available()==0 while TLS still has ciphertext
// to decrypt — a read() call is required. But read() blocks up to setTimeout(), so
// when available()==0 we only probe 1 byte with a 1s timeout (not the full io_ms).
// abs_deadline prevents "idle forever" when the peer dribbles 1 byte every few seconds.
static int http_read_exact(Client &client, char *dst, int n, uint32_t idle_ms, unsigned long *idle_start,
                           unsigned long abs_deadline) {
    int got = 0;
    const uint32_t full_to_s = (idle_ms + 999) / 1000;
    while (got < n) {
        const unsigned long now = millis();
        if ((long)(now - *idle_start) >= (long)idle_ms || (long)(now - abs_deadline) >= 0) {
            break;
        }
        const int avail = client.available();
        int r = 0;
        if (avail > 0) {
            int want = n - got;
            if (want > avail) {
                want = avail;
            }
            r = client.read((uint8_t *)dst + got, want);
        } else if (!client.connected()) {
            break;
        } else {
            // Short probe read to pump TLS without hanging the net task for 90s.
            client.setTimeout(1);
            r = client.read((uint8_t *)dst + got, 1);
            client.setTimeout(full_to_s > 0 ? full_to_s : 1);
        }
        if (r > 0) {
            got += r;
            *idle_start = millis();
            delay(0);
        } else if (!client.connected()) {
            break;
        } else {
            delay(2);
        }
    }
    return got;
}

static bool (*g_http_early_ok)(const char *buf, size_t len) = nullptr;

static bool read_http_body(Client &client, String *body, uint32_t io_ms, bool keep_alive) {
    if (!ensure_http_buf()) {
        return false;
    }
    unsigned long start = millis();
    // Hard cap for the whole response — idle timer alone can hang forever on dribble traffic.
    const unsigned long abs_deadline = start + io_ms;
    size_t len = 0;
    int header_end = -1;
    uint8_t chunk[512];
    const uint32_t hdr_ms = io_ms > 8000 ? 8000 : io_ms;
    while (millis() - start < hdr_ms) {
        const int avail = client.available();
        int n = 0;
        if (avail > 0) {
            n = client.read(chunk, min(avail, (int)sizeof(chunk)));
        } else if (!client.connected()) {
            set_err("HTTP header closed");
            return false;
        } else {
            client.setTimeout(1);
            n = client.read(chunk, 1);
            client.setTimeout((io_ms + 999) / 1000);
        }
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
    g_redirect_loc[0] = 0;
    if (code == 301 || code == 302 || code == 303 || code == 307 || code == 308) {
        if (header_find_value(g_http_buf, header_end, "Location", g_redirect_loc,
                              sizeof(g_redirect_loc))) {
            set_err("HTTP redirect");
        } else {
            snprintf(g_last_err, sizeof(g_last_err), "HTTP status %d (no Location)", code);
        }
        return false;
    }
    if (code != 200) {
        char msg[48];
        snprintf(msg, sizeof(msg), "HTTP status %d", code);
        set_err(msg);
        return false;
    }
    const int content_len = header_content_length(g_http_buf, header_end);
    const bool chunked = header_is_chunked(g_http_buf, header_end);
    size_t body_off = (size_t)header_end + 4;
    size_t pre_len = len > body_off ? len - body_off : 0;

    // Prefetch bytes already read past headers — feed into pushback for chunked,
    // or into the body buffer for Content-Length / raw. 16 KB covers a full TCP
    // window past the header terminator (old 2 KB guard false-failed large FA pages).
    static char pushback[16 * 1024];
    size_t push_n = 0;
    if (chunked) {
        if (pre_len > sizeof(pushback)) {
            set_err("HTTP chunk prefetch large");
            return false;
        }
        if (pre_len) {
            memcpy(pushback, g_http_buf + body_off, pre_len);
            push_n = pre_len;
        }
        len = 0;
        g_http_buf[0] = 0;
        start = millis();
        // Idle between chunk bytes. FA often pauses >12s mid-page on Wi-Fi; 12s caused
        // "HTTP chunk data timeout" before boards arrived. Absolute deadline still caps total.
        const uint32_t idle_ms = io_ms > 30000 ? 30000 : io_ms;

        auto take_byte = [&](uint8_t *out) -> bool {
            if (push_n > 0) {
                *out = (uint8_t)pushback[0];
                memmove(pushback, pushback + 1, --push_n);
                return true;
            }
            if (http_read_exact(client, (char *)out, 1, idle_ms, &start, abs_deadline) != 1) {
                return false;
            }
            return true;
        };

        auto take_bytes = [&](char *dst, int n) -> bool {
            int filled = 0;
            if (push_n > 0) {
                int take = (int)push_n < n ? (int)push_n : n;
                memcpy(dst, pushback, (size_t)take);
                memmove(pushback, pushback + take, push_n - (size_t)take);
                push_n -= (size_t)take;
                filled = take;
            }
            if (filled < n) {
                if (http_read_exact(client, dst + filled, n - filled, idle_ms, &start, abs_deadline) !=
                    (n - filled)) {
                    return false;
                }
            }
            return true;
        };

        for (;;) {
            if ((long)(millis() - abs_deadline) >= 0) {
                // Prefer a clean peer-close; only keep a partial if we already have usable HTML.
                if (len >= 64 * 1024 || (g_http_early_ok && g_http_early_ok(g_http_buf, len))) {
                    Serial.printf("HTTP body deadline at %u KB — using partial\n", (unsigned)(len / 1024));
                    goto body_done;
                }
                set_err("HTTP body deadline");
                return false;
            }
            // Chunk-size line.
            char line[48];
            size_t li = 0;
            for (;;) {
                uint8_t b = 0;
                if (!take_byte(&b)) {
                    // Already have enough HTML (airport boards ~65–120 KB in) — keep it.
                    if (len >= 64 * 1024 || (g_http_early_ok && g_http_early_ok(g_http_buf, len))) {
                        Serial.printf("HTTP chunk-size stall after %u KB — using partial body\n",
                                      (unsigned)(len / 1024));
                        goto body_done;
                    }
                    set_err("HTTP chunk size timeout");
                    return false;
                }
                if (b == '\n') {
                    if (li > 0 && line[li - 1] == '\r') {
                        li--;
                    }
                    line[li] = 0;
                    break;
                }
                if (li + 1 < sizeof(line)) {
                    line[li++] = (char)b;
                }
            }
            // Ignore chunk extensions after ';'.
            char *semi = strchr(line, ';');
            if (semi) {
                *semi = 0;
            }
            long csz = strtol(line, nullptr, 16);
            if (csz < 0) {
                set_err("HTTP bad chunk size");
                return false;
            }
            if (csz == 0) {
                // Consume trailers until blank line.
                for (;;) {
                    li = 0;
                    for (;;) {
                        uint8_t b = 0;
                        if (!take_byte(&b)) {
                            break;
                        }
                        if (b == '\n') {
                            if (li > 0 && line[li - 1] == '\r') {
                                li--;
                            }
                            line[li] = 0;
                            break;
                        }
                        if (li + 1 < sizeof(line)) {
                            line[li++] = (char)b;
                        }
                    }
                    if (li == 0) {
                        break;
                    }
                }
                break;
            }
            if (len + (size_t)csz >= g_http_cap) {
                // FA flight pages can exceed the buffer; keep what we have and parse it.
                Serial.printf("HTTP body truncated at %u KB (chunked)\n", (unsigned)(len / 1024));
                break;
            }
            if (!take_bytes(g_http_buf + len, (int)csz)) {
                // Only keep a partial body if we already have enough HTML to parse.
                // (Airport boards sit ~65–120 KB in; tiny stalls must NOT count as success.)
                if (len >= 64 * 1024 || (g_http_early_ok && g_http_early_ok(g_http_buf, len))) {
                    Serial.printf("HTTP chunk stall after %u KB — using partial body\n",
                                  (unsigned)(len / 1024));
                    break;
                }
                set_err("HTTP chunk data timeout");
                return false;
            }
            len += (size_t)csz;
            g_http_buf[len] = 0;
            if ((len / 32768) != ((len - (size_t)csz) / 32768)) {
                char st[40];
                snprintf(st, sizeof(st), "Download %u KB…", (unsigned)(len / 1024));
                status_msg(st);
            }
            // Do NOT mid-stream abort on early_ok — that RST poisons the next FA TLS handshake.
            // Trailing CRLF after chunk data.
            uint8_t crlf[2];
            if (!take_bytes((char *)crlf, 2)) {
                if (len >= 64 * 1024 || (g_http_early_ok && g_http_early_ok(g_http_buf, len))) {
                    break;
                }
                set_err("HTTP chunk CRLF timeout");
                return false;
            }
        }
    body_done:
        (void)0;
    } else {
        if (pre_len > 0) {
            memmove(g_http_buf, g_http_buf + body_off, pre_len);
        }
        len = pre_len;
        g_http_buf[len] = 0;
        // Absolute deadline — do not reset on each byte (that hung FA downloads).
        while ((long)(millis() - abs_deadline) < 0) {
            if (content_len >= 0 && (int)len >= content_len) {
                len = (size_t)content_len;
                g_http_buf[len] = 0;
                break;
            }
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
            const int avail = client.available();
            int n = 0;
            if (avail > 0) {
                n = client.read(chunk, min(avail, min((int)sizeof(chunk), space)));
            } else if (!client.connected()) {
                break;
            } else {
                client.setTimeout(1);
                n = client.read(chunk, 1);
                client.setTimeout((io_ms + 999) / 1000);
            }
            if (n > 0) {
                memcpy(g_http_buf + len, chunk, (size_t)n);
                len += (size_t)n;
                g_http_buf[len] = 0;
                delay(0);
            } else if (!client.connected()) {
                break;
            } else if (content_len >= 0 && (int)len >= content_len) {
                break;
            } else {
                delay(2);
            }
        }
    }
    if (keep_alive && content_len < 0 && !chunked) {
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

/** Resolve absolute/relative Location into out_url using the current request URL pieces. */
static bool resolve_redirect_url(bool cur_tls, const char *cur_host, uint16_t cur_port,
                                 const char *cur_path, const char *loc, char *out_url,
                                 size_t out_len) {
    if (!loc || !loc[0] || !out_url || out_len < 16) {
        return false;
    }
    if (strncmp(loc, "https://", 8) == 0 || strncmp(loc, "http://", 7) == 0) {
        strncpy(out_url, loc, out_len - 1);
        out_url[out_len - 1] = 0;
        return true;
    }
    const char *scheme = cur_tls ? "https" : "http";
    if (loc[0] == '/') {
        if (cur_port != 0 && cur_port != (cur_tls ? 443 : 80)) {
            snprintf(out_url, out_len, "%s://%s:%u%s", scheme, cur_host, cur_port, loc);
        } else {
            snprintf(out_url, out_len, "%s://%s%s", scheme, cur_host, loc);
        }
        return true;
    }
    // Relative to current path directory (static — avoid 1KB on net-task stack).
    static char dir[1024];
    strncpy(dir, cur_path && cur_path[0] ? cur_path : "/", sizeof(dir) - 1);
    dir[sizeof(dir) - 1] = 0;
    char *slash = strrchr(dir, '/');
    if (slash) {
        slash[1] = 0;
    } else {
        strncpy(dir, "/", sizeof(dir) - 1);
    }
    if (cur_port != 0 && cur_port != (cur_tls ? 443 : 80)) {
        snprintf(out_url, out_len, "%s://%s:%u%s%s", scheme, cur_host, cur_port, dir, loc);
    } else {
        snprintf(out_url, out_len, "%s://%s%s%s", scheme, cur_host, dir, loc);
    }
    return true;
}

bool https_get(const char *url, String *body, const char *user_agent, uint32_t connect_ms, uint32_t io_ms,
               const char *accept, bool force_close, bool (*early_ok)(const char *buf, size_t len)) {
    // Mutable URL buffer so we can follow redirects without recursion.
    static char cur_url[768];
    if (!url || !url[0]) {
        set_err("Bad URL");
        return false;
    }
    strncpy(cur_url, url, sizeof(cur_url) - 1);
    cur_url[sizeof(cur_url) - 1] = 0;

    const char *ua = user_agent ? user_agent : kUa;
    const char *acc = (accept && accept[0]) ? accept : "application/json";
    g_http_early_ok = early_ok;

    for (int hop = 0; hop < 6; hop++) {
        bool tls = false;
        uint16_t port = 80;
        char host[80];
        // Static buffer: keep path off the net-task stack.
        static char path[2048];
        if (!parse_url(cur_url, &tls, host, sizeof(host), &port, path, sizeof(path))) {
            set_err("Bad URL");
            return false;
        }
        Serial.printf("GET %s://%s:%u%s\n", tls ? "https" : "http", host, port, path);

        bool ok = false;
        if (tls) {
            // Always close between hops — redirect often changes host.
            tls_close();
            NetworkClientSecure *client = tls_acquire(host, port, connect_ms, io_ms);
            if (!client) {
                return false;
            }
            client->printf("GET %s HTTP/1.1\r\n", path);
            client->printf("Host: %s\r\n", host);
            client->printf("User-Agent: %s\r\n", ua);
            client->printf("Accept: %s\r\n", acc);
            client->print("Accept-Encoding: identity\r\n");
            // Redirects + large bodies: prefer close so Location parsing isn't kept-alive junk.
            client->print("Connection: close\r\n\r\n");
            ok = read_http_body(*client, body, io_ms, false);
            tls_close();
            (void)force_close;
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
            client.printf("Accept: %s\r\n", acc);
            client.print("Connection: close\r\n\r\n");
            ok = read_http_body(client, body, io_ms, false);
            client.stop();
        }
        if (ok) {
            g_last_err[0] = 0;
            g_http_early_ok = nullptr;
            return true;
        }
        if (strcmp(g_last_err, "HTTP redirect") != 0 || !g_redirect_loc[0]) {
            g_http_early_ok = nullptr;
            return false;
        }
        static char next_url[768];
        if (!resolve_redirect_url(tls, host, port, path, g_redirect_loc, next_url, sizeof(next_url))) {
            set_err("Bad redirect Location");
            g_http_early_ok = nullptr;
            return false;
        }
        Serial.printf("HTTP redirect → %s\n", next_url);
        strncpy(cur_url, next_url, sizeof(cur_url) - 1);
        cur_url[sizeof(cur_url) - 1] = 0;
        g_redirect_loc[0] = 0;
    }
    set_err("Too many redirects");
    g_http_early_ok = nullptr;
    return false;
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

int net_fetch_airports(float lat, float lon, float range_nm, Airport *out, size_t out_max, bool class_b_only) {
    // Onboard OurAirports DB: airports with longest runway >= 3500 ft.
    // class_b_only (follow mode): large hubs only (rec.size == 0).
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
        if (class_b_only && rec.size != 0) {
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
        dst.is_turf = r.is_turf != 0;
        dst.east_ft = (int)r.east_ft;
        dst.north_ft = (int)r.north_ft;
        n++;
        if ((size_t)n >= out_max) {
            break;
        }
    }
    return n;
}

bool net_lookup_airport(const char *icao, float *lat, float *lon, int *rwy_ft, int *elev_ft) {
    if (!icao || !icao[0] || !kAptDb || K_APT_DB_COUNT == 0) {
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
        if (elev_ft) {
            *elev_ft = (int)rec.elev_ft;
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

// =============================================================================
// Recovered API surface (added to match ui.cpp / main.cpp on this branch).
// Real: ADS-B solo lookups, airport/IATA lookups, AWC radios + city labels, airport
// search, flight-number parsing, SMTP email, SD data-pack download.
// FlightStats schedule, FlightAware plan, airport board and route-flights scrapers are
// recovered from agent transcripts (see the section above the SMTP email code).
// =============================================================================

static void upper_inplace(char *s) {
    for (; s && *s; s++) {
        *s = (char)toupper((unsigned char)*s);
    }
}

// ---- Solo ADS-B (followed aircraft) ----------------------------------------

static int adsb_solo(const char *path_fmt_lol, const char *path_fmt_fi, const char *key, Aircraft *out) {
    if (!key || !key[0] || !out) {
        return -1;
    }
    char url[160];
    bool any_ok = false;
    for (int src = 0; src < 2; src++) {
        snprintf(url, sizeof(url), src == 0 ? path_fmt_lol : path_fmt_fi, key);
        if (!https_get(url, nullptr, kUa, 5000, 10000)) {
            continue;
        }
        size_t blen = 0;
        const char *buf = http_body_buf(&blen);
        const int n = parse_adsblol(buf, blen, out, 1);
        if (n > 0) {
            return n;
        }
        if (n == 0) {
            any_ok = true;
        }
    }
    return any_ok ? 0 : -1;
}

int net_fetch_aircraft_by_hex(const char *hex, Aircraft *out) {
    if (!hex || !hex[0]) {
        return -1;
    }
    char h[16];
    copy_trunc(h, sizeof(h), hex);
    for (char *p = h; *p; p++) {
        *p = (char)tolower((unsigned char)*p);
    }
    return adsb_solo("https://api.adsb.lol/v2/hex/%s", "https://opendata.adsb.fi/api/v2/hex/%s", h, out);
}

int net_fetch_aircraft_by_callsign(const char *callsign, Aircraft *out) {
    if (!callsign || !callsign[0]) {
        return -1;
    }
    char cs[16];
    copy_trunc(cs, sizeof(cs), callsign);
    for (char *p = cs; *p; p++) {
        if (*p == ' ') {
            *p = 0;
            break;
        }
    }
    upper_inplace(cs);
    return adsb_solo("https://api.adsb.lol/v2/callsign/%s", "https://opendata.adsb.fi/api/v2/callsign/%s", cs, out);
}

// ---- Airport lookups from the SD-loaded DB ----------------------------------

bool net_lookup_airport_iata(const char *code, char *icao_out, size_t icao_len, float *lat, float *lon) {
    if (icao_out && icao_len) {
        icao_out[0] = 0;
    }
    if (!code || !code[0] || !kAptDb || K_APT_DB_COUNT == 0) {
        return false;
    }
    char c[8];
    copy_trunc(c, sizeof(c), code);
    upper_inplace(c);
    const size_t cl = strlen(c);
    if (cl < 3 || cl > 4) {
        return false;
    }
    for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
        const AptRec &rec = kAptDb[i];
        const bool hit = (cl == 3) ? (rec.iata[0] && strcmp(rec.iata, c) == 0) : (strcmp(rec.icao, c) == 0);
        if (!hit) {
            continue;
        }
        if (icao_out && icao_len) {
            copy_trunc(icao_out, icao_len, rec.icao);
        }
        if (lat) {
            *lat = rec.lat_c / 100.0f;
        }
        if (lon) {
            *lon = rec.lon_c / 100.0f;
        }
        return true;
    }
    return false;
}

int net_search_airports(const char *query, AptSearchHit *out, size_t out_max) {
    if (!query || !out || out_max == 0) {
        return 0;
    }
    char q[16];
    copy_trunc(q, sizeof(q), query);
    upper_inplace(q);
    const size_t ql = strlen(q);
    if (ql == 0) {
        return 0;
    }
    char kq[8] = {0};
    if (ql == 3) {
        snprintf(kq, sizeof(kq), "K%s", q);
    }
    int n = 0;
    for (int pass = 0; pass < 2 && (size_t)n < out_max; pass++) {
        for (size_t i = 0; i < K_APT_DB_COUNT && (size_t)n < out_max; i++) {
            const AptRec &rec = kAptDb[i];
            bool hit;
            if (pass == 0) {
                hit = strcmp(rec.icao, q) == 0 || (rec.iata[0] && strcmp(rec.iata, q) == 0) ||
                      (kq[0] && strcmp(rec.icao, kq) == 0);
            } else {
                hit = ql >= 2 && (strncmp(rec.icao, q, ql) == 0 || (rec.iata[0] && strncmp(rec.iata, q, ql) == 0));
            }
            if (!hit) {
                continue;
            }
            bool dup = false;
            for (int j = 0; j < n; j++) {
                if (strcmp(out[j].icao, rec.icao) == 0) {
                    dup = true;
                    break;
                }
            }
            if (dup) {
                continue;
            }
            AptSearchHit &h = out[n++];
            memset(&h, 0, sizeof(h));
            copy_trunc(h.icao, sizeof(h.icao), rec.icao);
            copy_trunc(h.iata, sizeof(h.iata), rec.iata);
            snprintf(h.label, sizeof(h.label), rec.iata[0] ? "%s / %s" : "%s", rec.icao, rec.iata);
            h.lat = rec.lat_c / 100.0f;
            h.lon = rec.lon_c / 100.0f;
            h.valid = true;
        }
    }
    return n;
}

// ---- AWC airport feed: city names + radio frequencies -----------------------

/** "JACKSONVILLE INTL" -> "Jacksonville". */
static void city_from_facility(const char *name, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (!name || !name[0]) {
        return;
    }
    char tmp[64];
    copy_trunc(tmp, sizeof(tmp), name);
    static const char *kCut[] = {" INTL", " INTERNATIONAL", " RGNL", " REGIONAL", " MUNI", " MUNICIPAL", " AIRPORT",
                                 " FIELD", " FLD", " AIR", " ARPT", "/", ","};
    char up[64];
    strncpy(up, tmp, sizeof(up) - 1);
    up[sizeof(up) - 1] = 0;
    upper_inplace(up);
    size_t cut = strlen(tmp);
    for (size_t k = 0; k < sizeof(kCut) / sizeof(kCut[0]); k++) {
        const char *f = strstr(up, kCut[k]);
        if (f && (size_t)(f - up) < cut && (size_t)(f - up) > 0) {
            cut = (size_t)(f - up);
        }
    }
    tmp[cut] = 0;
    trim_inplace(tmp);
    bool start = true;
    size_t o = 0;
    for (size_t i = 0; tmp[i] && o + 1 < out_len; i++) {
        const unsigned char c = (unsigned char)tmp[i];
        out[o++] = (char)(start ? toupper(c) : tolower(c));
        start = (c == ' ' || c == '-' || c == '.');
    }
    out[o] = 0;
}

static bool awc_airports(const char *ids, JsonDocument &doc) {
    char url[256];
    snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/airport?ids=%s&format=json", ids);
    String body;
    if (!https_get(url, &body, kUa, 5000, 8000)) {
        return false;
    }
    return !deserializeJson(doc, body) && doc.is<JsonArray>();
}

static void fmt_freq(const char *num, char *out, size_t out_len) {
    // 118.700 -> 118.70 (keep 2 decimals minimum; trailing zero trimmed from 3).
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%.3f", atof(num));
    size_t l = strlen(tmp);
    if (l > 3 && tmp[l - 1] == '0') {
        tmp[l - 1] = 0;
    }
    copy_trunc(out, out_len, tmp);
}

static void parse_awc_freqs(const char *freqs, AirportRadios *out) {
    if (!freqs || !freqs[0]) {
        return;
    }
    char buf[256];
    strncpy(buf, freqs, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    char *save = nullptr;
    for (char *tok = strtok_r(buf, ";", &save); tok; tok = strtok_r(nullptr, ";", &save)) {
        // Frequency = last "ddd.ddd" run; label = everything before it.
        char *dot = strchr(tok, '.');
        if (!dot) {
            continue;
        }
        char *fs = dot;
        while (fs > tok && isdigit((unsigned char)fs[-1])) {
            fs--;
        }
        char num[16];
        size_t nl = 0;
        for (char *p = fs; *p && (isdigit((unsigned char)*p) || *p == '.') && nl + 1 < sizeof(num); p++) {
            num[nl++] = *p;
        }
        num[nl] = 0;
        char label[40];
        size_t ll = (size_t)(fs - tok);
        if (ll >= sizeof(label)) {
            ll = sizeof(label) - 1;
        }
        memcpy(label, tok, ll);
        label[ll] = 0;
        upper_inplace(label);
        char f[12];
        fmt_freq(num, f, sizeof(f));
        if (strstr(label, "ATIS") && !out->atis[0]) {
            copy_trunc(out->atis, sizeof(out->atis), f);
            out->atis_is_awos = false;
        } else if ((strstr(label, "AWOS") || strstr(label, "ASOS")) && !out->atis[0]) {
            copy_trunc(out->atis, sizeof(out->atis), f);
            out->atis_is_awos = true;
        } else if ((strstr(label, "TWR") || strstr(label, "TOWER") || strstr(label, "LCL")) && !out->atc[0]) {
            copy_trunc(out->atc, sizeof(out->atc), f);
            out->atc_is_center = false;
        } else if ((strstr(label, "GND") || strstr(label, "GROUND")) && !out->gnd[0]) {
            copy_trunc(out->gnd, sizeof(out->gnd), f);
        } else if ((strstr(label, "CTAF") || strstr(label, "UNICOM")) && !out->ctaf[0]) {
            copy_trunc(out->ctaf, sizeof(out->ctaf), f);
        } else if ((strstr(label, "CNTR") || strstr(label, "CENTER") || strstr(label, "ARTCC")) && !out->atc[0]) {
            copy_trunc(out->atc, sizeof(out->atc), f);
            out->atc_is_center = true;
        }
    }
}

bool net_lookup_airport_radios(const char *icao, AirportRadios *out) {
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!icao || !icao[0]) {
        return false;
    }
    JsonDocument doc;
    if (!awc_airports(icao, doc) || doc.as<JsonArray>().size() == 0) {
        return false;
    }
    JsonObject a = doc[0];
    const char *freqs = a["freqs"] | "";
    parse_awc_freqs(freqs, out);
    return out->ctaf[0] || out->atc[0] || out->gnd[0] || out->atis[0];
}

bool net_enrich_airport_home_label(GeoFix *fix) {
    if (!fix || !fix->icao[0] || strstr(fix->label, " - ")) {
        return false;
    }
    JsonDocument doc;
    if (!awc_airports(fix->icao, doc) || doc.as<JsonArray>().size() == 0) {
        return false;
    }
    char city[40];
    city_from_facility(doc[0]["name"] | "", city, sizeof(city));
    if (!city[0]) {
        return false;
    }
    char icao[8];
    copy_trunc(icao, sizeof(icao), fix->icao);
    upper_inplace(icao);
    snprintf(fix->label, sizeof(fix->label), "%s - %s", icao, city);
    return true;
}

int net_fetch_airport_cities(const char icaos[][8], char cities[][28], int n) {
    if (n <= 0) {
        return 0;
    }
    char ids[200] = {0};
    size_t pos = 0;
    for (int i = 0; i < n; i++) {
        cities[i][0] = 0;
        const size_t l = strnlen(icaos[i], 8);
        if (l == 0 || pos + l + 2 >= sizeof(ids)) {
            continue;
        }
        if (pos) {
            ids[pos++] = ',';
        }
        memcpy(ids + pos, icaos[i], l);
        pos += l;
        ids[pos] = 0;
    }
    if (!pos) {
        return 0;
    }
    JsonDocument doc;
    if (!awc_airports(ids, doc)) {
        return 0;
    }
    int hits = 0;
    for (JsonObject a : doc.as<JsonArray>()) {
        const char *id = a["icaoId"] | "";
        for (int i = 0; i < n; i++) {
            if (strcasecmp(icaos[i], id) == 0 && !cities[i][0]) {
                city_from_facility(a["name"] | "", cities[i], 28);
                if (cities[i][0]) {
                    hits++;
                }
            }
        }
    }
    return hits;
}

// ---- Flight number parsing --------------------------------------------------

struct AirlineMap {
    const char *iata;
    const char *icao;
};
static const AirlineMap kAirlines[] = {
    {"AA", "AAL"}, {"UA", "UAL"}, {"DL", "DAL"}, {"WN", "SWA"}, {"B6", "JBU"}, {"AS", "ASA"}, {"NK", "NKS"},
    {"F9", "FFT"}, {"G4", "AAY"}, {"HA", "HAL"}, {"SY", "SCX"}, {"AC", "ACA"}, {"BA", "BAW"}, {"LH", "DLH"},
    {"AF", "AFR"}, {"KL", "KLM"}, {"EK", "UAE"}, {"QR", "QTR"}, {"QF", "QFA"}, {"AM", "AMX"}, {"WS", "WJA"},
    {"9E", "EDV"}, {"OO", "SKW"}, {"YX", "RPA"}, {"MQ", "ENY"}, {"OH", "JIA"}, {"YV", "ASH"}, {"CP", "CPZ"},
    {"FX", "FDX"}, {"5X", "UPS"}, {"EI", "EIN"}, {"IB", "IBE"}, {"TK", "THY"}, {"VS", "VIR"}, {"SQ", "SIA"},
    {"NH", "ANA"}, {"JL", "JAL"}, {"CX", "CPA"}, {"LX", "SWR"}, {"SK", "SAS"}, {"AY", "FIN"}, {"AZ", "ITY"},
    {"TP", "TAP"}, {"LA", "LAN"}, {"CM", "CMP"}, {"AV", "AVA"}, {"KE", "KAL"}, {"OZ", "AAR"}, {"EY", "ETD"},
    {"TG", "THA"}, {"VX", "VRD"}, {"XP", "CXP"}, {"MX", "MXY"}, {"QX", "QXE"}, {"ZW", "AWI"},
};

bool net_parse_flight_number(const char *query, char *icao_cs, size_t icao_cs_len, char *iata_al,
                             size_t iata_al_len, char *iata_num, size_t iata_num_len) {
    if (icao_cs && icao_cs_len) {
        icao_cs[0] = 0;
    }
    if (iata_al && iata_al_len) {
        iata_al[0] = 0;
    }
    if (iata_num && iata_num_len) {
        iata_num[0] = 0;
    }
    if (!query) {
        return false;
    }
    char q[24];
    size_t qn = 0;
    for (size_t i = 0; query[i] && qn + 1 < sizeof(q); i++) {
        if (!isspace((unsigned char)query[i]) && query[i] != '-') {
            q[qn++] = (char)toupper((unsigned char)query[i]);
        }
    }
    q[qn] = 0;
    // Digits run at the end = flight number; the rest is the airline designator.
    size_t d = qn;
    while (d > 0 && isdigit((unsigned char)q[d - 1])) {
        d--;
    }
    const size_t al_len = d;
    const size_t num_len = qn - d;
    if (num_len == 0 || num_len > 4 || (al_len != 2 && al_len != 3)) {
        return false;
    }
    char al[4] = {0};
    memcpy(al, q, al_len);
    const char *num = q + d;
    const char *map_icao = nullptr;
    const char *map_iata = nullptr;
    for (size_t i = 0; i < sizeof(kAirlines) / sizeof(kAirlines[0]); i++) {
        if (al_len == 3 && strcmp(kAirlines[i].icao, al) == 0) {
            map_icao = kAirlines[i].icao;
            map_iata = kAirlines[i].iata;
            break;
        }
        if (al_len == 2 && strcmp(kAirlines[i].iata, al) == 0) {
            map_icao = kAirlines[i].icao;
            map_iata = kAirlines[i].iata;
            break;
        }
    }
    if (al_len == 3) {
        if (icao_cs && icao_cs_len) {
            snprintf(icao_cs, icao_cs_len, "%s%s", al, num);
        }
        if (map_iata && iata_al && iata_al_len) {
            copy_trunc(iata_al, iata_al_len, map_iata);
        }
    } else {
        if (iata_al && iata_al_len) {
            copy_trunc(iata_al, iata_al_len, al);
        }
        if (map_icao && icao_cs && icao_cs_len) {
            snprintf(icao_cs, icao_cs_len, "%s%s", map_icao, num);
        }
    }
    if (iata_num && iata_num_len) {
        copy_trunc(iata_num, iata_num_len, num);
    }
    return true;
}

// ---- FlightStats schedule / FlightAware plan / airport board / route flights (recovered) ----

static const char *icao_to_iata(const char *icao3) {
    if (!icao3 || strlen(icao3) != 3) {
        return nullptr;
    }
    for (size_t i = 0; i < sizeof(kAirlines) / sizeof(kAirlines[0]); i++) {
        if (strcasecmp(kAirlines[i].icao, icao3) == 0) {
            return kAirlines[i].iata;
        }
    }
    return nullptr;
}

static const char *iata_to_icao(const char *iata2) {
    if (!iata2 || strlen(iata2) != 2) {
        return nullptr;
    }
    for (size_t i = 0; i < sizeof(kAirlines) / sizeof(kAirlines[0]); i++) {
        if (strcasecmp(kAirlines[i].iata, iata2) == 0) {
            return kAirlines[i].icao;
        }
    }
    return nullptr;
}

static char *match_brace_end(char *open) {
    if (!open || *open != '{') {
        return nullptr;
    }
    int depth = 0;
    bool in_str = false;
    bool esc = false;
    for (char *p = open; *p; p++) {
        const char c = *p;
        if (in_str) {
            if (esc) {
                esc = false;
            } else if (c == '\\') {
                esc = true;
            } else if (c == '"') {
                in_str = false;
            }
            continue;
        }
        if (c == '"') {
            in_str = true;
        } else if (c == '{') {
            depth++;
        } else if (c == '}') {
            depth--;
            if (depth == 0) {
                return p;
            }
        }
    }
    return nullptr;
}

static char *extract_flightstats_tracker(char *html) {
    char *marker = strstr(html, "\"flightTracker\":{");
    if (!marker) {
        marker = strstr(html, "\"flightTracker\": {");
    }
    if (!marker) {
        return nullptr;
    }
    char *start = strchr(marker + strlen("\"flightTracker\""), '{');
    if (!start) {
        return nullptr;
    }
    char *end = match_brace_end(start);
    if (!end) {
        return nullptr;
    }
    end[1] = 0;
    return start;
}

/** Civil date → days since 1970-01-01 (Howard Hinnant). */
static long civil_days_unix(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return (long)era * 146097L + (long)doe - 719468L;
}

/** True UTC epoch from ISO — does not depend on device TZ (unlike mktime). */
static long timegm_civil(int y, int mo, int d, int h, int mi, int s) {
    if (y < 2020 || mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59 ||
        s < 0 || s > 60) {
        return 0;
    }
    const long days = civil_days_unix(y, (unsigned)mo, (unsigned)d);
    return days * 86400L + (long)h * 3600L + (long)mi * 60L + (long)s;
}

static long parse_iso_utc(const char *iso) {
    if (!iso || strlen(iso) < 19) {
        return 0;
    }
    const char *z = strrchr(iso, 'Z');
    const char *p00 = strstr(iso, "+00:00");
    if (!z && !p00) {
        // Also accept trailing +0000
        const size_t n = strlen(iso);
        if (n < 5 || !(iso[n - 5] == '+' && iso[n - 4] == '0' && iso[n - 3] == '0' && iso[n - 2] == '0' &&
                       iso[n - 1] == '0')) {
            return 0;
        }
    }
    int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
    if (sscanf(iso, "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &s) < 5) {
        return 0;
    }
    return timegm_civil(y, mo, d, h, mi, s);
}

static void format_fs_local_time(JsonObjectConst times_obj, const char *which, char *out, size_t out_len,
                                 char *title_out, size_t title_len) {
    if (out && out_len) {
        out[0] = 0;
    }
    if (title_out && title_len) {
        title_out[0] = 0;
    }
    if (times_obj.isNull() || !which) {
        return;
    }
    JsonObjectConst block = times_obj[which];
    if (block.isNull()) {
        return;
    }
    const char *t24 = block["time24"] | "";
    const char *tz = block["timezone"] | "";
    const char *title = block["title"] | "";
    if (t24[0] && out && out_len) {
        if (tz[0]) {
            snprintf(out, out_len, "%s %s", t24, tz);
        } else {
            copy_trunc(out, out_len, t24);
        }
    }
    if (title[0] && title_out && title_len) {
        copy_trunc(title_out, title_len, title);
    }
}

/** UTC epoch from a FlightStats times block (dateUtc), or 0 if missing/unparsed. */
static uint32_t fs_times_utc_epoch(JsonObjectConst times_obj, const char *which) {
    if (times_obj.isNull() || !which) {
        return 0;
    }
    JsonObjectConst block = times_obj[which];
    if (block.isNull()) {
        return 0;
    }
    const char *utc = block["dateUtc"] | "";
    long e = parse_iso_utc(utc);
    if (e <= 0 && utc[0] && strlen(utc) >= 19) {
        // Some tracker payloads omit the trailing Z on dateUtc — field name is authoritative.
        int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
        if (sscanf(utc, "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &s) >= 5) {
            e = timegm_civil(y, mo, d, h, mi, s);
        }
    }
    return (e > 0) ? (uint32_t)e : 0;
}

static void fs_fill_airport(JsonObjectConst apt, char *icao, size_t icao_len, char *name, size_t name_len,
                            char *gate, size_t gate_len, char *term, size_t term_len) {
    if (icao && icao_len) {
        icao[0] = 0;
    }
    if (name && name_len) {
        name[0] = 0;
    }
    if (gate && gate_len) {
        gate[0] = 0;
    }
    if (term && term_len) {
        term[0] = 0;
    }
    if (apt.isNull()) {
        return;
    }
    const char *fs = apt["fs"] | "";
    const char *ia = apt["iata"] | "";
    const char *code = ia[0] ? ia : fs;
    if (icao && icao_len && code[0]) {
        char resolved[8] = {0};
        float lat = 0, lon = 0;
        if (net_lookup_airport_iata(code, resolved, sizeof(resolved), &lat, &lon) && resolved[0]) {
            copy_trunc(icao, icao_len, resolved);
        } else {
            copy_trunc(icao, icao_len, code);
        }
    }
    const char *nm = apt["name"] | "";
    if (name && name_len && nm[0]) {
        copy_trunc(name, name_len, nm);
    }
    if (gate && gate_len && !apt["gate"].isNull()) {
        if (apt["gate"].is<const char *>()) {
            copy_trunc(gate, gate_len, apt["gate"] | "");
        } else if (apt["gate"].is<int>()) {
            snprintf(gate, gate_len, "%d", apt["gate"].as<int>());
        }
    }
    if (term && term_len && !apt["terminal"].isNull()) {
        if (apt["terminal"].is<const char *>()) {
            copy_trunc(term, term_len, apt["terminal"] | "");
        } else if (apt["terminal"].is<int>()) {
            snprintf(term, term_len, "%d", apt["terminal"].as<int>());
        }
    }
}

static bool fs_flight_is_live(JsonObjectConst fl) {
    if (fl.isNull()) {
        return false;
    }
    if (fl["isTracking"] | false) {
        return true;
    }
    if (fl["isLanded"] | false) {
        return true;
    }
    JsonObjectConst note = fl["flightNote"];
    if (!note.isNull()) {
        if (note["tracking"] | false) {
            return true;
        }
        if (note["hasDepartedGate"] | false) {
            return true;
        }
        if (note["hasDepartedRunway"] | false) {
            return true;
        }
        if (note["landed"] | false) {
            return true;
        }
    }
    const char *st = fl["status"]["status"] | "";
    const char *code = fl["status"]["statusCode"] | "";
    const char *state = fl["flightState"] | "";
    if (strcasecmp(code, "S") == 0) {
        return false;
    }
    if (st[0] && strcasecmp(st, "Scheduled") != 0) {
        return true;
    }
    if (state[0] && strcasecmp(state, "scheduled") != 0) {
        return true;
    }
    return false;
}

static void fs_apply_flight(JsonObjectConst fl, FlightInfo *fi) {
    if (fl.isNull() || !fi) {
        return;
    }
    JsonObjectConst note = fl["flightNote"];
    const bool left_gate = !note.isNull() && (note["hasDepartedGate"] | false);
    const bool left_rwy = !note.isNull() && (note["hasDepartedRunway"] | false);
    const bool tracking = (fl["isTracking"] | false) || (!note.isNull() && (note["tracking"] | false));
    const bool landed = (fl["isLanded"] | false) || (!note.isNull() && (note["landed"] | false));
    fi->departed = left_gate || left_rwy || tracking || landed;

    const char *phase = !note.isNull() ? (note["phase"] | "") : "";
    const char *st = fl["status"]["status"] | "";
    const char *st_code = fl["status"]["statusCode"] | "";
    const char *state = fl["flightState"] | "";
    const char *desc = fl["status"]["delayStatus"] | fl["status"]["statusDescription"] | "";
    // Single status only — concatenating with " · " draws as a missing-glyph box in LVGL fonts.
    // Prefer operational phase (Boarding / Climbing / Cruising / …) when present.
    if (phase[0]) {
        copy_trunc(fi->status, sizeof(fi->status), phase);
    } else if (st[0]) {
        copy_trunc(fi->status, sizeof(fi->status), st);
    } else if (desc[0]) {
        copy_trunc(fi->status, sizeof(fi->status), desc);
    } else if (state[0]) {
        copy_trunc(fi->status, sizeof(fi->status), state);
    }
    // Status codes / wording that clearly mean airborne or completed.
    if (!fi->departed) {
        if (strcasecmp(st_code, "A") == 0 || strcasecmp(st_code, "L") == 0 || strcasecmp(st_code, "D") == 0) {
            // A=active/airborne, L=landed, D=diverted — treat as departed from origin.
            fi->departed = true;
        }
        if (st[0] && (strcasestr(st, "en route") || strcasestr(st, "enroute") || strcasestr(st, "airborne") ||
                      strcasestr(st, "landed") || strcasestr(st, "arrived"))) {
            fi->departed = true;
        }
        if (state[0] && (strcasecmp(state, "airborne") == 0 || strcasecmp(state, "landed") == 0 ||
                         strcasecmp(state, "active") == 0)) {
            fi->departed = true;
        }
    }

    fs_fill_airport(fl["departureAirport"], fi->origin, sizeof(fi->origin), fi->origin_name,
                    sizeof(fi->origin_name), fi->dep_gate, sizeof(fi->dep_gate), fi->dep_terminal,
                    sizeof(fi->dep_terminal));
    fs_fill_airport(fl["arrivalAirport"], fi->dest, sizeof(fi->dest), fi->dest_name, sizeof(fi->dest_name),
                    fi->arr_gate, sizeof(fi->arr_gate), fi->arr_terminal, sizeof(fi->arr_terminal));

    auto fs_title_is_actual = [](const char *title) -> bool {
        if (!title || !title[0]) {
            return false;
        }
        return strcasestr(title, "actual") != nullptr && strcasestr(title, "estim") == nullptr;
    };
    auto fs_short_iso = [](const char *s, char *buf, size_t buf_len) {
        if (!buf || buf_len == 0) {
            return;
        }
        buf[0] = 0;
        if (!s || !s[0]) {
            return;
        }
        copy_trunc(buf, buf_len, s);
        if (strlen(buf) >= 16) {
            buf[16] = 0;  // YYYY-MM-DDTHH:MM
        }
    };
    auto fs_fill_est_act = [&](JsonObjectConst times_obj, char *est, size_t est_len, char *act, size_t act_len) {
        if (est && est_len) {
            est[0] = 0;
        }
        if (act && act_len) {
            act[0] = 0;
        }
        format_fs_local_time(times_obj, "estimated", est, est_len, nullptr, 0);
        format_fs_local_time(times_obj, "actual", act, act_len, nullptr, 0);
        if ((est && est[0]) || (act && act[0])) {
            return;
        }
        char ea[28] = {0}, title[16] = {0};
        format_fs_local_time(times_obj, "estimatedActual", ea, sizeof(ea), title, sizeof(title));
        if (!ea[0]) {
            return;
        }
        if (fs_title_is_actual(title)) {
            if (act && act_len) {
                copy_trunc(act, act_len, ea);
            }
        } else if (est && est_len) {
            copy_trunc(est, est_len, ea);
        }
    };

    format_fs_local_time(fl["departureAirport"]["times"], "scheduled", fi->dep_time, sizeof(fi->dep_time), nullptr,
                         0);
    format_fs_local_time(fl["arrivalAirport"]["times"], "scheduled", fi->arr_time, sizeof(fi->arr_time), nullptr, 0);
    fs_fill_est_act(fl["departureAirport"]["times"], fi->dep_est, sizeof(fi->dep_est), fi->dep_act,
                    sizeof(fi->dep_act));
    fs_fill_est_act(fl["arrivalAirport"]["times"], fi->arr_est, sizeof(fi->arr_est), fi->arr_act,
                    sizeof(fi->arr_act));

    // Actual takeoff / arrival UTC epochs for elapsed flight-time (now − takeoff).
    fi->dep_act_epoch = fs_times_utc_epoch(fl["departureAirport"]["times"], "actual");
    if (!fi->dep_act_epoch) {
        // estimatedActual may be labeled Actual once wheels-up is known.
        JsonObjectConst dep_times = fl["departureAirport"]["times"];
        if (!dep_times.isNull()) {
            JsonObjectConst ea = dep_times["estimatedActual"];
            if (!ea.isNull()) {
                char title[16] = {0};
                copy_trunc(title, sizeof(title), ea["title"] | "");
                if (fs_title_is_actual(title)) {
                    fi->dep_act_epoch = fs_times_utc_epoch(dep_times, "estimatedActual");
                }
            }
        }
    }
    fi->arr_act_epoch = fs_times_utc_epoch(fl["arrivalAirport"]["times"], "actual");
    fi->arr_est_epoch = fs_times_utc_epoch(fl["arrivalAirport"]["times"], "estimated");
    if (!fi->arr_act_epoch || !fi->arr_est_epoch) {
        JsonObjectConst arr_times = fl["arrivalAirport"]["times"];
        if (!arr_times.isNull()) {
            JsonObjectConst ea = arr_times["estimatedActual"];
            if (!ea.isNull()) {
                char title[16] = {0};
                copy_trunc(title, sizeof(title), ea["title"] | "");
                const uint32_t ea_ep = fs_times_utc_epoch(arr_times, "estimatedActual");
                if (fs_title_is_actual(title)) {
                    if (!fi->arr_act_epoch) {
                        fi->arr_act_epoch = ea_ep;
                    }
                } else if (!fi->arr_est_epoch) {
                    fi->arr_est_epoch = ea_ep;
                }
            }
        }
    }

    // UTC schedule fallbacks when local airport times missing.
    JsonObjectConst sched = fl["schedule"];
    if (!sched.isNull()) {
        if (!fi->dep_time[0]) {
            char buf[20];
            fs_short_iso(sched["scheduledDeparture"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->dep_time, sizeof(fi->dep_time), buf);
            }
        }
        if (!fi->arr_time[0]) {
            char buf[20];
            fs_short_iso(sched["scheduledArrival"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->arr_time, sizeof(fi->arr_time), buf);
            }
        }
        if (!fi->dep_est[0] && !fi->dep_act[0]) {
            char buf[20];
            fs_short_iso(sched["estimatedActualDeparture"] | "", buf, sizeof(buf));
            const char *title = sched["estimatedActualDepartureTitle"] | "";
            if (buf[0]) {
                if (fs_title_is_actual(title)) {
                    copy_trunc(fi->dep_act, sizeof(fi->dep_act), buf);
                } else {
                    copy_trunc(fi->dep_est, sizeof(fi->dep_est), buf);
                }
            }
        }
        if (!fi->arr_est[0] && !fi->arr_act[0]) {
            char buf[20];
            fs_short_iso(sched["estimatedActualArrival"] | "", buf, sizeof(buf));
            const char *title = sched["estimatedActualArrivalTitle"] | "";
            if (buf[0]) {
                if (fs_title_is_actual(title)) {
                    copy_trunc(fi->arr_act, sizeof(fi->arr_act), buf);
                } else {
                    copy_trunc(fi->arr_est, sizeof(fi->arr_est), buf);
                }
            }
        }
        // Prefer dedicated estimated / actual UTC fields when present.
        if (!fi->dep_est[0]) {
            char buf[20];
            fs_short_iso(sched["estimatedDeparture"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->dep_est, sizeof(fi->dep_est), buf);
            }
        }
        if (!fi->dep_act[0]) {
            char buf[20];
            fs_short_iso(sched["actualDeparture"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->dep_act, sizeof(fi->dep_act), buf);
            }
        }
        if (!fi->arr_est[0]) {
            char buf[20];
            fs_short_iso(sched["estimatedArrival"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->arr_est, sizeof(fi->arr_est), buf);
            }
        }
        if (!fi->arr_act[0]) {
            char buf[20];
            fs_short_iso(sched["actualArrival"] | "", buf, sizeof(buf));
            if (buf[0]) {
                copy_trunc(fi->arr_act, sizeof(fi->arr_act), buf);
            }
        }
        if (!fi->dep_act_epoch) {
            long e = parse_iso_utc(sched["actualDeparture"] | "");
            if (e <= 0) {
                const char *title = sched["estimatedActualDepartureTitle"] | "";
                if (fs_title_is_actual(title)) {
                    e = parse_iso_utc(sched["estimatedActualDeparture"] | "");
                }
            }
            if (e > 0) {
                fi->dep_act_epoch = (uint32_t)e;
            }
        }
        if (!fi->arr_act_epoch) {
            long e = parse_iso_utc(sched["actualArrival"] | "");
            if (e <= 0) {
                const char *title = sched["estimatedActualArrivalTitle"] | "";
                if (fs_title_is_actual(title)) {
                    e = parse_iso_utc(sched["estimatedActualArrival"] | "");
                }
            }
            if (e > 0) {
                fi->arr_act_epoch = (uint32_t)e;
            }
        }
        if (!fi->arr_est_epoch) {
            long e = parse_iso_utc(sched["estimatedArrival"] | "");
            if (e <= 0) {
                const char *title = sched["estimatedActualArrivalTitle"] | "";
                if (!fs_title_is_actual(title)) {
                    e = parse_iso_utc(sched["estimatedActualArrival"] | "");
                }
            }
            if (e > 0) {
                fi->arr_est_epoch = (uint32_t)e;
            }
        }
    }

    // Actual departure time is a strong signal the flight left the gate.
    if (fi->dep_act[0] || fi->dep_act_epoch) {
        fi->departed = true;
    }

    // Flight calendar date from schedule ISO (prefer scheduled departure).
    auto fs_format_flight_date = [](const char *iso, char *out, size_t out_len) {
        if (!out || out_len == 0) {
            return;
        }
        out[0] = 0;
        if (!iso || !iso[0]) {
            return;
        }
        int y = 0, mo = 0, d = 0;
        if (sscanf(iso, "%d-%d-%d", &y, &mo, &d) != 3 || y < 2000 || mo < 1 || mo > 12 || d < 1 || d > 31) {
            return;
        }
        struct tm tm{};
        tm.tm_year = y - 1900;
        tm.tm_mon = mo - 1;
        tm.tm_mday = d;
        tm.tm_hour = 12;
        tm.tm_isdst = -1;
        if (mktime(&tm) == (time_t)-1) {
            snprintf(out, out_len, "%04d-%02d-%02d", y, mo, d);
            return;
        }
        strftime(out, out_len, "%a %b %d", &tm);  // e.g. Sat Oct 18
    };
    if (!fi->flight_date[0]) {
        const char *date_src = nullptr;
        if (!sched.isNull()) {
            date_src = sched["scheduledDeparture"] | sched["scheduledDepartureUTC"] |
                       sched["estimatedDeparture"] | sched["actualDeparture"] | "";
        }
        if ((!date_src || !date_src[0]) && !fl["departureAirport"]["times"]["scheduled"].isNull()) {
            date_src = fl["departureAirport"]["times"]["scheduled"]["dateLocal"] |
                       fl["departureAirport"]["times"]["scheduled"]["dateUtc"] | "";
        }
        if (date_src && date_src[0]) {
            fs_format_flight_date(date_src, fi->flight_date, sizeof(fi->flight_date));
        }
    }

    if (fi->origin[0] && fi->dest[0]) {
        fi->route_ok = true;
    }
    fi->schedule_ok = fi->dep_time[0] || fi->arr_time[0] || fi->dep_est[0] || fi->arr_est[0] || fi->dep_act[0] ||
                      fi->arr_act[0] || fi->dep_gate[0] || fi->arr_gate[0] || fi->dep_terminal[0] ||
                      fi->arr_terminal[0] || fi->status[0] || fi->route_ok || fi->flight_date[0];
}

static bool fs_pick_current_url(JsonObjectConst tracker, char *url_out, size_t url_len) {
    if (url_out && url_len) {
        url_out[0] = 0;
    }
    if (tracker.isNull() || !url_out || url_len == 0) {
        return false;
    }
    JsonObjectConst cur = tracker["flight"];
    if (fs_flight_is_live(cur)) {
        return false;  // already on the current page
    }

    const time_t now = time(nullptr);
    const bool clock_ok = (now > 1700000000);  // ~2023+
    const char *cur_utc = cur["schedule"]["scheduledDepartureUTC"] | cur["schedule"]["scheduledDeparture"] | "";
    long cur_ts = parse_iso_utc(cur_utc);
    if (cur_ts <= 0) {
        cur_ts = parse_iso_utc(cur["sortTime"] | "");
    }

    long best_ts = 0;
    const char *best_url = nullptr;
    JsonArrayConst days = tracker["otherDays"];
    if (!days.isNull()) {
        for (JsonObjectConst day : days) {
            JsonArrayConst flights = day["flights"];
            if (flights.isNull()) {
                continue;
            }
            for (JsonObjectConst f : flights) {
                const char *u = f["url"] | "";
                const char *st = f["sortTime"] | "";
                if (!u[0] || !st[0]) {
                    continue;
                }
                const long ts = parse_iso_utc(st);
                if (ts <= 0) {
                    continue;
                }
                if (clock_ok) {
                    // Latest departure at or before ~now+2h (covers airborne / just departed).
                    if (ts > (long)now + 2 * 3600) {
                        continue;
                    }
                } else if (cur_ts > 0) {
                    // Default page is often the *next* scheduled sortie — take the latest before it.
                    if (ts >= cur_ts) {
                        continue;
                    }
                }
                if (ts >= best_ts) {
                    best_ts = ts;
                    best_url = u;
                }
            }
        }
    }

    if (!best_url) {
        return false;
    }
    // Relative path like /flight-tracker/AA/100?year=...
    if (best_url[0] == '/') {
        snprintf(url_out, url_len, "https://www.flightstats.com/v2%s", best_url);
    } else if (strncmp(best_url, "http", 4) == 0) {
        copy_trunc(url_out, url_len, best_url);
    } else {
        snprintf(url_out, url_len, "https://www.flightstats.com/v2/flight-tracker/%s", best_url);
    }
    return true;
}

bool net_fetch_flight_schedule(const char *flight_num, FlightInfo *fi) {
    if (!fi || !flight_num || !flight_num[0]) {
        return false;
    }
    char icao_cs[16] = {0}, iata_al[4] = {0}, iata_num[8] = {0};
    if (!net_parse_flight_number(flight_num, icao_cs, sizeof(icao_cs), iata_al, sizeof(iata_al), iata_num,
                                 sizeof(iata_num))) {
        return false;
    }
    if (!iata_al[0] || !iata_num[0]) {
        return false;
    }

    status_msg("FlightStats lookup...");
    char url[192];
    snprintf(url, sizeof(url), "https://www.flightstats.com/v2/flight-tracker/%s/%s", iata_al, iata_num);
    if (!https_get(url, nullptr, kUaBrowser, 8000, 20000)) {
        return false;
    }
    size_t blen = 0;
    if (!http_body_buf(&blen) || blen < 500) {
        return false;
    }
    char *tracker_json = extract_flightstats_tracker(g_http_buf);
    if (!tracker_json) {
        Serial.println("FlightStats: flightTracker missing");
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, tracker_json);
    if (err) {
        Serial.printf("FlightStats JSON: %s\n", err.c_str());
        return false;
    }

    char better[192] = {0};
    if (fs_pick_current_url(doc.as<JsonObjectConst>(), better, sizeof(better))) {
        Serial.printf("FlightStats: refetch current %s\n", better);
        status_msg("FlightStats current...");
        if (https_get(better, nullptr, kUaBrowser, 8000, 20000)) {
            blen = 0;
            if (http_body_buf(&blen) && blen > 500) {
                tracker_json = extract_flightstats_tracker(g_http_buf);
                if (tracker_json) {
                    doc.clear();
                    err = deserializeJson(doc, tracker_json);
                    if (err) {
                        Serial.printf("FlightStats refetch JSON: %s\n", err.c_str());
                    }
                }
            }
        }
    }

    JsonObjectConst fl = doc["flight"];
    if (fl.isNull()) {
        return false;
    }
    if (icao_cs[0] && !fi->flight[0]) {
        copy_trunc(fi->flight, sizeof(fi->flight), icao_cs);
    } else if (!fi->flight[0]) {
        snprintf(fi->flight, sizeof(fi->flight), "%s%s", iata_al, iata_num);
    }
    fs_apply_flight(fl, fi);
    return fi->schedule_ok;
}

static bool decode_icao_ll_fix(const char *tok, float *lat, float *lon) {
    if (!tok || !lat || !lon || !tok[0]) {
        return false;
    }

    // Form: 4500N/07300W or 45N/073W
    const char *slash = strchr(tok, '/');
    if (slash && slash > tok) {
        char ns = (char)toupper((unsigned char)slash[-1]);
        if (ns != 'N' && ns != 'S') {
            return false;
        }
        int lat_digits = (int)(slash - tok - 1);
        if (lat_digits < 2 || lat_digits > 4) {
            return false;
        }
        for (int i = 0; i < lat_digits; i++) {
            if (!isdigit((unsigned char)tok[i])) {
                return false;
            }
        }
        const char *lon_s = slash + 1;
        size_t lon_n = strlen(lon_s);
        if (lon_n < 3 || lon_n > 6) {
            return false;
        }
        char ew = (char)toupper((unsigned char)lon_s[lon_n - 1]);
        if (ew != 'E' && ew != 'W') {
            return false;
        }
        int lon_digits = (int)lon_n - 1;
        if (lon_digits < 3 || lon_digits > 5) {
            return false;
        }
        for (int i = 0; i < lon_digits; i++) {
            if (!isdigit((unsigned char)lon_s[i])) {
                return false;
            }
        }
        int lat_raw = atoi(tok);
        int lon_raw = atoi(lon_s);
        float la, lo;
        if (lat_digits <= 2) {
            la = (float)lat_raw;
        } else {
            // DDMM
            int dd = lat_raw / 100;
            int mm = lat_raw % 100;
            if (mm >= 60 || dd > 90) {
                return false;
            }
            la = (float)dd + (float)mm / 60.0f;
        }
        if (lon_digits <= 3) {
            lo = (float)lon_raw;
        } else {
            // DDDMM
            int ddd = lon_raw / 100;
            int mm = lon_raw % 100;
            if (mm >= 60 || ddd > 180) {
                return false;
            }
            lo = (float)ddd + (float)mm / 60.0f;
        }
        if (la > 90.0f || lo > 180.0f) {
            return false;
        }
        *lat = la * (ns == 'S' ? -1.0f : 1.0f);
        *lon = lo * (ew == 'W' ? -1.0f : 1.0f);
        return true;
    }

    // Form: 23S163E, 05S179W, 01N170W (2° lat, 2–3° lon).
    size_t n = strlen(tok);
    if (n < 5 || n > 7) {
        return false;
    }
    if (!isdigit((unsigned char)tok[0]) || !isdigit((unsigned char)tok[1])) {
        return false;
    }
    char ns = (char)toupper((unsigned char)tok[2]);
    if (ns != 'N' && ns != 'S') {
        return false;
    }
    int lon_digits = 0;
    int lon_i = 3;
    while (tok[lon_i] && isdigit((unsigned char)tok[lon_i])) {
        lon_digits++;
        lon_i++;
    }
    if (lon_digits < 2 || lon_digits > 3 || !tok[lon_i]) {
        return false;
    }
    char ew = (char)toupper((unsigned char)tok[lon_i]);
    if ((ew != 'E' && ew != 'W') || tok[lon_i + 1]) {
        return false;
    }
    int ilat = (tok[0] - '0') * 10 + (tok[1] - '0');
    int ilon = atoi(tok + 3);
    if (ilat > 90 || ilon > 180) {
        return false;
    }
    *lat = (float)ilat * (ns == 'S' ? -1.0f : 1.0f);
    *lon = (float)ilon * (ew == 'W' ? -1.0f : 1.0f);
    return true;
}

static bool fa_is_speed_alt_token(const char *t) {
    // M082F290 / N0482F410
    if (!t || !t[0]) {
        return false;
    }
    char c0 = (char)toupper((unsigned char)t[0]);
    if (c0 != 'M' && c0 != 'N') {
        return false;
    }
    const char *f = strchr(t, 'F');
    if (!f || f == t + 1) {
        return false;
    }
    for (const char *p = t + 1; p < f; p++) {
        if (!isdigit((unsigned char)*p)) {
            return false;
        }
    }
    for (const char *p = f + 1; *p; p++) {
        if (!isdigit((unsigned char)*p)) {
            return false;
        }
    }
    return f[1] != 0;
}

static void fa_format_coord_name(char *out, size_t out_len, float lat, float lon) {
    if (!out || out_len < 8) {
        return;
    }
    int lat_mm = (int)lroundf(fabsf(lat) * 60.0f);
    int lon_mm = (int)lroundf(fabsf(lon) * 60.0f);
    if (lat_mm > 90 * 60) {
        lat_mm = 90 * 60;
    }
    if (lon_mm > 180 * 60) {
        lon_mm = 180 * 60;
    }
    snprintf(out, out_len, "%02d%02d%c/%03d%02d%c", lat_mm / 60, lat_mm % 60, lat >= 0 ? 'N' : 'S',
             lon_mm / 60, lon_mm % 60, lon >= 0 ? 'E' : 'W');
}

static bool fa_nearest_airport_name(char *out, size_t out_len, float lat, float lon, float max_nm) {
    if (!out || out_len < 5) {
        return false;
    }
    float best = max_nm * max_nm;
    const char *best_icao = nullptr;
    for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
        const AptRec &rec = kAptDb[i];
        float alat = rec.lat_c / 100.0f;
        float alon = rec.lon_c / 100.0f;
        float dlat = (alat - lat) * 60.0f;
        float dlon = (alon - lon) * 60.0f * cosf(lat * FR_DEG2RAD);
        float d2 = dlat * dlat + dlon * dlon;
        if (d2 < best) {
            best = d2;
            best_icao = rec.icao;
        }
    }
    if (!best_icao) {
        return false;
    }
    copy_trunc(out, out_len, best_icao);
    return true;
}

static bool fa_is_airway_or_proc(const char *t) {
    // Airways (B474, Q140) and SID/STAR (HTOWN3): letters + digits, not pure alpha.
    if (!t || !t[0]) {
        return false;
    }
    bool has_alpha = false, has_digit = false;
    for (const char *p = t; *p; p++) {
        if (isalpha((unsigned char)*p)) {
            has_alpha = true;
        } else if (isdigit((unsigned char)*p)) {
            has_digit = true;
        } else {
            return false;
        }
    }
    return has_alpha && has_digit;
}

static bool net_lookup_waypoint(const char *name, float *lat, float *lon) {
    if (!name || !name[0]) {
        return false;
    }
    char key[8];
    copy_trunc(key, sizeof(key), name);
    for (char *p = key; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    if (!kWptDb || K_WPT_DB_COUNT == 0) {
        return false;
    }
    // kWptDb is sorted by name — binary search.
    size_t lo = 0, hi = K_WPT_DB_COUNT;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        int cmp = strcmp(kWptDb[mid].name, key);
        if (cmp == 0) {
            if (lat) {
                *lat = kWptDb[mid].lat_c / 100.0f;
            }
            if (lon) {
                *lon = kWptDb[mid].lon_c / 100.0f;
            }
            return true;
        }
        if (cmp < 0) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return false;
}

static bool net_nearest_waypoint(float lat, float lon, float max_nm, char *name_out, size_t name_len) {
    if (!name_out || name_len < 4) {
        return false;
    }
    if (!kWptDb || K_WPT_DB_COUNT == 0) {
        return false;
    }
    float best = max_nm * max_nm;
    const char *best_name = nullptr;
    const float cos_lat = cosf(lat * FR_DEG2RAD);
    for (size_t i = 0; i < K_WPT_DB_COUNT; i++) {
        float wlat = kWptDb[i].lat_c / 100.0f;
        float wlon = kWptDb[i].lon_c / 100.0f;
        float dlat = (wlat - lat) * 60.0f;
        float dlon = (wlon - lon) * 60.0f * cos_lat;
        float d2 = dlat * dlat + dlon * dlon;
        if (d2 < best) {
            best = d2;
            best_name = kWptDb[i].name;
        }
    }
    if (!best_name) {
        return false;
    }
    copy_trunc(name_out, name_len, best_name);
    return true;
}

static bool fa_resolve_named_fix(const char *name, float *lat, float *lon) {
    if (!name || !name[0] || !lat || !lon) {
        return false;
    }
    if (net_lookup_airport(name, lat, lon, nullptr)) {
        return true;
    }
    char icao[8] = {0};
    if (net_lookup_airport_iata(name, icao, sizeof(icao), lat, lon)) {
        return true;
    }
    // Common US ICAO when route uses 3-letter airport/VOR id without leading K.
    if (strlen(name) == 3) {
        char kcode[8];
        snprintf(kcode, sizeof(kcode), "K%s", name);
        if (net_lookup_airport(kcode, lat, lon, nullptr)) {
            return true;
        }
    }
    // FAA 5-letter fixes / intersections (GREKI, DUTES-style when in NASR).
    if (net_lookup_waypoint(name, lat, lon)) {
        return true;
    }
    return false;
}

static int fa_parse_waypoints_array(const char *html, float *lats, float *lons, int max_n) {
    if (!html || !lats || !lons || max_n <= 0) {
        return 0;
    }
    // FlightAware: "waypoints":[[lon,lat],[lon,lat],...]
    const char *p = strstr(html, "\"waypoints\":[[");
    if (!p) {
        return 0;
    }
    p += strlen("\"waypoints\":");
    if (p[0] != '[' || p[1] != '[') {
        return 0;
    }
    p += 2;  // first lon digit / sign (skip outer '[' and first point '[')
    int n = 0;
    for (;;) {
        while (*p == ' ' || *p == ',') {
            p++;
        }
        if (*p == ']' || !*p) {
            break;  // end of outer array
        }
        if (*p == '[') {
            p++;  // next [lon,lat]
        }
        char *end = nullptr;
        float lon = strtof(p, &end);
        if (!end || end == p) {
            break;
        }
        p = end;
        while (*p == ' ' || *p == ',') {
            p++;
        }
        float lat = strtof(p, &end);
        if (!end || end == p) {
            break;
        }
        p = end;
        while (*p && *p != ']') {
            p++;
        }
        if (*p == ']') {
            p++;  // end of this [lon,lat]
        }
        if (n < max_n && lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f) {
            lats[n] = lat;
            lons[n] = lon;
            n++;
        }
        if (n >= max_n) {
            break;
        }
    }
    return n;
}

static bool fa_extract_route_string(const char *html, char *out, size_t out_len) {
    if (!html || !out || out_len < 8) {
        return false;
    }
    out[0] = 0;
    // Prefer flightPlan.route inside trackpoll (filed IFR as shown in Flight Data).
    const char *fp = strstr(html, "\"flightPlan\":{");
    const char *p = nullptr;
    if (fp) {
        p = strstr(fp, "\"route\":\"");
        if (p && (size_t)(p - fp) > 800) {
            p = nullptr;
        }
    }
    if (!p) {
        return false;
    }
    p += strlen("\"route\":\"");
    size_t i = 0;
    while (p[i] && p[i] != '"' && i + 1 < out_len) {
        out[i] = p[i];
        i++;
    }
    out[i] = 0;
    return i > 0;  // empty string (common for some live legs) → false; use waypoints
}

static void fa_push_route_wpt(FlightInfo *fi, const char *name, float lat, float lon) {
    if (!fi || fi->route_wpt_n >= FR_MAX_ROUTE_WPTS) {
        return;
    }
    // Skip duplicates (same name or nearly same position).
    for (int i = 0; i < fi->route_wpt_n; i++) {
        if (name && name[0] && strcasecmp(fi->route_wpts[i].name, name) == 0) {
            return;
        }
        float dlat = fi->route_wpts[i].lat - lat;
        float dlon = fi->route_wpts[i].lon - lon;
        if (dlat * dlat + dlon * dlon < 1e-6f) {
            // Upgrade blank label if we now have a real name.
            if (name && name[0] && !fi->route_wpts[i].name[0]) {
                copy_trunc(fi->route_wpts[i].name, sizeof(fi->route_wpts[i].name), name);
                for (char *q = fi->route_wpts[i].name; *q; q++) {
                    *q = (char)toupper((unsigned char)*q);
                }
            }
            return;
        }
    }
    RouteWaypoint &w = fi->route_wpts[fi->route_wpt_n++];
    if (name && name[0]) {
        copy_trunc(w.name, sizeof(w.name), name);
        for (char *q = w.name; *q; q++) {
            *q = (char)toupper((unsigned char)*q);
        }
    } else {
        w.name[0] = 0;
    }
    w.lat = lat;
    w.lon = lon;
    w.ok = true;
}

static int fa_build_route_wpts(FlightInfo *fi, const char *route, const float *wlats, const float *wlons,
                               int wn) {
    if (!fi) {
        return 0;
    }
    fi->route_wpt_n = 0;
    memset(fi->route_wpts, 0, sizeof(fi->route_wpts));

    // Seed with origin / destination airports when known.
    if (fi->origin[0]) {
        float lat = 0, lon = 0;
        if (net_lookup_airport(fi->origin, &lat, &lon, nullptr) ||
            net_lookup_airport_iata(fi->origin, nullptr, 0, &lat, &lon)) {
            fa_push_route_wpt(fi, fi->origin, lat, lon);
        } else if (wn > 0) {
            fa_push_route_wpt(fi, fi->origin, wlats[0], wlons[0]);
        }
    } else if (wn > 0) {
        fa_push_route_wpt(fi, "DEP", wlats[0], wlons[0]);
    }

    int wp_cursor = (fi->route_wpt_n > 0 && wn > 1) ? 1 : 0;
    if (!route || !route[0]) {
        // No filed string — draw FA polyline; name via nearby airport or lat/lon fix style.
        const int last = wn - (wn > 1 ? 1 : 0);
        const int span = last - wp_cursor;
        const int step = (span > 20) ? (span / 16) : 1;
        for (int i = wp_cursor; i < last; i++) {
            if (fi->route_wpt_n >= FR_MAX_ROUTE_WPTS - 1) {
                break;
            }
            if (i != wp_cursor && i != last - 1 && step > 1 && ((i - wp_cursor) % step) != 0) {
                continue;
            }
            char lbl[12];
            // Prefer nearby airport, else nearest 5-letter FAA fix — never W# / raw lat-lon tags.
            if (fa_nearest_airport_name(lbl, sizeof(lbl), wlats[i], wlons[i], 8.0f)) {
                fa_push_route_wpt(fi, lbl, wlats[i], wlons[i]);
            } else if (net_nearest_waypoint(wlats[i], wlons[i], 25.0f, lbl, sizeof(lbl))) {
                fa_push_route_wpt(fi, lbl, wlats[i], wlons[i]);
            } else {
                // Keep geometry without a fake name; UI skips empty labels.
                fa_push_route_wpt(fi, "", wlats[i], wlons[i]);
            }
        }
    } else {
        char buf[384];
        copy_trunc(buf, sizeof(buf), route);
        char *save = nullptr;
        for (char *tok = strtok_r(buf, " ", &save); tok; tok = strtok_r(nullptr, " ", &save)) {
            if (!tok[0] || strcasecmp(tok, "DCT") == 0) {
                continue;
            }
            // Only strip /M082F290-style altitude — keep 4500N/07300W intact.
            char *slash = strchr(tok, '/');
            if (slash && fa_is_speed_alt_token(slash + 1)) {
                *slash = 0;
            }
            if (!tok[0] || fa_is_speed_alt_token(tok)) {
                continue;
            }

            float lat = 0, lon = 0;
            bool have = false;
            char label[14];
            copy_trunc(label, sizeof(label), tok);
            for (char *q = label; *q; q++) {
                *q = (char)toupper((unsigned char)*q);
            }

            // Lat/lon fixes first (23S163E or 4500N/07300W).
            if (decode_icao_ll_fix(label, &lat, &lon)) {
                have = true;
            } else if (fa_is_airway_or_proc(label)) {
                continue;
            } else if (fa_resolve_named_fix(label, &lat, &lon)) {
                have = true;
            } else if (wn > 0 && wp_cursor < wn) {
                // Unresolved named fix (GREKI, NOBAR, …): place on FA polyline, keep the name.
                lat = wlats[wp_cursor];
                lon = wlons[wp_cursor];
                wp_cursor++;
                have = true;
            }

            if (have) {
                if (wn > 0) {
                    float best_d = 1e30f;
                    int best_i = wp_cursor;
                    for (int i = wp_cursor; i < wn; i++) {
                        float dlat = wlats[i] - lat;
                        float dlon = wlons[i] - lon;
                        float d = dlat * dlat + dlon * dlon;
                        if (d < best_d) {
                            best_d = d;
                            best_i = i;
                        }
                        if (i > wp_cursor + 2 && d > best_d * 4.0f && best_d < 25.0f) {
                            break;
                        }
                    }
                    if (best_i + 1 > wp_cursor) {
                        wp_cursor = best_i + 1;
                    }
                }
                fa_push_route_wpt(fi, label, lat, lon);
            }
        }
    }

    if (fi->dest[0]) {
        float lat = 0, lon = 0;
        if (net_lookup_airport(fi->dest, &lat, &lon, nullptr) ||
            net_lookup_airport_iata(fi->dest, nullptr, 0, &lat, &lon)) {
            fa_push_route_wpt(fi, fi->dest, lat, lon);
        } else if (wn > 0) {
            fa_push_route_wpt(fi, fi->dest, wlats[wn - 1], wlons[wn - 1]);
        }
    } else if (wn > 0) {
        fa_push_route_wpt(fi, "ARR", wlats[wn - 1], wlons[wn - 1]);
    }

    return fi->route_wpt_n;
}

/** Parse FlightAware trackpoll `"track":[{"timestamp":…,"alt":…,"gs":…},…]`.
 *  alt is hundreds of feet; gs is knots. Coalesces to one sample per UTC minute
 *  (latest in each minute) so a full flight fits without truncating the end. */
static int fa_parse_track_array(const char *html, FlightTrackPt *out, int out_max) {
    if (!html || !out || out_max <= 0) {
        return 0;
    }
    const char *p = strstr(html, "\"track\":[");
    if (!p) {
        p = strstr(html, "\"track\": [");
    }
    if (!p) {
        return 0;
    }
    p = strchr(p, '[');
    if (!p) {
        return 0;
    }
    p++;
    int n = 0;
    while (*p) {
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',') {
            p++;
        }
        if (*p == ']') {
            break;
        }
        if (*p != '{') {
            break;
        }
        const char *obj = p;
        const char *end = strchr(obj + 1, '}');
        if (!end) {
            break;
        }
        // Bound field scans to this object.
        long ts = 0;
        int alt100 = 0;
        int gs = 0;
        bool have_ts = false, have_alt = false, have_gs = false;
        for (const char *q = obj; q < end; q++) {
            if (!have_ts && q + 13 < end && strncmp(q, "\"timestamp\":", 12) == 0) {
                ts = strtol(q + 12, nullptr, 10);
                have_ts = ts > 1700000000L;
            } else if (!have_alt && q + 7 < end && strncmp(q, "\"alt\":", 6) == 0) {
                alt100 = (int)strtol(q + 6, nullptr, 10);
                have_alt = true;
            } else if (!have_gs && q + 6 < end && strncmp(q, "\"gs\":", 5) == 0) {
                gs = (int)strtol(q + 5, nullptr, 10);
                have_gs = true;
            }
            if (have_ts && have_alt && have_gs) {
                break;
            }
        }
        if (have_ts && (have_alt || have_gs)) {
            if (alt100 < 0) {
                alt100 = 0;
            }
            if (gs < 0) {
                gs = 0;
            }
            const uint32_t t_unix = (uint32_t)ts;
            const uint32_t minute = t_unix / 60u;
            if (n > 0 && (out[n - 1].t_unix / 60u) == minute) {
                // Same UTC minute — keep the latest sample.
                out[n - 1].t_unix = t_unix;
                out[n - 1].alt_ft = (float)alt100 * 100.0f;
                out[n - 1].gs_kts = (float)gs;
            } else if (n < out_max) {
                out[n].t_unix = t_unix;
                out[n].alt_ft = (float)alt100 * 100.0f;
                out[n].gs_kts = (float)gs;
                n++;
            } else {
                // Buffer full: drop oldest minute, append (sliding window).
                memmove(out, out + 1, sizeof(FlightTrackPt) * (size_t)(out_max - 1));
                out[out_max - 1].t_unix = t_unix;
                out[out_max - 1].alt_ft = (float)alt100 * 100.0f;
                out[out_max - 1].gs_kts = (float)gs;
            }
        }
        p = end + 1;
    }
    return n;
}

bool net_fetch_flight_plan(const char *callsign, FlightInfo *fi, FlightTrackPt *track_out, int track_max,
                           int *track_n) {
    if (track_n) {
        *track_n = 0;
    }
    if (!fi || !callsign || !callsign[0]) {
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

    status_msg("FlightAware route...");
    char enc[32];
    url_encode(cs, enc, sizeof(enc));
    char url[160];
    snprintf(url, sizeof(url), "https://www.flightaware.com/live/flight/%s", enc);
    // ~500 KB chunked HTML; 60s idle + partial-body OK in read_http_body.
    if (!https_get(url, nullptr, kUaBrowser, 10000, 60000, "text/html,application/xhtml+xml,*/*", true)) {
        Serial.printf("FlightAware fetch fail: %s\n", g_last_err);
        return false;
    }
    size_t blen = 0;
    const char *html = http_body_buf(&blen);
    if (!html || blen < 800) {
        Serial.println("FlightAware: empty body");
        return false;
    }
    Serial.printf("FlightAware body %u KB\n", (unsigned)(blen / 1024));

    // Historical altitude / speed for the follow chart (same page as route).
    if (track_out && track_max > 0) {
        const int tn = fa_parse_track_array(html, track_out, track_max);
        if (track_n) {
            *track_n = tn;
        }
        Serial.printf("FlightAware track pts=%d\n", tn);
    }

    char route[384] = {0};
    fa_extract_route_string(html, route, sizeof(route));

    float wlats[96];
    float wlons[96];
    int wn = fa_parse_waypoints_array(html, wlats, wlons, 96);
    Serial.printf("FlightAware route_len=%u waypoints=%d\n", (unsigned)strlen(route), wn);

    // Live legs often have empty flightPlan.route — try the operated history URL for the filed string.
    if (!route[0]) {
        const char *op = strstr(html, "\"operated\":\"/live/flight/");
        if (op) {
            op += strlen("\"operated\":\"");
            char path[160];
            size_t pi = 0;
            while (op[pi] && op[pi] != '"' && pi + 1 < sizeof(path)) {
                path[pi] = op[pi];
                pi++;
            }
            path[pi] = 0;
            if (path[0] == '/') {
                char hist_url[192];
                snprintf(hist_url, sizeof(hist_url), "https://www.flightaware.com%s", path);
                Serial.printf("FlightAware history %s\n", path);
                status_msg("FlightAware filed route...");
                if (https_get(hist_url, nullptr, kUaBrowser, 10000, 90000,
                              "text/html,application/xhtml+xml,*/*", true)) {
                    size_t hlen = 0;
                    const char *hhtml = http_body_buf(&hlen);
                    if (hhtml && hlen > 800) {
                        fa_extract_route_string(hhtml, route, sizeof(route));
                        if (wn <= 0) {
                            wn = fa_parse_waypoints_array(hhtml, wlats, wlons, 96);
                        }
                        // Prefer history-page track when live page had few/no samples.
                        if (track_out && track_max > 0 && (!track_n || *track_n < 8)) {
                            const int tn = fa_parse_track_array(hhtml, track_out, track_max);
                            if (track_n) {
                                *track_n = tn;
                            }
                            Serial.printf("FlightAware history track pts=%d\n", tn);
                        }
                        Serial.printf("FlightAware history route_len=%u waypoints=%d\n",
                                      (unsigned)strlen(route), wn);
                    }
                }
            }
        }
    }

    const int have_track = (track_n && *track_n > 0) ? *track_n : 0;
    if (!route[0] && wn <= 0 && have_track <= 0) {
        Serial.println("FlightAware: no flightPlan.route / waypoints / track");
        return false;
    }
    if (route[0]) {
        Serial.printf("FlightAware Flight Data route: %s\n", route);
    }

    int n = 0;
    if (route[0] || wn > 0) {
        n = fa_build_route_wpts(fi, route[0] ? route : nullptr, wlats, wlons, wn);
    }
    if (n > 0) {
        fi->route_ok = true;
        Serial.printf("FlightAware decoded %d waypoints\n", n);
        for (int i = 0; i < n && i < 12; i++) {
            Serial.printf("  %s %.2f %.2f\n", fi->route_wpts[i].name, fi->route_wpts[i].lat,
                          fi->route_wpts[i].lon);
        }
    }
    return n > 0 || have_track > 0;
}

static void fa_html_unescape_nbsp(char *s) {
    if (!s) {
        return;
    }
    char *d = s;
    for (char *p = s; *p;) {
        if (p[0] == '&' && strncmp(p, "&nbsp;", 6) == 0) {
            *d++ = ' ';
            p += 6;
        } else {
            *d++ = *p++;
        }
    }
    *d = 0;
}

static void fa_strip_tags_to(const char *in, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    size_t o = 0;
    bool in_tag = false;
    for (const char *p = in ? in : ""; *p && o + 1 < out_len; p++) {
        if (*p == '<') {
            in_tag = true;
            continue;
        }
        if (*p == '>') {
            in_tag = false;
            continue;
        }
        if (in_tag) {
            continue;
        }
        out[o++] = *p;
    }
    out[o] = 0;
    fa_html_unescape_nbsp(out);
    char *w = out;
    bool sp = false;
    for (char *p = out; *p; p++) {
        if (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
            if (!sp && w != out) {
                *w++ = ' ';
                sp = true;
            }
        } else {
            *w++ = *p;
            sp = false;
        }
    }
    *w = 0;
    trim_inplace(out);
}

static bool fa_parse_clock_hm(const char *s, int *hh_out, int *mm_out) {
    if (!s || !hh_out || !mm_out) {
        return false;
    }
    int h = 0, m = 0;
    char ap = 0;
    if (sscanf(s, "%d:%d%c", &h, &m, &ap) < 2) {
        return false;
    }
    if (h < 0 || h > 23 || m < 0 || m > 59) {
        return false;
    }
    if (ap == 'p' || ap == 'P') {
        if (h < 12) {
            h += 12;
        }
    } else if (ap == 'a' || ap == 'A') {
        if (h == 12) {
            h = 0;
        }
    }
    *hh_out = h;
    *mm_out = m;
    return true;
}

static bool fa_parse_hist_dep_epoch(const char *href, uint32_t *epoch_out) {
    if (!href || !epoch_out) {
        return false;
    }
    const char *p = strstr(href, "/history/");
    if (!p) {
        return false;
    }
    p += 9;
    int y = 0, mo = 0, d = 0, hh = 0, mm = 0;
    if (sscanf(p, "%4d%2d%2d/%2d%2dZ", &y, &mo, &d, &hh, &mm) != 5) {
        return false;
    }
    if (y < 2020 || mo < 1 || mo > 12 || d < 1 || d > 31) {
        return false;
    }
    struct tm t {};
    t.tm_year = y - 1900;
    t.tm_mon = mo - 1;
    t.tm_mday = d;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = 0;
    // Device TZ is UTC (configTime(0,0,...)).
    time_t e = mktime(&t);
    if (e < 0) {
        return false;
    }
    *epoch_out = (uint32_t)e;
    return true;
}

static void fa_board_fill_progress(AirportBoardRow *row, bool prefer_complete) {
    if (!row || !row->valid) {
        return;
    }
    const time_t now = time(nullptr);
    if (now < 1700000000) {
        row->progress_pct = prefer_complete ? 100 : 0;
        return;
    }
    if (!row->dep_epoch) {
        row->progress_pct = prefer_complete ? 100 : 0;
        return;
    }
    uint32_t arr = row->arr_epoch;
    if (!arr || arr <= row->dep_epoch) {
        arr = row->dep_epoch + 2u * 3600u;
        row->arr_epoch = arr;
    }
    if ((uint32_t)now <= row->dep_epoch) {
        row->progress_pct = 0;
        return;
    }
    if ((uint32_t)now >= arr) {
        row->progress_pct = 100;
        return;
    }
    const uint32_t span = arr - row->dep_epoch;
    const uint32_t el = (uint32_t)now - row->dep_epoch;
    row->progress_pct = (uint8_t)((el * 100u) / span);
    if (row->progress_pct > 99) {
        row->progress_pct = 99;
    }
}

static int fa_parse_airport_board_section(const char *html, const char *board_id, AirportBoardRow *out,
                                          int out_max, bool landed_bias) {
    if (!html || !board_id || !out || out_max <= 0) {
        return 0;
    }
    char needle[48];
    snprintf(needle, sizeof(needle), "id=\"%s\"", board_id);
    const char *sec = strstr(html, needle);
    if (!sec) {
        return 0;
    }
    sec += strlen(needle);

    const char *sec_end = nullptr;
    static const char *kNext[] = {"id=\"arrivals-board\"",   "id=\"departures-board\"",
                                  "id=\"enroute-board\"",    "id=\"scheduled-board\"",
                                  "id=\"arrivals-departures", nullptr};
    for (int i = 0; kNext[i]; i++) {
        if (strstr(kNext[i], board_id)) {
            continue;
        }
        const char *nx = strstr(sec, kNext[i]);
        if (nx && (!sec_end || nx < sec_end)) {
            sec_end = nx;
        }
    }
    if (!sec_end) {
        sec_end = sec + strlen(sec);
    }

    int n = 0;
    const char *p = sec;
    while (n < out_max && p < sec_end) {
        // Yield so the LVGL/display task can run during large FA HTML scans.
        if ((n & 7) == 7) {
            vTaskDelay(1);
        }
        const char *tr = strstr(p, "id=\"Row_");
        if (!tr || tr >= sec_end) {
            break;
        }
        const char *tr_end = strstr(tr, "</tr>");
        if (!tr_end || tr_end >= sec_end) {
            break;
        }
        const size_t tlen = (size_t)(tr_end - tr);
        // Static — board HTML rows are large; keep off net-task stack (was rebooting).
        static char rowbuf[1600];
        if (tlen >= sizeof(rowbuf)) {
            p = tr_end + 5;
            continue;
        }
        memcpy(rowbuf, tr, tlen);
        rowbuf[tlen] = 0;

        AirportBoardRow row{};
        const char *fl = strstr(rowbuf, "/live/flight/");
        if (fl) {
            const char *href0 = fl;
            fl += strlen("/live/flight/");
            size_t fi = 0;
            while (fl[fi] && fl[fi] != '/' && fl[fi] != '"' && fi + 1 < sizeof(row.flight)) {
                row.flight[fi] = (char)toupper((unsigned char)fl[fi]);
                fi++;
            }
            row.flight[fi] = 0;
            char hist[120];
            size_t hi = 0;
            const char *hp = href0;
            while (*hp && *hp != '"' && hi + 1 < sizeof(hist)) {
                hist[hi++] = *hp++;
            }
            hist[hi] = 0;
            fa_parse_hist_dep_epoch(hist, &row.dep_epoch);
            const char *slash = strrchr(hist, '/');
            if (slash && slash > hist) {
                const char *slash2 = slash - 1;
                while (slash2 > hist && *slash2 != '/') {
                    slash2--;
                }
                if (*slash2 == '/') {
                    char a[8] = {0}, b[8] = {0};
                    if (sscanf(slash2, "/%7[^/]/%7s", a, b) == 2) {
                        // Departures + scheduled: other = destination (b).
                        // Arrivals / enroute: other = origin (a).
                        // (strstr "depart" alone misses scheduled-board — Next Hour
                        // preferred those rows and showed the dep city in TO.)
                        const bool dep_side = strstr(board_id, "depart") != nullptr ||
                                              strstr(board_id, "scheduled") != nullptr;
                        copy_trunc(row.other_apt, sizeof(row.other_apt), dep_side ? b : a);
                    }
                }
            }
        }
        const char *tp = strstr(rowbuf, "/live/aircrafttype/");
        if (tp) {
            tp += strlen("/live/aircrafttype/");
            size_t ti = 0;
            while (tp[ti] && tp[ti] != '"' && tp[ti] != '<' && ti + 1 < sizeof(row.type)) {
                row.type[ti] = (char)toupper((unsigned char)tp[ti]);
                ti++;
            }
            row.type[ti] = 0;
        }
        const char *hint = strstr(rowbuf, "class=\"hint\"");
        if (hint) {
            const char *title = strstr(hint, "title=\"");
            if (title) {
                title += 7;
                char raw[80];
                size_t ri = 0;
                while (title[ri] && title[ri] != '"' && ri + 1 < sizeof(raw)) {
                    raw[ri] = title[ri];
                    ri++;
                }
                raw[ri] = 0;
                char *dash = strstr(raw, " - ");
                if (dash) {
                    *dash = 0;
                }
                copy_trunc(row.other_name, sizeof(row.other_name), raw);
            }
        }
        if (!row.other_apt[0]) {
            // Prefer the last /live/airport/ in the row (usually the opposite end).
            const char *ap = nullptr;
            for (const char *q = rowbuf; (q = strstr(q, "/live/airport/")) != nullptr; q += 14) {
                ap = q;
            }
            if (ap) {
                ap += strlen("/live/airport/");
                size_t ai = 0;
                while (ap[ai] && ap[ai] != '"' && ai + 1 < sizeof(row.other_apt)) {
                    row.other_apt[ai] = (char)toupper((unsigned char)ap[ai]);
                    ai++;
                }
                row.other_apt[ai] = 0;
            }
        }

        char dep_disp[24] = {0}, arr_disp[24] = {0};
        int dep_hh = -1, dep_mm = -1, arr_hh = -1, arr_mm = -1;
        int td_i = 0;
        const char *td = rowbuf;
        while ((td = strstr(td, "<td")) != nullptr) {
            const char *tdc = strchr(td, '>');
            if (!tdc) {
                break;
            }
            tdc++;
            const char *tde = strstr(tdc, "</td>");
            if (!tde) {
                break;
            }
            char cell[96];
            size_t clen = (size_t)(tde - tdc);
            if (clen >= sizeof(cell)) {
                clen = sizeof(cell) - 1;
            }
            memcpy(cell, tdc, clen);
            cell[clen] = 0;
            char plain[96];
            fa_strip_tags_to(cell, plain, sizeof(plain));
            if (td_i == 3 && plain[0]) {
                copy_trunc(dep_disp, sizeof(dep_disp), plain);
                fa_parse_clock_hm(plain, &dep_hh, &dep_mm);
            } else if (td_i == 5 && plain[0]) {
                copy_trunc(arr_disp, sizeof(arr_disp), plain);
                fa_parse_clock_hm(plain, &arr_hh, &arr_mm);
            }
            td_i++;
            td = tde + 5;
        }
        copy_trunc(row.dep_time, sizeof(row.dep_time), dep_disp);
        copy_trunc(row.arr_time, sizeof(row.arr_time), arr_disp);

        if (row.dep_epoch && dep_hh >= 0 && arr_hh >= 0) {
            int dep_m = dep_hh * 60 + dep_mm;
            int arr_m = arr_hh * 60 + arr_mm;
            int dur = arr_m - dep_m;
            if (dur < 15) {
                dur += 24 * 60;
            }
            if (dur > 15 && dur < 20 * 60) {
                row.arr_epoch = row.dep_epoch + (uint32_t)dur * 60u;
            }
        }

        if (row.flight[0]) {
            row.valid = true;
            fa_board_fill_progress(&row, landed_bias);
            out[n++] = row;
        }
        p = tr_end + 5;
    }
    return n;
}

bool net_fetch_airport_board(const char *icao, AirportBoardRow *arr, int arr_max, int *arr_n,
                             AirportBoardRow *dep, int dep_max, int *dep_n, char *label_out,
                             size_t label_len) {
    // Reopen within ~3 min reuses last parse — second FA TLS right after a ~140KB
    // download often fails with "TLS generic error" on this stack.
    static char s_cache_icao[8] = {0};
    static char s_cache_label[72] = {0};
    static AirportBoardRow s_cache_arr[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_cache_dep[FR_MAX_BOARD_ROWS];
    static int s_cache_an = 0;
    static int s_cache_dn = 0;
    static uint32_t s_cache_ms = 0;
    static constexpr uint32_t kBoardCacheMs = 180000u;

    if (arr_n) {
        *arr_n = 0;
    }
    if (dep_n) {
        *dep_n = 0;
    }
    if (label_out && label_len) {
        label_out[0] = 0;
    }
    if (!icao || !icao[0] || !arr || !dep || arr_max <= 0 || dep_max <= 0) {
        set_err("Bad airport board args");
        return false;
    }
    char code[8];
    copy_trunc(code, sizeof(code), icao);
    for (char *q = code; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    if (strlen(code) == 3) {
        char icao4[8] = {0};
        if (net_lookup_airport_iata(code, icao4, sizeof(icao4), nullptr, nullptr) && icao4[0]) {
            copy_trunc(code, sizeof(code), icao4);
        }
    }
    if (label_out && label_len) {
        snprintf(label_out, label_len, "%s", code);
    }

    if (s_cache_icao[0] && strcasecmp(s_cache_icao, code) == 0 && s_cache_ms &&
        (millis() - s_cache_ms) < kBoardCacheMs && (s_cache_an > 0 || s_cache_dn > 0)) {
        const int na = (s_cache_an < arr_max) ? s_cache_an : arr_max;
        const int nd = (s_cache_dn < dep_max) ? s_cache_dn : dep_max;
        for (int i = 0; i < na; i++) {
            arr[i] = s_cache_arr[i];
        }
        for (int i = 0; i < nd; i++) {
            dep[i] = s_cache_dep[i];
        }
        if (arr_n) {
            *arr_n = na;
        }
        if (dep_n) {
            *dep_n = nd;
        }
        if (label_out && label_len && s_cache_label[0]) {
            snprintf(label_out, label_len, "%s", s_cache_label);
        }
        Serial.printf("FA board cache hit %s (%d arr / %d dep)\n", code, na, nd);
        status_msg("Airport board (cached)");
        return true;
    }

    status_msg("Airport board...");
    char enc[16];
    url_encode(code, enc, sizeof(enc));
    char url[160];
    snprintf(url, sizeof(url), "https://www.flightaware.com/live/airport/%s", enc);
    // early_ok is stall-only (never mid-stream abort) — keeps partial HTML if boards are in.
    auto fa_board_body_ok = [](const char *buf, size_t len) -> bool {
        if (!buf || len < 4096) {
            return false;
        }
        return strstr(buf, "id=\"enroute-board\"") != nullptr ||
               strstr(buf, "id=\"arrivals-board\"") != nullptr ||
               strstr(buf, "id=\"departures-board\"") != nullptr ||
               strstr(buf, "id=\"scheduled-board\"") != nullptr;
    };
    bool got = false;
    for (int attempt = 1; attempt <= 2; attempt++) {
        if (s_tls_last_close_ms) {
            const uint32_t since = millis() - s_tls_last_close_ms;
            if (since < 2500u) {
                delay(2500u - since);
            }
        }
        tls_close();
        status_msg(attempt == 1 ? "Airport board..." : "Airport board retry...");
        got = https_get(url, nullptr, kUaBrowser, 10000, 90000, "text/html,application/xhtml+xml,*/*",
                        true, fa_board_body_ok);
        if (got) {
            break;
        }
        Serial.printf("FA board try %d fail: %s\n", attempt, g_last_err);
        if (attempt == 1) {
            delay(2500);
            yield();
        }
    }
    if (!got) {
        return false;
    }
    size_t blen = 0;
    const char *html = http_body_buf(&blen);
    if (!html || blen < 800) {
        set_err("Airport board empty");
        return false;
    }
    Serial.printf("FA airport body %u KB\n", (unsigned)(blen / 1024));
    tls_close();

    // Static / PSRAM scratch — keep large row buffers off internal DRAM + stack.
    static AirportBoardRow s_board_tmp[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_board_sched[FR_MAX_BOARD_ROWS];
    // FA departures-board lists recent past first; upcoming often sit past row 20.
    static constexpr int kDepParseMax = 40;
    static AirportBoardRow *s_dep_raw = nullptr;
    if (!s_dep_raw) {
        s_dep_raw = (AirportBoardRow *)heap_caps_malloc(sizeof(AirportBoardRow) * (size_t)kDepParseMax,
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_dep_raw) {
            s_dep_raw = (AirportBoardRow *)heap_caps_malloc(sizeof(AirportBoardRow) * (size_t)kDepParseMax,
                                                            MALLOC_CAP_8BIT);
        }
    }
    if (!s_dep_raw) {
        set_err("Board parse OOM");
        return false;
    }
    memset(s_dep_raw, 0, sizeof(AirportBoardRow) * (size_t)kDepParseMax);

    int na = fa_parse_airport_board_section(html, "enroute-board", arr, arr_max, false);
    if (na < arr_max) {
        const int n2 = fa_parse_airport_board_section(html, "arrivals-board", s_board_tmp, FR_MAX_BOARD_ROWS, true);
        for (int i = 0; i < n2 && na < arr_max; i++) {
            bool dup = false;
            for (int j = 0; j < na; j++) {
                if (strcasecmp(arr[j].flight, s_board_tmp[i].flight) == 0) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                arr[na++] = s_board_tmp[i];
            }
        }
    }

    const int n_raw = fa_parse_airport_board_section(html, "departures-board", s_dep_raw, kDepParseMax, false);
    const int ns = fa_parse_airport_board_section(html, "scheduled-board", s_board_sched, FR_MAX_BOARD_ROWS, false);
    const time_t now_t = time(nullptr);
    const uint32_t now_u = (now_t > 1700000000) ? (uint32_t)now_t : 0;
    auto dep_is_upcoming = [&](const AirportBoardRow &r) -> bool {
        if (now_u && r.dep_epoch) {
            // Keep not-yet-gone and up to 10 min past (boarding / late push).
            return r.dep_epoch + 600u >= now_u;
        }
        return r.progress_pct < 100;
    };
    auto dep_already = [&](const AirportBoardRow *list, int n, const char *flight) -> bool {
        for (int j = 0; j < n; j++) {
            if (strcasecmp(list[j].flight, flight) == 0) {
                return true;
            }
        }
        return false;
    };
    int nd = 0;
    // Prefer upcoming (scheduled + still-listed deps) so Next Hour is not empty.
    for (int i = 0; i < ns && nd < dep_max; i++) {
        if (!s_board_sched[i].flight[0] || !dep_is_upcoming(s_board_sched[i])) {
            continue;
        }
        if (!dep_already(dep, nd, s_board_sched[i].flight)) {
            dep[nd++] = s_board_sched[i];
        }
    }
    for (int i = 0; i < n_raw && nd < dep_max; i++) {
        if (!s_dep_raw[i].flight[0] || !dep_is_upcoming(s_dep_raw[i])) {
            continue;
        }
        if (!dep_already(dep, nd, s_dep_raw[i].flight)) {
            dep[nd++] = s_dep_raw[i];
        }
    }
    // Fill remaining slots with recent past departures (board default look).
    for (int i = 0; i < n_raw && nd < dep_max; i++) {
        if (!s_dep_raw[i].flight[0] || dep_is_upcoming(s_dep_raw[i])) {
            continue;
        }
        if (!dep_already(dep, nd, s_dep_raw[i].flight)) {
            dep[nd++] = s_dep_raw[i];
        }
    }
    if (arr_n) {
        *arr_n = na;
    }
    if (dep_n) {
        *dep_n = nd;
    }

    if (label_out && label_len) {
        const char *og = strstr(html, "og:description\" content=\"");
        if (!og) {
            og = strstr(html, "og:description\" content='");
        }
        if (og) {
            const char *c = strstr(og, "content=\"");
            if (c) {
                c += 9;
                char desc[120];
                size_t di = 0;
                while (c[di] && c[di] != '"' && di + 1 < sizeof(desc)) {
                    desc[di] = c[di];
                    di++;
                }
                desc[di] = 0;
                char *paren = strchr(desc, '(');
                if (paren) {
                    *paren = 0;
                }
                trim_inplace(desc);
                if (desc[0]) {
                    snprintf(label_out, label_len, "%s (%s)", desc, code);
                }
            }
        }
    }

    if (na > 0 || nd > 0) {
        copy_trunc(s_cache_icao, sizeof(s_cache_icao), code);
        s_cache_an = (na < FR_MAX_BOARD_ROWS) ? na : FR_MAX_BOARD_ROWS;
        s_cache_dn = (nd < FR_MAX_BOARD_ROWS) ? nd : FR_MAX_BOARD_ROWS;
        for (int i = 0; i < s_cache_an; i++) {
            s_cache_arr[i] = arr[i];
        }
        for (int i = 0; i < s_cache_dn; i++) {
            s_cache_dep[i] = dep[i];
        }
        if (label_out && label_out[0]) {
            copy_trunc(s_cache_label, sizeof(s_cache_label), label_out);
        } else {
            copy_trunc(s_cache_label, sizeof(s_cache_label), code);
        }
        s_cache_ms = millis();
    }
    return na > 0 || nd > 0;
}

static bool apt_codes_match(const char *a, const char *b) {
    if (!a || !b || !a[0] || !b[0]) {
        return false;
    }
    if (strcasecmp(a, b) == 0) {
        return true;
    }
    // KATL ↔ ATL
    if (strlen(a) == 4 && (a[0] == 'K' || a[0] == 'k') && strcasecmp(a + 1, b) == 0) {
        return true;
    }
    if (strlen(b) == 4 && (b[0] == 'K' || b[0] == 'k') && strcasecmp(b + 1, a) == 0) {
        return true;
    }
    return false;
}

static int route_push_unique(AirportBoardRow *out, int n, int out_max, const AirportBoardRow &row) {
    if (!out || n < 0 || n >= out_max || !row.flight[0]) {
        return n;
    }
    for (int i = 0; i < n; i++) {
        if (strcasecmp(out[i].flight, row.flight) == 0) {
            if (!out[i].dep_time[0] && row.dep_time[0]) {
                copy_trunc(out[i].dep_time, sizeof(out[i].dep_time), row.dep_time);
            }
            if (!out[i].arr_time[0] && row.arr_time[0]) {
                copy_trunc(out[i].arr_time, sizeof(out[i].arr_time), row.arr_time);
            }
            if (!out[i].dep_est[0] && row.dep_est[0]) {
                copy_trunc(out[i].dep_est, sizeof(out[i].dep_est), row.dep_est);
            }
            if (!out[i].arr_est[0] && row.arr_est[0]) {
                copy_trunc(out[i].arr_est, sizeof(out[i].arr_est), row.arr_est);
            }
            if (!out[i].type[0] && row.type[0]) {
                copy_trunc(out[i].type, sizeof(out[i].type), row.type);
            }
            if (!out[i].other_name[0] && row.other_name[0]) {
                copy_trunc(out[i].other_name, sizeof(out[i].other_name), row.other_name);
            }
            if (!out[i].other_apt[0] && row.other_apt[0]) {
                copy_trunc(out[i].other_apt, sizeof(out[i].other_apt), row.other_apt);
            }
            if (row.progress_pct > out[i].progress_pct) {
                out[i].progress_pct = row.progress_pct;
            }
            return n;
        }
    }
    out[n] = row;
    return n + 1;
}

/** "2026-10-18 19:03:00" -> "7:03p" (12h) for compact board cells. */
static void fa_iso_local_to_hm12(const char *iso, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (!iso || !iso[0]) {
        return;
    }
    int hh = -1, mm = -1, ss = 0;
    const char *sp = strchr(iso, ' ');
    if (sp) {
        if (sscanf(sp + 1, "%d:%d:%d", &hh, &mm, &ss) < 2) {
            return;
        }
    } else if (sscanf(iso, "%d:%d", &hh, &mm) < 2) {
        return;
    }
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59) {
        return;
    }
    const char *ampm = (hh >= 12) ? "p" : "a";
    int h12 = hh % 12;
    if (h12 == 0) {
        h12 = 12;
    }
    snprintf(out, out_len, "%d:%02d%s", h12, mm, ampm);
}

static bool fa_json_str_field(const char *obj, const char *obj_end, const char *key, char *out, size_t out_len) {
    if (!obj || !key || !out || out_len == 0) {
        return false;
    }
    out[0] = 0;
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":\"", key);
    const char *p = obj;
    while (p && p < obj_end) {
        const char *hit = strstr(p, needle);
        if (!hit || hit >= obj_end) {
            return false;
        }
        // Ensure we matched at a key boundary (start or after ,/{).
        if (hit > obj && hit[-1] != '{' && hit[-1] != ',' && hit[-1] != ' ') {
            p = hit + 1;
            continue;
        }
        hit += strlen(needle);
        size_t i = 0;
        while (hit[i] && hit[i] != '"' && hit + i < obj_end && i + 1 < out_len) {
            out[i] = hit[i];
            i++;
        }
        out[i] = 0;
        return i > 0;
    }
    return false;
}

/** Parse FA.findflight.flights JSON + matching tr url= idents for sch/est times. */
static int fa_parse_findflight_rows(const char *html, AirportBoardRow *out, int out_max, const char *arr_icao) {
    if (!html || !out || out_max <= 0) {
        return 0;
    }
    // Ordered idents from results table rows.
    char idents[FR_MAX_BOARD_ROWS][12];
    int ident_n = 0;
    const char *ip = html;
    while (ident_n < FR_MAX_BOARD_ROWS && (ip = strstr(ip, "url='/live/flight/id/")) != nullptr) {
        ip += strlen("url='/live/flight/id/");
        size_t ii = 0;
        while (ip[ii] && ip[ii] != '-' && ip[ii] != '\'' && ii + 1 < sizeof(idents[0])) {
            idents[ident_n][ii] = (char)toupper((unsigned char)ip[ii]);
            ii++;
        }
        idents[ident_n][ii] = 0;
        if (idents[ident_n][0] && ii >= 3) {
            ident_n++;
        }
        ip += ii ? ii : 1;
    }
    if (ident_n == 0) {
        return 0;
    }

    const char *flights = strstr(html, "FA.findflight.flights");
    if (!flights) {
        // Fallback: idents only (no times).
        int n = 0;
        for (int i = 0; i < ident_n && n < out_max; i++) {
            AirportBoardRow row{};
            copy_trunc(row.flight, sizeof(row.flight), idents[i]);
            if (arr_icao) {
                copy_trunc(row.other_apt, sizeof(row.other_apt), arr_icao);
            }
            row.valid = true;
            n = route_push_unique(out, n, out_max, row);
        }
        return n;
    }
    flights = strchr(flights, '[');
    if (!flights) {
        return 0;
    }

    int n = 0;
    int idx = 0;
    const char *p = flights;
    while (n < out_max && idx < ident_n && (p = strstr(p, "{\"act_runway")) != nullptr) {
        const char *end = strchr(p, '}');
        if (!end) {
            break;
        }
        end++;
        AirportBoardRow row{};
        copy_trunc(row.flight, sizeof(row.flight), idents[idx]);
        if (arr_icao) {
            copy_trunc(row.other_apt, sizeof(row.other_apt), arr_icao);
        }
        char sch_out[32] = {0}, sch_in[32] = {0};
        char est_out[32] = {0}, est_in[32] = {0};
        char status[24] = {0}, atype[40] = {0}, city[48] = {0};
        fa_json_str_field(p, end, "sch_block_out", sch_out, sizeof(sch_out));
        fa_json_str_field(p, end, "sch_block_in", sch_in, sizeof(sch_in));
        fa_json_str_field(p, end, "estimateddeparturetime", est_out, sizeof(est_out));
        if (!est_out[0]) {
            fa_json_str_field(p, end, "est_block_out", est_out, sizeof(est_out));
        }
        fa_json_str_field(p, end, "estimatedarrivaltime", est_in, sizeof(est_in));
        if (!est_in[0]) {
            fa_json_str_field(p, end, "est_block_in", est_in, sizeof(est_in));
        }
        fa_json_str_field(p, end, "status", status, sizeof(status));
        fa_json_str_field(p, end, "aircrafttype_friendly", atype, sizeof(atype));
        fa_json_str_field(p, end, "destination_city", city, sizeof(city));

        fa_iso_local_to_hm12(sch_out, row.dep_time, sizeof(row.dep_time));
        fa_iso_local_to_hm12(sch_in, row.arr_time, sizeof(row.arr_time));
        fa_iso_local_to_hm12(est_out, row.dep_est, sizeof(row.dep_est));
        fa_iso_local_to_hm12(est_in, row.arr_est, sizeof(row.arr_est));
        // If scheduled missing, fall back estimated into primary columns.
        if (!row.dep_time[0] && row.dep_est[0]) {
            copy_trunc(row.dep_time, sizeof(row.dep_time), row.dep_est);
        }
        if (!row.arr_time[0] && row.arr_est[0]) {
            copy_trunc(row.arr_time, sizeof(row.arr_time), row.arr_est);
        }
        if (atype[0]) {
            // Keep a short type token from the friendly name (last space-separated word).
            const char *tok = strrchr(atype, ' ');
            tok = tok ? tok + 1 : atype;
            copy_trunc(row.type, sizeof(row.type), tok);
        }
        if (city[0]) {
            char *comma = strchr(city, ',');
            if (comma) {
                *comma = 0;
            }
            copy_trunc(row.other_name, sizeof(row.other_name), city);
        }
        if (strcasecmp(status, "En Route") == 0 || strcasecmp(status, "Airborne") == 0) {
            row.progress_pct = 50;
        } else if (strcasecmp(status, "Arrived") == 0 || strcasecmp(status, "Landed") == 0) {
            row.progress_pct = 100;
        } else {
            row.progress_pct = 0;  // Scheduled / departed gate / etc.
        }
        row.valid = true;
        n = route_push_unique(out, n, out_max, row);
        idx++;
        p = end;
    }
    // Any leftover idents without JSON objects.
    while (n < out_max && idx < ident_n) {
        AirportBoardRow row{};
        copy_trunc(row.flight, sizeof(row.flight), idents[idx++]);
        if (arr_icao) {
            copy_trunc(row.other_apt, sizeof(row.other_apt), arr_icao);
        }
        row.valid = true;
        n = route_push_unique(out, n, out_max, row);
    }
    return n;
}

bool net_fetch_route_flights(const char *dep_icao, const char *arr_icao, AirportBoardRow *out, int out_max,
                             int *out_n, char *label_out, size_t label_len) {
    if (out_n) {
        *out_n = 0;
    }
    if (label_out && label_len) {
        label_out[0] = 0;
    }
    if (!dep_icao || !arr_icao || !dep_icao[0] || !arr_icao[0] || !out || out_max <= 0) {
        set_err("Bad route search args");
        return false;
    }
    char dep[8], arr[8];
    copy_trunc(dep, sizeof(dep), dep_icao);
    copy_trunc(arr, sizeof(arr), arr_icao);
    for (char *q = dep; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    for (char *q = arr; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    if (label_out && label_len) {
        snprintf(label_out, label_len, "%s to %s", dep, arr);
    }

    // File-scope scratch buffers (net task is single-threaded for these fetches).
    static AirportBoardRow s_route_board_arr[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_route_board_dep[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_route_ff[FR_MAX_BOARD_ROWS];

    int n = 0;
    int an = 0, dn = 0;
    char lab[48] = {0};

    status_msg("Route: departures...");
    if (net_fetch_airport_board(dep, s_route_board_arr, FR_MAX_BOARD_ROWS, &an, s_route_board_dep,
                                FR_MAX_BOARD_ROWS, &dn, lab, sizeof(lab))) {
        for (int i = 0; i < dn && n < out_max; i++) {
            if (apt_codes_match(s_route_board_dep[i].other_apt, arr)) {
                n = route_push_unique(out, n, out_max, s_route_board_dep[i]);
            }
        }
    }

    status_msg("Route: arrivals...");
    an = 0;
    dn = 0;
    if (net_fetch_airport_board(arr, s_route_board_arr, FR_MAX_BOARD_ROWS, &an, s_route_board_dep,
                                FR_MAX_BOARD_ROWS, &dn, lab, sizeof(lab))) {
        for (int i = 0; i < an && n < out_max; i++) {
            if (apt_codes_match(s_route_board_arr[i].other_apt, dep)) {
                n = route_push_unique(out, n, out_max, s_route_board_arr[i]);
            }
        }
    }

    // Primary source for scheduled + estimated times (FA.findflight.flights JSON).
    status_msg("Route: findflight...");
    char enc_d[16], enc_a[16];
    url_encode(dep, enc_d, sizeof(enc_d));
    url_encode(arr, enc_a, sizeof(enc_a));
    char url[192];
    snprintf(url, sizeof(url), "https://www.flightaware.com/live/findflight/%s/%s", enc_d, enc_a);
    if (https_get(url, nullptr, kUaBrowser, 10000, 90000, "text/html,application/xhtml+xml,*/*", true)) {
        size_t blen = 0;
        const char *html = http_body_buf(&blen);
        if (html && blen > 200) {
            const int fn = fa_parse_findflight_rows(html, s_route_ff, FR_MAX_BOARD_ROWS, arr);
            for (int i = 0; i < fn && n < out_max; i++) {
                n = route_push_unique(out, n, out_max, s_route_ff[i]);
            }
        }
    }

    if (out_n) {
        *out_n = n;
    }
    return n > 0;
}

// ---- SMTP email (implicit TLS) -----------------------------------------------

static bool tls_write_all(NetworkClientSecure &c, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    size_t off = 0;
    const uint32_t t0 = millis();
    while (off < len) {
        if (!c.connected()) {
            return false;
        }
        const size_t chunk = (len - off) > 1400 ? 1400 : (len - off);
        const size_t w = c.write(data + off, chunk);
        if (w == 0) {
            if (millis() - t0 > timeout_ms) {
                return false;
            }
            delay(2);
            continue;
        }
        off += w;
    }
    return true;
}

static bool smtp_send(NetworkClientSecure &c, const char *s) {
    return tls_write_all(c, (const uint8_t *)s, strlen(s), 15000);
}

/** Read a (possibly multi-line) SMTP reply; returns numeric code or -1. */
static int smtp_read(NetworkClientSecure &c, char *last_line, size_t last_len, uint32_t timeout_ms) {
    char line[160];
    int code = -1;
    const uint32_t t0 = millis();
    for (;;) {
        size_t n = 0;
        while (millis() - t0 < timeout_ms) {
            if (!c.available()) {
                if (!c.connected()) {
                    return -1;
                }
                delay(5);
                continue;
            }
            const int ch = c.read();
            if (ch < 0) {
                continue;
            }
            if (ch == '\n') {
                break;
            }
            if (ch != '\r' && n + 1 < sizeof(line)) {
                line[n++] = (char)ch;
            }
        }
        line[n] = 0;
        if (n < 3 || !isdigit((unsigned char)line[0])) {
            return -1;
        }
        code = (line[0] - '0') * 100 + (line[1] - '0') * 10 + (line[2] - '0');
        if (last_line && last_len) {
            copy_trunc(last_line, last_len, line);
        }
        if (n < 4 || line[3] != '-') {
            return code;
        }
        if (millis() - t0 >= timeout_ms) {
            return -1;
        }
    }
}

static bool smtp_expect(NetworkClientSecure &c, int want, const char *what) {
    char line[160] = {0};
    const int code = smtp_read(c, line, sizeof(line), 15000);
    if (code != want) {
        char msg[96];
        snprintf(msg, sizeof(msg), "SMTP %s: %s", what, line[0] ? line : "no reply");
        set_err(msg);
        return false;
    }
    return true;
}

static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/** Encode len bytes -> out (needs 4*ceil(len/3)+1). Returns chars written. */
static size_t b64_encode(const uint8_t *in, size_t len, char *out) {
    size_t o = 0;
    for (size_t i = 0; i < len; i += 3) {
        const uint32_t b0 = in[i];
        const uint32_t b1 = (i + 1 < len) ? in[i + 1] : 0;
        const uint32_t b2 = (i + 2 < len) ? in[i + 2] : 0;
        const uint32_t v = (b0 << 16) | (b1 << 8) | b2;
        out[o++] = kB64[(v >> 18) & 63];
        out[o++] = kB64[(v >> 12) & 63];
        out[o++] = (i + 1 < len) ? kB64[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < len) ? kB64[v & 63] : '=';
    }
    out[o] = 0;
    return o;
}

bool net_send_email_attachment(const EmailConfig *email, const uint8_t *data, size_t len, const char *filename,
                               const char *mime, const char *subject, const char *body_text) {
    if (!email || !data || len == 0 || !filename || !mime) {
        set_err("Email: bad arguments");
        return false;
    }
    if (!email->smtp_host[0] || !email->from_addr[0] || !email->to_addr[0]) {
        set_err("Email: set SMTP host/from/to");
        return false;
    }
    const uint16_t port = email->smtp_port ? email->smtp_port : 465;
    const char *user = email->username[0] ? email->username : email->from_addr;

    status_msg("Connecting SMTP...");
    NetworkClientSecure client;
    client.setInsecure();
    client.setTimeout(15);
    if (!client.connect(email->smtp_host, port, 10000)) {
        set_err("SMTP connect failed (use SSL port 465)");
        return false;
    }
    char b64[200];
    char cmd[200];
    bool ok = smtp_expect(client, 220, "greeting");
    ok = ok && smtp_send(client, "EHLO flightradar\r\n") && smtp_expect(client, 250, "EHLO");
    if (ok && email->password[0]) {
        ok = smtp_send(client, "AUTH LOGIN\r\n") && smtp_expect(client, 334, "AUTH");
        if (ok) {
            b64_encode((const uint8_t *)user, strlen(user), b64);
            snprintf(cmd, sizeof(cmd), "%s\r\n", b64);
            ok = smtp_send(client, cmd) && smtp_expect(client, 334, "user");
        }
        if (ok) {
            b64_encode((const uint8_t *)email->password, strlen(email->password), b64);
            snprintf(cmd, sizeof(cmd), "%s\r\n", b64);
            ok = smtp_send(client, cmd) && smtp_expect(client, 235, "login");
        }
    }
    if (ok) {
        snprintf(cmd, sizeof(cmd), "MAIL FROM:<%s>\r\n", email->from_addr);
        ok = smtp_send(client, cmd) && smtp_expect(client, 250, "MAIL FROM");
    }
    if (ok) {
        snprintf(cmd, sizeof(cmd), "RCPT TO:<%s>\r\n", email->to_addr);
        ok = smtp_send(client, cmd) && smtp_expect(client, 250, "RCPT TO");
    }
    ok = ok && smtp_send(client, "DATA\r\n") && smtp_expect(client, 354, "DATA");
    if (!ok) {
        client.stop();
        return false;
    }

    static const char *kBoundary = "=_FlightRadar_7MA4YWxk";
    char hdr[640];
    snprintf(hdr, sizeof(hdr),
             "From: Flight Radar <%s>\r\n"
             "To: <%s>\r\n"
             "Subject: %s\r\n"
             "MIME-Version: 1.0\r\n"
             "Content-Type: multipart/mixed; boundary=\"%s\"\r\n"
             "\r\n"
             "--%s\r\n"
             "Content-Type: text/plain; charset=UTF-8\r\n"
             "\r\n"
             "%s\r\n"
             "\r\n"
             "--%s\r\n"
             "Content-Type: %s; name=\"%s\"\r\n"
             "Content-Transfer-Encoding: base64\r\n"
             "Content-Disposition: attachment; filename=\"%s\"\r\n"
             "\r\n",
             email->from_addr, email->to_addr, subject && subject[0] ? subject : "Flight Radar", kBoundary,
             kBoundary, body_text && body_text[0] ? body_text : "Attached.", kBoundary, mime, filename, filename);
    status_msg("Sending attachment...");
    ok = smtp_send(client, hdr);

    // Base64 body: 57 input bytes -> one 76-char line. Batch many lines per write.
    constexpr size_t kLinesPerWrite = 24;
    char *out = (char *)heap_caps_malloc(kLinesPerWrite * 78 + 8, MALLOC_CAP_8BIT);
    if (!out) {
        client.stop();
        set_err("Email: out of memory");
        return false;
    }
    size_t off = 0;
    while (ok && off < len) {
        size_t o = 0;
        for (size_t l = 0; l < kLinesPerWrite && off < len; l++) {
            size_t n = len - off;
            if (n > 57) {
                n = 57;
            }
            o += b64_encode(data + off, n, out + o);
            out[o++] = '\r';
            out[o++] = '\n';
            off += n;
        }
        ok = tls_write_all(client, (const uint8_t *)out, o, 20000);
    }
    heap_caps_free(out);
    if (ok) {
        snprintf(hdr, sizeof(hdr), "\r\n--%s--\r\n.\r\n", kBoundary);
        ok = smtp_send(client, hdr);
    }
    if (!ok) {
        client.stop();
        set_err("SMTP send failed");
        return false;
    }
    ok = smtp_expect(client, 250, "send");
    if (ok) {
        smtp_send(client, "QUIT\r\n");
    }
    client.stop();
    return ok;
}

bool net_send_screenshot_email(const EmailConfig *email, const uint8_t *bmp, size_t bmp_len) {
    return net_send_email_attachment(email, bmp, bmp_len, "radar.bmp", "image/bmp", "Flight Radar screenshot",
                                     "Screenshot from Flight Radar.");
}

// ---- SD data pack download ------------------------------------------------------

static bool download_to_sd(const char *url, const char *dest_path, uint32_t expect_size) {
    bool tls = true;
    char host[80], path[200];
    uint16_t port = 443;
    if (!parse_url(url, &tls, host, sizeof(host), &port, path, sizeof(path))) {
        set_err("Update: bad URL");
        return false;
    }
    NetworkClientSecure client;
    client.setInsecure();
    client.setTimeout(15);
    if (!client.connect(host, port, 10000)) {
        set_err("Update: connect failed");
        return false;
    }
    // HTTP/1.0 keeps the reply un-chunked so the body can be streamed straight to SD.
    char req[320];
    snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: %s\r\nConnection: close\r\n\r\n", path, host,
             kUa);
    if (!tls_write_all(client, (const uint8_t *)req, strlen(req), 10000)) {
        set_err("Update: request failed");
        return false;
    }
    // Status line + headers.
    char line[160];
    int status = 0;
    bool first = true;
    for (;;) {
        size_t n = 0;
        const uint32_t t0 = millis();
        while (millis() - t0 < 15000) {
            if (!client.available()) {
                if (!client.connected()) {
                    break;
                }
                delay(5);
                continue;
            }
            const int ch = client.read();
            if (ch < 0) {
                continue;
            }
            if (ch == '\n') {
                break;
            }
            if (ch != '\r' && n + 1 < sizeof(line)) {
                line[n++] = (char)ch;
            }
        }
        line[n] = 0;
        if (first) {
            first = false;
            status = (n > 12) ? atoi(line + 9) : 0;
            if (status != 200) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Update: HTTP %d", status);
                set_err(msg);
                return false;
            }
            continue;
        }
        if (n == 0) {
            break;  // blank line = end of headers
        }
    }

    char tmp_path[200];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", dest_path);
    sd_lock();
    FILE *f = fopen(tmp_path, "wb");
    sd_unlock();
    if (!f) {
        set_err("Update: SD open failed");
        return false;
    }
    uint8_t *buf = (uint8_t *)heap_caps_malloc(4096, MALLOC_CAP_8BIT);
    if (!buf) {
        sd_lock();
        fclose(f);
        sd_unlock();
        set_err("Update: out of memory");
        return false;
    }
    size_t total = 0;
    bool ok = true;
    uint32_t last_data = millis();
    while (client.connected() || client.available()) {
        const int avail = client.available();
        if (avail <= 0) {
            if (millis() - last_data > 15000) {
                ok = false;
                break;
            }
            delay(2);
            continue;
        }
        const int want = avail > 4096 ? 4096 : avail;
        const int got = client.read(buf, want);
        if (got <= 0) {
            continue;
        }
        last_data = millis();
        sd_lock();
        const size_t w = fwrite(buf, 1, (size_t)got, f);
        sd_unlock();
        if (w != (size_t)got) {
            ok = false;
            break;
        }
        total += (size_t)got;
    }
    heap_caps_free(buf);
    sd_lock();
    const int crc = fclose(f);
    sd_unlock();
    client.stop();
    if (!ok || crc != 0 || total == 0 || (expect_size && total != expect_size)) {
        sd_lock();
        remove(tmp_path);
        sd_unlock();
        set_err(ok ? "Update: size mismatch" : "Update: download failed");
        return false;
    }
    sd_lock();
    remove(dest_path);
    const int rr = rename(tmp_path, dest_path);
    sd_unlock();
    if (rr != 0) {
        set_err("Update: SD rename failed");
        return false;
    }
    return true;
}

bool net_install_fr_data() {
    if (!sd_mount()) {
        set_err("Update: SD not mounted");
        return false;
    }
    sd_mkdir_p(SD_MOUNT_POINT "/fr_data");
    sd_mkdir_p(SD_MOUNT_POINT "/fr_data/plates");
    char url[256];
    snprintf(url, sizeof(url), "%s/manifest.json", FR_SD_DATA_BASE_URL);
    status_msg("Update: manifest...");
    String body;
    if (!https_get(url, &body, kUa, 8000, 12000)) {
        return false;
    }
    JsonDocument doc;
    if (deserializeJson(doc, body) || !doc["files"].is<JsonArray>()) {
        set_err("Update: bad manifest");
        return false;
    }
    JsonArray files = doc["files"].as<JsonArray>();
    const int total = (int)files.size();
    int idx = 0;
    for (JsonObject fo : files) {
        const char *rel = fo["path"] | "";
        const uint32_t size = fo["size"] | 0u;
        idx++;
        // Reject anything that could escape fr_data.
        if (!rel[0] || strstr(rel, "..") || rel[0] == '/') {
            continue;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "Update %d/%d: %s", idx, total, rel);
        status_msg(msg);
        char dest[160];
        snprintf(dest, sizeof(dest), "%s/fr_data/%s", SD_MOUNT_POINT, rel);
        snprintf(url, sizeof(url), "%s/%s", FR_SD_DATA_BASE_URL, rel);
        if (!download_to_sd(url, dest, size)) {
            return false;
        }
    }
    return true;
}
