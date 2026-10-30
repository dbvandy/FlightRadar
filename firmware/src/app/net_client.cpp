#include "net_client.h"
#include "airport_db.h"
#include "runway_db.h"
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
#include <cmath>
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

// HTTP body buffer — prefer PSRAM so internal DMA stays free for ESP-Hosted WiFi
// + mbedTLS. Steady-state stays at 192 KB (ADS-B / boards). FlightAware live pages
// are ~500–600 KB with Flight Data near EOF — grow to 768 KB only for that fetch,
// then shrink. A permanent 768 KB allocation left int-dma fragmented and killed TLS.
static char *g_http_buf = nullptr;
static size_t g_http_cap = 0;
static size_t g_http_len = 0;
// Steady-state: airport boards are ~170 KB now; 192 KB left no headroom after headers.
static constexpr size_t kHttpBufCap = 256 * 1024;
static constexpr size_t kHttpBufCapFa = 768 * 1024;

static bool ensure_http_buf_sized(size_t need) {
    if (need < kHttpBufCap) {
        need = kHttpBufCap;
    }
    if (g_http_buf && g_http_cap >= need) {
        return true;
    }
    // Large bodies must stay in PSRAM — internal fallback starves esp-aes DMA.
    // Allocate the new slab BEFORE freeing the old one so a failed grow never
    // leaves g_http_buf null (that broke airport board after FA flight-plan).
    char *p = (char *)heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p && need <= 64 * 1024) {
        p = (char *)heap_caps_malloc(need, MALLOC_CAP_8BIT);
    }
    if (!p) {
        set_err(need > 64 * 1024 ? "No HTTP PSRAM" : "No HTTP body RAM");
        return false;
    }
    if (g_http_buf) {
        heap_caps_free(g_http_buf);
    }
    g_http_buf = p;
    g_http_cap = need;
    g_http_len = 0;
    Serial.printf("HTTP body buf: %u KB\n", (unsigned)(g_http_cap / 1024));
    return true;
}

static bool ensure_http_buf() {
    return ensure_http_buf_sized(kHttpBufCap);
}

static void http_buf_shrink_default() {
    if (!g_http_buf || g_http_cap <= kHttpBufCap) {
        return;
    }
    // Alloc-before-free: never drop the large buffer unless the smaller one exists.
    char *p = (char *)heap_caps_malloc(kHttpBufCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        Serial.printf("HTTP shrink skipped (keep %u KB)\n", (unsigned)(g_http_cap / 1024));
        g_http_len = 0;
        return;
    }
    heap_caps_free(g_http_buf);
    g_http_buf = p;
    g_http_cap = kHttpBufCap;
    g_http_len = 0;
    Serial.printf("HTTP body buf: %u KB\n", (unsigned)(g_http_cap / 1024));
}

void net_init_buffers() {
    // Do NOT pre-reserve internal DMA here — that starved ESP-Hosted WiFi
    // (rpc Req_WifiInit timeout / esp_wifi_init FAIL) at boot.
    ensure_http_buf();
}

// Park a contiguous INTERNAL+DMA slab after WiFi is up so esp-aes can grab it
// for the next TLS handshake (LVGL fragments the heap). Never call before
// ESP-Hosted WiFi init — that caused Req_WifiInit timeouts.
static constexpr uint32_t kDmaInternal = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
static void *s_dma_tls_reserve = nullptr;
static size_t s_dma_tls_reserve_sz = 0;

static void dma_tls_reserve_release() {
    if (s_dma_tls_reserve) {
        heap_caps_free(s_dma_tls_reserve);
        s_dma_tls_reserve = nullptr;
        s_dma_tls_reserve_sz = 0;
    }
}

static void dma_tls_reserve_hold() {
    if (s_dma_tls_reserve) {
        return;
    }
    // Prefer ~40 KB; fall back so we still leave a usable hole for esp-aes.
    for (size_t kb = 40; kb >= 16; kb -= 4) {
        const size_t sz = kb * 1024;
        void *p = heap_caps_malloc(sz, kDmaInternal);
        if (p) {
            s_dma_tls_reserve = p;
            s_dma_tls_reserve_sz = sz;
            Serial.printf("DMA TLS reserve +%u KB (int-dma free=%u largest=%u)\n", (unsigned)kb,
                          (unsigned)heap_caps_get_free_size(kDmaInternal),
                          (unsigned)heap_caps_get_largest_free_block(kDmaInternal));
            return;
        }
    }
    Serial.printf("DMA TLS reserve FAIL (int-dma free=%u largest=%u)\n",
                  (unsigned)heap_caps_get_free_size(kDmaInternal),
                  (unsigned)heap_caps_get_largest_free_block(kDmaInternal));
}

void net_arm_tls_dma_after_wifi() {
    dma_tls_reserve_hold();
}

// Sticky TLS session — full handshakes every ADS-B poll thrash internal DMA RAM
// (esp-aes descriptors) once LVGL/airports have fragmented the heap. Keep-alive
// reuses one session on the preferred host; FA pages use force_close.
static NetworkClientSecure *g_tls = nullptr;
static char g_tls_host[80] = "";
static uint16_t g_tls_port = 0;
// Set on every tls_close so the next handshake can wait for stack teardown.
static uint32_t s_tls_last_close_ms = 0;
static bool s_tls_last_was_fa = false;

static bool host_is_flightaware(const char *host) {
    return host && (strcasecmp(host, "www.flightaware.com") == 0 ||
                    strcasecmp(host, "flightaware.com") == 0);
}

static void tls_close() {
    if (g_tls) {
        s_tls_last_was_fa = host_is_flightaware(g_tls_host);
        g_tls->stop();
        delete g_tls;
        g_tls = nullptr;
        s_tls_last_close_ms = millis();
    }
    g_tls_host[0] = 0;
    g_tls_port = 0;
    // Session gone — park contiguous DMA again for the next handshake.
    dma_tls_reserve_hold();
}

void net_tls_reset() {
    tls_close();
}

static NetworkClientSecure *tls_acquire(const char *host, uint16_t port, uint32_t connect_ms, uint32_t io_ms) {
    if (g_tls && g_tls->connected() && g_tls_port == port && strcmp(g_tls_host, host) == 0) {
        return g_tls;
    }
    tls_close();
    if (WiFi.status() != WL_CONNECTED) {
        set_err("TLS Wi-Fi down");
        return nullptr;
    }
    // Long settle only after / before FlightAware — ADS-B must stay snappy.
    const bool fa = host_is_flightaware(host) || s_tls_last_was_fa;
    const uint32_t settle_ms = fa ? 1500u : 200u;
    if (s_tls_last_close_ms) {
        const uint32_t since = millis() - s_tls_last_close_ms;
        if (since < settle_ms) {
            delay(settle_ms - since);
        }
    }

    // Free the parked slab so esp-aes can grab contiguous DMA descriptors.
    dma_tls_reserve_release();

    char last_tlserr[72] = {0};
    for (int attempt = 1; attempt <= 2; attempt++) {
        NetworkClientSecure *client = new NetworkClientSecure();
        if (!client) {
            set_err("No TLS client RAM");
            dma_tls_reserve_hold();
            return nullptr;
        }
        client->setInsecure();
        client->setHandshakeTimeout((connect_ms + 999) / 1000);
        client->setTimeout((io_ms + 999) / 1000);
        Serial.printf("TLS connect %s:%u try %d (free=%u int-dma=%u largest=%u)\n", host, port, attempt,
                      (unsigned)ESP.getFreeHeap(), (unsigned)heap_caps_get_free_size(kDmaInternal),
                      (unsigned)heap_caps_get_largest_free_block(kDmaInternal));
        if (client->connect(host, port, (int32_t)connect_ms)) {
            g_tls = client;
            strncpy(g_tls_host, host, sizeof(g_tls_host) - 1);
            g_tls_host[sizeof(g_tls_host) - 1] = 0;
            g_tls_port = port;
            s_tls_last_was_fa = false;
            Serial.printf("TLS session open %s:%u\n", host, port);
            return g_tls;
        }
        client->lastError(last_tlserr, sizeof(last_tlserr));
        Serial.printf("TLS fail try %d: %s\n", attempt, last_tlserr[0] ? last_tlserr : "(none)");
        client->stop();
        delete client;
        s_tls_last_close_ms = millis();
        if (attempt == 1) {
            delay(fa ? 2000 : 400);
            yield();
        }
    }
    dma_tls_reserve_hold();
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

// When a page-specific early_ok is set (FA board), NEVER keep a partial just
// because we hit 64 KB — arrivals sit ~68 KB in and departures ~83 KB; a 64 KB
// stall left the dep column empty. Generic downloads still use the 64 KB floor.
static bool http_partial_usable(const char *buf, size_t len) {
    if (g_http_early_ok) {
        return g_http_early_ok(buf, len);
    }
    return len >= 64 * 1024;
}

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
    // Headers are tiny but ESP-Hosted + first TLS write can be slow after join.
    const uint32_t hdr_ms = io_ms > 20000 ? 20000 : (io_ms < 12000 ? 12000 : io_ms);
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
        Serial.printf("HTTP hdr timeout after %u ms (got %u B, conn=%d int-dma largest=%u)\n",
                      (unsigned)(millis() - start), (unsigned)len, client.connected() ? 1 : 0,
                      (unsigned)heap_caps_get_largest_free_block(kDmaInternal));
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
    // or into the body buffer for Content-Length / raw. Keep this in PSRAM so
    // internal DMA stays free for esp-aes during the TLS body read.
    static char *pushback = nullptr;
    static constexpr size_t kPushbackCap = 16 * 1024;
    if (!pushback) {
        pushback = (char *)heap_caps_malloc(kPushbackCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!pushback) {
            pushback = (char *)heap_caps_malloc(kPushbackCap, MALLOC_CAP_8BIT);
        }
        if (!pushback) {
            set_err("No HTTP pushback RAM");
            return false;
        }
    }
    size_t push_n = 0;
    if (chunked) {
        if (pre_len > kPushbackCap) {
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
                if (http_partial_usable(g_http_buf, len)) {
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
                    if (http_partial_usable(g_http_buf, len)) {
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
                if (http_partial_usable(g_http_buf, len)) {
                    Serial.printf("HTTP chunk stall after %u KB — using partial body\n",
                                  (unsigned)(len / 1024));
                    break;
                }
                Serial.printf("HTTP chunk data timeout at %u KB\n", (unsigned)(len / 1024));
                set_err("HTTP chunk data timeout");
                return false;
            }
            len += (size_t)csz;
            g_http_buf[len] = 0;
            if ((len / 16384) != ((len - (size_t)csz) / 16384)) {
                char st[40];
                snprintf(st, sizeof(st), "Download %u KB…", (unsigned)(len / 1024));
                status_msg(st);
            }
            // FA board: stop as soon as boards are present (force_close path).
            if (g_http_early_ok && g_http_early_ok(g_http_buf, len)) {
                Serial.printf("HTTP early-complete at %u KB\n", (unsigned)(len / 1024));
                goto body_done;
            }
            // Trailing CRLF after chunk data.
            uint8_t crlf[2];
            if (!take_bytes((char *)crlf, 2)) {
                if (http_partial_usable(g_http_buf, len)) {
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
            // Sticky keep-alive for ADS-B (force_close=false). Closing every poll
            // forces a new mbedTLS handshake and runs out of internal DMA RAM
            // (esp-aes descriptor alloc fails → "TLS generic error").
            // FA HTML / redirects: force_close=true → Connection: close each time.
            const bool same_host =
                g_tls && g_tls->connected() && g_tls_port == port && strcmp(g_tls_host, host) == 0;
            if (force_close || !same_host) {
                tls_close();
            }
            NetworkClientSecure *client = tls_acquire(host, port, connect_ms, io_ms);
            if (!client) {
                g_http_early_ok = nullptr;
                return false;
            }
            // Build request in one write so TLS record framing is clean on Hosted WiFi.
            static char req[1536];
            const int req_n =
                snprintf(req, sizeof(req),
                         "GET %s HTTP/1.1\r\n"
                         "Host: %s\r\n"
                         "User-Agent: %s\r\n"
                         "Accept: %s\r\n"
                         "Accept-Encoding: identity\r\n"
                         "Connection: %s\r\n\r\n",
                         path, host, ua, acc, force_close ? "close" : "keep-alive");
            if (req_n <= 0 || (size_t)req_n >= sizeof(req)) {
                set_err("HTTP request too large");
                tls_close();
                g_http_early_ok = nullptr;
                return false;
            }
            const int wrote = client->write((const uint8_t *)req, (size_t)req_n);
            if (wrote != req_n) {
                Serial.printf("HTTP write short %d/%d\n", wrote, req_n);
                set_err("HTTP write failed");
                tls_close();
                g_http_early_ok = nullptr;
                return false;
            }
            client->flush();
            Serial.printf("HTTP req %d B sent (int-dma free=%u largest=%u)\n", req_n,
                          (unsigned)heap_caps_get_free_size(kDmaInternal),
                          (unsigned)heap_caps_get_largest_free_block(kDmaInternal));
            if (force_close) {
                ok = read_http_body(*client, body, io_ms, false);
                tls_close();
            } else {
                ok = read_http_body(*client, body, io_ms, true);
                if (!ok) {
                    tls_close();
                }
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

static bool looks_like_airport_code(const char *q) {
    if (!q || !q[0]) {
        return false;
    }
    size_t n = 0;
    for (const char *p = q; *p; p++) {
        if (*p == ' ' || *p == '-' || *p == '/') {
            continue;
        }
        if (!isalnum((unsigned char)*p)) {
            return false;
        }
        n++;
        if (n > 4) {
            return false;
        }
    }
    return n == 3 || n == 4;
}

static int geocode_airport_db(const char *query, GeoFix *out, size_t out_max) {
    if (!query || !out || out_max == 0 || !kAptDb || K_APT_DB_COUNT == 0) {
        return 0;
    }
    // First alphanumeric token only ("kocf", "OCF", "KOCF airport").
    char tok[16];
    size_t w = 0;
    bool started = false;
    for (size_t i = 0; query[i] && w + 1 < sizeof(tok); i++) {
        const unsigned char c = (unsigned char)query[i];
        if (!started && (c == ' ' || c == '\t' || c == '-' || c == '/')) {
            continue;
        }
        if (!isalnum(c)) {
            if (started) {
                break;
            }
            continue;
        }
        started = true;
        tok[w++] = (char)toupper(c);
    }
    tok[w] = 0;
    if (w != 3 && w != 4) {
        return 0;
    }
    // Reuse the same matcher as Airport Board / Dep-Arr search (exact + K-prefix).
    AptSearchHit hits[4];
    const int hn = net_search_airports(tok, hits, sizeof(hits) / sizeof(hits[0]));
    int n = 0;
    for (int i = 0; i < hn && (size_t)n < out_max; i++) {
        // Exact / K-prefix only — drop prefix-pass noise like "KOC*" for "KOCF".
        const bool exact = strcmp(hits[i].icao, tok) == 0 ||
                           (hits[i].iata[0] && strcmp(hits[i].iata, tok) == 0) ||
                           (w == 3 && hits[i].icao[0] == 'K' && strcmp(hits[i].icao + 1, tok) == 0);
        if (!exact) {
            continue;
        }
        memset(&out[n], 0, sizeof(out[n]));
        copy_trunc(out[n].query, sizeof(out[n].query), query);
        copy_trunc(out[n].icao, sizeof(out[n].icao), hits[i].icao);
        copy_trunc(out[n].label, sizeof(out[n].label), hits[i].icao);  // ICAO only
        out[n].lat = hits[i].lat;
        out[n].lon = hits[i].lon;
        out[n].valid = true;
        n++;
    }
    return n;
}

/** If lat/lon sit on an airport, bind ICAO and replace label with that code. */
static bool bind_nearest_airport_home(GeoFix *fix, float max_nm) {
    if (!fix || !fix->valid || !kAptDb || K_APT_DB_COUNT == 0) {
        return false;
    }
    const float max2 = max_nm * max_nm;
    int best = -1;
    float best_d2 = max2;
    for (size_t i = 0; i < K_APT_DB_COUNT; i++) {
        const AptRec &rec = kAptDb[i];
        const float alat = rec.lat_c / 100.0f;
        const float alon = rec.lon_c / 100.0f;
        float east, north;
        ll_to_nm(alat, alon, fix->lat, fix->lon, &east, &north);
        const float d2 = east * east + north * north;
        if (d2 <= best_d2) {
            best_d2 = d2;
            best = (int)i;
        }
    }
    if (best < 0) {
        return false;
    }
    const AptRec &rec = kAptDb[best];
    char icao[8];
    copy_trunc(icao, sizeof(icao), rec.icao);
    const bool changed = strcmp(fix->icao, icao) != 0 || strcmp(fix->label, icao) != 0;
    copy_trunc(fix->icao, sizeof(fix->icao), icao);
    copy_trunc(fix->label, sizeof(fix->label), icao);
    fix->lat = rec.lat_c / 100.0f;
    fix->lon = rec.lon_c / 100.0f;
    return changed;
}

int net_geocode(const char *query, GeoFix *out, size_t out_max) {
    char q[96];
    copy_trunc(q, sizeof(q), query);
    if (q[0] == 0 || out_max == 0) {
        return 0;
    }
    int n = 0;
    // ICAO/IATA first — never return a facility name / street address for airport homes.
    if (looks_like_airport_code(q)) {
        n = geocode_airport_db(q, out, out_max);
        Serial.printf("[geo] code='%s' apt_hits=%d db=%u\n", q, n, (unsigned)K_APT_DB_COUNT);
        // Pure airport codes must not fall through to Nominatim (addresses / wrong place).
        if (n > 0) {
            return n;
        }
        return 0;
    }
    if (looks_like_us_zip(q)) {
        n = geocode_zippopotam(q, out, out_max);
    }
    if (n == 0) {
        n = geocode_nominatim(q, out, out_max);
        // Snap Nominatim "… Airport …" rows to ICAO; leave cities/ZIPs alone.
        auto label_looks_airport = [](const char *lab) -> bool {
            if (!lab || !lab[0]) {
                return false;
            }
            char up[96];
            copy_trunc(up, sizeof(up), lab);
            for (char *p = up; *p; p++) {
                *p = (char)toupper((unsigned char)*p);
            }
            return strstr(up, "AIRPORT") || strstr(up, "AERODROME") || strstr(up, "AIRFIELD") ||
                   strstr(up, " AIR ") || strstr(up, "INTL");
        };
        for (int i = 0; i < n; i++) {
            if (label_looks_airport(out[i].label)) {
                bind_nearest_airport_home(&out[i], 5.0f);
            }
        }
    }
    Serial.printf("[geo] q='%s' hits=%d\n", q, n);
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
    // adsb.lol → "ac"; adsb.fi → "aircraft".
    JsonArray ac = doc["ac"].as<JsonArray>();
    if (ac.isNull()) {
        ac = doc["aircraft"].as<JsonArray>();
    }
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

    // Prefer one host per poll. A second TLS handshake (failover) often fails
    // once internal DMA is fragmented — empty airspace is a valid answer.
    for (int attempt = 0; attempt < nsrc; attempt++) {
        const int idx = (g_adsb_pref + attempt) % nsrc;
        char msg[48];
        snprintf(msg, sizeof(msg), "ADS-B %s...", kSrc[idx].name);
        status_msg(msg);
        snprintf(url, sizeof(url), kSrc[idx].fmt, lat, lon, dist);
        // nullptr body → fill g_http_buf only (no second PSRAM String copy).
        // force_close=true: Hosted Wi‑Fi + sticky keep-alive was stalling after
        // handshake (HTTP header timeout) and then breaking DNS.
        if (!https_get(url, nullptr, kUa, 8000, 25000, nullptr, true)) {
            continue;
        }
        size_t blen = 0;
        const char *buf = http_body_buf(&blen);
        const int n = parse_adsblol(buf, blen, out, out_max);
        if (n > 0) {
            g_adsb_pref = idx;
            return n;
        }
        if (n < 0) {
            // Bad JSON / wrong shape — try the other host this poll.
            continue;
        }
        // HTTP OK with 0 aircraft — do not open a second TLS session this poll.
        g_adsb_pref = idx;
        any_ok = true;
        best_empty = 0;
        break;
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
        // Flash-embedded RwyRec has no turf/plate offsets (those were SD-only).
        dst.is_turf = false;
        dst.east_ft = 0;
        dst.north_ft = 0;
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
        if (!https_get(url, nullptr, kUa, 8000, 20000)) {
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

/** "JACKSONVILLE INTL" / "ATLANTA/HARTSFIELD/..." -> "Jacksonville" / "Atlanta". */
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
    // AWC often uses "CITY/FACILITY..." — city is the first segment.
    if (char *slash = strchr(tmp, '/')) {
        *slash = 0;
        trim_inplace(tmp);
    }
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
    if (!fix || !fix->valid) {
        return false;
    }
    char before_icao[8];
    char before_label[96];
    copy_trunc(before_icao, sizeof(before_icao), fix->icao);
    copy_trunc(before_label, sizeof(before_label), fix->label);

    // Resolve ICAO from query / existing icao / nearest DB airport.
    if (!fix->icao[0] && looks_like_airport_code(fix->query)) {
        GeoFix hit{};
        if (geocode_airport_db(fix->query, &hit, 1) == 1) {
            copy_trunc(fix->icao, sizeof(fix->icao), hit.icao);
            fix->lat = hit.lat;
            fix->lon = hit.lon;
        }
    }
    if (!fix->icao[0] && looks_like_airport_code(fix->label)) {
        GeoFix hit{};
        if (geocode_airport_db(fix->label, &hit, 1) == 1) {
            copy_trunc(fix->icao, sizeof(fix->icao), hit.icao);
            fix->lat = hit.lat;
            fix->lon = hit.lon;
        }
    }
    if (!fix->icao[0]) {
        // Nominatim "… Airport …" / Center-without-icao: snap if sitting on a field.
        bind_nearest_airport_home(fix, 2.0f);
    }
    if (!fix->icao[0]) {
        return false;  // city/ZIP home — leave label alone
    }
    upper_inplace(fix->icao);
    // Display + persist ICAO only (no facility name / city / street address).
    copy_trunc(fix->label, sizeof(fix->label), fix->icao);
    return strcmp(before_icao, fix->icao) != 0 || strcmp(before_label, fix->label) != 0;
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

/** True when the FlightStats page is the airborne/current sortie (not landed, not next scheduled). */
static bool fs_flight_is_current_airborne(JsonObjectConst fl) {
    if (fl.isNull()) {
        return false;
    }
    if (fl["isLanded"] | false) {
        return false;  // previous leg already on the ground
    }
    if (fl["isTracking"] | false) {
        return true;
    }
    JsonObjectConst note = fl["flightNote"];
    if (!note.isNull()) {
        if (note["landed"] | false) {
            return false;
        }
        if ((note["tracking"] | false) || (note["hasDepartedRunway"] | false) ||
            (note["hasDepartedGate"] | false)) {
            return true;
        }
    }
    const char *st = fl["status"]["status"] | "";
    const char *code = fl["status"]["statusCode"] | "";
    const char *state = fl["flightState"] | "";
    if (strcasecmp(code, "S") == 0 || strcasecmp(st, "Scheduled") == 0 ||
        strcasecmp(state, "scheduled") == 0 || strcasecmp(state, "future") == 0) {
        return false;
    }
    if (st[0] && (strcasestr(st, "en route") || strcasestr(st, "enroute") || strcasestr(st, "airborne") ||
                  strcasestr(st, "active"))) {
        return true;
    }
    if (state[0] && (strcasecmp(state, "airborne") == 0 || strcasecmp(state, "active") == 0)) {
        return true;
    }
    return false;
}

/** Decode a few JSON \\uXXXX escapes in-place (FlightStats URLs use \\u0026 for &). */
static void fs_unescape_url(char *s) {
    if (!s) {
        return;
    }
    char *r = s, *w = s;
    while (*r) {
        if (r[0] == '\\' && r[1] == 'u' && isxdigit((unsigned char)r[2]) && isxdigit((unsigned char)r[3]) &&
            isxdigit((unsigned char)r[4]) && isxdigit((unsigned char)r[5])) {
            const int v = (int)strtol(r + 2, nullptr, 16);
            *w++ = (char)(v & 0xff);
            r += 6;
        } else {
            *w++ = *r++;
        }
    }
    *w = 0;
}

static bool fs_pick_current_url(JsonObjectConst tracker, char *url_out, size_t url_len) {
    if (url_out && url_len) {
        url_out[0] = 0;
    }
    if (tracker.isNull() || !url_out || url_len == 0) {
        return false;
    }
    JsonObjectConst cur = tracker["flight"];
    // Default FS page is often the *next* scheduled OR a just-landed prior turn.
    // Only skip the otherDays scan when we're clearly on the airborne leg.
    if (fs_flight_is_current_airborne(cur)) {
        return false;
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
                    // Ignore far-future scheduled turns (tomorrow's ATL-ORD, etc.).
                    if (ts > (long)now + 2 * 3600) {
                        continue;
                    }
                    // Ignore ancient history (keep the current operating day ± prior turn).
                    if (ts < (long)now - 18 * 3600) {
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
    char path[192];
    copy_trunc(path, sizeof(path), best_url);
    fs_unescape_url(path);
    if (path[0] == '/') {
        snprintf(url_out, url_len, "https://www.flightstats.com/v2%s", path);
    } else if (strncmp(path, "http", 4) == 0) {
        copy_trunc(url_out, url_len, path);
    } else {
        snprintf(url_out, url_len, "https://www.flightstats.com/v2/flight-tracker/%s", path);
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

/** True jet/victor/RNAV airway designator (Q118, J80, V8, T295…) — not a SID/STAR. */
static bool fa_is_airway(const char *t) {
    if (!t || !t[0] || !t[1]) {
        return false;
    }
    const char c0 = (char)toupper((unsigned char)t[0]);
    // Single-letter airways: A/B/G/J/L/Q/R/T/V/W + digits.
    if ((c0 == 'A' || c0 == 'B' || c0 == 'G' || c0 == 'J' || c0 == 'L' || c0 == 'Q' || c0 == 'R' ||
         c0 == 'T' || c0 == 'V' || c0 == 'W') &&
        isdigit((unsigned char)t[1])) {
        for (const char *p = t + 1; *p; p++) {
            if (!isdigit((unsigned char)*p)) {
                return false;
            }
        }
        return true;
    }
    // Two-letter continental airways (UG1, UL612, UA35…).
    if (isalpha((unsigned char)t[0]) && isalpha((unsigned char)t[1])) {
        bool dig = false;
        for (const char *p = t + 2; *p; p++) {
            if (isdigit((unsigned char)*p)) {
                dig = true;
            } else if (!isalpha((unsigned char)*p)) {
                return false;
            } else if (dig) {
                return false;
            }
        }
        return dig && strlen(t) <= 6;
    }
    return false;
}

/** SID/STAR like VARNM2 / VEECK5 / CHPPR1 — letters then trailing version digits. */
static bool fa_sid_star_base(const char *t, char *base, size_t base_len) {
    if (!t || !base || base_len < 3) {
        return false;
    }
    size_t n = strlen(t);
    if (n < 4 || n > 10) {
        return false;
    }
    size_t i = n;
    while (i > 0 && isdigit((unsigned char)t[i - 1])) {
        i--;
    }
    if (i < 3 || i == n) {
        return false;  // need ≥3 letters and ≥1 trailing digit
    }
    for (size_t k = 0; k < i; k++) {
        if (!isalpha((unsigned char)t[k])) {
            return false;
        }
    }
    if (i + 1 > base_len) {
        return false;
    }
    memcpy(base, t, i);
    base[i] = 0;
    for (char *q = base; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    return true;
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
    // SID/STAR designator → underlying fix (VARNM2→VARNM, VEECK5→VEECK).
    char base[12];
    if (fa_sid_star_base(name, base, sizeof(base)) && net_lookup_waypoint(base, lat, lon)) {
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

/** Copy a JSON \"route\":\"…\" value at p (points at first char after opening quote). */
static bool fa_copy_route_value(const char *p, char *out, size_t out_len) {
    if (!p || !out || out_len < 8) {
        return false;
    }
    size_t i = 0;
    while (p[i] && p[i] != '"' && i + 1 < out_len) {
        out[i] = p[i];
        i++;
    }
    out[i] = 0;
    return i > 0;
}

static bool fa_extract_route_string(const char *html, char *out, size_t out_len) {
    if (!html || !out || out_len < 8) {
        return false;
    }
    out[0] = 0;
    // Prefer the airborne flightPlan (activityLog lists future legs first with empty routes).
    const char *air = strstr(html, "\"flightStatus\":\"airborne\"");
    if (air && air > html + 200) {
        const char *fp = nullptr;
        for (const char *q = air - 14; q >= html && (size_t)(air - q) < 3000; q--) {
            if (strncmp(q, "\"flightPlan\":{", 14) == 0) {
                fp = q;
                break;
            }
        }
        if (fp) {
            const char *p = strstr(fp, "\"route\":\"");
            if (p && (size_t)(p - fp) < 800) {
                p += strlen("\"route\":\"");
                if (fa_copy_route_value(p, out, out_len)) {
                    return true;
                }
            }
        }
    }

    // FlightAware Flight Data → Route is flightPlan.route in trackpollBootstrap.
    // Live/scheduled legs often have "route":"" — skip empties and keep scanning
    // (history page / prior activity entries carry the filed IFR string).
    const char *search = html;
    for (int attempt = 0; attempt < 24; attempt++) {
        const char *fp = strstr(search, "\"flightPlan\":{");
        if (!fp) {
            break;
        }
        const char *p = strstr(fp, "\"route\":\"");
        if (!p || (size_t)(p - fp) > 800) {
            search = fp + 14;
            continue;
        }
        p += strlen("\"route\":\"");
        if (fa_copy_route_value(p, out, out_len)) {
            return true;
        }
        search = fp + 14;
    }
    return false;
}

/** True when a FA times object has a numeric \"actual\" (not null). */
static bool fa_times_actual_set(const char *obj, size_t span) {
    if (!obj) {
        return false;
    }
    const char *limit = obj + (span ? span : 160);
    const char *a = strstr(obj, "\"actual\":");
    if (!a || a >= limit) {
        return false;
    }
    a += 9;
    while (*a == ' ' || *a == '\t') {
        a++;
    }
    if (strncmp(a, "null", 4) == 0) {
        return false;
    }
    return (*a >= '0' && *a <= '9');
}

/** Extract ICAO pair from FA history path …/history/YYYYMMDD/HHMMZ/KATL/KORD. */
static bool fa_hist_path_airports(const char *path, char *origin, size_t origin_len, char *dest,
                                  size_t dest_len) {
    if (!path || !origin || !dest) {
        return false;
    }
    const char *h = strstr(path, "/history/");
    if (!h) {
        return false;
    }
    // /history/20261029/2337Z/KATL/KORD
    const char *p = h + 9;
    int slash = 0;
    const char *o = nullptr;
    const char *d = nullptr;
    for (; *p; p++) {
        if (*p == '/') {
            slash++;
            if (slash == 2) {
                o = p + 1;
            } else if (slash == 3) {
                d = p + 1;
                break;
            }
        }
    }
    if (!o || !d) {
        return false;
    }
    size_t ol = 0;
    while (o[ol] && o[ol] != '/' && o[ol] != '"' && ol + 1 < origin_len) {
        ol++;
    }
    size_t dl = 0;
    while (d[dl] && d[dl] != '/' && d[dl] != '"' && d[dl] != '?' && dl + 1 < dest_len) {
        dl++;
    }
    if (ol < 3 || ol > 4 || dl < 3 || dl > 4) {
        return false;
    }
    memcpy(origin, o, ol);
    origin[ol] = 0;
    memcpy(dest, d, dl);
    dest[dl] = 0;
    for (char *q = origin; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    for (char *q = dest; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    return true;
}

/**
 * Pick the current activityLog leg: prefer airborne (actual takeoff, no actual landing).
 * Falls back to the most recent completed takeoff. Fills ICAO origin/dest + history path.
 * \"operated\" on the live page often points at a *future* scheduled turn — do not use it.
 */
static bool fa_pick_current_activity(const char *html, char *origin, size_t origin_len, char *dest,
                                     size_t dest_len, char *hist_path, size_t hist_len) {
    if (origin && origin_len) {
        origin[0] = 0;
    }
    if (dest && dest_len) {
        dest[0] = 0;
    }
    if (hist_path && hist_len) {
        hist_path[0] = 0;
    }
    if (!html || !origin || !dest || !hist_path || origin_len < 5 || dest_len < 5 || hist_len < 24) {
        return false;
    }
    const char *log = strstr(html, "\"activityLog\"");
    if (!log) {
        return false;
    }
    // activityLog sits near the end of the bootstrap blob (~60–80 KB of flights).
    const char *end = log + 90000;
    const char *boot = strstr(log, "trackpollBootstrap");
    if (boot && boot < end) {
        end = boot;
    }

    const char *best_air_to = nullptr;
    const char *best_done_to = nullptr;
    for (const char *p = log; p && p < end;) {
        const char *to = strstr(p, "\"takeoffTimes\":{");
        if (!to || to >= end) {
            break;
        }
        if (!fa_times_actual_set(to, 160)) {
            p = to + 16;
            continue;
        }
        const char *ld = strstr(to, "\"landingTimes\":{");
        if (!ld || ld > to + 240) {
            p = to + 16;
            continue;
        }
        const bool landed = fa_times_actual_set(ld, 160);
        if (!landed) {
            // activityLog is newest-first — first airborne entry is the current leg.
            best_air_to = to;
            break;
        }
        if (!best_done_to) {
            best_done_to = to;  // first completed = most recent landing
        }
        p = to + 16;
    }

    const char *pick = best_air_to ? best_air_to : best_done_to;
    if (!pick) {
        return false;
    }

    // History URL is usually after the times objects in the same flight record.
    const char *url = nullptr;
    for (const char *u = pick; u && u < pick + 2500 && u < end;) {
        const char *hit = strstr(u, "\"/live/flight/");
        if (!hit || hit >= pick + 2500 || hit >= end) {
            break;
        }
        if (strstr(hit, "/history/") && hit < pick + 2500) {
            url = hit + 1;  // skip leading quote
            break;
        }
        u = hit + 14;
    }
    // Also search backward a bit (URL can precede times in some payloads).
    if (!url) {
        const char *window = pick - 2000;
        if (window < log) {
            window = log;
        }
        for (const char *u = window; u && u < pick;) {
            const char *hit = strstr(u, "\"/live/flight/");
            if (!hit || hit >= pick) {
                break;
            }
            if (strstr(hit, "/history/")) {
                url = hit + 1;
            }
            u = hit + 14;
        }
    }
    if (!url) {
        return false;
    }

    size_t pi = 0;
    while (url[pi] && url[pi] != '"' && pi + 1 < hist_len) {
        hist_path[pi] = url[pi];
        pi++;
    }
    hist_path[pi] = 0;

    if (!fa_hist_path_airports(hist_path, origin, origin_len, dest, dest_len)) {
        return false;
    }
    Serial.printf("FlightAware activity leg %s→%s %s\n", origin, dest, hist_path);
    return true;
}

static void fa_push_route_wpt(FlightInfo *fi, const char *name, float lat, float lon) {
    if (!fi || fi->route_wpt_n >= FR_MAX_ROUTE_WPTS) {
        return;
    }
    // Skip nearly-identical positions; allow the same fix name only if far apart
    // (nearest-FAA labeling often repeats a name across consecutive polyline samples).
    for (int i = 0; i < fi->route_wpt_n; i++) {
        float dlat = fi->route_wpts[i].lat - lat;
        float dlon = fi->route_wpts[i].lon - lon;
        if (dlat * dlat + dlon * dlon < 1e-4f) {  // ~0.6 nm
            if (name && name[0] && !fi->route_wpts[i].name[0]) {
                copy_trunc(fi->route_wpts[i].name, sizeof(fi->route_wpts[i].name), name);
                for (char *q = fi->route_wpts[i].name; *q; q++) {
                    *q = (char)toupper((unsigned char)*q);
                }
            }
            return;
        }
        if (name && name[0] && fi->route_wpts[i].name[0] &&
            strcasecmp(fi->route_wpts[i].name, name) == 0) {
            // Same label already on path — keep the earlier pin unless this is distant.
            if (dlat * dlat + dlon * dlon < 0.25f) {  // < ~30 nm
                return;
            }
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

/** Approx great-circle distance in nm (flat-earth); local helper for snap threshold. */
static float wpt_dist_proxy_nm(float lat1, float lon1, float lat2, float lon2) {
    const float mid = (lat1 + lat2) * 0.5f;
    float cos_lat = cosf(mid * FR_DEG2RAD);
    if (cos_lat < 0.15f) {
        cos_lat = 0.15f;
    }
    const float dlat = (lat2 - lat1) * 60.0f;
    const float dlon = (lon2 - lon1) * 60.0f * cos_lat;
    return sqrtf(dlat * dlat + dlon * dlon);
}

/** Label a polyline vertex from the FAA fix DB (preferred) or a nearby airport. */
static void fa_label_path_point(float lat, float lon, char *lbl, size_t lbl_len) {
    if (!lbl || lbl_len < 4) {
        return;
    }
    lbl[0] = 0;
    // Prefer enroute fixes over airports so the path isn't all "Kxxx".
    if (net_nearest_waypoint(lat, lon, 35.0f, lbl, lbl_len) && lbl[0]) {
        return;
    }
    if (fa_nearest_airport_name(lbl, lbl_len, lat, lon, 12.0f) && lbl[0]) {
        return;
    }
    copy_trunc(lbl, lbl_len, "WPT");
}

/**
 * Build follow waypoints: FA path polyline → many points, each named from the
 * flash FAA NASR fix DB. Filed IFR tokens (GLAZR, VOSTK, …) are force-included
 * when resolvable; airways (Q118) are skipped (geometry comes from the polyline).
 */
static int fa_build_route_wpts(FlightInfo *fi, const char *route, const float *wlats, const float *wlons,
                               int wn) {
    if (!fi) {
        return 0;
    }
    fi->route_wpt_n = 0;
    memset(fi->route_wpts, 0, sizeof(fi->route_wpts));

    auto push_origin = [&]() {
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
    };
    auto push_dest = [&]() {
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
    };

    push_origin();

    // --- Primary: dense FlightAware path, named via FAA fix DB ---
    // Walk the polyline; emit a pin when the nearest FAA fix changes or we've
    // traveled far enough — yields many named enroute points without an airway DB.
    if (wn >= 3) {
        char prev_lbl[14] = {0};
        float prev_lat = 0, prev_lon = 0;
        bool have_prev = false;
        for (int i = 1; i < wn - 1 && fi->route_wpt_n < FR_MAX_ROUTE_WPTS - 1; i++) {
            char lbl[14];
            fa_label_path_point(wlats[i], wlons[i], lbl, sizeof(lbl));
            float flat = wlats[i], flon = wlons[i];
            float dlat = 0, dlon = 0;
            if (lbl[0] && strcmp(lbl, "WPT") != 0 && net_lookup_waypoint(lbl, &dlat, &dlon)) {
                if (wpt_dist_proxy_nm(wlats[i], wlons[i], dlat, dlon) < 20.0f) {
                    flat = dlat;
                    flon = dlon;
                }
            }
            const bool same_fix = have_prev && lbl[0] && prev_lbl[0] && strcasecmp(lbl, prev_lbl) == 0;
            const float since = have_prev ? wpt_dist_proxy_nm(prev_lat, prev_lon, flat, flon) : 999.0f;
            // New fix name, or same fix but we've moved ≥25 nm along track.
            if (!have_prev || !same_fix || since >= 25.0f) {
                fa_push_route_wpt(fi, lbl, flat, flon);
                copy_trunc(prev_lbl, sizeof(prev_lbl), lbl);
                prev_lat = flat;
                prev_lon = flon;
                have_prev = true;
            }
        }
    } else if (route && route[0]) {
        // No polyline — resolve filed tokens against FAA DB / airports.
        char buf[384];
        copy_trunc(buf, sizeof(buf), route);
        char *save = nullptr;
        for (char *tok = strtok_r(buf, " ", &save); tok; tok = strtok_r(nullptr, " ", &save)) {
            if (!tok[0] || strcasecmp(tok, "DCT") == 0) {
                continue;
            }
            char *slash = strchr(tok, '/');
            if (slash && fa_is_speed_alt_token(slash + 1)) {
                *slash = 0;
            }
            if (!tok[0] || fa_is_speed_alt_token(tok) || fa_is_airway(tok)) {
                continue;
            }
            char label[14];
            copy_trunc(label, sizeof(label), tok);
            for (char *q = label; *q; q++) {
                *q = (char)toupper((unsigned char)*q);
            }
            float lat = 0, lon = 0;
            char base[12];
            if (decode_icao_ll_fix(label, &lat, &lon)) {
                fa_push_route_wpt(fi, label, lat, lon);
            } else if (fa_resolve_named_fix(label, &lat, &lon)) {
                if (fa_sid_star_base(label, base, sizeof(base)) && net_lookup_waypoint(base, &lat, &lon)) {
                    fa_push_route_wpt(fi, base, lat, lon);
                } else {
                    fa_push_route_wpt(fi, label, lat, lon);
                }
            }
        }
    }

    // --- Upgrade / inject filed IFR fix names onto the path ---
    if (route && route[0] && wn >= 2) {
        char buf[384];
        copy_trunc(buf, sizeof(buf), route);
        char *save = nullptr;
        for (char *tok = strtok_r(buf, " ", &save); tok; tok = strtok_r(nullptr, " ", &save)) {
            if (!tok[0] || strcasecmp(tok, "DCT") == 0 || fa_is_airway(tok) || fa_is_speed_alt_token(tok)) {
                continue;
            }
            char *slash = strchr(tok, '/');
            if (slash && fa_is_speed_alt_token(slash + 1)) {
                *slash = 0;
            }
            char label[14];
            copy_trunc(label, sizeof(label), tok);
            for (char *q = label; *q; q++) {
                *q = (char)toupper((unsigned char)*q);
            }
            float lat = 0, lon = 0;
            char use_name[14];
            copy_trunc(use_name, sizeof(use_name), label);
            char base[12];
            if (fa_sid_star_base(label, base, sizeof(base)) && net_lookup_waypoint(base, &lat, &lon)) {
                copy_trunc(use_name, sizeof(use_name), base);
            } else if (!fa_resolve_named_fix(label, &lat, &lon)) {
                continue;
            }
            // If already near an existing path point, just rename it; else insert.
            int best_i = -1;
            float best_d = 1e30f;
            for (int i = 0; i < fi->route_wpt_n; i++) {
                float dlat = fi->route_wpts[i].lat - lat;
                float dlon = fi->route_wpts[i].lon - lon;
                float d = dlat * dlat + dlon * dlon;
                if (d < best_d) {
                    best_d = d;
                    best_i = i;
                }
            }
            if (best_i >= 0 && best_d < (0.35f * 0.35f)) {  // ~20 nm @ mid-lat-ish
                copy_trunc(fi->route_wpts[best_i].name, sizeof(fi->route_wpts[best_i].name), use_name);
                fi->route_wpts[best_i].lat = lat;
                fi->route_wpts[best_i].lon = lon;
            } else if (fi->route_wpt_n < FR_MAX_ROUTE_WPTS) {
                fa_push_route_wpt(fi, use_name, lat, lon);
            }
        }
    }

    push_dest();
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
    // Grow to 768 KB only for this fetch (route/waypoints sit ~450 KB into the page).
    // Shrink afterward so ADS-B / hexdb TLS keep contiguous int-dma.
    if (!ensure_http_buf_sized(kHttpBufCapFa)) {
        Serial.printf("FlightAware buf fail: %s\n", g_last_err);
        return false;
    }
    if (!https_get(url, nullptr, kUaBrowser, 10000, 90000, "text/html,application/xhtml+xml,*/*", true)) {
        Serial.printf("FlightAware fetch fail: %s\n", g_last_err);
        http_buf_shrink_default();
        return false;
    }
    size_t blen = 0;
    const char *html = http_body_buf(&blen);
    if (!html || blen < 800) {
        Serial.println("FlightAware: empty body");
        http_buf_shrink_default();
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

    // Prefer the airborne activityLog leg over \"operated\" (often tomorrow's schedule).
    char act_origin[8] = {0};
    char act_dest[8] = {0};
    char hist_path[160] = {0};
    const bool have_act =
        fa_pick_current_activity(html, act_origin, sizeof(act_origin), act_dest, sizeof(act_dest),
                                 hist_path, sizeof(hist_path));
    if (have_act) {
        const bool od_changed =
            !fi->origin[0] || !fi->dest[0] || strcasecmp(fi->origin, act_origin) != 0 ||
            strcasecmp(fi->dest, act_dest) != 0;
        if (od_changed) {
            // Drop times/gates/names from a wrong (future or prior) FlightStats leg.
            fi->dep_time[0] = fi->dep_act[0] = fi->dep_est[0] = 0;
            fi->arr_time[0] = fi->arr_act[0] = fi->arr_est[0] = 0;
            fi->dep_gate[0] = fi->arr_gate[0] = 0;
            fi->dep_terminal[0] = fi->arr_terminal[0] = 0;
            fi->dep_act_epoch = fi->arr_act_epoch = fi->arr_est_epoch = 0;
            fi->origin_name[0] = fi->dest_name[0] = 0;
        }
        // Overwrite schedule OD when FA has a clearer current/airborne turn.
        copy_trunc(fi->origin, sizeof(fi->origin), act_origin);
        copy_trunc(fi->dest, sizeof(fi->dest), act_dest);
        fi->departed = true;
        fi->schedule_ok = true;
        fi->route_ok = true;
        if (!fi->status[0] || strcasecmp(fi->status, "Scheduled") == 0) {
            copy_trunc(fi->status, sizeof(fi->status), "En Route");
        }
        if (!fi->origin_name[0] || !fi->dest_name[0]) {
            net_fetch_airport_names(fi->origin, fi->dest, fi->origin_name, sizeof(fi->origin_name),
                                    fi->dest_name, sizeof(fi->dest_name));
        }
    }

    char route[384] = {0};
    fa_extract_route_string(html, route, sizeof(route));

    constexpr int kFaPolyMax = 128;
    float wlats[kFaPolyMax];
    float wlons[kFaPolyMax];
    int wn = fa_parse_waypoints_array(html, wlats, wlons, kFaPolyMax);
    Serial.printf("FlightAware route_len=%u waypoints=%d\n", (unsigned)strlen(route), wn);

    // Live/future pages often have empty flightPlan.route — pull filed string + path
    // from the current activity history URL (not \"operated\", which may be future).
    if (!hist_path[0]) {
        const char *op = strstr(html, "\"operated\":\"/live/flight/");
        if (op) {
            op += strlen("\"operated\":\"");
            size_t pi = 0;
            while (op[pi] && op[pi] != '"' && pi + 1 < sizeof(hist_path)) {
                hist_path[pi] = op[pi];
                pi++;
            }
            hist_path[pi] = 0;
        }
    }
    if (hist_path[0] == '/') {
        char hist_url[192];
        snprintf(hist_url, sizeof(hist_url), "https://www.flightaware.com%s", hist_path);
        Serial.printf("FlightAware history %s\n", hist_path);
        status_msg("FlightAware filed route...");
        if (https_get(hist_url, nullptr, kUaBrowser, 10000, 90000, "text/html,application/xhtml+xml,*/*",
                      true)) {
            size_t hlen = 0;
            const char *hhtml = http_body_buf(&hlen);
            if (hhtml && hlen > 800) {
                char hist_route[384] = {0};
                fa_extract_route_string(hhtml, hist_route, sizeof(hist_route));
                if (hist_route[0]) {
                    copy_trunc(route, sizeof(route), hist_route);
                }
                const int hwn = fa_parse_waypoints_array(hhtml, wlats, wlons, kFaPolyMax);
                if (hwn > 0) {
                    wn = hwn;  // history page path is for this leg
                }
                // Prefer history-page track when live page had few/no samples.
                if (track_out && track_max > 0 && (!track_n || *track_n < 8)) {
                    const int tn = fa_parse_track_array(hhtml, track_out, track_max);
                    if (track_n) {
                        *track_n = tn;
                    }
                    Serial.printf("FlightAware history track pts=%d\n", tn);
                }
                Serial.printf("FlightAware history route_len=%u waypoints=%d\n", (unsigned)strlen(route),
                              wn);
            }
        }
    }

    // Drop the oversized FA buffer before route decode / next ADS-B poll.
    html = nullptr;
    http_buf_shrink_default();

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

// ---- Airport board (live ADS-B + hexdb routes; replaces FA HTML scrape) ----

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

// Day-of-week 0=Sun … 6=Sat (Sakamoto).
static int tz_civil_dow(int y, int m, int d) {
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) {
        y -= 1;
    }
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static int tz_nth_weekday_of_month(int y, int m, int weekday, int n) {
    const int first = tz_civil_dow(y, m, 1);
    return 1 + (weekday - first + 7) % 7 + (n - 1) * 7;
}

static int tz_last_weekday_of_month(int y, int m, int weekday) {
    const int dim[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int last = dim[m - 1];
    if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))) {
        last = 29;
    }
    const int dow = tz_civil_dow(y, m, last);
    return last - (dow - weekday + 7) % 7;
}

static bool tz_us_dst_active(time_t utc, int std_off_sec) {
    // US: 2nd Sun Mar 02:00 → 1st Sun Nov 01:00 (local standard time).
    time_t local = utc + (time_t)std_off_sec;
    struct tm t {};
    gmtime_r(&local, &t);
    const int y = t.tm_year + 1900;
    const int start_d = tz_nth_weekday_of_month(y, 3, 0, 2);
    const int end_d = tz_nth_weekday_of_month(y, 11, 0, 1);
    const int now_k = (t.tm_mon + 1) * 1000000 + t.tm_mday * 10000 + t.tm_hour * 100 + t.tm_min;
    const int start_k = 3 * 1000000 + start_d * 10000 + 2 * 100;
    const int end_k = 11 * 1000000 + end_d * 10000 + 1 * 100;
    return now_k >= start_k && now_k < end_k;
}

static bool tz_eu_dst_active(time_t utc, int std_off_sec) {
    // EU: last Sun Mar 01:00 UTC → last Sun Oct 01:00 UTC (approx via local std).
    time_t local = utc + (time_t)std_off_sec;
    struct tm t {};
    gmtime_r(&local, &t);
    const int y = t.tm_year + 1900;
    const int start_d = tz_last_weekday_of_month(y, 3, 0);
    const int end_d = tz_last_weekday_of_month(y, 10, 0);
    const int now_k = (t.tm_mon + 1) * 1000000 + t.tm_mday * 10000 + t.tm_hour * 100 + t.tm_min;
    const int start_k = 3 * 1000000 + start_d * 10000 + 1 * 100;
    const int end_k = 10 * 1000000 + end_d * 10000 + 1 * 100;
    return now_k >= start_k && now_k < end_k;
}

int net_tz_offset_for_ll(float lat, float lon, time_t when_utc) {
    int off = (int)lroundf(lon / 15.0f) * 3600;
    if (off < -12 * 3600) {
        off = -12 * 3600;
    }
    if (off > 14 * 3600) {
        off = 14 * 3600;
    }

    time_t utc = when_utc;
    if (utc <= 0) {
        utc = time(nullptr);
    }
    if (utc > 1700000000) {
        // US/Canada DST (excl. most of AZ / HI).
        const bool namerica = (lat >= 24.0f && lat <= 60.0f && lon <= -52.0f && lon >= -170.0f);
        const bool arizona = (lat >= 31.0f && lat <= 37.5f && lon <= -109.0f && lon >= -115.0f);
        const bool hawaii = (lat >= 18.0f && lat <= 23.0f && lon <= -154.0f && lon >= -161.0f);
        if (namerica && !arizona && !hawaii) {
            if (tz_us_dst_active(utc, off)) {
                off += 3600;
            }
        } else if (lat >= 35.0f && lat <= 72.0f && lon >= -10.0f && lon <= 40.0f) {
            if (tz_eu_dst_active(utc, off)) {
                off += 3600;
            }
        }
    }
    return off;
}

// ---- FlightAware airport board HTML parse ----

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
    row->progress_pct = (int)((el * 100u) / span);
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
                // "Fort Lauderdale Intl (Fort Lauderdale, FL) - FLL" → city in parens.
                char *lp = strchr(raw, '(');
                char *rp = lp ? strchr(lp, ')') : nullptr;
                if (lp && rp && rp > lp + 1) {
                    *rp = 0;
                    copy_trunc(row.other_name, sizeof(row.other_name), lp + 1);
                    trim_inplace(row.other_name);
                } else {
                    char *dash = strstr(raw, " - ");
                    if (dash) {
                        *dash = 0;
                    }
                    copy_trunc(row.other_name, sizeof(row.other_name), raw);
                }
            }
        }
        // Live progress bar: style="width: N%"
        bool have_bar = false;
        const char *pf = strstr(rowbuf, "track-panel-progress-fill");
        if (pf) {
            const char *w = strstr(pf, "width:");
            if (w) {
                int pct = atoi(w + 6);
                if (pct < 0) {
                    pct = 0;
                }
                if (pct > 100) {
                    pct = 100;
                }
                row.progress_pct = pct;
                have_bar = true;
            }
        }
        if (!row.other_apt[0]) {
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
            if (!have_bar) {
                fa_board_fill_progress(&row, landed_bias);
            }
            out[n++] = row;
        }
        p = tr_end + 5;
    }
    return n;
}

static bool fa_board_body_ok(const char *buf, size_t len) {
    // FA order on live/airport/{ICAO}:
    //   arrivals → departures (~70–90 KB) → enroute (~100–120 KB) → scheduled (~125 KB+)
    // We need enroute + departures. Stopping at arrivals-board (~50–70 KB) left both
    // parsers empty ("FA board: no flights"). scheduled-board means enroute is complete.
    if (!buf || len < 80 * 1024) {
        return false;
    }
    if (strstr(buf, "id=\"scheduled-board\"") != nullptr) {
        return true;
    }
    const char *en = strstr(buf, "id=\"enroute-board\"");
    const char *dep = strstr(buf, "id=\"departures-board\"");
    // Enough bytes after enroute marker for a full inbound table (~20 rows).
    return en && dep && (size_t)(buf + len - en) > 12 * 1024;
}

static void board_resolve_ll(const char *code, float *lat, float *lon) {
    if (!lat || !lon || !code || !code[0]) {
        return;
    }
    *lat = 0;
    *lon = 0;
    if (net_lookup_airport(code, lat, lon, nullptr, nullptr)) {
        return;
    }
    char resolved[8] = {0};
    if (net_lookup_airport_iata(code, resolved, sizeof(resolved), lat, lon) &&
        (*lat != 0.0f || *lon != 0.0f)) {
        return;
    }
    JsonDocument doc;
    if (awc_airports(code, doc) && doc.as<JsonArray>().size() > 0) {
        *lat = doc[0]["lat"] | 0.0f;
        *lon = doc[0]["lon"] | 0.0f;
        return;
    }
    if (strlen(code) == 3) {
        char kcode[8];
        snprintf(kcode, sizeof(kcode), "K%s", code);
        if (awc_airports(kcode, doc) && doc.as<JsonArray>().size() > 0) {
            *lat = doc[0]["lat"] | 0.0f;
            *lon = doc[0]["lon"] | 0.0f;
        }
    }
}

static void board_normalize_icao(const char *icao, char *code, size_t code_len) {
    copy_trunc(code, code_len, icao ? icao : "");
    for (char *q = code; *q; q++) {
        *q = (char)toupper((unsigned char)*q);
    }
    if (strlen(code) == 3) {
        char icao4[8] = {0};
        if (net_lookup_airport_iata(code, icao4, sizeof(icao4), nullptr, nullptr) && icao4[0]) {
            copy_trunc(code, code_len, icao4);
        } else {
            char kcode[8];
            snprintf(kcode, sizeof(kcode), "K%s", code);
            copy_trunc(code, code_len, kcode);
        }
    }
}

/** Local HH:MM at apt for UTC + hours offset (ETA helper). */
static void board_fmt_local_hm(char *out, size_t out_len, float apt_lat, float apt_lon, float hours_ahead) {
    if (!out || out_len < 6) {
        return;
    }
    out[0] = 0;
    time_t now = time(nullptr);
    if (now < 1700000000) {
        return;
    }
    if (hours_ahead < 0.0f) {
        hours_ahead = 0.0f;
    }
    if (hours_ahead > 48.0f) {
        hours_ahead = 48.0f;
    }
    time_t when = now + (time_t)lroundf(hours_ahead * 3600.0f);
    when += (time_t)net_tz_offset_for_ll(apt_lat, apt_lon, when);
    struct tm t {};
    gmtime_r(&when, &t);
    snprintf(out, out_len, "%02d:%02d", t.tm_hour, t.tm_min);
}

static void board_apply_other_city(AirportBoardRow *rows, int n) {
    if (!rows || n <= 0) {
        return;
    }
    char icaos[FR_MAX_BOARD_ROWS][8];
    char cities[FR_MAX_BOARD_ROWS][28];
    int nu = 0;
    for (int i = 0; i < n && nu < FR_MAX_BOARD_ROWS; i++) {
        if (!rows[i].other_apt[0] || rows[i].other_name[0]) {
            continue;
        }
        bool seen = false;
        for (int j = 0; j < nu; j++) {
            if (strcasecmp(icaos[j], rows[i].other_apt) == 0) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            copy_trunc(icaos[nu], sizeof(icaos[nu]), rows[i].other_apt);
            cities[nu][0] = 0;
            nu++;
        }
    }
    if (nu <= 0) {
        return;
    }
    status_msg("Board cities...");
    net_fetch_airport_cities(icaos, cities, nu);
    for (int i = 0; i < n; i++) {
        if (!rows[i].other_apt[0] || rows[i].other_name[0]) {
            continue;
        }
        for (int j = 0; j < nu; j++) {
            if (strcasecmp(icaos[j], rows[i].other_apt) == 0 && cities[j][0]) {
                copy_trunc(rows[i].other_name, sizeof(rows[i].other_name), cities[j]);
                break;
            }
        }
        if (!rows[i].other_name[0]) {
            // Fallback: show the airport code rather than blank / distance.
            copy_trunc(rows[i].other_name, sizeof(rows[i].other_name), rows[i].other_apt);
        }
    }
}

static void board_enrich_row_route_times(AirportBoardRow *row, bool inbound, const char *board_icao,
                                         float apt_lat, float apt_lon, const Aircraft &ac, float dst_nm) {
    if (!row || !row->flight[0]) {
        return;
    }

    char origin[8] = {0}, dest[8] = {0};
    if (net_fetch_route(row->flight, origin, sizeof(origin), dest, sizeof(dest))) {
        const char *other = nullptr;
        if (inbound) {
            if (apt_codes_match(dest, board_icao)) {
                other = origin;
            } else if (apt_codes_match(origin, board_icao)) {
                other = dest;
            } else {
                other = origin;
            }
        } else {
            if (apt_codes_match(origin, board_icao)) {
                other = dest;
            } else if (apt_codes_match(dest, board_icao)) {
                other = origin;
            } else {
                other = dest;
            }
        }
        if (other && other[0]) {
            copy_trunc(row->other_apt, sizeof(row->other_apt), other);
        }
    }

    // FlightStats: scheduled / estimated dep + arr (and city names).
    FlightInfo fi{};
    if (net_fetch_flight_schedule(row->flight, &fi) && fi.schedule_ok) {
        if (!row->other_apt[0]) {
            const char *other = inbound ? fi.origin : fi.dest;
            if (other && other[0]) {
                copy_trunc(row->other_apt, sizeof(row->other_apt), other);
            }
        }
        if (!row->other_name[0]) {
            const char *nm = inbound ? fi.origin_name : fi.dest_name;
            if (nm && nm[0]) {
                city_from_facility(nm, row->other_name, sizeof(row->other_name));
            }
        }
        copy_trunc(row->dep_time, sizeof(row->dep_time), fi.dep_time);
        // Prefer actual wheels-up when airborne; else estimated; else scheduled (already in dep_time).
        if (fi.dep_act[0]) {
            copy_trunc(row->dep_est, sizeof(row->dep_est), fi.dep_act);
        } else {
            copy_trunc(row->dep_est, sizeof(row->dep_est), fi.dep_est);
        }
        copy_trunc(row->arr_time, sizeof(row->arr_time), fi.arr_time);
        copy_trunc(row->arr_est, sizeof(row->arr_est), fi.arr_est);
        if (fi.dep_act_epoch) {
            row->dep_epoch = fi.dep_act_epoch;
        }
        if (fi.arr_est_epoch) {
            row->arr_epoch = fi.arr_est_epoch;
        } else if (fi.arr_act_epoch) {
            row->arr_epoch = fi.arr_act_epoch;
        }
    }

    // ETA fallback when schedule missing (still better than altitude / GS placeholders).
    if (!row->arr_est[0] && !row->arr_time[0] && ac.gs_kts > 40.0f) {
        float rem_nm = dst_nm;
        if (!inbound && row->other_apt[0]) {
            float olat = 0, olon = 0;
            if (net_lookup_airport(row->other_apt, &olat, &olon, nullptr) ||
                net_lookup_airport_iata(row->other_apt, nullptr, 0, &olat, &olon)) {
                float east = 0, north = 0;
                ll_to_nm(ac.lat, ac.lon, olat, olon, &east, &north);
                rem_nm = sqrtf(east * east + north * north);
            }
        }
        board_fmt_local_hm(row->arr_est, sizeof(row->arr_est), apt_lat, apt_lon, rem_nm / ac.gs_kts);
    }
}

static bool board_fill_from_aircraft(const char *code, float apt_lat, float apt_lon, const Aircraft *acs,
                                     int n_ac, AirportBoardRow *arr, int arr_max, int *arr_n,
                                     AirportBoardRow *dep, int dep_max, int *dep_n) {
    if (arr_n) {
        *arr_n = 0;
    }
    if (dep_n) {
        *dep_n = 0;
    }
    if (!acs || n_ac <= 0 || !arr || !dep) {
        return false;
    }

    auto ang_diff = [](float a, float b) -> float {
        float d = a - b;
        while (d > 180.0f) {
            d -= 360.0f;
        }
        while (d < -180.0f) {
            d += 360.0f;
        }
        return fabsf(d);
    };
    auto fill_row = [&](AirportBoardRow *row, const Aircraft &ac, float dst_nm, bool inbound) {
        memset(row, 0, sizeof(*row));
        copy_trunc(row->flight, sizeof(row->flight), ac.flight[0] ? ac.flight : ac.reg);
        for (char *p = row->flight; *p; p++) {
            if (*p == ' ') {
                *p = 0;
                break;
            }
        }
        copy_trunc(row->type, sizeof(row->type), ac.type);
        // TO/FROM + DEP/ARR filled by enrichment (city + schedule). Do not stuff nm/alt/GS.
        int pct = inbound ? (int)lroundf(100.0f - dst_nm * 2.5f) : (int)lroundf(dst_nm * 2.5f);
        if (pct < 0) {
            pct = 0;
        }
        if (pct > 100) {
            pct = 100;
        }
        row->progress_pct = pct;
        row->valid = row->flight[0] != 0;
        (void)dst_nm;
    };

    struct Cand {
        int idx;
        float dst;
        bool inbound;
    };
    Cand cands[96];
    int nc = 0;
    for (int i = 0; i < n_ac && nc < 96; i++) {
        if (!acs[i].valid || (!acs[i].flight[0] && !acs[i].reg[0])) {
            continue;
        }
        // Board is airborne-only — skip taxi / parked / ground returns.
        if (acs[i].on_ground || acs[i].alt_ft < 200.0f) {
            continue;
        }
        float east = 0, north = 0;
        ll_to_nm(acs[i].lat, acs[i].lon, apt_lat, apt_lon, &east, &north);
        const float dst = sqrtf(east * east + north * north);
        if (dst > 40.0f) {
            continue;
        }
        const float brg = atan2f(east, north) * 57.2957795f;
        const float dhdg = ang_diff(acs[i].track_deg, brg);
        bool inbound = false;
        if (acs[i].vrate_fpm < -200.0f || dhdg < 70.0f) {
            inbound = true;
        } else if (acs[i].vrate_fpm > 200.0f || dhdg > 110.0f) {
            inbound = false;
        } else {
            inbound = dhdg < 90.0f;
        }
        cands[nc++] = Cand{i, dst, inbound};
    }
    for (int i = 0; i + 1 < nc; i++) {
        for (int j = i + 1; j < nc; j++) {
            if (cands[j].dst < cands[i].dst) {
                Cand t = cands[i];
                cands[i] = cands[j];
                cands[j] = t;
            }
        }
    }

    int na = 0, nd = 0;
    // Remember aircraft index + distance for enrichment (ETA / route).
    int arr_ac_idx[FR_MAX_BOARD_ROWS];
    float arr_dst[FR_MAX_BOARD_ROWS];
    int dep_ac_idx[FR_MAX_BOARD_ROWS];
    float dep_dst[FR_MAX_BOARD_ROWS];
    for (int k = 0; k < nc; k++) {
        const Aircraft &ac = acs[cands[k].idx];
        if (cands[k].inbound) {
            if (na < arr_max && na < FR_MAX_BOARD_ROWS) {
                fill_row(&arr[na], ac, cands[k].dst, true);
                if (arr[na].valid) {
                    arr_ac_idx[na] = cands[k].idx;
                    arr_dst[na] = cands[k].dst;
                    na++;
                }
            }
        } else if (nd < dep_max && nd < FR_MAX_BOARD_ROWS) {
            fill_row(&dep[nd], ac, cands[k].dst, false);
            if (dep[nd].valid) {
                dep_ac_idx[nd] = cands[k].idx;
                dep_dst[nd] = cands[k].dst;
                nd++;
            }
        }
    }

    // Cap schedule lookups so a busy hub doesn't thrash TLS for minutes.
    static constexpr int kBoardSchedMax = 8;
    const int n_sched_arr = na < kBoardSchedMax ? na : kBoardSchedMax;
    const int n_sched_dep = nd < kBoardSchedMax ? nd : kBoardSchedMax;
    for (int i = 0; i < n_sched_arr; i++) {
        char st[40];
        snprintf(st, sizeof(st), "Board arr %d/%d…", i + 1, n_sched_arr);
        status_msg(st);
        board_enrich_row_route_times(&arr[i], true, code, apt_lat, apt_lon, acs[arr_ac_idx[i]], arr_dst[i]);
    }
    for (int i = 0; i < n_sched_dep; i++) {
        char st[40];
        snprintf(st, sizeof(st), "Board dep %d/%d…", i + 1, n_sched_dep);
        status_msg(st);
        board_enrich_row_route_times(&dep[i], false, code, apt_lat, apt_lon, acs[dep_ac_idx[i]], dep_dst[i]);
    }
    // Remaining rows: hexdb route only (city batch covers names).
    for (int i = n_sched_arr; i < na; i++) {
        char origin[8] = {0}, dest[8] = {0};
        if (net_fetch_route(arr[i].flight, origin, sizeof(origin), dest, sizeof(dest))) {
            const char *other = nullptr;
            if (apt_codes_match(dest, code)) {
                other = origin;
            } else if (apt_codes_match(origin, code)) {
                other = dest;
            } else if (origin[0]) {
                other = origin;
            }
            if (other && other[0]) {
                copy_trunc(arr[i].other_apt, sizeof(arr[i].other_apt), other);
            }
        }
        if (!arr[i].arr_est[0] && acs[arr_ac_idx[i]].gs_kts > 40.0f) {
            board_fmt_local_hm(arr[i].arr_est, sizeof(arr[i].arr_est), apt_lat, apt_lon,
                               arr_dst[i] / acs[arr_ac_idx[i]].gs_kts);
        }
    }
    for (int i = n_sched_dep; i < nd; i++) {
        char origin[8] = {0}, dest[8] = {0};
        if (net_fetch_route(dep[i].flight, origin, sizeof(origin), dest, sizeof(dest))) {
            const char *other = nullptr;
            if (apt_codes_match(origin, code)) {
                other = dest;
            } else if (apt_codes_match(dest, code)) {
                other = origin;
            } else if (dest[0]) {
                other = dest;
            }
            if (other && other[0]) {
                copy_trunc(dep[i].other_apt, sizeof(dep[i].other_apt), other);
            }
        }
    }

    board_apply_other_city(arr, na);
    board_apply_other_city(dep, nd);

    if (arr_n) {
        *arr_n = na;
    }
    if (dep_n) {
        *dep_n = nd;
    }
    Serial.printf("ADS-B board %s: %d arr / %d dep (from %d airborne near)\n", code, na, nd, nc);
    return na > 0 || nd > 0;
}

bool net_build_airport_board_from_traffic(const char *icao, const Aircraft *ac, int ac_n,
                                          AirportBoardRow *arr, int arr_max, int *arr_n,
                                          AirportBoardRow *dep, int dep_max, int *dep_n, char *label_out,
                                          size_t label_len, float *out_lat, float *out_lon) {
    if (arr_n) {
        *arr_n = 0;
    }
    if (dep_n) {
        *dep_n = 0;
    }
    if (label_out && label_len) {
        label_out[0] = 0;
    }
    if (out_lat) {
        *out_lat = 0;
    }
    if (out_lon) {
        *out_lon = 0;
    }
    if (!icao || !icao[0] || !arr || !dep || arr_max <= 0 || dep_max <= 0) {
        set_err("Bad airport board args");
        return false;
    }
    char code[8];
    board_normalize_icao(icao, code, sizeof(code));
    float lat = 0, lon = 0;
    board_resolve_ll(code, &lat, &lon);
    if (fabsf(lat) + fabsf(lon) < 0.01f) {
        set_err("Airport not in DB");
        return false;
    }
    if (out_lat) {
        *out_lat = lat;
    }
    if (out_lon) {
        *out_lon = lon;
    }
    if (label_out && label_len) {
        snprintf(label_out, label_len, "%s (live)", code);
    }
    if (!board_fill_from_aircraft(code, lat, lon, ac, ac_n, arr, arr_max, arr_n, dep, dep_max, dep_n)) {
        set_err("No traffic near airport");
        return false;
    }
    return true;
}

bool net_fetch_airport_board(const char *icao, AirportBoardRow *arr, int arr_max, int *arr_n,
                             AirportBoardRow *dep, int dep_max, int *dep_n, char *label_out,
                             size_t label_len, float *out_lat, float *out_lon) {
    static char s_cache_icao[8] = {0};
    static char s_cache_label[72] = {0};
    static AirportBoardRow s_cache_arr[FR_MAX_BOARD_ROWS];
    static AirportBoardRow s_cache_dep[FR_MAX_BOARD_ROWS];
    static int s_cache_an = 0;
    static int s_cache_dn = 0;
    static float s_cache_lat = 0, s_cache_lon = 0;
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
    if (out_lat) {
        *out_lat = 0;
    }
    if (out_lon) {
        *out_lon = 0;
    }
    if (!icao || !icao[0] || !arr || !dep || arr_max <= 0 || dep_max <= 0) {
        set_err("Bad airport board args");
        return false;
    }

    char code[8];
    board_normalize_icao(icao, code, sizeof(code));

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
        if (out_lat) {
            *out_lat = s_cache_lat;
        }
        if (out_lon) {
            *out_lon = s_cache_lon;
        }
        Serial.printf("Board cache hit %s (%d arr / %d dep)\n", code, na, nd);
        status_msg("Airport board (cached)");
        return true;
    }

    float lat = 0, lon = 0;
    board_resolve_ll(code, &lat, &lon);

    // One page: https://www.flightaware.com/live/airport/KATL
    // Arrivals column ← En Route (airborne inbound; Arrivals table is already landed).
    // Departures column ← Departures (left gate; Scheduled are still on the ground).
    // Settle TLS after FA flight-plan (768 KB) thrash; ensure body buffer is alive.
    net_tls_reset();
    delay(400);
    if (!ensure_http_buf()) {
        set_err(g_last_err[0] ? g_last_err : "No HTTP buffer");
        return false;
    }
    status_msg("FlightAware board...");
    char url[96];
    snprintf(url, sizeof(url), "https://www.flightaware.com/live/airport/%s", code);
    if (!https_get(url, nullptr, kUaBrowser, 10000, 90000, "text/html,application/xhtml+xml,*/*", true,
                   fa_board_body_ok)) {
        Serial.printf("FA board fetch fail %s: %s\n", code, g_last_err);
        set_err(g_last_err[0] ? g_last_err : "FA board fetch failed");
        return false;
    }
    size_t blen = 0;
    const char *html = http_body_buf(&blen);
    if (!html || blen < 800) {
        set_err("FA board empty");
        return false;
    }
    Serial.printf("FA board %s body %u KB\n", code, (unsigned)(blen / 1024));

    // En Route = airborne inbound; Departures = left gate.
    int na = fa_parse_airport_board_section(html, "enroute-board", arr, arr_max, false);
    int nd = fa_parse_airport_board_section(html, "departures-board", dep, dep_max, false);
    Serial.printf("FA board parse enroute=%d departures=%d (html has en=%d dep=%d sch=%d)\n", na, nd,
                  strstr(html, "id=\"enroute-board\"") ? 1 : 0,
                  strstr(html, "id=\"departures-board\"") ? 1 : 0,
                  strstr(html, "id=\"scheduled-board\"") ? 1 : 0);
    // Drop completed legs (progress bar at 100%).
    auto drop_done = [](AirportBoardRow *rows, int n) -> int {
        int w = 0;
        for (int i = 0; i < n; i++) {
            if (rows[i].progress_pct >= 100) {
                continue;
            }
            if (w != i) {
                rows[w] = rows[i];
            }
            w++;
        }
        return w;
    };
    na = drop_done(arr, na);
    nd = drop_done(dep, nd);

    if (na <= 0 && nd <= 0) {
        set_err("FA board: no flights");
        return false;
    }
    if (arr_n) {
        *arr_n = na;
    }
    if (dep_n) {
        *dep_n = nd;
    }
    if (label_out && label_len) {
        snprintf(label_out, label_len, "%s (FA)", code);
    }
    if (out_lat) {
        *out_lat = lat;
    }
    if (out_lon) {
        *out_lon = lon;
    }

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
    s_cache_lat = lat;
    s_cache_lon = lon;
    s_cache_ms = millis();
    Serial.printf("FA board %s: %d arr / %d dep\n", code, na, nd);
    // Large FA HTML thrash Hosted Wi-Fi DNS/TLS; settle before the next poll/track.
    net_tls_reset();
    delay(750);
    return true;
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
