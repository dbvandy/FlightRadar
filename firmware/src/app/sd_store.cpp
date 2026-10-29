#include "sd_store.h"

#include <Arduino.h>
#include <ctype.h>
#include <dirent.h>
#include <driver/sdmmc_host.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "board_config.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Recursive lock shared by UI / net / plate-DB SD access.
static SemaphoreHandle_t s_sd_mutex = nullptr;
static portMUX_TYPE s_sd_mutex_init_mux = portMUX_INITIALIZER_UNLOCKED;

void sd_lock(void) {
    if (!s_sd_mutex) {
        SemaphoreHandle_t m = xSemaphoreCreateRecursiveMutex();
        portENTER_CRITICAL(&s_sd_mutex_init_mux);
        if (!s_sd_mutex) {
            s_sd_mutex = m;
            m = nullptr;
        }
        portEXIT_CRITICAL(&s_sd_mutex_init_mux);
        if (m) {
            vSemaphoreDelete(m);  // lost the init race
        }
    }
    if (s_sd_mutex) {
        xSemaphoreTakeRecursive(s_sd_mutex, portMAX_DELAY);
    }
}

void sd_unlock(void) {
    if (s_sd_mutex) {
        xSemaphoreGiveRecursive(s_sd_mutex);
    }
}

static sdmmc_card_t *s_card = nullptr;
static bool s_mounted = false;
static char s_status[96] = "SD: not mounted";

const char *sd_status_text(void) {
    return s_status;
}

bool sd_is_mounted(void) {
    return s_mounted && s_card != nullptr;
}

void sd_unmount(void) {
    if (s_mounted && s_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    }
    s_card = nullptr;
    s_mounted = false;
    strncpy(s_status, "SD: not mounted", sizeof(s_status) - 1);
}

bool sd_mount(void) {
    if (s_mounted && s_card) {
        strncpy(s_status, "SD: ready", sizeof(s_status) - 1);
        return true;
    }
    sd_unmount();

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = 10000;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = (gpio_num_t)SD_GPIO_MMC_CLK;
    slot_config.cmd = (gpio_num_t)SD_GPIO_MMC_CMD;
    slot_config.d0 = (gpio_num_t)SD_GPIO_MMC_D0;
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err =
        esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (err != ESP_OK) {
        s_card = nullptr;
        s_mounted = false;
        snprintf(s_status, sizeof(s_status), "SD mount failed (%s)", esp_err_to_name(err));
        Serial.printf("[sd] %s\n", s_status);
        return false;
    }
    s_mounted = true;
    if (s_card) {
        const uint64_t mb = ((uint64_t)s_card->csd.capacity * s_card->csd.sector_size) / (1024ull * 1024ull);
        snprintf(s_status, sizeof(s_status), "SD ready — %llu MB", (unsigned long long)mb);
    } else {
        strncpy(s_status, "SD: ready", sizeof(s_status) - 1);
    }
    Serial.printf("[sd] %s\n", s_status);
    return true;
}

bool sd_remount(void) {
    sd_lock();
    sd_unmount();
    const bool ok = sd_mount();
    sd_unlock();
    return ok;
}

bool sd_mkdir_p(const char *path) {
    if (!path || !path[0] || !sd_mount()) {
        return false;
    }
    char tmp[160];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;
    const size_t mp = strlen(SD_MOUNT_POINT);
    for (char *p = tmp + mp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            struct stat st;
            if (stat(tmp, &st) != 0) {
                mkdir(tmp, 0775);
            }
            *p = '/';
        }
    }
    struct stat st;
    if (stat(tmp, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(tmp, 0775) == 0;
}

bool sd_format(void) {
    if (!sd_mount()) {
        return false;
    }
    strncpy(s_status, "SD formatting…", sizeof(s_status) - 1);
    esp_err_t err = esp_vfs_fat_sdcard_format(SD_MOUNT_POINT, s_card);
    if (err != ESP_OK) {
        snprintf(s_status, sizeof(s_status), "SD format failed (%s)", esp_err_to_name(err));
        Serial.printf("[sd] %s\n", s_status);
        return false;
    }
    strncpy(s_status, "SD formatted — FAT ready", sizeof(s_status) - 1);
    return true;
}

static void sanitize_flight(const char *in, char *out, size_t out_len) {
    size_t j = 0;
    if (!in) {
        in = "";
    }
    for (size_t i = 0; in[i] && j + 1 < out_len; i++) {
        const unsigned char c = (unsigned char)in[i];
        if (isalnum(c) || c == '-' || c == '_') {
            out[j++] = (char)toupper(c);
        }
    }
    out[j] = 0;
    if (!out[0]) {
        strncpy(out, "UNKNOWN", out_len - 1);
        out[out_len - 1] = 0;
    }
}

bool sd_ensure_grab_folder(const char *flight, int tz_off_sec, char *out_path, size_t out_len) {
    if (!out_path || out_len < 16) {
        return false;
    }
    if (!sd_mount()) {
        return false;
    }
    time_t now = time(nullptr);
    if (now < 100000) {
        // Clock not set — still allow a folder.
        now = 0;
    }
    now += (time_t)tz_off_sec;
    struct tm t;
    gmtime_r(&now, &t);
    char fl[24];
    sanitize_flight(flight, fl, sizeof(fl));
    char folder[48];
    if (now > 0) {
        snprintf(folder, sizeof(folder), "%02d%02d%02d %s", t.tm_mon + 1, t.tm_mday,
                 (t.tm_year + 1900) % 100, fl);
    } else {
        snprintf(folder, sizeof(folder), "000000 %s", fl);
    }
    snprintf(out_path, out_len, "%s/%s", SD_MOUNT_POINT, folder);
    struct stat st {};
    if (stat(out_path, &st) == 0 && S_ISDIR(st.st_mode)) {
        return true;
    }
    if (mkdir(out_path, 0775) != 0) {
        snprintf(s_status, sizeof(s_status), "SD mkdir failed");
        return false;
    }
    return true;
}

bool sd_write_file(const char *path, const void *data, size_t len) {
    if (!path || !data || len == 0) {
        return false;
    }
    if (!sd_mount()) {
        return false;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        snprintf(s_status, sizeof(s_status), "SD open failed");
        return false;
    }
    const size_t n = fwrite(data, 1, len, f);
    const int rc = fclose(f);
    if (n != len || rc != 0) {
        snprintf(s_status, sizeof(s_status), "SD write failed");
        return false;
    }
    return true;
}

bool sd_write_grab_bmp(const char *folder_path, const void *bmp, size_t len, char *out_name,
                       size_t out_name_len) {
    if (!folder_path || !bmp || len == 0) {
        return false;
    }
    int next = 1;
    for (; next < 1000; next++) {
        char path[160];
        snprintf(path, sizeof(path), "%s/grab_%03d.bmp", folder_path, next);
        struct stat st {};
        if (stat(path, &st) != 0) {
            if (out_name && out_name_len) {
                snprintf(out_name, out_name_len, "grab_%03d.bmp", next);
            }
            return sd_write_file(path, bmp, len);
        }
    }
    snprintf(s_status, sizeof(s_status), "SD folder full");
    return false;
}

bool sd_write_timelapse_avi(const char *folder_path, const void *avi, size_t len) {
    if (!folder_path || !avi || len == 0) {
        return false;
    }
    char path[160];
    snprintf(path, sizeof(path), "%s/timelapse.avi", folder_path);
    return sd_write_file(path, avi, len);
}

void sd_path_join(char *out, size_t out_len, const char *parent, const char *name) {
    if (!out || out_len == 0) {
        return;
    }
    if (!parent || !parent[0]) {
        parent = SD_MOUNT_POINT;
    }
    if (!name || !name[0]) {
        strncpy(out, parent, out_len - 1);
        out[out_len - 1] = 0;
        return;
    }
    const size_t plen = strlen(parent);
    if (plen > 0 && parent[plen - 1] == '/') {
        snprintf(out, out_len, "%s%s", parent, name);
    } else {
        snprintf(out, out_len, "%s/%s", parent, name);
    }
}

int sd_list_dir(const char *path, SdListEntry *out, int max_out) {
    if (!out || max_out <= 0) {
        return 0;
    }
    if (!sd_mount()) {
        return 0;
    }
    const char *dirpath = (path && path[0]) ? path : SD_MOUNT_POINT;
    DIR *d = opendir(dirpath);
    if (!d) {
        snprintf(s_status, sizeof(s_status), "SD list failed");
        return 0;
    }
    int n = 0;
    // Optional parent ".." if not at root
    if (strcmp(dirpath, SD_MOUNT_POINT) != 0 && n < max_out) {
        strncpy(out[n].name, "..", sizeof(out[n].name) - 1);
        out[n].is_dir = true;
        n++;
    }
    struct dirent *ent;
    while ((ent = readdir(d)) != nullptr && n < max_out) {
        if (!ent->d_name[0] || strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
            continue;
        }
        strncpy(out[n].name, ent->d_name, sizeof(out[n].name) - 1);
        out[n].name[sizeof(out[n].name) - 1] = 0;
        char full[192];
        sd_path_join(full, sizeof(full), dirpath, ent->d_name);
        struct stat st {};
        out[n].is_dir = (stat(full, &st) == 0 && S_ISDIR(st.st_mode));
        n++;
    }
    closedir(d);
    return n;
}
