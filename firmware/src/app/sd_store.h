#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_MAX_LIST 64
#define SD_NAME_MAX 64

typedef struct {
    char name[SD_NAME_MAX];
    bool is_dir;
} SdListEntry;

/** Serialize all SDMMC access (UI + net + plate DB). Recursive. */
void sd_lock(void);
void sd_unlock(void);

/** Mount FAT on CrowPanel SDMMC (idempotent). */
bool sd_mount(void);

/** Force unmount + mount (recovery after EIO). */
bool sd_remount(void);

/** True if a card is mounted. */
bool sd_is_mounted(void);

/** Unmount (safe before format/reinsert). */
void sd_unmount(void);

/** Format mounted card as FAT. Card must be mounted first. */
bool sd_format(void);

/** Status text for UI (mount/card info or last error). */
const char *sd_status_text(void);

/**
 * Grab folder under /sdcard/fr_data/grabs/mmddyy_FLIGHT (falls back to fr_data/).
 * flight may be null → "UNKNOWN". Returns full path in out_path.
 */
bool sd_ensure_grab_folder(const char *flight, int tz_off_sec, char *out_path, size_t out_len);

/** Create directory tree (e.g. /sdcard/fr_data/plates). Idempotent. */
bool sd_mkdir_p(const char *path);

/** Write bytes to path (creates/truncates). Internal-RAM bounce; remounts once on EIO. */
bool sd_write_file(const char *path, const void *data, size_t len);

/**
 * Write next Gnnn.BMP into folder_path.
 * On success fills out_rel with filename (not full path).
 */
bool sd_write_grab_bmp(const char *folder_path, const void *bmp, size_t len, char *out_name,
                       size_t out_name_len);

/** Write timelapse.avi into folder_path. */
bool sd_write_timelapse_avi(const char *folder_path, const void *avi, size_t len);

/** List directory (non-recursive). Returns count, capped at SD_MAX_LIST. */
int sd_list_dir(const char *path, SdListEntry *out, int max_out);

/** Join parent + name into out (FAT-safe). */
void sd_path_join(char *out, size_t out_len, const char *parent, const char *name);

#ifdef __cplusplus
}
#endif
