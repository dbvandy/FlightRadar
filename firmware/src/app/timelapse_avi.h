#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Max frames on SD (ring; drops oldest file). */
#define FR_TL_MAX_FRAMES 48
/** Encoded frame size: 200x200 crop centered on the followed aircraft. */
#define FR_TL_W 200
#define FR_TL_H 200

/** Set SD directory for JPEG frame files (e.g. grab folder). Empty clears. */
void tl_set_storage_dir(const char *dir);

/** Clear frame metadata and delete any temp JPEG files under the storage dir. */
void tl_clear(void);

/** How many frames are buffered (on SD). */
int tl_frame_count(void);

/**
 * Downscale RGB565 snapshot → JPEG, write to SD as tl_NNN.jpg.
 * Keeps only length/id metadata in RAM (not JPEG bytes).
 * snap_rgb565: LV_IMG_CF_TRUE_COLOR / RGB565 LE, w×h.
 */
bool tl_add_rgb565_frame(const uint16_t *snap_rgb565, int snap_w, int snap_h);

/**
 * Mux SD JPEG frames into an MJPEG AVI at path (streaming write).
 * Reads one JPEG at a time; peak RAM ≈ one frame + AVI write buffer.
 * On success deletes temp frame files and clears; *out_len is file size.
 */
bool tl_build_avi_to_file(const char *path, int fps, size_t *out_len);

#ifdef __cplusplus
}
#endif
