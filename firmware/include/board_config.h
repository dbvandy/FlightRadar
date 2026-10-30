#pragma once

// CrowPanel Advanced 10.1" ESP32-P4 (V1.2): EK79007 1024x600 MIPI-DSI + GT911.

#define PIN_LED             (48)
#define LED_ON              (1)
#define LED_OFF             (0)

#define Touch_GPIO_RST      (40)
#define Touch_GPIO_INT      (42)

#define I2C_GPIO_SCL        (46)
#define I2C_GPIO_SDA        (45)

#define LCD_GPIO_BLIGHT     (31)
#define BLIGHT_PWM_Hz       (30000)
#define BLIGHT_ON_LEVEL     (1)

#define LCD_GPIO_RST        (41)

// Panel power enable (active low on CrowPanel Advanced V1.2)
#define LCD_GPIO_POWER      (29)

// I2S speaker (no ES8311; amp enable only). Pins match Elecrow speaker.py.
#define AUDIO_GPIO_LRCLK    (21)  // WS
#define AUDIO_GPIO_BCLK     (22)
#define AUDIO_GPIO_SDATA    (23)  // DOUT
#define AUDIO_GPIO_CTRL     (30)  // PA enable
// Elecrow bsp_audio set_Audio_ctrl(state): gpio_set_level(CTRL, !state)
// → active-low (LOW = amp on). Kept for docs; audio_click.cpp drives it directly.
#ifndef AUDIO_PA_ACTIVE_HIGH
#define AUDIO_PA_ACTIVE_HIGH 0
#endif

#define V_size              (600)
#define H_size              (1024)
#define LCD_CLK_MHZ         (51)
#define LCD_HPW             (70)
#define LCD_HBP             (160)
#define LCD_HFP             (160)
#define LCD_VPW             (10)
#define LCD_VBP             (23)
#define LCD_VFP             (21)

// microSD (SDMMC 1-bit) — Elecrow CrowPanel Advanced 10.1" Lesson08
#define SD_GPIO_MMC_CLK     (43)
#define SD_GPIO_MMC_CMD     (44)
#define SD_GPIO_MMC_D0      (39)
#define SD_MOUNT_POINT      "/sdcard"

// Raw GitHub base for SD Card Setup → Update Data (plates + airport/runway DBs).
// Push firmware/sd_pack/fr_data to this path on GitHub, then the panel can reinstall
// after a format or new card. Override in platformio build_flags if the repo differs.
#ifndef FR_SD_DATA_BASE_URL
#define FR_SD_DATA_BASE_URL \
    "https://raw.githubusercontent.com/dbvandy/FlightRadar/main/firmware/sd_pack/fr_data"
#endif
