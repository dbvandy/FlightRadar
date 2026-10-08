# ESP32 Flight Radar

Touch-screen ADS-B radar for the **Waveshare ESP32-S3-Touch-LCD-7** (7-inch 800×480 IPS, GT911 touch).

The board pulls live aircraft from the public web, draws a sweeping PPI radar, plots airports in FlightAware-style colors, and lets you tap a plane or airport for details and METAR/TAF weather.

## What you get

1. **Wi-Fi setup screen** — scan networks, tap an SSID, type the password on the on-screen keyboard, connect. Credentials are stored in flash.
2. **Location screen** — enter a US ZIP, city, or street address. ZIP uses Zippopotam; everything else uses OpenStreetMap Nominatim. Tap a match to center the radar anywhere in the world.
3. **Sweeping radar** — north-up display with range rings, a rotating sweep, heading-pointed aircraft, and zoom 15–250 nm.
4. **Airports** — NOAA Aviation Weather Center airport list. Large fields are blue, medium magenta, smaller gray-green, with ICAO/IATA labels.
5. **Tap aircraft** — callsign, ICAO24, type, registration, altitude, speed, track, vertical rate, squawk, position.
6. **Tap airport** — identity, elevation, runway length, METAR (including flight category, wind, temp, visibility) and TAF.

## Hardware

- Waveshare [ESP32-S3-Touch-LCD-7](https://www.waveshare.com/esp32-s3-touch-lcd-7.htm) (touch version)
- USB-C cable on the **UART** Type-C port for flashing and serial logs
- 2.4 GHz Wi-Fi (the radio is 2.4 GHz only)

The wiki lists an ESP32-S3-WROOM-1-**N16R8** (16 MB flash, 8 MB OPI PSRAM). If your board is an 8 MB unit, change `board_upload.flash_size` in `platformio.ini` and use an 8 MB partition table.

## Data sources

| Data | Source | Notes |
| --- | --- | --- |
| Aircraft | [adsb.lol](https://api.adsb.lol/docs) | Primary, no API key, radius up to 250 nm |
| Aircraft fallback | [OpenSky Network](https://opensky-network.org) | Used if adsb.lol fails; anonymous rate limits apply |
| Airports / METAR / TAF | [NOAA Aviation Weather Center](https://aviationweather.gov/data/api/) | Free JSON API |
| ZIP lookup | [Zippopotam.us](https://www.zippopotam.us/) | US 5-digit ZIP |
| Address / city | [Nominatim](https://nominatim.org/) | Identify this app; be polite with search rate |

These are community/government feeds, not FlightAware’s commercial stream. Coverage depends on volunteer ADS-B receivers and public weather stations. Use them for a hobby display, not for navigation.

## Build and flash (PlatformIO)

From this project folder:

```bash
pio run -t upload
pio device monitor
```

Arduino IDE alternative:

1. Install Arduino-ESP32 **3.0+** and board **ESP32S3 Dev Module**.
2. Tools: PSRAM **OPI**, Flash Mode **QIO 80 MHz**, Flash Size **16 MB**, USB CDC On Boot **Disabled** if you use the UART port.
3. Libraries: LVGL 8.4.0, ESP32_Display_Panel, ESP32_IO_Expander, esp-lib-utils, ArduinoJson 7.
4. Copy `firmware/include/*.h` and `firmware/src/**` into a sketch named `FlightRadar`, with `main.cpp` renamed to `FlightRadar.ino`.

After upload, press **RESET** on the board.

## First use

1. Power the board. The Wi-Fi screen scans automatically.
2. Tap your network, type the password, tap **Connect**.
3. Enter a ZIP, city, or address, tap **Search**, then tap the correct result.
4. Use **ZOOM + / ZOOM -** for region size (capped at 250 nm by the ADS-B API).
5. Tap a yellow chevron for the aircraft card; tap a colored airport square for weather.
6. **Change location** and **Wi-Fi setup** stay available from the right-hand panel.

## Project layout

```
platformio.ini          PlatformIO env for the 7" Waveshare board
firmware/include/       LVGL + ESP32_Display_Panel board config
firmware/src/main.cpp   Boot, Wi-Fi, background fetch task
firmware/src/app/       UI, NVS config, HTTPS clients
firmware/src/vendor/    Official LVGL v8 RGB port
```

## Tuning

- Traffic poll interval: `10000` ms in `firmware/src/main.cpp`
- Sweep speed: `sweep_cb` period `40` ms and `sweep_deg += 3.6` in `ui.cpp`
- If the RGB panel looks shifted, leave anti-tearing mode 3 enabled in `lvgl_v8_port.h` (already set)

## Credits

Display bring-up follows Espressif’s ESP32_Display_Panel LVGL v8 port and Waveshare’s ESP32-S3-Touch-LCD-7 pinout (CH422G expander, ST7262 RGB, GT911).
