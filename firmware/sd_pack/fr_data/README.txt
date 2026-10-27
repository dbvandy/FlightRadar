SD data pack for CrowPanel Flight Radar

Option A — copy this folder to the SD card root:
  /sdcard/fr_data/plates/*.frpl
  /sdcard/fr_data/*.bin
  /sdcard/fr_data/manifest.json

Option B — push this folder to GitHub at:
  firmware/sd_pack/fr_data/
Then on the device: SD Card Setup → Update Data (Wi-Fi).
Default URL (board_config.h FR_SD_DATA_BASE_URL):
  https://raw.githubusercontent.com/dbvandy/FlightRadar/main/firmware/sd_pack/fr_data
