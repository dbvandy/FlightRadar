#!/usr/bin/env python3
"""Pack FAA plates + POD databases into firmware/sd_pack/fr_data for the SD card.

Copy the resulting fr_data/ folder to the root of the CrowPanel SD card:
  /sdcard/fr_data/plates/*.frpl
  /sdcard/fr_data/waypoints.bin
  /sdcard/fr_data/airports.bin
  /sdcard/fr_data/runways.bin

Usage:
  python firmware/tools/pack_sd_assets.py
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "firmware" / "src" / "app"
PLATES_DIR = Path(__file__).resolve().parent / "faa_plates"
OUT = ROOT / "firmware" / "sd_pack" / "fr_data"

# Match plates currently embedded in faa_plate_db.h
PLATE_ICAOS = [
    "KATL",
    "KBNA",
    "KCLT",
    "KDFW",
    "KDTW",
    "KGNV",
    "KHOU",
    "KINF",
    "KJAX",
    "KMCO",
    "KOCF",
    "KTPA",
    "KX60",
]
PLATE_SIZE = 280
# LV_IMG_CF_TRUE_COLOR
LV_IMG_CF_TRUE_COLOR = 4

FRPL_MAGIC = b"FRPL"
WPT_MAGIC = b"FRWP"
APT_MAGIC = b"FRAP"
RWY_MAGIC = b"FRRW"


def to_rgb565(img: Image.Image) -> bytes:
    rgb = img.convert("RGB")
    out = bytearray(rgb.width * rgb.height * 2)
    i = 0
    for r, g, b in rgb.getdata():
        c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        out[i] = c & 0xFF
        out[i + 1] = (c >> 8) & 0xFF
        i += 2
    return bytes(out)


def pack_plates() -> int:
    dest = OUT / "plates"
    dest.mkdir(parents=True, exist_ok=True)
    n = 0
    for icao in PLATE_ICAOS:
        src = PLATES_DIR / f"{icao}.png"
        if not src.is_file():
            print(f"  SKIP plate {icao}: missing {src.name}")
            continue
        img = Image.open(src).convert("RGB")
        if img.size != (PLATE_SIZE, PLATE_SIZE):
            img = img.resize((PLATE_SIZE, PLATE_SIZE), Image.Resampling.LANCZOS)
        data = to_rgb565(img)
        # header: magic, w, h, cf, reserved, data_size
        hdr = struct.pack(
            "<4sHHHHI",
            FRPL_MAGIC,
            PLATE_SIZE,
            PLATE_SIZE,
            LV_IMG_CF_TRUE_COLOR,
            0,
            len(data),
        )
        (dest / f"{icao}.frpl").write_bytes(hdr + data)
        print(f"  plate {icao}.frpl  {len(data)} B")
        n += 1
    return n


def pack_waypoints() -> bool:
    hdr = APP / "waypoint_db.h"
    text = hdr.read_text(encoding="utf-8", errors="replace")
    rows = re.findall(
        r'\{\s*"([^"]+)"\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\}',
        text,
    )
    if not rows:
        print("  FAIL waypoints: no records parsed")
        return False
    body = bytearray()
    for name, lat, lon in rows:
        nm = name.encode("ascii")[:5].ljust(6, b"\0")
        body += nm + struct.pack("<hh", int(lat), int(lon))
    out = OUT / "waypoints.bin"
    out.write_bytes(struct.pack("<4sI", WPT_MAGIC, len(rows)) + body)
    print(f"  waypoints.bin  {len(rows)} recs  {out.stat().st_size} B")
    return True


def pack_airports() -> bool:
    hdr = APP / "airport_db.h"
    text = hdr.read_text(encoding="utf-8", errors="replace")
    # {"00AN", "", 5909, -15646, 80, 4517, 2},
    rows = re.findall(
        r'\{\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}',
        text,
    )
    if not rows:
        print("  FAIL airports: no records parsed")
        return False
    body = bytearray()
    for icao, iata, lat, lon, elev, rwy, size in rows:
        body += icao.encode("ascii")[:4].ljust(5, b"\0")
        body += iata.encode("ascii")[:3].ljust(4, b"\0")
        body += struct.pack("<hhhHB", int(lat), int(lon), int(elev), int(rwy), int(size))
    out = OUT / "airports.bin"
    out.write_bytes(struct.pack("<4sI", APT_MAGIC, len(rows)) + body)
    print(f"  airports.bin  {len(rows)} recs  {out.stat().st_size} B")
    return True


def pack_runways() -> bool:
    """Pack packed RwyRec: icao[5], le[4], he[4], len_ft, hdg_deg, width_ft."""
    hdr = APP / "runway_db.h"
    text = hdr.read_text(encoding="utf-8", errors="replace")
    # {"ZYYK", "04", "22", 8202, 40, 100},
    rows = re.findall(
        r'\{\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,'
        r"\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}",
        text,
    )
    if not rows:
        print("  SKIP runways: record pattern mismatch — leave in flash")
        return False
    body = bytearray()
    for icao, le, he, length, hdg, width in rows:
        body += icao.encode("ascii")[:4].ljust(5, b"\0")
        body += le.encode("ascii")[:3].ljust(4, b"\0")
        body += he.encode("ascii")[:3].ljust(4, b"\0")
        body += struct.pack("<HHB", int(length), int(hdg), int(width) & 0xFF)
    out = OUT / "runways.bin"
    out.write_bytes(struct.pack("<4sI", RWY_MAGIC, len(rows)) + body)
    print(f"  runways.bin  {len(rows)} recs  {out.stat().st_size} B")
    return True


def write_manifest() -> None:
    import json

    files: list[dict] = []
    for name in ("airports.bin", "runways.bin", "waypoints.bin"):
        p = OUT / name
        if p.is_file():
            files.append({"path": name, "size": p.stat().st_size})
    plates = OUT / "plates"
    if plates.is_dir():
        for p in sorted(plates.glob("*.frpl")):
            files.append({"path": f"plates/{p.name}", "size": p.stat().st_size})
    (OUT / "manifest.json").write_text(
        json.dumps({"version": 1, "files": files}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"  manifest.json  {len(files)} files")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    print(f"Packing → {OUT}")
    n_plates = pack_plates()
    ok_w = pack_waypoints()
    ok_a = pack_airports()
    ok_r = pack_runways()
    write_manifest()
    readme = OUT / "README.txt"
    readme.write_text(
        "SD data pack for CrowPanel Flight Radar\n"
        "\n"
        "Option A — copy this folder to the SD card root:\n"
        "  /sdcard/fr_data/plates/*.frpl\n"
        "  /sdcard/fr_data/*.bin\n"
        "  /sdcard/fr_data/manifest.json\n"
        "\n"
        "Option B — push this folder to GitHub at:\n"
        "  firmware/sd_pack/fr_data/\n"
        "Then on the device: SD Card Setup → Update Data (Wi-Fi).\n"
        "Default URL (board_config.h FR_SD_DATA_BASE_URL):\n"
        "  https://raw.githubusercontent.com/dbvandy/FlightRadar/main/firmware/sd_pack/fr_data\n",
        encoding="utf-8",
    )
    print(f"Done: {n_plates} plates, waypoints={ok_w}, airports={ok_a}, runways={ok_r}")
    if n_plates < 1 or not ok_a or not ok_r:
        return 1
    if not ok_w:
        print("  NOTE: waypoints.bin skipped — regenerate waypoint_db.h then re-run pack")
    return 0


if __name__ == "__main__":
    sys.exit(main())
