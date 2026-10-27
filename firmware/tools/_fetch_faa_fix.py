#!/usr/bin/env python3
"""Download FAA NASR FIX_BASE.csv and build waypoints.bin for SD pack."""
from __future__ import annotations

import csv
import io
import re
import struct
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT_BIN = ROOT / "firmware" / "sd_pack" / "fr_data" / "waypoints.bin"
CACHE = Path(__file__).resolve().parent / "_cache_faa_fix"
UA = {"User-Agent": "Mozilla/5.0 (FlightRadar-waypoint-pack/1.0)"}

# Try current + recent cycles (YYYY-MM-DD folder pages).
CYCLES = [
    "2026-10-01",
    "2026-09-03",
    "2026-08-06",
    "2025-09-04",
]


def get(url: str) -> bytes:
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=180) as r:
        return r.read()


def find_fix_zip_url(cycle: str) -> str | None:
    page = f"https://www.faa.gov/air_traffic/flight_info/aeronav/aero_data/NASR_Subscription/{cycle}"
    print(f"GET {page}")
    try:
        html = get(page).decode("utf-8", "replace")
    except Exception as e:
        print(f"  page fail: {e}")
        return None
    # Prefer CSV FIX group zip.
    hrefs = re.findall(r'href="([^"]+)"', html)
    candidates = []
    for h in hrefs:
        hu = h.replace("&amp;", "&")
        if ".zip" not in hu.lower():
            continue
        if "FIX" in hu.upper() or "fix" in hu.lower():
            candidates.append(hu)
    # Also absolute CDN-style URLs embedded in the page.
    candidates += re.findall(
        r'https?://[^"\s]+FIX[^"\s]+\.zip', html, flags=re.I
    )
    # De-dupe, prefer CSV over TXT.
    seen = set()
    ordered = []
    for c in candidates:
        if c.startswith("/"):
            c = "https://www.faa.gov" + c
        if c in seen:
            continue
        seen.add(c)
        ordered.append(c)
    ordered.sort(key=lambda u: (0 if "CSV" in u.upper() or "csv" in u else 1, u))
    for u in ordered:
        print(f"  candidate {u}")
    return ordered[0] if ordered else None


def parse_fix_base_csv(text: str) -> list[tuple[str, int, int]]:
    """Parse FIX_BASE.csv — columns vary slightly by cycle."""
    f = io.StringIO(text)
    # Detect header
    sample = text[:2000]
    dialect = csv.Sniffer().sniff(sample, delimiters=",")
    reader = csv.DictReader(f, dialect=dialect)
    rows: list[tuple[str, int, int]] = []
    if not reader.fieldnames:
        return rows
    fields = {n.upper(): n for n in reader.fieldnames}

    def col(*names: str) -> str | None:
        for n in names:
            if n.upper() in fields:
                return fields[n.upper()]
        # fuzzy
        for k, orig in fields.items():
            for n in names:
                if n.upper() in k:
                    return orig
        return None

    id_col = col("FIX_ID", "FIX_IDENTIFIER", "FIX_BASE_ID", "IDENTIFIER")
    lat_col = col("LAT_DECIMAL", "LAT_DEG", "LATITUDE_DECIMAL", "LAT")
    lon_col = col("LONG_DECIMAL", "LON_DECIMAL", "LONGITUDE_DECIMAL", "LON", "LONG")
    # DMS fallbacks in description-style fields
    lat_dms = col("LAT_DEG_MIN_SEC", "LATITUDE")
    lon_dms = col("LONG_DEG_MIN_SEC", "LONGITUDE")
    print(f"  cols id={id_col} lat={lat_col} lon={lon_col} lat_dms={lat_dms} lon_dms={lon_dms}")
    print(f"  all fields: {reader.fieldnames[:20]}...")

    dms_re = re.compile(
        r"(\d+)-(\d+)-(\d+(?:\.\d+)?)([NS])\s+(\d+)-(\d+)-(\d+(?:\.\d+)?)([EW])"
    )

    def dms_to_c(d, m, s, hemi) -> int:
        val = int(d) + int(m) / 60.0 + float(s) / 3600.0
        if hemi in ("S", "W"):
            val = -val
        return int(round(val * 100.0))

    for row in reader:
        name = (row.get(id_col) or "").strip().upper() if id_col else ""
        if not re.fullmatch(r"[A-Z0-9]{3,5}", name):
            continue
        lat_c = lon_c = None
        if lat_col and lon_col:
            try:
                lat = float(row[lat_col])
                lon = float(row[lon_col])
                lat_c = int(round(lat * 100.0))
                lon_c = int(round(lon * 100.0))
            except (TypeError, ValueError, KeyError):
                lat_c = lon_c = None
        if (lat_c is None or lon_c is None) and lat_dms and lon_dms:
            blob = f"{row.get(lat_dms, '')} {row.get(lon_dms, '')}"
            m = dms_re.search(blob)
            if m:
                lat_c = dms_to_c(m.group(1), m.group(2), m.group(3), m.group(4))
                lon_c = dms_to_c(m.group(5), m.group(6), m.group(7), m.group(8))
        if lat_c is None or lon_c is None:
            continue
        if lat_c < -9000 or lat_c > 9000 or lon_c < -18000 or lon_c > 18000:
            continue
        rows.append((name, lat_c, lon_c))
    return rows


def extract_fix_base(zbytes: bytes) -> str:
    with zipfile.ZipFile(io.BytesIO(zbytes)) as zf:
        names = zf.namelist()
        print("  zip contains:", names[:30], ("..." if len(names) > 30 else ""))
        for n in names:
            base = Path(n).name.upper()
            if base == "FIX_BASE.CSV" or base.endswith("FIX_BASE.CSV"):
                return zf.read(n).decode("utf-8", "replace")
        # Sometimes nested
        for n in names:
            if "FIX_BASE" in Path(n).name.upper() and n.lower().endswith(".csv"):
                return zf.read(n).decode("utf-8", "replace")
    raise FileNotFoundError("FIX_BASE.csv not in zip")


def write_bin(rows: list[tuple[str, int, int]], dest: Path) -> None:
    seen: set[str] = set()
    unique: list[tuple[str, int, int]] = []
    for name, lat_c, lon_c in sorted(rows, key=lambda t: t[0]):
        if name in seen:
            continue
        seen.add(name)
        unique.append((name, lat_c, lon_c))
    body = bytearray()
    for name, lat_c, lon_c in unique:
        body += name.encode("ascii")[:5].ljust(6, b"\0")
        body += struct.pack("<hh", lat_c, lon_c)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(struct.pack("<4sI", b"FRWP", len(unique)) + body)
    print(f"Wrote {dest}  {len(unique)} waypoints  {dest.stat().st_size} B")


def main() -> int:
    CACHE.mkdir(parents=True, exist_ok=True)
    zpath = CACHE / "FIX_CSV.zip"
    csv_path = CACHE / "FIX_BASE.csv"

    if not csv_path.is_file() or csv_path.stat().st_size < 1000:
        zip_url = None
        for cycle in CYCLES:
            zip_url = find_fix_zip_url(cycle)
            if zip_url:
                break
        if not zip_url:
            # Direct known pattern used by FAA downloads CDN (may change).
            print("No FIX zip link found on subscription pages")
            return 1
        print(f"Downloading {zip_url}")
        zbytes = get(zip_url)
        zpath.write_bytes(zbytes)
        print(f"  saved {zpath} ({len(zbytes)} B)")
        text = extract_fix_base(zbytes)
        csv_path.write_text(text, encoding="utf-8")
    else:
        print(f"Using cached {csv_path}")
        text = csv_path.read_text(encoding="utf-8", errors="replace")

    rows = parse_fix_base_csv(text)
    print(f"Parsed {len(rows)} fix rows")
    if len(rows) < 1000:
        print("Too few rows — abort")
        return 1
    write_bin(rows, OUT_BIN)
    # Spot-check a few well-known Florida / enroute fixes if present.
    data = OUT_BIN.read_bytes()
    count = struct.unpack_from("<I", data, 4)[0]
    names = set()
    for i in range(count):
        o = 8 + i * 10
        nm = data[o : o + 6].split(b"\0", 1)[0].decode("ascii", "replace")
        names.add(nm)
    for probe in ("KNOST", "PINEL", "ORATE", "CWRLD", "LEESE", "ATL"):
        print(f"  has {probe}: {probe in names}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
