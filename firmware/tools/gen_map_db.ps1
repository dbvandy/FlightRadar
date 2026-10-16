# Generate map_db.h from Natural Earth GeoJSON (coastline, lakes, country borders, US states).
# Major roads (range <= 25 nm) are generated separately: python firmware/tools/gen_road_db.py
$ErrorActionPreference = "Stop"
$outPath = Join-Path $PSScriptRoot "..\src\app\map_db.h"
$cacheDir = Join-Path $PSScriptRoot "ne_cache"
New-Item -ItemType Directory -Force -Path $cacheDir | Out-Null

$sources = @(
    @{ name = "coastline"; url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_coastline.geojson"; filter = $null },
    @{ name = "lakes"; url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_lakes.geojson"; filter = $null },
    @{ name = "borders"; url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_admin_0_boundary_lines_land.geojson"; filter = $null },
    # 50m US state polygons — outer rings include coastlines (lines-only borders omit ocean edges).
    @{ name = "us_states_poly"; url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_50m_admin_1_states_provinces.geojson"; filter = "USA" },
    # Florida county outlines (Census FIPS STATE=12) from Plotly US counties GeoJSON.
    @{ name = "us_counties"; url = "https://raw.githubusercontent.com/plotly/datasets/master/geojson-counties-fips.json"; filter = "STATE:12" }
)

function Get-GeoJson($name, $url) {
    $path = Join-Path $cacheDir "$name.geojson"
    if (-not (Test-Path $path) -or (Get-Item $path).Length -lt 1000) {
        Write-Host "Downloading $name..."
        Invoke-WebRequest -Uri $url -OutFile $path -UseBasicParsing
    }
    return (Get-Content -Raw -Path $path | ConvertFrom-Json)
}

function Add-Ring([System.Collections.Generic.List[object]]$pts, [System.Collections.Generic.List[object]]$polys, $ring) {
    if ($null -eq $ring -or $ring.Count -lt 2) { return }
    $start = $pts.Count
    $prevLat = [int]::MinValue
    $prevLon = [int]::MinValue
    $count = 0
    foreach ($c in $ring) {
        if ($c.Count -lt 2) { continue }
        $lon = [double]$c[0]
        $lat = [double]$c[1]
        if ([double]::IsNaN($lat) -or [double]::IsNaN($lon)) { continue }
        if ($lat -lt -85 -or $lat -gt 85) { continue }
        $latQ = [int][math]::Round($lat * 50.0)
        $lonQ = [int][math]::Round($lon * 50.0)
        if ($latQ -lt -4500) { $latQ = -4500 }
        if ($latQ -gt 4500) { $latQ = 4500 }
        if ($lonQ -lt -9000) { $lonQ = -9000 }
        if ($lonQ -gt 9000) { $lonQ = 9000 }
        if ($latQ -eq $prevLat -and $lonQ -eq $prevLon) { continue }
        $pts.Add([pscustomobject]@{ lat_q = [int16]$latQ; lon_q = [int16]$lonQ })
        $prevLat = $latQ
        $prevLon = $lonQ
        $count++
    }
    if ($count -ge 2) {
        $polys.Add([pscustomobject]@{ start = [uint16]$start; count = [uint16]$count })
    } else {
        while ($pts.Count -gt $start) { $pts.RemoveAt($pts.Count - 1) }
    }
}

function Add-Geometry($pts, $polys, $geom) {
    if ($null -eq $geom) { return }
    $t = [string]$geom.type
    switch ($t) {
        "LineString" {
            Add-Ring $pts $polys $geom.coordinates
        }
        "MultiLineString" {
            foreach ($line in $geom.coordinates) { Add-Ring $pts $polys $line }
        }
        "Polygon" {
            # Outer ring only for sketch outline.
            if ($geom.coordinates.Count -ge 1) { Add-Ring $pts $polys $geom.coordinates[0] }
        }
        "MultiPolygon" {
            foreach ($poly in $geom.coordinates) {
                if ($poly.Count -ge 1) { Add-Ring $pts $polys $poly[0] }
            }
        }
    }
}

$pts = New-Object 'System.Collections.Generic.List[object]'
$polys = New-Object 'System.Collections.Generic.List[object]'

function Test-FeatureFilter($f, $filter) {
    if ([string]::IsNullOrEmpty($filter)) { return $true }
    $p = $f.properties
    if ($null -eq $p) { return $false }
    if ($filter -match '^STATE:(.+)$') {
        $want = [string]$Matches[1]
        $st = [string]$p.STATE
        if ($st -eq $want) { return $true }
        $id = [string]$f.id
        if (-not [string]::IsNullOrEmpty($id) -and $id.StartsWith($want)) { return $true }
        return $false
    }
    foreach ($key in @("adm0_a3", "ADM0_A3", "adm0_a3_us", "iso_a2", "ISO_A2", "gu_a3", "sov_a3", "SOV_A3")) {
        $v = $p.$key
        if ($null -ne $v -and [string]$v -eq $filter) { return $true }
    }
    $admin = [string]$p.admin
    if ($admin -match "United States") { return $true }
    $name = [string]$p.name
    # Some line features tag both sides (e.g. "United States|Canada").
    if ($name -match "United States") { return $true }
    return $false
}

foreach ($src in $sources) {
    $gj = Get-GeoJson $src.name $src.url
    $kept = 0
    foreach ($f in $gj.features) {
        if (-not (Test-FeatureFilter $f $src.filter)) { continue }
        Add-Geometry $pts $polys $f.geometry
        $kept++
    }
    Write-Host ("Parsing {0}: {1}/{2} features kept" -f $src.name, $kept, $gj.features.Count)
}

Write-Host ("Writing {0} points, {1} polylines -> {2}" -f $pts.Count, $polys.Count, $outPath)

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine("#pragma once")
[void]$sb.AppendLine("#include <stdint.h>")
[void]$sb.AppendLine("// Natural Earth coastline/borders/lakes + US state outlines + FL counties. lat_q/lon_q = degrees * 50.")
[void]$sb.AppendLine("struct MapPt { int16_t lat_q; int16_t lon_q; };")
[void]$sb.AppendLine("struct MapPoly { uint16_t start; uint16_t count; };")
[void]$sb.AppendLine("static const MapPt kMapPts[] = {")
foreach ($p in $pts) {
    [void]$sb.AppendLine(("  {{{0}, {1}}}," -f $p.lat_q, $p.lon_q))
}
[void]$sb.AppendLine("};")
[void]$sb.AppendLine(("static const uint16_t kMapPtCount = {0};" -f $pts.Count))
[void]$sb.AppendLine("static const MapPoly kMapPolys[] = {")
foreach ($p in $polys) {
    [void]$sb.AppendLine(("  {{{0}, {1}}}," -f $p.start, $p.count))
}
[void]$sb.AppendLine("};")
[void]$sb.AppendLine(("static const uint16_t kMapPolyCount = {0};" -f $polys.Count))

[System.IO.File]::WriteAllText((Resolve-Path (Split-Path $outPath -Parent)).Path + "\map_db.h", $sb.ToString())
# Also write via absolute path in case Resolve-Path quirks
$resolved = [System.IO.Path]::GetFullPath($outPath)
[System.IO.File]::WriteAllText($resolved, $sb.ToString())

$sketchOut = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\FlightRadar\map_db.h"))
[System.IO.File]::WriteAllText($sketchOut, $sb.ToString())
Write-Host ("Also wrote {0}" -f $sketchOut)

# Sanity: sample US East point presence
$us = 0
foreach ($p in $pts) {
    if ($p.lat_q -ge 1800 -and $p.lat_q -le 2300 -and $p.lon_q -ge -4200 -and $p.lon_q -le -3400) { $us++ }
}
Write-Host ("US-east-ish points: {0}" -f $us)
Write-Host "Done."
