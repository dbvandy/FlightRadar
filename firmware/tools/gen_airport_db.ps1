# Generate airport_db.h from OurAirports (airports with longest runway >= 3500 ft).
$ErrorActionPreference = "Stop"
$outPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\src\app\airport_db.h") )
$cacheDir = Join-Path $PSScriptRoot "oa_cache"
New-Item -ItemType Directory -Force -Path $cacheDir | Out-Null

$minRwy = 3500
$airportsUrl = "https://davidmegginson.github.io/ourairports-data/airports.csv"
$runwaysUrl = "https://davidmegginson.github.io/ourairports-data/runways.csv"
$airportsPath = Join-Path $cacheDir "airports.csv"
$runwaysPath = Join-Path $cacheDir "runways.csv"

function Ensure-File($path, $url) {
    if (-not (Test-Path $path) -or (Get-Item $path).Length -lt 100000) {
        Write-Host "Downloading $url ..."
        Invoke-WebRequest -Uri $url -OutFile $path -UseBasicParsing
    }
}

Ensure-File $runwaysPath $runwaysUrl
Ensure-File $airportsPath $airportsUrl

Write-Host "Indexing longest runway lengths..."
$rwyMax = @{}
$rwyReader = [System.IO.StreamReader]::new($runwaysPath)
try {
    $header = $rwyReader.ReadLine()
    $cols = @($header.Split(',') | ForEach-Object { $_.Trim().Trim('"') })
    $idxIdent = [array]::IndexOf($cols, "airport_ident")
    $idxLen = [array]::IndexOf($cols, "length_ft")
    $idxClosed = [array]::IndexOf($cols, "closed")
    if ($idxIdent -lt 0 -or $idxLen -lt 0) { throw "runways.csv missing columns: $($cols -join '|')" }
    while ($null -ne ($line = $rwyReader.ReadLine())) {
        # Fast path: most fields are unquoted. Use regex CSV split only when needed.
        if ($line.Contains('"')) {
            $fields = [regex]::Split($line, ',(?=(?:[^"]*"[^"]*")*[^"]*$)')
        } else {
            $fields = $line.Split(',')
        }
        if ($fields.Count -le [Math]::Max($idxIdent, $idxLen)) { continue }
        if ($idxClosed -ge 0 -and $fields.Count -gt $idxClosed -and $fields[$idxClosed] -eq "1") { continue }
        $ident = $fields[$idxIdent].Trim('"')
        $lenStr = $fields[$idxLen].Trim('"')
        if (-not $ident -or -not $lenStr) { continue }
        $len = 0
        if (-not [int]::TryParse($lenStr, [ref]$len)) { continue }
        if ($len -le 0) { continue }
        if (-not $rwyMax.ContainsKey($ident) -or $len -gt $rwyMax[$ident]) {
            $rwyMax[$ident] = $len
        }
    }
} finally {
    $rwyReader.Close()
}
Write-Host ("Runway entries for {0} airports" -f $rwyMax.Count)

Write-Host "Selecting airports with runway >= $minRwy ft..."
$selected = New-Object 'System.Collections.Generic.List[object]'
$aptReader = [System.IO.StreamReader]::new($airportsPath)
try {
    $header = $aptReader.ReadLine()
    $cols = @($header.Split(',') | ForEach-Object { $_.Trim().Trim('"') })
    $idxIdent = [array]::IndexOf($cols, "ident")
    $idxType = [array]::IndexOf($cols, "type")
    $idxLat = [array]::IndexOf($cols, "latitude_deg")
    $idxLon = [array]::IndexOf($cols, "longitude_deg")
    $idxIata = [array]::IndexOf($cols, "iata_code")
    $idxElev = [array]::IndexOf($cols, "elevation_ft")
    if ($idxIdent -lt 0 -or $idxType -lt 0 -or $idxLat -lt 0 -or $idxLon -lt 0) {
        throw "airports.csv missing columns: $($cols -join '|')"
    }
    while ($null -ne ($line = $aptReader.ReadLine())) {
        if ($line.Contains('"')) {
            $fields = [regex]::Split($line, ',(?=(?:[^"]*"[^"]*")*[^"]*$)')
        } else {
            $fields = $line.Split(',')
        }
        $need = [Math]::Max([Math]::Max($idxIdent, $idxType), [Math]::Max($idxLat, $idxLon))
        if ($fields.Count -le $need) { continue }
        $type = $fields[$idxType].Trim('"')
        if ($type -ne "large_airport" -and $type -ne "medium_airport" -and $type -ne "small_airport") {
            continue
        }
        $ident = $fields[$idxIdent].Trim('"')
        if (-not $ident -or $ident.Length -gt 4) { continue }
        if (-not $rwyMax.ContainsKey($ident)) { continue }
        $rwy = [int]$rwyMax[$ident]
        if ($rwy -lt $minRwy) { continue }
        if ($rwy -gt 65535) { $rwy = 65535 }

        $lat = 0.0
        $lon = 0.0
        if (-not [double]::TryParse($fields[$idxLat].Trim('"'), [ref]$lat)) { continue }
        if (-not [double]::TryParse($fields[$idxLon].Trim('"'), [ref]$lon)) { continue }
        if ($lat -lt -90 -or $lat -gt 90 -or $lon -lt -180 -or $lon -gt 180) { continue }

        $iata = ""
        if ($idxIata -ge 0 -and $fields.Count -gt $idxIata) {
            $iata = $fields[$idxIata].Trim('"')
            if ($iata.Length -gt 3) { $iata = $iata.Substring(0, 3) }
        }

        $size = 2
        if ($type -eq "large_airport") { $size = 0 }
        elseif ($type -eq "medium_airport") { $size = 1 }

        $latC = [int][math]::Round($lat * 100.0)
        $lonC = [int][math]::Round($lon * 100.0)
        if ($latC -lt -9000) { $latC = -9000 }
        if ($latC -gt 9000) { $latC = 9000 }
        if ($lonC -lt -18000) { $lonC = -18000 }
        if ($lonC -gt 18000) { $lonC = 18000 }

        $elev = 0
        if ($idxElev -ge 0 -and $fields.Count -gt $idxElev) {
            $elevStr = $fields[$idxElev].Trim('"')
            $elevTmp = 0
            if ($elevStr -and [int]::TryParse($elevStr, [ref]$elevTmp)) {
                if ($elevTmp -lt -2000) { $elevTmp = -2000 }
                if ($elevTmp -gt 30000) { $elevTmp = 30000 }
                $elev = $elevTmp
            }
        }

        $selected.Add([pscustomobject]@{
            icao = $ident.ToUpperInvariant()
            iata = $iata.ToUpperInvariant()
            lat_c = [int16]$latC
            lon_c = [int16]$lonC
            elev_ft = [int16]$elev
            rwy_ft = [uint16]$rwy
            size = [byte]$size
        })
    }
} finally {
    $aptReader.Close()
}

# Stable sort by ICAO
$selected = @($selected | Sort-Object icao)
Write-Host ("Keeping {0} airports" -f $selected.Count)

$sb = New-Object System.Text.StringBuilder 4MB
[void]$sb.AppendLine("#pragma once")
[void]$sb.AppendLine("#include <stdint.h>")
[void]$sb.AppendLine("// Auto-generated from OurAirports. lat_c/lon_c = degrees * 100.")
[void]$sb.AppendLine("// Only airports with longest runway >= $minRwy ft. size: 0=large 1=medium 2=small")
[void]$sb.AppendLine("struct AptRec { char icao[5]; char iata[4]; int16_t lat_c; int16_t lon_c; int16_t elev_ft; uint16_t rwy_ft; uint8_t size; };")
[void]$sb.AppendLine("static const AptRec kAptDb[] = {")
foreach ($a in $selected) {
    $iataLit = $a.iata
    [void]$sb.AppendLine(('  {{"{0}", "{1}", {2}, {3}, {4}, {5}, {6}}},' -f $a.icao, $iataLit, $a.lat_c, $a.lon_c, $a.elev_ft, $a.rwy_ft, $a.size))
}
[void]$sb.AppendLine("};")
[void]$sb.AppendLine(("#define K_APT_DB_COUNT ({0})" -f $selected.Count))
[System.IO.File]::WriteAllText($outPath, $sb.ToString())
Write-Host "Wrote $outPath"
