# Generate runway_db.h for airports already in airport_db.h (rwy >= 3500).
$ErrorActionPreference = "Stop"
$aptDbPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\src\app\airport_db.h"))
$outPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\src\app\runway_db.h"))
$cacheDir = Join-Path $PSScriptRoot "oa_cache"
$runwaysPath = Join-Path $cacheDir "runways.csv"
if (-not (Test-Path $runwaysPath)) { throw "Missing $runwaysPath — run gen_airport_db.ps1 first" }
if (-not (Test-Path $aptDbPath)) { throw "Missing $aptDbPath" }

Write-Host "Loading ICAO set from airport_db.h..."
$icaoSet = @{}
Select-String -Path $aptDbPath -Pattern '\{"([A-Z0-9]{2,4})"' | ForEach-Object {
    $icaoSet[$_.Matches[0].Groups[1].Value] = $true
}
Write-Host ("Airport ICAOs: {0}" -f $icaoSet.Count)

Write-Host "Parsing runways.csv..."
$rwys = New-Object 'System.Collections.Generic.List[object]'
$reader = [System.IO.StreamReader]::new($runwaysPath)
try {
    $header = $reader.ReadLine()
    $cols = @($header.Split(',') | ForEach-Object { $_.Trim().Trim('"') })
    $idxIdent = [array]::IndexOf($cols, "airport_ident")
    $idxLen = [array]::IndexOf($cols, "length_ft")
    $idxWidth = [array]::IndexOf($cols, "width_ft")
    $idxClosed = [array]::IndexOf($cols, "closed")
    $idxLe = [array]::IndexOf($cols, "le_ident")
    $idxHe = [array]::IndexOf($cols, "he_ident")
    $idxHdg = [array]::IndexOf($cols, "le_heading_degT")
    if ($idxIdent -lt 0 -or $idxLen -lt 0) { throw "runways.csv missing columns" }

    while ($null -ne ($line = $reader.ReadLine())) {
        if ($line.Contains('"')) {
            $fields = [regex]::Split($line, ',(?=(?:[^"]*"[^"]*")*[^"]*$)')
        } else {
            $fields = $line.Split(',')
        }
        $need = [Math]::Max($idxIdent, $idxLen)
        if ($fields.Count -le $need) { continue }
        $ident = $fields[$idxIdent].Trim('"').ToUpperInvariant()
        if (-not $icaoSet.ContainsKey($ident)) { continue }
        if ($idxClosed -ge 0 -and $fields.Count -gt $idxClosed -and $fields[$idxClosed] -eq "1") { continue }

        $len = 0
        if (-not [int]::TryParse($fields[$idxLen].Trim('"'), [ref]$len) -or $len -lt 3500) { continue }
        if ($len -gt 65535) { $len = 65535 }

        $width = 100
        if ($idxWidth -ge 0 -and $fields.Count -gt $idxWidth) {
            $w = 0
            if ([int]::TryParse($fields[$idxWidth].Trim('"'), [ref]$w) -and $w -gt 0) {
                $width = [Math]::Min($w, 255)
            }
        }

        $le = ""
        $he = ""
        if ($idxLe -ge 0 -and $fields.Count -gt $idxLe) { $le = $fields[$idxLe].Trim('"').ToUpperInvariant() }
        if ($idxHe -ge 0 -and $fields.Count -gt $idxHe) { $he = $fields[$idxHe].Trim('"').ToUpperInvariant() }
        if ($le.Length -gt 3) { $le = $le.Substring(0, 3) }
        if ($he.Length -gt 3) { $he = $he.Substring(0, 3) }

        $hdg = 0.0
        $haveHdg = $false
        if ($idxHdg -ge 0 -and $fields.Count -gt $idxHdg) {
            $hs = $fields[$idxHdg].Trim('"')
            if ($hs -and [double]::TryParse($hs, [ref]$hdg)) { $haveHdg = $true }
        }
        if (-not $haveHdg -and $le) {
            # Derive from runway number digits (e.g. 26L -> 260).
            $digits = ($le -replace '[^0-9]', '')
            $n = 0
            if ([int]::TryParse($digits, [ref]$n) -and $n -ge 1 -and $n -le 36) {
                $hdg = $n * 10.0
                $haveHdg = $true
            }
        }
        if (-not $haveHdg) { continue }
        while ($hdg -lt 0) { $hdg += 360 }
        while ($hdg -ge 360) { $hdg -= 360 }
        $hdgI = [int][math]::Round($hdg)
        if ($hdgI -ge 360) { $hdgI = 0 }

        if (-not $le) { $le = ("{0:D2}" -f [int][math]::Round($hdgI / 10.0)) }
        if (-not $he) {
            $opp = ($hdgI + 180) % 360
            $he = ("{0:D2}" -f [int][math]::Round($opp / 10.0))
            if ($he -eq "00") { $he = "36" }
        }

        $rwys.Add([pscustomobject]@{
            icao = $ident
            le = $le
            he = $he
            len_ft = [uint16]$len
            hdg_deg = [uint16]$hdgI
            width_ft = [byte]$width
        })
    }
} finally {
    $reader.Close()
}

$rwys = @($rwys | Sort-Object icao, len_ft -Descending)
Write-Host ("Runways kept: {0}" -f $rwys.Count)

$sb = New-Object System.Text.StringBuilder 4MB
[void]$sb.AppendLine("#pragma once")
[void]$sb.AppendLine("#include <stdint.h>")
[void]$sb.AppendLine("// Auto-generated from OurAirports runways for airports with longest rwy >= 3500 ft.")
[void]$sb.AppendLine("// hdg_deg is true heading of the LE end.")
[void]$sb.AppendLine("struct RwyRec { char icao[5]; char le[4]; char he[4]; uint16_t len_ft; uint16_t hdg_deg; uint8_t width_ft; };")
[void]$sb.AppendLine("static const RwyRec kRwyDb[] = {")
foreach ($r in $rwys) {
    [void]$sb.AppendLine(('  {{"{0}", "{1}", "{2}", {3}, {4}, {5}}},' -f $r.icao, $r.le, $r.he, $r.len_ft, $r.hdg_deg, $r.width_ft))
}
[void]$sb.AppendLine("};")
[void]$sb.AppendLine(("#define K_RWY_DB_COUNT ({0})" -f $rwys.Count))
[System.IO.File]::WriteAllText($outPath, $sb.ToString())
Write-Host "Wrote $outPath"
