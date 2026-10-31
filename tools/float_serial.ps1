# Floating always-on-top live serial console for CrowPanel (COM5 @ 115200)
$ErrorActionPreference = 'Stop'
$PortName = 'COM5'
$Baud = 115200

Add-Type @"
using System;
using System.Runtime.InteropServices;
public class FrFloatWin {
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);
  [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
  public static readonly IntPtr HWND_TOPMOST = new IntPtr(-1);
  public const uint SWP_SHOWWINDOW = 0x0040;
}
"@

function Set-TopMost([IntPtr]$Hwnd) {
  if ($Hwnd -eq [IntPtr]::Zero) { return }
  [void][FrFloatWin]::SetWindowPos($Hwnd, [FrFloatWin]::HWND_TOPMOST, 60, 40, 1050, 620, [FrFloatWin]::SWP_SHOWWINDOW)
}

$Host.UI.RawUI.WindowTitle = "Flight Radar Serial - $PortName (always on top)"
try { $Host.UI.RawUI.BufferSize = New-Object System.Management.Automation.Host.Size(160, 5000) } catch {}
Start-Sleep -Milliseconds 400
Set-TopMost (Get-Process -Id $PID).MainWindowHandle
try { Set-TopMost ([FrFloatWin]::GetForegroundWindow()) } catch {}

function Write-Banner {
  Clear-Host
  Write-Host ''
  Write-Host "  Flight Radar live serial   $PortName @ $Baud   always on top" -ForegroundColor Cyan
  Write-Host '  Highlighted: FA DNS | hard recover | FA board | ADS-B ac= | TLS | DNS fail' -ForegroundColor DarkGray
  Write-Host '  Close this window to stop.' -ForegroundColor DarkGray
  Write-Host ''
}

function Write-LineColored([string]$line) {
  $ts = Get-Date -Format 'HH:mm:ss.fff'
  $color = 'Gray'
  if ($line -match 'FA DNS ok|ADS-B ac=|FA board .* body|session open|early-complete|enroute=') { $color = 'Green' }
  elseif ($line -match 'hard recover|Rejoining|FA DNS dead|Preparing board') { $color = 'Yellow' }
  elseif ($line -match 'DNS Failed|TLS fail|TLS ERROR|board fetch fail|Generic error|chunk data timeout') { $color = 'Red' }
  elseif ($line -match 'GET https://|TLS connect|FA board') { $color = 'Cyan' }
  Write-Host ("[$ts] " + $line) -ForegroundColor $color
}

Write-Banner

$port = $null
while ($true) {
  try {
    if (-not $port -or -not $port.IsOpen) {
      if ($port) { try { $port.Dispose() } catch {} }
      $port = New-Object System.IO.Ports.SerialPort $PortName, $Baud, None, 8, One
      $port.ReadTimeout = 200
      $port.WriteTimeout = 200
      $port.DtrEnable = $false
      $port.RtsEnable = $false
      $port.NewLine = "`n"
      $port.Open()
      Write-Host ("  Connected to {0}  (DTR/RTS off — no reset)" -f $PortName) -ForegroundColor Green
      Write-Host ''
    }

    $chunk = $port.ReadExisting()
    if ($chunk) {
      foreach ($line in ($chunk -split "`r?`n")) {
        if ($line.Length -gt 0) { Write-LineColored $line }
      }
    } else {
      Start-Sleep -Milliseconds 40
    }
  } catch {
    Write-Host ("  [{0}] {1} — retrying in 1s..." -f (Get-Date -Format 'HH:mm:ss'), $_.Exception.Message) -ForegroundColor Yellow
    if ($port) { try { $port.Close() } catch {}; try { $port.Dispose() } catch {}; $port = $null }
    Start-Sleep -Seconds 1
  }
}
