<#
.SYNOPSIS
  Program the bootable pair to flash and restart the board into it.

.DESCRIPTION
  This is the one that removes the power cycles.

  An XIP image has to be entered by the boot ROM, because the ROM is what
  configures FlexSPI. A debug reset does not redo that - LinkServer connects,
  resets, and the ROM does not re-boot from flash - so for a long time the only
  way to run a freshly programmed image was to physically power the board.

  `LinkServer run` does both halves: it programs the image and resets the target
  into it. Verified rather than assumed - after it, SYS:STAT? reported
  uptime_ms=35007 against a board that had been up for far longer, and the
  capture core was back to not-started.

  It then sits monitoring the target's output, which is not wanted here, so it is
  killed once the board is up. The board keeps running: it is executing from its
  own flash and LinkServer was only watching.

.EXAMPLE
  scripts\reload.ps1
  scripts\reload.ps1 -Build          # build the pair first
#>
param(
    [switch]$Build,
    [int]$WaitSeconds = 20
)

$ErrorActionPreference = 'Stop'

$Root = Split-Path -Parent $PSScriptRoot
$Ls   = 'C:/NXP/LinkServer_26.6.137/LinkServer.exe'

if ($Build) {
    & "$Root/scripts/build.ps1" -Pair
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
}

$Elf = Get-ChildItem "$Root/build/bridge_xip/*.elf" -ErrorAction SilentlyContinue |
       Select-Object -First 1
if (-not $Elf) {
    throw "no XIP image in build/bridge_xip - run scripts\build.ps1 -Pair"
}

# A stale gdbserver or a previous `run` holds the probe, and the next connection
# silently reuses it - which produces failures that look like hardware faults.
Get-Process |
    Where-Object { $_.ProcessName -match 'LinkServer|crt_emu|redlink|arm-none-eabi-gdb' } |
    Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

Write-Host "programming and restarting: $($Elf.Name)" -ForegroundColor Cyan
$proc = Start-Process -FilePath $Ls `
                      -ArgumentList 'run','MIMXRT1186:FRDM-IMXRT1186',$Elf.FullName `
                      -PassThru -WindowStyle Hidden

Start-Sleep -Seconds $WaitSeconds

if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
Get-Process |
    Where-Object { $_.ProcessName -match 'LinkServer|crt_emu|redlink' } |
    Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

$dev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
       Where-Object { $_.InstanceId -like '*VID_1209*' }
if ($dev) {
    Write-Host ("board up: {0} {1}" -f $dev.Status, $dev.InstanceId) -ForegroundColor Green
} else {
    # Named rather than thrown: the image may be fine and simply slower to
    # enumerate, and a failure here is not the same as a failed program.
    Write-Host "board did not enumerate - check SYS:STAT? over COM19" -ForegroundColor Yellow
}
