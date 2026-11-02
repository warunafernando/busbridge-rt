<#
.SYNOPSIS
  Flash or debug a BUSBRIDGE-RT image with LinkServer.

.DESCRIPTION
  LinkServer, not pyOCD. pyOCD cannot drive this part at all: the RT1180 is an
  ADIv6 / CoreSight SoC-600 design whose access ports are addressed by base
  address, and pyOCD enumerates ADIv5 indices - so it finds no cores however it
  is configured. See E:/RT1186/README.md.

  Two things about the CM7 that are not obvious:

    - Its access port is invisible while the core is held in reset, so
      LinkServer wakes it into a spin loop ("landing zone") first.
    - On rev A0 the CM7's VTOR cannot be changed once set, so the landing zone
      must sit at the same address as the application's vector table. That is
      why there is one connect script per RAM region and why this uses the ITCM
      variant - the capture image links to 0x00000000.

.EXAMPLE
  scripts\flash.ps1                        # load bridge into TCM and run
  scripts\flash.ps1 -Profile capture
  scripts\flash.ps1 -Profile bridge -Xip    # program QSPI NOR instead
#>
param(
    [ValidateSet('bridge','peripheral','capture')] [string]$Profile = 'bridge',
    [switch]$Xip,
    [switch]$Debug
)

$ErrorActionPreference = 'Stop'

$Root = Split-Path -Parent $PSScriptRoot
$Ls   = 'C:/NXP/LinkServer_26.6.137/LinkServer.exe'

# -Xip programs the image that build.ps1 -Pair produces, and that one lands in
# build/bridge_xip, not build/$Profile. This used to read build/$Profile either
# way and so offered LinkServer a TCM-linked image to write to QSPI NOR - which
# fails with "No flash configured at the specified address", a message that
# says nothing about the image being the wrong one. Worse, a stale XIP build
# left in build/bridge would have programmed and verified happily.
$Dir = if ($Xip -and $Profile -eq 'bridge') { "$Root/build/bridge_xip" }
       else { "$Root/build/$Profile" }
$Elf = Get-ChildItem "$Dir/*.elf" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $Elf) {
    $how = if ($Xip -and $Profile -eq 'bridge') { 'scripts\build.ps1 -Pair' }
           else { "scripts\build.ps1 -Profile $Profile" }
    throw "no image in $Dir - run $how"
}

# A stale gdbserver holds the probe and the next connection silently reuses it,
# which produces failures that look like hardware faults.
Get-Process | Where-Object { $_.ProcessName -match 'LinkServer|crt_emu|redlink' } |
    Stop-Process -Force -ErrorAction SilentlyContinue

$Device = if ($Profile -eq 'capture') {
    # cm7 as the primary core, and the ITCM connect script - see the note above.
    'E:/RT1186/linkserver/FRDM-IMXRT1186-CM7-primary-ITCM.json'
} else {
    'MIMXRT1186:FRDM-IMXRT1186'
}
$Core = if ($Profile -eq 'capture') { 'cm7' } else { 'cm33' }

if ($Xip) {
    Write-Host "programming QSPI NOR from $($Elf.Name)" -ForegroundColor Cyan
    & $Ls flash $Device load $Elf.FullName
    & $Ls flash $Device verify $Elf.FullName
    return
}

Write-Host "gdbserver for $Profile on $Core - attach with:" -ForegroundColor Cyan
Write-Host "  arm-none-eabi-gdb -ex 'target remote localhost:3333' -ex load -ex continue $($Elf.FullName)"
& $Ls gdbserver --core $Core $Device
