<#
.SYNOPSIS
  Build a BUSBRIDGE-RT image.

.DESCRIPTION
  Three images come out of this tree, and which core they run on is not a choice:

    bridge      CM33  the bus controller - USB, command parser, all five buses
    peripheral  CM33  the device under test - answers as a target
    capture     CM7   the sniffer's capture engine

  The core is derived from the profile rather than asked for, because getting it
  wrong produces an image that links and then does nothing useful.

  -Pair builds the bootable two-core image: the capture core first, then its
  binary embedded into a flexspi_nor (XIP) controller image that copies it into
  the CM7's ITCM and starts that core. That is the only configuration in which
  both cores run without a debugger.

  The RAM target cannot carry the capture image - its linker script stores .data
  initialisers in m_text, which the controller already fills to 96% of 127 KB -
  so -Pair implies the XIP memory target.

.EXAMPLE
  scripts\build.ps1
  scripts\build.ps1 -Profile capture
  scripts\build.ps1 -Pair                                       # bootable pair
  scripts\build.ps1 -Profile bridge -Memory flexspi_nor_debug   # XIP, no image
#>
param(
    [ValidateSet('bridge','peripheral','capture')] [string]$Profile = 'bridge',
    [string]$Board  = 'frdm_imxrt1186',
    [string]$Memory = 'debug',
    [switch]$Clean,
    [switch]$Pair
)

$ErrorActionPreference = 'Stop'

$Root = Split-Path -Parent $PSScriptRoot
$Sdk  = 'E:/RT1186/sdk/mcuxsdk'
$Venv = 'E:/RT1186/.venv/Scripts/python.exe'

# The core follows the profile. See the note above.
$Core = if ($Profile -eq 'capture') { 'cm7' } else { 'cm33' }
$Out  = "$Root/build/$Profile"

if ($Pair) {
    # The pair is built here rather than by a CMake dependency so the ordering
    # is visible: the controller embeds a *file* produced by the capture build,
    # and a stale one would be copied into ITCM and run.
    $Profile = 'bridge'
    $Core    = 'cm33'
    $Memory  = 'flexspi_nor_debug'
    $Out     = "$Root/build/bridge_xip"
}

if ($Clean -and (Test-Path $Out)) { Remove-Item -Recurse -Force $Out }

$env:ARMGCC_DIR = 'E:/RT1186/tools/gcc-arm-14.3'
$env:PATH = "E:/RT1186/tools/gcc-arm-14.3/bin;E:/RT1186/tools/ninja;$env:PATH"

Write-Host "building $Profile on $Core ($Memory)" -ForegroundColor Cyan

# Arguments as an array rather than a backtick-continued line. The first
# version used continuations and PowerShell silently dropped everything after
# the first one, which reached CMake as a build with no core_id and failed in
# Kconfig - a long way from the missing backtick.
$argv = @(
    '-m', 'west', 'build',
    '-b', 'frdmimxrt1186', "$Root/firmware",
    '--toolchain', 'armgcc',
    '--config', $Memory,
    "-Dcore_id=$Core",
    "-DBB_PROFILE=$Profile",
    "-DBB_BOARD=$Board",
    '-d', $Out
)
if ($Pair) { $argv += '-DBB_EMBED_CM7=ON' }

if ($Pair) {
    # 1. the capture core, into its own build directory
    Write-Host "  [1/3] capture core (cm7)" -ForegroundColor DarkCyan
    & $PSCommandPath -Profile capture -Board $Board -Memory 'debug'
    if ($LASTEXITCODE -ne 0) { throw "capture build failed" }

    # 2. its binary as a C source the controller can carry
    Write-Host "  [2/3] embedding the capture image" -ForegroundColor DarkCyan
    $bin = "$Root/build/capture/busbridge_rt_cm7.bin"
    if (-not (Test-Path $bin)) { throw "no capture binary at $bin" }
    & $Venv "$Root/scripts/mkcm7image.py" $bin "$Root/firmware/generated/bb_cm7_image.c"
    if ($LASTEXITCODE -ne 0) { throw "mkcm7image failed" }

    Write-Host "  [3/3] controller (cm33, XIP, image embedded)" -ForegroundColor DarkCyan
}

Push-Location $Sdk
try {
    & $Venv @argv
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
} finally {
    Pop-Location
}

Get-ChildItem "$Out/*.elf" | ForEach-Object {
    Write-Host ("  {0}  {1:N0} bytes" -f $_.Name, $_.Length) -ForegroundColor Green
}

if ($Pair) {
    Write-Host ""
    Write-Host "Bootable pair built. To run it:" -ForegroundColor Cyan
    Write-Host "  scripts\flash.ps1 -Profile bridge -Xip"
    Write-Host "  then power-cycle the board with the boot mode set to FlexSPI NOR."
    Write-Host ""
    Write-Host "The ROM has to do the booting - it is what configures FlexSPI." -ForegroundColor DarkGray
    Write-Host "With the boot mode set elsewhere the image programs and verifies" -ForegroundColor DarkGray
    Write-Host "correctly and is simply never entered." -ForegroundColor DarkGray
}
