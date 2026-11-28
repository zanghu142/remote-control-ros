[CmdletBinding()]
param(
    [string]$WebRtcRoot,
    [string]$NvCodecRoot,
    [ValidateSet('Release', 'RelWithDebInfo', 'Debug')]
    [string]$Configuration,
    [string]$BuildDirectory,
    [string]$InstallDirectory,
    [ValidateSet('Auto', '2022', '2026')]
    [string]$VisualStudioVersion,
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'src/remote_drive/dozer_rtc_console/build_windows.ps1') @PSBoundParameters
