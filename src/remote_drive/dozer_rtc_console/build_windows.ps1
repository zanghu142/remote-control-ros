[CmdletBinding()]
param(
    [string]$WebRtcRoot,
    [string]$NvCodecRoot,
    [ValidateSet('Release', 'RelWithDebInfo', 'Debug')]
    [string]$Configuration = 'Release',
    [string]$BuildDirectory,
    [string]$InstallDirectory,
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
$workspace = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $workspace 'build/windows-console' }
if (-not $InstallDirectory) { $InstallDirectory = Join-Path $workspace 'install/windows-console' }

function Invoke-Checked([string]$Program, [string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}

$configure = @('-S', $PSScriptRoot, '-B', $BuildDirectory,
    '-G', 'Visual Studio 18 2026', '-A', 'x64', '-DBUILD_TESTING=ON')
if ($WebRtcRoot) { $configure += "-DDOZER_WEBRTC_ROOT=$WebRtcRoot" }
if ($NvCodecRoot) { $configure += "-DDOZER_NVCODEC_ROOT=$NvCodecRoot" }
Invoke-Checked 'cmake' $configure
Invoke-Checked 'cmake' @('--build', $BuildDirectory, '--config', $Configuration, '--parallel')
if (-not $SkipTests) {
    Invoke-Checked 'ctest' @('--test-dir', $BuildDirectory, '-C', $Configuration, '--output-on-failure')
}
Invoke-Checked 'cmake' @('--install', $BuildDirectory, '--config', $Configuration,
    '--prefix', $InstallDirectory, '--component', 'Runtime')
Write-Host "Console: $(Join-Path $InstallDirectory 'bin/rtc_console.exe')"
