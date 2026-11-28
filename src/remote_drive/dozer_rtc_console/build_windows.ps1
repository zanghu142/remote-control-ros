[CmdletBinding()]
param(
    [string]$WebRtcRoot,
    [string]$NvCodecRoot,
    [ValidateSet('Release', 'RelWithDebInfo', 'Debug')]
    [string]$Configuration = 'Release',
    [string]$BuildDirectory,
    [string]$InstallDirectory,
    [ValidateSet('Auto', '2022', '2026')]
    [string]$VisualStudioVersion = 'Auto',
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

$generators = @{ '2022' = 'Visual Studio 17 2022'; '2026' = 'Visual Studio 18 2026' }
$cachedGenerator = $null
$cachePath = Join-Path $BuildDirectory 'CMakeCache.txt'
if (Test-Path -LiteralPath $cachePath) {
    $cacheEntry = Select-String -LiteralPath $cachePath -Pattern '^CMAKE_GENERATOR:INTERNAL=(.*)$'
    if ($cacheEntry) { $cachedGenerator = $cacheEntry.Matches[0].Groups[1].Value }
}
if ($VisualStudioVersion -eq 'Auto' -and $cachedGenerator) {
    $VisualStudioVersion = switch ($cachedGenerator) {
        'Visual Studio 17 2022' { '2022' }
        'Visual Studio 18 2026' { '2026' }
        default { throw "Unsupported cached generator '$cachedGenerator'. Use -BuildDirectory with a new directory." }
    }
}
if ($VisualStudioVersion -ne 'Auto' -and $cachedGenerator -and
    $cachedGenerator -ne $generators[$VisualStudioVersion]) {
    throw "Build directory uses '$cachedGenerator'. Select a new -BuildDirectory when switching Visual Studio versions."
}

$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw 'Visual Studio Installer/vswhere.exe not found. Install Visual Studio 2022 or 2026 with Desktop development with C++.'
}
$capabilitiesJson = & cmake -E capabilities
if ($LASTEXITCODE -ne 0) { throw 'Cannot query CMake capabilities. Check the CMake installation and PATH.' }
$capabilities = ($capabilitiesJson -join "`n") | ConvertFrom-Json
$cmakeVersion = [version]("{0}.{1}.{2}" -f $capabilities.version.major,
    $capabilities.version.minor, $capabilities.version.patch)
if ($cmakeVersion -lt [version]'3.24') {
    throw 'The Windows console requires CMake 3.24 or newer (VS 2026 requires CMake 4.2 or newer).'
}
$supportedGenerators = @($capabilities.generators | ForEach-Object { $_.name })
$candidates = if ($VisualStudioVersion -eq 'Auto') { @('2022', '2026') } else { @($VisualStudioVersion) }
$generator = $null
foreach ($candidate in $candidates) {
    $versionRange = if ($candidate -eq '2022') { '[17.0,18.0)' } else { '[18.0,19.0)' }
    $installation = @(& $vswhere -latest -products '*' -version $versionRange `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath)
    if ($LASTEXITCODE -ne 0) { throw 'Visual Studio installation detection failed.' }
    if (-not $installation.Count) { continue }
    if ($supportedGenerators -notcontains $generators[$candidate]) {
        if ($VisualStudioVersion -ne 'Auto') {
            throw "CMake does not support '$($generators[$candidate])'. Update CMake (VS 2026 requires 4.2 or newer)."
        }
        continue
    }
    $generator = $generators[$candidate]
    break
}
if (-not $generator) {
    throw "No usable Visual Studio $VisualStudioVersion installation found. Install C++ desktop tools and a Windows SDK, and use CMake supporting that Visual Studio version. If this directory caches an uninstalled version, use a new -BuildDirectory."
}
Write-Host "Generator: $generator"
$configure = @('-S', $PSScriptRoot, '-B', $BuildDirectory,
    '-G', $generator, '-A', 'x64', '-DBUILD_TESTING=ON')
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
