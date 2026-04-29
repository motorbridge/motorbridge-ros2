$ErrorActionPreference = 'Stop'

Write-Host '[check] Rust toolchain'
rustc --version
cargo --version

Write-Host '[check] MSVC linker'
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (Test-Path $vswhere) {
  & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
}

Write-Host '[check] Packet.lib (Npcap SDK)'
$candidates = @(
  'C:\Program Files\Npcap SDK\Lib\x64\Packet.lib',
  'C:\Program Files (x86)\Npcap SDK\Lib\x64\Packet.lib',
  'C:\WpdPack\Lib\x64\Packet.lib'
)
$found = $candidates | Where-Object { Test-Path $_ }
if ($found.Count -eq 0) {
  Write-Host 'Packet.lib not found.' -ForegroundColor Yellow
  Write-Host 'Install Npcap SDK, then set LIB path before release build:'
  Write-Host '$env:LIB = "C:\Program Files\Npcap SDK\Lib\x64;" + $env:LIB'
  exit 2
}

Write-Host "Found Packet.lib: $($found[0])" -ForegroundColor Green
$libDir = Split-Path -Parent $found[0]
$env:LIB = "$libDir;" + $env:LIB

Write-Host '[check] cargo build --release'
cargo build --release
