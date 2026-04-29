param(
  [string]$Root = ".",
  [switch]$UseSubmodule,
  [string]$MotorbridgeDir = "",
  [string]$RustDDSDir = "",
  [string]$ZenohDir = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path (Split-Path $MyInvocation.MyCommand.Path -Parent) "..")
$thirdParty = Join-Path $projectRoot "third_party"
New-Item -ItemType Directory -Force $thirdParty | Out-Null

function Ensure-LinkOrPath([string]$name, [string]$targetPath) {
  $dst = Join-Path $thirdParty $name
  if (Test-Path $dst) { return }
  New-Item -ItemType Junction -Path $dst -Target $targetPath | Out-Null
}

if ($UseSubmodule) {
  Push-Location $projectRoot
  git submodule update --init --recursive third_party/motorbridge third_party/RustDDS third_party/zenoh
  Pop-Location
} else {
  if ([string]::IsNullOrWhiteSpace($MotorbridgeDir)) { $MotorbridgeDir = Join-Path $Root "motorbridge" }
  if ([string]::IsNullOrWhiteSpace($RustDDSDir)) { $RustDDSDir = Join-Path $Root "RustDDS" }
  if ([string]::IsNullOrWhiteSpace($ZenohDir)) { $ZenohDir = Join-Path $Root "zenoh" }

  Ensure-LinkOrPath "motorbridge" (Resolve-Path $MotorbridgeDir)
  Ensure-LinkOrPath "RustDDS" (Resolve-Path $RustDDSDir)
  Ensure-LinkOrPath "zenoh" (Resolve-Path $ZenohDir)
}

$env:MOTORBRIDGE_SRC_DIR = Join-Path $thirdParty "motorbridge"
Push-Location $projectRoot
cargo build --release
Pop-Location

Write-Host "Bootstrap done."
Write-Host "Mode: " ($UseSubmodule ? "submodule" : "custom-path")
Write-Host "third_party path: $thirdParty"
