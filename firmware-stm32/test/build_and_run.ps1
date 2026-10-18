# build_and_run.ps1
# ---------------------------------------------------------------------------
# NOTE: ASCII-only on purpose. Windows PowerShell 5.1 reads .ps1 as ANSI unless
#       there is a UTF-8 BOM, so non-ASCII text in a BOM-less script breaks it.
#
# Two modes:
#   powershell -ExecutionPolicy Bypass -NoProfile -File test\build_and_run.ps1
#       -> build + run the 89 PC-side unit tests of the hand-written Modbus stack
#   powershell -ExecutionPolicy Bypass -NoProfile -File test\build_and_run.ps1 -SyntaxOnly
#       -> syntax/type check App/*.c against the HAL stubs in test/stubs
#          (use this after editing app_main.c; real compilation happens in CubeMX)
#
# Needs a host C compiler: gcc (MSYS2 ucrt64) or clang.
param(
    [string]$Compiler = "gcc",
    [switch]$SyntaxOnly
)

$ErrorActionPreference = "Stop"
$root  = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$app   = Join-Path $root "App"
$stubs = Join-Path $root "test\stubs"
$ext   = if ($IsLinux -or $IsMacOS) { "" } else { ".exe" }
$out   = Join-Path $root "test\modbus_tests$ext"

# NOTE: App/ is split into sub-directories by responsibility (see App/README.md):
#   protocol/ = Modbus stack | regmap/ = register table
#   features/ = feature algorithm | app/ = task orchestration
# So each source path and each include dir has to be listed explicitly.
# WARNING: keep this file ASCII-only. PowerShell 5.1 reads a BOM-less .ps1 as GBK,
#          so any UTF-8 Chinese in here breaks parsing (documented in README).
$appProtocol = Join-Path $app "protocol"
$appRegmap   = Join-Path $app "regmap"
$appFeatures = Join-Path $app "features"
$appApp      = Join-Path $app "app"

$commonSources = @(
    (Join-Path $appProtocol "modbus_crc.c"),
    (Join-Path $appProtocol "modbus_slave.c"),
    (Join-Path $appRegmap   "regbank.c"),
    (Join-Path $appFeatures "vibration_feat.c")
)

$appIncludes = @(
    "-I", $appProtocol,
    "-I", $appRegmap,
    "-I", $appFeatures,
    "-I", $appApp
)

if ($SyntaxOnly) {
    Write-Host "== syntax check (HAL stubs) ==" -ForegroundColor Cyan
    $targets = $commonSources + (Join-Path $appApp "app_main.c")
    & $Compiler -std=c99 -Wall -Wextra -fsyntax-only -I $stubs @appIncludes @targets
    if ($LASTEXITCODE -ne 0) {
        Write-Host "syntax check FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
        exit $LASTEXITCODE
    }
    Write-Host "syntax check passed" -ForegroundColor Green
    exit 0
}

Write-Host "== compile unit tests ==" -ForegroundColor Cyan
& $Compiler -std=c99 -Wall -Wextra @appIncludes @commonSources (Join-Path $root "test\test_modbus.c") -o $out
if ($LASTEXITCODE -ne 0) {
    Write-Host "compile FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host "== run ==" -ForegroundColor Cyan
& $out
$code = $LASTEXITCODE
Remove-Item $out -ErrorAction SilentlyContinue

if ($code -eq 0) {
    Write-Host "ALL TESTS PASSED" -ForegroundColor Green
} else {
    Write-Host "SOME TESTS FAILED" -ForegroundColor Red
}
exit $code
