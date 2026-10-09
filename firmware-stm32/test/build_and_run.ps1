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

$commonSources = @(
    (Join-Path $app "modbus_crc.c"),
    (Join-Path $app "modbus_slave.c"),
    (Join-Path $app "regbank.c")
)

if ($SyntaxOnly) {
    Write-Host "== syntax check (HAL stubs) ==" -ForegroundColor Cyan
    $targets = $commonSources + (Join-Path $app "app_main.c")
    & $Compiler -std=c99 -Wall -Wextra -fsyntax-only -I $stubs -I $app @targets
    if ($LASTEXITCODE -ne 0) {
        Write-Host "syntax check FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
        exit $LASTEXITCODE
    }
    Write-Host "syntax check passed" -ForegroundColor Green
    exit 0
}

Write-Host "== compile unit tests ==" -ForegroundColor Cyan
& $Compiler -std=c99 -Wall -Wextra -I $app @commonSources (Join-Path $root "test\test_modbus.c") -o $out
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
