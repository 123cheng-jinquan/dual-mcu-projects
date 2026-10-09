# =============================================================================
# 从源码里扫出所有汉字 → tools/font_symbols.txt
# -----------------------------------------------------------------------------
# 为什么需要：LVGL 用的是"稀疏字库"（只打进用到的字）。
#   如果源码里加了新汉字却没重跑字库生成，那个字会**显示成方框且不报错**。
#   所以"扫汉字 → 生成字库"要当成一个固定步骤。
#
# 用法（在仓库任意位置）：
#   powershell -ExecutionPolicy Bypass -File firmware-esp32\tools\extract_symbols.ps1
#   ⚠ 必须带 -ExecutionPolicy Bypass：Windows 默认禁止运行未签名的 .ps1
#     （直接 `& script.ps1` 会报 "is not digitally signed"）
# 然后：
#   cd D:\MyProject
#   node firmware-esp32\tools\gen_cn_font.js 14 2
#   node firmware-esp32\tools\gen_cn_font.js 16 2
#   node firmware-esp32\tools\gen_cn_font.js 20 2
#
# 扫描范围：src 下所有 .c/.cpp/.h —— **包括 oled.cpp**，
#   因为 OLED 那一路也有中文（它有自己的一套字模，不受 LVGL 影响，
#   但把它的汉字一起收进来没坏处，将来若统一到 LVGL 就不用再改）。
# 注意：会把**注释里的汉字**也收进来（比如"温度"出现在注释中）。
#   这只会让字库略大一点，不会出错 —— 宁可多收不要漏收。
# =============================================================================

$ErrorActionPreference = 'Stop'

$srcDir  = Join-Path $PSScriptRoot '..\src'
$outFile = Join-Path $PSScriptRoot 'font_symbols.txt'

$files = Get-ChildItem -Path $srcDir -Recurse -Include *.c, *.cpp, *.h -File
if (-not $files) { throw "在 $srcDir 下没找到源文件" }

$sb = New-Object System.Text.StringBuilder
foreach ($f in $files) { [void]$sb.Append([System.IO.File]::ReadAllText($f.FullName)) }
$text = $sb.ToString()

# CJK 统一汉字区 U+4E00~U+9FFF
$chars = [regex]::Matches($text, '[\u4e00-\u9fff]') |
         ForEach-Object { $_.Value } |
         Sort-Object -Unique

if ($chars.Count -eq 0) { throw "没扫到任何汉字 —— 检查 $srcDir 是否正确" }

# 必须写 UTF-8 **无 BOM**：Node 读取时按 UTF-8 解，带 BOM 会把 BOM 当字符传进去
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($outFile, ($chars -join ''), $utf8NoBom)

Write-Host "扫描源文件 : $($files.Count) 个"
Write-Host "不同汉字   : $($chars.Count) 个"
Write-Host "已写入     : $outFile"
Write-Host ""
Write-Host "下一步（在 D:\MyProject 下）："
Write-Host "  node firmware-esp32\tools\gen_cn_font.js 14 2"
Write-Host "  node firmware-esp32\tools\gen_cn_font.js 16 2"
Write-Host "  node firmware-esp32\tools\gen_cn_font.js 20 2"
